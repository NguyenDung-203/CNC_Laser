#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <FS.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <stdlib.h>

#ifndef BOARD_ESP32C6_SUPERMINI
#define BOARD_ESP32C6_SUPERMINI 0
#endif

#ifndef HAS_LCD_UI
#if (BOARD_ESP32C6_SUPERMINI != 0)
#define HAS_LCD_UI 0
#else
#define HAS_LCD_UI 1
#endif
#endif

#ifndef RGB_STATUS_LED_PIN
#if (BOARD_ESP32C6_SUPERMINI != 0)
#define RGB_STATUS_LED_PIN 8
#else
#define RGB_STATUS_LED_PIN -1
#endif
#endif

#if (RGB_STATUS_LED_PIN >= 0)
#define HAS_RGB_STATUS_LED 1
#else
#define HAS_RGB_STATUS_LED 0
#endif

#if (HAS_LCD_UI != 0)
#include <LovyanGFX.hpp>
#endif

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#endif

#ifndef WIFI_CONFIG_AP_RETRY_MS
#define WIFI_CONFIG_AP_RETRY_MS 60000UL
#endif

#ifndef WIFI_USE_STATIC_IP
#define WIFI_USE_STATIC_IP 0
#endif

#ifndef WIFI_LOCAL_IP
#define WIFI_LOCAL_IP IPAddress(0, 0, 0, 0)
#endif

#ifndef WIFI_GATEWAY
#define WIFI_GATEWAY IPAddress(0, 0, 0, 0)
#endif

#ifndef WIFI_SUBNET
#define WIFI_SUBNET IPAddress(255, 255, 255, 0)
#endif

#ifndef WIFI_DNS1
#define WIFI_DNS1 WIFI_GATEWAY
#endif

#ifndef WIFI_DNS2
#define WIFI_DNS2 IPAddress(8, 8, 8, 8)
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "cnc-laser-esp32"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD "cnclaser"
#endif

#ifndef TELNET_PORT
#define TELNET_PORT 23
#endif

#ifndef HTTP_PORT
#define HTTP_PORT 80
#endif

#ifndef CONFIG_AP_SSID
#define CONFIG_AP_SSID "CNC-Laser-Setup"
#endif

#ifndef CONFIG_AP_PASSWORD
#define CONFIG_AP_PASSWORD "cnclaser"
#endif

#ifndef WIFI_RESET_BUTTON_PIN
#if (BOARD_ESP32C6_SUPERMINI != 0)
#define WIFI_RESET_BUTTON_PIN 9
#else
#define WIFI_RESET_BUTTON_PIN -1
#endif
#endif

#ifndef WIFI_RESET_HOLD_MS
#define WIFI_RESET_HOLD_MS 3000UL
#endif

#if (WIFI_RESET_BUTTON_PIN >= 0)
#define HAS_WIFI_RESET_BUTTON 1
#else
#define HAS_WIFI_RESET_BUTTON 0
#endif

#if (HAS_LCD_UI != 0)
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9341 panel;
  lgfx::Bus_SPI bus;
  lgfx::Light_PWM light;
  lgfx::Touch_XPT2046 touch;

public:
  LGFX(void)
  {
    {
      auto cfg = bus.config();
      cfg.spi_host = HSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = 1;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc = 2;
      bus.config(cfg);
      panel.setBus(&bus);
    }
    {
      auto cfg = panel.config();
      cfg.pin_cs = 15;
      cfg.pin_rst = -1;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.memory_width = 240;
      cfg.memory_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      panel.config(cfg);
    }
    {
      auto cfg = light.config();
      cfg.pin_bl = -1;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      light.config(cfg);
    }
    {
      auto cfg = touch.config();
      cfg.spi_host = VSPI_HOST;
      cfg.freq = 2500000;
      cfg.pin_sclk = 25;
      cfg.pin_mosi = 32;
      cfg.pin_miso = 39;
      cfg.pin_cs = 33;
      cfg.pin_int = 36;
      cfg.bus_shared = false;
      cfg.x_min = 200;
      cfg.x_max = 3900;
      cfg.y_min = 200;
      cfg.y_max = 3900;
      touch.config(cfg);
      panel.setTouch(&touch);
    }
    setPanel(&panel);
  }
};
#endif

#ifndef STM32_UART_RX_GPIO
#define STM32_UART_RX_GPIO 16
#endif

#ifndef STM32_UART_TX_GPIO
#define STM32_UART_TX_GPIO 17
#endif

static const int STM32_UART_RX_PIN = STM32_UART_RX_GPIO;
static const int STM32_UART_TX_PIN = STM32_UART_TX_GPIO;
#if (BOARD_ESP32C6_SUPERMINI != 0)
static HardwareSerial stm32_uart(1);
#else
static HardwareSerial stm32_uart(2);
#endif
#if (HAS_LCD_UI != 0)
static const int LCD_BACKLIGHT_PIN = 21;
#else
static const int LCD_BACKLIGHT_PIN = -1;
#endif
static const uint32_t DEBUG_BAUD = 115200;
static const size_t BRIDGE_BUF_SIZE = 128;
static const size_t GCODE_LINE_BUF_SIZE = 96;
static const size_t STM32_LINE_BUF_SIZE = 160;
static const uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000UL;
static const uint32_t STM32_OK_TIMEOUT_MS = 120000UL;
static const uint32_t STM32_BOOTLOADER_WAIT_MS = 15000UL;
static const uint32_t STM32_BOOTLOADER_LINE_TIMEOUT_MS = 30000UL;
static const size_t STM32_FW_CHUNK_SIZE = 256;
static const byte DNS_PORT = 53;
static const char *GCODE_UPLOAD_PATH = "/job.gcode";
static const char *MANUAL_JOB_PATH = "/manual.gcode";
static const char *STM32_APP_UPLOAD_PATH = "/stm32_app.bin";
static const float XY_CENTER_X_MM = 164.0f;
static const float XY_CENTER_Y_MM = 142.0f;
static const float MANUAL_STEPS_MM[] = {1.0f, 5.0f, 10.0f, 50.0f};
static const size_t MANUAL_STEP_COUNT = sizeof(MANUAL_STEPS_MM) / sizeof(MANUAL_STEPS_MM[0]);
static const float MANUAL_XY_FEED_MM_MIN = 3000.0f;
static const IPAddress CONFIG_AP_IP(192, 168, 4, 1);
static const IPAddress CONFIG_AP_SUBNET(255, 255, 255, 0);

enum GcodeJobState
{
  JOB_IDLE = 0,
  JOB_STREAMING,
  JOB_WAIT_OK,
  JOB_DONE,
  JOB_ERROR,
  JOB_STOPPED
};

#if (HAS_LCD_UI != 0)
enum LcdAction : uint8_t
{
  LCD_ACT_NONE = 0,
  LCD_ACT_X_MINUS,
  LCD_ACT_X_PLUS,
  LCD_ACT_Y_PLUS,
  LCD_ACT_Y_MINUS,
  LCD_ACT_Z_UP,
  LCD_ACT_Z_DOWN,
  LCD_ACT_STEP,
  LCD_ACT_HOME_CENTER,
  LCD_ACT_CENTER,
  LCD_ACT_RUN_LAST,
  LCD_ACT_STOP
};

struct LcdButton
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  const char *label;
  uint8_t action;
  uint16_t color;
};

static const uint16_t UI_BG = 0x10A2;
static const uint16_t UI_PANEL = 0xFFFF;
static const uint16_t UI_LINE = 0xC638;
static const uint16_t UI_TEXT = 0x1082;
static const uint16_t UI_MUTED = 0x6B4D;
static const uint16_t UI_ACCENT = 0x0595;
static const uint16_t UI_BLUE = 0x2C7F;
static const uint16_t UI_DANGER = 0xD145;

static const LcdButton LCD_BUTTONS[] = {
    {82, 42, 58, 42, "Y+", LCD_ACT_Y_PLUS, UI_BLUE},
    {22, 90, 58, 42, "X-", LCD_ACT_X_MINUS, UI_BLUE},
    {82, 90, 58, 42, "STEP", LCD_ACT_STEP, 0xE73C},
    {142, 90, 58, 42, "X+", LCD_ACT_X_PLUS, UI_BLUE},
    {82, 138, 58, 42, "Y-", LCD_ACT_Y_MINUS, UI_BLUE},
    {210, 42, 96, 32, "HOME", LCD_ACT_HOME_CENTER, UI_ACCENT},
    {210, 78, 96, 32, "CENTER", LCD_ACT_CENTER, UI_ACCENT},
    {210, 114, 96, 32, "RUN", LCD_ACT_RUN_LAST, UI_ACCENT},
    {210, 150, 46, 34, "Z+", LCD_ACT_Z_UP, 0x7BEF},
    {260, 150, 46, 34, "Z-", LCD_ACT_Z_DOWN, 0x7BEF},
    {210, 190, 96, 34, "STOP", LCD_ACT_STOP, UI_DANGER},
};
static const size_t LCD_BUTTON_COUNT = sizeof(LCD_BUTTONS) / sizeof(LCD_BUTTONS[0]);
#endif

static WiFiServer telnet_server(TELNET_PORT);
static WiFiClient telnet_client;
static WebServer http_server(HTTP_PORT);
static DNSServer dns_server;
#if (HAS_LCD_UI != 0)
static LGFX lcd;
#endif
static File upload_file;
static File job_file;
static bool network_services_started = false;
static bool config_ap_active = false;
static bool fs_ready = false;
static bool upload_failed = false;
static bool firmware_upload_failed = false;
static bool esp32_ota_failed = false;
static size_t esp32_ota_written = 0;
static String esp32_ota_message = "idle";
static bool rgb_status_led_ready = false;
static uint8_t rgb_status_r = 0;
static uint8_t rgb_status_g = 0;
static uint8_t rgb_status_b = 0;
static uint32_t last_rgb_status_ms = 0;
static bool ota_update_active = false;
static int ota_last_logged_percent = -1;
static uint32_t last_wifi_attempt_ms = 0;
static GcodeJobState job_state = JOB_IDLE;
static char job_line_buf[GCODE_LINE_BUF_SIZE];
static size_t job_line_len = 0;
static char stm32_line_buf[STM32_LINE_BUF_SIZE];
static size_t stm32_line_len = 0;
static char firmware_line_buf[STM32_LINE_BUF_SIZE];
static size_t firmware_line_len = 0;
static uint32_t job_started_ms = 0;
static uint32_t job_wait_started_ms = 0;
static uint32_t job_lines_sent = 0;
static uint32_t job_lines_ok = 0;
static uint32_t job_lines_skipped = 0;
static size_t job_file_size = 0;
static size_t job_file_pos = 0;
static String job_message = "idle";
static String firmware_message = "idle";
static String last_stm32_line = "";
static size_t manual_step_index = 1;
static bool lcd_ready = false;
static bool lcd_dirty = true;
static uint32_t last_lcd_draw_ms = 0;
static uint32_t last_touch_ms = 0;
static String wifi_ssid = WIFI_SSID;
static String wifi_password = WIFI_PASSWORD;
static bool wifi_force_setup_portal = false;
static bool wifi_use_static_ip = (WIFI_USE_STATIC_IP != 0);
static IPAddress wifi_local_ip = WIFI_LOCAL_IP;
static IPAddress wifi_gateway = WIFI_GATEWAY;
static IPAddress wifi_subnet = WIFI_SUBNET;
static IPAddress wifi_dns1 = WIFI_DNS1;
static IPAddress wifi_dns2 = WIFI_DNS2;
static bool wifi_reset_button_was_down = false;
static bool wifi_reset_button_triggered = false;
static uint32_t wifi_reset_button_down_ms = 0;

static void logLine(const String &line);
static void loadWifiSettings(void);
static void saveWifiSettings(void);
static void clearWifiSettings(bool force_setup_portal);
static bool parseIpSetting(const String &text, IPAddress *out);
static bool isStaConnected(void);
static bool isConfigOnlyMode(void);
static bool requireStaModeForMachine(void);
static void setupWifiResetButton(void);
static void wifiResetButtonTask(void);
static void clearWifiByButton(void);
static bool connectStaWifi(void);
static void startConfigAp(void);
static void stopConfigAp(void);
static void handleWifi(void);
static void startNetworkServices(void);
static void setupOta(void);
static void setupTelnet(void);
static void setupHttp(void);
static void handleTelnetClient(void);
static void bridgeUsbToStm32(void);
static void bridgeTelnetToStm32(void);
static void bridgeStm32ToOutputs(void);
static void writeToTelnet(const uint8_t *data, size_t len);
static void printNetworkInfo(void);
static void handleRoot(void);
static void handleWifiConfigPage(void);
static void handleWifiSave(void);
static void handleWifiClear(void);
static void handleNotFound(void);
static String htmlEscape(const String &value);
static void handleStatus(void);
static void handleStop(void);
static void handleJog(void);
static void handleZJog(void);
static void handleLaserWork(void);
static void handleLaserOff(void);
static void handleHomeCenter(void);
static void handleCenter(void);
static void handleRunLast(void);
static void handleLcdTest(void);
static void handleGcodeUploadDone(void);
static void handleGcodeUploadData(void);
static void handleFirmwarePage(void);
static void handleFirmwareUploadDone(void);
static void handleFirmwareUploadData(void);
static void handleEsp32OtaPage(void);
static void handleEsp32OtaUploadDone(void);
static void handleEsp32OtaUploadData(void);
static void prepareEsp32OtaUpdate(const char *source);
static bool startGcodeJob(void);
static bool startGcodeJobFromPath(const char *path, const String &message);
static bool writeManualJob(const String &content);
static bool startManualJob(const String &label, const String &content);
static bool startManualJog(float x_mm, float y_mm);
static bool startManualZ(bool up);
static bool startManualLaserWork(void);
static bool startManualLaserOff(void);
static bool startManualHomeCenter(void);
static bool startManualCenter(void);
static bool startLastUploadedJob(void);
static uint8_t currentJobProgressPercent(void);
static float currentManualStepMm(void);
static String formatMm(float value);
static void gcodeJobTask(void);
static bool readNextGcodeLine(void);
static bool isRunnableGcodeLine(const char *line);
static void sendCurrentGcodeLine(void);
static void stopGcodeJob(GcodeJobState state, const String &message);
static bool isJobRunning(void);
static const char *jobStateName(GcodeJobState state);
static void handleStm32JobByte(uint8_t data);
static void handleStm32JobLine(const char *line);
static bool runStm32FirmwareUpdate(void);
static bool calcFileCrc32(const char *path, uint32_t *size_out, uint32_t *crc_out);
static uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len);
static bool waitStm32FirmwareLine(String &line, uint32_t timeout_ms);
static bool waitStm32FirmwareContains(const char *pattern, uint32_t timeout_ms);
static bool waitStm32Bootloader(void);
static bool parseNextRequest(const String &line, uint32_t *offset, uint32_t *len);
static void setupStatusLed(void);
static void statusLedTask(void);
static void setStatusRgb(uint8_t r, uint8_t g, uint8_t b);
static void setupLcdUi(void);
static void lcdUiTask(void);
static void drawLcdUi(bool force);
#if (HAS_LCD_UI != 0)
static void drawLcdButton(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t color, uint16_t text_color);
static void handleLcdTouch(void);
static void runLcdAction(uint8_t action);
#endif

void setup()
{
  Serial.begin(DEBUG_BAUD);
  stm32_uart.begin(DEBUG_BAUD, SERIAL_8N1, STM32_UART_RX_PIN, STM32_UART_TX_PIN);
  delay(300);
  setupStatusLed();
  setupWifiResetButton();

  logLine("");
  logLine(BOARD_ESP32C6_SUPERMINI != 0 ? "ESP32-C6 CNC Laser bridge boot" : "ESP32 CNC Laser bridge boot");
  logLine(String("STM32 UART: GPIO") + STM32_UART_TX_PIN + " TX -> STM32 RX, GPIO" +
          STM32_UART_RX_PIN + " RX <- STM32 TX");

  setupLcdUi();
  loadWifiSettings();
  if (!connectStaWifi())
  {
    startConfigAp();
  }
  startNetworkServices();
  printNetworkInfo();
  lcd_dirty = true;
  logLine(isConfigOnlyMode() ? "Bridge ready: USB Serial <-> STM32 UART, WiFi config portal only"
                             : "Bridge ready: USB Serial + Telnet <-> STM32 UART");
}

void loop()
{
  handleWifi();
  if (config_ap_active)
  {
    dns_server.processNextRequest();
  }
  if (network_services_started)
  {
    http_server.handleClient();
    if (!isConfigOnlyMode())
    {
      ArduinoOTA.handle();
      handleTelnetClient();
      bridgeTelnetToStm32();
    }
  }
  bridgeUsbToStm32();
  bridgeStm32ToOutputs();
  gcodeJobTask();
  lcdUiTask();
  statusLedTask();
  wifiResetButtonTask();
}

static void logLine(const String &line)
{
  Serial.println(line);
  if (telnet_client && telnet_client.connected())
  {
    telnet_client.println(line);
  }
}

static void loadWifiSettings(void)
{
  Preferences prefs;
  String saved_ssid;

  if (!prefs.begin("wifi", false))
  {
    logLine("WiFi prefs open failed, using build defaults");
    return;
  }

  wifi_force_setup_portal = prefs.getBool("force_portal", false);
  if (wifi_force_setup_portal)
  {
    wifi_ssid = "";
    wifi_password = "";
    logLine("WiFi setup portal forced by reset button/clear request");
    prefs.end();
    return;
  }

  saved_ssid = prefs.isKey("ssid") ? prefs.getString("ssid", "") : "";
  if (saved_ssid.length() > 0)
  {
    wifi_ssid = saved_ssid;
    wifi_password = prefs.getString("pass", "");
    wifi_use_static_ip = prefs.getBool("static", wifi_use_static_ip);
    (void)wifi_local_ip.fromString(prefs.getString("ip", wifi_local_ip.toString()));
    (void)wifi_gateway.fromString(prefs.getString("gw", wifi_gateway.toString()));
    (void)wifi_subnet.fromString(prefs.getString("subnet", wifi_subnet.toString()));
    (void)wifi_dns1.fromString(prefs.getString("dns1", wifi_dns1.toString()));
    (void)wifi_dns2.fromString(prefs.getString("dns2", wifi_dns2.toString()));
    logLine(String("WiFi config loaded from NVS: ") + wifi_ssid);
  }
  else
  {
    logLine(String("WiFi using build default SSID: ") + (wifi_ssid.length() > 0 ? wifi_ssid : "(empty)"));
  }

  prefs.end();
}

static void saveWifiSettings(void)
{
  Preferences prefs;

  if (!prefs.begin("wifi", false))
  {
    logLine("WiFi prefs save failed");
    return;
  }

  prefs.putString("ssid", wifi_ssid);
  prefs.putString("pass", wifi_password);
  prefs.putBool("static", wifi_use_static_ip);
  prefs.putString("ip", wifi_local_ip.toString());
  prefs.putString("gw", wifi_gateway.toString());
  prefs.putString("subnet", wifi_subnet.toString());
  prefs.putString("dns1", wifi_dns1.toString());
  prefs.putString("dns2", wifi_dns2.toString());
  prefs.putBool("force_portal", false);
  prefs.end();
  wifi_force_setup_portal = false;
}

static void clearWifiSettings(bool force_setup_portal)
{
  Preferences prefs;

  if (prefs.begin("wifi", false))
  {
    prefs.clear();
    if (force_setup_portal)
    {
      prefs.putBool("force_portal", true);
    }
    prefs.end();
  }

  wifi_force_setup_portal = force_setup_portal;
  wifi_ssid = "";
  wifi_password = "";
}

static bool parseIpSetting(const String &text, IPAddress *out)
{
  IPAddress parsed;

  if (text.length() == 0)
  {
    return false;
  }
  if (!parsed.fromString(text))
  {
    return false;
  }

  *out = parsed;
  return true;
}

static bool isStaConnected(void)
{
  return WiFi.status() == WL_CONNECTED;
}

static bool isConfigOnlyMode(void)
{
  return config_ap_active && !isStaConnected();
}

static bool requireStaModeForMachine(void)
{
  if (!isConfigOnlyMode())
  {
    return true;
  }

  http_server.sendHeader("Cache-Control", "no-store");
  http_server.send(503, "text/plain",
                   "config AP only: save WiFi settings, reconnect ESP32 to STA WiFi, then use CNC console\n");
  return false;
}

static void setupWifiResetButton(void)
{
#if (HAS_WIFI_RESET_BUTTON != 0)
  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);
#endif
}

static void wifiResetButtonTask(void)
{
#if (HAS_WIFI_RESET_BUTTON != 0)
  const uint32_t now_ms = millis();
  const bool button_down = digitalRead(WIFI_RESET_BUTTON_PIN) == LOW;

  if (!button_down)
  {
    wifi_reset_button_was_down = false;
    wifi_reset_button_triggered = false;
    return;
  }

  if (!wifi_reset_button_was_down)
  {
    wifi_reset_button_was_down = true;
    wifi_reset_button_triggered = false;
    wifi_reset_button_down_ms = now_ms;
    return;
  }

  if (!wifi_reset_button_triggered && ((now_ms - wifi_reset_button_down_ms) >= WIFI_RESET_HOLD_MS))
  {
    wifi_reset_button_triggered = true;
    clearWifiByButton();
  }
#endif
}

static void clearWifiByButton(void)
{
  logLine("BOOT held 3s: clearing saved WiFi and restarting in config portal");
  clearWifiSettings(true);
  if (isJobRunning())
  {
    stm32_uart.print("!\n");
    stopGcodeJob(JOB_STOPPED, "stopped: WiFi reset button");
  }
  setStatusRgb(32, 0, 32);
  delay(600);
  ESP.restart();
}

static bool connectStaWifi(void)
{
  if (wifi_ssid.length() == 0)
  {
    logLine("No WiFi SSID configured.");
    return false;
  }

  WiFi.mode(config_ap_active ? WIFI_AP_STA : WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(OTA_HOSTNAME);

  if (wifi_use_static_ip)
  {
    if (!WiFi.config(wifi_local_ip, wifi_gateway, wifi_subnet, wifi_dns1, wifi_dns2))
    {
      logLine("WiFi static IP config failed");
    }
    else
    {
      logLine(String("WiFi static IP configured: ") + wifi_local_ip.toString());
    }
  }

  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

  Serial.print("Connecting WiFi SSID: ");
  Serial.println(wifi_ssid);

  const uint32_t started_ms = millis();
  while ((WiFi.status() != WL_CONNECTED) &&
         ((millis() - started_ms) < WIFI_CONNECT_TIMEOUT_MS))
  {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    logLine("WiFi STA connected");
    stopConfigAp();
    return true;
  }

  WiFi.disconnect(false);
  logLine("WiFi STA failed. Config AP will remain active.");
  return false;
}

static void startConfigAp(void)
{
  if (config_ap_active)
  {
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  if (!WiFi.softAPConfig(CONFIG_AP_IP, CONFIG_AP_IP, CONFIG_AP_SUBNET))
  {
    logLine("Config AP IP setup failed");
  }

  if (strlen(CONFIG_AP_PASSWORD) >= 8)
  {
    WiFi.softAP(CONFIG_AP_SSID, CONFIG_AP_PASSWORD);
  }
  else
  {
    WiFi.softAP(CONFIG_AP_SSID);
  }

  config_ap_active = true;
  if (!isStaConnected())
  {
    telnet_client.stop();
  }
  last_wifi_attempt_ms = millis();
  dns_server.start(DNS_PORT, "*", CONFIG_AP_IP);
  logLine(String("Config AP ready: SSID=") + CONFIG_AP_SSID + " IP=" + WiFi.softAPIP().toString());
}

static void stopConfigAp(void)
{
  if (!config_ap_active)
  {
    return;
  }

  dns_server.stop();
  WiFi.softAPdisconnect(true);
  config_ap_active = false;
  WiFi.mode(WIFI_STA);
}

static void handleWifi(void)
{
  uint32_t retry_interval_ms = config_ap_active ? WIFI_CONFIG_AP_RETRY_MS : WIFI_RECONNECT_INTERVAL_MS;

  if (isConfigOnlyMode())
  {
    return;
  }

  if ((wifi_ssid.length() == 0) || isStaConnected())
  {
    return;
  }

  if ((millis() - last_wifi_attempt_ms) < retry_interval_ms)
  {
    return;
  }

  last_wifi_attempt_ms = millis();
  if (connectStaWifi())
  {
    startNetworkServices();
    printNetworkInfo();
  }
  else if (!config_ap_active)
  {
    startConfigAp();
    startNetworkServices();
    printNetworkInfo();
  }
}

static void startNetworkServices(void)
{
  if (network_services_started || ((WiFi.status() != WL_CONNECTED) && !config_ap_active))
  {
    return;
  }

  setupHttp();
  if (!isConfigOnlyMode())
  {
    setupOta();
    setupTelnet();
  }
  else
  {
    logLine("STA-only console disabled while in config AP mode");
  }
  network_services_started = true;
}

static void setupOta(void)
{
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setTimeout(30000);
  if (strlen(OTA_PASSWORD) > 0)
  {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    prepareEsp32OtaUpdate("ArduinoOTA");
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() {
    ota_update_active = false;
    Serial.println();
    Serial.println("OTA end");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percent = (int)((progress * 100U) / total);
    if ((percent != ota_last_logged_percent) && ((percent % 5) == 0))
    {
      ota_last_logged_percent = percent;
      Serial.printf("OTA progress: %d%%\n", percent);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    ota_update_active = false;
    setStatusRgb(32, 0, 0);
    Serial.printf("OTA error[%u]\n", error);
  });

  ArduinoOTA.begin();
  logLine(String("OTA ready: ") + OTA_HOSTNAME);
}

static void setupTelnet(void)
{
  telnet_server.begin();
  telnet_server.setNoDelay(true);
  logLine(String("Telnet ready on port ") + TELNET_PORT);
}

static void setupHttp(void)
{
  fs_ready = SPIFFS.begin(true);
  if (!fs_ready)
  {
    logLine("SPIFFS mount failed, web upload disabled");
  }

  http_server.on("/", HTTP_GET, handleRoot);
  http_server.on("/wifi", HTTP_GET, handleWifiConfigPage);
  http_server.on("/wifi-save", HTTP_POST, handleWifiSave);
  http_server.on("/wifi-clear", HTTP_POST, handleWifiClear);
  http_server.on("/status", HTTP_GET, handleStatus);
  http_server.on("/stop", HTTP_POST, handleStop);
  http_server.on("/jog", HTTP_POST, handleJog);
  http_server.on("/z", HTTP_POST, handleZJog);
  http_server.on("/laser-work", HTTP_POST, handleLaserWork);
  http_server.on("/laser-off", HTTP_POST, handleLaserOff);
  http_server.on("/home-center", HTTP_POST, handleHomeCenter);
  http_server.on("/center", HTTP_POST, handleCenter);
  http_server.on("/run-last", HTTP_POST, handleRunLast);
  http_server.on("/lcd-test", HTTP_POST, handleLcdTest);
  http_server.on("/upload", HTTP_POST, handleGcodeUploadDone, handleGcodeUploadData);
  http_server.on("/firmware", HTTP_GET, handleFirmwarePage);
  http_server.on("/fw-upload", HTTP_POST, handleFirmwareUploadDone, handleFirmwareUploadData);
  http_server.on("/esp32-ota", HTTP_GET, handleEsp32OtaPage);
  http_server.on("/esp32-ota", HTTP_POST, handleEsp32OtaUploadDone, handleEsp32OtaUploadData);
  http_server.onNotFound(handleNotFound);
  http_server.begin();
  if (isConfigOnlyMode())
  {
    logLine(String("HTTP config portal ready: http://") + WiFi.softAPIP().toString() + "/wifi");
  }
  else
  {
    logLine(String("HTTP upload ready: http://") + WiFi.localIP().toString() + "/");
  }
}

static void handleTelnetClient(void)
{
  WiFiClient new_client = telnet_server.accept();
  if (!new_client)
  {
    return;
  }

  if (telnet_client && telnet_client.connected())
  {
    telnet_client.println("New Telnet client connected, closing this session");
    telnet_client.stop();
  }

  telnet_client = new_client;
  telnet_client.setNoDelay(true);
  telnet_client.println("Connected to ESP32 CNC Laser bridge");
  telnet_client.println("Raw terminal: bytes are forwarded to STM32 USART1");
  telnet_client.println("Try STM32 commands: ?, s, h, H, C, J X5 F3000, !, R, x, X, y, Y, z, Z, d");
  printNetworkInfo();
}

static void bridgeUsbToStm32(void)
{
  uint8_t buf[BRIDGE_BUF_SIZE];
  size_t len = 0;

  while ((Serial.available() > 0) && (len < sizeof(buf)))
  {
    buf[len++] = (uint8_t)Serial.read();
  }

  if (len > 0)
  {
    stm32_uart.write(buf, len);
  }
}

static void bridgeTelnetToStm32(void)
{
  uint8_t buf[BRIDGE_BUF_SIZE];
  size_t len = 0;

  if (!telnet_client || !telnet_client.connected())
  {
    return;
  }

  while ((telnet_client.available() > 0) && (len < sizeof(buf)))
  {
    buf[len++] = (uint8_t)telnet_client.read();
  }

  if (len > 0)
  {
    stm32_uart.write(buf, len);
  }
}

static void bridgeStm32ToOutputs(void)
{
  uint8_t buf[BRIDGE_BUF_SIZE];
  size_t len = 0;

  while ((stm32_uart.available() > 0) && (len < sizeof(buf)))
  {
    buf[len++] = (uint8_t)stm32_uart.read();
  }

  if (len > 0)
  {
    for (size_t i = 0; i < len; i++)
    {
      handleStm32JobByte(buf[i]);
    }
    Serial.write(buf, len);
    writeToTelnet(buf, len);
  }
}

static void writeToTelnet(const uint8_t *data, size_t len)
{
  if (telnet_client && telnet_client.connected())
  {
    telnet_client.write(data, len);
  }
}

static void printNetworkInfo(void)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    if (config_ap_active)
    {
      logLine(String("Mode: Config AP, SSID: ") + CONFIG_AP_SSID + ", IP: " + WiFi.softAPIP().toString());
      logLine(String("Config portal: http://") + WiFi.softAPIP().toString() + "/wifi");
    }
    else
    {
      logLine("Mode: WiFi not connected");
    }
    return;
  }

  logLine(String("Mode: STA, IP: ") + WiFi.localIP().toString());
  logLine(String("OTA upload target: ") + OTA_HOSTNAME + ".local");
  logLine(String("Telnet target: ") + OTA_HOSTNAME + ".local:" + TELNET_PORT);
  logLine(String("Web upload target: http://") + WiFi.localIP().toString() + "/");
  if (config_ap_active)
  {
    logLine(String("Config AP also active: http://") + WiFi.softAPIP().toString() + "/wifi");
  }
}

static void handleRoot(void)
{
  String html;

  if (isConfigOnlyMode())
  {
    http_server.sendHeader("Location", "/wifi", true);
    http_server.send(302, "text/plain", "wifi setup\n");
    return;
  }

  html.reserve(14000);
  html += F("<!doctype html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>CNC Laser Console</title>");
  html += F("<style>");
  html += F(":root{color-scheme:light;--ink:#18212f;--muted:#667085;--line:#d7dde8;--panel:#fff;--accent:#0f766e;--blue:#2563eb;--danger:#b42318;--bg:#f4f7fb;--soft:#eef3f8}");
  html += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif}");
  html += F(".wrap{max-width:1160px;margin:0 auto;padding:20px 14px 34px}.top{display:flex;justify-content:space-between;gap:16px;align-items:flex-start;margin-bottom:14px}");
  html += F("h1{font-size:28px;line-height:1.1;margin:0}.sub{color:var(--muted);margin-top:6px}.pill{display:inline-flex;align-items:center;gap:8px;border:1px solid var(--line);background:#eef6f5;padding:8px 12px;border-radius:999px;font-weight:800}");
  html += F(".dot{width:10px;height:10px;border-radius:50%;background:var(--accent)}.grid{display:grid;grid-template-columns:1fr 1fr .95fr;gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;box-shadow:0 10px 24px rgba(25,36,64,.08)}");
  html += F("h2{font-size:18px;margin:0 0 12px}.drop{border:2px dashed #a8b3c4;border-radius:8px;min-height:150px;display:flex;align-items:center;justify-content:center;text-align:center;padding:18px;background:#fbfcff;cursor:pointer;transition:.15s}");
  html += F(".drop.drag{border-color:var(--blue);background:#eef5ff}.drop strong{font-size:18px}.drop span{display:block;color:var(--muted);margin-top:8px}.file{margin-top:10px;color:var(--muted);min-height:22px}");
  html += F(".actions{display:flex;gap:9px;flex-wrap:wrap;margin-top:12px}button{appearance:none;border:0;border-radius:8px;padding:11px 13px;font-size:14px;font-weight:800;cursor:pointer;min-height:42px}");
  html += F(".primary{background:var(--accent);color:#fff}.secondary{background:#e8eef7;color:#1f2a44}.blue{background:var(--blue);color:#fff}.danger{background:#fee4e2;color:var(--danger)}button:disabled{opacity:.45;cursor:not-allowed}");
  html += F(".bar{height:9px;background:#e8eef7;border-radius:999px;overflow:hidden;margin-top:12px}.bar>div{height:100%;width:0;background:var(--blue);transition:width .2s}");
  html += F(".jog{display:grid;grid-template-columns:70px 70px 70px;grid-template-rows:54px 54px 54px;gap:8px;align-items:stretch;justify-content:center}.jog button{font-size:18px}.jog .blank{visibility:hidden}.step{background:#fff7d6;color:#473a05;border:1px solid #efd777}");
  html += F(".stepbar{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:12px 0}.stepbar button{background:#edf2f7;color:#23304a}.stepbar button.on{background:#fff1b8;color:#3f3000;border:1px solid #e6c85f}");
  html += F(".cards{display:grid;grid-template-columns:repeat(4,1fr);gap:9px}.card{border:1px solid var(--line);border-radius:8px;padding:10px;background:#fbfcff}.label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.04em}.value{font-size:22px;font-weight:850;margin-top:3px}");
  html += F("pre{white-space:pre-wrap;word-break:break-word;background:#111827;color:#f9fafb;border-radius:8px;padding:11px;min-height:144px;margin:10px 0 0}.msg{color:var(--muted);font-size:14px;line-height:1.45;margin-top:10px;min-height:20px}.links a{color:var(--blue);text-decoration:none;font-weight:800}");
  html += F("@media(max-width:980px){.grid{grid-template-columns:1fr 1fr}.status{grid-column:1/-1}}@media(max-width:700px){.top,.grid{display:block}.pill{margin-top:12px}.panel{margin-top:12px}.cards{grid-template-columns:1fr 1fr}.jog{grid-template-columns:1fr 1fr 1fr}}");
  html += F("</style></head><body><main class='wrap'>");
  html += F("<section class='top'><div><h1>CNC Laser Console</h1><div class='sub'>Import G-code, jog truc, home/center, update STM32 qua WiFi.</div></div><div class='pill'><i class='dot'></i><span id='state'>");
  html += jobStateName(job_state);
  html += F("</span></div></section>");
  html += F("<section class='grid'><div class='panel'><h2>Import</h2>");
  html += F("<div id='drop' class='drop'><div><strong>Drop G-code</strong><span>.gcode .nc .gc .txt</span></div></div>");
  html += F("<input id='file' type='file' accept='.gcode,.gc,.nc,.txt' hidden><div id='fileName' class='file'>Chua chon file</div>");
  html += F("<div class='actions'><button id='upload' class='primary' disabled>Upload + Start</button><button id='runLast' class='blue'>Run Stored</button><button id='clear' class='secondary'>Clear</button></div>");
  html += F("<div class='bar'><div id='progress'></div></div><div id='uploadMsg' class='msg'>Ready.</div>");
  html += F("<div class='actions'><button id='homeCenter' class='secondary'>Home + Center</button><button id='center' class='secondary'>Center</button><button id='stop' class='danger'>Stop</button></div>");
  html += F("<p class='msg links'><a href='/firmware'>STM32 firmware</a> | <a href='/wifi'>WiFi setup</a> | <a href='/status'>Plain status</a></p></div>");

  html += F("<div class='panel'><h2>Manual</h2>");
  html += F("<div class='stepbar'><button data-step='1'>1mm</button><button data-step='5' class='on'>5mm</button><button data-step='10'>10mm</button><button data-step='50'>50mm</button></div>");
  html += F("<div class='jog'><span class='blank'></span><button data-x='0' data-y='1' class='blue'>Y+</button><span class='blank'></span><button data-x='-1' data-y='0' class='blue'>X-</button><button id='stepRead' class='step'>5mm</button><button data-x='1' data-y='0' class='blue'>X+</button><span class='blank'></span><button data-x='0' data-y='-1' class='blue'>Y-</button><span class='blank'></span></div>");
  html += F("<div class='actions'><button id='zUp' class='secondary'>Z Up</button><button id='zDown' class='secondary'>Z Down</button><button id='laserWork' class='primary'>Work + Laser</button><button id='laserOff' class='danger'>Laser Off</button></div><div id='manualMsg' class='msg'>Manual jog keeps Z unchanged.</div></div>");

  html += F("<div class='panel status'><h2>Trang thai</h2><div class='cards'><div class='card'><div class='label'>Sent</div><div id='sent' class='value'>");
  html += job_lines_sent;
  html += F("</div></div><div class='card'><div class='label'>OK</div><div id='ok' class='value'>");
  html += job_lines_ok;
  html += F("</div></div><div class='card'><div class='label'>Skip</div><div id='skipped' class='value'>");
  html += job_lines_skipped;
  html += F("</div></div><div class='card'><div class='label'>Job</div><div id='progressPct' class='value'>");
  html += (int)currentJobProgressPercent();
  html += F("%</div></div></div><div class='bar'><div id='jobProgress'></div></div><pre id='statusText'>Loading...</pre>");
  html += F("</div></section></main>");
  html += F("<script>");
  html += F("const fileInput=document.getElementById('file'),drop=document.getElementById('drop'),uploadBtn=document.getElementById('upload'),clearBtn=document.getElementById('clear'),stopBtn=document.getElementById('stop'),fileName=document.getElementById('fileName'),progress=document.getElementById('progress'),uploadMsg=document.getElementById('uploadMsg'),manualMsg=document.getElementById('manualMsg');let selected=null,step=5;");
  html += F("function setFile(f){selected=f||null;fileName.textContent=selected?selected.name+' ('+Math.ceil(selected.size/1024)+' KB)':'Chua chon file';uploadBtn.disabled=!selected;progress.style.width='0%';}");
  html += F("drop.onclick=()=>fileInput.click();fileInput.onchange=()=>setFile(fileInput.files[0]);['dragenter','dragover'].forEach(e=>drop.addEventListener(e,x=>{x.preventDefault();drop.classList.add('drag')}));['dragleave','drop'].forEach(e=>drop.addEventListener(e,x=>{x.preventDefault();drop.classList.remove('drag')}));drop.addEventListener('drop',e=>setFile(e.dataTransfer.files[0]));clearBtn.onclick=()=>{fileInput.value='';setFile(null);uploadMsg.textContent='Da xoa file dang chon.'};");
  html += F("uploadBtn.onclick=()=>{if(!selected)return;const fd=new FormData();fd.append('file',selected,selected.name);const xhr=new XMLHttpRequest();xhr.open('POST','/upload');uploadBtn.disabled=true;uploadMsg.textContent='Dang upload...';xhr.upload.onprogress=e=>{if(e.lengthComputable)progress.style.width=Math.round(e.loaded*100/e.total)+'%'};xhr.onload=()=>{progress.style.width='100%';uploadMsg.textContent=xhr.status<300?'Upload xong, job da bat dau.':'Upload loi: '+xhr.responseText;uploadBtn.disabled=false;poll()};xhr.onerror=()=>{uploadMsg.textContent='Khong ket noi duoc ESP32 khi upload.';uploadBtn.disabled=false};xhr.send(fd)};");
  html += F("function api(url,msg){manualMsg.textContent=msg||'Dang gui...';return fetch(url,{method:'POST',cache:'no-store'}).then(r=>r.text().then(t=>{manualMsg.textContent=(r.ok?t:'Loi: '+t).trim();poll();return r.ok})).catch(()=>{manualMsg.textContent='Mat ket noi toi ESP32';return false})}");
  html += F("document.querySelectorAll('[data-step]').forEach(b=>b.onclick=()=>{step=Number(b.dataset.step);document.querySelectorAll('[data-step]').forEach(x=>x.classList.toggle('on',x===b));document.getElementById('stepRead').textContent=step+'mm'});");
  html += F("document.querySelectorAll('[data-x]').forEach(b=>b.onclick=()=>api('/jog?x='+(Number(b.dataset.x)*step)+'&y='+(Number(b.dataset.y)*step),'Jog '+step+'mm'));");
  html += F("document.getElementById('stepRead').onclick=()=>{const a=[1,5,10,50];step=a[(a.indexOf(step)+1)%a.length];document.querySelector('[data-step=\"'+step+'\"]').click()};");
  html += F("document.getElementById('zUp').onclick=()=>api('/z?dir=up','Z up');document.getElementById('zDown').onclick=()=>api('/z?dir=down','Z down');document.getElementById('laserWork').onclick=()=>api('/laser-work','Z work + laser on');document.getElementById('laserOff').onclick=()=>api('/laser-off','Laser off');");
  html += F("document.getElementById('homeCenter').onclick=()=>api('/home-center','Home + center');document.getElementById('center').onclick=()=>api('/center','Center');document.getElementById('runLast').onclick=()=>api('/run-last','Run stored G-code');");
  html += F("stopBtn.onclick=()=>api('/stop','Stop');");
  html += F("function parseStatus(t){const o={};t.trim().split('\\n').forEach(l=>{const i=l.indexOf('=');if(i>0)o[l.slice(0,i)]=l.slice(i+1)});return o}");
  html += F("function poll(){fetch('/status',{cache:'no-store'}).then(r=>r.text()).then(t=>{const s=parseStatus(t),p=Math.max(0,Math.min(100,Number(s.progress_percent||0)));document.getElementById('state').textContent=s.state||'-';document.getElementById('sent').textContent=s.sent||'0';document.getElementById('ok').textContent=s.ok||'0';document.getElementById('skipped').textContent=s.skipped||'0';document.getElementById('progressPct').textContent=p+'%';document.getElementById('jobProgress').style.width=p+'%';document.getElementById('statusText').textContent=t}).catch(()=>{document.getElementById('statusText').textContent='Mat ket noi toi ESP32'})}");
  html += F("poll();setInterval(poll,1000);</script></body></html>");
  http_server.sendHeader("Cache-Control", "no-store");
  http_server.send(200, "text/html", html);
}

static void handleWifiConfigPage(void)
{
  String html;
  int network_count;

  html.reserve(9000);
  html += F("<!doctype html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>CNC Laser WiFi</title><style>");
  html += F(":root{--ink:#172033;--muted:#667085;--line:#d7dde8;--bg:#f4f7fb;--panel:#fff;--accent:#0f766e;--danger:#b42318}");
  html += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif}.wrap{max-width:760px;margin:0 auto;padding:22px 14px 34px}");
  html += F(".panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:16px;box-shadow:0 10px 24px rgba(25,36,64,.08)}h1{font-size:26px;margin:0 0 8px}.sub,.hint{color:var(--muted);line-height:1.45}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}");
  html += F("label{display:block;font-weight:800;margin-top:12px}input{width:100%;border:1px solid var(--line);border-radius:8px;padding:11px;font-size:15px;background:#fff}input[type=checkbox]{width:auto;margin-right:8px}.actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px}");
  html += F("button,a.btn{border:0;border-radius:8px;padding:11px 13px;font-weight:850;text-decoration:none;display:inline-block}.primary{background:var(--accent);color:#fff}.secondary{background:#e8eef7;color:#1f2a44}.danger{background:#fee4e2;color:var(--danger)}code{background:#edf2f7;padding:2px 5px;border-radius:5px}@media(max-width:650px){.grid{display:block}}");
  html += F("</style></head><body><main class='wrap'><div class='panel'>");
  html += F("<h1>WiFi Setup</h1><div class='sub'>Chon WiFi cho ESP32. Neu ket noi that bai, ESP32 se phat AP <code>");
  html += CONFIG_AP_SSID;
  html += F("</code> tai <code>192.168.4.1</code>. AP nay chi dung de cau hinh WiFi, khong dung de upload/chay job.</div>");
  html += F("<p class='hint'>STA: ");
  html += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("not connected");
  html += F(" | AP: ");
  html += config_ap_active ? WiFi.softAPIP().toString() : String("off");
  html += F("</p><form method='post' action='/wifi-save'>");
  html += F("<label>SSID</label><input name='ssid' list='nets' value='");
  html += htmlEscape(wifi_ssid);
  html += F("' required><datalist id='nets'>");

  network_count = WiFi.scanNetworks(false, true);
  for (int i = 0; i < network_count; i++)
  {
    html += F("<option value='");
    html += htmlEscape(WiFi.SSID(i));
    html += F("'>");
  }
  WiFi.scanDelete();

  html += F("</datalist><label>Password</label><input name='pass' type='password' placeholder='De trong de giu mat khau cu neu SSID khong doi'>");
  html += F("<label>Kieu IP</label><div class='actions'><label><input name='static' type='radio' value='0'");
  if (!wifi_use_static_ip)
  {
    html += F(" checked");
  }
  html += F(">DHCP tu router</label><label><input name='static' type='radio' value='1'");
  if (wifi_use_static_ip)
  {
    html += F(" checked");
  }
  html += F(">IP tinh</label></div><div class='grid'>");
  html += F("<div><label>IP</label><input name='ip' value='");
  html += wifi_local_ip.toString();
  html += F("'></div><div><label>Gateway</label><input name='gw' value='");
  html += wifi_gateway.toString();
  html += F("'></div><div><label>Subnet</label><input name='subnet' value='");
  html += wifi_subnet.toString();
  html += F("'></div><div><label>DNS 1</label><input name='dns1' value='");
  html += wifi_dns1.toString();
  html += F("'></div><div><label>DNS 2</label><input name='dns2' value='");
  html += wifi_dns2.toString();
  html += F("'></div></div><div class='actions'><button class='primary' type='submit'>Save & Restart</button>");
  if (!isConfigOnlyMode())
  {
    html += F("<a class='btn secondary' href='/'>Back</a>");
  }
  html += F("</div></form>");
  html += F("<form method='post' action='/wifi-clear' class='actions'><button class='danger' type='submit'>Clear Saved WiFi</button></form>");
  html += F("</div></main></body></html>");

  http_server.sendHeader("Cache-Control", "no-store");
  http_server.send(200, "text/html", html);
}

static void handleWifiSave(void)
{
  String new_ssid = http_server.arg("ssid");
  String new_pass = http_server.arg("pass");
  bool new_static = http_server.hasArg("static") && (http_server.arg("static") == "1");
  IPAddress new_ip = wifi_local_ip;
  IPAddress new_gw = wifi_gateway;
  IPAddress new_subnet = wifi_subnet;
  IPAddress new_dns1 = wifi_dns1;
  IPAddress new_dns2 = wifi_dns2;

  new_ssid.trim();
  if (new_ssid.length() == 0)
  {
    http_server.send(400, "text/plain", "ssid required\n");
    return;
  }

  if ((new_pass.length() == 0) && (new_ssid == wifi_ssid))
  {
    new_pass = wifi_password;
  }

  if (new_static)
  {
    if (!parseIpSetting(http_server.arg("ip"), &new_ip) ||
        !parseIpSetting(http_server.arg("gw"), &new_gw) ||
        !parseIpSetting(http_server.arg("subnet"), &new_subnet) ||
        !parseIpSetting(http_server.arg("dns1"), &new_dns1) ||
        !parseIpSetting(http_server.arg("dns2"), &new_dns2))
    {
      http_server.send(400, "text/plain", "bad static ip field\n");
      return;
    }
  }

  wifi_ssid = new_ssid;
  wifi_password = new_pass;
  wifi_use_static_ip = new_static;
  wifi_local_ip = new_ip;
  wifi_gateway = new_gw;
  wifi_subnet = new_subnet;
  wifi_dns1 = new_dns1;
  wifi_dns2 = new_dns2;
  saveWifiSettings();

  http_server.send(200, "text/html", "<!doctype html><meta charset='utf-8'><p>WiFi saved. ESP32 restarting...</p>");
  delay(700);
  ESP.restart();
}

static void handleWifiClear(void)
{
  clearWifiSettings(true);
  http_server.send(200, "text/html", "<!doctype html><meta charset='utf-8'><p>Saved WiFi cleared. ESP32 restarting in setup portal...</p>");
  delay(700);
  ESP.restart();
}

static void handleNotFound(void)
{
  if (config_ap_active)
  {
    http_server.sendHeader("Location", "/wifi", true);
    http_server.send(302, "text/plain", "wifi setup\n");
    return;
  }

  http_server.send(404, "text/plain", "not found\n");
}

static String htmlEscape(const String &value)
{
  String escaped;

  escaped.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++)
  {
    char ch = value[i];
    switch (ch)
    {
      case '&':
        escaped += F("&amp;");
        break;
      case '<':
        escaped += F("&lt;");
        break;
      case '>':
        escaped += F("&gt;");
        break;
      case '"':
        escaped += F("&quot;");
        break;
      case '\'':
        escaped += F("&#39;");
        break;
      default:
        escaped += ch;
        break;
    }
  }

  return escaped;
}

static void handleStatus(void)
{
  String text;

  text.reserve(620);
  text += "state=";
  text += jobStateName(job_state);
  text += "\nmessage=";
  text += job_message;
  text += "\nprogress_percent=";
  text += (int)currentJobProgressPercent();
  text += "\njob_pos=";
  text += job_file_pos;
  text += "\njob_size=";
  text += job_file_size;
  text += "\nsent=";
  text += job_lines_sent;
  text += "\nok=";
  text += job_lines_ok;
  text += "\nskipped=";
  text += job_lines_skipped;
  text += "\nfirmware=";
  text += firmware_message;
  text += "\nlast=";
  text += last_stm32_line;
  text += "\nstep_mm=";
  text += formatMm(currentManualStepMm());
  text += "\nlcd=";
  text += lcd_ready ? "ready" : "not_ready";
  text += "\nbacklight_gpio=";
  text += LCD_BACKLIGHT_PIN;
  text += "\nstm32_uart_rx_gpio=";
  text += STM32_UART_RX_PIN;
  text += "\nstm32_uart_tx_gpio=";
  text += STM32_UART_TX_PIN;
  text += "\nrgb_led=";
  text += (HAS_RGB_STATUS_LED != 0) ? "enabled" : "disabled";
  text += "\nrgb_led_gpio=";
  text += RGB_STATUS_LED_PIN;
  text += "\nwifi_reset_button_gpio=";
  text += WIFI_RESET_BUTTON_PIN;
  text += "\nwifi_force_setup=";
  text += wifi_force_setup_portal ? "1" : "0";
  text += "\nota_active=";
  text += ota_update_active ? "1" : "0";
  text += "\nesp32_ota=";
  text += esp32_ota_message;
  text += "\nwifi_mode=";
  if (WiFi.status() == WL_CONNECTED)
  {
    text += config_ap_active ? "sta_ap" : "sta";
  }
  else
  {
    text += config_ap_active ? "ap_config" : "offline";
  }
  text += "\nwifi_ssid=";
  text += wifi_ssid;
  if (WiFi.status() == WL_CONNECTED)
  {
    text += "\nsta_ip=";
    text += WiFi.localIP().toString();
    text += "\nip=";
    text += WiFi.localIP().toString();
  }
  if (config_ap_active)
  {
    text += "\nap_ip=";
    text += WiFi.softAPIP().toString();
  }
  text += "\nheap=";
  text += ESP.getFreeHeap();
  text += "\nuptime_ms=";
  text += millis();
  text += "\n";
  http_server.sendHeader("Cache-Control", "no-store");
  http_server.send(200, "text/plain", text);
}

static void handleStop(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

  stm32_uart.print("!\n");
  stopGcodeJob(JOB_STOPPED, "stopped from web");
  lcd_dirty = true;
  http_server.send(200, "text/plain", "stopped\n");
}

static void handleJog(void)
{
  float x_mm = 0.0f;
  float y_mm = 0.0f;

  if (!requireStaModeForMachine())
  {
    return;
  }

  if (http_server.hasArg("x"))
  {
    x_mm = http_server.arg("x").toFloat();
  }
  if (http_server.hasArg("y"))
  {
    y_mm = http_server.arg("y").toFloat();
  }

  if (!startManualJog(x_mm, y_mm))
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleZJog(void)
{
  String dir = http_server.hasArg("dir") ? http_server.arg("dir") : "";
  dir.toLowerCase();

  if (!requireStaModeForMachine())
  {
    return;
  }

  if ((dir != "up") && (dir != "down"))
  {
    http_server.send(400, "text/plain", "bad z dir\n");
    return;
  }

  if (!startManualZ(dir == "up"))
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleLaserWork(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

  if (!startManualLaserWork())
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleLaserOff(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

  if (!startManualLaserOff())
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleHomeCenter(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

  if (!startManualHomeCenter())
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleCenter(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

  if (!startManualCenter())
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleRunLast(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

  if (!startLastUploadedJob())
  {
    lcd_dirty = true;
    http_server.send(409, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleLcdTest(void)
{
  if (!requireStaModeForMachine())
  {
    return;
  }

#if (HAS_LCD_UI != 0)
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);

  if (lcd_ready)
  {
    lcd.fillScreen(0xFFFF);
    delay(120);
    lcd.fillRect(0, 0, 107, 240, 0xF800);
    lcd.fillRect(107, 0, 106, 240, 0x07E0);
    lcd.fillRect(213, 0, 107, 240, 0x001F);
    lcd.setTextSize(2);
    lcd.setTextColor(0xFFFF, 0x001F);
    lcd.setCursor(222, 110);
    lcd.print("LCD");
    delay(450);
    lcd_dirty = true;
  }

  http_server.send(200, "text/plain", "lcd test: GPIO21 HIGH, RGB pattern drawn\n");
#elif (HAS_RGB_STATUS_LED != 0)
  setStatusRgb(24, 0, 0);
  delay(180);
  setStatusRgb(0, 24, 0);
  delay(180);
  setStatusRgb(0, 0, 24);
  delay(180);
  setStatusRgb(0, 0, 0);
  http_server.send(200, "text/plain", "rgb test: WS2812 GPIO8 pattern drawn\n");
#else
  http_server.send(200, "text/plain", "no LCD/RGB test output configured\n");
#endif
}

static void handleGcodeUploadDone(void)
{
  if (upload_file)
  {
    upload_file.close();
  }

  if (!requireStaModeForMachine())
  {
    if (fs_ready)
    {
      SPIFFS.remove(GCODE_UPLOAD_PATH);
    }
    upload_failed = false;
    return;
  }

  if (upload_failed)
  {
    lcd_dirty = true;
    http_server.send(500, "text/plain", job_message + "\n");
    return;
  }

  if (!startGcodeJob())
  {
    lcd_dirty = true;
    http_server.send(500, "text/plain", job_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", job_message + "\n");
}

static void handleGcodeUploadData(void)
{
  HTTPUpload &upload = http_server.upload();

  if (isConfigOnlyMode())
  {
    upload_failed = true;
    job_message = "config AP only: G-code upload disabled";
    return;
  }

  if (upload.status == UPLOAD_FILE_START)
  {
    upload_failed = false;
    if (isJobRunning())
    {
      upload_failed = true;
      job_message = "upload rejected: job is running";
      return;
    }
    if (!fs_ready)
    {
      upload_failed = true;
      job_message = "upload rejected: SPIFFS not ready";
      return;
    }

    if (upload_file)
    {
      upload_file.close();
    }
    SPIFFS.remove(GCODE_UPLOAD_PATH);
    upload_file = SPIFFS.open(GCODE_UPLOAD_PATH, "w");
    if (!upload_file)
    {
      upload_failed = true;
      job_message = "upload rejected: cannot open file";
      return;
    }
    job_message = String("uploading ") + upload.filename;
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (!upload_failed && upload_file)
    {
      if (upload_file.write(upload.buf, upload.currentSize) != upload.currentSize)
      {
        upload_failed = true;
        job_message = "upload failed: write error";
      }
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (upload_file)
    {
      upload_file.close();
    }
    if (!upload_failed)
    {
      job_message = String("upload complete, bytes=") + upload.totalSize;
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    if (upload_file)
    {
      upload_file.close();
    }
    upload_failed = true;
    job_message = "upload aborted";
  }
}

static void handleFirmwarePage(void)
{
  String html;

  if (!requireStaModeForMachine())
  {
    return;
  }

  html.reserve(2600);
  html += F("<!doctype html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>STM32 Firmware Update</title>");
  html += F("<style>");
  html += F(":root{--ink:#18212f;--muted:#667085;--line:#d7dde8;--panel:#fff;--accent:#0f766e;--danger:#b42318;--bg:#f4f7fb}");
  html += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif}");
  html += F(".wrap{max-width:760px;margin:0 auto;padding:24px 16px 36px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:18px;box-shadow:0 10px 24px rgba(25,36,64,.08)}");
  html += F("h1{font-size:28px;margin:0 0 8px}.sub,.note{color:var(--muted);line-height:1.45}.warn{border-left:4px solid var(--danger);padding:10px 12px;background:#fff4f2;border-radius:4px;margin:14px 0}");
  html += F("input{display:block;width:100%;padding:12px;border:1px solid var(--line);border-radius:8px;margin-top:14px;background:#fbfcff}");
  html += F("button{appearance:none;border:0;border-radius:8px;padding:12px 14px;font-size:15px;font-weight:750;background:var(--accent);color:#fff;cursor:pointer;margin-top:14px}");
  html += F("a{color:#2563eb;text-decoration:none;font-weight:700}pre{white-space:pre-wrap;word-break:break-word;background:#111827;color:#f9fafb;border-radius:8px;padding:12px;min-height:110px}");
  html += F("</style></head><body><main class='wrap'><section class='panel'>");
  html += F("<h1>STM32 Firmware Update</h1>");
  html += F("<div class='sub'>Upload file app .bin da link o dia chi 0x08010000. ESP32 se dua STM32 vao bootloader va ghi qua UART.</div>");
  html += F("<div class='warn'>Lan dau cai bootloader van can ST-LINK. Sau do moi update app qua trang nay.</div>");
  html += F("<form method='post' action='/fw-upload' enctype='multipart/form-data'><input type='file' name='file' accept='.bin' required><button>Upload STM32 app</button></form>");
  html += F("<p class='note'>Dung file: CNC_Laser_Fw/build/Debug/CNC_Laser_Fw.bin. Khong upload bootloader bin vao day.</p>");
  html += F("<pre>");
  html += firmware_message;
  html += F("</pre><p><a href='/'>Back to G-code console</a> | <a href='/status'>Plain status</a></p>");
  html += F("</section></main></body></html>");
  http_server.sendHeader("Cache-Control", "no-store");
  http_server.send(200, "text/html", html);
}

static void handleFirmwareUploadDone(void)
{
  if (upload_file)
  {
    upload_file.close();
  }

  if (!requireStaModeForMachine())
  {
    if (fs_ready)
    {
      SPIFFS.remove(STM32_APP_UPLOAD_PATH);
    }
    firmware_upload_failed = false;
    return;
  }

  if (firmware_upload_failed)
  {
    http_server.send(500, "text/plain", firmware_message + "\n");
    return;
  }

  if (!runStm32FirmwareUpdate())
  {
    http_server.send(500, "text/plain", firmware_message + "\n");
    return;
  }

  http_server.send(200, "text/plain", firmware_message + "\n");
}

static void handleFirmwareUploadData(void)
{
  HTTPUpload &upload = http_server.upload();

  if (isConfigOnlyMode())
  {
    firmware_upload_failed = true;
    firmware_message = "config AP only: STM32 firmware upload disabled";
    return;
  }

  if (upload.status == UPLOAD_FILE_START)
  {
    firmware_upload_failed = false;
    if (isJobRunning())
    {
      firmware_upload_failed = true;
      firmware_message = "firmware upload rejected: G-code job is running";
      return;
    }
    if (!fs_ready)
    {
      firmware_upload_failed = true;
      firmware_message = "firmware upload rejected: SPIFFS not ready";
      return;
    }

    if (upload_file)
    {
      upload_file.close();
    }
    SPIFFS.remove(STM32_APP_UPLOAD_PATH);
    upload_file = SPIFFS.open(STM32_APP_UPLOAD_PATH, "w");
    if (!upload_file)
    {
      firmware_upload_failed = true;
      firmware_message = "firmware upload rejected: cannot open file";
      return;
    }
    firmware_message = String("uploading STM32 app ") + upload.filename;
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (!firmware_upload_failed && upload_file)
    {
      if (upload_file.write(upload.buf, upload.currentSize) != upload.currentSize)
      {
        firmware_upload_failed = true;
        firmware_message = "firmware upload failed: write error";
      }
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (upload_file)
    {
      upload_file.close();
    }
    if (!firmware_upload_failed)
    {
      firmware_message = String("firmware upload complete, bytes=") + upload.totalSize;
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    if (upload_file)
    {
      upload_file.close();
    }
    firmware_upload_failed = true;
    firmware_message = "firmware upload aborted";
  }
}

static void handleEsp32OtaPage(void)
{
  String html;

  if (!requireStaModeForMachine())
  {
    return;
  }

  html.reserve(2200);
  html += F("<!doctype html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>ESP32 OTA</title>");
  html += F("<style>");
  html += F(":root{--ink:#18212f;--muted:#667085;--line:#d7dde8;--panel:#fff;--accent:#2563eb;--danger:#b42318;--bg:#f4f7fb}");
  html += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif}");
  html += F(".wrap{max-width:760px;margin:0 auto;padding:24px 16px 36px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:18px;box-shadow:0 10px 24px rgba(25,36,64,.08)}");
  html += F("h1{font-size:28px;margin:0 0 8px}.sub,.note{color:var(--muted);line-height:1.45}.warn{border-left:4px solid var(--danger);padding:10px 12px;background:#fff4f2;border-radius:4px;margin:14px 0}");
  html += F("input{display:block;width:100%;padding:12px;border:1px solid var(--line);border-radius:8px;margin-top:14px;background:#fbfcff}");
  html += F("button{appearance:none;border:0;border-radius:8px;padding:12px 14px;font-size:15px;font-weight:750;background:var(--accent);color:#fff;cursor:pointer;margin-top:14px}");
  html += F("a{color:#2563eb;text-decoration:none;font-weight:700}pre{white-space:pre-wrap;word-break:break-word;background:#111827;color:#f9fafb;border-radius:8px;padding:12px;min-height:90px}");
  html += F("</style></head><body><main class='wrap'><section class='panel'>");
  html += F("<h1>ESP32 OTA</h1>");
  html += F("<div class='sub'>Upload firmware.bin cua ESP32 qua WiFi chinh. AP fallback chi de cau hinh WiFi.</div>");
  html += F("<div class='warn'>May se dung job hien tai neu co, ghi firmware moi, roi tu restart.</div>");
  html += F("<form method='post' action='/esp32-ota' enctype='multipart/form-data'><input type='file' name='file' accept='.bin' required><button>Upload ESP32 firmware</button></form>");
  html += F("<p class='note'>Dung file: tools/esp32_uart_bridge/.pio/build/esp32c6_supermini_ota/firmware.bin.</p>");
  html += F("<pre>");
  html += esp32_ota_message;
  html += F("</pre><p><a href='/'>Back to G-code console</a> | <a href='/status'>Plain status</a></p>");
  html += F("</section></main></body></html>");
  http_server.sendHeader("Cache-Control", "no-store");
  http_server.send(200, "text/html", html);
}

static void handleEsp32OtaUploadDone(void)
{
  if (!requireStaModeForMachine())
  {
    esp32_ota_failed = false;
    ota_update_active = false;
    return;
  }

  if (esp32_ota_failed)
  {
    ota_update_active = false;
    http_server.send(500, "text/plain", esp32_ota_message + "\n");
    return;
  }

  esp32_ota_message = String("ESP32 OTA complete, bytes=") + esp32_ota_written + ", restarting";
  http_server.send(200, "text/plain", esp32_ota_message + "\n");
  delay(700);
  ESP.restart();
}

static void handleEsp32OtaUploadData(void)
{
  HTTPUpload &upload = http_server.upload();

  if (isConfigOnlyMode())
  {
    esp32_ota_failed = true;
    esp32_ota_message = "config AP only: ESP32 OTA disabled";
    return;
  }

  if (upload.status == UPLOAD_FILE_START)
  {
    prepareEsp32OtaUpdate("HTTP");
    esp32_ota_failed = false;
    esp32_ota_written = 0;
    esp32_ota_message = String("uploading ESP32 firmware ") + upload.filename;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH))
    {
      esp32_ota_failed = true;
      ota_update_active = false;
      esp32_ota_message = String("ESP32 OTA begin failed: ") + Update.errorString();
      logLine(esp32_ota_message);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (!esp32_ota_failed && (upload.currentSize > 0))
    {
      size_t written = Update.write(upload.buf, upload.currentSize);
      esp32_ota_written += written;
      if (written != upload.currentSize)
      {
        esp32_ota_failed = true;
        ota_update_active = false;
        esp32_ota_message = String("ESP32 OTA write failed: ") + Update.errorString();
        Update.abort();
        logLine(esp32_ota_message);
      }
      yield();
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (!esp32_ota_failed)
    {
      if (!Update.end(true))
      {
        esp32_ota_failed = true;
        ota_update_active = false;
        esp32_ota_message = String("ESP32 OTA end failed: ") + Update.errorString();
        logLine(esp32_ota_message);
      }
      else
      {
        esp32_ota_message = String("ESP32 OTA upload received, bytes=") + esp32_ota_written;
        logLine(esp32_ota_message);
      }
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    esp32_ota_failed = true;
    ota_update_active = false;
    esp32_ota_message = "ESP32 OTA upload aborted";
    Update.abort();
    logLine(esp32_ota_message);
  }
}

static void prepareEsp32OtaUpdate(const char *source)
{
  ota_update_active = true;
  ota_last_logged_percent = -1;
  telnet_client.stop();
  if (upload_file)
  {
    upload_file.close();
  }
  if (job_file)
  {
    job_file.close();
  }
  if (isJobRunning())
  {
    stm32_uart.print("!\n");
    stopGcodeJob(JOB_STOPPED, "stopped: ESP32 OTA");
  }
  setStatusRgb(26, 10, 0);
  logLine(String(source) + " ESP32 OTA start");
}

static bool startGcodeJob(void)
{
  return startGcodeJobFromPath(GCODE_UPLOAD_PATH, "streaming uploaded gcode");
}

static bool startGcodeJobFromPath(const char *path, const String &message)
{
  if (!fs_ready)
  {
    job_message = "SPIFFS not ready";
    return false;
  }
  if (isJobRunning())
  {
    job_message = "job already running";
    return false;
  }

  if (job_file)
  {
    job_file.close();
  }
  job_file = SPIFFS.open(path, "r");
  if (!job_file)
  {
    job_message = String("cannot open ") + path;
    return false;
  }

  job_line_len = 0;
  stm32_line_len = 0;
  job_lines_sent = 0;
  job_lines_ok = 0;
  job_lines_skipped = 0;
  job_file_size = job_file.size();
  job_file_pos = 0;
  job_started_ms = millis();
  job_wait_started_ms = 0;
  job_state = JOB_STREAMING;
  job_message = message;
  lcd_dirty = true;
  logLine(job_message);
  return true;
}

static bool writeManualJob(const String &content)
{
  if (!fs_ready)
  {
    job_message = "SPIFFS not ready";
    return false;
  }
  if (isJobRunning())
  {
    job_message = "manual rejected: job is running";
    return false;
  }

  File manual = SPIFFS.open(MANUAL_JOB_PATH, "w");
  if (!manual)
  {
    job_message = "manual rejected: cannot open file";
    return false;
  }

  size_t written = manual.print(content);
  manual.close();
  if (written != content.length())
  {
    job_message = "manual rejected: write error";
    return false;
  }

  return true;
}

static bool startManualJob(const String &label, const String &content)
{
  if (!writeManualJob(content))
  {
    return false;
  }
  return startGcodeJobFromPath(MANUAL_JOB_PATH, label);
}

static bool startManualJog(float x_mm, float y_mm)
{
  String content;
  String label;

  if ((x_mm == 0.0f) && (y_mm == 0.0f))
  {
    job_message = "manual jog rejected: zero move";
    return false;
  }

  content.reserve(64);
  label.reserve(48);
  content += "J";
  label += "manual jog";
  if (x_mm != 0.0f)
  {
    content += " X";
    content += formatMm(x_mm);
    label += " X";
    label += formatMm(x_mm);
  }
  if (y_mm != 0.0f)
  {
    content += " Y";
    content += formatMm(y_mm);
    label += " Y";
    label += formatMm(y_mm);
  }
  content += " F";
  content += formatMm(MANUAL_XY_FEED_MM_MIN);
  content += "\n";

  return startManualJob(label, content);
}

static bool startManualZ(bool up)
{
  return startManualJob(up ? "manual Z up" : "manual Z down", up ? "Z\n" : "z\n");
}

static bool startManualLaserWork(void)
{
  return startManualJob("manual Z work + laser on", "M3 S1200\n");
}

static bool startManualLaserOff(void)
{
  return startManualJob("manual laser off", "M5\n");
}

static bool startManualHomeCenter(void)
{
  return startManualJob("manual home + center", "H\nC\n");
}

static bool startManualCenter(void)
{
  return startManualJob("manual center", "C\n");
}

static bool startLastUploadedJob(void)
{
  return startGcodeJobFromPath(GCODE_UPLOAD_PATH, "streaming stored gcode");
}

static uint8_t currentJobProgressPercent(void)
{
  uint32_t percent;
  size_t pos = job_file_pos;

  if (job_file_size == 0)
  {
    return job_state == JOB_DONE ? 100U : 0U;
  }

  if (job_state == JOB_DONE)
  {
    return 100U;
  }

  if (pos > job_file_size)
  {
    pos = job_file_size;
  }

  percent = (uint32_t)((((uint64_t)pos * 100ULL) + (job_file_size / 2U)) / job_file_size);
  if (percent > 100U)
  {
    percent = 100U;
  }

  return (uint8_t)percent;
}

static float currentManualStepMm(void)
{
  if (manual_step_index >= MANUAL_STEP_COUNT)
  {
    manual_step_index = 1;
  }
  return MANUAL_STEPS_MM[manual_step_index];
}

static String formatMm(float value)
{
  char buf[24];
  snprintf(buf, sizeof(buf), "%.3f", value);
  return String(buf);
}

static void gcodeJobTask(void)
{
  if (job_state == JOB_STREAMING)
  {
    if (readNextGcodeLine())
    {
      sendCurrentGcodeLine();
    }
    else
    {
      if (job_state == JOB_STREAMING)
      {
        stopGcodeJob(JOB_DONE, "job complete");
      }
    }
  }
  else if (job_state == JOB_WAIT_OK)
  {
    if ((millis() - job_wait_started_ms) > STM32_OK_TIMEOUT_MS)
    {
      stm32_uart.print("!\n");
      stopGcodeJob(JOB_ERROR, "timeout waiting for STM32 ok");
    }
  }
}

static bool readNextGcodeLine(void)
{
  int ch;

  job_line_len = 0;
  while (job_file && job_file.available())
  {
    ch = job_file.read();
    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      job_line_buf[job_line_len] = '\0';
      if (isRunnableGcodeLine(job_line_buf))
      {
        job_file_pos = job_file.position();
        return true;
      }
      job_line_len = 0;
      job_lines_skipped++;
      job_file_pos = job_file.position();
      continue;
    }
    if (job_line_len >= (GCODE_LINE_BUF_SIZE - 1))
    {
      job_file_pos = job_file.position();
      stopGcodeJob(JOB_ERROR, "gcode line too long");
      return false;
    }
    job_line_buf[job_line_len++] = (char)ch;
  }

  if (job_line_len > 0)
  {
    job_line_buf[job_line_len] = '\0';
    if (isRunnableGcodeLine(job_line_buf))
    {
      job_file_pos = job_file.position();
      return true;
    }
    job_lines_skipped++;
    job_file_pos = job_file.position();
  }

  return false;
}

static bool isRunnableGcodeLine(const char *line)
{
  while ((*line == ' ') || (*line == '\t'))
  {
    line++;
  }

  return ((*line != '\0') && (*line != ';') && (*line != '('));
}

static void sendCurrentGcodeLine(void)
{
  stm32_uart.print(job_line_buf);
  stm32_uart.print('\n');
  job_lines_sent++;
  job_wait_started_ms = millis();
  job_state = JOB_WAIT_OK;
  job_message = String("sent line ") + job_lines_sent + ": " + job_line_buf;
}

static void stopGcodeJob(GcodeJobState state, const String &message)
{
  if (state == JOB_DONE)
  {
    job_file_pos = job_file_size;
  }
  if (job_file)
  {
    job_file.close();
  }
  job_state = state;
  job_message = message;
  lcd_dirty = true;
}

static bool isJobRunning(void)
{
  return (job_state == JOB_STREAMING) || (job_state == JOB_WAIT_OK);
}

static const char *jobStateName(GcodeJobState state)
{
  switch (state)
  {
    case JOB_IDLE:
      return "idle";
    case JOB_STREAMING:
      return "streaming";
    case JOB_WAIT_OK:
      return "wait_ok";
    case JOB_DONE:
      return "done";
    case JOB_ERROR:
      return "error";
    case JOB_STOPPED:
      return "stopped";
    default:
      return "unknown";
  }
}

static void handleStm32JobByte(uint8_t data)
{
  if ((data == '\r') || (data == '\n'))
  {
    if (stm32_line_len > 0)
    {
      stm32_line_buf[stm32_line_len] = '\0';
      handleStm32JobLine(stm32_line_buf);
      stm32_line_len = 0;
    }
    return;
  }

  if (stm32_line_len < (STM32_LINE_BUF_SIZE - 1))
  {
    stm32_line_buf[stm32_line_len++] = (char)data;
  }
  else
  {
    stm32_line_len = 0;
  }
}

static void handleStm32JobLine(const char *line)
{
  last_stm32_line = line;
  lcd_dirty = true;

  if (job_state != JOB_WAIT_OK)
  {
    return;
  }

  while ((*line == ' ') || (*line == '\t'))
  {
    line++;
  }

  if ((line[0] == 'o') && (line[1] == 'k'))
  {
    job_lines_ok++;
    job_state = JOB_STREAMING;
    job_message = String("ok line ") + job_lines_ok;
    lcd_dirty = true;
  }
  else if (strncmp(line, "error", 5) == 0)
  {
    stm32_uart.print("!\n");
    stopGcodeJob(JOB_ERROR, String("STM32 ") + line);
  }
}

static void setupStatusLed(void)
{
#if (HAS_RGB_STATUS_LED != 0)
  pinMode(RGB_STATUS_LED_PIN, OUTPUT);
  rgb_status_led_ready = true;
  setStatusRgb(0, 0, 0);
#else
  rgb_status_led_ready = false;
#endif
}

static void statusLedTask(void)
{
#if (HAS_RGB_STATUS_LED != 0)
  uint32_t now_ms = millis();
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  bool blink = false;
  bool on_phase = true;

  if (!rgb_status_led_ready || ((now_ms - last_rgb_status_ms) < 120UL))
  {
    return;
  }
  last_rgb_status_ms = now_ms;

  if (firmware_message.indexOf("firmware update failed") >= 0)
  {
    r = 28;
    blink = true;
  }
  else if ((firmware_message.indexOf("firmware sent ") >= 0) ||
           (firmware_message.indexOf("STM32 bootloader") >= 0) ||
           (firmware_message.indexOf("requested STM32 bootloader") >= 0))
  {
    r = 26;
    g = 10;
    blink = true;
  }
  else if (job_state == JOB_ERROR)
  {
    r = 28;
    blink = true;
  }
  else if (isJobRunning())
  {
    b = 28;
  }
  else if (job_state == JOB_STOPPED)
  {
    r = 24;
    g = 16;
  }
  else if (config_ap_active && (WiFi.status() != WL_CONNECTED))
  {
    r = 16;
    b = 24;
    blink = true;
  }
  else if (WiFi.status() == WL_CONNECTED)
  {
    g = 18;
  }
  else
  {
    r = 20;
    g = 10;
    blink = true;
  }

  if (blink)
  {
    on_phase = ((now_ms / 350UL) & 1UL) == 0UL;
  }
  setStatusRgb(on_phase ? r : 0, on_phase ? g : 0, on_phase ? b : 0);
#endif
}

static void setStatusRgb(uint8_t r, uint8_t g, uint8_t b)
{
#if (HAS_RGB_STATUS_LED != 0)
  if ((r == rgb_status_r) && (g == rgb_status_g) && (b == rgb_status_b))
  {
    return;
  }
  rgb_status_r = r;
  rgb_status_g = g;
  rgb_status_b = b;
  rgbLedWrite(RGB_STATUS_LED_PIN, r, g, b);
#else
  (void)r;
  (void)g;
  (void)b;
#endif
}

static void setupLcdUi(void)
{
#if (HAS_LCD_UI != 0)
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  delay(20);

  lcd.init();
  lcd.setRotation(1);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  lcd_ready = true;
  lcd_dirty = true;
  drawLcdUi(true);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
#else
  lcd_ready = false;
  lcd_dirty = false;
#endif
}

static void lcdUiTask(void)
{
#if (HAS_LCD_UI != 0)
  if (!lcd_ready)
  {
    return;
  }

  handleLcdTouch();
  drawLcdUi(false);
#endif
}

#if (HAS_LCD_UI != 0)
static void drawLcdUi(bool force)
{
  uint32_t now_ms = millis();
  uint8_t progress_percent = currentJobProgressPercent();

  if (!force && !lcd_dirty && ((now_ms - last_lcd_draw_ms) < 1000UL))
  {
    return;
  }

  last_lcd_draw_ms = now_ms;
  lcd_dirty = false;
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);

  lcd.fillScreen(UI_BG);
  lcd.fillRect(0, 0, 320, 34, UI_PANEL);
  lcd.setTextSize(2);
  lcd.setTextColor(UI_TEXT, UI_PANEL);
  lcd.setCursor(8, 9);
  lcd.print("CNC Laser");
  lcd.setTextColor(isJobRunning() ? UI_BLUE : UI_ACCENT, UI_PANEL);
  lcd.setCursor(214, 9);
  lcd.print(jobStateName(job_state));

  for (size_t i = 0; i < LCD_BUTTON_COUNT; i++)
  {
    char step_label[12];
    const LcdButton &button = LCD_BUTTONS[i];
    const char *label = button.label;
    uint16_t text_color = (button.color == 0x7BEF) || (button.action == LCD_ACT_STEP) ? UI_TEXT : 0xFFFF;

    if (button.action == LCD_ACT_STEP)
    {
      snprintf(step_label, sizeof(step_label), "%.0fmm", currentManualStepMm());
      label = step_label;
    }
    drawLcdButton(button.x, button.y, button.w, button.h, label, button.color, text_color);
  }

  lcd.setTextSize(1);
  lcd.setTextColor(0xFFFF, UI_BG);
  lcd.setCursor(8, 208);
  lcd.print("P:");
  lcd.print(progress_percent);
  lcd.print("% S:");
  lcd.print(job_lines_sent);
  lcd.print(" OK:");
  lcd.print(job_lines_ok);
  lcd.print(" SK:");
  lcd.print(job_lines_skipped);
  if (WiFi.status() == WL_CONNECTED)
  {
    lcd.setCursor(112, 208);
    lcd.print(WiFi.localIP().toString());
  }
  else if (config_ap_active)
  {
    lcd.setCursor(112, 208);
    lcd.print("AP ");
    lcd.print(WiFi.softAPIP().toString());
  }

  lcd.setTextColor(0xE71C, UI_BG);
  lcd.setCursor(8, 224);
  String line = last_stm32_line.length() > 0 ? last_stm32_line : job_message;
  if (line.length() > 48)
  {
    line = line.substring(0, 48);
  }
  lcd.print(line);
}

static void drawLcdButton(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t color, uint16_t text_color)
{
  int16_t text_w = (int16_t)strlen(label) * 12;
  int16_t text_x = x + ((w - text_w) / 2);
  int16_t text_y = y + ((h - 16) / 2);

  if (text_x < (x + 4))
  {
    text_x = x + 4;
  }
  if (text_y < (y + 4))
  {
    text_y = y + 4;
  }

  lcd.fillRoundRect(x, y, w, h, 6, color);
  lcd.drawRoundRect(x, y, w, h, 6, UI_LINE);
  lcd.setTextSize(2);
  lcd.setTextColor(text_color, color);
  lcd.setCursor(text_x, text_y);
  lcd.print(label);
}

static void handleLcdTouch(void)
{
  uint16_t touch_x = 0;
  uint16_t touch_y = 0;

  if (!lcd.getTouch(&touch_x, &touch_y))
  {
    return;
  }

  if ((millis() - last_touch_ms) < 250UL)
  {
    return;
  }
  last_touch_ms = millis();

  for (size_t i = 0; i < LCD_BUTTON_COUNT; i++)
  {
    const LcdButton &button = LCD_BUTTONS[i];
    if ((touch_x >= button.x) && (touch_x < (button.x + button.w)) &&
        (touch_y >= button.y) && (touch_y < (button.y + button.h)))
    {
      runLcdAction(button.action);
      return;
    }
  }
}

static void runLcdAction(uint8_t action)
{
  float step_mm = currentManualStepMm();
  bool started = false;

  switch (action)
  {
    case LCD_ACT_X_MINUS:
      started = startManualJog(-step_mm, 0.0f);
      break;
    case LCD_ACT_X_PLUS:
      started = startManualJog(step_mm, 0.0f);
      break;
    case LCD_ACT_Y_PLUS:
      started = startManualJog(0.0f, step_mm);
      break;
    case LCD_ACT_Y_MINUS:
      started = startManualJog(0.0f, -step_mm);
      break;
    case LCD_ACT_Z_UP:
      started = startManualZ(true);
      break;
    case LCD_ACT_Z_DOWN:
      started = startManualZ(false);
      break;
    case LCD_ACT_STEP:
      manual_step_index = (manual_step_index + 1U) % MANUAL_STEP_COUNT;
      job_message = String("manual step ") + formatMm(currentManualStepMm()) + "mm";
      started = true;
      break;
    case LCD_ACT_HOME_CENTER:
      started = startManualHomeCenter();
      break;
    case LCD_ACT_CENTER:
      started = startManualCenter();
      break;
    case LCD_ACT_RUN_LAST:
      started = startLastUploadedJob();
      break;
    case LCD_ACT_STOP:
      stm32_uart.print("!\n");
      stopGcodeJob(JOB_STOPPED, "stopped from LCD");
      started = true;
      break;
    default:
      break;
  }

  if (!started)
  {
    logLine(job_message);
  }
  lcd_dirty = true;
}
#else
static void drawLcdUi(bool force)
{
  (void)force;
}
#endif

static bool runStm32FirmwareUpdate(void)
{
  uint8_t buf[STM32_FW_CHUNK_SIZE];
  uint32_t image_size = 0;
  uint32_t image_crc = 0;
  uint32_t sent = 0;

  if (isJobRunning())
  {
    firmware_message = "firmware update rejected: G-code job is running";
    return false;
  }

  if (!calcFileCrc32(STM32_APP_UPLOAD_PATH, &image_size, &image_crc))
  {
    return false;
  }

  firmware_message = String("STM32 app size=") + image_size + " crc=0x" + String(image_crc, HEX);
  logLine(firmware_message);

  if (!waitStm32Bootloader())
  {
    return false;
  }

  stm32_uart.printf("FWUP %lu %08lX\n", (unsigned long)image_size, (unsigned long)image_crc);
  if (!waitStm32FirmwareContains("READY DATA", STM32_BOOTLOADER_LINE_TIMEOUT_MS))
  {
    return false;
  }

  File app = SPIFFS.open(STM32_APP_UPLOAD_PATH, "r");
  if (!app)
  {
    firmware_message = "firmware update failed: cannot reopen STM32 app";
    return false;
  }

  while (sent < image_size)
  {
    String line;
    uint32_t offset = 0;
    uint32_t len = 0;

    if (!waitStm32FirmwareLine(line, STM32_BOOTLOADER_LINE_TIMEOUT_MS))
    {
      app.close();
      firmware_message = "firmware update failed: timeout waiting NEXT";
      return false;
    }

    if (line.startsWith("ERR"))
    {
      app.close();
      firmware_message = String("firmware update failed: STM32 ") + line;
      return false;
    }

    if (!parseNextRequest(line, &offset, &len))
    {
      continue;
    }

    if ((offset != sent) || (len == 0U) || (len > STM32_FW_CHUNK_SIZE) || ((sent + len) > image_size))
    {
      app.close();
      firmware_message = "firmware update failed: bad NEXT request";
      return false;
    }

    size_t read_len = app.read(buf, len);
    if (read_len != len)
    {
      app.close();
      firmware_message = "firmware update failed: file read short";
      return false;
    }

    stm32_uart.write(buf, read_len);
    stm32_uart.flush();
    sent += len;
    firmware_message = String("firmware sent ") + sent + "/" + image_size;
    yield();
  }

  app.close();

  if (!waitStm32FirmwareContains("OK FW", STM32_BOOTLOADER_LINE_TIMEOUT_MS))
  {
    return false;
  }

  firmware_message = String("firmware update done, bytes=") + image_size;
  logLine(firmware_message);
  return true;
}

static bool calcFileCrc32(const char *path, uint32_t *size_out, uint32_t *crc_out)
{
  uint8_t buf[STM32_FW_CHUNK_SIZE];
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t total = 0;
  File app = SPIFFS.open(path, "r");

  if (!app)
  {
    firmware_message = "firmware update failed: cannot open STM32 app";
    return false;
  }

  while (app.available())
  {
    size_t len = app.read(buf, sizeof(buf));
    if (len == 0)
    {
      break;
    }
    crc = crc32Update(crc, buf, len);
    total += (uint32_t)len;
    yield();
  }
  app.close();

  if (total == 0U)
  {
    firmware_message = "firmware update failed: empty app file";
    return false;
  }

  *size_out = total;
  *crc_out = crc ^ 0xFFFFFFFFUL;
  return true;
}

static uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static bool waitStm32FirmwareLine(String &line, uint32_t timeout_ms)
{
  const uint32_t started_ms = millis();

  while ((millis() - started_ms) < timeout_ms)
  {
    while (stm32_uart.available() > 0)
    {
      uint8_t data = (uint8_t)stm32_uart.read();

      if ((data == '\r') || (data == '\n'))
      {
        if (firmware_line_len > 0)
        {
          firmware_line_buf[firmware_line_len] = '\0';
          line = firmware_line_buf;
          firmware_line_len = 0;
          logLine(String("STM32 FW: ") + line);
          return true;
        }
        continue;
      }

      if ((data < 0x20U) || (data > 0x7EU))
      {
        continue;
      }

      if (firmware_line_len < (STM32_LINE_BUF_SIZE - 1))
      {
        firmware_line_buf[firmware_line_len++] = (char)data;
      }
      else
      {
        firmware_line_len = 0;
      }
    }
    delay(2);
    yield();
  }

  return false;
}

static bool waitStm32FirmwareContains(const char *pattern, uint32_t timeout_ms)
{
  const uint32_t started_ms = millis();

  while ((millis() - started_ms) < timeout_ms)
  {
    String line;
    uint32_t remaining_ms = timeout_ms - (millis() - started_ms);
    if (!waitStm32FirmwareLine(line, remaining_ms > 1000UL ? 1000UL : remaining_ms))
    {
      continue;
    }
    if (line.indexOf(pattern) >= 0)
    {
      return true;
    }
    if (line.startsWith("ERR"))
    {
      firmware_message = String("firmware update failed: STM32 ") + line;
      return false;
    }
  }

  firmware_message = String("firmware update failed: timeout waiting ") + pattern;
  return false;
}

static bool waitStm32Bootloader(void)
{
  uint32_t started_ms;
  uint32_t last_ping_ms = 0;

  firmware_line_len = 0;
  while (stm32_uart.available() > 0)
  {
    (void)stm32_uart.read();
  }

  stm32_uart.print("B\n");
  firmware_message = "requested STM32 bootloader";
  logLine(firmware_message);

  started_ms = millis();
  while ((millis() - started_ms) < STM32_BOOTLOADER_WAIT_MS)
  {
    String line;
    if ((millis() - last_ping_ms) > 500UL)
    {
      stm32_uart.print("PING\n");
      last_ping_ms = millis();
    }

    if (waitStm32FirmwareLine(line, 100))
    {
      if ((line.indexOf("BL READY") >= 0) || (line.indexOf("PONG") >= 0))
      {
        firmware_message = "STM32 bootloader ready";
        logLine(firmware_message);
        return true;
      }
    }
  }

  firmware_message = "firmware update failed: STM32 bootloader not responding";
  return false;
}

static bool parseNextRequest(const String &line, uint32_t *offset, uint32_t *len)
{
  const char *p = line.c_str();
  char *end = nullptr;
  unsigned long parsed_offset;
  unsigned long parsed_len;

  if (strncmp(p, "NEXT ", 5) != 0)
  {
    return false;
  }

  p += 5;
  parsed_offset = strtoul(p, &end, 10);
  if (end == p)
  {
    return false;
  }

  p = end;
  parsed_len = strtoul(p, &end, 10);
  if (end == p)
  {
    return false;
  }

  *offset = (uint32_t)parsed_offset;
  *len = (uint32_t)parsed_len;
  return true;
}
