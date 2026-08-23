#pragma once

// Copy this file to include/wifi_config.h and fill in your WiFi details.
// include/wifi_config.h is git-ignored so credentials stay local.

#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

#define WIFI_USE_STATIC_IP 1
#define WIFI_LOCAL_IP IPAddress(192, 168, 10, 140)
#define WIFI_GATEWAY IPAddress(192, 168, 10, 1)
#define WIFI_SUBNET IPAddress(255, 255, 255, 0)
#define WIFI_DNS1 IPAddress(192, 168, 10, 1)
#define WIFI_DNS2 IPAddress(8, 8, 8, 8)

// OTA target will be cnc-laser-esp32.local if mDNS works on your network.
#define OTA_HOSTNAME "cnc-laser-esp32"

// Change this password before using OTA on a shared network.
#define OTA_PASSWORD "cnclaser"

#define TELNET_PORT 23
