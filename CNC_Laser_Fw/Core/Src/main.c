/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  const char *name;
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *ena_port;
  uint16_t ena_pin;
  uint32_t step_period_us;
} AxisDebug_t;

typedef struct
{
  const char *name;
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t mask;
} LimitDebug_t;

typedef struct
{
  const char *home_name;
  const AxisDebug_t *axis;
  GPIO_PinState home_dir;
  uint32_t max_pulses;
  uint32_t estimated_start_um;
  uint8_t home_limit_mask;
} HomeDebug_t;

typedef enum
{
  MOTION_RUN_IDLE = 0,
  MOTION_RUN_ACTIVE,
  MOTION_RUN_DONE,
  MOTION_RUN_ABORTED
} MotionRunState_t;

typedef struct
{
  volatile MotionRunState_t state;
  volatile uint8_t limit_abort;
  volatile uint32_t step_index;
  uint32_t major_steps;
  uint32_t x_steps;
  uint32_t y_steps;
  uint32_t z_steps;
  uint32_t x_acc;
  uint32_t y_acc;
  uint32_t z_acc;
  int32_t dx_steps;
  int32_t dy_steps;
  int32_t dz_steps;
  int32_t target_x_steps;
  int32_t target_y_steps;
  int32_t target_z_steps;
  int32_t target_x_um;
  int32_t target_y_um;
  int32_t target_z_um;
  uint8_t rapid;
} MotionRun_t;

typedef struct
{
  uint8_t active;
  uint8_t failed;
  uint32_t segment_index;
  uint32_t total_segments;
  int32_t center_x_um;
  int32_t center_y_um;
  int32_t target_x_um;
  int32_t target_y_um;
  float radius_um;
  float start_angle;
  float sweep_angle;
  uint32_t feed_mm_min;
} ArcRun_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t record_size;
  uint32_t sequence;
  int32_t x_steps;
  int32_t y_steps;
  int32_t z_steps;
  int32_t x_um;
  int32_t y_um;
  int32_t z_um;
  uint32_t flags;
  uint32_t crc32;
} PositionStoreRecord_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define XY_DEBUG_PULSES_PER_REV 400U
#define XY_DEBUG_XY_STEP_FREQ_HZ 8000U
#define XY_DEBUG_Z_STEP_FREQ_HZ  8000U
#define XY_DEBUG_STEP_HIGH_US   10U
#define XY_DEBUG_XY_STEP_PERIOD_US (1000000U / XY_DEBUG_XY_STEP_FREQ_HZ)
#define XY_DEBUG_Z_STEP_PERIOD_US  (1000000U / XY_DEBUG_Z_STEP_FREQ_HZ)
#define LED_FLASH_PERIOD_MS     1000U
#define LED_FLASH_ON_MS         2U
#define LED_FLASH_ON_STATE      GPIO_PIN_RESET
#define LED_FLASH_OFF_STATE     GPIO_PIN_SET
#define LASER_ON_STATE         GPIO_PIN_SET
#define LASER_OFF_STATE        GPIO_PIN_RESET
#define LASER_PWM_FREQ_HZ      20000U
#define LASER_POWER_MAX        1000U
#define LASER_POWER_DEFAULT    LASER_POWER_MAX
#define LASER_BINARY_ONLY      1U
#define X_CENTER_FROM_HOME_PULSES 26369U
#define Y_CENTER_FROM_HOME_PULSES 22720U
#define X_TRAVEL_UM            328000
#define Y_TRAVEL_UM            284000
#define Z_TRAVEL_UM            5400
#define X_TRAVEL_STEPS         52738
#define Y_TRAVEL_STEPS         45440
#define Z_TRAVEL_STEPS         40377U
#define Z_JOB_MID_FROM_HOME_PULSES ((Z_TRAVEL_STEPS + 1U) / 2U)
#define XY_DEFAULT_FEED_MM_MIN 1200U
#define XY_RAPID_FEED_MM_MIN   3000U
#define XY_INTERP_MIN_PERIOD_US XY_DEBUG_XY_STEP_PERIOD_US
#define XY_INTERP_MAX_PERIOD_US 10000U
#define UART_LINE_BUF_SIZE     96U
#define MOTION_TIMER_HZ        1000000U
#define ARC_SEGMENT_UM         2000U
#define ARC_MIN_SEGMENTS       12U
#define ARC_MAX_SEGMENTS       240U
#define ARC_PI_F               3.14159265358979323846f
#define W25Q_CMD_WRITE_ENABLE  0x06U
#define W25Q_CMD_READ_STATUS1  0x05U
#define W25Q_CMD_PAGE_PROGRAM  0x02U
#define W25Q_CMD_READ_DATA     0x03U
#define W25Q_CMD_SECTOR_ERASE  0x20U
#define W25Q_CMD_JEDEC_ID      0x9FU
#define W25Q_PAGE_SIZE         256U
#define W25Q_SECTOR_SIZE       4096U
#define W25Q128_TOTAL_BYTES    0x01000000UL
#define W25Q_DEFAULT_POSITION_ADDR (W25Q128_TOTAL_BYTES - W25Q_SECTOR_SIZE)
#define POSITION_STORE_MAGIC   0x50434E43UL
#define POSITION_STORE_VERSION 1U
#define POSITION_STORE_FLAG_HOMED 0x00000001UL
#define POSITION_STORE_SAVE_DELAY_MS 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim4;
static volatile uint8_t led_flash_ready = 0U;
static char uart_line_buf[UART_LINE_BUF_SIZE];
static uint32_t uart_line_len = 0U;
static char uart_motion_line_buf[UART_LINE_BUF_SIZE];
static uint32_t uart_motion_line_len = 0U;
static uint8_t uart_motion_line_ready = 0U;
static int32_t motion_x_steps = 0;
static int32_t motion_y_steps = 0;
static int32_t motion_z_steps = 0;
static int32_t motion_x_um = 0;
static int32_t motion_y_um = 0;
static int32_t motion_z_um = 0;
static uint32_t motion_feed_mm_min = XY_DEFAULT_FEED_MM_MIN;
static uint8_t motion_absolute_mode = 1U;
static MotionRun_t motion_run;
static ArcRun_t arc_run;
static uint8_t motion_debug_print = 1U;
static uint8_t gcode_job_z_mid_done = 0U;
static uint8_t gcode_laser_active = 0U;
static uint8_t laser_pwm_ready = 0U;
static uint32_t gcode_laser_power = LASER_POWER_DEFAULT;
static uint8_t position_store_ready = 0U;
static uint8_t position_store_restored = 0U;
static uint8_t position_store_dirty = 0U;
static uint8_t position_store_suppress_dirty = 0U;
static uint32_t position_store_dirty_since_ms = 0U;
static uint32_t position_store_sequence = 0U;
static uint32_t position_store_address = W25Q_DEFAULT_POSITION_ADDR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Debug_Print(const char *text);
static void Debug_PrintHelp(void);
static void XY_Debug_InitCycleCounter(void);
static void Startup_AutoHomeAndCenter(void);
static uint8_t Homing_Calibration_RunOnce(void);
static uint8_t Homing_Calibration_RunTogether(void);
static uint8_t Homing_RunAxis(const HomeDebug_t *home);
static void Homing_PrepareLimitPins(void);
static uint8_t Homing_ReadLimitMask(void);
static int32_t Homing_FindChangedLimit(uint8_t baseline, uint8_t *current_mask);
static void Homing_PrintLimitMask(uint8_t mask);
static void Homing_PrintDistanceEstimate(uint32_t start_um, uint32_t pulses);
static void Debug_PrintUmAsMm(uint32_t value_um);
static void Debug_PrintU32(uint32_t value);
static void Debug_PrintHex8(uint8_t value);
static void Laser_PWM_Init(void);
static void Laser_SetOutput(uint8_t enabled);
static void Laser_SetPower(uint32_t power);
static uint32_t Laser_ClampPower(int32_t s_value);
static void PositionStore_Init(void);
static void PositionStore_Task(void);
static void PositionStore_MarkDirty(void);
static void PositionStore_FlushIfDirty(void);
static uint8_t PositionStore_Load(void);
static uint8_t PositionStore_SaveNow(void);
static uint8_t PositionStore_RecordIsValid(const PositionStoreRecord_t *record);
static uint32_t PositionStore_Crc32(const uint8_t *data, uint32_t length);
static void W25Q_Select(void);
static void W25Q_Unselect(void);
static uint8_t W25Q_ReadJedecId(uint8_t id[3]);
static uint8_t W25Q_ReadStatus1(uint8_t *status);
static uint8_t W25Q_WriteEnable(void);
static uint8_t W25Q_WaitReady(uint32_t timeout_ms);
static uint8_t W25Q_ReadData(uint32_t address, uint8_t *data, uint32_t length);
static uint8_t W25Q_SectorErase(uint32_t address);
static uint8_t W25Q_PageProgram(uint32_t address, const uint8_t *data, uint32_t length);
static void LED_TimerTick(void);
static void LED_SetFlashOn(uint8_t enabled);
static void XY_Debug_Task(void);
static void UART_ProcessByte(uint8_t data);
static void UART_BufferByteDuringMotion(uint8_t data);
static uint8_t UART_IsImmediateCommand(uint8_t data);
static void Debug_HandleImmediateCommand(uint8_t cmd);
static void ManualJog_ProcessLine(const char *line);
static void GCode_ProcessLine(const char *line);
static uint8_t GCode_EnsureJobZMid(void);
static uint8_t GCode_RunProgramEndHome(void);
static const char *GCode_SkipSpaces(const char *p);
static uint8_t GCode_ParseSignedDecimalUm(const char **p, int32_t *value_um);
static int32_t Motion_DivRoundClosest(int64_t numerator, int32_t denominator);
static void Motion_TimerInit(void);
static void Motion_Task(void);
static void Motion_TimerTick(void);
static void Motion_RequestAbort(void);
static uint8_t GCode_StartArcMove(int32_t target_x_um, int32_t target_y_um, int32_t i_um, int32_t j_um, uint32_t feed_mm_min, uint8_t clockwise);
static uint8_t Arc_StartNextSegment(void);
static int32_t Motion_XUmToSteps(int32_t value_um);
static int32_t Motion_YUmToSteps(int32_t value_um);
static int32_t Motion_ZUmToSteps(int32_t value_um);
static int32_t Motion_XStepsToUm(int32_t steps);
static int32_t Motion_YStepsToUm(int32_t steps);
static int32_t Motion_ZStepsToUm(int32_t steps);
static void Motion_SetPositionSteps(int32_t x_steps, int32_t y_steps, int32_t z_steps);
static uint8_t Motion_LineMoveTo(int32_t target_x_steps, int32_t target_y_steps, int32_t target_z_steps, int32_t target_x_um, int32_t target_y_um, int32_t target_z_um, uint32_t feed_mm_min, uint8_t rapid);
static uint32_t Motion_CalcStepPeriodUs(int32_t dx_um, int32_t dy_um, int32_t dz_um, uint32_t major_steps, uint32_t feed_mm_min);
static uint32_t Motion_Isqrt64(uint64_t value);
static void XY_Debug_PrintStatus(void);
static void XY_Debug_PrepareStepPins(void);
static void XY_Debug_MoveOneRev(const AxisDebug_t *axis, GPIO_PinState dir_state);
static uint8_t XY_Debug_MovePulses(const AxisDebug_t *axis, GPIO_PinState dir_state, uint32_t pulses, const char *label);
static void XY_Debug_MoveToCenter(void);
static uint8_t XY_Debug_AbortRequested(void);
static void XY_Debug_DelayUs(uint32_t delay_us);
static void XY_Debug_Pulse(GPIO_TypeDef *step_port, uint16_t step_pin, uint32_t pulses, uint32_t step_period_us);
static void XY_Debug_DisableAllDrivers(void);
static void XY_Debug_EnableXY(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  led_flash_ready = 1U;
  LED_SetFlashOn(0U);
  XY_Debug_InitCycleCounter();
  MX_USART1_UART_Init();
  XY_Debug_DisableAllDrivers();
  Laser_PWM_Init();
  Laser_SetOutput(0U);
  HAL_GPIO_WritePin(MOTOR775_GPIO_Port, MOTOR775_Pin, GPIO_PIN_RESET);
  Debug_Print("\r\nBOOT CNC laser UART debug\r\n");
  /* Direction test build: skip SD init so the jog test runs without a card. */
  Debug_Print("SD init skipped\r\n");
  /* MX_SDIO_SD_Init(); */
  MX_SPI1_Init();
  PositionStore_Init();
  MX_TIM1_Init();
  XY_Debug_PrepareStepPins();
  Motion_TimerInit();
  Homing_PrepareLimitPins();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Debug_Print("HOME CALIBRATION TEST\r\n");
  Debug_Print("STEP frequency: X/Y/Z 8 kHz\r\n");
  if (position_store_restored != 0U)
  {
    Debug_Print("Startup restore: W25Q position valid, skip auto home. Send H to re-home.\r\n");
    XY_Debug_PrintStatus();
  }
  else
  {
    Debug_Print("Startup auto: no saved W25Q position, home Z/X/Y together, then move X/Y to center. Z stays at top.\r\n");
    Startup_AutoHomeAndCenter();
  }
  Debug_PrintHelp();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Motion_Task();
    XY_Debug_Task();
    PositionStore_Task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static const AxisDebug_t debug_axis_y =
{
  "Y/CH1", STEP1_GPIO_Port, STEP1_Pin, DIR1_GPIO_Port, DIR1_Pin, ENA1_GPIO_Port, ENA1_Pin, XY_DEBUG_XY_STEP_PERIOD_US
};

static const AxisDebug_t debug_axis_x =
{
  "X/CH2", STEP2_GPIO_Port, STEP2_Pin, DIR2_GPIO_Port, DIR2_Pin, ENA2_GPIO_Port, ENA2_Pin, XY_DEBUG_XY_STEP_PERIOD_US
};

static const AxisDebug_t debug_axis_z =
{
  "Z/CH3", STEP3_GPIO_Port, STEP3_Pin, DIR3_GPIO_Port, DIR3_Pin, ENA3_GPIO_Port, ENA3_Pin, XY_DEBUG_Z_STEP_PERIOD_US
};

static const AxisDebug_t debug_axis_ch4 =
{
  "CH4", STEP4_GPIO_Port, STEP4_Pin, DIR4_GPIO_Port, DIR4_Pin, ENA4_GPIO_Port, ENA4_Pin, XY_DEBUG_XY_STEP_PERIOD_US
};

static const LimitDebug_t debug_limits[] =
{
  { "LIMIT1", LIMIT1_GPIO_Port, LIMIT1_Pin, 0x01U },
  { "LIMIT2", LIMIT2_GPIO_Port, LIMIT2_Pin, 0x02U },
  { "LIMIT3", LIMIT3_GPIO_Port, LIMIT3_Pin, 0x04U },
  { "LIMIT4", LIMIT4_GPIO_Port, LIMIT4_Pin, 0x08U },
};

static const HomeDebug_t home_axes[] =
{
  { "Z home/up", &debug_axis_z, GPIO_PIN_SET, 45000U, 5400U, 0x04U },
  { "X home/min", &debug_axis_x, GPIO_PIN_SET, 60000U, 328000U, 0x02U },
  { "Y home/min", &debug_axis_y, GPIO_PIN_RESET, 60000U, 284000U, 0x01U },
};

static void Debug_Print(const char *text)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 1000);
}

static void Debug_PrintHelp(void)
{
  Debug_Print("READY 115200 8N1\r\n");
  Debug_Print("Commands:\r\n");
  Debug_Print("  x: X 400 pulses DIR=0\r\n");
  Debug_Print("  X: X 400 pulses DIR=1\r\n");
  Debug_Print("  y: Y 400 pulses DIR=0\r\n");
  Debug_Print("  Y: Y 400 pulses DIR=1\r\n");
  Debug_Print("  z: Z 400 pulses DIR=0\r\n");
  Debug_Print("  Z: Z 400 pulses DIR=1\r\n");
  Debug_Print("  4: CH4 400 pulses DIR=0\r\n");
  Debug_Print("  a: X then Y then Z, DIR=0\r\n");
  Debug_Print("  h: home Z only with limit stop\r\n");
  Debug_Print("  H: full home Z/X/Y with limit stop\r\n");
  Debug_Print("  C: move X/Y to center together, keep Z unchanged\r\n");
  Debug_Print("  J X.. Y.. F..: manual relative XY jog, keep Z unchanged\r\n");
  Debug_Print("  s: status / limit inputs\r\n");
  Debug_Print("  !: emergency stop while moving\r\n");
  Debug_Print("  R: software reset STM32\r\n");
  Debug_Print("  B: reset into STM32 UART bootloader window\r\n");
  Debug_Print("  e: enable X/Y hold\r\n");
  Debug_Print("  d: disable all drivers\r\n");
  Debug_Print("  G21/G90/G91/G17/G0/G1/G2/G3 X.. Y.. Z.. I.. J.. F.. S..: G-code, laser PWM S0..S1000\r\n");
  Debug_Print("  M3/M4: arm laser and lower Z, M5: laser off, M2/M30: home\r\n");
  Debug_Print("  ?: help\r\n");
}

static void XY_Debug_InitCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Startup_AutoHomeAndCenter(void)
{
  Debug_Print("STARTUP AUTO HOME + CENTER\r\n");
  if (Homing_Calibration_RunOnce() != 0U)
  {
    XY_Debug_MoveToCenter();
  }
  else
  {
    Debug_Print("CENTER SKIPPED, homing failed or aborted\r\n");
    XY_Debug_DisableAllDrivers();
  }
}

static uint8_t Homing_Calibration_RunOnce(void)
{
  uint8_t all_ok;

  Debug_Print("FULL HOME CALIBRATION, XYZ TOGETHER\r\n");
  all_ok = Homing_Calibration_RunTogether();

  Debug_Print(all_ok != 0U ? "HOME CALIBRATION DONE, drivers disabled\r\n" : "HOME CALIBRATION FAILED, drivers disabled\r\n");
  XY_Debug_DisableAllDrivers();
  gcode_job_z_mid_done = 0U;
  gcode_laser_active = 0U;
  Laser_SetOutput(0U);
  if (all_ok != 0U)
  {
    Motion_SetPositionSteps(0, 0, 0);
    (void)PositionStore_SaveNow();
  }
  return all_ok;
}

static uint8_t Homing_Calibration_RunTogether(void)
{
  const uint32_t axis_count = sizeof(home_axes) / sizeof(home_axes[0]);
  uint8_t active[sizeof(home_axes) / sizeof(home_axes[0])];
  uint8_t done[sizeof(home_axes) / sizeof(home_axes[0])];
  uint8_t failed[sizeof(home_axes) / sizeof(home_axes[0])];
  uint32_t pulses[sizeof(home_axes) / sizeof(home_axes[0])];
  uint8_t baseline = Homing_ReadLimitMask();
  uint8_t current_mask = baseline;
  uint8_t any_active = 0U;
  uint8_t all_ok = 1U;
  uint8_t aborted = 0U;
  uint32_t step_period_us = 0U;

  Debug_Print("HOME XYZ TOGETHER baseline ");
  Homing_PrintLimitMask(baseline);

  for (uint32_t i = 0; i < axis_count; i++)
  {
    const HomeDebug_t *home = &home_axes[i];
    active[i] = 0U;
    done[i] = 0U;
    failed[i] = 0U;
    pulses[i] = 0U;

    Debug_Print("HOME ");
    Debug_Print(home->home_name);
    Debug_Print(" axis=");
    Debug_Print(home->axis->name);
    Debug_Print(home->home_dir == GPIO_PIN_RESET ? " DIR=0" : " DIR=1");

    if ((baseline & home->home_limit_mask) == 0U)
    {
      done[i] = 1U;
      Debug_Print(" already-on-limit\r\n");
      continue;
    }

    active[i] = 1U;
    any_active = 1U;
    if (home->axis->step_period_us > step_period_us)
    {
      step_period_us = home->axis->step_period_us;
    }
    HAL_GPIO_WritePin(home->axis->dir_port, home->axis->dir_pin, home->home_dir);
    HAL_GPIO_WritePin(home->axis->ena_port, home->axis->ena_pin, GPIO_PIN_RESET);
    Debug_Print(" active\r\n");
  }

  if (any_active == 0U)
  {
    return 1U;
  }

  if (step_period_us <= XY_DEBUG_STEP_HIGH_US)
  {
    step_period_us = XY_DEBUG_STEP_HIGH_US + 1U;
  }

  HAL_Delay(300);

  while (any_active != 0U)
  {
    any_active = 0U;

    for (uint32_t i = 0; i < axis_count; i++)
    {
      if (active[i] != 0U)
      {
        HAL_GPIO_WritePin(home_axes[i].axis->step_port, home_axes[i].axis->step_pin, GPIO_PIN_SET);
      }
    }

    XY_Debug_DelayUs(XY_DEBUG_STEP_HIGH_US);

    for (uint32_t i = 0; i < axis_count; i++)
    {
      if (active[i] != 0U)
      {
        HAL_GPIO_WritePin(home_axes[i].axis->step_port, home_axes[i].axis->step_pin, GPIO_PIN_RESET);
        pulses[i]++;
      }
    }

    XY_Debug_DelayUs(step_period_us - XY_DEBUG_STEP_HIGH_US);

    if (XY_Debug_AbortRequested() != 0U)
    {
      aborted = 1U;
      break;
    }

    current_mask = Homing_ReadLimitMask();
    for (uint32_t i = 0; i < axis_count; i++)
    {
      const HomeDebug_t *home = &home_axes[i];

      if (active[i] == 0U)
      {
        continue;
      }

      if ((current_mask & home->home_limit_mask) == 0U)
      {
        HAL_Delay(5);
        current_mask = Homing_ReadLimitMask();
        if ((current_mask & home->home_limit_mask) == 0U)
        {
          active[i] = 0U;
          done[i] = 1U;
          HAL_GPIO_WritePin(home->axis->ena_port, home->axis->ena_pin, GPIO_PIN_SET);
          Debug_Print("HIT ");
          Debug_Print(home->axis->name);
          Debug_Print(" pulses=");
          Debug_PrintU32(pulses[i]);
          Debug_Print(" ");
          Homing_PrintLimitMask(current_mask);
          Homing_PrintDistanceEstimate(home->estimated_start_um, pulses[i]);
          continue;
        }
      }

      if (pulses[i] >= home->max_pulses)
      {
        active[i] = 0U;
        failed[i] = 1U;
        HAL_GPIO_WritePin(home->axis->ena_port, home->axis->ena_pin, GPIO_PIN_SET);
        Debug_Print("NO LIMIT HIT ");
        Debug_Print(home->axis->name);
        Debug_Print(" pulses=");
        Debug_PrintU32(pulses[i]);
        Debug_Print("\r\n");
        continue;
      }

      any_active = 1U;
    }
  }

  if (aborted != 0U)
  {
    all_ok = 0U;
    Debug_Print("ABORTED HOME XYZ TOGETHER\r\n");
  }

  for (uint32_t i = 0; i < axis_count; i++)
  {
    HAL_GPIO_WritePin(home_axes[i].axis->ena_port, home_axes[i].axis->ena_pin, GPIO_PIN_SET);
    if ((done[i] == 0U) || (failed[i] != 0U))
    {
      all_ok = 0U;
    }
  }

  return all_ok;
}

static uint8_t Homing_RunAxis(const HomeDebug_t *home)
{
  uint8_t baseline = Homing_ReadLimitMask();
  uint8_t current_mask = baseline;
  int32_t hit_limit = -1;
  uint32_t pulses = 0;
  uint8_t aborted = 0U;

  Debug_Print("HOME ");
  Debug_Print(home->home_name);
  Debug_Print(" axis=");
  Debug_Print(home->axis->name);
  Debug_Print(home->home_dir == GPIO_PIN_RESET ? " DIR=0\r\n" : " DIR=1\r\n");
  Debug_Print("baseline ");
  Homing_PrintLimitMask(baseline);

  if ((baseline & home->home_limit_mask) == 0U)
  {
    XY_Debug_DisableAllDrivers();
    Debug_Print("ALREADY ON HOME LIMIT, skip move ");
    Debug_Print(home->axis->name);
    Debug_Print("\r\n");
    return 1U;
  }

  HAL_GPIO_WritePin(home->axis->dir_port, home->axis->dir_pin, home->home_dir);
  HAL_GPIO_WritePin(home->axis->ena_port, home->axis->ena_pin, GPIO_PIN_RESET);
  HAL_Delay(300);

  while (pulses < home->max_pulses)
  {
    XY_Debug_Pulse(home->axis->step_port, home->axis->step_pin, 1U, home->axis->step_period_us);
    pulses++;

    if (XY_Debug_AbortRequested() != 0U)
    {
      aborted = 1U;
      break;
    }

    hit_limit = Homing_FindChangedLimit(baseline, &current_mask);
    if (hit_limit >= 0)
    {
      break;
    }

  }

  XY_Debug_DisableAllDrivers();

  Debug_Print("STOP ");
  Debug_Print(home->axis->name);
  Debug_Print(" pulses=");
  Debug_PrintU32(pulses);
  Debug_Print(" ");
  Homing_PrintLimitMask(current_mask);

  if (hit_limit >= 0)
  {
    Debug_Print("HIT ");
    Debug_Print(debug_limits[hit_limit].name);
    Debug_Print(" -> ");
    Debug_Print(home->axis->name);
    Debug_Print("\r\n");
    Homing_PrintDistanceEstimate(home->estimated_start_um, pulses);
    return 1U;
  }
  else if (aborted != 0U)
  {
    Debug_Print("ABORTED ");
    Debug_Print(home->axis->name);
    Debug_Print("\r\n");
  }
  else
  {
    Debug_Print("NO LIMIT HIT, max pulses reached\r\n");
    XY_Debug_DisableAllDrivers();
  }

  return 0U;
}

static void Homing_PrepareLimitPins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LIMIT1_Pin|LIMIT2_Pin|LIMIT3_Pin|LIMIT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static uint8_t Homing_ReadLimitMask(void)
{
  uint8_t mask = 0;

  if (HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x01U;
  }
  if (HAL_GPIO_ReadPin(LIMIT2_GPIO_Port, LIMIT2_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x02U;
  }
  if (HAL_GPIO_ReadPin(LIMIT3_GPIO_Port, LIMIT3_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x04U;
  }
  if (HAL_GPIO_ReadPin(LIMIT4_GPIO_Port, LIMIT4_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x08U;
  }

  return mask;
}

static int32_t Homing_FindChangedLimit(uint8_t baseline, uint8_t *current_mask)
{
  uint8_t changed;
  uint8_t confirmed_changed;

  *current_mask = Homing_ReadLimitMask();
  changed = (*current_mask ^ baseline) & 0x0FU;
  if (changed == 0U)
  {
    return -1;
  }

  HAL_Delay(5);
  *current_mask = Homing_ReadLimitMask();
  confirmed_changed = (*current_mask ^ baseline) & changed;
  if (confirmed_changed == 0U)
  {
    return -1;
  }

  for (uint32_t i = 0; i < (sizeof(debug_limits) / sizeof(debug_limits[0])); i++)
  {
    if ((confirmed_changed & debug_limits[i].mask) != 0U)
    {
      return (int32_t)i;
    }
  }

  return -1;
}

static void Homing_PrintLimitMask(uint8_t mask)
{
  Debug_Print("limits L1=");
  Debug_Print((mask & 0x01U) != 0U ? "1 " : "0 ");
  Debug_Print("L2=");
  Debug_Print((mask & 0x02U) != 0U ? "1 " : "0 ");
  Debug_Print("L3=");
  Debug_Print((mask & 0x04U) != 0U ? "1 " : "0 ");
  Debug_Print("L4=");
  Debug_Print((mask & 0x08U) != 0U ? "1\r\n" : "0\r\n");
}

static void Homing_PrintDistanceEstimate(uint32_t start_um, uint32_t pulses)
{
  if ((start_um == 0U) || (pulses == 0U))
  {
    return;
  }

  Debug_Print("estimate from ");
  Debug_PrintUmAsMm(start_um);
  Debug_Print("mm travel: um_per_rev_fullstep200=");
  Debug_PrintU32((start_um * 200U) / pulses);
  Debug_Print(", um_per_rev_1_16_3200=");
  Debug_PrintU32((start_um * 3200U) / pulses);
  Debug_Print("\r\n");
}

static void Debug_PrintUmAsMm(uint32_t value_um)
{
  uint32_t fraction = value_um % 1000U;
  char digits[3];

  Debug_PrintU32(value_um / 1000U);
  Debug_Print(".");

  digits[0] = (char)('0' + ((fraction / 100U) % 10U));
  digits[1] = (char)('0' + ((fraction / 10U) % 10U));
  digits[2] = (char)('0' + (fraction % 10U));
  HAL_UART_Transmit(&huart1, (uint8_t *)digits, sizeof(digits), 1000);
}

static void Debug_PrintU32(uint32_t value)
{
  char digits[10];
  uint32_t count = 0;

  if (value == 0U)
  {
    Debug_Print("0");
    return;
  }

  while (value > 0U)
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (count > 0U)
  {
    count--;
    HAL_UART_Transmit(&huart1, (uint8_t *)&digits[count], 1, 1000);
  }
}

static void Debug_PrintHex8(uint8_t value)
{
  const char hex[] = "0123456789ABCDEF";
  char text[2];

  text[0] = hex[(value >> 4) & 0x0FU];
  text[1] = hex[value & 0x0FU];
  HAL_UART_Transmit(&huart1, (uint8_t *)text, sizeof(text), 1000);
}

static void Laser_PWM_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  uint32_t pclk1_hz;
  uint32_t tim_clock_hz;
  uint32_t prescaler = 0U;
  uint32_t period_ticks;

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, LASER_OFF_STATE);

  pclk1_hz = HAL_RCC_GetPCLK1Freq();
  tim_clock_hz = pclk1_hz;
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1)
  {
    tim_clock_hz = pclk1_hz * 2U;
  }

  period_ticks = tim_clock_hz / LASER_PWM_FREQ_HZ;
  while (period_ticks > 65536U)
  {
    prescaler++;
    period_ticks = tim_clock_hz / ((prescaler + 1U) * LASER_PWM_FREQ_HZ);
  }
  if (period_ticks == 0U)
  {
    period_ticks = 1U;
  }

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = prescaler;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = period_ticks - 1U;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  GPIO_InitStruct.Pin = LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(LASER_GPIO_Port, &GPIO_InitStruct);

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  laser_pwm_ready = 1U;
  Laser_SetPower(0U);
}

static void Laser_SetOutput(uint8_t enabled)
{
  Laser_SetPower(enabled != 0U ? gcode_laser_power : 0U);
}

static void Laser_SetPower(uint32_t power)
{
  uint32_t compare;
  uint32_t pwm_ticks;

  if (power > LASER_POWER_MAX)
  {
    power = LASER_POWER_MAX;
  }

#if (LASER_BINARY_ONLY != 0U)
  if (power > 0U)
  {
    power = LASER_POWER_MAX;
  }
#endif

  if (laser_pwm_ready == 0U)
  {
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, power > 0U ? LASER_ON_STATE : LASER_OFF_STATE);
    return;
  }

  pwm_ticks = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
  compare = (uint32_t)((((uint64_t)pwm_ticks * power) + (LASER_POWER_MAX / 2U)) / LASER_POWER_MAX);
  if (compare > pwm_ticks)
  {
    compare = pwm_ticks;
  }

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare);
}

static uint32_t Laser_ClampPower(int32_t s_value)
{
  if (s_value <= 0)
  {
    return 0U;
  }
  if ((uint32_t)s_value > LASER_POWER_MAX)
  {
    return LASER_POWER_MAX;
  }
  return (uint32_t)s_value;
}

static void PositionStore_Init(void)
{
  uint8_t jedec_id[3] = {0U, 0U, 0U};
  uint32_t capacity_bytes = W25Q128_TOTAL_BYTES;

  W25Q_Unselect();
  HAL_Delay(1);

  if (W25Q_ReadJedecId(jedec_id) == 0U)
  {
    Debug_Print("W25Q JEDEC read failed, position persistence off\r\n");
    position_store_ready = 0U;
    return;
  }

  Debug_Print("W25Q JEDEC ");
  Debug_PrintHex8(jedec_id[0]);
  Debug_Print(" ");
  Debug_PrintHex8(jedec_id[1]);
  Debug_Print(" ");
  Debug_PrintHex8(jedec_id[2]);
  Debug_Print("\r\n");

  if ((jedec_id[0] == 0x00U) || (jedec_id[0] == 0xFFU) ||
      (jedec_id[1] == 0x00U) || (jedec_id[1] == 0xFFU))
  {
    Debug_Print("W25Q ID invalid, position persistence off\r\n");
    position_store_ready = 0U;
    return;
  }

  if ((jedec_id[2] >= 16U) && (jedec_id[2] <= 24U))
  {
    capacity_bytes = 1UL << jedec_id[2];
  }
  position_store_address = capacity_bytes - W25Q_SECTOR_SIZE;
  Debug_Print("W25Q size MiB=");
  Debug_PrintU32(capacity_bytes / (1024UL * 1024UL));
  Debug_Print(" position_sector=0x");
  Debug_PrintHex8((uint8_t)((position_store_address >> 16) & 0xFFU));
  Debug_PrintHex8((uint8_t)((position_store_address >> 8) & 0xFFU));
  Debug_PrintHex8((uint8_t)(position_store_address & 0xFFU));
  Debug_Print("\r\n");

  position_store_ready = 1U;
  if (PositionStore_Load() != 0U)
  {
    Debug_Print("W25Q position restored seq=");
    Debug_PrintU32(position_store_sequence);
    Debug_Print("\r\n");
  }
  else
  {
    Debug_Print("W25Q no valid saved position yet\r\n");
  }
}

static void PositionStore_Task(void)
{
  if ((position_store_ready == 0U) || (position_store_dirty == 0U))
  {
    return;
  }

  if ((motion_run.state != MOTION_RUN_IDLE) || (arc_run.active != 0U))
  {
    return;
  }

  if ((HAL_GetTick() - position_store_dirty_since_ms) < POSITION_STORE_SAVE_DELAY_MS)
  {
    return;
  }

  (void)PositionStore_SaveNow();
}

static void PositionStore_MarkDirty(void)
{
  if ((position_store_ready == 0U) || (position_store_suppress_dirty != 0U))
  {
    return;
  }

  position_store_dirty = 1U;
  position_store_dirty_since_ms = HAL_GetTick();
}

static void PositionStore_FlushIfDirty(void)
{
  if (position_store_dirty != 0U)
  {
    (void)PositionStore_SaveNow();
  }
}

static uint8_t PositionStore_Load(void)
{
  PositionStoreRecord_t record;

  if (W25Q_ReadData(position_store_address, (uint8_t *)&record, sizeof(record)) == 0U)
  {
    Debug_Print("W25Q position read failed\r\n");
    return 0U;
  }

  if (PositionStore_RecordIsValid(&record) == 0U)
  {
    return 0U;
  }

  position_store_suppress_dirty = 1U;
  Motion_SetPositionSteps(record.x_steps, record.y_steps, record.z_steps);
  position_store_suppress_dirty = 0U;
  position_store_sequence = record.sequence;
  position_store_dirty = 0U;
  position_store_restored = 1U;
  gcode_job_z_mid_done = (record.z_steps == Motion_ZUmToSteps(Z_TRAVEL_UM / 2)) ? 1U : 0U;
  gcode_laser_active = 0U;
  Laser_SetOutput(0U);
  return 1U;
}

static uint8_t PositionStore_SaveNow(void)
{
  PositionStoreRecord_t record;
  PositionStoreRecord_t verify;

  if (position_store_ready == 0U)
  {
    return 0U;
  }

  record.magic = POSITION_STORE_MAGIC;
  record.version = POSITION_STORE_VERSION;
  record.record_size = sizeof(record);
  record.sequence = position_store_sequence + 1U;
  record.x_steps = motion_x_steps;
  record.y_steps = motion_y_steps;
  record.z_steps = motion_z_steps;
  record.x_um = motion_x_um;
  record.y_um = motion_y_um;
  record.z_um = motion_z_um;
  record.flags = POSITION_STORE_FLAG_HOMED;
  record.crc32 = PositionStore_Crc32((const uint8_t *)&record, sizeof(record) - sizeof(record.crc32));

  if (W25Q_SectorErase(position_store_address) == 0U)
  {
    Debug_Print("W25Q position save erase failed\r\n");
    return 0U;
  }

  if (W25Q_PageProgram(position_store_address, (const uint8_t *)&record, sizeof(record)) == 0U)
  {
    Debug_Print("W25Q position save program failed\r\n");
    return 0U;
  }

  if (W25Q_ReadData(position_store_address, (uint8_t *)&verify, sizeof(verify)) == 0U)
  {
    Debug_Print("W25Q position verify read failed\r\n");
    return 0U;
  }

  if (PositionStore_RecordIsValid(&verify) == 0U)
  {
    Debug_Print("W25Q position verify failed\r\n");
    return 0U;
  }

  position_store_sequence = record.sequence;
  position_store_dirty = 0U;
  position_store_restored = 1U;
  Debug_Print("W25Q position saved seq=");
  Debug_PrintU32(position_store_sequence);
  Debug_Print("\r\n");
  return 1U;
}

static uint8_t PositionStore_RecordIsValid(const PositionStoreRecord_t *record)
{
  uint32_t crc;

  if ((record->magic != POSITION_STORE_MAGIC) ||
      (record->version != POSITION_STORE_VERSION) ||
      (record->record_size != sizeof(PositionStoreRecord_t)))
  {
    return 0U;
  }

  crc = PositionStore_Crc32((const uint8_t *)record, sizeof(PositionStoreRecord_t) - sizeof(record->crc32));
  if (crc != record->crc32)
  {
    return 0U;
  }

  if ((record->x_steps < 0) || (record->x_steps > (int32_t)X_TRAVEL_STEPS) ||
      (record->y_steps < 0) || (record->y_steps > (int32_t)Y_TRAVEL_STEPS) ||
      (record->z_steps < 0) || (record->z_steps > (int32_t)Z_TRAVEL_STEPS))
  {
    return 0U;
  }

  return 1U;
}

static uint32_t PositionStore_Crc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;

  for (uint32_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint32_t bit = 0; bit < 8U; bit++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

static void W25Q_Select(void)
{
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static void W25Q_Unselect(void)
{
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

static uint8_t W25Q_ReadJedecId(uint8_t id[3])
{
  uint8_t cmd = W25Q_CMD_JEDEC_ID;
  HAL_StatusTypeDef status;

  W25Q_Select();
  status = HAL_SPI_Transmit(&hspi1, &cmd, 1U, 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, id, 3U, 1000U);
  }
  W25Q_Unselect();

  return status == HAL_OK ? 1U : 0U;
}

static uint8_t W25Q_ReadStatus1(uint8_t *status_reg)
{
  uint8_t cmd = W25Q_CMD_READ_STATUS1;
  HAL_StatusTypeDef status;

  W25Q_Select();
  status = HAL_SPI_Transmit(&hspi1, &cmd, 1U, 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, status_reg, 1U, 1000U);
  }
  W25Q_Unselect();

  return status == HAL_OK ? 1U : 0U;
}

static uint8_t W25Q_WriteEnable(void)
{
  uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
  HAL_StatusTypeDef status;

  W25Q_Select();
  status = HAL_SPI_Transmit(&hspi1, &cmd, 1U, 1000U);
  W25Q_Unselect();

  return status == HAL_OK ? 1U : 0U;
}

static uint8_t W25Q_WaitReady(uint32_t timeout_ms)
{
  uint32_t started_ms = HAL_GetTick();
  uint8_t status_reg = 0U;

  do
  {
    if (W25Q_ReadStatus1(&status_reg) == 0U)
    {
      return 0U;
    }
    if ((status_reg & 0x01U) == 0U)
    {
      return 1U;
    }
    HAL_Delay(1);
  } while ((HAL_GetTick() - started_ms) < timeout_ms);

  return 0U;
}

static uint8_t W25Q_ReadData(uint32_t address, uint8_t *data, uint32_t length)
{
  uint8_t cmd[4];
  HAL_StatusTypeDef status;

  cmd[0] = W25Q_CMD_READ_DATA;
  cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(address & 0xFFU);

  W25Q_Select();
  status = HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, data, (uint16_t)length, 1000U);
  }
  W25Q_Unselect();

  return status == HAL_OK ? 1U : 0U;
}

static uint8_t W25Q_SectorErase(uint32_t address)
{
  uint8_t cmd[4];

  if (W25Q_WaitReady(1000U) == 0U)
  {
    return 0U;
  }
  if (W25Q_WriteEnable() == 0U)
  {
    return 0U;
  }

  cmd[0] = W25Q_CMD_SECTOR_ERASE;
  cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(address & 0xFFU);

  W25Q_Select();
  if (HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 1000U) != HAL_OK)
  {
    W25Q_Unselect();
    return 0U;
  }
  W25Q_Unselect();

  return W25Q_WaitReady(5000U);
}

static uint8_t W25Q_PageProgram(uint32_t address, const uint8_t *data, uint32_t length)
{
  uint8_t cmd[4];
  HAL_StatusTypeDef status;

  if ((length == 0U) || (length > W25Q_PAGE_SIZE))
  {
    return 0U;
  }

  if (W25Q_WaitReady(1000U) == 0U)
  {
    return 0U;
  }
  if (W25Q_WriteEnable() == 0U)
  {
    return 0U;
  }

  cmd[0] = W25Q_CMD_PAGE_PROGRAM;
  cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(address & 0xFFU);

  W25Q_Select();
  status = HAL_SPI_Transmit(&hspi1, cmd, sizeof(cmd), 1000U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, (uint16_t)length, 1000U);
  }
  W25Q_Unselect();

  if (status != HAL_OK)
  {
    return 0U;
  }

  return W25Q_WaitReady(1000U);
}

static void XY_Debug_Task(void)
{
  uint8_t cmd = 0;

  if ((uart_motion_line_ready != 0U) && (motion_run.state != MOTION_RUN_ACTIVE))
  {
    uart_motion_line_ready = 0U;
    if ((uart_motion_line_buf[0] != '\0') &&
        (uart_motion_line_buf[1] == '\0') &&
        (UART_IsImmediateCommand((uint8_t)uart_motion_line_buf[0]) != 0U))
    {
      Debug_HandleImmediateCommand((uint8_t)uart_motion_line_buf[0]);
    }
    else
    {
      GCode_ProcessLine(uart_motion_line_buf);
    }
    return;
  }

  while (HAL_UART_Receive(&huart1, &cmd, 1, 0) == HAL_OK)
  {
    UART_ProcessByte(cmd);
  }
}

static void UART_ProcessByte(uint8_t data)
{
  if ((uart_line_len == 0U) && (UART_IsImmediateCommand(data) != 0U))
  {
    Debug_HandleImmediateCommand(data);
    return;
  }

  if (motion_run.state == MOTION_RUN_ACTIVE)
  {
    UART_BufferByteDuringMotion(data);
    return;
  }

  if ((data == '\r') || (data == '\n'))
  {
    if (uart_line_len > 0U)
    {
      uart_line_buf[uart_line_len] = '\0';
      GCode_ProcessLine(uart_line_buf);
      uart_line_len = 0U;
    }
    return;
  }

  if ((data < 0x20U) || (data > 0x7EU))
  {
    return;
  }

  if (uart_line_len >= (UART_LINE_BUF_SIZE - 1U))
  {
    uart_line_len = 0U;
    Debug_Print("error: line too long\r\n");
    return;
  }

  uart_line_buf[uart_line_len++] = (char)data;
}

static void UART_BufferByteDuringMotion(uint8_t data)
{
  if ((data == '\r') || (data == '\n'))
  {
    if (uart_motion_line_len > 0U)
    {
      if (uart_motion_line_ready == 0U)
      {
        uart_motion_line_buf[uart_motion_line_len] = '\0';
        uart_motion_line_ready = 1U;
      }
      uart_motion_line_len = 0U;
    }
    return;
  }

  if ((data < 0x20U) || (data > 0x7EU))
  {
    return;
  }

  if ((uart_motion_line_ready != 0U) || (uart_motion_line_len >= (UART_LINE_BUF_SIZE - 1U)))
  {
    uart_motion_line_len = 0U;
    return;
  }

  uart_motion_line_buf[uart_motion_line_len++] = (char)data;
}

static uint8_t UART_IsImmediateCommand(uint8_t data)
{
  switch (data)
  {
    case 'x':
    case 'X':
    case 'y':
    case 'Y':
    case 'z':
    case 'Z':
    case '4':
    case 'a':
    case 'h':
    case 'H':
    case 'C':
    case 's':
    case '!':
    case 'R':
    case 'B':
    case 'e':
    case 'd':
    case '?':
      return 1U;
    default:
      return 0U;
  }
}

static void Debug_HandleImmediateCommand(uint8_t cmd)
{
  if (motion_run.state == MOTION_RUN_ACTIVE)
  {
    if ((cmd == '!') || (cmd == 'd'))
    {
      Motion_RequestAbort();
      XY_Debug_DisableAllDrivers();
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      Debug_Print("emergency stop, drivers disabled\r\n");
      return;
    }
    if (cmd == 's')
    {
      XY_Debug_PrintStatus();
      return;
    }

    Debug_Print("error: motion busy\r\n");
    return;
  }

  switch (cmd)
  {
    case 'x':
      XY_Debug_MoveOneRev(&debug_axis_x, GPIO_PIN_RESET);
      Debug_Print("ok\r\n");
      break;
    case 'X':
      XY_Debug_MoveOneRev(&debug_axis_x, GPIO_PIN_SET);
      Debug_Print("ok\r\n");
      break;
    case 'y':
      XY_Debug_MoveOneRev(&debug_axis_y, GPIO_PIN_RESET);
      Debug_Print("ok\r\n");
      break;
    case 'Y':
      XY_Debug_MoveOneRev(&debug_axis_y, GPIO_PIN_SET);
      Debug_Print("ok\r\n");
      break;
    case 'z':
      XY_Debug_MoveOneRev(&debug_axis_z, GPIO_PIN_RESET);
      Debug_Print("ok\r\n");
      break;
    case 'Z':
      XY_Debug_MoveOneRev(&debug_axis_z, GPIO_PIN_SET);
      Debug_Print("ok\r\n");
      break;
    case '4':
      XY_Debug_MoveOneRev(&debug_axis_ch4, GPIO_PIN_RESET);
      Debug_Print("ok\r\n");
      break;
    case 'a':
      XY_Debug_MoveOneRev(&debug_axis_x, GPIO_PIN_RESET);
      HAL_Delay(1000);
      XY_Debug_MoveOneRev(&debug_axis_y, GPIO_PIN_RESET);
      HAL_Delay(1000);
      XY_Debug_MoveOneRev(&debug_axis_z, GPIO_PIN_RESET);
      Debug_Print("ok\r\n");
      break;
    case 'h':
      Debug_Print("Z HOME RETEST\r\n");
      if (Homing_RunAxis(&home_axes[0]) != 0U)
      {
        Motion_SetPositionSteps(motion_x_steps, motion_y_steps, 0);
        (void)PositionStore_SaveNow();
      }
      gcode_job_z_mid_done = 0U;
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      Debug_Print("Z HOME RETEST DONE, drivers disabled\r\n");
      Debug_Print("ok\r\n");
      break;
    case 'H':
      Homing_Calibration_RunOnce();
      Debug_Print("ok\r\n");
      break;
    case 'C':
      XY_Debug_MoveToCenter();
      break;
    case 's':
      XY_Debug_PrintStatus();
      break;
    case '!':
      Motion_RequestAbort();
      XY_Debug_DisableAllDrivers();
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      Debug_Print("emergency stop, drivers disabled\r\n");
      break;
    case 'R':
      XY_Debug_DisableAllDrivers();
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      PositionStore_FlushIfDirty();
      Debug_Print("software reset STM32\r\n");
      HAL_Delay(100);
      NVIC_SystemReset();
      break;
    case 'B':
      Motion_RequestAbort();
      XY_Debug_DisableAllDrivers();
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      HAL_GPIO_WritePin(MOTOR775_GPIO_Port, MOTOR775_Pin, GPIO_PIN_RESET);
      PositionStore_FlushIfDirty();
      Debug_Print("reset to bootloader window\r\n");
      HAL_Delay(100);
      NVIC_SystemReset();
      break;
    case 'e':
      XY_Debug_EnableXY();
      Debug_Print("XY enabled\r\n");
      Debug_Print("ok\r\n");
      break;
    case 'd':
      Motion_RequestAbort();
      XY_Debug_DisableAllDrivers();
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      Debug_Print("drivers disabled\r\n");
      Debug_Print("ok\r\n");
      break;
    case '?':
      Debug_PrintHelp();
      break;
    case '\r':
    case '\n':
      break;
    default:
      Debug_Print("unknown command, send ?\r\n");
      break;
  }
}

static void ManualJog_ProcessLine(const char *line)
{
  const char *p = line;
  int32_t x_um = 0;
  int32_t y_um = 0;
  int32_t f_um_min = 0;
  uint8_t has_x = 0U;
  uint8_t has_y = 0U;
  uint8_t has_f = 0U;
  uint32_t jog_feed = XY_RAPID_FEED_MM_MIN;
  int32_t target_x_um;
  int32_t target_y_um;
  int32_t target_x_steps;
  int32_t target_y_steps;

  while (*p != '\0')
  {
    char letter;
    int32_t value_um = 0;

    p = GCode_SkipSpaces(p);
    if ((*p == '\0') || (*p == ';') || (*p == '*'))
    {
      break;
    }
    if (*p == '(')
    {
      while ((*p != '\0') && (*p != ')'))
      {
        p++;
      }
      if (*p == ')')
      {
        p++;
      }
      continue;
    }

    letter = *p++;
    if ((letter >= 'a') && (letter <= 'z'))
    {
      letter = (char)(letter - 'a' + 'A');
    }

    if (GCode_ParseSignedDecimalUm(&p, &value_um) == 0U)
    {
      Debug_Print("error: bad jog number\r\n");
      return;
    }

    switch (letter)
    {
      case 'X':
        x_um = value_um;
        has_x = 1U;
        break;
      case 'Y':
        y_um = value_um;
        has_y = 1U;
        break;
      case 'F':
        f_um_min = value_um;
        has_f = 1U;
        break;
      default:
        Debug_Print("error: unsupported jog word\r\n");
        return;
    }
  }

  if ((has_x == 0U) && (has_y == 0U))
  {
    Debug_Print("error: jog needs X or Y\r\n");
    return;
  }

  if (has_f != 0U)
  {
    int32_t parsed_feed = Motion_DivRoundClosest(f_um_min, 1000);
    if (parsed_feed <= 0)
    {
      parsed_feed = 1;
    }
    jog_feed = (uint32_t)parsed_feed;
  }

  target_x_um = motion_x_um + x_um;
  target_y_um = motion_y_um + y_um;
  target_x_steps = motion_x_steps + Motion_XUmToSteps(x_um);
  target_y_steps = motion_y_steps + Motion_YUmToSteps(y_um);

  (void)Motion_LineMoveTo(target_x_steps, target_y_steps, motion_z_steps, target_x_um, target_y_um, motion_z_um, jog_feed, 1U);
}

static void GCode_ProcessLine(const char *line)
{
  const char *p = line;
  int32_t move_code = -1;
  int32_t m_code = -1;
  int32_t x_um = 0;
  int32_t y_um = 0;
  int32_t z_um = 0;
  int32_t i_um = 0;
  int32_t j_um = 0;
  int32_t f_um_min = 0;
  int32_t s_value = 0;
  uint8_t has_x = 0U;
  uint8_t has_y = 0U;
  uint8_t has_z = 0U;
  uint8_t has_i = 0U;
  uint8_t has_j = 0U;
  uint8_t has_f = 0U;
  uint8_t has_s = 0U;
  uint8_t changed_modal = 0U;

  p = GCode_SkipSpaces(p);
  if ((*p == 'J') || (*p == 'j'))
  {
    p++;
    ManualJog_ProcessLine(p);
    return;
  }

  while (*p != '\0')
  {
    char letter;
    int32_t value_um = 0;

    p = GCode_SkipSpaces(p);
    if ((*p == '\0') || (*p == ';') || (*p == '*'))
    {
      break;
    }
    if (*p == '(')
    {
      while ((*p != '\0') && (*p != ')'))
      {
        p++;
      }
      if (*p == ')')
      {
        p++;
      }
      continue;
    }

    letter = *p++;
    if ((letter >= 'a') && (letter <= 'z'))
    {
      letter = (char)(letter - 'a' + 'A');
    }

    if (GCode_ParseSignedDecimalUm(&p, &value_um) == 0U)
    {
      Debug_Print("error: bad number\r\n");
      return;
    }

    switch (letter)
    {
      case 'G':
      {
        int32_t g_code = value_um / 1000;

        if ((g_code == 0) || (g_code == 1) || (g_code == 2) || (g_code == 3))
        {
          move_code = g_code;
          changed_modal = 1U;
        }
        else if (g_code == 17)
        {
          changed_modal = 1U;
        }
        else if (g_code == 21)
        {
          changed_modal = 1U;
        }
        else if (g_code == 90)
        {
          motion_absolute_mode = 1U;
          changed_modal = 1U;
        }
        else if (g_code == 91)
        {
          motion_absolute_mode = 0U;
          changed_modal = 1U;
        }
        else
        {
          Debug_Print("error: unsupported G code\r\n");
          return;
        }
        break;
      }
      case 'M':
        m_code = value_um / 1000;
        break;
      case 'X':
        x_um = value_um;
        has_x = 1U;
        break;
      case 'Y':
        y_um = value_um;
        has_y = 1U;
        break;
      case 'Z':
        z_um = value_um;
        has_z = 1U;
        break;
      case 'I':
        i_um = value_um;
        has_i = 1U;
        break;
      case 'J':
        j_um = value_um;
        has_j = 1U;
        break;
      case 'F':
        f_um_min = value_um;
        has_f = 1U;
        break;
      case 'S':
        s_value = value_um / 1000;
        has_s = 1U;
        break;
      case 'N':
        break;
      default:
        Debug_Print("error: unsupported word\r\n");
        return;
    }
  }

  if (has_f != 0U)
  {
    int32_t parsed_feed = Motion_DivRoundClosest(f_um_min, 1000);
    if (parsed_feed <= 0)
    {
      parsed_feed = 1U;
    }
    motion_feed_mm_min = (uint32_t)parsed_feed;
  }

  if (has_s != 0U)
  {
    gcode_laser_power = Laser_ClampPower(s_value);
    if (gcode_laser_power == 0U)
    {
      Laser_SetOutput(0U);
    }
  }

  if (m_code >= 0)
  {
    if ((m_code == 3) || (m_code == 4))
    {
      if (GCode_EnsureJobZMid() == 0U)
      {
        Debug_Print("error: Z mid abort\r\n");
        return;
      }
      gcode_laser_active = 1U;
      Laser_SetOutput(gcode_laser_power > 0U ? 1U : 0U);
      Debug_Print("ok M3/M4 armed, Z mid, S=");
      Debug_PrintU32(gcode_laser_power);
      Debug_Print("/");
      Debug_PrintU32(LASER_POWER_MAX);
      Debug_Print(gcode_laser_power > 0U ? ", laser pwm on\r\n" : ", laser off\r\n");
      return;
    }
    if (m_code == 5)
    {
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      Debug_Print("ok M5 laser off\r\n");
      return;
    }
    if ((m_code == 2) || (m_code == 30))
    {
      gcode_laser_active = 0U;
      Laser_SetOutput(0U);
      (void)GCode_RunProgramEndHome();
      return;
    }

    Debug_Print("error: unsupported M code\r\n");
    return;
  }

  if ((move_code < 0) && ((has_x != 0U) || (has_y != 0U) || (has_z != 0U)))
  {
    move_code = 1;
  }

  if (move_code >= 0)
  {
    int32_t target_x_um = motion_x_um;
    int32_t target_y_um = motion_y_um;
    int32_t target_x_steps = motion_x_steps;
    int32_t target_y_steps = motion_y_steps;
    uint32_t move_feed = (move_code == 0) ? XY_RAPID_FEED_MM_MIN : motion_feed_mm_min;

    if ((has_z != 0U) && (has_x == 0U) && (has_y == 0U) && (move_code <= 1))
    {
      (void)z_um;
      if (gcode_laser_active != 0U)
      {
        if (GCode_EnsureJobZMid() == 0U)
        {
          Debug_Print("error: Z mid abort\r\n");
          return;
        }
        Debug_Print("ok Z mid\r\n");
      }
      else
      {
        Debug_Print("ok Z ignored until M3/M4\r\n");
      }
      return;
    }

    if (has_s != 0U)
    {
      if (gcode_laser_power == 0U)
      {
        Laser_SetOutput(0U);
      }
    }

    if (has_x != 0U)
    {
      if (motion_absolute_mode != 0U)
      {
        target_x_um = x_um;
        target_x_steps = Motion_XUmToSteps(x_um);
      }
      else
      {
        target_x_um = motion_x_um + x_um;
        target_x_steps = motion_x_steps + Motion_XUmToSteps(x_um);
      }
    }

    if (has_y != 0U)
    {
      if (motion_absolute_mode != 0U)
      {
        target_y_um = y_um;
        target_y_steps = Motion_YUmToSteps(y_um);
      }
      else
      {
        target_y_um = motion_y_um + y_um;
        target_y_steps = motion_y_steps + Motion_YUmToSteps(y_um);
      }
    }

    if ((move_code == 2) || (move_code == 3))
    {
      if ((has_i == 0U) || (has_j == 0U))
      {
        Debug_Print("error: arc needs I/J\r\n");
        return;
      }
      if ((gcode_laser_active != 0U) && (GCode_EnsureJobZMid() == 0U))
      {
        Debug_Print("error: Z mid abort\r\n");
        return;
      }
      (void)GCode_StartArcMove(target_x_um, target_y_um, i_um, j_um, move_feed, (uint8_t)(move_code == 2));
      return;
    }

    if (((has_x != 0U) || (has_y != 0U)) &&
        (gcode_laser_active != 0U) &&
        (GCode_EnsureJobZMid() == 0U))
    {
      Debug_Print("error: Z mid abort\r\n");
      return;
    }

    (void)Motion_LineMoveTo(target_x_steps, target_y_steps, motion_z_steps, target_x_um, target_y_um, motion_z_um, move_feed, (uint8_t)(move_code == 0));
    return;
  }

  if ((changed_modal != 0U) || (has_f != 0U) || (has_s != 0U) || (has_z != 0U))
  {
    Debug_Print("ok\r\n");
    return;
  }

  Debug_Print("ok\r\n");
}

static uint8_t GCode_EnsureJobZMid(void)
{
  int32_t target_z_steps = Motion_ZUmToSteps(Z_TRAVEL_UM / 2);
  int32_t target_z_um = Motion_ZStepsToUm(target_z_steps);
  uint8_t saved_debug_print = motion_debug_print;

  if (gcode_job_z_mid_done != 0U)
  {
    return 1U;
  }

  Debug_Print("JOB Z MID: move Z down pulses=");
  Debug_PrintU32(Z_JOB_MID_FROM_HOME_PULSES);
  Debug_Print("\r\n");

  motion_debug_print = 0U;
  if (Motion_LineMoveTo(motion_x_steps, motion_y_steps, target_z_steps, motion_x_um, motion_y_um, target_z_um, XY_RAPID_FEED_MM_MIN, 1U) == 0U)
  {
    motion_debug_print = saved_debug_print;
    XY_Debug_DisableAllDrivers();
    return 0U;
  }

  while (motion_run.state == MOTION_RUN_ACTIVE)
  {
    if (XY_Debug_AbortRequested() != 0U)
    {
      Motion_RequestAbort();
    }
    Motion_Task();
  }

  Motion_Task();
  motion_debug_print = saved_debug_print;
  if (motion_z_steps != target_z_steps)
  {
    XY_Debug_DisableAllDrivers();
    return 0U;
  }

  XY_Debug_DisableAllDrivers();
  gcode_job_z_mid_done = 1U;
  return 1U;
}

static uint8_t GCode_RunProgramEndHome(void)
{
  Debug_Print("PROGRAM END: HOME Z/X/Y\r\n");
  if (Homing_Calibration_RunOnce() == 0U)
  {
    Debug_Print("error: program end home failed\r\n");
    return 0U;
  }

  Debug_Print("ok program end home\r\n");
  return 1U;
}

static const char *GCode_SkipSpaces(const char *p)
{
  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }
  return p;
}

static uint8_t GCode_ParseSignedDecimalUm(const char **p, int32_t *value_um)
{
  int32_t sign = 1;
  int64_t integer_part = 0;
  int32_t fraction = 0;
  uint32_t fraction_digits = 0U;
  uint8_t has_digit = 0U;

  *p = GCode_SkipSpaces(*p);
  if (**p == '-')
  {
    sign = -1;
    (*p)++;
  }
  else if (**p == '+')
  {
    (*p)++;
  }

  while ((**p >= '0') && (**p <= '9'))
  {
    has_digit = 1U;
    integer_part = (integer_part * 10) + (**p - '0');
    (*p)++;
  }

  if (**p == '.')
  {
    (*p)++;
    while ((**p >= '0') && (**p <= '9'))
    {
      has_digit = 1U;
      if (fraction_digits < 3U)
      {
        fraction = (fraction * 10) + (**p - '0');
        fraction_digits++;
      }
      (*p)++;
    }
  }

  if (has_digit == 0U)
  {
    return 0U;
  }

  while (fraction_digits < 3U)
  {
    fraction *= 10;
    fraction_digits++;
  }

  *value_um = (int32_t)(((integer_part * 1000) + fraction) * sign);
  return 1U;
}

static int32_t Motion_DivRoundClosest(int64_t numerator, int32_t denominator)
{
  if (numerator >= 0)
  {
    return (int32_t)((numerator + (denominator / 2)) / denominator);
  }

  return (int32_t)(-(((-numerator) + (denominator / 2)) / denominator));
}

static void Motion_TimerInit(void)
{
  uint32_t tim_clock_hz;
  uint32_t pclk1_hz;
  uint32_t prescaler;

  __HAL_RCC_TIM5_CLK_ENABLE();

  pclk1_hz = HAL_RCC_GetPCLK1Freq();
  tim_clock_hz = pclk1_hz;
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1)
  {
    tim_clock_hz = pclk1_hz * 2U;
  }

  prescaler = (tim_clock_hz / MOTION_TIMER_HZ) - 1U;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = prescaler;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = XY_INTERP_MIN_PERIOD_US - 1U;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

static void Motion_Task(void)
{
  MotionRunState_t state = motion_run.state;

  if ((state != MOTION_RUN_DONE) && (state != MOTION_RUN_ABORTED))
  {
    return;
  }

  HAL_TIM_Base_Stop_IT(&htim5);
  Laser_SetOutput(0U);
  HAL_GPIO_WritePin(debug_axis_x.step_port, debug_axis_x.step_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(debug_axis_y.step_port, debug_axis_y.step_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(debug_axis_z.step_port, debug_axis_z.step_pin, GPIO_PIN_RESET);

  if (state == MOTION_RUN_DONE)
  {
    motion_x_steps = motion_run.target_x_steps;
    motion_y_steps = motion_run.target_y_steps;
    motion_z_steps = motion_run.target_z_steps;
    motion_x_um = motion_run.target_x_um;
    motion_y_um = motion_run.target_y_um;
    motion_z_um = motion_run.target_z_um;
    motion_run.state = MOTION_RUN_IDLE;
    PositionStore_MarkDirty();

    if (arc_run.active != 0U)
    {
      if (Arc_StartNextSegment() != 0U)
      {
        return;
      }

      motion_debug_print = 1U;
      XY_Debug_DisableAllDrivers();
      if (arc_run.failed != 0U)
      {
        arc_run.failed = 0U;
        Debug_Print("error: arc segment\r\n");
        return;
      }

      Debug_Print("arc done, drivers disabled\r\n");
      Debug_Print("ok\r\n");
      return;
    }

    XY_Debug_DisableAllDrivers();
    if (motion_debug_print != 0U)
    {
      Debug_Print("line done, drivers disabled\r\n");
      Debug_Print("ok\r\n");
    }
  }
  else
  {
    arc_run.active = 0U;
    arc_run.failed = 0U;
    motion_debug_print = 1U;
    XY_Debug_DisableAllDrivers();
    motion_run.state = MOTION_RUN_IDLE;
    Debug_Print(motion_run.limit_abort != 0U ? "error: limit abort\r\n" : "error: motion aborted\r\n");
  }
}

static void Motion_TimerTick(void)
{
  uint8_t pulse_x = 0U;
  uint8_t pulse_y = 0U;
  uint8_t pulse_z = 0U;
  uint8_t limit_mask;

  if (motion_run.state != MOTION_RUN_ACTIVE)
  {
    return;
  }

  if (motion_run.step_index >= motion_run.major_steps)
  {
    motion_run.state = MOTION_RUN_DONE;
    return;
  }

  if ((motion_run.step_index & 0x1FU) == 0U)
  {
    limit_mask = Homing_ReadLimitMask();
    if (((motion_run.dx_steps < 0) && ((limit_mask & 0x02U) == 0U)) ||
        ((motion_run.dy_steps < 0) && ((limit_mask & 0x01U) == 0U)) ||
        ((motion_run.dz_steps < 0) && ((limit_mask & 0x04U) == 0U)))
    {
      motion_run.limit_abort = 1U;
      motion_run.state = MOTION_RUN_ABORTED;
      Laser_SetOutput(0U);
      return;
    }
  }

  motion_run.x_acc += motion_run.x_steps;
  if (motion_run.x_acc >= motion_run.major_steps)
  {
    motion_run.x_acc -= motion_run.major_steps;
    pulse_x = 1U;
  }

  motion_run.y_acc += motion_run.y_steps;
  if (motion_run.y_acc >= motion_run.major_steps)
  {
    motion_run.y_acc -= motion_run.major_steps;
    pulse_y = 1U;
  }

  motion_run.z_acc += motion_run.z_steps;
  if (motion_run.z_acc >= motion_run.major_steps)
  {
    motion_run.z_acc -= motion_run.major_steps;
    pulse_z = 1U;
  }

  if (pulse_x != 0U)
  {
    HAL_GPIO_WritePin(debug_axis_x.step_port, debug_axis_x.step_pin, GPIO_PIN_SET);
  }
  if (pulse_y != 0U)
  {
    HAL_GPIO_WritePin(debug_axis_y.step_port, debug_axis_y.step_pin, GPIO_PIN_SET);
  }
  if (pulse_z != 0U)
  {
    HAL_GPIO_WritePin(debug_axis_z.step_port, debug_axis_z.step_pin, GPIO_PIN_SET);
  }

  XY_Debug_DelayUs(XY_DEBUG_STEP_HIGH_US);

  if (pulse_x != 0U)
  {
    HAL_GPIO_WritePin(debug_axis_x.step_port, debug_axis_x.step_pin, GPIO_PIN_RESET);
  }
  if (pulse_y != 0U)
  {
    HAL_GPIO_WritePin(debug_axis_y.step_port, debug_axis_y.step_pin, GPIO_PIN_RESET);
  }
  if (pulse_z != 0U)
  {
    HAL_GPIO_WritePin(debug_axis_z.step_port, debug_axis_z.step_pin, GPIO_PIN_RESET);
  }

  motion_run.step_index++;
  if (motion_run.step_index >= motion_run.major_steps)
  {
    motion_run.state = MOTION_RUN_DONE;
  }
}

static void Motion_RequestAbort(void)
{
  if (motion_run.state == MOTION_RUN_ACTIVE)
  {
    Laser_SetOutput(0U);
    motion_run.limit_abort = 0U;
    motion_run.state = MOTION_RUN_ABORTED;
  }
}

static uint8_t GCode_StartArcMove(int32_t target_x_um, int32_t target_y_um, int32_t i_um, int32_t j_um, uint32_t feed_mm_min, uint8_t clockwise)
{
  float center_x = (float)(motion_x_um + i_um);
  float center_y = (float)(motion_y_um + j_um);
  float start_dx = (float)motion_x_um - center_x;
  float start_dy = (float)motion_y_um - center_y;
  float end_dx = (float)target_x_um - center_x;
  float end_dy = (float)target_y_um - center_y;
  float radius_um = sqrtf((start_dx * start_dx) + (start_dy * start_dy));
  float end_radius_um = sqrtf((end_dx * end_dx) + (end_dy * end_dy));
  float start_angle;
  float end_angle;
  float sweep_angle;
  float arc_length_um;
  uint32_t segments;

  if (motion_run.state == MOTION_RUN_ACTIVE)
  {
    Debug_Print("error: motion busy\r\n");
    return 0U;
  }

  if ((target_x_um < 0) || (target_x_um > X_TRAVEL_UM) ||
      (target_y_um < 0) || (target_y_um > Y_TRAVEL_UM))
  {
    Debug_Print("error: soft limit\r\n");
    return 0U;
  }

  if ((radius_um < 1.0f) || (end_radius_um < 1.0f))
  {
    Debug_Print("error: bad arc radius\r\n");
    return 0U;
  }

  if (fabsf(radius_um - end_radius_um) > 2000.0f)
  {
    Debug_Print("error: arc radius mismatch\r\n");
    return 0U;
  }

  start_angle = atan2f(start_dy, start_dx);
  end_angle = atan2f(end_dy, end_dx);
  sweep_angle = end_angle - start_angle;

  if (clockwise != 0U)
  {
    if (sweep_angle >= 0.0f)
    {
      sweep_angle -= (2.0f * ARC_PI_F);
    }
  }
  else
  {
    if (sweep_angle <= 0.0f)
    {
      sweep_angle += (2.0f * ARC_PI_F);
    }
  }

  arc_length_um = fabsf(sweep_angle) * radius_um;
  segments = (uint32_t)ceilf(arc_length_um / (float)ARC_SEGMENT_UM);
  if (segments < ARC_MIN_SEGMENTS)
  {
    segments = ARC_MIN_SEGMENTS;
  }
  if (segments > ARC_MAX_SEGMENTS)
  {
    segments = ARC_MAX_SEGMENTS;
  }

  arc_run.active = 1U;
  arc_run.failed = 0U;
  arc_run.segment_index = 0U;
  arc_run.total_segments = segments;
  arc_run.center_x_um = motion_x_um + i_um;
  arc_run.center_y_um = motion_y_um + j_um;
  arc_run.target_x_um = target_x_um;
  arc_run.target_y_um = target_y_um;
  arc_run.radius_um = radius_um;
  arc_run.start_angle = start_angle;
  arc_run.sweep_angle = sweep_angle;
  arc_run.feed_mm_min = feed_mm_min;

  Debug_Print(clockwise != 0U ? "G2 arc segments=" : "G3 arc segments=");
  Debug_PrintU32(segments);
  Debug_Print("\r\n");

  if (Arc_StartNextSegment() != 0U)
  {
    return 1U;
  }

  motion_debug_print = 1U;
  arc_run.active = 0U;
  if (arc_run.failed != 0U)
  {
    arc_run.failed = 0U;
    Debug_Print("error: arc segment\r\n");
    return 0U;
  }

  Debug_Print("arc done, drivers disabled\r\n");
  Debug_Print("ok\r\n");
  return 1U;
}

static uint8_t Arc_StartNextSegment(void)
{
  while (arc_run.segment_index < arc_run.total_segments)
  {
    float segment_ratio;
    float angle;
    float next_x;
    float next_y;
    int32_t next_x_um;
    int32_t next_y_um;
    int32_t next_x_steps;
    int32_t next_y_steps;

    arc_run.segment_index++;
    if (arc_run.segment_index >= arc_run.total_segments)
    {
      next_x_um = arc_run.target_x_um;
      next_y_um = arc_run.target_y_um;
    }
    else
    {
      segment_ratio = (float)arc_run.segment_index / (float)arc_run.total_segments;
      angle = arc_run.start_angle + (arc_run.sweep_angle * segment_ratio);
      next_x = (float)arc_run.center_x_um + (arc_run.radius_um * cosf(angle));
      next_y = (float)arc_run.center_y_um + (arc_run.radius_um * sinf(angle));
      next_x_um = (int32_t)(next_x + (next_x >= 0.0f ? 0.5f : -0.5f));
      next_y_um = (int32_t)(next_y + (next_y >= 0.0f ? 0.5f : -0.5f));
    }

    next_x_steps = Motion_XUmToSteps(next_x_um);
    next_y_steps = Motion_YUmToSteps(next_y_um);
    if ((next_x_steps == motion_x_steps) && (next_y_steps == motion_y_steps))
    {
      motion_x_um = next_x_um;
      motion_y_um = next_y_um;
      continue;
    }

    motion_debug_print = 0U;
    if (Motion_LineMoveTo(next_x_steps, next_y_steps, motion_z_steps, next_x_um, next_y_um, motion_z_um, arc_run.feed_mm_min, 0U) == 0U)
    {
      motion_debug_print = 1U;
      arc_run.active = 0U;
      arc_run.failed = 1U;
      return 0U;
    }
    return 1U;
  }

  arc_run.active = 0U;
  return 0U;
}

static int32_t Motion_XUmToSteps(int32_t value_um)
{
  return Motion_DivRoundClosest((int64_t)value_um * X_TRAVEL_STEPS, X_TRAVEL_UM);
}

static int32_t Motion_YUmToSteps(int32_t value_um)
{
  return Motion_DivRoundClosest((int64_t)value_um * Y_TRAVEL_STEPS, Y_TRAVEL_UM);
}

static int32_t Motion_ZUmToSteps(int32_t value_um)
{
  return Motion_DivRoundClosest((int64_t)value_um * Z_TRAVEL_STEPS, Z_TRAVEL_UM);
}

static int32_t Motion_XStepsToUm(int32_t steps)
{
  return Motion_DivRoundClosest((int64_t)steps * X_TRAVEL_UM, X_TRAVEL_STEPS);
}

static int32_t Motion_YStepsToUm(int32_t steps)
{
  return Motion_DivRoundClosest((int64_t)steps * Y_TRAVEL_UM, Y_TRAVEL_STEPS);
}

static int32_t Motion_ZStepsToUm(int32_t steps)
{
  return Motion_DivRoundClosest((int64_t)steps * Z_TRAVEL_UM, Z_TRAVEL_STEPS);
}

static void Motion_SetPositionSteps(int32_t x_steps, int32_t y_steps, int32_t z_steps)
{
  motion_x_steps = x_steps;
  motion_y_steps = y_steps;
  motion_z_steps = z_steps;
  motion_x_um = Motion_XStepsToUm(x_steps);
  motion_y_um = Motion_YStepsToUm(y_steps);
  motion_z_um = Motion_ZStepsToUm(z_steps);
  PositionStore_MarkDirty();
}

static uint8_t Motion_LineMoveTo(int32_t target_x_steps, int32_t target_y_steps, int32_t target_z_steps, int32_t target_x_um, int32_t target_y_um, int32_t target_z_um, uint32_t feed_mm_min, uint8_t rapid)
{
  int32_t dx_steps = target_x_steps - motion_x_steps;
  int32_t dy_steps = target_y_steps - motion_y_steps;
  int32_t dz_steps = target_z_steps - motion_z_steps;
  int32_t dx_um = target_x_um - motion_x_um;
  int32_t dy_um = target_y_um - motion_y_um;
  int32_t dz_um = target_z_um - motion_z_um;
  uint32_t x_steps = (dx_steps < 0) ? (uint32_t)(-dx_steps) : (uint32_t)dx_steps;
  uint32_t y_steps = (dy_steps < 0) ? (uint32_t)(-dy_steps) : (uint32_t)dy_steps;
  uint32_t z_steps = (dz_steps < 0) ? (uint32_t)(-dz_steps) : (uint32_t)dz_steps;
  uint32_t major_steps = (x_steps > y_steps) ? x_steps : y_steps;
  uint32_t step_period_us;

  if (z_steps > major_steps)
  {
    major_steps = z_steps;
  }

  Laser_SetOutput((gcode_laser_active != 0U) && (rapid == 0U) ? 1U : 0U);

  if (motion_run.state == MOTION_RUN_ACTIVE)
  {
    Debug_Print("error: motion busy\r\n");
    return 0U;
  }

  if ((target_x_steps < 0) || (target_x_steps > X_TRAVEL_STEPS) ||
      (target_y_steps < 0) || (target_y_steps > Y_TRAVEL_STEPS) ||
      (target_z_steps < 0) || (target_z_steps > (int32_t)Z_TRAVEL_STEPS) ||
      (target_x_um < 0) || (target_x_um > X_TRAVEL_UM) ||
      (target_y_um < 0) || (target_y_um > Y_TRAVEL_UM) ||
      (target_z_um < 0) || (target_z_um > Z_TRAVEL_UM))
  {
    Debug_Print("error: soft limit\r\n");
    return 0U;
  }

  if (major_steps == 0U)
  {
    motion_x_steps = target_x_steps;
    motion_y_steps = target_y_steps;
    motion_z_steps = target_z_steps;
    motion_x_um = target_x_um;
    motion_y_um = target_y_um;
    motion_z_um = target_z_um;
    PositionStore_MarkDirty();
    if (motion_debug_print != 0U)
    {
      Debug_Print("line noop\r\n");
      Debug_Print("ok\r\n");
    }
    return 1U;
  }

  step_period_us = Motion_CalcStepPeriodUs(dx_um, dy_um, dz_um, major_steps, feed_mm_min);

  if (motion_debug_print != 0U)
  {
    Debug_Print(rapid != 0U ? "G0 line " : "G1 line ");
    Debug_Print("Xsteps=");
    Debug_PrintU32(x_steps);
    Debug_Print(" Ysteps=");
    Debug_PrintU32(y_steps);
    Debug_Print(" Zsteps=");
    Debug_PrintU32(z_steps);
    Debug_Print(" period_us=");
    Debug_PrintU32(step_period_us);
    Debug_Print("\r\n");
  }

  if (x_steps > 0U)
  {
    HAL_GPIO_WritePin(debug_axis_x.dir_port, debug_axis_x.dir_pin, dx_steps > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(debug_axis_x.ena_port, debug_axis_x.ena_pin, GPIO_PIN_RESET);
  }
  if (y_steps > 0U)
  {
    HAL_GPIO_WritePin(debug_axis_y.dir_port, debug_axis_y.dir_pin, dy_steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(debug_axis_y.ena_port, debug_axis_y.ena_pin, GPIO_PIN_RESET);
  }
  if (z_steps > 0U)
  {
    HAL_GPIO_WritePin(debug_axis_z.dir_port, debug_axis_z.dir_pin, dz_steps > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(debug_axis_z.ena_port, debug_axis_z.ena_pin, GPIO_PIN_RESET);
  }

  __disable_irq();
  motion_run.state = MOTION_RUN_IDLE;
  motion_run.limit_abort = 0U;
  motion_run.step_index = 0U;
  motion_run.major_steps = major_steps;
  motion_run.x_steps = x_steps;
  motion_run.y_steps = y_steps;
  motion_run.z_steps = z_steps;
  motion_run.x_acc = 0U;
  motion_run.y_acc = 0U;
  motion_run.z_acc = 0U;
  motion_run.dx_steps = dx_steps;
  motion_run.dy_steps = dy_steps;
  motion_run.dz_steps = dz_steps;
  motion_run.target_x_steps = target_x_steps;
  motion_run.target_y_steps = target_y_steps;
  motion_run.target_z_steps = target_z_steps;
  motion_run.target_x_um = target_x_um;
  motion_run.target_y_um = target_y_um;
  motion_run.target_z_um = target_z_um;
  motion_run.rapid = rapid;
  __HAL_TIM_SET_AUTORELOAD(&htim5, step_period_us - 1U);
  __HAL_TIM_SET_COUNTER(&htim5, 0U);
  motion_run.state = MOTION_RUN_ACTIVE;
  __enable_irq();

  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
  {
    Motion_RequestAbort();
    Debug_Print("error: timer start\r\n");
    return 0U;
  }

  return 1U;
}

static uint32_t Motion_CalcStepPeriodUs(int32_t dx_um, int32_t dy_um, int32_t dz_um, uint32_t major_steps, uint32_t feed_mm_min)
{
  uint64_t x = (dx_um < 0) ? (uint64_t)(-dx_um) : (uint64_t)dx_um;
  uint64_t y = (dy_um < 0) ? (uint64_t)(-dy_um) : (uint64_t)dy_um;
  uint64_t z = (dz_um < 0) ? (uint64_t)(-dz_um) : (uint64_t)dz_um;
  uint32_t length_um = Motion_Isqrt64((x * x) + (y * y) + (z * z));
  uint64_t period_us;

  if ((feed_mm_min == 0U) || (major_steps == 0U) || (length_um == 0U))
  {
    return XY_INTERP_MIN_PERIOD_US;
  }

  period_us = ((uint64_t)length_um * 60000ULL) / ((uint64_t)feed_mm_min * major_steps);
  if (period_us < XY_INTERP_MIN_PERIOD_US)
  {
    period_us = XY_INTERP_MIN_PERIOD_US;
  }
  if (period_us > XY_INTERP_MAX_PERIOD_US)
  {
    period_us = XY_INTERP_MAX_PERIOD_US;
  }

  return (uint32_t)period_us;
}

static uint32_t Motion_Isqrt64(uint64_t value)
{
  uint64_t result = 0;
  uint64_t bit = 1ULL << 62;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0U)
  {
    if (value >= (result + bit))
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }
    bit >>= 2;
  }

  return (uint32_t)result;
}

static void LED_TimerTick(void)
{
  uint32_t phase_ms = HAL_GetTick() % LED_FLASH_PERIOD_MS;

  if (led_flash_ready == 0U)
  {
    return;
  }

  LED_SetFlashOn(phase_ms < LED_FLASH_ON_MS ? 1U : 0U);
}

static void LED_SetFlashOn(uint8_t enabled)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, enabled != 0U ? LED_FLASH_ON_STATE : LED_FLASH_OFF_STATE);
}

static void XY_Debug_PrintStatus(void)
{
  Debug_Print("STATUS ");
  Homing_PrintLimitMask(Homing_ReadLimitMask());
  Debug_Print("POS X=");
  Debug_PrintUmAsMm((uint32_t)motion_x_um);
  Debug_Print("mm Y=");
  Debug_PrintUmAsMm((uint32_t)motion_y_um);
  Debug_Print("mm Zdown=");
  Debug_PrintUmAsMm((uint32_t)motion_z_um);
  Debug_Print("mm Xsteps=");
  Debug_PrintU32((uint32_t)motion_x_steps);
  Debug_Print(" Ysteps=");
  Debug_PrintU32((uint32_t)motion_y_steps);
  Debug_Print(" Zsteps=");
  Debug_PrintU32((uint32_t)motion_z_steps);
  Debug_Print(" feed=");
  Debug_PrintU32(motion_feed_mm_min);
  Debug_Print(motion_absolute_mode != 0U ? " G90" : " G91");
  Debug_Print(" laser=");
  Debug_Print(gcode_laser_active != 0U ? "armed" : "off");
  Debug_Print(" S=");
  Debug_PrintU32(gcode_laser_power);
  Debug_Print("/");
  Debug_PrintU32(LASER_POWER_MAX);
#if (LASER_BINARY_ONLY != 0U)
  Debug_Print(" mode=binary");
#else
  Debug_Print(" mode=pwm");
#endif
  Debug_Print(" W25Q=");
  if (position_store_ready == 0U)
  {
    Debug_Print("off");
  }
  else if (position_store_dirty != 0U)
  {
    Debug_Print("dirty");
  }
  else
  {
    Debug_Print(position_store_restored != 0U ? "saved" : "empty");
  }
  Debug_Print(" seq=");
  Debug_PrintU32(position_store_sequence);
  Debug_Print("\r\n");
}

static void XY_Debug_PrepareStepPins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = STEP1_Pin|STEP2_Pin|STEP3_Pin|STEP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOE, STEP1_Pin|STEP2_Pin|STEP3_Pin|STEP4_Pin, GPIO_PIN_RESET);
}

static void XY_Debug_MoveOneRev(const AxisDebug_t *axis, GPIO_PinState dir_state)
{
  (void)XY_Debug_MovePulses(axis, dir_state, XY_DEBUG_PULSES_PER_REV, "jog");
}

static uint8_t XY_Debug_MovePulses(const AxisDebug_t *axis, GPIO_PinState dir_state, uint32_t pulses, const char *label)
{
  uint8_t aborted = 0U;

  Debug_Print(label);
  Debug_Print(" ");
  Debug_Print(axis->name);
  Debug_Print(dir_state == GPIO_PIN_RESET ? " DIR=0 pulses=" : " DIR=1 pulses=");
  Debug_PrintU32(pulses);
  Debug_Print("\r\n");

  HAL_GPIO_WritePin(axis->dir_port, axis->dir_pin, dir_state);
  HAL_GPIO_WritePin(axis->ena_port, axis->ena_pin, GPIO_PIN_RESET);
  HAL_Delay(200);

  for (uint32_t i = 0; i < pulses; i++)
  {
    XY_Debug_Pulse(axis->step_port, axis->step_pin, 1U, axis->step_period_us);

    if (XY_Debug_AbortRequested() != 0U)
    {
      aborted = 1U;
      break;
    }

  }

  HAL_GPIO_WritePin(axis->ena_port, axis->ena_pin, GPIO_PIN_SET);

  Debug_Print(aborted != 0U ? "aborted " : "done ");
  Debug_Print(axis->name);
  Debug_Print("\r\n");

  return aborted == 0U ? 1U : 0U;
}

static void XY_Debug_MoveToCenter(void)
{
  int32_t target_x_steps = (int32_t)X_CENTER_FROM_HOME_PULSES;
  int32_t target_y_steps = (int32_t)Y_CENTER_FROM_HOME_PULSES;
  int32_t target_z_steps = motion_z_steps;
  int32_t target_x_um = Motion_XStepsToUm(target_x_steps);
  int32_t target_y_um = Motion_YStepsToUm(target_y_steps);
  int32_t target_z_um = motion_z_um;
  uint8_t saved_debug_print = motion_debug_print;

  Debug_Print("MOVE X/Y TO CENTER TOGETHER, Z unchanged\r\n");

  motion_debug_print = 0U;
  if (Motion_LineMoveTo(target_x_steps, target_y_steps, target_z_steps, target_x_um, target_y_um, target_z_um, XY_RAPID_FEED_MM_MIN, 1U) == 0U)
  {
    motion_debug_print = saved_debug_print;
    XY_Debug_DisableAllDrivers();
    Debug_Print("CENTER ABORTED, move start failed\r\n");
    return;
  }

  while (motion_run.state == MOTION_RUN_ACTIVE)
  {
    if (XY_Debug_AbortRequested() != 0U)
    {
      Motion_RequestAbort();
    }
    Motion_Task();
  }

  Motion_Task();
  motion_debug_print = saved_debug_print;
  if ((motion_x_steps == target_x_steps) && (motion_y_steps == target_y_steps) && (motion_z_steps == target_z_steps))
  {
    Debug_Print("CENTER DONE, drivers disabled\r\n");
    Debug_Print("ok\r\n");
  }
  else
  {
    Debug_Print("error: center aborted\r\n");
  }
}

static uint8_t XY_Debug_AbortRequested(void)
{
  uint8_t cmd = 0;
  uint8_t aborted = 0U;

  while (HAL_UART_Receive(&huart1, &cmd, 1, 0) == HAL_OK)
  {
    if ((cmd == '!') || (cmd == 'd'))
    {
      aborted = 1U;
    }
    else
    {
      UART_BufferByteDuringMotion(cmd);
    }
  }

  if (aborted != 0U)
  {
    XY_Debug_DisableAllDrivers();
    Debug_Print("ABORT motion, drivers disabled\r\n");
  }

  return aborted;
}

static void XY_Debug_Pulse(GPIO_TypeDef *step_port, uint16_t step_pin, uint32_t pulses, uint32_t step_period_us)
{
  for (uint32_t i = 0; i < pulses; i++)
  {
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
    XY_Debug_DelayUs(XY_DEBUG_STEP_HIGH_US);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
    XY_Debug_DelayUs(step_period_us - XY_DEBUG_STEP_HIGH_US);
  }
}

static void XY_Debug_DelayUs(uint32_t delay_us)
{
  const uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  const uint32_t start = DWT->CYCCNT;
  const uint32_t cycles = delay_us * cycles_per_us;

  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

static void XY_Debug_DisableAllDrivers(void)
{
  HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENA3_GPIO_Port, ENA3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENA4_GPIO_Port, ENA4_Pin, GPIO_PIN_SET);
}

static void XY_Debug_EnableXY(void)
{
  HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
    LED_TimerTick();
  }
  else if (htim->Instance == TIM5)
  {
    Motion_TimerTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
