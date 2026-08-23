#include "stm32f4xx_hal.h"

#include <string.h>

#define APP_BASE_ADDR        0x08010000UL
#define FLASH_END_ADDR       0x08080000UL
#define APP_MAX_SIZE         (FLASH_END_ADDR - APP_BASE_ADDR)
#define BOOT_WAIT_MS         5000U
#define UART_LINE_BUF_SIZE   96U
#define UART_RX_CHUNK_SIZE   256U
#define UART_BOOT_TIMEOUT_MS 10000U

#define LED_Pin              GPIO_PIN_13
#define LED_GPIO_Port        GPIOC
#define ENA2_Pin             GPIO_PIN_0
#define ENA2_GPIO_Port       GPIOB
#define ENA3_Pin             GPIO_PIN_10
#define ENA3_GPIO_Port       GPIOE
#define ENA4_Pin             GPIO_PIN_12
#define ENA4_GPIO_Port       GPIOE
#define ENA1_Pin             GPIO_PIN_15
#define ENA1_GPIO_Port       GPIOE
#define LASER_Pin            GPIO_PIN_14
#define LASER_GPIO_Port      GPIOD
#define MOTOR775_Pin         GPIO_PIN_15
#define MOTOR775_GPIO_Port   GPIOD

UART_HandleTypeDef huart1;

static void SystemClock_Config(void);
static void Boot_GPIO_Init(void);
static void Boot_UART_Init(void);
static void Boot_Print(const char *text);
static void Boot_PrintU32(uint32_t value);
static void Boot_PrintHex32(uint32_t value);
static uint8_t Boot_ReadLine(char *buf, uint32_t len, uint32_t timeout_ms);
static uint8_t Boot_ParseU32(const char **p, uint32_t *value);
static uint8_t Boot_ParseHex32(const char **p, uint32_t *value);
static const char *Boot_SkipSpaces(const char *p);
static uint8_t Boot_HandleCommand(const char *line);
static uint8_t Boot_UpdateApp(uint32_t image_size, uint32_t expected_crc);
static uint8_t Boot_EraseAppFlash(void);
static uint8_t Boot_ProgramByte(uint32_t *address, uint8_t *word_bytes, uint32_t *word_len, uint8_t value);
static uint8_t Boot_FlushWord(uint32_t *address, uint8_t *word_bytes, uint32_t *word_len);
static uint32_t Boot_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t len);
static uint8_t Boot_AppIsValid(void);
static void Boot_JumpToApp(void);
static void Error_Handler(void);

void SysTick_Handler(void)
{
  HAL_IncTick();
}

int main(void)
{
  char line[UART_LINE_BUF_SIZE];
  uint32_t boot_deadline;

  HAL_Init();
  SystemClock_Config();
  Boot_GPIO_Init();
  Boot_UART_Init();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  Boot_Print("\r\nBL READY APP=0x");
  Boot_PrintHex32(APP_BASE_ADDR);
  Boot_Print(" MAX=");
  Boot_PrintU32(APP_MAX_SIZE);
  Boot_Print("\r\n");

  boot_deadline = HAL_GetTick() + BOOT_WAIT_MS;
  while (1)
  {
    uint32_t now = HAL_GetTick();
    uint32_t timeout = HAL_MAX_DELAY;

    if (Boot_AppIsValid() != 0U)
    {
      if ((int32_t)(boot_deadline - now) <= 0)
      {
        Boot_JumpToApp();
      }
      timeout = boot_deadline - now;
    }

    if (Boot_ReadLine(line, sizeof(line), timeout) == 0U)
    {
      if (Boot_AppIsValid() != 0U)
      {
        Boot_JumpToApp();
      }
      continue;
    }

    boot_deadline = HAL_GetTick() + BOOT_WAIT_MS;
    (void)Boot_HandleCommand(line);
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Boot_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, ENA1_Pin | ENA3_Pin | ENA4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LASER_Pin | MOTOR775_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENA2_Pin;
  HAL_GPIO_Init(ENA2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENA1_Pin | ENA3_Pin | ENA4_Pin;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LASER_Pin | MOTOR775_Pin;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void Boot_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Boot_Print(const char *text)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 1000);
}

static void Boot_PrintU32(uint32_t value)
{
  char digits[10];
  uint32_t count = 0;

  if (value == 0U)
  {
    Boot_Print("0");
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

static void Boot_PrintHex32(uint32_t value)
{
  static const char hex[] = "0123456789ABCDEF";
  char out[8];

  for (uint32_t i = 0; i < 8U; i++)
  {
    out[7U - i] = hex[value & 0x0FU];
    value >>= 4;
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)out, sizeof(out), 1000);
}

static uint8_t Boot_ReadLine(char *buf, uint32_t len, uint32_t timeout_ms)
{
  uint32_t pos = 0;
  uint32_t start = HAL_GetTick();

  if (len == 0U)
  {
    return 0U;
  }

  while ((timeout_ms == HAL_MAX_DELAY) || ((HAL_GetTick() - start) < timeout_ms))
  {
    uint8_t ch;
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) != HAL_OK)
    {
      continue;
    }

    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      buf[pos] = '\0';
      return pos > 0U ? 1U : 0U;
    }
    if ((ch < 0x20U) || (ch > 0x7EU))
    {
      continue;
    }
    if (pos >= (len - 1U))
    {
      pos = 0;
      Boot_Print("ERR LINE\r\n");
      continue;
    }
    buf[pos++] = (char)ch;
  }

  return 0U;
}

static uint8_t Boot_ParseU32(const char **p, uint32_t *value)
{
  uint32_t result = 0;
  uint8_t has_digit = 0U;

  *p = Boot_SkipSpaces(*p);
  while ((**p >= '0') && (**p <= '9'))
  {
    result = (result * 10U) + (uint32_t)(**p - '0');
    has_digit = 1U;
    (*p)++;
  }

  *value = result;
  return has_digit;
}

static uint8_t Boot_ParseHex32(const char **p, uint32_t *value)
{
  uint32_t result = 0;
  uint8_t has_digit = 0U;

  *p = Boot_SkipSpaces(*p);
  if (((*p)[0] == '0') && (((*p)[1] == 'x') || ((*p)[1] == 'X')))
  {
    *p += 2;
  }

  while (1)
  {
    char ch = **p;
    uint32_t digit;

    if ((ch >= '0') && (ch <= '9'))
    {
      digit = (uint32_t)(ch - '0');
    }
    else if ((ch >= 'a') && (ch <= 'f'))
    {
      digit = (uint32_t)(ch - 'a' + 10);
    }
    else if ((ch >= 'A') && (ch <= 'F'))
    {
      digit = (uint32_t)(ch - 'A' + 10);
    }
    else
    {
      break;
    }

    result = (result << 4) | digit;
    has_digit = 1U;
    (*p)++;
  }

  *value = result;
  return has_digit;
}

static const char *Boot_SkipSpaces(const char *p)
{
  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }
  return p;
}

static uint8_t Boot_HandleCommand(const char *line)
{
  if (strcmp(line, "PING") == 0)
  {
    Boot_Print("PONG\r\n");
    return 1U;
  }

  if (strcmp(line, "BOOT") == 0)
  {
    if (Boot_AppIsValid() != 0U)
    {
      Boot_JumpToApp();
    }
    Boot_Print("ERR NOAPP\r\n");
    return 0U;
  }

  if (strncmp(line, "FWUP", 4) == 0)
  {
    const char *p = line + 4;
    uint32_t image_size = 0;
    uint32_t expected_crc = 0;

    if ((Boot_ParseU32(&p, &image_size) == 0U) || (Boot_ParseHex32(&p, &expected_crc) == 0U))
    {
      Boot_Print("ERR FWUP ARG\r\n");
      return 0U;
    }
    return Boot_UpdateApp(image_size, expected_crc);
  }

  Boot_Print("ERR CMD\r\n");
  return 0U;
}

static uint8_t Boot_UpdateApp(uint32_t image_size, uint32_t expected_crc)
{
  uint8_t rx[UART_RX_CHUNK_SIZE];
  uint8_t word_bytes[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
  uint32_t word_len = 0;
  uint32_t address = APP_BASE_ADDR;
  uint32_t received = 0;
  uint32_t crc = 0xFFFFFFFFU;

  if ((image_size == 0U) || (image_size > APP_MAX_SIZE))
  {
    Boot_Print("ERR SIZE\r\n");
    return 0U;
  }

  Boot_Print("ERASE\r\n");
  if (Boot_EraseAppFlash() == 0U)
  {
    Boot_Print("ERR ERASE\r\n");
    return 0U;
  }

  Boot_Print("READY DATA\r\n");
  while (received < image_size)
  {
    uint32_t chunk = image_size - received;
    if (chunk > UART_RX_CHUNK_SIZE)
    {
      chunk = UART_RX_CHUNK_SIZE;
    }

    Boot_Print("NEXT ");
    Boot_PrintU32(received);
    Boot_Print(" ");
    Boot_PrintU32(chunk);
    Boot_Print("\r\n");

    if (HAL_UART_Receive(&huart1, rx, (uint16_t)chunk, UART_BOOT_TIMEOUT_MS) != HAL_OK)
    {
      HAL_FLASH_Lock();
      Boot_Print("ERR RX\r\n");
      return 0U;
    }

    crc = Boot_Crc32Update(crc, rx, chunk);
    for (uint32_t i = 0; i < chunk; i++)
    {
      if (Boot_ProgramByte(&address, word_bytes, &word_len, rx[i]) == 0U)
      {
        HAL_FLASH_Lock();
        Boot_Print("ERR WRITE\r\n");
        return 0U;
      }
    }

    received += chunk;
  }

  if (Boot_FlushWord(&address, word_bytes, &word_len) == 0U)
  {
    HAL_FLASH_Lock();
    Boot_Print("ERR WRITE\r\n");
    return 0U;
  }

  HAL_FLASH_Lock();
  crc ^= 0xFFFFFFFFU;
  if (crc != expected_crc)
  {
    Boot_Print("ERR CRC GOT=0x");
    Boot_PrintHex32(crc);
    Boot_Print("\r\n");
    return 0U;
  }

  if (Boot_AppIsValid() == 0U)
  {
    Boot_Print("ERR VECTOR\r\n");
    return 0U;
  }

  Boot_Print("OK FW\r\n");
  HAL_Delay(200);
  Boot_JumpToApp();
  return 1U;
}

static uint8_t Boot_EraseAppFlash(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_4;
  erase.NbSectors = 4;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  return 1U;
}

static uint8_t Boot_ProgramByte(uint32_t *address, uint8_t *word_bytes, uint32_t *word_len, uint8_t value)
{
  word_bytes[*word_len] = value;
  (*word_len)++;

  if (*word_len < 4U)
  {
    return 1U;
  }

  return Boot_FlushWord(address, word_bytes, word_len);
}

static uint8_t Boot_FlushWord(uint32_t *address, uint8_t *word_bytes, uint32_t *word_len)
{
  uint32_t word;

  if (*word_len == 0U)
  {
    return 1U;
  }

  while (*word_len < 4U)
  {
    word_bytes[*word_len] = 0xFFU;
    (*word_len)++;
  }

  if ((*address < APP_BASE_ADDR) || ((*address + 4U) > FLASH_END_ADDR))
  {
    return 0U;
  }

  word = ((uint32_t)word_bytes[0]) |
         ((uint32_t)word_bytes[1] << 8) |
         ((uint32_t)word_bytes[2] << 16) |
         ((uint32_t)word_bytes[3] << 24);

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *address, word) != HAL_OK)
  {
    return 0U;
  }

  *address += 4U;
  *word_len = 0U;
  word_bytes[0] = 0xFFU;
  word_bytes[1] = 0xFFU;
  word_bytes[2] = 0xFFU;
  word_bytes[3] = 0xFFU;
  return 1U;
}

static uint32_t Boot_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
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
  return crc;
}

static uint8_t Boot_AppIsValid(void)
{
  uint32_t stack = *(volatile uint32_t *)APP_BASE_ADDR;
  uint32_t reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4U);

  if ((stack < 0x20000000UL) || (stack > 0x20020000UL))
  {
    return 0U;
  }
  if ((reset < APP_BASE_ADDR) || (reset >= FLASH_END_ADDR))
  {
    return 0U;
  }
  return 1U;
}

static void Boot_JumpToApp(void)
{
  typedef void (*AppEntry_t)(void);
  uint32_t app_stack = *(volatile uint32_t *)APP_BASE_ADDR;
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4U);
  AppEntry_t app_entry = (AppEntry_t)app_reset;

  Boot_Print("BOOT APP\r\n");
  HAL_Delay(20);
  HAL_UART_DeInit(&huart1);
  HAL_DeInit();

  __disable_irq();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  for (uint32_t i = 0; i < 8U; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  SCB->VTOR = APP_BASE_ADDR;
  __set_MSP(app_stack);
  __DSB();
  __ISB();
  __enable_irq();
  app_entry();
}

static void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    for (volatile uint32_t i = 0; i < 500000U; i++)
    {
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
