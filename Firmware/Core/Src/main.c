

/* USER CODE BEGIN Header */
/*
 * STM32L475 + HTS221 (I2C2) — HAL only
 * - Escáner I2C, init de HTS221 (WHO_AM_I, CTRL_REG1, AV_CONF)
 * - Lectura de temperatura con calibración (T0/T1, T0_OUT/T1_OUT)
 * - Print por UART1 @115200 cada 1 s
 * - LED2 parpadea para “heartbeat”
 */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>

/* ==== Peripherals handles (generados por CubeMX normalmente) ==== */
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;

/* ==== GPIO LED (ajusta si tu LED es otro pin/puerto) ==== */
#define LED2_GPIO_Port GPIOB
#define LED2_Pin       GPIO_PIN_14

/* ==== I2C+HTS221 defs ==== */
#define HTS221_ADDR_7B         0x5F
#define I2C_ADDR(x)            ((x) << 1)

#define HTS221_WHO_AM_I        0x0F
#define HTS221_WHO_AM_I_VAL    0xBC
#define HTS221_AV_CONF         0x10
#define HTS221_CTRL_REG1       0x20
#define HTS221_STATUS_REG      0x27
#define HTS221_TEMP_OUT_L      0x2A
#define HTS221_TEMP_OUT_H      0x2B
#define HTS221_H0_rH_x2        0x30
#define HTS221_H1_rH_x2        0x31
#define HTS221_T0_degC_x8      0x32
#define HTS221_T1_degC_x8      0x33
#define HTS221_T0_T1_MSB       0x35
#define HTS221_T0_OUT_L        0x3C
#define HTS221_T0_OUT_H        0x3D
#define HTS221_T1_OUT_L        0x3E
#define HTS221_T1_OUT_H        0x3F

// --- LPS22HB (Pressure) I2C ---
// Dirección 7-bit = 0x5C -> para HAL se pasa 8-bit (<<1)
#define LPS22HB_ADDR        (0x5C << 1)
#define LPS22HB_WHO_AM_I    0x0F
#define LPS22HB_WHOAMI_VAL  0xB1
#define LPS22HB_CTRL_REG1   0x10
#define LPS22HB_CTRL_REG2   0x11
#define LPS22HB_PRESS_OUT_XL 0x28

// CTRL_REG1: ODR=1 Hz (001<<4 = 0x10), BDU=1 (0x02) => 0x12
#define LPS22HB_CTRL1_CFG   0x12
// CTRL_REG2: IF_ADD_INC=1 (bit4) => 0x10 (auto-increment en lecturas)
#define LPS22HB_CTRL2_CFG   0x10

/* ==== Prototipos mínimos ==== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);

static HAL_StatusTypeDef LPS22HB_Init(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef LPS22HB_ReadPressure_hPa(I2C_HandleTypeDef *hi2c, float *out_hPa);


/* ==== Utils UART ==== */
static void uprintln(const char *s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 200);
  const char crlf[] = "\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)crlf, 2, 200);
}
static void uprintf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 200);
}

/* ==== I2C helpers ==== */
static void I2C_Scan(I2C_HandleTypeDef *hi2c, const char *name) {
  uprintf("[I2C] Scanning %s ...\r\n", name);
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (HAL_I2C_IsDeviceReady(hi2c, I2C_ADDR(a), 1, 10) == HAL_OK) {
      uprintf("  - Found: 0x%02X (8-bit 0x%02X)\r\n", a, I2C_ADDR(a));
    }
  }
}

/* ==== Calibración temperatura ==== */
typedef struct {
  float T0_degC;
  float T1_degC;
  int16_t T0_OUT;
  int16_t T1_OUT;
} HTS221_TCal;

static HAL_StatusTypeDef HTS221_ReadCal_T(HTS221_TCal *cal) {
  uint8_t t0_l, t1_l, t_msb, b[2];

  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T0_degC_x8,
                       I2C_MEMADD_SIZE_8BIT, &t0_l, 1, 50) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T1_degC_x8,
                       I2C_MEMADD_SIZE_8BIT, &t1_l, 1, 50) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T0_T1_MSB,
                       I2C_MEMADD_SIZE_8BIT, &t_msb, 1, 50) != HAL_OK) return HAL_ERROR;

  uint16_t T0_x8 = ((t_msb & 0x03) << 8) | t0_l;   // 10 bits
  uint16_t T1_x8 = ((t_msb & 0x0C) << 6) | t1_l;   // 10 bits
  cal->T0_degC = T0_x8 / 8.0f;
  cal->T1_degC = T1_x8 / 8.0f;

  // T0_OUT
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T0_OUT_L,
                       I2C_MEMADD_SIZE_8BIT, b, 1, 50) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T0_OUT_H,
                       I2C_MEMADD_SIZE_8BIT, b+1, 1, 50) != HAL_OK) return HAL_ERROR;
  cal->T0_OUT = (int16_t)((b[1] << 8) | b[0]);

  // T1_OUT
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T1_OUT_L,
                       I2C_MEMADD_SIZE_8BIT, b, 1, 50) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_T1_OUT_H,
                       I2C_MEMADD_SIZE_8BIT, b+1, 1, 50) != HAL_OK) return HAL_ERROR;
  cal->T1_OUT = (int16_t)((b[1] << 8) | b[0]);

  return HAL_OK;
}

static HAL_StatusTypeDef HTS221_Init(void) {
  uint8_t v = 0;

  // WHO_AM_I
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &v, 1, 50) != HAL_OK) {
    uprintln("HTS221 WHO_AM_I read failed (I2C)");
    return HAL_ERROR;
  }
  if (v != HTS221_WHO_AM_I_VAL) {
    uprintf("HTS221 WHO_AM_I mismatch: 0x%02X\r\n", v);
    return HAL_ERROR;
  }

  // AV_CONF: promedios (opcional). Por ejemplo temp avg = 16 samples, hum = 32 samples.
  uint8_t av = 0b00011110; // H_AVG=32 (101), T_AVG=16 (100) -> ver tabla datasheet
  if (HAL_I2C_Mem_Write(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_AV_CONF,
                        I2C_MEMADD_SIZE_8BIT, &av, 1, 50) != HAL_OK) {
    uprintln("HTS221 AV_CONF write failed");
    return HAL_ERROR;
  }

  // CTRL_REG1: PD=1 (bit7), BDU=1(bit2), ODR=01 (1 Hz)
  uint8_t ctrl1 = 0x80 | 0x04 | 0x01; // 0x85
  if (HAL_I2C_Mem_Write(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_CTRL_REG1,
                        I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 50) != HAL_OK) {
    uprintln("HTS221 CTRL_REG1 write failed");
    return HAL_ERROR;
  }

  uprintln("HTS221 configured");
  return HAL_OK;
}

static HAL_StatusTypeDef HTS221_ReadTemp_C(float *tC, const HTS221_TCal *cal) {
  // Leer TEMP_OUT_H/L
  uint8_t b[2];
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_TEMP_OUT_L,
                       I2C_MEMADD_SIZE_8BIT, &b[0], 1, 50) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR(HTS221_ADDR_7B), HTS221_TEMP_OUT_H,
                       I2C_MEMADD_SIZE_8BIT, &b[1], 1, 50) != HAL_OK) return HAL_ERROR;
  int16_t T_OUT = (int16_t)((b[1] << 8) | b[0]);

  // Interpolación lineal según AN
  // T(°C) = T0 + (T_OUT - T0_OUT) * (T1 - T0) / (T1_OUT - T0_OUT)
  float num = (float)(T_OUT - cal->T0_OUT) * (cal->T1_degC - cal->T0_degC);
  float den = (float)(cal->T1_OUT - cal->T0_OUT);
  if (fabsf(den) < 1e-3f) return HAL_ERROR;

  *tC = cal->T0_degC + (num / den);
  return HAL_OK;
}

static HAL_StatusTypeDef LPS22HB_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t who = 0;
  if (HAL_I2C_Mem_Read(hi2c, LPS22HB_ADDR, LPS22HB_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                       &who, 1, 100) != HAL_OK) {
    return HAL_ERROR;
  }
  if (who != LPS22HB_WHOAMI_VAL) {
    return HAL_ERROR;
  }

  uint8_t v;
  // CTRL_REG2: IF_ADD_INC=1 para autoincremento
  v = LPS22HB_CTRL2_CFG;
  if (HAL_I2C_Mem_Write(hi2c, LPS22HB_ADDR, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT,
                        &v, 1, 100) != HAL_OK) {
    return HAL_ERROR;
  }

  // CTRL_REG1: ODR=1Hz, BDU=1
  v = LPS22HB_CTRL1_CFG;
  if (HAL_I2C_Mem_Write(hi2c, LPS22HB_ADDR, LPS22HB_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
                        &v, 1, 100) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef LPS22HB_ReadPressure_hPa(I2C_HandleTypeDef *hi2c, float *out_hPa)
{
  uint8_t buf[3];
  if (HAL_I2C_Mem_Read(hi2c, LPS22HB_ADDR, LPS22HB_PRESS_OUT_XL, I2C_MEMADD_SIZE_8BIT,
                       buf, 3, 100) != HAL_OK) {
    return HAL_ERROR;
  }
  // 24-bit two's complement, 4096 LSB/hPa
  int32_t raw = (int32_t)((((uint32_t)buf[2]) << 16) | (((uint32_t)buf[1]) << 8) | buf[0]);
  if (raw & 0x00800000) { raw |= 0xFF000000; } // sign-extend
  *out_hPa = ((float)raw) / 4096.0f;
  return HAL_OK;
}


/* ==== MAIN ==== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();

  uprintln("\r\nSTM32 HAL + HTS221 (no RTOS)");

  // Escáner I2C2
  I2C_Scan(&hi2c2, "I2C2");

  // Init sensor
  if (HTS221_Init() != HAL_OK) {
    uprintln("HTS221 init ERROR. Verifica I2C2 y direccion 0x5F.");
    // seguimos, pero indicamos fallo parpadeando rápido
  } else {
    uprintln("HTS221 init OK");
  }
  if (LPS22HB_Init(&hi2c2) == HAL_OK) {
    uprintf("LPS22HB init OK\r\n");
  } else {
    uprintf("LPS22HB init FAIL\r\n");
  }


  // Leer calibración
  HTS221_TCal cal = {0};
  if (HTS221_ReadCal_T(&cal) != HAL_OK) {
    uprintln("HTS221 cal read ERROR");
  } else {
    uprintf("Cal: T0=%.2f C, T1=%.2f C, T0_OUT=%d, T1_OUT=%d\r\n",
            cal.T0_degC, cal.T1_degC, cal.T0_OUT, cal.T1_OUT);
  }

  // Loop
  // Loop
  while (1) {
    float tC = NAN;
    float p_hPa = NAN;

    // Lee temperatura usando nuestra función correcta (con calibración)
    if (HTS221_ReadTemp_C(&tC, &cal) != HAL_OK) {
      tC = NAN;
    }

    // Lee presión del LPS22HB
    if (LPS22HB_ReadPressure_hPa(&hi2c2, &p_hPa) != HAL_OK) {
      p_hPa = NAN;
    }

    // Imprime JSON por UART
    uprintf("{\"temp_c\":%.2f,\"pres_hpa\":%.2f}\r\n", tC, p_hPa);

    HAL_Delay(1000); // <- sin RTOS usamos HAL_Delay
  }
}

/* ==== INITs generados por CubeMX — simplificados (ajusta si hace falta) ==== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // 4 MHz base + PLL abajo
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 80 MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC; // ~100kHz típico L4 (ajustado por CubeMX suele ser mejor)
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

  /** Configure Analogue filter */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
  /** Configure Digital filter */
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
}

/* Error Handler mínimo */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    HAL_Delay(100);
  }

}


