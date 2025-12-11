#include "main.h"
#include <stdio.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* ======================= CONFIG ======================= */

// --- MPU-6500 (як MPU-6050) ---
#define MPU6500_ADDR             (0x68 << 1)

#define MPU6500_REG_SMPLRT_DIV   0x19
#define MPU6500_REG_CONFIG       0x1A
#define MPU6500_REG_GYRO_CONFIG  0x1B
#define MPU6500_REG_ACCEL_CONFIG 0x1C
#define MPU6500_REG_PWR_MGMT_1   0x6B
#define MPU6500_REG_WHO_AM_I     0x75

#define MPU6500_REG_ACCEL_XOUT_H 0x3B
#define MPU6500_REG_GYRO_XOUT_H  0x43

// --- I2C LCD 1602 (PCF8574) ---
// якщо на твоєму модулі адреса 0x3F – зміни тут на (0x3F<<1)
#define LCD_ADDR        (0x27 << 1)

#define LCD_BACKLIGHT   0x08
#define LCD_EN          0x04
#define LCD_RS          0x01

// "мертва зона" для гіроскопа, щоб дрібний шум не інтегрувався
#define GYRO_DEADZONE   0.5f   // град/с

/* ======================= Глобальні змінні ======================= */

// можна дивитись у Live Expressions
volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;

volatile float angle_x_deg = 0.0f;   // крен
volatile float angle_y_deg = 0.0f;   // тангаж
volatile float angle_z_deg = 0.0f;   // курс

// зсуви гіроскопа (калібрування)
static float offset_x_dps = 0.0f;
static float offset_y_dps = 0.0f;
static float offset_z_dps = 0.0f;

/* ======================= Прототипи ======================= */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* MPU6500 */
static void MPU6500_WriteReg(uint8_t reg, uint8_t value);
static void MPU6500_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t MPU6500_Init(void);
static void MPU6500_ReadGyroRaw(float *gx, float *gy, float *gz);
static void MPU6500_ReadGyro(void);
static void MPU6500_Calibrate(uint16_t samples);
static void MPU6500_ReadAccelAngles(float *roll_deg, float *pitch_deg, float *acc_norm);
static void UpdateAngles_Complementary(float dt);

/* LCD */
static void lcd_send(uint8_t data, uint8_t rs);
static void lcd_send_cmd(uint8_t cmd);
static void lcd_send_data(uint8_t data);
static void lcd_init(void);
static void lcd_clear(void);
static void lcd_set_cursor(uint8_t row, uint8_t col);
static void lcd_send_string(char *str);
static void lcd_display_angles(void);

/* ======================= main() ======================= */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  // --- Ініціалізація MPU ---
  if (!MPU6500_Init())
  {
    // WHO_AM_I невірний – сидимо тут
    while (1) { }
  }

  // --- Калібрування гіроскопа (плата нерухома!) ---
  MPU6500_Calibrate(500);   // ~2.5 с

  // Початкові кути з акселерометра
  float acc_roll0 = 0.0f, acc_pitch0 = 0.0f, acc_norm0 = 0.0f;
  MPU6500_ReadAccelAngles(&acc_roll0, &acc_pitch0, &acc_norm0);
  angle_x_deg = acc_roll0;
  angle_y_deg = acc_pitch0;
  angle_z_deg = 0.0f;

  // --- Ініціалізація LCD (адреса фіксована) ---
  lcd_init();
  lcd_set_cursor(0, 0);
  lcd_send_string("MPU6500 ready");
  HAL_Delay(1000);
  lcd_clear();

  uint32_t last_update_ms = HAL_GetTick(); // для обчислення dt
  uint16_t lcd_counter = 0;

  while (1)
  {
    // реальний період вибірки
    uint32_t now = HAL_GetTick();
    float dt = (now - last_update_ms) / 1000.0f;

    if (dt >= 0.005f)   // ~200 Гц
    {
      last_update_ms = now;

      // оновлюємо кути за комплементарним фільтром
      UpdateAngles_Complementary(dt);

      // раз на ~100 мс оновлюємо LCD
      if (++lcd_counter >= 20)
      {
        lcd_counter = 0;
        lcd_display_angles();
      }
    }

    HAL_Delay(1);  // щоб не крутитись в пусту
  }
}

/* ======================= MPU6500 ======================= */

static void MPU6500_WriteReg(uint8_t reg, uint8_t value)
{
  HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, reg,
                    I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static void MPU6500_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, reg,
                   I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

static uint8_t MPU6500_Init(void)
{
  uint8_t who = 0;
  MPU6500_ReadRegs(MPU6500_REG_WHO_AM_I, &who, 1);
  // 0x70 – MPU6500, 0x68 – сумісні/MPU6050
  if (who != 0x70 && who != 0x68)
  {
    return 0;
  }

  // вийти зі sleep
  MPU6500_WriteReg(MPU6500_REG_PWR_MGMT_1, 0x00);
  HAL_Delay(100);

  // DLPF=3 (≈42 Гц)
  MPU6500_WriteReg(MPU6500_REG_CONFIG, 0x03);
  // 1 кГц / (1+4) = 200 Гц
  MPU6500_WriteReg(MPU6500_REG_SMPLRT_DIV, 4);
  // гіро ±250°/с
  MPU6500_WriteReg(MPU6500_REG_GYRO_CONFIG, 0x00);
  // аксель ±2g
  MPU6500_WriteReg(MPU6500_REG_ACCEL_CONFIG, 0x00);

  return 1;
}

// читаємо "сирий" гіроскоп у град/с (без віднімання зсуву)
static void MPU6500_ReadGyroRaw(float *gx, float *gy, float *gz)
{
  uint8_t data[6];
  MPU6500_ReadRegs(MPU6500_REG_GYRO_XOUT_H, data, 6);

  int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
  int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
  int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

  const float sens = 131.0f; // LSB / (°/s)

  *gx = (float)raw_x / sens;
  *gy = (float)raw_y / sens;
  *gz = (float)raw_z / sens;
}

// читаємо гіроскоп з урахуванням зсуву та dead-zone
static void MPU6500_ReadGyro(void)
{
  float gx_raw, gy_raw, gz_raw;
  MPU6500_ReadGyroRaw(&gx_raw, &gy_raw, &gz_raw);

  float gx = gx_raw - offset_x_dps;
  float gy = gy_raw - offset_y_dps;
  float gz = gz_raw - offset_z_dps;

  if (fabsf(gx) < GYRO_DEADZONE) gx = 0.0f;
  if (fabsf(gy) < GYRO_DEADZONE) gy = 0.0f;
  if (fabsf(gz) < GYRO_DEADZONE) gz = 0.0f;

  gyro_x_dps = gx;
  gyro_y_dps = gy;
  gyro_z_dps = gz;
}

// калібрування – плата нерухома!
static void MPU6500_Calibrate(uint16_t samples)
{
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

  for (uint16_t i = 0; i < samples; i++)
  {
    float gx_raw, gy_raw, gz_raw;
    MPU6500_ReadGyroRaw(&gx_raw, &gy_raw, &gz_raw);
    sum_x += gx_raw;
    sum_y += gy_raw;
    sum_z += gz_raw;
    HAL_Delay(5); // ~200 Гц
  }

  offset_x_dps = sum_x / samples;
  offset_y_dps = sum_y / samples;
  offset_z_dps = sum_z / samples;
}

// кути з акселерометра + норма прискорення
static void MPU6500_ReadAccelAngles(float *roll_deg, float *pitch_deg, float *acc_norm)
{
  uint8_t data[6];
  MPU6500_ReadRegs(MPU6500_REG_ACCEL_XOUT_H, data, 6);

  int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
  int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
  int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);

  const float sens = 16384.0f; // LSB/g для ±2g

  float ax = (float)raw_ax / sens;
  float ay = (float)raw_ay / sens;
  float az = (float)raw_az / sens;

  float roll  = atan2f(ay, az) * 57.2957795f;                           // γ
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;    // θ
  float norm  = sqrtf(ax*ax + ay*ay + az*az);

  *roll_deg  = roll;
  *pitch_deg = pitch;
  *acc_norm  = norm;
}

// комплементарний фільтр (roll/pitch), yaw – чистий гіроскоп
static void UpdateAngles_Complementary(float dt)
{
  float acc_roll_deg, acc_pitch_deg, acc_norm;
  // 1) читаємо гіроскоп
  MPU6500_ReadGyro();
  // 2) читаємо акселерометр
  MPU6500_ReadAccelAngles(&acc_roll_deg, &acc_pitch_deg, &acc_norm);

  const float alpha = 0.98f; // 98% гіроскоп, 2% аксель

  float gyro_roll  = angle_x_deg + gyro_x_dps * dt;
  float gyro_pitch = angle_y_deg + gyro_y_dps * dt;
  float gyro_yaw   = angle_z_deg + gyro_z_dps * dt;

  // roll, pitch – комплементарний фільтр
  angle_x_deg = alpha * gyro_roll  + (1.0f - alpha) * acc_roll_deg;
  angle_y_deg = alpha * gyro_pitch + (1.0f - alpha) * acc_pitch_deg;

  // yaw – тільки гіроскоп (без магнітометра)
  angle_z_deg = gyro_yaw;

  // якщо сенсор лежить спокійно – обнуляємо курс (щоб не тікав на столі)
  if (fabsf(gyro_x_dps) < 0.05f && fabsf(gyro_y_dps) < 0.05f && fabsf(gyro_z_dps) < 0.05f
      && fabsf(acc_norm - 1.0f) < 0.05f)
  {
    angle_z_deg = 0.0f;
  }

  // обмеження в діапазоні [-180; 180]
  if (angle_x_deg > 180.0f)  angle_x_deg -= 360.0f;
  if (angle_x_deg < -180.0f) angle_x_deg += 360.0f;

  if (angle_y_deg > 180.0f)  angle_y_deg -= 360.0f;
  if (angle_y_deg < -180.0f) angle_y_deg += 360.0f;

  if (angle_z_deg > 180.0f)  angle_z_deg -= 360.0f;
  if (angle_z_deg < -180.0f) angle_z_deg += 360.0f;
}

/* ======================= LCD ======================= */

static void lcd_send(uint8_t data, uint8_t rs)
{
  uint8_t high = data & 0xF0;
  uint8_t low  = (data << 4) & 0xF0;
  uint8_t buf[4];

  uint8_t rs_mask = rs ? LCD_RS : 0x00;

  buf[0] = high | LCD_BACKLIGHT | rs_mask | LCD_EN;
  buf[1] = high | LCD_BACKLIGHT | rs_mask;
  buf[2] = low  | LCD_BACKLIGHT | rs_mask | LCD_EN;
  buf[3] = low  | LCD_BACKLIGHT | rs_mask;

  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, buf, 4, 100);
}

static void lcd_send_cmd(uint8_t cmd)
{
  lcd_send(cmd, 0);
}

static void lcd_send_data(uint8_t data)
{
  lcd_send(data, 1);
}

static void lcd_clear(void)
{
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
  uint8_t addr = (row == 0) ? 0x00 : 0x40;
  addr += col;
  lcd_send_cmd(0x80 | addr);
}

static void lcd_send_string(char *str)
{
  while (*str)
  {
    lcd_send_data((uint8_t)*str++);
  }
}

static void lcd_init(void)
{
  HAL_Delay(50);
  lcd_send_cmd(0x33);
  lcd_send_cmd(0x32);
  lcd_send_cmd(0x28);   // 4-bit, 2 рядки
  lcd_send_cmd(0x0C);   // дисплей ON, курсор OFF
  lcd_send_cmd(0x06);   // інкремент адреси
  lcd_clear();
}

// Вивід X,Y,Z з однією цифрою після коми
static void lcd_display_angles(void)
{
  int16_t x10 = (int16_t)(angle_x_deg * 10.0f);
  int16_t y10 = (int16_t)(angle_y_deg * 10.0f);
  int16_t z10 = (int16_t)(angle_z_deg * 10.0f);

  int16_t xi = x10 / 10;
  int16_t yi = y10 / 10;
  int16_t zi = z10 / 10;

  int16_t xd = x10 - xi * 10;
  int16_t yd = y10 - yi * 10;
  int16_t zd = z10 - zi * 10;

  if (xd < 0) xd = -xd;
  if (yd < 0) yd = -yd;
  if (zd < 0) zd = -zd;

  char line1[17];
  char line2[17];

  snprintf(line1, sizeof(line1), "X=%3d.%1d Y=%3d.%1d", xi, xd, yi, yd);
  snprintf(line2, sizeof(line2), "Z=%3d.%1d        ", zi, zd);

  lcd_set_cursor(0, 0);
  lcd_send_string(line1);
  lcd_set_cursor(1, 0);
  lcd_send_string(line2);
}

/* ======================= Системні функції ======================= */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                              | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1
                              | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;  // 100 кГц – надійно
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();   // PB6/PB7 – I2C1
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
