# Лабораторна робота 4–5  
## Визначення просторового положення за даними гіроскопа (MPU-6500)  
STM32F401CBU6, варіант 2

---
## Використане обладнання
- Датчик: **MPU-6500** (I²C, 3-осьовий гіроскоп + акселерометр).
- Дисплей: **LCD 1602** з I²C-адаптером (PCF8574, адреса 0x27 або 0x3F).
- Макетна плата, дроти «мама–мама».
---

## Фото фізичної збірки

<img width="1280" height="960" alt="image" src="https://github.com/user-attachments/assets/0a661515-f59c-4444-b2fd-7daaf971831e" />


## Розпіновка I2C в CubeIDE

<img width="480" height="427" alt="image" src="https://github.com/user-attachments/assets/2f334ed4-9487-4cd7-8632-a49f86a4dd9e" />

## Опис роботи прошивки
### Ініціалізація MPU-6500

У функції `MPU6500_Init()` виконується:

- Читання регістра **WHO_AM_I** (`0x75`) та перевірка, що значення = `0x70` (MPU6500) або `0x68` (сумісні).
- Вихід з режиму сну:
  - запис у `PWR_MGMT_1` (`0x6B`) значення `0x00`.
- Налаштування фільтра та частоти дискретизації:
  - `CONFIG` (`0x1A`) = `0x03` → DLPF ≈ 42 Гц;
  - `SMPLRT_DIV` (`0x19`) = `4` → 1 кГц / (1 + 4) = **200 Гц** (варіант 2).
- Діапазони вимірювання:
  - гіроскоп: `GYRO_CONFIG` (`0x1B`) = `0x00` → ±250 °/с;
  - акселерометр: `ACCEL_CONFIG` (`0x1C`) = `0x00` → ±2 g.

### Калібрування гіроскопа

Функція `MPU6500_Calibrate(samples)`:
- Плата лежить нерухомо;
- Робиться 500 вимірів сирого гіроскопа;
- Обчислюється середнє по X, Y, Z → ці значення записуються як зсуви:
  - `offset_x_dps`, `offset_y_dps`, `offset_z_dps`;
- Далі при читанні гіроскопа ці зсуви віднімаються.


### Обчислення кутів

- Значення гіроскопа (`gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps`) виражені в **°/с**.
- З акселерометра обчислюються кути:
  - (*roll*) і (*pitch*) через `atan2`:
  
'''
roll  = atan2(ay, az);
pitch = atan2(-ax, sqrt(ay*ay + az*az));
'''

Використовується комплементарний фільтр:
'''
angle_x_deg = alpha * (angle_x_deg + gyro_x_dps * dt) + (1.0f - alpha) * acc_roll_deg;
angle_y_deg = alpha * (angle_y_deg + gyro_y_dps * dt) + (1.0f - alpha) * acc_pitch_deg;
angle_z_deg = angle_z_deg + gyro_z_dps * dt;    // yaw – тільки гіроскоп
'''
де alpha ≈ 0.98, а dt — реальний інтервал часу між вимірюваннями (через HAL_GetTick()).

Для зменшення накопичення дрібного шуму використовується dead-zone:
'''
if (fabsf(gx) < 0.5f) gx = 0.0f;
'''

*** Виведення значень
Live Expressions

<img width="675" height="217" alt="image" src="https://github.com/user-attachments/assets/775ad018-c7a5-48ec-b266-d9b4047829a4" />

## Демонстрація роботи

[![Переглянути відео](https://github.com/user-attachments/assets/2bb4dbcc-9d4f-4bd8-bf9d-bb7b0b7cf5ae)](https://drive.google.com/file/d/13qyvj5gRWeiZHrXewYpmiSxDpugw7d8-/view?usp=sharing)
