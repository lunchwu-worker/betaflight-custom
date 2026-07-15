/*
 * PR_F722-BT v1.5 legacy target for Betaflight 4.4.0
 * Pinout from unified config PR_F722-BT_v1.5
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "PF72"
#define USBD_PRODUCT_STRING     "PR_F722_BT"

#define USE_TARGET_CONFIG

#define DSHOT_BITBANG_DEFAULT DSHOT_BITBANG_ON
#define DSHOT_BITBANGED_TIMER_DEFAULT DSHOT_BITBANGED_TIMER_TIM1

#define LED0_PIN                PB0

#define USE_I2C_PULLUP

#define USE_BEEPER
#define BEEPER_PIN              PA4
#define BEEPER_INVERTED

#define MOTOR1_PIN              PC8
#define MOTOR2_PIN              PC9
#define MOTOR3_PIN              PA9
#define MOTOR4_PIN              PA10
#define MOTOR5_PIN              PB14
#define MOTOR6_PIN              PB15
#define MOTOR7_PIN              PA8
#define MOTOR8_PIN              PB1

#define USE_PINIO
#define PINIO1_PIN              PC4
#define PINIO2_PIN              PC13

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PB12
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_CS_PIN           PA5
#define GYRO_1_SPI_INSTANCE     SPI2

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_GYRO_SPI_LSM6DSV16X
#define USE_ACCGYRO_LSM6DSV16X
#define USE_ACCGYRO_SCS3304
#define USE_ACCGYRO_SCS3302
#define USE_ACCGYRO_IIM42652
#define USE_ACCGYRO_ICM40608
#define GYRO_1_ALIGN            CW0_DEG

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_ACC_SPI_LSM6DSV16X

#define USE_VCP

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PC10
#define UART3_RX_PIN            PC11

#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define SERIAL_PORT_COUNT       7

#define USE_SPI
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_PY25Q128HA
#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define BARO_I2C_INSTANCE       (I2CDEV_1)

#define USE_MAG
#define MAG_I2C_INSTANCE        (I2CDEV_1)

#define USE_GPS
#define USE_GPS_NMEA
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE
#define USE_LED_STRIP

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            0

#define VBAT_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC0

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2
#define SERIALRX_PROVIDER       SERIALRX_CRSF

#define DEFAULT_FEATURES        (FEATURE_GPS | FEATURE_TELEMETRY | FEATURE_LED_STRIP)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT     250

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 4
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(13))

#undef USE_SDCARD
#undef USE_SDCARD_SPI
#undef USE_SDCARD_SDIO
