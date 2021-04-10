/*
 * BMP280.h
 *
 *  Created on: Jan 5, 2021
 *      Author: HAOO
 */

#ifndef SRC_BMP280_H_
#define SRC_BMP280_H_

#include "stm32f4xx_hal.h"

/*!
 * Registers available on the sensor.
 */
enum {
  BMP280_REGISTER_DIG_T1 = 0x88,
  BMP280_REGISTER_DIG_T2 = 0x8A,
  BMP280_REGISTER_DIG_T3 = 0x8C,
  BMP280_REGISTER_DIG_P1 = 0x8E,
  BMP280_REGISTER_DIG_P2 = 0x90,
  BMP280_REGISTER_DIG_P3 = 0x92,
  BMP280_REGISTER_DIG_P4 = 0x94,
  BMP280_REGISTER_DIG_P5 = 0x96,
  BMP280_REGISTER_DIG_P6 = 0x98,
  BMP280_REGISTER_DIG_P7 = 0x9A,
  BMP280_REGISTER_DIG_P8 = 0x9C,
  BMP280_REGISTER_DIG_P9 = 0x9E,
  BMP280_REGISTER_CHIPID = 0xD0,
  BMP280_REGISTER_VERSION = 0xD1,
  BMP280_REGISTER_SOFTRESET = 0xE0,
  BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_REGISTER_STATUS = 0xF3,
  BMP280_REGISTER_CONTROL = 0xF4,
  BMP280_REGISTER_CONFIG = 0xF5,
  BMP280_REGISTER_PRESSUREDATA = 0xF7,
  BMP280_REGISTER_TEMPDATA = 0xFA,
};

#define RESET_VALUE 0xB6
#define BMP280_CHIPID 0x58

typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csBarPinBank;
	uint16_t 		   csBarPin;


	/* DMA */
	uint8_t readingBar;
	uint8_t barTxBuf[15];
	volatile uint8_t BarRxBuf[15];

	//calibration data.
	uint16_t dig_T1; /**< dig_T1 cal register. */
	int16_t dig_T2;  /**<  dig_T2 cal register. */
	int16_t dig_T3;  /**< dig_T3 cal register. */

	uint16_t dig_P1; /**< dig_P1 cal register. */
	int16_t dig_P2;  /**< dig_P2 cal register. */
	int16_t dig_P3;  /**< dig_P3 cal register. */
	int16_t dig_P4;  /**< dig_P4 cal register. */
	int16_t dig_P5;  /**< dig_P5 cal register. */
	int16_t dig_P6;  /**< dig_P6 cal register. */
	int16_t dig_P7;  /**< dig_P7 cal register. */
	int16_t dig_P8;  /**< dig_P8 cal register. */
	int16_t dig_P9;  /**< dig_P9 cal register. */


	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float barConversion;
	float tempConversion;

	/* x-y-z measurements */
	float bar_mps2;
	float imu_temp;

} BMP280;


/*
 *
 * INITIALISATION
 *
 */
uint8_t BMP580_Init(BMP280 *bar,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csBarPinBank, uint16_t csBarPin);

uint8_t BMP280_ReadBarRegister(BMP280 *imu, uint8_t regAddr, uint8_t *data);
uint8_t BMP280_ReadBarRegisterU16(BMP280 *imu, uint8_t regAddr, uint16_t *data);
uint8_t BMP280_WriteBarRegister(BMP280 *imu, uint8_t regAddr, uint8_t data);

#endif /* SRC_BMP280_H_ */
