/*
 * BMP280.c
 *
 *  Created on: Jan 5, 2021
 *      Author: HAOO
 */

#include "BMP280.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMP580_Init(BMP280 *bar,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csBarPinBank, uint16_t csBarPin){

	/* Store interface parameters in struct */
	bar->spiHandle 		= spiHandle;
	bar->csBarPinBank 	= csBarPinBank;
	bar->csBarPin 		= csBarPin;

	bar->barConversion  = 9.81998f / 2048.0f;
	bar->tempConversion = 1.0f / 340.0f;
	/* Clear DMA flags */
	bar->readingBar = 0;

	/* Set accelerometer TX buffer for DMA */
	//bar->accTxBuf[0] = MPU_RA_ACCEL_XOUT_H | 0x80;

	uint8_t status = 0;

	/* Barometer requires rising edge on CSB at start-up to activate SPI */
	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_SET);
	HAL_Delay(50);



	// reset the device configuration
	status += BMP280_WriteBarRegister(bar, BMP280_REGISTER_SOFTRESET, RESET_VALUE);
	HAL_Delay(100);

	/* Check chip ID */
	uint8_t chipID;
	status += BMP280_ReadBarRegister(bar, BMP280_REGISTER_CHIPID, &chipID);

	if(chipID != BMP280_CHIPID)
	{
		return 0;
	}

/*	BMP280_ReadBarRegisterU16(bar, BMP280_REGISTER_DIG_T1, bar->dig_T1);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_T2, bar->dig_T2);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_T3, bar->dig_T3);

	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P1, bar->dig_P1);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P2, bar->dig_P2);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P3, bar->dig_P3);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P4, bar->dig_P4);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P5, bar->dig_P5);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P6, bar->dig_P6);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P7, bar->dig_P7);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P8, bar->dig_P8);
	BMP280_ReadBarRegister(bar, BMP280_REGISTER_DIG_P9, bar->dig_P9);*/

	return status;
}

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMP280_ReadBarRegister(BMP280 *bar, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr & 0x80, 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(bar->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[1];

	}

	return status;

}

uint8_t BMP280_ReadBarCalibration(BMP280 *bar) {

	uint8_t txBuf[25] = {BMP280_REGISTER_DIG_T1 & 0x80, 0x00, 0x00};
	uint8_t rxBuf[25];

	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(bar->spiHandle, txBuf, rxBuf, 25, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_SET);

	if (status == 1) {

		bar->dig_T1 = (uint16_t) ((rxBuf[1] << 8) | rxBuf[2]);
		bar->dig_T2 = (int16_t) ((rxBuf[2] << 8) | rxBuf[3]);
		bar->dig_T3 = (int16_t) ((rxBuf[4] << 8) | rxBuf[5]);
	}

	return status;

}

uint8_t BMP280_WriteBarRegister(BMP280 *bar, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(bar->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(bar->csBarPinBank, bar->csBarPin, GPIO_PIN_SET);

	return status;

}
