/*
 * MPU6000.c
 *
 *  Created on: Dec 31, 2020
 *      Author: HAOO
 */

#include "MPU6000.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t MPU6000_Init(MPU6000 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csAccPinBank, uint16_t csAccPin) {

	/* Store interface parameters in struct */
	imu->spiHandle 		= spiHandle;
	imu->csAccPinBank 	= csAccPinBank;
	imu->csAccPin 		= csAccPin;
	imu->csGyrPinBank	= csAccPinBank;
	imu->csGyrPin 		= csAccPin;
	imu->accConversion  = 9.81998f / 2048.0f;
	imu->gyrConversion  = 1.0f / 16.4f;
	imu->tempConversion = 1.0f / 340.0f;
	/* Clear DMA flags */
	imu->readingAcc = 0;
	imu->readingGyr = 0;
	/* Set gyroscope TX buffer for DMA */
	imu->gyrTxBuf[0] = MPU_RA_GYRO_XOUT_H | 0x80;

	/* Set accelerometer TX buffer for DMA */
	imu->accTxBuf[0] = MPU_RA_ACCEL_XOUT_H | 0x80;

	uint8_t status = 0;

	/*
	 *
	 * ACCELEROMETER
	 *
	 */

	/* Accelerometer requires rising edge on CSB at start-up to activate SPI */
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
	HAL_Delay(50);


	// reset the device configuration
	status += MPU6000_WriteAccRegister(imu, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	HAL_Delay(100);

    // reset the device signal paths
	status += MPU6000_WriteAccRegister(imu, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	HAL_Delay(100);  // datasheet specifies a 100ms delay after signal path reset

	/* Check chip ID */
	uint8_t chipID;
	status += MPU6000_ReadAccRegister(imu, MPU_RA_WHO_AM_I, &chipID);

	if (chipID != 0x68) {

	//	return 0;

	}
	HAL_Delay(10);

	/* Configure imu  */

	// Clock Source PPL with Z axis gyro reference
	status += MPU6000_WriteAccRegister(imu, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	HAL_Delay(15);

    // Disable Primary I2C Interface
	status += MPU6000_WriteAccRegister(imu, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    HAL_Delay(15);

    status += MPU6000_WriteAccRegister(imu, MPU_RA_PWR_MGMT_2, 0x00);
    HAL_Delay(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    status += MPU6000_WriteAccRegister(imu, MPU_RA_SMPLRT_DIV, 0x00);
    HAL_Delay(15);

    // Gyro +/- 2000 DPS Full Scale
    status += MPU6000_WriteAccRegister(imu, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    HAL_Delay(15);

    // Accel +/- 16 G Full Scale
    status += MPU6000_WriteAccRegister(imu, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    HAL_Delay(15);

    status += MPU6000_WriteAccRegister(imu, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    HAL_Delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    status += MPU6000_WriteAccRegister(imu, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    HAL_Delay(15);
#endif



	return status;

}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t MPU6000_ReadAccRegister(MPU6000 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80, 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[1];

	}

	return status;

}

uint8_t MPU6000_ReadGyrRegister(MPU6000 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80, 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[1];

	}

	return status;

}

uint8_t MPU6000_WriteAccRegister(MPU6000 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	return status;

}

uint8_t MPU6000_WriteGyrRegister(MPU6000 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	return status;

}



/*
 *
 * POLLING
 *
 */
uint8_t MPU6000_ReadAccelerometer(MPU6000 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[7] = {(MPU_RA_ACCEL_XOUT_H | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
	uint8_t rxBuf[7];
	char TxBuf[50];
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[1] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[3] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[5] << 8) | rxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	sprintf(TxBuf, "%f\n", imu->acc_mps2[2]);
	CDC_Transmit_FS((uint8_t *) TxBuf, strlen(TxBuf));

	return status;

}

uint8_t MPU6000_ReadGyroscope(MPU6000 *imu) {

	/* Read raw gyroscope data */
	uint8_t txBuf[7] = {(MPU_RA_GYRO_XOUT_H | 0x80), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];
	char TxBuf[30];
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t gyrX = (int16_t) ((rxBuf[1] << 8) | rxBuf[2]);
	int16_t gyrY = (int16_t) ((rxBuf[3] << 8) | rxBuf[5]);
	int16_t gyrZ = (int16_t) ((rxBuf[5] << 8) | rxBuf[6]);

	/* Convert to rad/s */
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

	sprintf(TxBuf, "%f,%f,%f\n", imu->gyr_rps[0],imu->gyr_rps[1],imu->gyr_rps[2]);
	CDC_Transmit_FS((uint8_t *) TxBuf, strlen(TxBuf));

	return status;

}

uint8_t MPU6000_ReadTemperature(MPU6000 *imu) {

	/* Read raw gyroscope data */
	uint8_t txBuf[3] = {(MPU_RA_TEMP_OUT_H | 0x80), 0xFF, 0xFF}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[3];
	char TxBuf[30];
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t temp = (int16_t) ((rxBuf[1] << 8) | rxBuf[2]);

	/* Convert to rad/s */
	imu->imu_temp = (imu->tempConversion * temp) + 35;


	sprintf(TxBuf, "%u,%u,%f\n",rxBuf[1],rxBuf[2],imu->imu_temp);
	CDC_Transmit_FS((uint8_t *) TxBuf, strlen(TxBuf));

	return status;

}

/*
 *
 * DMA
 *
 */
uint8_t MPU6000_ReadDataDMA(MPU6000 *imu) {

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->accTxBuf, (uint8_t *) imu->accRxBuf, 15) == HAL_OK) {

		imu->readingAcc = 1;
		return 1;

	} else {

		HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
		return 0;

	}

}

void MPU6000_ReadDataDMA_Complete(MPU6000 *imu) {


	uint8_t status = 0;
	MPU6000_ReadAccRegister(imu, MPU_RA_INT_STATUS, &status);

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
	imu->readingAcc = 0;

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((imu->accRxBuf[1] << 8) | imu->accRxBuf[2]);
	int16_t accY = (int16_t) ((imu->accRxBuf[3] << 8) | imu->accRxBuf[4]);
	int16_t accZ = (int16_t) ((imu->accRxBuf[5] << 8) | imu->accRxBuf[6]);

	/* Form signed 16-bit integers */
	int16_t temp = (int16_t) ((imu->accRxBuf[7] << 8) | imu->accRxBuf[8]);

	/* Form signed 16-bit integers */
	int16_t gyrX = (int16_t) ((imu->accRxBuf[9] << 8) | imu->accRxBuf[10]);
	int16_t gyrY = (int16_t) ((imu->accRxBuf[11] << 8) | imu->accRxBuf[12]);
	int16_t gyrZ = (int16_t) ((imu->accRxBuf[13] << 8) | imu->accRxBuf[14]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	imu->imu_temp = (imu->tempConversion * temp) + 35;

	/* Convert to deg/s */
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

}

