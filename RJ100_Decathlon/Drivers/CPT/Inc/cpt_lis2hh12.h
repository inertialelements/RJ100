/**
 * @file	cpt_lis2hh12.h
 * @brief	This file drives the LIS2HH12 acccelerometer
 * @defgroup	Changes in the LIS2HH12 cpt files for nrf51 controller & driver
 * @{
 */

#ifndef CPT_LIS2HH12_H_
#define CPT_LIS2HH12_H_

#include "stdint.h"

#ifdef LIS2DH12_I2C
    #include "nrf_drv_twi.h"
#endif

//#ifdef LIS2DH12_SPI
//    #include "drv_spi.h"
//#endif


/* Pins to connect LIS2HH12.
 */
#define LIS2HH12_I2C_SCL_PIN 22
#define LIS2HH12_I2C_SDA_PIN 23


#define LIS2HH12_I2C_BUFFER_SIZE     	6 // 12 byte buffers will suffice to read accelerometer.
#define LIS2HH12_I2C_TIMEOUT 			10000
#define LIS2HH12_ADDRESS     			0x1E

//
#define TWI_INSTANCE_ID     0


//Functions

/**
 * @brief Function for Writing into the accelerometer register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] data     The value of the register is stored in the data variable
 *
 * @retval NRF_SUCCESS             If data write was successful.
 */

extern uint32_t cpt_lis2hh12_WriteSingleRegister(uint8_t reg, uint8_t data);

/**
 * @brief Function for Writing into multiple accelerometer register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] pData    The value of the register is stored in the data variable
 * @param[in] length   Data length of the input parameter data
 *
 * @retval NRF_SUCCESS             If transfer was successful.
 */

extern uint32_t cpt_lis2hh12_WriteRegisters(uint8_t reg, uint8_t * pData, uint32_t length);


/**
 * @brief Function for reading accelerometer register
 *
 * @param[in] reg      	Address of the register is stored here.
 * @param[in] p_data    The value of the register is stored in the data variable
 * @param[in] length    Length of the data that will be stored n the
 *
 * @retval The value stored in the register whose address is the input
 * @retval NRF_SUCCESS             If transfer was successful.
 */

extern uint32_t cpt_lis2hh12_ReadRegisters(uint8_t reg, uint8_t * pData, uint8_t length);

/**
 * @brief Function to switch on the accelerometer
 * @details This function
 * @param[in] scl    Pin at which the SCL of the I2C is connected.
 * @param[in] sda    Pin at which the SDA of the I2C is connected.
 * @param[in] odr    The output sampling rate.
 * Keep the output sampling rate at 10 Hz as we need only to study the interrupt
 */

extern uint32_t cpt_accelerometer_On(uint8_t scl,uint8_t sda,uint8_t odr);
/**
 * @brief Function to read the accelrometer interrupt on GPIO pins
 *
 * @param[in] gpioIntPin    Pin at which the Interupt should be read.
 * @retval The gpio pin read
 *
 */

extern uint32_t cpt_gpio_AccInit(uint8_t gpioIntPin);


#endif /* CPT_LIS2DH12TR_H_ */

/** @} */
