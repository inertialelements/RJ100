/*
 * cpt_MAX30110.c
 *
 *  Created on: 20-Apr-2019
 *      Author: Inertial Elements
 *      Version: 1.0
 *       @brief: This file contains the function s for initializing, reading and writing to the
 *       HRM sensor via spi communication.
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../Inc/cpt_max30110.h"
#include "hal_drv_spi.h"
#include "app_util_platform.h"


#define singlebyte 1

static const nrf_drv_spi_t m_SpiInstance = NRF_DRV_SPI_INSTANCE(1);
volatile static bool g_SpiTxDone = false;

void cpt_afe_SpiEventHandler(nrf_drv_spi_evt_t const * p_context)
{

	g_SpiTxDone = true;
}

/**
 * @brief SPI initialization.
 * Just the usual way. Nothing special here
 */
uint32_t cpt_afe_Init(uint8_t sckPin,uint8_t mosiPin,uint8_t misoPin,uint8_t csPin)
{

    const nrf_drv_spi_config_t spi_afe_config = {                                                            \
        .sck_pin      = sckPin,
        .mosi_pin     = mosiPin,
        .miso_pin     = misoPin,
        .ss_pin       = csPin,
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_1M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    return nrf_drv_spi_init(&m_SpiInstance, &spi_afe_config, cpt_afe_SpiEventHandler);
}


/**@brief Function to merge a register and a buffer of data
 */
static void cpt_afe_MergeRegisterAndData(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint8_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}


/**@brief Function to write a series of bytes. The function merges
 * the register and the data.
 */
uint32_t cpt_afe_WriteRegisters(uint8_t reg, uint8_t * pData, uint8_t length)
{
    uint32_t errCode;
    uint8_t spiTxBuffer[MAX30110_SPI_BUFFER_SIZE];

    if(length > MAX30110_SPI_BUFFER_SIZE - 1) // Must be space for register byte in buffer
    {
        return NRF_ERROR_DATA_SIZE;
    }

    uint32_t timeout = MAX30110_SPI_TIMEOUT;
    // Add write bit to register.
    reg = reg | MAX30110_SPI_WRITE_BIT;

    cpt_afe_MergeRegisterAndData(spiTxBuffer, reg, pData, length + 1);

    errCode = nrf_drv_spi_transfer(&m_SpiInstance, spiTxBuffer, length + 1, NULL, 0);
    if(errCode != NRF_SUCCESS)
    	return errCode;


    while((!g_SpiTxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    g_SpiTxDone = false;

    return errCode;
}

uint32_t cpt_afe_WriteSingleRegister(uint8_t reg, uint8_t data)
{
    uint32_t errCode;
    uint32_t timeout = MAX30110_SPI_TIMEOUT;
    uint8_t writeCmd = MAX30110_SPI_WRITE_BIT;
    uint8_t packet[3] = {reg, writeCmd , data};

    // Add write bit to register.

    errCode = nrf_drv_spi_transfer(&m_SpiInstance, packet, 3, NULL, 0);
    if(errCode != NRF_SUCCESS) return errCode;

    while((!g_SpiTxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    g_SpiTxDone = false;

    return errCode;
}

uint32_t cpt_afe_ReadRegister(uint8_t reg, uint8_t * p_data, uint8_t length)
{
    uint32_t errCode;
    uint32_t timeout = MAX30110_SPI_TIMEOUT;

    uint8_t spiRxBuffer[MAX30110_SPI_BUFFER_SIZE];

    /// Add read bit to register to read .

    uint8_t read_cmd = MAX30110_SPI_READ_BIT ;
    uint8_t packet[2] = {reg, read_cmd};

    /// Read data over SPI and store incoming data in spi_rx_buffer

    errCode = nrf_drv_spi_transfer(&m_SpiInstance, packet, 2, spiRxBuffer, length + 1); // Length + 1 because register byte has to be clocked out before MAX30110 returns data of length 'length'
    if(errCode != NRF_SUCCESS) return errCode;

    while((!g_SpiTxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    g_SpiTxDone = false;

	/// Copy data from spi_rx_buffer over to p_data

	memcpy(p_data, &spiRxBuffer[1], sizeof(p_data));

    return NRF_SUCCESS;
}

//////////////////Utility function for AFE/////////////////

/**
 * @brief Read the Part ID of the Device
 */
uint8_t cpt_afe_GetPartID(void)
{
	uint8_t * partID=0;
	uint8_t value0;
	cpt_afe_ReadRegister(PART_ID, partID,singlebyte);
	value0 = * partID;
	return value0;
}

/**
 * @Read interrupt status 1 from the AFE
 */
uint8_t cpt_afe_GetInterruptStatus_1(void)
{
	uint8_t * int_status1=0;
	uint8_t value1;
	cpt_afe_ReadRegister(INTERRUPT_STATUS_1, int_status1,singlebyte);
	value1 = * int_status1;
	return value1;
}


/*
 * @brief Read interrupt status 2 from the AFE
 */
uint8_t cpt_afe_GetInterruptStatus_2(void)
{
	uint8_t *int_status2=0;
	uint8_t value2;
	cpt_afe_ReadRegister(INTERRUPT_STATUS_2, int_status2,singlebyte);
	value2 = * int_status2;
	return value2 ;
}

/*
 * @brief Set the Interrupt Enable 1 register of the AFE
 */
void cpt_afe_SetInterrupt_1(uint8_t enInterrupt)
{
	cpt_afe_WriteSingleRegister(INTERRUPT_ENABLE_1, enInterrupt);
}

/**
 * @brief Set the FIFO Write Pointer of the AFE
 */
void cpt_afe_SetFifoWritePointer(uint8_t pointerAddress)
{
	uint8_t mask =0x1F;
	pointerAddress = pointerAddress & mask ;
	cpt_afe_WriteSingleRegister(FIFO_WRITE_PTR, pointerAddress);
}

/**
 * @brief Set the FIFO Read Pointer of the AFE
 */
void cpt_afe_SetFifoReadPointer(uint8_t pointerAddress)
{
	uint8_t mask =0x1F;
	pointerAddress = pointerAddress & mask ;
	cpt_afe_WriteSingleRegister(FIFO_READ_PTR, pointerAddress);
}

/**
 * @brief Read the FIFO Write Pointer of the AFE
 */
uint8_t cpt_afe_GetFifoWritePointer(void)
{
	uint8_t *writepointerAddress=0;
	uint8_t value2;
	cpt_afe_ReadRegister(FIFO_WRITE_PTR,writepointerAddress,singlebyte );
	value2=*writepointerAddress;
	return value2;
}

/**
 * @brief Read the FIFO Read Pointer of the AFE
 */
uint8_t cpt_afe_GetFifoReadPointer(void)
{
	uint8_t *readpointerAddress=0;
	uint8_t value2;
	cpt_afe_ReadRegister(FIFO_WRITE_PTR,readpointerAddress,singlebyte);
	value2=*readpointerAddress;
	return value2;
}

/**
 * @brief Read the number of overflow bytes i.e in case of FIFO overflow
 */
uint8_t cpt_afe_GetOverflowRegister(void)
{
	uint8_t * overflowNrBytes=0;
	uint8_t value4;
	cpt_afe_ReadRegister(PART_ID, overflowNrBytes,singlebyte);
	value4 = *overflowNrBytes;
	return value4;
}

/**
 * @brief Set the amount of free spaces in FIFO when Interrupt Occurs
 */
void cpt_afe_SetFifoFreeSpaceAtInterrupt(uint8_t freeFifoSpace)
{
	const uint8_t mask = 0xF0;
	uint8_t value = 0;
	value &= mask;
	value |= freeFifoSpace;
	cpt_afe_WriteSingleRegister(FIFO_CONFIGURATION, value);
}

/**
 * @brief Set roll over of FIFO in case all the FIFO registers are full with 32 samples
 */
void cpt_afe_SetFifoRollOver(bool rollOver)
{
	const uint8_t mask = 0xEF;
	uint8_t value =0;
	value &= mask;
	value |= rollOver;
	cpt_afe_WriteSingleRegister(FIFO_CONFIGURATION, value);
}

/*
 * @brief Set whether the interrupt bit will be set at every sample that is pushed after the first interrupt
 * or just once i.e for the first time when the interrupt has taken place.
*/
void cpt_afe_SetFifoFullType(uint8_t fullType)
{
	const uint8_t mask = 0xDF;
	uint8_t value =0;
	value &= mask;
	value |= fullType;
	cpt_afe_WriteSingleRegister(FIFO_CONFIGURATION, value);
}
/*
 * @brief Set whether the interrupt bit will be cleared only by reading the status register or by reading the FIFO data
*/

void cpt_afe_SetFifoFullInterruptClear(uint8_t clearInterrupt)
{
	const uint8_t mask = 0xBF;
	uint8_t value =0;
	value &= mask;
	value |= clearInterrupt;
	cpt_afe_WriteSingleRegister(FIFO_CONFIGURATION, value);
}


/*
 * @brief Set the fifo data control which you want to read FD1 (most probably LED1)
*/

void cpt_afe_SetFifoDataCntrlFD1(uint8_t fd1)
{
	const uint8_t mask = 0xF0;
	uint8_t value =0;
	value &= mask;
	value |= fd1;
	cpt_afe_WriteSingleRegister(FIFO_DATA_CNTRL1, value);
}


/*
 * @brief Set the fifo data control which you want to read FD2 (most probably LED2)
*/

void cpt_afe_SetFifoDataCntrlFD2(uint8_t fd2)
{
	const uint8_t mask = 0x0F;
	uint8_t value =0;
	value &= mask;
	value |= (fd2<<4);
	cpt_afe_WriteSingleRegister(FIFO_DATA_CNTRL1, value);
}

/*Set the fifo data control which you want to read FD3 (most probably ambient light)
*/

void cpt_afe_SetFifoDataCntrlFD3(uint8_t fd3)
{
	const uint8_t mask = 0xF0;
	uint8_t value =0;
	value &= mask;
	value |= fd3;
	cpt_afe_WriteSingleRegister(FIFO_DATA_CNTRL2, value);
}

/*Set the fifo data control which you want to read FD4 (free but pilot Led if available can be read)
*/

void cpt_afe_SetFifoDataCntrlFD4(uint8_t fd4)
{
	const uint8_t mask = 0x0F;
	uint8_t value = 0;
	value &= mask;
	value |= (fd4<<4);
	cpt_afe_WriteSingleRegister(FIFO_DATA_CNTRL2, value);
}

/*Enable the fifo so that the data can be pushed to the fifo register
*/
void cpt_afe_SetFIFOenable(void)
{
	//Reset the FIF0 en bit and then set it on

	const uint8_t mask = 0xFB;
	uint8_t value = 0;
	value &= mask;
	value |= 0x04;
	cpt_afe_WriteSingleRegister(SYS_CNTRL, value);
}


/*Reset the system
*/
void cpt_afe_Reset(void)
{
	//Reset the system

	const uint8_t mask = 0xFE;
	uint8_t value = 0;
	value &= mask;
	value |= 0x01;
	cpt_afe_WriteSingleRegister(SYS_CNTRL, value);
}

/*Set the ADC range of the photodiode sensor
*/
void cpt_afe_SetADCRangeofPhotodiode(uint8_t setADCPhotodiode)
{
	//Set the range of the photo diode

	const uint8_t mask = 0x3F;
	uint8_t value = 0;
	value &= mask;
	value |= setADCPhotodiode;
	cpt_afe_WriteSingleRegister(PPG_CONFIGURATION_1, value);
}

/*Set the Sampling rate of the LED pulse. Set it at 100 Hz or less for dual pulse mode
*/

void cpt_afe_SetSamplingRateHRM(uint8_t samplingRate)
{
	//Set the sampling rate of the LEDs pulses

	const uint8_t mask = 0xC3;
	uint8_t value = 0;
	value &= mask;
	value |= samplingRate;
	cpt_afe_WriteSingleRegister(PPG_CONFIGURATION_1, value);
}



/*Set the Integration Time of the the LED pulse. This determines the pulse width .
 * More the ADC resolution bigger is the integration time. For 19 bit resolution
 * the integration time should be set at 419 us.
*/

void cpt_afe_SetIntegrationTime(uint8_t intTime)
{
	//Set the sampling rate of the LEDs pulses

	const uint8_t mask = 0xFC;
	uint8_t value = 0;
	value &= mask;
	value |= intTime;
	cpt_afe_WriteSingleRegister(PPG_CONFIGURATION_1, value);
}


/*Set the LED Delay Settling Time. This is another factor that determines the pulse width .
 *
*/

void cpt_afe_SetLEDDelay(uint8_t settlingTime)
{
	//Set the settling time  of the LEDs pulses
	const uint8_t mask = 0xE7;
	uint8_t value = 0;
	value &= mask;
	value |= settlingTime;
	cpt_afe_WriteSingleRegister(PPG_CONFIGURATION_2, value);

}

/*Set the LED Internal Average Window. This factor  determines the final output rate of the samples
 * through  FIFO.
 *
*/
void cpt_afe_SetLEDAverage(uint8_t nosSamples)
{
	//Set the number of LED sample average
	const uint8_t mask = 0xF8;
	uint8_t value = 0;
	value &= mask;
	value |= nosSamples;
	cpt_afe_WriteSingleRegister(PPG_CONFIGURATION_2, value);
}

/*This function set the proximity threshold for interrupt generation. Here Each of the 8 bit correspond to
 * the 8 MSB bit of the ADC
*/

void cpt_afe_SetProxityThreshold(uint8_t resolutionBit)
{
	cpt_afe_WriteSingleRegister(PROX_INTERRUPT_THRESHOLD, resolutionBit);
}

/*This function set the LED1 nominal peak current pulse amplitude. This is very important in terms of
 * saving power and performance
*/

void cpt_afe_SetLED1PA(uint8_t pa1)
{
	cpt_afe_WriteSingleRegister(LED_PA_1, pa1);
}

/*This function set the LED2 nominal peak current pulse amplitude. This is very important in terms of
 * saving power and performance
*/

void cpt_afe_SetLED2PA(uint8_t pa2)
{
	cpt_afe_WriteSingleRegister(LED_PA_2, pa2);
}

/*This function set the LED1 nominal peak current pulse amplitude in the proximity detection mode.
 * This is very important in terms of saving power and performance
*/
void cpt_afe_SetPilotLEDPA(uint8_t paP)
{
	cpt_afe_WriteSingleRegister(LED_PILOT_PA, paP);
}

/*This function set the LED1 current range. There are four current ranges viz. 50 mA, 100 mA, 150 mA, 200 mA
*/
void cpt_afe_SetLED1Range(uint8_t range1)
{
	const uint8_t mask = 0xFC;
	uint8_t value = 0;
	value &= mask;
	value |= range1;
	cpt_afe_WriteSingleRegister(LED_RANGE, range1);
}

/*This function set the LED2 current range. There are four current ranges viz. 50 mA, 100 mA, 150 mA, 200 mA
*/
void cpt_afe_SetLED2Range(uint8_t range2)
{
	const uint8_t mask = 0xF3;
	uint8_t value = 0;
	value &= mask;
	value |= range2;
	cpt_afe_WriteSingleRegister(LED_RANGE, range2);
}


