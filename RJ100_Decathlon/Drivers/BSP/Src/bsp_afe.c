/*
 * bsp_afe.c
 *
 *  Created on: 25-Apr-2019
 *      Author: Inertial Elements
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpt_max30110.h"
#include "bsp_afe.h"

int32_t g_LEDSample[32]; /**< This variable array stores the value of the LED read from fifo*/
uint8_t *g_fifoData[96]; /**< This variable array stores the value of each fifo data byte*/


/**
 * @brief Function for setting different configuration parameter of the FIFO of max30110.
 * @remark The parameters are set so that the FIFO full interrupt is triggered when the FIFO has 15 samples space left.
 * Roll over parameter is also set. Also the Fifo interrupt will be cleared only when the fifo data register
 * value is read.
 * @see cpt_max30110.h
 *
 */


#define FIFO_CLEAR 0<<5
#define PHOTODIODE_RANGE 0x01<<6
#define SET_AFULL15 0x0F
#define ROLL_OVER_AT_FULL true<<4
#define CLR_INTRPT_AT_REGISTER_READ 0<<6
#define SAMPLING_AT100_DUAL_PULSE  0x15<<2
#define INTEGRATION_417US 0X03
#define LED_DELAY_TIME_20MS 0x03<<3
#define AVG_4SAMPLE 0x02
#define CURRENT_4MA 0x0A
#define MAXCURRENT_100MA_1 0x01
#define MAXCURRENT_100MA_2 0x01<<2



static void bsp_afe_SetFIFOConfiguration(void)
{
	/**Initialize the SPI communication
	 */
	cpt_afe_Init(MAX30110_SPI_MISO_PIN, MAX30110_SPI_MOSI_PIN, MAX30110_SPI_SCL_PIN,MAX30110_SPI_CS_PIN);
	/**A_FULL set as 15 so there can be only 17 samples before interrupt
	 */
	cpt_afe_SetFifoFreeSpaceAtInterrupt(SET_AFULL15);
	/** Allow roll over even if the FIFO is full
	 */
	cpt_afe_SetFifoRollOver(ROLL_OVER_AT_FULL);
	/**Do not clear the interrupt until some action like reading the read register has been taken
	 */
	cpt_afe_SetFifoFullType(FIFO_CLEAR);
	cpt_afe_SetFifoFullInterruptClear(CLR_INTRPT_AT_REGISTER_READ);
	/**Read in dual pulse mode so LED1 data and LED 2 data and the ambient light data will be pushed in FIFO
	 */
	cpt_afe_SetFifoDataCntrlFD1(LED_1);
	cpt_afe_SetFifoDataCntrlFD2(LED_2);
	/**Ambient light is read to filter out the ambient light noise
	 */
	cpt_afe_SetFifoDataCntrlFD4(DIRECT_AMBIENT);
	/**enable the data to be pushed to fifo
	 */
	cpt_afe_SetFIFOenable();
}

/**
 * @brief Function for setting different configuration parameter of the LED driver of max30110.
 * @remark The LED parameters are so chosen such that minimum power is utilized. The sampling rate is set at 100 Hz.
 * Dual pulse mode is selected. However one can alternate these parameters based on performance and power.
 * 19 bit ADC resolution is selected.The LED settling time is also set at maximum. This is so chosen so that
 * we don't miss out on any sample.
 * @see cpt_max30110.h
 *
 */
static void bsp_afe_SetLEDConfiguration(void)
{
	/**Range of the photodiode ADC is set in this function
	 */
	cpt_afe_SetADCRangeofPhotodiode(PHOTODIODE_RANGE);
	/**Set the sampling rate at 100 Hz in dual pulse mode operation
	 */
	cpt_afe_SetSamplingRateHRM(SAMPLING_AT100_DUAL_PULSE);
	/**Set the integration time a 417 us and the ADC operate at the maximum 19 bit resolution mode
	 */
	cpt_afe_SetIntegrationTime(INTEGRATION_417US );
	/**Set the Delay time for settling of the LEDs at 20 ms
	 */
	cpt_afe_SetLEDDelay(LED_DELAY_TIME_20MS);
	/**The average window at 4 samples
	 */
	cpt_afe_SetLEDAverage(AVG_4SAMPLE);
	/**Set both LEDs pulse Amplitude at 4mA.
	 */
	cpt_afe_SetLED1PA(CURRENT_4MA);
	cpt_afe_SetLED1PA(CURRENT_4MA);
	/**Set the LED range max current at 100 mA
	 */
	cpt_afe_SetLED1Range(MAXCURRENT_100MA_1);
	cpt_afe_SetLED2Range(MAXCURRENT_100MA_2);

}

/**
 * @brief Function for reading the fifo data register of max30110.
 * @remark The LED parameters are so chosen such that minimum power is utilized. The sampling rate is set at 100 Hz.
 * Dual pulse mode is selected. However one can alternate these parameters based on performance and power.
 * 19 bit ADC resolution is selected.The LED settling time is also set at maximum. This is so chosen so that
 * we don't miss out on any sample.
 * @see cpt_max30110.h
 *
 */

void bsp_afe_ReadFIFO(void)
{
	uint8_t fifoWritePointer=0;
	uint8_t fifoReadPointer=0;

	fifoWritePointer= cpt_afe_GetFifoWritePointer();
	fifoReadPointer = cpt_afe_GetFifoReadPointer();

	uint8_t lengthDataAvailable=0;
	if( fifoWritePointer>fifoReadPointer)
	{
		lengthDataAvailable = fifoWritePointer - fifoReadPointer;
	}
	else if (fifoWritePointer<fifoReadPointer)
	{
		lengthDataAvailable = (0x1f+fifoWritePointer-fifoReadPointer);
	}

	cpt_afe_ReadRegister(FIFO_DATA_REGISTER, *g_fifoData ,lengthDataAvailable);

	for(uint8_t i=0;i<32;i++)
	{
		g_LEDSample[i] = (((int32_t)g_fifoData[i*3]<<13)|((int32_t)g_fifoData[i*3+1]<<8)|((int32_t)g_fifoData[i*3+2]));

	}

}

void bsp_afe_GetData(void)
{
	bsp_afe_SetLEDConfiguration();
	bsp_afe_SetFIFOConfiguration();
	bsp_afe_ReadFIFO();
}
