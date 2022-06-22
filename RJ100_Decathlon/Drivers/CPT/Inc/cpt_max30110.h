/*
 * cpt_MAX30110.h
 *
 *  Created on: 20-Apr-2019
 *      Author: Inertial Elements
 */
//@brief	This file drives the MAX30110 heart rate monitor sensor

#ifndef DRIVERS_CPT_INC_CPT_MAX30110_H_
#define DRIVERS_CPT_INC_CPT_MAX30110_H_


/* Pins to connect G4 ppg.
 */
#define MAX30110_SPI_MISO_PIN    20 /**< MAX30110 SDO. 'MIS0' on which the G4 ppg is connected*/
#define MAX30110_SPI_MOSI_PIN    19 /**< MAX30110 SDI. 'MOSI' on which the G4 ppg is connected*/
#define MAX30110_SPI_SCL_PIN     18  /**< MAX30110 SCLK. 'SCL' on which the G4 ppg is connected*/
#define MAX30110_SPI_CS_PIN      17 /**< MAX30110 nCS. 'NCS' on which the G4 ppg is connected*/


#define MAX30110_SPI_BUFFER_SIZE     96 /**< 96 bytes from FIFO byte in one transmission.*/
#define MAX30110_SPI_WRITE_BIT       0x00
#define MAX30110_SPI_READ_BIT        0x80
#define MAX30110_SPI_TIMEOUT         10000


/**
 * \brief AFE Internal Registers
 */
typedef enum
{
	//Interrupts Register
	INTERRUPT_STATUS_1 = 0x00,
	INTERRUPT_STATUS_2 = 0x01,
	INTERRUPT_ENABLE_1=  0X02,
	INTERRUPT_ENABLE_2 = 0X03,
	// FIFO Registers
	FIFO_WRITE_PTR = 0X04,
	OVERFLOW_COUNTER = 0X05,
	FIFO_READ_PTR = 0X06,
	FIFO_DATA_REGISTER = 0X07,
	FIFO_CONFIGURATION = 0X08,
	// FIFO Data Control
	FIFO_DATA_CNTRL1 = 0X09,
	FIFO_DATA_CNTRL2 = 0X0A,
	//System Control
	SYS_CNTRL= 0x0D,
	///PPG Configuration
	PPG_CONFIGURATION_1 = 0x0E,
	PPG_CONFIGURATION_2 = 0x0F,
	PROX_INTERRUPT_THRESHOLD = 0x10,
	//LED Pulse Amplitude
	LED_PA_1 = 0x11,
	LED_PA_2 = 0x12,
	LED_RANGE = 0X14,
	LED_PILOT_PA = 0X15,
	//PART ID
	PART_ID = 0xFF,

}AfeRegister;


typedef enum
{
	LED_1 = 0x01,
	LED_2 = 0x02,
	PILOT_LED1 = 0x05,
	DIRECT_AMBIENT = 0x0C,
	LED1ndLED2 = 0x0D,
}afeDataControl;

typedef enum
{
	//Single Pulse mode
	S_20Hz = (0x00<<2),
	S_25Hz = (0x01<<2),
	S_50Hz = (0x02<<2),
	S_84Hz = (0x03<<2),
	S_100Hz =  (0x04<<2),
	S_200Hz =  (0x05<<2),
	S_400Hz =  (0x06<<2),
	S_800Hz =  (0x07<<2),
	S_1000Hz = (0x08<<2),
	S_1600Hz = (0x09<<2),
	S_3200Hz = (0x0A<<2),
	//Dual Pulse mode
	D_20Hz =  (0x0B<<2),
	D_25Hz =  (0x0C<<2),
	D_50Hz =  (0x0D<<2),
	D_84Hz =  (0x0E<<2),
	D_100Hz = (0x0E<<2),

}afeSamplingOder;


/**
 * @brief Function for initializing the MAX30110 SPI communication.
 * @param[in] sckPin  This is the clock pin of the SPI communication
 * @param[in] mosiPin This is the pin where the Master out slave in pin is connected
 * @param[in] misoPin This is the pin where the Master in slave out pin is connected
 * @param[in] csPin   This is the chip select pin of the SPI communication
 *
 * @retval NRF_SUCCESS             If the spi communication between the Controller and the AFE is successful.
 */
extern uint32_t cpt_afe_Init(uint8_t sckPin,uint8_t mosiPin,uint8_t misoPin,uint8_t csPin);

/**
 * @brief Function for Writing into the AFE register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] pdata     The value of the register is stored in the data variable
 *
 * @retval NRF_SUCCESS             If data write was successful.
 */
extern uint32_t cpt_afe_WriteRegisters(uint8_t reg, uint8_t * pData, uint8_t length);

/**
 * @brief Function for Writing into the AFE register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] pdata     The value of the register is stored in the data variable
 *
 * @retval NRF_SUCCESS             If data write was successful.
 */
extern uint32_t cpt_afe_WriteSingleRegister(uint8_t reg, uint8_t data);

/**
 * @brief Function for reading AFE register
 *
 * @param[in] reg      	Address of the register is stored here.
 * @param[in] p_data    The value of the register is stored in the data variable
 * @param[in] length    Length of the data that will be stored n the
 *
 * @retval The value stored in the register whose address is the input
 * @retval NRF_SUCCESS             If transfer was successful.
 */
uint32_t cpt_afe_ReadRegister(uint8_t reg, uint8_t * p_data, uint8_t length);

/**
 * @brief Function for Reading the part id of the AFE just like reading the Who Am I registers in accelerometers .
 *
 * @retval Part ID of the AFE
 *
 */
extern uint8_t cpt_afe_GetPartID(void);

/**
 * @brief Function for Reading the interrupt 1 status .
 *
 * @retval The status of the interrupt 1
 */
extern uint8_t cpt_afe_GetInterruptStatus_1(void);

/**
 * @brief Function for Reading the interrupt 2 status .
 *
 * @retval The status of the interrupt 2
 */
extern uint8_t cpt_afe_GetInterruptStatus_2(void);

/**
 * @brief Function for enabling different settings of the Afe interrupt 1.
 *
 * @param[in] enInterrupt      Value that is written on the Interrupt 1 Enable Register for different settings .
 *
 */
extern void cpt_afe_SetInterrupt_1(uint8_t enInterrupt);
/**
 * @brief Function for setting the FIFO write pointer .
 *
 * @param[in] pointerAddress      Value of the FIFO write pointer Address.
 *
 */
extern void cpt_afe_SetFifoWritePointer(uint8_t pointerAddress);
/**
 * @brief Function for setting the FIFO read pointer.
 *
 * @param[in] pointerAddress    Value of the FIFO write pointer Address.
 *
 */
extern void cpt_afe_SetFifoReadPointer(uint8_t pointerAddress);
/**
 * Read the number of overflow bytes i.e in case of FIFO overflow
 */
extern uint8_t cpt_afe_GetOverflowRegister(void);

/**
 * @brief Read the FIFO Read Pointer of the AFE
 *
 * @retval The value of the FiFo read pointer
 */
uint8_t cpt_afe_GetFifoReadPointer(void);

/**
 * @brief Read the FIFO Write Pointer of the AFE
 * @retval The value of the FiFo write pointer
 */
uint8_t cpt_afe_GetFifoWritePointer(void);

///Set the amount of free spaces in FIFO when Interrupt Occurs
extern void cpt_afe_SetFifoFreeSpaceAtInterrupt(uint8_t freeFifoSpace);

///Set roll over of FIFO in case all the FIFO registers are full with 32 samples
extern void cpt_afe_SetFifoRollOver(bool rollOver);

/*Set whether the interrupt bit will be set at every sample that is pushed after the first interrupt
 * or just once i.e for the first time when the interrupt has taken place.
*/
extern void cpt_afe_SetFifoFullType(uint8_t fullType);

/*Set whether the interrupt bit will be cleared only by reading the status register or by reading the FIFO data
*/

extern void cpt_afe_SetFifoFullInterruptClear(uint8_t clearInterrupt);

/*Set the fifo data control which you want to read FD1 (most probably LED1)
*/

extern void cpt_afe_SetFifoDataCntrlFD1(uint8_t fd1);

/*Set the fifo data control which you want to read FD2 (most probably LED2)
*/

extern void cpt_afe_SetFifoDataCntrlFD2(uint8_t fd2);

/*Set the fifo data control which you want to read FD3 (most probably ambient light)
*/

extern void cpt_afe_SetFifoDataCntrlFD3(uint8_t fd3);

/*Set the fifo data control which you want to read FD4 (free but pilot Led if available can be read)
*/

extern void cpt_afe_SetFifoDataCntrlFD4(uint8_t fd4);

/*Enable the fifo so that the data can be pushed to the fifo register
*/
extern void cpt_afe_SetFIFOenable(void);

/*Reset the system
*/
extern void cpt_afe_Reset(void);

/*Set the ADC range of the photodiode sensor
*/
extern void cpt_afe_SetADCRangeofPhotodiode(uint8_t setADCPhotodiode);

/*Set the Sampling rate of the LED pulse. Set it at 100 Hz or less for dual pulse mode
*/

extern void cpt_afe_SetSamplingRateHRM(uint8_t samplingRate);

/*Set the Integration Time of the the LED pulse. This determines the pulse width .
 * More the ADC resolution bigger is the integration time. For 19 bit resolution
 * the integration time should be set at 419 us.
*/

extern void cpt_afe_SetIntegrationTime(uint8_t intTime);


/*Set the LED Delay Settling Time. This is another factor that determines the pulse width .
 *
*/

extern void cpt_afe_SetLEDDelay(uint8_t settlingTime);


/*Set the LED Internal Average Window. This factor  determines the final output rate of the samples
 * through  FIFO.
 *
*/
extern void cpt_afe_SetLEDAverage(uint8_t nosSamples);

/*This function set the proximity threshold for interrupt generation. Here Each of the 8 bit correspond to
 * the 8 MSB bit of the ADC
*/

extern void cpt_afe_SetProxityThreshold(uint8_t resolutionBit);


/*This function set the LED1 nominal peak current pulse amplitude. This is very important in terms of
 * saving power and performance
*/

extern void cpt_afe_SetLED1PA(uint8_t pa1);


/*This function set the LED2 nominal peak current pulse amplitude. This is very important in terms of
 * saving power and performance
*/

extern void cpt_afe_SetLED2PA(uint8_t pa2);

/*This function set the LED1 nominal peak current pulse amplitude in the proximity detection mode.
 * This is very important in terms of saving power and performance
*/
extern void cpt_afe_SetPilotLEDPA(uint8_t paP);

/*
 * @brief This function set the LED1 current range. There are four current ranges viz. 50 mA, 100 mA, 150 mA, 200 mA
 *
 * @param[in] range2 Set the current range of the LED1
 *
*/
extern void cpt_afe_SetLED1Range(uint8_t range1);

/*
 * @brief This function set the LED2 current range. There are four current ranges viz. 50 mA, 100 mA, 150 mA, 200 mA
 *
 * @param[in] range2 Set the current range of the LED2
*/
extern void cpt_afe_SetLED2Range(uint8_t range2);



#endif /* DRIVERS_CPT_INC_CPT_MAX30110_H_ */
