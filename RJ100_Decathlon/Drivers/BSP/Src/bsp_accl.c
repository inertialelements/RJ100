/*
 * bsp_accl.c
 *
 *  Created on: 25-Apr-2019
 *      Author: Inertial Elements
 *
 *      @brief This file contains all the useful functions for reading setting the parameters of
 *      the accelerometer like sampling frequency, setting sda, scl pins, setting of the interrupt
 *      etc. Apart from setting the parameters it also contains function for reading accelerometer
 *      data.
 */


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpt_lis2hh12.h"
#include "bsp_accl.h"
#include "app_util_platform.h"
#include "cpt_lis2hh12_cfg.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"



#define interrupt 1
#define sixbytes 6
#define singlebyte 1

#define BURST_MODE 0x08
#define INTERRUPT_1_PIN1 0x08
#define INTERRUPT_2_PIN2 0x10
#define XACTTHRESHOLD_2G 0x9F
#define YACTTHRESHOLD_2G 0x9F
#define ZACTTHRESHOLD_2G 0xBF
#define DURACT_1S 0xC8
#define INACTTHRESHOLD_1G_ALLAXIS 0xBF
#define DURINACT_1S 0xD8
#define SET_ACTTHRES_ON1AXIS 0x60
#define SET_INACTTHRES_ON3AXIS 0xC3
/**Switch on the Accelerometer
 *
 */
void bsp_accl_SwitchonAccelo(void)
{
	// The SCL and the SDA pins are set and the Output data  rate is set at 10 Hz
	//uint32_t errCode;
	cpt_accelerometer_On(LIS2HH12_I2C_SCL_PIN, LIS2HH12_I2C_SDA_PIN, ODR_100);
//	if(errCode != NRF_SUCCESS) return errCode;
	//Set the accelerometer full scale range at 2g required for activity detection.

	cpt_lis2hh12_WriteSingleRegister(CTRL4,BURST_MODE);//0x08 is for burst mode reading, 0 for 2g scale
}


/**Read Accelerometer Data
 *
 */

uint32_t bsp_accl_ReadAcceloData(accel_values_t * accel_values)
{
    uint32_t errCode;
    uint8_t rawValues[sixbytes];
    bsp_accl_SwitchonAccelo();
    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<sixbytes; i++)
    {
    	errCode = cpt_lis2hh12_ReadRegisters(OUT_X_L+i, rawValues,sixbytes);
    	//if(errCode != NRF_SUCCESS) return errCode;
    	*data = rawValues[1];
        data++;
    }
    return errCode;
}


/*@Brief This function is to set the interrupt pin1 for interrupt1 (ACTIVITY) and interrupt pin2
 * for interrupt2 (INACTIVITY).
 * Interrupt1 for Activity
 * Interrupt2 for Inactivity
*/

void bsp_accl_SetInterruptPin(void)
{
	// Interrupt1 for Activity on Interrupt 1 pin

	cpt_lis2hh12_WriteSingleRegister(CTRL3,INTERRUPT_1_PIN1);

	// Interrupt2 for Inactivity on Interrupt 2 pin

	cpt_lis2hh12_WriteSingleRegister(CTRL6,INTERRUPT_2_PIN2);
	//cpt_lis2hh12_WriteSingleRegister(CTRL7,0x0C);
}

/*@Brief This function is to set the threshold and duration for the Activity detected
* Activity threshold will be set for 2g and the activity duration will be set for 1s
* Activty threshold = Value*(Full Scale/255)
* Activity_duration = Value*(1/odr)
*/
void bsp_accl_SetActivityThresDur(void)
{
	// Set the threshold as 2g for threshold for X direction

	cpt_lis2hh12_WriteSingleRegister(IG_THS_X1,XACTTHRESHOLD_2G);

	// Set the threshold as 2g for threshold for Y direction

	cpt_lis2hh12_WriteSingleRegister(IG_THS_Y1,YACTTHRESHOLD_2G);

	// Set the threshold as 2g for threshold for Z direction

	cpt_lis2hh12_WriteSingleRegister(IG_THS_Z1,ZACTTHRESHOLD_2G);

	// Set the threshold on the ACT_THRS register as 1 s

	cpt_lis2hh12_WriteSingleRegister(IG_DUR1,DURACT_1S);
}

/*@Brief This function is to set the threshold and duration for the Inactivity detected.
* Inactivity threshold will be set for 1g and the activity duration will be set for 500ms
* Inactivity threshold = Value*(Full Scale/255)
* Inactivity_duration = Value*(1/odr)
*/

void bsp_accl_SetInactivityThresDur(void)
{
	// Set the threshold on the Inactivity threshold register as 1g

	cpt_lis2hh12_WriteSingleRegister(IG_THS2,INACTTHRESHOLD_1G_ALLAXIS);

	// Set the duration on the Inactivity  register as 500 ms.

	cpt_lis2hh12_WriteSingleRegister(IG_DUR2,DURINACT_1S);
}


/*@Brief This function is to set the configuration of the of the Activity Register.
 * The threshold has to to be reached in either of the 3 axis .
 * The threshold is based on the movement only
 *
*/
void bsp_accl_SetActivityConfig(void)
{
	cpt_lis2hh12_WriteSingleRegister(IG_CFG1,SET_ACTTHRES_ON1AXIS);
}

/*@Brief This function is to set the configuration of the of the Inactivity Register.
 * The threshold has to to be reached in all of the 3 axis .
 * The threshold is based on the movement in the all the axis.
*/
void bsp_accl_SetInactivityConfig(void)
{
	cpt_lis2hh12_WriteSingleRegister(IG_CFG2,SET_INACTTHRES_ON3AXIS);
}


/*@Brief This function is to generate interrupt on the Interrupt pin 1 and Interrupt pin 2
 *
*/
void bsp_accl_SetInterrupt(void)
{
	bsp_accl_SwitchonAccelo();

	//Set the corresponding interrupt on either of the interrupt pin

	bsp_accl_SetInterruptPin();

	bsp_accl_SetActivityConfig();

	bsp_accl_SetActivityThresDur();

	bsp_accl_SetInactivityConfig();

	bsp_accl_SetInactivityThresDur();
}

/**@brief Initialize the gpio to initialize both the  interrupt from the accelerometer
 *
 */
uint32_t bsp_gpio_IntInit(void)
{
	uint32_t errCode;
	errCode = cpt_gpio_AccInit(INTERRUPT_1_PIN);
	errCode = cpt_gpio_AccInit(INTERRUPT_2_PIN);
	return errCode;
}


/**Read Interrupt_Status1
 *
 */
uint8_t bsp_accl_ReadInt1Data(void)
{
    uint32_t errCode;
    uint8_t rawValues[singlebyte];
    for(uint8_t i = 0; i<singlebyte; i++)
    {
    	errCode = cpt_lis2hh12_ReadRegisters(IG_SRC1+i, rawValues,singlebyte);
    }
    return rawValues[0];
}

/**Read Interrupt_Status2
 *
 */
uint8_t bsp_accl_ReadInt2Data(void)
{
    uint32_t errCode;
    uint8_t rawValues[singlebyte];
    for(uint8_t i = 0; i<singlebyte; i++)
    {
    	errCode = cpt_lis2hh12_ReadRegisters(IG_SRC2+i, rawValues,singlebyte);
    }
    return rawValues[0];
}

