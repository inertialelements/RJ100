/*
 * bsp_accl.h
 *
 *  Created on: 25-Apr-2019
 *      Author: Inertial Elements
 */

#ifndef DRIVERS_BSP_INC_BSP_ACCL_H_
#define DRIVERS_BSP_INC_BSP_ACCL_H_

#define INTERRUPT_1_PIN 10
#define INTERRUPT_2_PIN 9




/**@brief Structure to hold acceleromter values.
 * Sequence of z, y, and x is important to correspond with
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}accel_values_t;

/*@Brief This function is used for initialization of the accelerometr
*/

void bsp_accl_SwitchonAccelo(void);

/*@Brief This function read the accelerometer  values from the sensor
 *param[out]accel_values  It will store the values  of all the axis i.e x,y and z
 *in the pointer specified here
*/

uint32_t bsp_accl_ReadAcceloData(accel_values_t * accel_values);

/*@Brief This function is to set the interrupt pin1 for interrupt1 (ACTIVITY) and interrupt pin2
 * for interrupt2 (INACTIVITY).
 * Interrupt1 for Activity
 * Interrupt2 for Inactivity
*/

void bsp_accl_SetInterruptPin(void);

/*@Brief This function is to set the threshold and duration for the Activity detected
* Activity threshold will be set for 2g and the activity duration will be set for 1s
* Activty threshold = Value*(Full Scale/255)
* Activity_duration = Value*(1/odr)
*/
void bsp_accl_SetActivityThresDur(void);

/*@Brief This function is to set the configuration of the of the Activity Register.
 * The threshold has to to be reached in either of the 3 axis .
 * The threshold is based on the movement only
 *
*/
void bsp_accl_SetActivityConfig(void);

/*@Brief This function is to generate interrupt on the Interrupt pin 1 and Interrupt pin 2.
 * This function is called generate an interrupt
 *
*/

extern void bsp_accl_SetInterrupt(void);

/* @Brief This function is to initialize the generated interrupt from the accelerometer on Pin1 and Pin2.
 * So that it can read the interrupt from the both the pins
*/

extern uint32_t bsp_gpio_IntInit(void);

/* @Brief This function is to read the generated interrupt from the accelerometer on Pin1 and Pin2.
 * So that it can read the interrupt from the Int1 pins
*/
extern uint8_t bsp_accl_ReadInt1Data(void);

/* @Brief This function is to read the generated interrupt from the accelerometer on Pin1 and Pin2.
 * So that it can read the interrupt from the Int2  pins
*/
extern uint8_t bsp_accl_ReadInt2Data(void);

#endif /* DRIVERS_BSP_INC_BSP_ACCL_H_ */
