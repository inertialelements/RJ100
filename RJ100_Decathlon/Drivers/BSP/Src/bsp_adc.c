/*
 * bsp_battery.c
 *
 *  Created on: 29-Apr-2019
 *      Author: Inertial Elements
 *      @brief ADC should read the voltage level and update the battery level
 */

#include "stdio.h"
#include "stdbool.h"

#include "../Inc/bsp_adc.h"
#include "cpt_adc.h"


/**@brief: Initialize the ADC
 *
 */
void bsp_adc_init(void)
{
	cpt_adc_int();
}

/**@brief: ADC should read the voltage level and update the battery level
 *
 */
uint16_t bsp_adc_Config(void)
{
	uint16_t adcValue;
	cpt_adc_sample();
	adcValue=g_ADCValue;
	return adcValue;

}
