/*
 * analog_interface.c
 *
 *  Created on: 06-May-2019
 *      Author: Inertial Elements
 *
 *      @brief This file includes different functions to read the voltage from the battery and then
 *      calculate the battery percentage. The battery percentage to be send to the BLE interface
 *      for sending via BLE.
 */

#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_util.h"
#include "bsp_adc.h"
#include "analog_interface.h"

//static nrf_adc_value_t adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
/**< Channel instance. Default configuration used.
 * The interrupt is set at a low priority.
 *
 */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     600                                          /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS    270                                          /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define DEAD_BEEF                         0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200                                         /**< Value in millivolts for voltage used as reference in ADC conversion on NRF51. */
#define ADC_INPUT_PRESCALER               3                                            /**< Input prescaler for ADC convestion on NRF51. */
#define ADC_RES_10BIT                     1024
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)
                                     /**< Maximum digital value for 10-bit ADC conversion. */



uint8_t analog_interface_BatteryCalculation(int16_t adc_value)
{
	uint16_t        batt_lvl_in_milli_volts;
	uint8_t         percentage_batt_lvl;

	batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_value) + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
	percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);
	return percentage_batt_lvl;
}


uint8_t analog_interface_BatteryUpdateLevel(void)
 {
	uint16_t adcResult = 0;
	adcResult =bsp_adc_Config();
	uint8_t bat_percent;
	bat_percent=analog_interface_BatteryCalculation(adcResult);
	return bat_percent;
 }





