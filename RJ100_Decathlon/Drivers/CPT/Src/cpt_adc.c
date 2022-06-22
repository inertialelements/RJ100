/*
 * cpt_adc.c
 *
 *  Created on: 26-Apr-2019
 *      Author: Inertial Elements
 *
 *     @brief  This file initialize the ADC and Reads the value that of the in ADC. Here all the parameters
 *      like the ADC scaling and storing of the value read in stored in the ADC buffer
 */




#include <stdint.h>
#include <stdio.h>
#include "hal_drv_adc.h"
#include "hal_adc.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_util.h"
#include "cpt_adc.h"

#define data_length 1

static  nrf_adc_value_t adc_buf[1];
nrf_adc_value_t g_ADCValue;

/** This function is the event handler on receive information  of a successful ADC value read.
 */

static void cpt_adc_EventHandler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
    	uint32_t        err_code;

    	g_ADCValue = p_event->data.done.p_buffer[0];
        err_code = nrf_drv_adc_buffer_convert(p_event->data.done.p_buffer, data_length);
        APP_ERROR_CHECK(err_code);
    }
}

uint32_t cpt_adc_int(void)
{
	ret_code_t err_code = nrf_drv_adc_init(NULL, cpt_adc_EventHandler);
	APP_ERROR_CHECK(err_code);
	static nrf_drv_adc_channel_t channel =
	NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_4);

	channel.config.config.input = (uint32_t)NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
	nrf_drv_adc_channel_enable(&channel);

	err_code = nrf_drv_adc_buffer_convert(&adc_buf[0],data_length);
	APP_ERROR_CHECK(err_code);
	return err_code;
}

void cpt_adc_sample(void)
{
	nrf_drv_adc_sample();
}


/** @} */
