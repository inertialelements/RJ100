/*
 * cpt_rtc.c
 *
 *  Created on: 30-Apr-2019
 *      Author: Inertial Elements
 *      @brief This file contains functions to use for setting up the rtc. Initialization of the rtc
 *      is done in this file.
 */




#include "hal_drv_rtc.h"
#include "hal_drv_clock.h"
#include "cpt_rtc.h"
#include "app_error.h"
#include <stdint.h>


#define channel 0

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
void cpt_rtc_Handler(nrf_drv_rtc_int_type_t int_type)
{
	// First it will compare with Compare 0

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
       //do nothing
    }

    // For the output it will compare with the Tick whose default value is set in the library

    else if (int_type == NRF_DRV_RTC_INT_TICK)
    {
       //do nothing
    }
}

//TODO Transmit the data and time to update the RTC

 void cpt_rtc_Config(uint16_t prescaler)
{
    uint32_t err_code;

    //Initialize RTC instance

    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;

    //Set the value of the prescaler

    config.prescaler = prescaler;
    err_code = nrf_drv_rtc_init(&rtc, &config, cpt_rtc_Handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt

    nrf_drv_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds

    err_code = nrf_drv_rtc_cc_set(&rtc,channel,COMPARE_COUNTERTIME,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance

    nrf_drv_rtc_enable(&rtc);
}

