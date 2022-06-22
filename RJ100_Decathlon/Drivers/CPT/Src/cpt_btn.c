/*
 * cpt_btn.c
 *
 *  Created on: 21-May-2019
 *      Author:Inertial Elements
 *
 *      @brief This file includes functionality to do any task based on the input of buttons if any
 */


/** @brief Function for handling the btn interrupt
 *
 */
#include "sdk_common.h"
#include "hal_drv_gpiote.h"

static void in_pin_BtnHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//TODO Based on philips library we have define event handler if any
}

uint32_t cpt_gpio_BtnInit(uint8_t gpioIntPin)
{
    ret_code_t errCode=0;
    nrf_drv_gpiote_uninit();
    errCode = nrf_drv_gpiote_init(); //Initializing the GPIO pin for reading the interrupt
    APP_ERROR_CHECK(errCode);
    // Choose high Sense setting for reading the  interrupt
    nrf_drv_gpiote_in_config_t in_config =  	GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    errCode = nrf_drv_gpiote_in_init(gpioIntPin, &in_config, in_pin_BtnHandler);// Config and event on initialization
    APP_ERROR_CHECK(errCode);
    nrf_drv_gpiote_in_event_enable(gpioIntPin, false);

    return errCode;
}




