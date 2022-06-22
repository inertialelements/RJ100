/*
 * main.c
 *
 *  Created on: 08-May-2019
 *  Author: Inertial Elements
 *
 *
 */
#include <stdint.h>
#include <string.h>

#include "../Drivers/BSP/Inc/bsp_adc.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "hrm_interface.h"
#include "ble_interface.h"
#include "analog_interface.h"
#include "bsp_accl.h"
#include "bsp_adc.h"
#include "nrf_delay.h"



#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/**@brief Function for application main entry.
 */

int main(void)
{
    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    bsp_accl_SetInterrupt();
    bsp_gpio_IntInit();
    bsp_adc_init();
    ble_init();

}

