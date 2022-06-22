/*
 * hrm_interface.c
 *
 *  Created on: 08-May-2019
 *      Author: Inertial Elements
 *
 *      @brief This file is to collect data from the accelerometer and hrm sensor and pass it through the
 *      Philips library and send out the data through the BLE.
 *      //TODO integrate the data with the Philips library to get the final value of the heart rate
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_util.h"
#include "bsp_accl.h"
#include "hrm_interface.h"




uint16_t heart_rate_measure(void)
{
    uint16_t        heartRate;

    heartRate = 72; //TODO Use the  real value here from the Philips library

    return heartRate;
}

