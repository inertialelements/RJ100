/*
 * analog_interface.h
 *
 *  Created on: 06-May-2019
 *      Author: Inertial Elements
 */

#ifndef APPLICATION_INC_ANALOG_INTERFACE_H_
#define APPLICATION_INC_ANALOG_INTERFACE_H_

#include "ble_bas.h"

/**
 * @brief Battery calculation.
 * Here as soon as the data from the ADC the battery level is calculated
 */
extern uint8_t analog_interface_BatteryCalculation(int16_t adc_value);
/**
 * @brief Receives the data from the adc and send the battery level output through the ble.
 */
extern uint8_t analog_interface_BatteryUpdateLevel(void);

#endif /* APPLICATION_INC_ANALOG_INTERFACE_H_ */
