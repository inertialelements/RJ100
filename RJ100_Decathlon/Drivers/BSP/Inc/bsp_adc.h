/*
 * bsp_battery.h
 *
 *  Created on: 29-Apr-2019
 *      Author: Inertial Elements
 */

#ifndef DRIVERS_BSP_INC_BSP_ADC_H_
#define DRIVERS_BSP_INC_BSP_ADC_H_

/**@brief: This function is to initialize the ADC
*/
void bsp_adc_init(void);

/**@brief: This function update the battery level at every 100 ms interval, the data will be send through the
 * BLE.
 * @retval The update volatge from battery
 */
extern uint16_t bsp_adc_Config(void);

#endif /* DRIVERS_BSP_INC_BSP_ADC_H_ */
