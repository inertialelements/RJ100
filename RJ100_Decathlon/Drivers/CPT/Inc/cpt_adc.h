/*
 * cpt_adc.h
 *
 *  Created on: 26-Apr-2019
 *      Author: Inertial Elements
 */

#ifndef DRIVERS_CPT_INC_CPT_ADC_H_
#define DRIVERS_CPT_INC_CPT_ADC_H_


typedef int16_t nrf_adc_value_t;
extern nrf_adc_value_t g_ADCValue;
/**
 * @brief Function for calling the adc output.
 * @details The adc output will not be called always.It will only be called only when the adc buffer is full  with adc in this case the battery out voltage
 * data. Each sample will be updated after every specified time sampling time (ms).
 *
 * @param[out] volatgelevel     The volatge level that is read in the pin
 *
 */
//extern uint32_t cpt_adc_Config(void);

extern uint32_t cpt_adc_int(void);
/**
 * @brief Function for calling the adc sampling. This function should run in a loop for reading the adc
 */
extern void cpt_adc_sample(void);

#endif /* DRIVERS_CPT_INC_CPT_ADC_H_ */
