/*
 * cpt_rtc.h
 *
 *  Created on: 30-Apr-2019
 *      Author: Inertial Elements
 */

#ifndef DRIVERS_CPT_INC_CPT_RTC_H_
#define DRIVERS_CPT_INC_CPT_RTC_H_


#define COMPARE_COUNTERTIME  (8*3UL)    /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */

/** @brief: Function for handling the RTC0 interrupts.Mock event handler based. Real will be used as and when
 * required
 * Triggered on TICK and COMPARE0 match.
 *
 * param[in] int_type This is the input type with which it will compare whether it is a preset value or a tick
 * It will match with it and trigger
 */
void cpt_rtc_Handler(nrf_drv_rtc_int_type_t int_type);

/* @brief: Function for initializing and configuring the rtc
 * param [in] prescaler Default value is 4095. But one can choose the prescaler accordingly

*/
void cpt_rtc_Config(uint16_t prescaler);

#endif /* DRIVERS_CPT_INC_CPT_RTC_H_ */
