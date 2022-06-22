/*
 * bsp_afe.h
 *
 *  Created on: 25-Apr-2019
 *      Author: Inertial Elements
 */

#ifndef DRIVERS_BSP_INC_BSP_AFE_H_
#define DRIVERS_BSP_INC_BSP_AFE_H_



/**
 * @brief Function for reading the fifo data register of max30110.
 * @remark The LED parameters are so chosen such that minimum power is utilized. The sampling rate is set at 100 Hz.
 * Dual pulse mode is selected. However one can alternate these parameters based on performance and power.
 * 19 bit ADC resolution is selected.The LED settling time is also set at maximum. This is so chosen so that
 * we don't miss out on any sample.
 * @see cpt_max30110.h
 *
 */
void bsp_afe_ReadFIFO(void);

/**
 * @brief Function to get the required data from the max30110.
 * @remark The final data of the hrs is send to the philips library. This send the LED light intensity.
 *
 */
void bsp_afe_GetData(void);


#endif /* DRIVERS_BSP_INC_BSP_AFE_H_ */
