/*
 * cpt_uart.h
 *
 *  Created on: 30-Apr-2019
 *      Author: Inertial Elements
 */

#ifndef DRIVERS_CPT_INC_CPT_UART_H_
#define DRIVERS_CPT_INC_CPT_UART_H_
#include "app_uart.h"

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

/**
 * @brief Function for error handling of UART transmission.
 *
 * param[in] p_event In case of transfer has occurred
 */

void cpt_uart_ErrorHandle(app_uart_evt_t * p_event);

/**
 * @brief Function for UART settings.
 */
void cpt_uart_Function(void);


#endif /* DRIVERS_CPT_INC_CPT_UART_H_ */
