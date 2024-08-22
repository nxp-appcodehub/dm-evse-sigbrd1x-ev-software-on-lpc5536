/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIN_SLAVE_H__
#define LIN_SLAVE_H__

#include "types.h"
#include "comm_port_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif
/*!
 * @brief Initializes the LIN port for host communication.
 */
void LIN_Slave_Init(void);

/*!
 * @brief Transmits a buffer data.
*
* @param uart_index index if UART instance.
* @param txBuff   	pointer to buffer array.
* @param txLength   Length of data in bytes.
*/
void LIN_Transmit(UART_INDEX uart_index, uint8* txBuff, uint16 txLength);
#if defined(__cplusplus)
}
#endif

/* @}*/

#endif /* LIN_SLAVE_H__ */
