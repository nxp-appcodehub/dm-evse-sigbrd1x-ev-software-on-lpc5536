/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2017,2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_usart.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_power.h"
#include "sigbrd_application.h"
#include "comm_port_driver.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define LIN_DEBUG
#ifdef LIN_DEBUG
#define LIN_USART          USART0
#define LIN_USART_CLK_FREQ CLOCK_GetFlexCommClkFreq(0U)
#define LIN_UART_CLK_ATTACH	kFRO12M_to_FLEXCOMM0
#else
#define LIN_USART          USART3
#define LIN_USART_CLK_FREQ CLOCK_GetFlexCommClkFreq(3U)
#define LIN_UART_CLK_ATTACH	kFRO12M_to_FLEXCOMM3
#endif

#define LIN_UART_BAUDRATE 9600

#define DisableLinBreak LIN_USART->INTENSET &= ~(USART_INTENSET_DELTARXBRKEN_MASK);
#define EnableLinBreak  LIN_USART->INTENSET |= USART_INTENSET_DELTARXBRKEN_MASK;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t Lin_BKflag=0;
volatile uint16_t cnt=0, lastCmdID=0;
uint8_t rxbuff[40] = {0};
uint8_t rxPid = 0x73;
volatile bool receivedFrame = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
#ifdef LIN_DEBUG
void FLEXCOMM0_IRQHandler()
#else
void FLEXCOMM3_IRQHandler()
#endif
{

	if(LIN_USART->STAT & USART_STAT_RXBRK_MASK) // detect LIN break

	{
		LIN_USART->STAT |= USART_INTENSET_DELTARXBRKEN_MASK;// clear the bit
		Lin_BKflag = 1;
		cnt = 0;
		uartRxPortStatus[UART_LIN_INDEX] = LIN_SYN;
		DisableLinBreak;

	}
	if(LIN_USART->STAT & USART_STAT_DELTARXBRK_MASK) // detect LIN break
	{
		LIN_USART->STAT |= USART_INTENSET_DELTARXBRKEN_MASK;// clear the bit
	}
	if((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(LIN_USART))
	{
		USART_ClearStatusFlags(LIN_USART,kUSART_TxError | kUSART_RxError);
		rxbuff[cnt] = USART_ReadByte(LIN_USART);;

		switch(uartRxPortStatus[UART_LIN_INDEX])
		{
		case LIN_SYN:
			if(0x55 == rxbuff[cnt])
			{
				uartRxPortStatus[UART_LIN_INDEX] = LIN_PID;
			}
			else
			{
				uartRxPortStatus[UART_LIN_INDEX] = UART_IDLE;
				DisableLinBreak;
			}
			break;
		case LIN_PID:
			if(0x32 == rxbuff[cnt])
			{
				/* got command */
				uartRxPortStatus[UART_LIN_INDEX] = UART_RXING;
			}
			else if(0X73 == rxbuff[cnt])
			{
				/* got receive request */
				uartRxPortStatus[UART_LIN_INDEX] = UART_PROCESSING;
				DisableIRQ(FLEXCOMM3_IRQn);
				receivedFrame = true;
			}
			else
			{
				uartRxPortStatus[UART_LIN_INDEX] = UART_IDLE;
				DisableLinBreak;
			}
			break;
		case UART_RXING:
			uartRxBuffer[UART_LIN_INDEX][uartRxBufIndex[UART_LIN_INDEX]] = rxbuff[cnt];
			uartRxBufIndex[UART_LIN_INDEX]++;
			//if(recdatacnt >= 3) // 2 Bytes data + 1 Bytes checksum
			if('\r' == rxbuff[cnt])
			{
				//DisableIRQ(FLEXCOMM3_IRQn);
				//lastCmdID = rxbuff[cnt-recdatacnt+1];
				uartRxPortStatus[UART_LIN_INDEX] = LIN_SYN;
				receivedFrame = true;
				EnableLinBreak;
			}

			break;
		default:break;

		}

		if(!receivedFrame)
		{
			cnt++;	// continue to receive at rxbuff
		}
		else
		{
			cnt = 0;	// reset rxbuff index
			receivedFrame = false;
		}
	}

	if(uartBytesToTx[UART_LIN_INDEX])
	{
		if(kUSART_TxFifoNotFullFlag & USART_GetStatusFlags(LIN_USART))
		{
			USART_WriteByte(LIN_USART,*(pUartTxBuffer[UART_LIN_INDEX] + uartTxBufIndex[UART_LIN_INDEX]++));
			uartBytesToTx[UART_LIN_INDEX]--;
			if (uartBytesToTx[UART_LIN_INDEX] == 0)
			{
				uartTxPortStatus[UART_LIN_INDEX] = UART_IDLE;
				uartTxBufIndex[UART_LIN_INDEX] = 0;
				uartRxBufIndex[UART_LIN_INDEX] = 0;
				USART_DisableInterrupts(LIN_USART, kUSART_TxLevelInterruptEnable);
			}
		}
	}
	SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Main function
 */
void LIN_Slave_Init(void)
{
	usart_config_t config;

	/*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUSART_ParityDisabled;
	 * config.stopBitCount = kUSART_OneStopBit;
	 * config.loopback = false;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = LIN_UART_BAUDRATE;
	config.enableTx     = true;
	config.enableRx     = true;

	USART_Init(LIN_USART, &config, LIN_USART_CLK_FREQ);
	LIN_USART->CFG|=USART_CFG_LINMODE(true);
	//LIN_USART->CFG|=USART_CFG_RXPOL_MASK;
	/* Enable RX interrupt. */
	LIN_USART->INTENSET |= USART_INTENSET_DELTARXBRKEN_MASK; //USART_INTENSET_STARTEN_MASK |
	USART_EnableInterrupts(LIN_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
#ifdef LIN_DEBUG
	EnableIRQ(FLEXCOMM0_IRQn);
#else
	EnableIRQ(FLEXCOMM3_IRQn);
#endif
}

/*!
 * @brief Calculates Enhanced check sum as per LIN2.0 specification
 */
uint8_t Calculate_Checksum(uint8_t *buff, uint8_t len)
{
	uint16_t checkSum;

	checkSum = rxPid;
	for(int i=0; i<len; i++)
	{
		checkSum += buff[i];
		checkSum = (checkSum >> 8) + (checkSum & 0xFF);
	}
	checkSum = (~checkSum & 0xFF);	// invert

	return checkSum;
}

/*!
 * @brief Transmit a buffer frame
 */
void LIN_Transmit(UART_INDEX uart_index, uint8* txBuff, uint16 txLength)
{
	txBuff[txLength] = Calculate_Checksum(txBuff, txLength);
	pUartTxBuffer[UART_LIN_INDEX] = txBuff;
	uartBytesToTx[UART_LIN_INDEX] = txLength+1;
	uartTxBufIndex[UART_LIN_INDEX] = 0;
	uartRxBufIndex[UART_LIN_INDEX] = 0;
	uartRxPortStatus[UART_LIN_INDEX] = LIN_SYN;
	uartTxPortStatus[UART_LIN_INDEX] = UART_TXING;
	EnableLinBreak;
	USART_EnableInterrupts(LIN_USART, kUSART_TxLevelInterruptEnable);
	EnableIRQ(FLEXCOMM3_IRQn);
}
