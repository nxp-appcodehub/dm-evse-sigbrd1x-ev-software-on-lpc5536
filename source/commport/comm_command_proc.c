/*
 * Copyright (c) 2022-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "stdio.h"
#include "sigbrd_application.h"
#include "comm_port_driver.h"
#include "math.h"
#include "pin_mux.h"
#include "lin_slave.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8  uartRxBuffer[NUM_UART_CHANNEL][zUARTBUFFER];
uint8  uartTxBuffer[NUM_UART_CHANNEL][zUARTBUFFER];
uint16 responseBufIndex;
bool respondingCmd = false;
uint8_t gpioState;
char *g_firmwareVersion;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief This function/task takes care of all communication specific sub-activities.
 * - Send the response to communication client for GFCI, PP, CP, ADC, Version and UNACK to unknown commands
 * - Checks whether the meter is powered up enough with supply voltage to sustain communication
 * - Executes any proprietary command for the meter and responds to the communication client.
 */
void Comm_Process(void)
{
	uint8  i;
	int16 ret_val;

	for(i=0; i<NUM_UART_CHANNEL; i++)
	{
		if (uartRxPortStatus[i] == UART_PROCESSING)
		{
			break;
		}
	}

	if(i == NUM_UART_CHANNEL)
	{
		return;
	}



	responseBufIndex = 0;

	/* Command parsing start */
	/* commands 'a' to 'z' for EVSE sig brd. */
	switch(uartRxBuffer[i][0])
	{
	/* commands 'a' to 'j' for EVSE sig brd. get commands */
	case 'b':
		/* get command */
		uartTxBuffer[i][responseBufIndex++] = '0' + g_PPState;
		uartTxBuffer[i][responseBufIndex++] = '[';
		uartTxBuffer[i][responseBufIndex++] = uartRxBuffer[i][0];
		uartTxBuffer[i][responseBufIndex++] = ']';
		uartTxBuffer[i][responseBufIndex++] = '\r';
		break;

	case 'g':
		/* get duty cycle in milli command */
		sprintf(&uartTxBuffer[i][responseBufIndex],  "%04d", pwmOnPercentMilli);
		responseBufIndex += 4;
		uartTxBuffer[i][responseBufIndex++] = '[';
		uartTxBuffer[i][responseBufIndex++] = uartRxBuffer[i][0];
		uartTxBuffer[i][responseBufIndex++] = ']';
		uartTxBuffer[i][responseBufIndex++] = '\r';
		break;

	case 'h':
		/* get CP resistor command */
		CP_GetResistor(uartRxBuffer[i][1] - '0', &gpioState);
		uartTxBuffer[i][responseBufIndex++] = '0'+ gpioState;
		uartTxBuffer[i][responseBufIndex++] = '[';
		uartTxBuffer[i][responseBufIndex++] = uartRxBuffer[i][0];
		uartTxBuffer[i][responseBufIndex++] = ']';
		uartTxBuffer[i][responseBufIndex++] = '\r';
		break;

	case 'a':
		/* get all */
		uartTxBuffer[i][responseBufIndex++] = '0' + g_PPState;
		uartTxBuffer[i][responseBufIndex++] = '[';
		uartTxBuffer[i][responseBufIndex++] = 'b';
		uartTxBuffer[i][responseBufIndex++] = ']';
		uartTxBuffer[i][responseBufIndex++] = '\r';
		break;

	case 's':
		/* set CP resistor command */
		CP_SetResistor(uartRxBuffer[i][1] - '0', uartRxBuffer[i][2] - '0');
		uartTxBuffer[i][responseBufIndex++] = 0;	// Null
		uartTxBuffer[i][responseBufIndex++] = '[';
		uartTxBuffer[i][responseBufIndex++] = uartRxBuffer[i][0];
		uartTxBuffer[i][responseBufIndex++] = ']';
		uartTxBuffer[i][responseBufIndex++] = '\r';
		break;
		
			case 'v':
		{
			g_firmwareVersion = SIGBOARD_VERSION;
            while(*g_firmwareVersion != '\0')
            {
              uartTxBuffer[i][responseBufIndex++]= *g_firmwareVersion;
              g_firmwareVersion++;

            }
			uartTxBuffer[i][responseBufIndex++] = '[';
			uartTxBuffer[i][responseBufIndex++] = uartRxBuffer[i][0];
			uartTxBuffer[i][responseBufIndex++] = ']';
			uartTxBuffer[i][responseBufIndex++] = END_OF_MSG;
            break;
		}

		case 'w':
		{
			uartTxBuffer[i][responseBufIndex++]= g_sigboardHWVer;
			uartTxBuffer[i][responseBufIndex++] = '[';
			uartTxBuffer[i][responseBufIndex++] = uartRxBuffer[i][0];
			uartTxBuffer[i][responseBufIndex++] = ']';
			uartTxBuffer[i][responseBufIndex++] = END_OF_MSG;
			break;
		}

	default: 
				/*Sending NACK for unrecognized commands*/
			uartTxBuffer[i][responseBufIndex++] = '[';
			uartTxBuffer[i][responseBufIndex++] = 'n';
			uartTxBuffer[i][responseBufIndex++] = ']';
			uartTxBuffer[i][responseBufIndex++] = END_OF_MSG;
			break;
	}

	/*******************************Commands end***********************************/
	/* Send the command response to the client */
	respondingCmd = true;
	if(i == UART_CONTROL_INDEX)
	{
		UART_Transmit(i, uartTxBuffer[i], responseBufIndex);
	}
	else
	{
		LIN_Transmit(i, uartTxBuffer[i], responseBufIndex);
	}
}


