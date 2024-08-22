/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "sigbrd_application.h"
#include "fsl_spi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef LPC_TO_SJA_SPI_COMM
#define EXAMPLE_SPI_MASTER          SPI2
#define EXAMPLE_SPI_MASTER_IRQ      FLEXCOMM2_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm2
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(2U)
#define EXAMPLE_SPI_SSEL            0
#define EXAMPLE_SPI_SPOL            kSPI_SpolActiveAllLow
#define BUFFER_SIZE (64)
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifdef LPC_TO_SJA_SPI_COMM
spi_master_config_t userConfig = {0};
uint32_t srcFreq               = 0;
uint32_t i                     = 0;
uint32_t err                   = 0;
spi_transfer_t xfer            = {0};
static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/
void LPC_SW_Comm(void)
{
#ifdef LPC_TO_SJA_SPI_COMM
	/*
	 * userConfig.enableLoopback = false;
	 * userConfig.enableMaster = true;
	 * userConfig.polarity = kSPI_ClockPolarityActiveHigh;
	 * userConfig.phase = kSPI_ClockPhaseFirstEdge;
	 * userConfig.direction = kSPI_MsbFirst;
	 * userConfig.baudRate_Bps = 500000U;
	 */
	SPI_MasterGetDefaultConfig(&userConfig);
	srcFreq            = EXAMPLE_SPI_MASTER_CLK_FREQ;
	userConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL;
	userConfig.sselPol = (spi_spol_t)EXAMPLE_SPI_SPOL;
	SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq);

	/* Init Buffer*/
	for (i = 0; i < BUFFER_SIZE; i++)
	{
		srcBuff[i] = i;
	}

	/*Start Transfer*/
	xfer.txData      = srcBuff;
	xfer.rxData      = destBuff;
	xfer.dataSize    = sizeof(destBuff);
	xfer.configFlags = kSPI_FrameAssert;
	SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
#endif
}
