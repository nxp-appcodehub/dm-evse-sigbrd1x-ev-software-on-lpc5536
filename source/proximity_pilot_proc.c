/*
 * Copyright 2022-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "sigbrd_application.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PP_CTIMER          CTIMER0         /* Timer 0 */
#define PP_CTIMER_MAT3_OUT kCTIMER_Match_3 /* Match output 3 */
#define PP_CTIMER_EMT3_OUT (1u << kCTIMER_Match_3)
#define PP_CTIMER_CLK_FREQ CLOCK_GetCTimerClkFreq(0U)

#define PP_DETECTED_LEVEL_MIN	38000	/* 2V at PP_DET */
#define PP_DETECTED_150R_MIN	16000
#define PP_DETECTED_150R_MAX	22000
#define PP_DETECTED_480R_MIN	32000
#define PP_DETECTED_480R_MAX	36000

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
kPPState g_PPState;
kPPState g_oldPPState;
volatile kPPState g_lastPPState;
volatile uint16_t ppVal;
volatile uint32_t ppValRead;
PP_CALLBACK g_ppAppCallback;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Initializes the Proximity Pilot.
 *
 * This function initializes the variables for Proximity pilot handling.
 *
 */
void PP_Init(void)
{
	ctimer_config_t config;

    /* Use 12 MHz clock for some of the Ctimers */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 0u, false);
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 1u, true);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);

    /* Initialize CTIMER for PWM period and ON time measurement */
    CTIMER_GetDefaultConfig(&config);

    /* Set pre-scale to run timer count @1MHz */
    config.prescale = (PP_CTIMER_CLK_FREQ/1000000) - 1;
    config.mode = kCTIMER_TimerMode;
    CTIMER_Init(PP_CTIMER, &config);

    /* set MAT3 to 1000 counts @1MHz clock counter = 1 msec interval */
    CTIMER_UpdatePwmPulsePeriod(PP_CTIMER, 3U, 1000);
    PP_CTIMER->EMR |= CTIMER_EMR_EMC3_MASK;
    PP_CTIMER->MCR |= CTIMER_MCR_MR3R_MASK;
    CTIMER_StartTimer(PP_CTIMER);
	g_PPState = kPPNotDetected;
	g_oldPPState = kPPNotDetected;
}

/*!
 * @brief Register callback function for Proximity Pilot.
 *
 * This function can be used to notify the caller/application
 * with any change indicated by PP_INFOTYPE.
 *
 */
void PP_RegisterAppCallback(PP_CALLBACK appCallback)
{
	g_ppAppCallback = appCallback;
}

/*!
 * @brief Process Proximity Pilot.
 *
 * This function analyze the state of proximity pilot signal.
 *
 */
void PP_Process(void)
{
	ppValRead = GPIO_PinRead(GPIO, BOARD_PP_WAKEUP_PORT, BOARD_PP_WAKEUP_PIN);

	lpadc_conv_result_t mLpadcResultConfigStruct;
	if (!g_LpadcChnPPConversionCompletedFlag)
	{
		return;
	}

	ppVal = g_LpadcChnPPResultConfigStruct.convValue;

	g_LpadcChnPPConversionCompletedFlag = false;

	g_lastPPState = g_PPState;
	if (ppVal > PP_DETECTED_LEVEL_MIN)
	{
		g_PPState = kPPNotDetected;
	}
	else if ((ppVal >= PP_DETECTED_150R_MIN) && (ppVal < PP_DETECTED_150R_MAX))
	{
		g_PPState = kPP150RDetected;
	}
	else if ((ppVal >= PP_DETECTED_480R_MIN) && (ppVal < PP_DETECTED_480R_MAX))
	{
		g_PPState = kPP480RDetected;
	}

	if(g_PPState != g_oldPPState)
	{
		if(g_ppAppCallback != NULL)
		{
			g_ppAppCallback(PP_INFOTYPE_STATE_CHANGED, g_oldPPState, g_PPState);
		}
		g_oldPPState = g_PPState;
	}
}
