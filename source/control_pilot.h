/*
 * Copyright 2023-2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CONTROL_PILOT_H__
#define CONTROL_PILOT_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include "types.h"
#include "board.h"
#include "fsl_pwm.h"
#include "fsl_lpadc.h"

#include "fsl_clock.h"
#include "fsl_inputmux.h"
#include "fsl_anactrl.h"

/*!
 * @addtogroup control_pilot
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*Timer Threshold to check the difference between TC & CR register of Timer*/
#define TIMER_THRESHOLD   2000

/*! @brief List of Control Pilot states */
typedef enum
{
	kCPStateHigh,
	kCPStateLow,
	kCPStatePWM,
}kCPState;

/*! @brief Info types for application callback function */
typedef enum
{
	kCPEVStateA,
	kCPEVStateB,
	kCPEVStateC,
	kCPEVStateD,
}kCPEVState;

typedef enum
{
	kCPEVResistor_270R,
	kCPEVResistor_1K3,
}kCPResistor;


/*******************************************************************************
 * Variables
 ******************************************************************************/
extern volatile bool g_LpadcChnCPConversionCompletedFlag;
extern uint32_t g_LpadcChnCPConversionCompletedCount;
extern lpadc_conv_result_t g_LpadcChnCPResultConfigStruct;
extern kCPState g_CPState;
extern volatile uint16_t g_dutyCycle;
extern lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
extern volatile float pwmOnPercent;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the Control Pilot PWM.
 *
 * This function initializes the PMW at fixed 1 KHz and variable duty cycle.
 * PWM internal signal is also used to trigger the ADC channel to measure
 * Control Pilot level.
 *
 */
void CP_Init(void);

/*!
 * @brief Sets up the duty cycle of Control Pilot PWM.
 *
 * This function dynamically change the duty cycle of the Control Pilot PWM.
 *
 * @param dutyCyclePercentMilli   duty cycle in milli percentage. Range 0-1000.
 */
void CP_SetDutyCycle(uint16_t dutyCyclePercentMilli);

/*!
 * @brief Control Process.
 *
 * This function measures the Control Pilot states and saves to user accessible variables.
 *
 */
void CP_Process(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* CONTROL_PILOT_H__ */
