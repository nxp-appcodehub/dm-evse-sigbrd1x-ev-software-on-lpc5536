/*
 * Copyright 2022-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SIGBRD_APPLICATION_H__
#define SIGBRD_APPLICATION_H__

#if defined(__cplusplus)
extern "C" {
#endif
#include "types.h"
#include "board.h"
#include "fsl_pwm.h"
#include "fsl_lpadc.h"

#include "fsl_clock.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_anactrl.h"
#include "fsl_pint.h"
#include "fsl_usart.h"
#include "fsl_ctimer.h"


#include "control_pilot.h"
#include "proximity_pilot.h"
#include "comm_port_driver.h"
/*!
 * @addtogroup sigbrd_application
 * @{
 */

/*! @file */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum
{
	kSTATE_OFF,
	kSTATE_ON,
}kGPIOState;

#define DEMO_LPADC_BASE                  ADC0
#define DEMO_LPADC_CP_CHANNEL_1X         1U
#define DEMO_LPADC_CP_CHANNEL_3X         3U
#define DEMO_LPADC_PP_CHANNEL          	 4U
#define DEMO_LPADC_PP_CMDID            	 1U                          /* CMD1 */
#define DEMO_LPADC_VREF_SOURCE           kLPADC_ReferenceVoltageAlt3 /* VDDA */
#define DEMO_LPADC_DO_OFFSET_CALIBRATION true
#define DEMO_LPADC_USE_HIGH_RESOLUTION   true

/*-------  Version Numbering of the SIGBOARD code -------*/
#define MAJOR_VER               01      /* Major Release number */
#define MINOR_VER               01      /* Minor Release number*/
#define BUG_VER                 04

#define CONV_STR(s) #s
#define DEF_TO_STR(s) CONV_STR(s)

#define SIGBOARD_VERSION DEF_TO_STR(MAJOR_VER)"." DEF_TO_STR(MINOR_VER)"." DEF_TO_STR(BUG_VER)

/*SIG-BRD Hardware Version*/
#define EVSE_SIGBRD_1X 1
#define EVSE_SIGBRD_2X 2
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern volatile bool g_LpadcChnCPConversionCompletedFlag;
extern volatile bool g_LpadcChnPPConversionCompletedFlag;
extern lpadc_conv_result_t g_LpadcChnCPResultConfigStruct;
extern lpadc_conv_result_t g_LpadcChnPPResultConfigStruct;
extern kCPState g_CPState;
extern volatile uint8_t g_chargeOnTime;
extern volatile uint8_t g_startChargingFlag;
extern kPPState g_PPState;
extern volatile kPPState g_lastPPState;
extern volatile bool g_GFCIOccurred;
extern lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
extern volatile float pwmOnPercent;
extern volatile uint16_t pwmOnPercentMilli;
extern volatile uint16_t ppVal;
extern volatile uint32_t ppState;
extern volatile uint8_t onRelay;
extern volatile uint8_t offRelay;
extern uint8_t noOfBCBToggles;
extern uint16_t durationBCBToggleinMS;
extern bool activatePilotSwitch;
extern uint8_t bcbToggleStates;
extern uint8_t g_sigboardHWVer;
/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

extern void CP_ActivatePilotSwitch(void);
extern void CP_SetResistor(kCPResistor cpResistor, kGPIOState gpioState);
extern void CP_GetResistor(kCPResistor cpResistor, kGPIOState *gpioState);
extern void CP_DetectPWM(void);
extern void DetectControlPilotPWM(void);
#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* SIGBRD_APPLICATION_H__ */
