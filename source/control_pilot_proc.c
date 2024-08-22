/*
 * Copyright 2022-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "control_pilot.h"
#include "comm_port_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CP_CTIMER          CTIMER1         /* Timer 1 */
#define CP_CTIMER_CLK_FREQ CLOCK_GetCTimerClkFreq(1U)

#define STATEA_MIN_LEVEL				61000
#define STATEA_MAX_LEVEL				62000
#define STATEB_MIN_LEVEL				54000
#define STATEB_MAX_LEVEL				55000
#define STATEC_MIN_LEVEL				48000
#define STATEC_MAX_LEVEL				49000
#define STATEF_MIN_LEVEL				25000	/* -12V low state value */
#define STATEF_MAX_LEVEL				27000	/* -12V low state value */

/* The PWM base address */
#define BOARD_PWM_BASEADDR        PWM1
#define PWM_SRC_CLK_FREQ          CLOCK_GetFreq(kCLOCK_BusClk)
#define DEMO_PWM_FAULT_LEVEL      true
#define APP_DEFAULT_PWM_FREQUENCE (1000UL)
/* Definition for default PWM frequency in Hz. */
#ifndef APP_DEFAULT_PWM_FREQUENCE
#define APP_DEFAULT_PWM_FREQUENCE (1000UL)
#endif

//#define CP_TESTMODE
#ifdef CP_TESTMODE
volatile uint8_t testMode = 1;
#endif

#ifdef CP_DETECTPWM
void CP_DetectPWM(void)
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void CP_ActivatePilotSwitch(void);

void ctimer_capturedOnRising_callback(uint32_t flags);
void ctimer_capturedOnFalling_callback(uint32_t flags);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table[] = {
		NULL, NULL, NULL, NULL, ctimer_capturedOnRising_callback, ctimer_capturedOnFalling_callback, NULL, NULL};

/*******************************************************************************
 * Variables
 ******************************************************************************/
kCPState g_CPState, g_OldCPState;
volatile uint8_t checkONPeriod = 1;
volatile uint8_t g_startChargingFlag = 1;			// Smart controller can indicate to start/stop charging
volatile uint8_t g_chargeOnTime = 0;
volatile uint32_t g_timeoutToWaitForA2ToB2 = 1000;	// SIG controller can wait for A2 to B2 transition
volatile uint32_t g_timeoutToWaitForA2ToA1 = 1000;
volatile uint32_t g_timeoutToWaitForB2ToB1 = 1000;
volatile uint32_t evDetPCGPIOVal = 0;
/* Structure of initialize PWM */
pwm_config_t pwmConfig;
pwm_fault_param_t faultConfig;
volatile uint8_t dutyCyclePercent = 50;
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
/* Match Configuration for Channel 1 */
static ctimer_match_config_t matchConfig1;
volatile uint32_t countRising  = 0;
volatile uint32_t countFalling  = 0;
volatile uint32_t countBothEdges = 0;
volatile uint32_t risingCaptureVal = 0;
volatile uint32_t fallingCaptureVal = 0;
volatile uint32_t lastRisingEdgeTmrVal, TmrPeriodCounts, onPeriodTmrVal;
volatile uint32_t ThisOnTmrVal, TmrOnCounts;
volatile uint32_t Ctimer1Val;
volatile float pwmOnPercent = 0.0f;
volatile uint16_t pwmOnPercentMilli;
volatile float pwmFrequency = 0.0f;
ctimer_config_t config;
volatile uint8_t checkTmrCounts = 0;
uint8_t noOfBCBToggles = 2;
uint16_t durationBCBToggleinMS = 300;
bool activatePilotSwitch = false;
uint8_t bcbToggleStates = 0;
kCPEVState oldCPEVState;

/*******************************************************************************
 * Code
 ******************************************************************************/
void ctimer_capturedOnRising_callback(uint32_t flags)
{
    countRising++;
    risingCaptureVal = CP_CTIMER->CR[0];
    TmrPeriodCounts = 0x100000000 + risingCaptureVal - lastRisingEdgeTmrVal;
    pwmFrequency = TmrPeriodCounts/1.0f;
    lastRisingEdgeTmrVal = risingCaptureVal;
}

void ctimer_capturedOnFalling_callback(uint32_t flags)
{
	countFalling++;
	fallingCaptureVal = CP_CTIMER->CR[1];
	TmrOnCounts = 0x100000000 + fallingCaptureVal - risingCaptureVal;
	pwmOnPercent = (float)TmrOnCounts/(float)TmrPeriodCounts;
	pwmOnPercentMilli = (uint16_t)(pwmOnPercent*1000);	// take the integer part
	pwmOnPercent *= 100;
}

/*!
 * @brief Initializes the Control Pilot PWM.
 *
 * This function initializes the PMW at fixed 1 KHz and variable duty cycle.
 * PWM internal signal is also used to trigger the ADC channel to measure
 * Control Pilot level.
 *
 */
void CP_Init(void)
{
	/* Use 12 MHz clock for some of the Ctimers */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer1Clk, 0u, false);
    CLOCK_SetClkDiv(kCLOCK_DivCtimer1Clk, 1u, true);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER1);

    /* Connect CTimer capture input form CTIMER_INP0 pin */
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_CtimerInp0ToTimer1Captsel);
    INPUTMUX_AttachSignal(INPUTMUX, 1U, kINPUTMUX_CtimerInp0ToTimer1Captsel);

    /* Initialize CTIMER for PWM period and ON time measurement */
    CTIMER_GetDefaultConfig(&config);

    /* Set pre-scale to run timer count @1MHz */
    config.prescale = (CP_CTIMER_CLK_FREQ/1000000) - 1;
    config.mode = kCTIMER_TimerMode;
    CTIMER_Init(CP_CTIMER, &config);

    CTIMER_RegisterCallBack(CP_CTIMER, &ctimer_callback_table[0], kCTIMER_MultipleCallback);
    CTIMER_SetupCapture(CP_CTIMER, kCTIMER_Capture_0, kCTIMER_Capture_RiseEdge, true);
    CTIMER_SetupCapture(CP_CTIMER, kCTIMER_Capture_1, kCTIMER_Capture_FallEdge, true);
    CTIMER_StartTimer(CP_CTIMER);
}

/* noOfBCBToggles = Number of BCB-Toggles 1 3 number
 * durationBCBToggleinMS = Duration of each state B or C within the BCB-Toggle = 200 to 400 msec.
 * Duration of BCB-Toggle sequence = 600 to 3500 msec
*/
void CP_ActivatePilotSwitch(void)
{
	bcbToggleStates = 2 * noOfBCBToggles;
	activatePilotSwitch = true;
}

/*!
 * @brief Set the CP resistor value.
 */
void CP_SetResistor(kCPResistor cpResistor, kGPIOState gpioState)
{
	switch(cpResistor)
	{
	case kCPEVResistor_1K3:
		if(gpioState == kSTATE_OFF)
		{
			GPIO_PortSet(GPIO, BOARD_PILOT_SWITCH_PORT, 1u << BOARD_PILOT_SWITCH_PIN);
		}
		else
		{
			GPIO_PortClear(GPIO, BOARD_PILOT_SWITCH_PORT, 1u << BOARD_PILOT_SWITCH_PIN);
		}
		break;

	default: break;
	}
}

/*!
 * @brief Get the CP resistor value.
 */
void CP_GetResistor(kCPResistor cpResistor, kGPIOState *gpioState)
{
	switch(cpResistor)
	{
	case kCPEVResistor_270R:
		*gpioState = kSTATE_OFF;
		break;

	case kCPEVResistor_1K3:
		if(GPIO_PortRead(GPIO, BOARD_PILOT_SWITCH_PORT) &  (1u << BOARD_PILOT_SWITCH_PIN))
		{
			*gpioState = kSTATE_OFF;
		}
		else
		{
			*gpioState = kSTATE_ON;
		}
		break;

	default: break;
	}
}

/*!
 * @brief Detect the CP PWM.
 */
void CP_DetectPWM(void)
{
	if((pwmOnPercent < 0.5f) || (pwmOnPercent > 98.5f))
	{
		/* program in GPIO mode */
		gpio_pin_config_t gpio0_pin1_config = {
				.pinDirection = kGPIO_DigitalInput,
				.outputLogic = 0U
		};
		/* Initialize GPIO functionality on pin PIO0_1  */
		GPIO_PinInit(GPIO, BOARD_PC_EV_DET_PORT, BOARD_PC_EV_DET_PIN, &gpio0_pin1_config);
		evDetPCGPIOVal = GPIO_PinRead(GPIO, BOARD_PC_EV_DET_PORT, BOARD_PC_EV_DET_PIN);
		if (evDetPCGPIOVal == 1)
		{
			g_CPState = kCPStateHigh;
		}
		else
		{
			g_CPState = kCPStateLow;
		}

		const uint32_t port0_pin1_config = (/* Pin is configured as CTIMER_INP14 */
				IOCON_PIO_FUNC3 |
				/* No addition pin function */
				IOCON_PIO_MODE_INACT |
				/* Standard mode, output slew rate control is enabled */
				IOCON_PIO_SLEW_STANDARD |
				/* Input function is not inverted */
				IOCON_PIO_INV_DI |
				/* Enables digital function */
				IOCON_PIO_DIGITAL_EN |
				/* Open drain is disabled */
				IOCON_PIO_OPENDRAIN_DI);
		/* PORT0 PIN9 is configured as CTIMER_INP0 */
		IOCON_PinMuxSet(IOCON, BOARD_PC_EV_DET_PORT, BOARD_PC_EV_DET_PIN, port0_pin1_config);
	}
	else
	{
		g_CPState = kCPStatePWM;
	}
}
