/*
 * Copyright 2022-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "sigbrd_application.h"
#include "lin_slave.h"
#include "fsl_spi.h"
#include "comm_port_driver.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPADC_IRQn                  ADC0_IRQn
#define DEMO_LPADC_IRQ_HANDLER_FUNC      ADC0_IRQHandler

#define GFCI_INT_PIN_INT0_SRC     kINPUTMUX_GpioPort1Pin30ToPintsel

#define APP_CTIMER          CTIMER2         /* Timer 2 */
#define APP_CTIMER_CLK_FREQ CLOCK_GetCTimerClkFreq(2U)
#define CTIMER_MAT_OUT kCTIMER_Match_3 /* Match output 3 */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void APP_LED_Blink(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_LpadcChnCPConversionCompletedFlag = false;
volatile bool g_LpadcChnPPConversionCompletedFlag = false;
volatile uint32_t g_LpadcInterruptCounter    = 0U;
volatile bool g_SysticFlag = false;
lpadc_conv_result_t g_LpadcChnPPResultConfigStruct;
lpadc_conv_trigger_config_t mLpadcTriggerConfigStruct;
const uint32_t g_LpadcFullRange   = 65536U;
const uint32_t g_LpadcResultShift = 0U;
uint8_t ledBlinkTimerSkipCount = 0;
uint8_t bcbToggleTimerSkipCount = 0;

/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
uint8_t g_sigboardHWVer=0;


void ctimer_match3_callback(uint32_t flags);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table_app[] = {
		NULL, NULL, NULL, ctimer_match3_callback, NULL, NULL, NULL, NULL};
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * brief Interrupt callback for timer used to toggle the LEDs
 * match occurs every 50 msec 
 * param flags
 */
void ctimer_match3_callback(uint32_t flags)
{
	if(ledBlinkTimerSkipCount == 5)	// 5 x 50msec = 250 msec
	{
		ledBlinkTimerSkipCount = 0;
		/* APP_LED_Blink */
		GPIO_PortToggle(GPIO, BOARD_LED2_PORT, 1u << BOARD_LED2_PIN);
		GPIO_PortToggle(GPIO, BOARD_LED1_PORT, 1u << BOARD_LED1_PIN);
	}
	ledBlinkTimerSkipCount++;

	if(activatePilotSwitch)
	{
		if(bcbToggleTimerSkipCount == 6)	// 300 msec(BCB toggle duration)/50 msec(timer tick duration)
		{
			bcbToggleTimerSkipCount = 0;

			if(bcbToggleStates)
			{
				/* do toggling */
				if(bcbToggleStates%2 == 0)
				{
					GPIO_PortClear(GPIO, BOARD_PILOT_SWITCH_PORT, 1u << BOARD_PILOT_SWITCH_PIN);
				}
				else
				{
					GPIO_PortSet(GPIO, BOARD_PILOT_SWITCH_PORT, 1u << BOARD_PILOT_SWITCH_PIN);
				}
				bcbToggleStates--;
			}
			else
			{
				/* stop toggling */
				activatePilotSwitch = false;
				bcbToggleTimerSkipCount = 0;
			}
		}
		bcbToggleTimerSkipCount++;
	}
}

void DEMO_LPADC_IRQ_HANDLER_FUNC(void)
{
	g_LpadcInterruptCounter++;

#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2U))
	if (LPADC_GetConvResult(DEMO_LPADC_BASE, &g_LpadcChnPPResultConfigStruct, 0U))
#else
		if (LPADC_GetConvResult(DEMO_LPADC_BASE, &g_LpadcChnPPResultConfigStruct))
#endif /* FSL_FEATURE_LPADC_FIFO_COUNT */
		{
			g_LpadcChnPPConversionCompletedFlag = true;
		}

	SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief LED initialization function
 */
void APP_LED_Init(void)
{
	ctimer_config_t config;

	/* Use 12 MHz clock for some of the Ctimers */
	CLOCK_SetClkDiv(kCLOCK_DivCtimer2Clk, 0u, false);
	CLOCK_SetClkDiv(kCLOCK_DivCtimer2Clk, 1u, true);
	CLOCK_AttachClk(kFRO_HF_to_CTIMER2);

	/* Initialize CTIMER for PWM period and ON time measurement */
	CTIMER_GetDefaultConfig(&config);

	/* Set pre-scale to run timer count @1MHz */
	config.prescale = (APP_CTIMER_CLK_FREQ/1000000) - 1;
	config.mode = kCTIMER_TimerMode;
	CTIMER_Init(APP_CTIMER, &config);

	/* Configuration 0 */
	matchConfig0.enableCounterReset = true;
	matchConfig0.enableCounterStop  = false;
	matchConfig0.matchValue         = 50000;
	matchConfig0.outControl         = kCTIMER_Output_Toggle;
	matchConfig0.outPinInitState    = false;
	matchConfig0.enableInterrupt    = true;

	CTIMER_RegisterCallBack(APP_CTIMER, &ctimer_callback_table_app[0], kCTIMER_MultipleCallback);
	CTIMER_SetupMatch(APP_CTIMER, CTIMER_MAT_OUT, &matchConfig0);
	CTIMER_StartTimer(APP_CTIMER);
}


/*!
 * @brief This function is pointing to PP_RegisterAppCallback.
 *
 * It can be used to notify the caller/application
 * with any change indicated by PP_INFOTYPE.
 *
 * @param kInfoType Info types for application callback function.
 * @param param1    1st parameter.
 * @param param2    2nd parameter.
 */
static void App_PP_Callback(PP_INFOTYPE kInfoType, void *param1, void *param2)
{
	switch (kInfoType)
	{
	case PP_INFOTYPE_STATE_CHANGED:
		break;
	default:
		break;
	}
}

/*!
 * @brief Main function
 */
int main(void)
{
	lpadc_config_t mLpadcConfigStruct;
	lpadc_conv_command_config_t mLpadcCommandConfigStruct;

	/* Board pin, clock, debug console init */
	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom0Clk, 0u, false);
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom0Clk, 1u, true);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
	/* attach 12 MHz clock to FLEXCOMM3 (LIN slave) */
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom3Clk, 0u, false);
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom3Clk, 1u, true);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
	/* attach 12 MHz clock to FLEXCOMM1 (M_UART) */
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom1Clk, 0u, false);
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom1Clk, 1u, true);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
#ifdef LPC_TO_SJA_SPI_COMM
	/* attach 12 MHz clock to SPI2 */
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 0u, false);
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 1u, true);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
#endif

	RESET_ClearPeripheralReset(kFC0_RST_SHIFT_RSTn);

	/* Init output HPGP_RESET GPIO. */
	GPIO_PortInit(GPIO, 0U);
	GPIO_PortInit(GPIO, 1U);

#ifdef LPC_TO_SJA_SPI_COMM
	/* reset FLEXCOMM for SPI */
	RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);
#endif

    /*Checking the pin status for getting the hardware version*/
	g_sigboardHWVer = BOARD_PinStatus();
	if (g_sigboardHWVer == 0)
	{
		g_sigboardHWVer = EVSE_SIGBRD_1X;
	}
	BOARD_InitPins();
	BOARD_BootClockPLL150M();
	BOARD_InitDebugConsole();

	CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 2U, true);
	CLOCK_AttachClk(kFRO_HF_to_ADC0);

	/* Disable VREF power down */
	POWER_DisablePD(kPDRUNCFG_PD_VREF);

	INPUTMUX_Init(INPUTMUX);
	/* Select ADC0 trigger input from CTIMER0 MAT3 */
	INPUTMUX->ADC0_TRIG[0] = INPUTMUX_ADC0_TRIGN_ADC0_TRIG_TRIGIN(0x05);

	Uart_ModuleInit();

	/* initialize the board LIN slave */
	LIN_Slave_Init();

	ANACTRL_Init(ANACTRL);
	ANACTRL_EnableVref1V(ANACTRL, true);

	LPADC_GetDefaultConfig(&mLpadcConfigStruct);
	mLpadcConfigStruct.enableAnalogPreliminary = true;
#if defined(DEMO_LPADC_VREF_SOURCE)
	mLpadcConfigStruct.referenceVoltageSource = DEMO_LPADC_VREF_SOURCE;
#endif /* DEMO_LPADC_VREF_SOURCE */
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS
	mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;
#endif /* FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS */
	LPADC_Init(DEMO_LPADC_BASE, &mLpadcConfigStruct);

#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFS) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFS
#if defined(FSL_FEATURE_LPADC_HAS_OFSTRIM) && FSL_FEATURE_LPADC_HAS_OFSTRIM
	/* Request offset calibration. */
#if defined(DEMO_LPADC_DO_OFFSET_CALIBRATION) && DEMO_LPADC_DO_OFFSET_CALIBRATION
	LPADC_DoOffsetCalibration(DEMO_LPADC_BASE);
#else
	LPADC_SetOffsetValue(DEMO_LPADC_BASE, DEMO_LPADC_OFFSET_VALUE_A, DEMO_LPADC_OFFSET_VALUE_B);
#endif /* DEMO_LPADC_DO_OFFSET_CALIBRATION */
#endif /* FSL_FEATURE_LPADC_HAS_OFSTRIM */
	/* Request gain calibration. */
	LPADC_DoAutoCalibration(DEMO_LPADC_BASE);
#endif /* FSL_FEATURE_LPADC_HAS_CTRL_CALOFS */

#if (defined(FSL_FEATURE_LPADC_HAS_CFG_CALOFS) && FSL_FEATURE_LPADC_HAS_CFG_CALOFS)
	/* Do auto calibration. */
	LPADC_DoAutoCalibration(DEMO_LPADC_BASE);
#endif /* FSL_FEATURE_LPADC_HAS_CFG_CALOFS */

	/* Set conversion CMD configuration for CP. */
	LPADC_GetDefaultConvCommandConfig(&mLpadcCommandConfigStruct);
	mLpadcCommandConfigStruct.channelNumber = DEMO_LPADC_PP_CHANNEL;
#if defined(DEMO_LPADC_USE_HIGH_RESOLUTION) && DEMO_LPADC_USE_HIGH_RESOLUTION
	mLpadcCommandConfigStruct.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif /* DEMO_LPADC_USE_HIGH_RESOLUTION */
	mLpadcCommandConfigStruct.sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(DEMO_LPADC_BASE, DEMO_LPADC_PP_CMDID, &mLpadcCommandConfigStruct);

	/* Set trigger configuration for PP. */
	LPADC_GetDefaultConvTriggerConfig(&mLpadcTriggerConfigStruct);
	mLpadcTriggerConfigStruct.enableHardwareTrigger = true;
	mLpadcTriggerConfigStruct.targetCommandId       = DEMO_LPADC_PP_CMDID;
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
	mLpadcTriggerConfigStruct.channelAFIFOSelect = 0U;
	mLpadcTriggerConfigStruct.channelBFIFOSelect = 1U;
#endif /* FSL_FEATURE_LPADC_FIFO_COUNT */
	LPADC_SetConvTriggerConfig(DEMO_LPADC_BASE, 0U, &mLpadcTriggerConfigStruct); /* Configurate the trigger0. */

	/* Enable the watermark interrupt. */
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2U))
	LPADC_EnableInterrupts(DEMO_LPADC_BASE, kLPADC_FIFO0WatermarkInterruptEnable);
#else
	LPADC_EnableInterrupts(DEMO_LPADC_BASE, kLPADC_FIFOWatermarkInterruptEnable);
#endif /* FSL_FEATURE_LPADC_FIFO_COUNT */
	EnableIRQ(DEMO_LPADC_IRQn);

	/* initialize Proximity Pilot */
	PP_Init();
	/* initialize Control Pilot */
	CP_Init();

	/* initialize for APP default task / LED toggle */
	APP_LED_Init();

	while (1U)
	{
		/* PP_Process detects Proximity Pilot detection /w load currnet capacity */
		PP_Process();

		/* CP_Process executes host controller commands and switch PWM state */
		CP_DetectPWM();

		/* handle communication over UART and LIN(TBD) */
		Comm_Process();
	}
}
