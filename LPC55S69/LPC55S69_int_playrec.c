/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Configuration

#define INTERRUPT_MODE_ENABLED  1                           // Set to '1' to enable interrupt mode, '0' for polling mode

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_wm8904.h"
#include "music.h"
#include "fsl_codec_common.h"
#include <stdbool.h>
#include "pin_mux.h"
#include "fsl_sysctl.h"
#include "fsl_codec_adapter.h"
#include "fsl_power.h"
#include "string.h"
#include "math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SAMPLE_LENGTH                   (256)               // Number of samples per frame
#define SAMPLE_RATE                     (48000)

#define FRAME_LENGTH_BYTES              (SAMPLE_LENGTH * sizeof(int32_t))   // Stereo. I.E. two 16 bit channels stored in one 32 bit word

#define DEMO_I2C                        (I2C4)
#define DEMO_I2S_MASTER_CLOCK_FREQUENCY 24576000
#define DEMO_I2S_TX                     (I2S7)
#define DEMO_I2S_RX                     (I2S6)
#define DEMO_I2S_CLOCK_DIVIDER          (CLOCK_GetPll0OutFreq() / ((unsigned)SAMPLE_RATE) / 16U / 2U)
#define DEMO_I2S_RX_MODE                (kI2S_MasterSlaveNormalSlave)
#define DEMO_I2S_TX_MODE                (kI2S_MasterSlaveNormalMaster)
#define DEMO_AUDIO_BIT_WIDTH            (16)
#define DEMO_AUDIO_SAMPLE_RATE          SAMPLE_RATE
#define DEMO_AUDIO_PROTOCOL             kCODEC_BusI2S

#define APP_SW_PORT                     BOARD_SW3_GPIO_PORT
#define APP_SW_PIN                      BOARD_SW3_GPIO_PIN

//#define APP_SW_AUDIO_INTERRUPT          FLEXCOMM5_IRQn
//#define APP_SW_AUDIO_INTERRUPT_HANDLER  FLEXCOMM5_IRQHandler
#define APP_SW_AUDIO_INTERRUPT          SDIO_IRQn
#define APP_SW_AUDIO_INTERRUPT_HANDLER  SDIO_IRQHandler

#define ANALOG_RX_AND_TX_NOT_READY      (0x0)               // Analog frame handling flags
#define ANALOG_RX_READY                 (0x1)
#define ANALOG_TX_READY                 (0x2)
#define ANALOG_RX_AND_TX_READY          (0x3)

#define Q_M                             (8)                 // Processing Q Format (m.n)
#define Q_N                             (24)


#define ONE_POLE_CUTOFF_FREQUENCY       (3840.)             // Filter cut-off frequency = 3840 Hz
#define PI                              (3.14159265359)

#define ONE_POLE_LOW_PASS_COEFF         (exp ((-2. * PI) * (ONE_POLE_CUTOFF_FREQUENCY  / SAMPLE_RATE)))
#define ONE_POLE_HIGH_PASS_COEFF        (-exp ((-2. * PI) * (0.5 - (ONE_POLE_CUTOFF_FREQUENCY  / SAMPLE_RATE))))

#define TWO_POW_24                      (16777216)
#define Q24_ONE                         (TWO_POW_24)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void StartDigitalLoopback(void);
static void TxCallback(I2S_Type *base, i2s_handle_t *handle, status_t completionStatus, void *userData);
static void RxCallback(I2S_Type *base, i2s_handle_t *handle, status_t completionStatus, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;

wm8904_config_t wm8904Config = {
    .i2cConfig          = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .recordSource       = kWM8904_RecordSourceLineInput,
    .recordChannelLeft  = kWM8904_RecordChannelLeft2,
    .recordChannelRight = kWM8904_RecordChannelRight2,
    .playSource         = kWM8904_PlaySourceDAC,
    .slaveAddress       = WM8904_I2C_ADDRESS,
    .protocol           = kWM8904_ProtocolI2S,
    .format             = {.sampleRate = kWM8904_SampleRate48kHz, .bitWidth = kWM8904_BitWidth16},
    .mclk_HZ            = DEMO_I2S_MASTER_CLOCK_FREQUENCY,
    .master             = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8904, .codecDevConfig = &wm8904Config};

//__ALIGN_BEGIN static uint8_t r_Buffer[SAMPLE_LENGTH * 4] __ALIGN_END; /* 100 samples => time about 2 ms */
//__ALIGN_BEGIN static uint8_t s_Buffer[SAMPLE_LENGTH * 4] __ALIGN_END; /* 100 samples => time about 2 ms */

// ADC Double Buffer
static uint32_t TxBufNum = 0;
static bool TxFrameReadyFlag = false;
__ALIGN_BEGIN static uint8_t s_TxBuffer_0[FRAME_LENGTH_BYTES] __ALIGN_END;
__ALIGN_BEGIN static uint8_t s_TxBuffer_1[FRAME_LENGTH_BYTES] __ALIGN_END;

// DAC Double Buffer
static uint32_t RxBufNum = 0;
static bool RxFrameReadyFlag = false;
__ALIGN_BEGIN static uint8_t s_RxBuffer_0[FRAME_LENGTH_BYTES] __ALIGN_END;
__ALIGN_BEGIN static uint8_t s_RxBuffer_1[FRAME_LENGTH_BYTES] __ALIGN_END;

// Active buffer pointers
static int16_t *ActiveTxBuffer = (int16_t*) s_TxBuffer_0;
static int16_t *ActiveRxBuffer = (int16_t*) s_RxBuffer_0;

__ALIGN_BEGIN static int32_t inputMultiplexedData[SAMPLE_LENGTH];       // Input multiplexed stereo data
__ALIGN_BEGIN static int32_t processedMultiplexedData[SAMPLE_LENGTH];   // Processed multiplexed stereo data

static int32_t q24_onePoleLowPassCoeff;                     // One-pole filter coefficients
static int32_t q24_onePoleHighPassCoeff;
static int32_t q24_onePoleOneMinusLpfc;
static int32_t q24_onePoleOneMinusHpfc;



static i2s_config_t s_TxConfig;
static i2s_config_t s_RxConfig;
static i2s_handle_t s_TxHandle;
static i2s_handle_t s_RxHandle;
static i2s_transfer_t s_TxTransfer_0;
static i2s_transfer_t s_TxTransfer_1;
static i2s_transfer_t s_RxTransfer_0;
static i2s_transfer_t s_RxTransfer_1;
extern codec_config_t boardCodecConfig;
codec_handle_t codecHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U) {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U) {
    }
}

void BOARD_InitSysctrl(void)
{
    SYSCTL_Init(SYSCTL);
    /* select signal source for share set */
    SYSCTL_SetShareSignalSrc(SYSCTL, kSYSCTL_ShareSet0, kSYSCTL_SharedCtrlSignalSCK, kSYSCTL_Flexcomm7);
    SYSCTL_SetShareSignalSrc(SYSCTL, kSYSCTL_ShareSet0, kSYSCTL_SharedCtrlSignalWS, kSYSCTL_Flexcomm7);
    /* select share set for special flexcomm signal */
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm7, kSYSCTL_FlexcommSignalSCK, kSYSCTL_ShareSet0);
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm7, kSYSCTL_FlexcommSignalWS, kSYSCTL_ShareSet0);
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm6, kSYSCTL_FlexcommSignalSCK, kSYSCTL_ShareSet0);
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm6, kSYSCTL_FlexcommSignalWS, kSYSCTL_ShareSet0);
}


/*!
 * @brief Main function
 */
int main(void)
{
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* I2C clock */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);

    PMC->PDRUNCFGCLR0 |= PMC_PDRUNCFG0_PDEN_XTAL32M_MASK;   /*!< Ensure XTAL16M is on  */
    PMC->PDRUNCFGCLR0 |= PMC_PDRUNCFG0_PDEN_LDOXO32M_MASK;  /*!< Ensure XTAL16M is on  */
    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK; /*!< Ensure CLK_IN is on  */
    ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_SYSTEM_CLK_OUT_MASK;

    CLOCK_AttachClk(kEXT_CLK_to_PLL0);

    const pll_setup_t pll0Setup = {
        .pllctrl = SYSCON_PLL0CTRL_CLKEN_MASK | SYSCON_PLL0CTRL_SELI(8U) | SYSCON_PLL0CTRL_SELP(31U),
        .pllndec = SYSCON_PLL0NDEC_NDIV(125U),
        .pllpdec = SYSCON_PLL0PDEC_PDIV(8U),
        .pllsscg = {0x0U, (SYSCON_PLL0SSCG1_MDIV_EXT(3072U) | SYSCON_PLL0SSCG1_SEL_EXT_MASK)},
        .pllRate = 24576000U,
        .flags   = PLL_SETUPFLAG_WAITLOCK,
    };
    /*!< Configure PLL to the desired values */
    CLOCK_SetPLL0Freq(&pll0Setup);

    /* Attach PLL clock to MCLK for I2S, no divider */
    CLOCK_AttachClk(kPLL0_to_MCLK);
    SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(0U);
    SYSCON->MCLKIO  = 1U;

    CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 0U, true);
    CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 1U, false);

    /*!< Switch PLL0 clock source selector to XTAL16M */
    /* I2S clocks */
    CLOCK_AttachClk(kPLL0_DIV_to_FLEXCOMM6);
    CLOCK_AttachClk(kPLL0_DIV_to_FLEXCOMM7);

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for I2S */
    RESET_PeripheralReset(kFC6_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

    NVIC_ClearPendingIRQ(FLEXCOMM6_IRQn);
    NVIC_ClearPendingIRQ(FLEXCOMM7_IRQn);
    /* Enable interrupts for I2S */
    EnableIRQ(FLEXCOMM6_IRQn);
    EnableIRQ(FLEXCOMM7_IRQn);

    /* Initialize the rest */
    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();
    BOARD_InitSysctrl();

    PRINTF("Audio DSP Example.\r\n");
    PRINTF("Configure WM8904 codec\r\n");

    /* protocol: i2s
     * sampleRate: 48K
     * bitwidth:16
     */
    if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success) {
        PRINTF("WM8904_Init failed!\r\n");
    }
    /* Initial volume kept low for hearing safety. */
    /* Adjust it to your needs, 0x0006 for -51 dB, 0x0039 for 0 dB etc. */
//    CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, 0x0006);
    CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, 0x0039);

    PRINTF("Configure I2S\r\n");

    /*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = true;
     * pack48 = false;
     */
    I2S_TxGetDefaultConfig(&s_TxConfig);
    s_TxConfig.divider     = DEMO_I2S_CLOCK_DIVIDER;
    s_TxConfig.masterSlave = DEMO_I2S_TX_MODE;

    /*
     * masterSlave = kI2S_MasterSlaveNormalSlave;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = false;
     * pack48 = false;
     */
    I2S_RxGetDefaultConfig(&s_RxConfig);
    s_TxConfig.divider     = DEMO_I2S_CLOCK_DIVIDER;
    s_RxConfig.masterSlave = DEMO_I2S_RX_MODE;

    I2S_TxInit(DEMO_I2S_TX, &s_TxConfig);
    I2S_RxInit(DEMO_I2S_RX, &s_RxConfig);


    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        while (1) {
        }
    }

    gpio_pin_config_t usr_gpio_config = {
        kGPIO_DigitalInput,
        0,
    };
    GPIO_PinInit(GPIO, APP_SW_PORT, APP_SW_PIN, &usr_gpio_config);
    PRINTF("Press USER button to print message\r\n");
    int pinState = GPIO_PinRead(GPIO, APP_SW_PORT, APP_SW_PIN);
    PRINTF("pinState: %x\r\n", pinState);


                                                            // Initialize Interrupts
#if (INTERRUPT_MODE_ENABLED == 1)
    NVIC_ClearPendingIRQ(APP_SW_AUDIO_INTERRUPT);
    EnableIRQ(APP_SW_AUDIO_INTERRUPT);

//    NVIC_SetPriority(FLEXCOMM6_IRQn, 10);
//    NVIC_SetPriority(FLEXCOMM7_IRQn, 10);

    uint32_t prio = NVIC_GetPriority(FLEXCOMM6_IRQn);
    PRINTF("FLEXCOMM6_IRQn Priority: %d\r\n", prio);
//    NVIC_SetPriority(APP_SW_AUDIO_INTERRUPT, prio+10);
    uint32_t priority = 100;// NVIC_EncodePriority(priorityGroup, 1, 6);     // Encode priority with 6 for subpriority and 1 for preempt priority
                                                                            // Note: priority depends on the used priority grouping */
//    NVIC_SetPriority(UART0_IRQn, priority);                               // Set new priority
    NVIC_SetPriority(APP_SW_AUDIO_INTERRUPT, priority);                     // Set new priority
    prio = NVIC_GetPriority(APP_SW_AUDIO_INTERRUPT);
    PRINTF("S/W Interrupt priority: %d\r\n", prio);
#endif


                                                            // Initialize DSP functions
    q24_onePoleLowPassCoeff  = ((int32_t)(TWO_POW_24*ONE_POLE_LOW_PASS_COEFF));
    q24_onePoleHighPassCoeff = ((int32_t)(TWO_POW_24*ONE_POLE_HIGH_PASS_COEFF));

    q24_onePoleOneMinusLpfc = ((int32_t)(TWO_POW_24*(1.-ONE_POLE_LOW_PASS_COEFF)));
    q24_onePoleOneMinusHpfc = ((int32_t)(TWO_POW_24*(1.-ONE_POLE_HIGH_PASS_COEFF)));


#if (INTERRUPT_MODE_ENABLED == 0)
    int32_t analogRxTxStatus = ANALOG_RX_AND_TX_NOT_READY;
#endif

    StartDigitalLoopback();

    while (1) {
#if (INTERRUPT_MODE_ENABLED == 0)
                                        // Always transfer the data first
        if (TxFrameReadyFlag == true) {
            memcpy(ActiveTxBuffer, processedMultiplexedData, FRAME_LENGTH_BYTES);   // Write processed data to output buffer
            TxFrameReadyFlag = false;
            analogRxTxStatus += ANALOG_TX_READY;
        }
        if (RxFrameReadyFlag == true) {
            memcpy(inputMultiplexedData, ActiveRxBuffer, FRAME_LENGTH_BYTES);       // Read in new data to process
            RxFrameReadyFlag = false;
            analogRxTxStatus += ANALOG_RX_READY;
        }

                                        // If we have handled both Tx and Rx buffers then process data
        if (analogRxTxStatus == ANALOG_RX_AND_TX_READY) {
//            PRINTF("int\r\n");

            for (int32_t i = 0; i < SAMPLE_LENGTH; i++) {           // Process data
                int32_t sampleL = inputMultiplexedData[i] << 16;    // Align sign bit MSB of 32 bit word
                int32_t sampleR = inputMultiplexedData[i] & 0xffff0000;

                sampleL >>= Q_M;                                    // Align Q format
                sampleR >>= Q_M;

                int pinState = GPIO_PinRead(GPIO, APP_SW_PORT, APP_SW_PIN);
                if (!pinState) {
//                PRINTF("Button Pushed: pinState: %x\r\n",pinState);

                    static int32_t lpOnePoleState = 0;
                    static int32_t hpOnePoleState = 0;

                                        // Normalized magnitude one-pole low-pass filter
                    sampleL = (((long long)sampleL * q24_onePoleOneMinusLpfc) >> Q_N) + (((long long)lpOnePoleState * q24_onePoleLowPassCoeff) >> Q_N);
                    lpOnePoleState = sampleL;

                                        // Normalized magnitude one-pole high-pass filter
                    sampleR = (((long long)sampleR * q24_onePoleOneMinusHpfc) >> Q_N) + (((long long)hpOnePoleState * q24_onePoleHighPassCoeff) >> Q_N);
                    hpOnePoleState = sampleR;
//                    sampleR = 0;
                }

                processedMultiplexedData[i] = ((sampleL >> (16 - Q_M)) & 0xffff) + ((sampleR & 0xffff0000) << Q_M);

                analogRxTxStatus = ANALOG_RX_AND_TX_NOT_READY;
            }
        }
#endif
    }
}

#if (INTERRUPT_MODE_ENABLED == 1)
void APP_SW_AUDIO_INTERRUPT_HANDLER(void)
{
    static int32_t analogRxTxStatus = ANALOG_RX_AND_TX_NOT_READY;

    NVIC_ClearPendingIRQ(APP_SW_AUDIO_INTERRUPT);

                                        // Always transfer the data first
    if (TxFrameReadyFlag == true) {
        memcpy(ActiveTxBuffer, processedMultiplexedData, FRAME_LENGTH_BYTES);   // Write processed data to output buffer
        TxFrameReadyFlag = false;
        analogRxTxStatus += ANALOG_TX_READY;
    }
    if (RxFrameReadyFlag == true) {
        memcpy(inputMultiplexedData, ActiveRxBuffer, FRAME_LENGTH_BYTES);   // Read in new data to process
        RxFrameReadyFlag = false;
        analogRxTxStatus += ANALOG_RX_READY;
    }

                                        // If we have handled both Tx and Rx buffers then process data
    if (analogRxTxStatus == ANALOG_RX_AND_TX_READY) {

        for (int32_t i = 0; i < SAMPLE_LENGTH; i++) {           // Process data
            int32_t sampleL = inputMultiplexedData[i] << 16;    // Align sign bit MSB of 32 bit word
            int32_t sampleR = inputMultiplexedData[i] & 0xffff0000;

            sampleL >>= Q_M;                                    // Align Q format
            sampleR >>= Q_M;

            int pinState = GPIO_PinRead(GPIO, APP_SW_PORT, APP_SW_PIN);
            if (!pinState) {
//                PRINTF("Button Pushed: pinState: %x\r\n",pinState);

                static int32_t lpOnePoleState = 0;
                static int32_t hpOnePoleState = 0;

                                    // Normalized magnitude one-pole low-pass filter
                sampleL = (((long long)sampleL * q24_onePoleOneMinusLpfc) >> Q_N) + (((long long)lpOnePoleState * q24_onePoleLowPassCoeff) >> Q_N);
                lpOnePoleState = sampleL;

                                    // Normalized magnitude one-pole high-pass filter
                sampleR = (((long long)sampleR * q24_onePoleOneMinusHpfc) >> Q_N) + (((long long)hpOnePoleState * q24_onePoleHighPassCoeff) >> Q_N);
                hpOnePoleState = sampleR;
            }

            processedMultiplexedData[i] = ((sampleL >> (16 - Q_M)) & 0xffff) + ((sampleR & 0xffff0000) << Q_M);

            analogRxTxStatus = ANALOG_RX_AND_TX_NOT_READY;
        }
    }
}
#endif


static void StartDigitalLoopback(void)
{
    PRINTF("Setup digital loopback\r\n");

    memset(s_TxBuffer_0, 0, FRAME_LENGTH_BYTES);            // Clear frame buffers
    memset(s_TxBuffer_1, 0, FRAME_LENGTH_BYTES);
    memset(s_RxBuffer_0, 0, FRAME_LENGTH_BYTES);
    memset(s_RxBuffer_1, 0, FRAME_LENGTH_BYTES);

    s_TxTransfer_0.data = &s_TxBuffer_0[0];
    s_TxTransfer_0.dataSize = sizeof(s_TxBuffer_0);
    s_TxTransfer_1.data = &s_TxBuffer_1[0];
    s_TxTransfer_1.dataSize = sizeof(s_TxBuffer_1);

    s_RxTransfer_0.data = &s_RxBuffer_0[0];
    s_RxTransfer_0.dataSize = sizeof(s_RxBuffer_0);
    s_RxTransfer_1.data = &s_RxBuffer_1[0];
    s_RxTransfer_1.dataSize = sizeof(s_RxBuffer_1);

    I2S_TxTransferCreateHandle(DEMO_I2S_TX, &s_TxHandle, TxCallback, NULL);
    I2S_RxTransferCreateHandle(DEMO_I2S_RX, &s_RxHandle, RxCallback, NULL);

    I2S_RxTransferNonBlocking(DEMO_I2S_RX, &s_RxHandle, s_RxTransfer_0);    // Push some frames into the pipeline
    I2S_TxTransferNonBlocking(DEMO_I2S_TX, &s_TxHandle, s_TxTransfer_0);

    I2S_RxTransferNonBlocking(DEMO_I2S_RX, &s_RxHandle, s_RxTransfer_1);
    I2S_TxTransferNonBlocking(DEMO_I2S_TX, &s_TxHandle, s_TxTransfer_1);


}

static void TxCallback(I2S_Type *base, i2s_handle_t *handle,
        status_t completionStatus, void *userData)
{
    if (completionStatus == kStatus_I2S_BufferComplete) {
        if (TxBufNum == 0) {
                                        // Enqueue next buffer
            I2S_TxTransferNonBlocking(base, handle, s_TxTransfer_0);

                                        // Switch active buffer
            ActiveTxBuffer = (int16_t*) s_TxBuffer_1;
            TxBufNum = 1;
        }

        else {
                                        // Enqueue next buffer
            I2S_TxTransferNonBlocking(base, handle, s_TxTransfer_1);

                                        // Switch active buffer
            ActiveTxBuffer = (int16_t*) s_TxBuffer_0;
            TxBufNum = 0;
        }
        TxFrameReadyFlag = true;
#if (INTERRUPT_MODE_ENABLED == 1)
        NVIC_SetPendingIRQ(APP_SW_AUDIO_INTERRUPT);         // Trigger frame interrupt
#endif
    }
}

static void RxCallback(I2S_Type *base, i2s_handle_t *handle,
        status_t completionStatus, void *userData)
{
    if (completionStatus == kStatus_I2S_BufferComplete) {
        if (RxBufNum == 0) {
                                        // Enqueue next buffer
            I2S_RxTransferNonBlocking(base, handle, s_RxTransfer_0);

                                        // Switch active buffer
            ActiveRxBuffer = (int16_t*) s_RxBuffer_1;
            RxBufNum = 1;
        }

        else {
                                        // Enqueue next buffer
            I2S_RxTransferNonBlocking(base, handle, s_RxTransfer_1);

                                        // Switch active buffer
            ActiveRxBuffer = (int16_t*) s_RxBuffer_0;
            RxBufNum = 0;
        }
        RxFrameReadyFlag = true;
#if (INTERRUPT_MODE_ENABLED == 1)
        NVIC_SetPendingIRQ(APP_SW_AUDIO_INTERRUPT);         // Trigger frame interrupt
#endif
    }
}

