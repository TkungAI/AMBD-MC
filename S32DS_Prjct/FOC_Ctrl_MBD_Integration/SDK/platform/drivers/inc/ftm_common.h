/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * @file ftm_common.h
 *
 * @page misra_violations MISRA-C:2012 violations
  *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, global typedef not referenced
 * The enumeration structure is used by user to enable, clear a list of interrupts.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros defined are used to define features for each driver, so this might be reported
 * when the analysis is made only on one driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * This macro is needed to improve the time efficiency.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially Boolean' type to 'essentially unsigned'.This is required by the
 * conversion of a bit-field of a bool type into a bit-field of a register type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7, Composite expression with smaller
 * essential type than other operand.
 * The expression is safe as the baud rate calculation algorithm cannot overflow
 * the result.

 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope
 * The variable is used in driver c file, so it must remain global.
 */
#ifndef FTM_COMMON_H
#define FTM_COMMON_H
#include <stddef.h>
#include "status.h"
#include "device_registers.h"
#include "callbacks.h"

/*!
 * @addtogroup ftm
 * @{
 */
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for FTM instances. */
extern FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT];

/*! @brief Interrupt vectors for the FTM peripheral. */
extern const IRQn_Type g_ftmIrqId[FTM_INSTANCE_COUNT][FEATURE_FTM_CHANNEL_COUNT];
extern const IRQn_Type g_ftmFaultIrqId[FTM_INSTANCE_COUNT];
extern const IRQn_Type g_ftmOverflowIrqId[FTM_INSTANCE_COUNT];
extern const IRQn_Type g_ftmReloadIrqId[FTM_INSTANCE_COUNT];

#ifdef ERRATA_E10856
extern bool faultDetection;
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief FTM_SC - Read and modify and write to Status And Control (RW)
 */
#define FTM_RMW_SC(base, mask, value) (((base)->SC) = ((((base)->SC) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNT - Read and modify and write to Counter (RW)
 */
#define FTM_RMW_CNT(base, mask, value) (((base)->CNT) = ((((base)->CNT) & ~(mask)) | (value)))

/*!
 * @brief FTM_MOD - Read and modify and write Modulo (RW)
 */
#define FTM_RMW_MOD(base, mask, value) (((base)->MOD) = ((((base)->MOD) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNTIN - Read and modify and write Counter Initial Value (RW)
 */
#define FTM_RMW_CNTIN(base, mask, value) (((base)->CNTIN) = ((((base)->CNTIN) & ~(mask)) | (value)))

/*!
 * @brief FTM_STATUS - Read and modify and write Capture And Compare Status (RW)
 */
#define FTM_RMW_STATUS(base, mask, value) (((base)->STATUS) = ((((base)->STATUS) & ~(mask)) | (value)))

/*!
 * @brief FTM_MODE -  Read and modify and write Counter Features Mode Selection (RW)
 */
#define FTM_RMW_MODE(base, mask, value) (((base)->MODE) = ((((base)->MODE) & ~(mask)) | (value)))

/*!
 * @brief FTM_CnSCV -  Read and modify and write Channel (n) Status And Control (RW)
 */
#define FTM_RMW_CnSCV_REG(base, channel, mask, value) (((base)->CONTROLS[channel].CnSC) = ((((base)->CONTROLS[channel].CnSC) & ~(mask)) | (value)))

/*!
 * @brief FTM_DEADTIME - Read and modify and write Dead-time Insertion Control (RW)
 */
#define FTM_RMW_DEADTIME(base, mask, value) (((base)->DEADTIME) = ((((base)->DEADTIME) & ~(mask)) | (value)))
/*!
 * @brief FTM_EXTTRIG - Read and modify and write External Trigger Control (RW)
 */
#define FTM_RMW_EXTTRIG_REG(base, mask, value) (((base)->EXTTRIG) = ((((base)->EXTTRIG) & ~(mask)) | (value)))

/*!
 * @brief FTM_FLTCTRL -  Read and modify and write Fault Control (RW)
 */
#define FTM_RMW_FLTCTRL(base, mask, value) (((base)->FLTCTRL) = ((((base)->FLTCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_FMS -  Read and modify and write Fault Mode Status (RW)
 */
#define FTM_RMW_FMS(base, mask, value) (((base)->FMS) = ((((base)->FMS) & ~(mask)) | (value)))

/*!
 * @brief FTM_CONF -  Read and modify and write Configuration (RW)
 */
#define FTM_RMW_CONF(base, mask, value) (((base)->CONF) = ((((base)->CONF) & ~(mask)) | (value)))

/*!
 * @brief POL -  Read and modify and write Polarity (RW)
 */
#define FTM_RMW_POL(base, mask, value) (((base)->POL) = ((((base)->POL) & ~(mask)) | (value)))

/*!
 * @brief FILTER -  Read and modify and write Filter (RW)
 */
#define FTM_RMW_FILTER(base, mask, value) (((base)->FILTER) = ((((base)->FILTER) & ~(mask)) | (value)))

/*!
 * @brief SYNC -  Read and modify and write Synchronization (RW)
 */
#define FTM_RMW_SYNC(base, mask, value) (((base)->SYNC) = ((((base)->SYNC) & ~(mask)) | (value)))

/*!
 * @brief QDCTRL -  Read and modify and write Quadrature Decoder Control And Status (RW)
 */
#define FTM_RMW_QDCTRL(base, mask, value) (((base)->QDCTRL) = ((((base)->QDCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR0DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 0 (RW)
 */
#define FTM_RMW_PAIR0DEADTIME(base, mask, value) (((base)->PAIR0DEADTIME) = ((((base)->PAIR0DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR1DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 1 (RW)
 */
#define FTM_RMW_PAIR1DEADTIME(base, mask, value) (((base)->PAIR1DEADTIME) = ((((base)->PAIR1DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR2DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 2 (RW)
 */
#define FTM_RMW_PAIR2DEADTIME(base, mask, value) (((base)->PAIR2DEADTIME) = ((((base)->PAIR2DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR3DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 3 (RW)
 */
#define FTM_RMW_PAIR3DEADTIME(base, mask, value) (((base)->PAIR3DEADTIME) = ((((base)->PAIR3DEADTIME) & ~(mask)) | (value)))

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief FTM_MOD_MIRROR - Read and modify and write mirror of modulo value for the FTM counter (RW)
 */
#define FTM_RMW_MOD_MIRROR(base, mask, value) (((base)->MOD_MIRROR) = ((((base)->MOD_MIRROR) & ~(mask)) | (value)))

/*!
 * @brief FTM_CnV_MIRROR -  Read and modify and write mirror of channel (n) match value (RW)
 */
#define FTM_RMW_CnV_MIRROR(base, channel, mask, value) (((base)->CV_MIRROR[channel]) = ((((base)->CV_MIRROR[channel]) & ~(mask)) | (value)))
#endif

/*!< @brief Channel number for CHAN0.*/
#define CHAN0_IDX (0U)
/*!< @brief Channel number for CHAN1.*/
#define CHAN1_IDX (1U)
/*!< @brief Channel number for CHAN2.*/
#define CHAN2_IDX (2U)
/*!< @brief Channel number for CHAN3.*/
#define CHAN3_IDX (3U)
/*!< @brief Channel number for CHAN4.*/
#define CHAN4_IDX (4U)
/*!< @brief Channel number for CHAN5.*/
#define CHAN5_IDX (5U)
/*!< @brief Channel number for CHAN6.*/
#define CHAN6_IDX (6U)
/*!< @brief Channel number for CHAN7.*/
#define CHAN7_IDX (7U)

/*******************************************************************************
 * Enumerations
 ******************************************************************************/
/*!
 * @brief FlexTimer operation mode
 *
 * Implements : ftm_config_mode_t_Class
 */
typedef enum
{
    FTM_MODE_NOT_INITIALIZED    = 0x00U,    /*!< The driver is not initialized */
    FTM_MODE_INPUT_CAPTURE      = 0x01U,    /*!< Input capture */
    FTM_MODE_OUTPUT_COMPARE     = 0x02U,    /*!< Output compare */
    FTM_MODE_EDGE_ALIGNED_PWM   = 0x03U,    /*!< Edge aligned PWM */
    FTM_MODE_CEN_ALIGNED_PWM    = 0x04U,    /*!< Center aligned PWM */
    FTM_MODE_QUADRATURE_DECODER = 0x05U,    /*!< Quadrature decoder */
    FTM_MODE_UP_TIMER           = 0x06U,    /*!< Timer with up counter */
    FTM_MODE_UP_DOWN_TIMER      = 0x07U     /*!< timer with up-down counter */
} ftm_config_mode_t;

/*!
 * @brief FlexTimer clock source selection
 *
 * Implements : ftm_clock_source_t_Class
 */
typedef enum
{
    FTM_CLOCK_SOURCE_NONE           = 0x00U,    /*!< None use clock for FTM  */
    FTM_CLOCK_SOURCE_SYSTEMCLK      = 0x01U,    /*!< System clock            */
    FTM_CLOCK_SOURCE_FIXEDCLK       = 0x02U,    /*!< Fixed clock             */
    FTM_CLOCK_SOURCE_EXTERNALCLK    = 0x03U     /*!< External clock          */
} ftm_clock_source_t;

/*!
 * @brief FlexTimer pre-scaler factor selection for the clock source.
 * In quadrature decoder mode set FTM_CLOCK_DIVID_BY_1
 *
 * Implements : ftm_clock_ps_t_Class
 */
typedef enum
{
    FTM_CLOCK_DIVID_BY_1    = 0x00U,    /*!< Divide by 1   */
    FTM_CLOCK_DIVID_BY_2    = 0x01U,    /*!< Divide by 2   */
    FTM_CLOCK_DIVID_BY_4    = 0x02U,    /*!< Divide by 4   */
    FTM_CLOCK_DIVID_BY_8    = 0x03U,    /*!< Divide by 8   */
    FTM_CLOCK_DIVID_BY_16   = 0x04U,    /*!< Divide by 16  */
    FTM_CLOCK_DIVID_BY_32   = 0x05U,    /*!< Divide by 32  */
    FTM_CLOCK_DIVID_BY_64   = 0x06U,    /*!< Divide by 64  */
    FTM_CLOCK_DIVID_BY_128  = 0x07U     /*!< Divide by 128 */
} ftm_clock_ps_t;

/*!
 * @brief List of FTM interrupts
 *
 * Implements : ftm_interrupt_option_t_Class
 */
typedef enum
{
    FTM_CHANNEL0_INT_ENABLE       = 0x00000001U,    /*!< Channel 0 interrupt */
    FTM_CHANNEL1_INT_ENABLE       = 0x00000002U,    /*!< Channel 1 interrupt */
    FTM_CHANNEL2_INT_ENABLE       = 0x00000004U,    /*!< Channel 2 interrupt */
    FTM_CHANNEL3_INT_ENABLE       = 0x00000008U,    /*!< Channel 3 interrupt */
    FTM_CHANNEL4_INT_ENABLE       = 0x00000010U,    /*!< Channel 4 interrupt */
    FTM_CHANNEL5_INT_ENABLE       = 0x00000020U,    /*!< Channel 5 interrupt */
    FTM_CHANNEL6_INT_ENABLE       = 0x00000040U,    /*!< Channel 6 interrupt */
    FTM_CHANNEL7_INT_ENABLE       = 0x00000080U,    /*!< Channel 7 interrupt */
    FTM_FAULT_INT_ENABLE          = 0x00000100U,    /*!< Fault interrupt */
    FTM_TIME_OVER_FLOW_INT_ENABLE = 0x00000200U,    /*!< Time overflow interrupt */
    FTM_RELOAD_INT_ENABLE         = 0x00000400U     /*!< Reload interrupt; Available only on certain SoC's */
} ftm_interrupt_option_t;

/*!
 * @brief List of FTM flags
 *
 * Implements : ftm_status_flag_t_Class
 */
typedef enum
{
    FTM_CHANNEL0_FLAG        = 0x00000001U,    /*!< Channel 0 Flag */
    FTM_CHANNEL1_FLAG        = 0x00000002U,    /*!< Channel 1 Flag */
    FTM_CHANNEL2_FLAG        = 0x00000004U,    /*!< Channel 2 Flag */
    FTM_CHANNEL3_FLAG        = 0x00000008U,    /*!< Channel 3 Flag */
    FTM_CHANNEL4_FLAG        = 0x00000010U,    /*!< Channel 4 Flag */
    FTM_CHANNEL5_FLAG        = 0x00000020U,    /*!< Channel 5 Flag */
    FTM_CHANNEL6_FLAG        = 0x00000040U,    /*!< Channel 6 Flag */
    FTM_CHANNEL7_FLAG        = 0x00000080U,    /*!< Channel 7 Flag */
    FTM_FAULT_FLAG           = 0x00000100U,    /*!< Fault Flag */
    FTM_TIME_OVER_FLOW_FLAG  = 0x00000200U,    /*!< Time overflow Flag */
    FTM_RELOAD_FLAG          = 0x00000400U,    /*!< Reload Flag; Available only on certain SoC's */
    FTM_CHANNEL_TRIGGER_FLAG = 0x00000800U     /*!< Channel trigger Flag */
} ftm_status_flag_t;

/*!
 * @brief FTM sync source
 *
 * Implements : ftm_reg_update_t_Class
 */
typedef enum
{
    FTM_SYSTEM_CLOCK    = 0U,       /*!< Register is updated with its buffer value at all rising
                                     *   edges of system clock */
    FTM_PWM_SYNC        = 1U        /*!< Register is updated with its buffer value at the
                                     *   FTM synchronization */
} ftm_reg_update_t;

/*!
 * @brief FTM update register
 *
 * Implements : ftm_pwm_sync_mode_t_Class
 */
typedef enum
{
    FTM_WAIT_LOADING_POINTS = 0U,   /*!< FTM register is updated at first loading point */
    FTM_UPDATE_NOW          = 1U    /*!< FTM register is updated immediately */
} ftm_pwm_sync_mode_t;

/*!
 * @brief FlexTimer pre-scaler factor for the dead-time insertion
 *
 * Implements : ftm_deadtime_ps_t_Class
 */
typedef enum
{
    FTM_DEADTIME_DIVID_BY_1  = 0x01U, /*!< Divide by 1   */
    FTM_DEADTIME_DIVID_BY_4  = 0x02U, /*!< Divide by 4   */
    FTM_DEADTIME_DIVID_BY_16 = 0x03U  /*!< Divide by 16  */
} ftm_deadtime_ps_t;

/*!
 * @brief Options for the FlexTimer behavior in BDM Mode
 *
 * Implements : ftm_bdm_mode_t_Class
 */
typedef enum
{
    FTM_BDM_MODE_00 = 0x00U,    /*!< FTM counter stopped, CH(n)F bit can be set, FTM channels
                                 *   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
                                 *   the register buffers */
    FTM_BDM_MODE_01 = 0x01U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                 *   outputs are forced to their safe value , writes to MOD,CNTIN and
                                 *   C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_10 = 0x02U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                *    outputs are frozen when chip enters in BDM mode, writes to MOD,
                                *    CNTIN and C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_11 = 0x03U     /*!< FTM counter in functional mode, CH(n)F bit can be set,
                                 *   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
                                 *   registers is in fully functional mode */
} ftm_bdm_mode_t;

/*!
 * @brief FlexTimer state structure of the driver
 *
 * Implements : ftm_state_t_Class
 */
typedef struct
{
    ftm_clock_source_t ftmClockSource;                             /*!< Clock source used by FTM counter */
    ftm_config_mode_t ftmMode;                                     /*!< Mode of operation for FTM */
    uint16_t ftmPeriod;                                            /*!< This field is used only in PWM mode to store signal period */
    uint32_t ftmSourceClockFrequency;                              /*!< The clock frequency is used for counting */
    uint16_t measurementResults[FEATURE_FTM_CHANNEL_COUNT];        /*!< This field is used only in input capture mode to store edges time stamps */
    void * channelsCallbacksParams[FEATURE_FTM_CHANNEL_COUNT];     /*!< The parameters of callback function for channels events */
    ic_callback_t channelsCallbacks[FEATURE_FTM_CHANNEL_COUNT];    /*!< The callback function for channels events */
    bool enableNotification[FEATURE_FTM_CHANNEL_COUNT];            /*!< To save channels enable the notification on the callback application */
} ftm_state_t;

/*!
 * @brief FlexTimer Registers sync parameters
 *        Please don't use software and hardware trigger simultaneously
 * Implements : ftm_pwm_sync_t_Class
 */
typedef struct
{
    bool softwareSync;                          /*!< True - enable software sync,
                                                 *   False - disable software sync */
    bool hardwareSync0;                         /*!< True - enable hardware 0 sync,
                                                 *   False - disable hardware 0 sync */
    bool hardwareSync1;                         /*!< True - enable hardware 1 sync,
                                                 *   False - disable hardware 1 sync */
    bool hardwareSync2;                         /*!< True - enable hardware 2 sync,
                                                 *   False - disable hardware 2 sync */
    bool maxLoadingPoint;                       /*!< True - enable maximum loading point,
                                                 *   False - disable maximum loading point */
    bool minLoadingPoint;                       /*!< True - enable minimum loading point,
                                                 *   False - disable minimum loading point */
    ftm_reg_update_t inverterSync;              /*!< Configures INVCTRL sync */
    ftm_reg_update_t outRegSync;                /*!< Configures SWOCTRL sync */
    ftm_reg_update_t maskRegSync;               /*!< Configures OUTMASK sync */
    ftm_reg_update_t initCounterSync;           /*!< Configures CNTIN sync */
    bool autoClearTrigger;                      /*!< Available only for hardware trigger */
    ftm_pwm_sync_mode_t syncPoint;              /*!< Configure synchronization method
                                                 *   (waiting next loading point or immediate) */
} ftm_pwm_sync_t;

/*!
 * @brief Configuration structure that the user needs to set
 *
 * Implements : ftm_user_config_t_Class
 */
typedef struct
{
    ftm_pwm_sync_t syncMethod;              /*!< Register sync options available in the
                                             *   ftm_sync_method_t enumeration  */
    ftm_config_mode_t ftmMode;              /*!< Mode of operation for FTM */
    ftm_clock_ps_t ftmPrescaler;            /*!< Register pre-scaler options available in the
                                             *   ftm_clock_ps_t enumeration  */
    ftm_clock_source_t ftmClockSource;      /*!< Select clock source for FTM */
    ftm_bdm_mode_t BDMMode;                 /*!< Select FTM behavior in BDM mode */
    bool isTofIsrEnabled;                   /*!< true: enable interrupt,
                                             *   false: write interrupt is disabled */
    bool enableInitializationTrigger;       /*!< true: enable the generation of initialization trigger
                                             *   false: disable the generation of initialization trigger */
} ftm_user_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointer to runtime state structure. */
extern ftm_state_t * ftmStatePtr[FTM_INSTANCE_COUNT];

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Sets the filter Pre-scaler divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] filterPrescale The FTM peripheral clock pre-scale divider
 *
 * Implements : FTM_DRV_SetClockFilterPs_Activity
 */
static inline void FTM_DRV_SetClockFilterPs(FTM_Type * const ftmBase,
                                            uint8_t filterPrescale)
{
    FTM_RMW_SC(ftmBase, FTM_SC_FLTPS_MASK, FTM_SC_FLTPS(filterPrescale));
}

/*!
 * @brief Reads the FTM filter clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM filter clock pre-scale divider
 *
 * Implements : FTM_DRV_GetClockFilterPs_Activity
 */
static inline uint8_t FTM_DRV_GetClockFilterPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_FLTPS_MASK) >> FTM_SC_FLTPS_SHIFT);
}

/*!
 * @brief Returns the FTM peripheral current counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The current FTM timer counter value
 *
 * Implements : FTM_DRV_GetCounter_Activity
 */
static inline uint16_t FTM_DRV_GetCounter(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->CNT) & FTM_CNT_COUNT_MASK) >> FTM_CNT_COUNT_SHIFT);
}

/*!
 * @brief Returns the FTM peripheral counter modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return FTM timer modulo value
 *
 * Implements : FTM_DRV_GetMod_Activity
 */
static inline uint16_t FTM_DRV_GetMod(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->MOD) & FTM_MOD_MOD_MASK) >> FTM_MOD_MOD_SHIFT);
}

/*!
 * @brief Returns the FTM peripheral counter initial value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return FTM timer counter initial value
 *
 * Implements : FTM_DRV_GetCounterInitVal_Activity
 */
static inline uint16_t FTM_DRV_GetCounterInitVal(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->CNTIN) & FTM_CNTIN_INIT_MASK) >> FTM_CNTIN_INIT_SHIFT);
}

/*!
 * @brief Clears the content of Channel (n) Status And Control.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel  The FTM peripheral channel number
 *
 * Implements : FTM_DRV_ClearChSC_Activity
 */
static inline void FTM_DRV_ClearChSC(FTM_Type * const ftmBase,
                                     uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->CONTROLS[channel].CnSC) = 0U;
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->CONTROLS[channel].CnSC;
#endif
}

/*!
 * @brief Gets the FTM peripheral timer channel edge level.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return The ELSnB:ELSnA mode value, will be 00, 01, 10, 11
 *
 * Implements : FTM_DRV_GetChnEdgeLevel_Activity
 */
static inline uint8_t FTM_DRV_GetChnEdgeLevel(const FTM_Type * ftmBase,
                                              uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint8_t retValue;

    retValue = (uint8_t)((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSA_MASK) >> FTM_CnSC_ELSA_SHIFT);

    retValue |= (uint8_t)(((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSB_MASK) >> FTM_CnSC_ELSB_SHIFT) << 1U);

    return retValue;
}

/*!
 * @brief Configure the feature of FTM counter reset by the selected input capture event.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enable the FTM counter reset
 *                   - true : FTM counter is reset
 *                   - false: FTM counter is not reset
 *
 * Implements : FTM_DRV_SetChnIcrstCmd_Activity
 */
static inline void FTM_DRV_SetChnIcrstCmd(FTM_Type * const ftmBase,
                                          uint8_t channel,
                                          bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write ICRST bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ICRST_MASK, FTM_CnSC_ICRST(enable));
}

/*!
 * @brief Returns whether the FTM FTM counter is reset.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the FTM peripheral timer channel ICRST
 *         - true : Enabled the FTM counter reset
 *         - false: Disabled the FTM counter reset
 *
 * Implements : FTM_DRV_IsChnIcrst_Activity
 */
static inline bool FTM_DRV_IsChnIcrst(const FTM_Type * ftmBase,
                                      uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ICRST_MASK) != 0U;
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel DMA.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enable DMA transfers for the channel
 *                   - true : Enabled DMA transfers
 *                   - false: Disabled DMA transfers
 *
 * Implements : FTM_DRV_SetChnDmaCmd_Activity
 */
static inline void FTM_DRV_SetChnDmaCmd(FTM_Type * const ftmBase,
                                        uint8_t channel,
                                        bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write DMA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_DMA_MASK, FTM_CnSC_DMA(enable));
}

/*!
 * @brief Returns whether the FTM peripheral timer channel DMA is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the FTM peripheral timer channel DMA
 *         - true : Enabled DMA transfers
 *         - false: Disabled DMA transfers
 *
 * Implements : FTM_DRV_IsChnDma_Activity
 */
static inline bool FTM_DRV_IsChnDma(const FTM_Type * ftmBase,
                                    uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_DMA_MASK) != 0U;
}

/*!
 * @brief Enables or disables the trigger generation on FTM channel outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Trigger mode control
 *                   - false : Enable PWM output without generating a pulse
 *                   - true  : Disable a trigger generation on channel output
 *
 * Implements : FTM_DRV_SetTrigModeControlCmd_Activity
 */
static inline void FTM_DRV_SetTrigModeControlCmd(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write TRIGMODE bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_TRIGMODE_MASK, FTM_CnSC_TRIGMODE((enable)));
}

/*!
 * @brief Returns whether the trigger mode is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the channel outputs
 *         - true : Enabled a trigger generation on channel output
 *         - false: PWM outputs without generating a pulse
 *
 * Implements : FTM_DRV_GetTriggerControled_Activity
 */
static inline bool FTM_DRV_GetTriggerControled(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_TRIGMODE_MASK) != 0U;
}

/*!
 * @brief Get the state of channel input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the channel inputs
 *         - true : The channel input is one
 *         - false: The channel input is zero
 *
 * Implements : FTM_DRV_GetChInputState_Activity
 */
static inline bool FTM_DRV_GetChInputState(const FTM_Type * ftmBase,
                                           uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIS_MASK) != 0U;
}

/*!
 * @brief Get the value of channel output.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return Value of the channel outputs
 *         - true : The channel output is one
 *         - false: The channel output is zero
 *
 * Implements : FTM_DRV_GetChOutputValue_Activity
 */
static inline bool FTM_DRV_GetChOutputValue(const FTM_Type * ftmBase,
                                            uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHOV_MASK) != 0U;
}

/*!
 * @brief Gets the FTM peripheral timer channel counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return Channel counter value
 *
 * Implements : FTM_DRV_GetChnCountVal_Activity
 */
static inline uint16_t FTM_DRV_GetChnCountVal(const FTM_Type * ftmBase,
                                              uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (uint16_t)((ftmBase)->CONTROLS[channel].CnV);
}

/*!
 * @brief Gets the FTM peripheral timer  channel event status.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return Channel event status
 *         - true  : A channel event has occurred
 *         - false : No channel event has occurred
 *
 * Implements : FTM_DRV_GetChnEventStatus_Activity
 */
static inline bool FTM_DRV_GetChnEventStatus(const FTM_Type * ftmBase,
                                             uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->STATUS) & (1UL << channel)) != 0U;
}

/*!
 * @brief Gets the FTM peripheral timer status info for all channels.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return Channel event status value
 *
 * Implements : FTM_DRV_GetEventStatus_Activity
 */
static inline uint32_t FTM_DRV_GetEventStatus(const FTM_Type * ftmBase)
{
    return ((ftmBase)->STATUS) & (0xFFU);
}

/*!
 * @brief Clears the FTM peripheral timer all channel event status.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_DRV_ClearChnEventStatus_Activity
 */
static inline void FTM_DRV_ClearChnEventStatus(FTM_Type * const ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->STATUS) &= (~(1UL << channel));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->STATUS;
#endif
}

/*!
 * @brief Sets the FTM peripheral timer channel output mask.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] mask Value to set Output Mask
 *                 - true : Channel output is masked
 *                 - false: Channel output is not masked
 *
 * Implements : FTM_DRV_SetChnOutputMask_Activity
 */
static inline void FTM_DRV_SetChnOutputMask(FTM_Type * const ftmBase,
                                            uint8_t channel,
                                            bool mask)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (mask)
    {
        ((ftmBase)->OUTMASK) |= (1UL << channel);
    }
    else
    {
        ((ftmBase)->OUTMASK) &= ~(1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel output initial state 0 or 1.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] state Initial state for channels output
 *                  - true : The initialization value is 1
 *                  - false: The initialization value is 0
 *
 * Implements : FTM_DRV_SetChnOutputInitStateCmd_Activity
 */
static inline void FTM_DRV_SetChnOutputInitStateCmd(FTM_Type * const ftmBase,
                                                    uint8_t channel,
                                                    bool state)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (state)
    {
        ((ftmBase)->OUTINIT) |= (1UL << channel);
    }
    else
    {
        ((ftmBase)->OUTINIT) &= ~(1UL << channel);
    }
}

/*!
 * @brief Disables the FTM peripheral timer fault interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_DisableFaultInt_Activity
 */
static inline void FTM_DRV_DisableFaultInt(FTM_Type * const ftmBase)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(0U));
}

/*!
 * @brief Enables or disables the FTM peripheral timer capture test mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Capture Test Mode Enable
 *            - true : Capture test mode is enabled
 *            - false: Capture test mode is disabled
 *
 * Implements : FTM_DRV_SetCaptureTestCmd_Activity
 */
static inline void FTM_DRV_SetCaptureTestCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_CAPTEST_MASK, FTM_MODE_CAPTEST(enable));
}

/*!
 * @brief Get status of the FTMEN bit in the FTM_MODE register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @return the FTM Enable status
 *         - true : TPM compatibility. Free running counter and synchronization compatible with TPM
 *         - false: Free running counter and synchronization are different from TPM behavior
 *
 * Implements : FTM_DRV_IsFtmEnable_Activity
 */
static inline bool FTM_DRV_IsFtmEnable(const FTM_Type * ftmBase)
{
    return ((ftmBase->MODE & FTM_MODE_FTMEN_MASK) >> FTM_MODE_FTMEN_SHIFT) != 0U;
}

/*!
 * @brief Determines if the FTM counter is re-initialized when the selected trigger for
 * synchronization is detected.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable FTM counter re-initialization selection
 *                   - true : To update FTM counter when triggered
 *                   - false: To count normally
 *
 * Implements : FTM_DRV_SetCountReinitSyncCmd_Activity
 */
static inline void FTM_DRV_SetCountReinitSyncCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_REINIT_MASK, FTM_SYNC_REINIT(enable));
}

/*!
 * @brief Checks whether the write protection is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return Write-protection status
 *         - true : If enabled
 *         - false: If not
 *
 * Implements : FTM_DRV_IsWriteProtectionEnabled_Activity
 */
static inline bool FTM_DRV_IsWriteProtectionEnabled(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_WPEN_MASK) != 0U;
}

/*!
 * @brief Checks whether the logic OR of the fault inputs is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the enabled fault inputs status
 *         - true : The logic OR of the enabled fault inputs is 1
 *         - false: The logic OR of the enabled fault inputs is 0
 *
 * Implements : FTM_DRV_IsFaultInputEnabled_Activity
 */
static inline bool FTM_DRV_IsFaultInputEnabled(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_FAULTIN_MASK) != 0U;
}

/*!
 * @brief Checks whether a fault condition is detected at the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel
 *
 * @return the fault condition status
 *         - true : A fault condition was detected at the fault input
 *         - false: No fault condition was detected at the fault input
 *
 * Implements : FTM_DRV_IsFaultFlagDetected_Activity
 */
static inline bool FTM_DRV_IsFaultFlagDetected(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < CHAN4_IDX);

    return (ftmBase->FMS & (FTM_FMS_FAULTF0_MASK << channel)) != 0U;
}

/*!
 * @brief Clear a fault condition is detected at the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel
 *
 * Implements : FTM_DRV_ClearFaultFlagDetected_Activity
 */
static inline void FTM_DRV_ClearFaultFlagDetected(FTM_Type * const ftmBase,
                                                  uint8_t channel)
{
    DEV_ASSERT(channel < CHAN4_IDX);

    ((ftmBase)->FMS) &= (~(1UL << channel));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->FMS;
#endif
}

/*!
 * @brief Enables or disables the channel invert for a channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel invert for a channel pair
 *                   - true : To enable channel inverting
 *                   - false: To disable channel inversion
 *
 * Implements : FTM_DRV_SetDualChnInvertCmd_Activity
 */
static inline void FTM_DRV_SetDualChnInvertCmd(FTM_Type * const ftmBase,
                                               uint8_t chnlPairNum,
                                               bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->INVCTRL) |=  (1UL << chnlPairNum);
    }
    else
    {
        ((ftmBase)->INVCTRL) &=  ~(1UL << chnlPairNum);
    }
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel Channel to be enabled or disabled
 * @param[in] enable State of channel software output control
 *                   - true : To enable the channel output will be affected by software output control
 *                   - false: To disable the channel output is unaffected
 *
 * Implements : FTM_DRV_SetChnSoftwareCtrlCmd_Activity
 */
static inline void FTM_DRV_SetChnSoftwareCtrlCmd(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1UL << channel);
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1UL << channel);
    }
}

/*!
 * @brief Sets the channel software output control value.
 * Despite the odd channels are configured as HIGH/LOW, they will be inverted in the following
 * configuration: COMP bit = 1 and CH(n)OCV and CH(n+1)OCV are HIGH. Please check Software
 * output control behavior chapter from RM.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] channel Channel to be configured
 * @param[in] enable State of software output control value
 *                   - true : to force 1 to the channel output
 *                   - false: to force 0 to the channel output
 *
 * Implements : FTM_DRV_SetChnSoftwareCtrlVal_Activity
 */
static inline void FTM_DRV_SetChnSoftwareCtrlVal(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if ((uint32_t)enable != (((uint32_t)ftmBase->POL >> (uint32_t)channel) & (uint32_t)1U))
    {
        ((ftmBase)->SWOCTRL) |=  (1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
}

/*FTM PWM load control*/
/*!
 * @brief Set the global load mechanism.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_SetGlobalLoadCmd_Activity
 */
static inline void FTM_DRV_SetGlobalLoadCmd(FTM_Type * const ftmBase)
{
    ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_GLDOK_SHIFT);
}

/*!
 * @brief Enable the global load.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of the global load mechanism
 *                   - true : Global Load OK enabled
 *                   - false: Global Load OK disabled
 *
 * Implements : FTM_DRV_SetLoadCmd_Activity
 */
static inline void FTM_DRV_SetLoadCmd(FTM_Type * const ftmBase,
                                      bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_GLEN_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_GLEN_SHIFT);
    }
}

/*!
 * @brief Enable the half cycle reload.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of the half cycle match as a reload opportunity
 *                   - true : Half cycle reload is enabled
 *                   - false: Half cycle reload is disabled
 *
 * Implements : FTM_DRV_SetHalfCycleCmd_Activity
 */
static inline void FTM_DRV_SetHalfCycleCmd(FTM_Type * const ftmBase,
                                           bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_HCSEL_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_HCSEL_SHIFT);
    }
}

/*!
 * @brief Enables or disables the loading of MOD, CNTIN and CV with values of their write buffer.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of loading updated values
 *                   - true : To enable the loading of value of their buffer
 *                   - false: To disable the loading of value of their buffer
 *
 * Implements : FTM_DRV_SetPwmLoadCmd_Activity
 */
static inline void FTM_DRV_SetPwmLoadCmd(FTM_Type * const ftmBase,
                                         bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_LDOK_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_LDOK_SHIFT);
    }
}

/*!
 * @brief Includes or excludes the channel in the matching process.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel Channel to be configured
 * @param[in] enable State of channel
 *                - true : means include the channel in the matching process
 *                - false: means do not include channel in the matching process
 *
 * Implements : FTM_DRV_SetPwmLoadChnSelCmd_Activity
 */
static inline void FTM_DRV_SetPwmLoadChnSelCmd(FTM_Type * const ftmBase,
                                               uint8_t channel,
                                               bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << channel);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << channel);
    }
}

/*FTM configuration*/
/*!
 * @brief Enables or disables the FTM initialization trigger on Reload Point.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable bit controls whether an initialization trigger is generated
 *                   - true : Trigger is generated when a reload point is reached
 *                   - false: Trigger is generated on counter wrap events
 *
 * Implements : FTM_DRV_SetInitTrigOnReloadCmd_Activity
 */
static inline void FTM_DRV_SetInitTrigOnReloadCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_ITRIGR_MASK) | FTM_CONF_ITRIGR(enable);
}

/*!
 * @brief Enables or disables the FTM global time base signal generation to other FTM's.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of global time base signal
 *                   - true : To enable the golobal time base generation to other FTM instances
 *                   - false: To disable the golobal time base generation to other FTM instances
 *
 * Implements : FTM_DRV_SetGlobalTimeBaseOutputCmd_Activity
 */
static inline void FTM_DRV_SetGlobalTimeBaseOutputCmd(FTM_Type * const ftmBase,
                                                      bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_GTBEOUT_MASK) | FTM_CONF_GTBEOUT(enable);
}

/*!
 * @brief Enables or disables the FTM timer global time base.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of global time base
 *                   - true : To enable an external global time base signal
 *                   - false: To disable an external global time base signal
 *
 * Implements : FTM_DRV_SetGlobalTimeBaseCmd_Activity
 */
static inline void FTM_DRV_SetGlobalTimeBaseCmd(FTM_Type * const ftmBase,
                                                bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_GTBEEN_MASK) | FTM_CONF_GTBEEN(enable);
}

/*!
 * @brief Sets the frequency of reload points
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val Value of the TOF bit set frequency
 *
 * Implements : FTM_DRV_SetLoadFreq_Activity
 */
static inline void FTM_DRV_SetLoadFreq(FTM_Type * const ftmBase,
                                       uint8_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_LDFQ_MASK, FTM_CONF_LDFQ(val));
}

/*!
 * @brief Sets the FTM extended dead-time value for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] value The FTM peripheral extend pre-scale divider using the concatenation with the dead-time value
 *
 * Implements : FTM_DRV_SetExtPairDeadtimeValue_Activity
 */
static inline void FTM_DRV_SetExtPairDeadtimeValue(FTM_Type * const ftmBase,
                                                   uint8_t channelPair,
                                                   uint8_t value)
{
    DEV_ASSERT(value < 16U);
    DEV_ASSERT(channelPair < CHAN4_IDX);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTVALEX_MASK, FTM_PAIR0DEADTIME_DTVALEX(value));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTVALEX_MASK, FTM_PAIR1DEADTIME_DTVALEX(value));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTVALEX_MASK, FTM_PAIR2DEADTIME_DTVALEX(value));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTVALEX_MASK, FTM_PAIR3DEADTIME_DTVALEX(value));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*!
 * @brief Sets the FTM dead time divider for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] divider The FTM peripheral pre-scaler divider
 *                    - FTM_DEADTIME_DIVID_BY_1 : Divide by 1
 *                    - FTM_DEADTIME_DIVID_BY_4 : Divide by 4
 *                    - FTM_DEADTIME_DIVID_BY_16: Divide by 16
 *
 * Implements : FTM_DRV_SetPairDeadtimePrescale_Activity
 */
static inline void FTM_DRV_SetPairDeadtimePrescale(FTM_Type * const ftmBase,
                                                   uint8_t channelPair,
                                                   ftm_deadtime_ps_t divider)
{
    DEV_ASSERT(channelPair < CHAN4_IDX);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTPS_MASK, FTM_PAIR0DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTPS_MASK, FTM_PAIR1DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTPS_MASK, FTM_PAIR2DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTPS_MASK, FTM_PAIR3DEADTIME_DTPS((uint8_t)divider));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*!
 * @brief Sets the FTM dead-time value for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] count The FTM peripheral selects the dead-time value
 *                  - 0U : no counts inserted
 *                  - 1U : 1 count is inserted
 *                  - 2U : 2 count is inserted
 *                  - ... up to a possible 63 counts
 *
 * Implements : FTM_DRV_SetPairDeadtimeCount_Activity
 */
static inline void FTM_DRV_SetPairDeadtimeCount(FTM_Type * const ftmBase,
                                                uint8_t channelPair,
                                                uint8_t count)
{
    DEV_ASSERT(channelPair < CHAN4_IDX);
    DEV_ASSERT(count < 64U);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTVAL_MASK, FTM_PAIR0DEADTIME_DTVAL(count));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTVAL_MASK, FTM_PAIR1DEADTIME_DTVAL(count));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTVAL_MASK, FTM_PAIR2DEADTIME_DTVAL(count));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTVAL_MASK, FTM_PAIR3DEADTIME_DTVAL(count));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief Sets the mirror of the modulo integer value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the timer modulo
 *
 * Implements : FTM_DRV_SetMirrorMod_Activity
 */
static inline void FTM_DRV_SetMirrorMod(FTM_Type * const ftmBase,
                                        uint16_t value)
{
    FTM_RMW_MOD_MIRROR(ftmBase, FTM_MOD_MIRROR_MOD_MASK, FTM_MOD_MIRROR_MOD(value));
}

/*!
 * @brief Returns the mirror of the FTM peripheral counter modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the mirror of the FTM timer modulo value
 *
 * Implements : FTM_DRV_GetMirrorMod_Activity
 */
static inline uint16_t FTM_DRV_GetMirrorMod(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->MOD_MIRROR) & FTM_MOD_MIRROR_MOD_MASK) >> FTM_MOD_MIRROR_MOD_SHIFT);
}

/*!
 * @brief Returns The modulo fractional value is used in the PWM period dithering.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the modulo fractional value
 *
 * Implements : FTM_DRV_GetModFracVal_Activity
 */
static inline uint8_t FTM_DRV_GetModFracVal(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->MOD_MIRROR) & FTM_MOD_MIRROR_FRACMOD_MASK) >> FTM_MOD_MIRROR_FRACMOD_SHIFT);
}

/*!
 * @brief Sets the mirror of the channel (n) match integer value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 * @param[in] value The value to be set to the mirror of the channel (n) match integer value
 *
 * Implements : FTM_DRV_SetMirrorChnMatchVal_Activity
 */
static inline void FTM_DRV_SetMirrorChnMatchVal(FTM_Type * const ftmBase,
                                                uint8_t channel,
                                                uint16_t value)
{
    FTM_RMW_CnV_MIRROR(ftmBase, channel, FTM_CV_MIRROR_VAL_MASK, FTM_CV_MIRROR_VAL(value));
}

/*!
 * @brief Returns the mirror of the channel (n) match integer value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 *
 * @return the mirror of the channel (n) match value
 *
 * Implements : FTM_DRV_GetMirrorChnMatchVal_Activity
 */
static inline uint16_t FTM_DRV_GetMirrorChnMatchVal(const FTM_Type * ftmBase,
                                                    uint8_t channel)
{
    return (uint16_t)((((ftmBase)->CV_MIRROR[channel]) & FTM_CV_MIRROR_VAL_MASK) >> FTM_CV_MIRROR_VAL_SHIFT);
}

/*!
 * @brief Returns the channel (n) match fractional value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 *
 * @return The channel (n) match fractional value is used in the PWM edge dithering
 *
 * Implements : FTM_DRV_GetChnMatchFracVal_Activity
 */
static inline uint8_t FTM_DRV_GetChnMatchFracVal(const FTM_Type * ftmBase,
                                                 uint8_t channel)
{
    return (uint8_t)((((ftmBase)->CV_MIRROR[channel]) & FTM_CV_MIRROR_FRACVAL_MASK) >> FTM_CV_MIRROR_FRACVAL_SHIFT);
}
#endif

/*!
 * @brief Initializes the FTM driver.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] info The FTM user configuration structure, see #ftm_user_config_t.
 * @param[out] state The FTM state structure of the driver.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_Init(uint32_t instance,
                      const ftm_user_config_t * info,
                      ftm_state_t * state);

/*!
 * @brief Shuts down the FTM driver.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_Deinit(uint32_t instance);

/*!
 * @brief This function will get the default configuration values
 *        in the structure which is used as a common use-case.
 * @param[out] config Pointer to the structure in which the
 *                    configuration will be saved.
 * @return None
 */
void FTM_DRV_GetDefaultConfig(ftm_user_config_t * const config);

/*!
 * @brief This function will mask the output of the channels and at match events will be ignored
 * by the masked channels.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsMask The mask which will select which channels will ignore match events.
 * @param [in] softwareTrigger If true a software trigger is generate to update PWM parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_MaskOutputChannels(uint32_t instance,
                                    uint32_t channelsMask,
                                    bool softwareTrigger);

/*!
 * @brief This function configure the initial counter value. The counter will get this
 * value after an overflow event.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] counterValue Initial counter value.
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */

status_t FTM_DRV_SetInitialCounterValue(uint32_t instance,
                                        uint16_t counterValue,
                                        bool softwareTrigger);

/*!
 * @brief This function configure the value of the counter which will generates an reload point.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] reloadPoint Counter value which generates the reload point.
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance,
                                         uint16_t reloadPoint,
                                         bool softwareTrigger);

/*!
 * @brief This function will force the output value of a channel to a specific value.
 * Before using this function it's mandatory to mask the match events using
 * FTM_DRV_MaskOutputChannels and to enable software output control using
 * FTM_DRV_SetSoftwareOutputChannelControl.
 * @Note: When the PWM signal is configured with LOW/HIGH polarity on the channel (n).
 * It should be set the safe state as LOW level state. However, We will have an issue
 * with COMP bit is zero and CH(n)OCV is HIGH and CH(n+1)OCV is LOW.in the independent channel configuration.
 * Code configuration:
 * {
 *      .polarity = FTM_POLARITY_HIGH,
 *      .safeState = FTM_POLARITY_LOW,
 *      .enableSecondChannelOutput = true,
 *      .secondChannelPolarity = FTM_MAIN_DUPLICATED,
 * }
 *
 * Workaround: Configure the safe state as HIGH level state. The expected output will be correctly controlling
 * Should change configuration as following:
 * {
 *      .polarity = FTM_POLARITY_HIGH,
 *      .safeState = FTM_HIGH_STATE,
 *      .enableSecondChannelOutput = true,
 *      .secondChannelPolarity = FTM_MAIN_DUPLICATED,
 * }
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsValues The values which will be software configured for channels.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance,
                                    uint8_t channelsValues,
                                    bool softwareTrigger);

/*!
 * @brief This function will configure which output channel can be software controlled.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsMask The mask which will configure the channels which can be software controlled.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance,
                                                 uint8_t channelsMask,
                                                 bool softwareTrigger);

/*!
 * @brief This function will control list of channels by software to force
 * the output to specified value.
 * Despite the odd channels are configured as HIGH/LOW, they will be inverted in the following
 * configuration: COMP bit = 1 and CH(n)OCV and CH(n+1)OCV are HIGH. Please check software
 * output control behavior chapter from reference manual.
 * @Note: When the PWM signal is configured with LOW/HIGH polarity on the channel (n).
 * It should be set the safe state as LOW level state. However, We will have an issue
 * with COMP bit is zero and CH(n)OCV is HIGH and CH(n+1)OCV is LOW.in the independent channel configuration.
 * Code configuration:
 * {
 *      .polarity = FTM_POLARITY_HIGH,
 *      .safeState = FTM_POLARITY_LOW,
 *      .enableSecondChannelOutput = true,
 *      .secondChannelPolarity = FTM_MAIN_DUPLICATED,
 * }
 *
 * Workaround: Configure the safe state as HIGH level state. The expected output will be correctly controlling
 * Should change configuration as following:
 * {
 *      .polarity = FTM_POLARITY_HIGH,
 *      .safeState = FTM_HIGH_STATE,
 *      .enableSecondChannelOutput = true,
 *      .secondChannelPolarity = FTM_MAIN_DUPLICATED,
 * }
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelMask The mask which will configure the channels which can be software controlled.
 * @param [in] channelValueMask The values which will be software configured for channels.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetAllChnSoftwareOutputControl(uint32_t instance,
                                                uint8_t channelMask,
                                                uint8_t channelValueMask,
                                                bool softwareTrigger);

/*!
 * @brief This function will configure if the second channel of a pair will be inverted or not.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsPairMask The mask which will configure which channel pair will invert the second channel.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetInvertingControl(uint32_t instance,
                                     uint8_t channelsPairMask,
                                     bool softwareTrigger);

/*!
 * @brief This function configure the maximum counter value.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] counterValue Maximum counter value
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetModuloCounterValue(uint32_t instance,
                                       uint16_t counterValue,
                                       bool softwareTrigger);

/*!
 * @brief This function will set the channel edge or level on the selection
 * of the channel mode.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channel The channel number.
 * @param [in] level The level or edge selection for channel mode.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_SetOutputlevel(uint32_t instance,
                                uint8_t channel,
                                uint8_t level);

/*!
 * @brief This function configures sync mechanism for some FTM registers (MOD, CNINT, HCR,
 *          CnV, OUTMASK, INVCTRL, SWOCTRL).
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] param The sync configuration structure.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetSync(uint32_t instance,
                         const ftm_pwm_sync_t * param);

/*!
 * @brief This function is used to configure a trigger source for FTM instance.
 * This allow a hardware trigger input which can be used in PWM synchronization.
 * Note that the hardware trigger is implemented only on trigger 1 for each instance.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_GenerateHardwareTrigger(uint32_t instance);

/*!
 * @brief This function will enable the generation a list of interrupts.
 * It includes the FTM overflow interrupts, the reload point interrupt, the fault
 * interrupt and the channel (n) interrupt.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] interruptMask The mask of interrupt. This is a logical OR of members of the
 *            enumeration ::ftm_interrupt_option_t
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_EnableInterrupts(uint32_t instance,
                                  uint32_t interruptMask);

/*!
 * @brief This function is used to disable some interrupts.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] interruptMask The mask of interrupt. This is a logical OR of members of the
 *            enumeration ::ftm_interrupt_option_t
 */
void FTM_DRV_DisableInterrupts(uint32_t instance,
                               uint32_t interruptMask);

/*!
 * @brief This function will get the enabled FTM interrupts.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::ftm_interrupt_option_t
 */
uint32_t FTM_DRV_GetEnabledInterrupts(uint32_t instance);

/*!
 * @brief This function will get the FTM status flags.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::ftm_status_flag_t
 */
uint32_t FTM_DRV_GetStatusFlags(uint32_t instance);

/*!
 * @brief This function is used to clear the FTM status flags.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] flagMask The status flags to clear. This is a logical OR of members of the
 *            enumeration ::ftm_status_flag_t
 */
void FTM_DRV_ClearStatusFlags(uint32_t instance,
                              uint32_t flagMask);

/*!
 * @brief Retrieves the frequency of the clock source feeding the FTM counter.
 *
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled
 *
 * @param [in] instance The FTM peripheral instance number.
 * @return The frequency of the clock source running the FTM counter (0 if counter is disabled)
 */
uint32_t FTM_DRV_GetFrequency(uint32_t instance);

/*!
 * @brief This function is used to covert the given frequency to period in ticks
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] freqencyHz Frequency value in Hz.
 *
 * @return The value in ticks of the frequency
 */
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                          uint32_t freqencyHz);

/*!
 * @brief This function will allow the FTM to restart the counter to
 * its initial counting value in the register.
 * Note that the configuration is set in the FTM_DRV_SetSync() function to make
 * sure that the FTM registers are updated by software trigger or hardware trigger.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] softwareTrigger Selects the software trigger or hardware trigger to update COUNT register.
 *            - true: A software trigger is generate to update register
 *            - false: A software trigger is not implemented and need to update later or
 *            select a hardware trigger and waiting an external trigger for updating register.
 */
status_t FTM_DRV_CounterReset(uint32_t instance,
                              bool softwareTrigger);

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup ftm_common */

#endif /* FTM_COMMON_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
