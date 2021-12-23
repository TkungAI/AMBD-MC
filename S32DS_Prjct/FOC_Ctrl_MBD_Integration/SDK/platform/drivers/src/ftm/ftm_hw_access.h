/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * @file ftm_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros defined are used to define features for each driver, so this might be reported
 * when the analysis is made only on one driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * This macro is needed in creating a common name for any IP.
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
 */

#ifndef FTM_HW_ACCESS_H
#define FTM_HW_ACCESS_H

#include <stdbool.h>
#include <stddef.h>
#include "status.h"
#include "device_registers.h"
#include "interrupt_manager.h"
#include "clock_manager.h"
#include "ftm_common.h"


/*!
 * @defgroup ftm_hw_access FTM HW ACCESS
 * @brief FlexTimer Module Hardware Abstraction Level.
 * FTM HW ACCESS provides low level APIs for reading and writing to all hardware features
 * of the FlexTimer module.
 * @{
 */

/*
 * S32K14x FTM
 *
 * FlexTimer Module
 *
 * Registers defined in this header file:
 * - FTM_SC - Status And Control
 * - FTM_CNT - Counter
 * - FTM_MOD - Modulo
 * - FTM_C0SC - Channel (n) Status And Control
 * - FTM_C0V - Channel (n) Value
 * - FTM_C1SC - Channel (n) Status And Control
 * - FTM_C1V - Channel (n) Value
 * - FTM_C2SC - Channel (n) Status And Control
 * - FTM_C2V - Channel (n) Value
 * - FTM_C3SC - Channel (n) Status And Control
 * - FTM_C3V - Channel (n) Value
 * - FTM_C4SC - Channel (n) Status And Control
 * - FTM_C4V - Channel (n) Value
 * - FTM_C5SC - Channel (n) Status And Control
 * - FTM_C5V - Channel (n) Value
 * - FTM_C6SC - Channel (n) Status And Control
 * - FTM_C6V - Channel (n) Value
 * - FTM_C7SC - Channel (n) Status And Control
 * - FTM_C7V - Channel (n) Value
 * - FTM_CNTIN - Counter Initial Value
 * - FTM_STATUS - Capture And Compare Status
 * - FTM_MODE - Features Mode Selection
 * - FTM_SYNC - Synchronization
 * - FTM_OUTINIT - Initial State For Channels Output
 * - FTM_OUTMASK - Output Mask
 * - FTM_COMBINE - Function For Linked Channels
 * - FTM_DEADTIME - Dead-time Insertion Control
 * - FTM_EXTTRIG - FTM External Trigger
 * - FTM_POL - Channels Polarity
 * - FTM_FMS - Fault Mode Status
 * - FTM_FILTER - Input Capture Filter Control
 * - FTM_FLTCTRL - Fault Control
 * - FTM_QDCTRL - Quadrature Decoder Control And Status
 * - FTM_CONF - Configuration
 * - FTM_FLTPOL - FTM Fault Input Polarity
 * - FTM_SYNCONF - Synchronization Configuration
 * - FTM_INVCTRL - FTM Inverting Control
 * - FTM_SWOCTRL - FTM Software Output Control
 * - FTM_PWMLOAD - FTM PWM Load
 * - FTM_HCR - Half Cycle Register
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Sets the value for the half cycle reload register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The 16 bit counter value
 */
static inline void FTM_DRV_SetHalfCycleValue(FTM_Type * const ftmBase,
                                             uint16_t value)
{
    ((ftmBase)->HCR) = value;
}

/*!
 * @brief Sets the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] clock The FTM peripheral clock selection
 *            - 00: No clock
 *            - 01: system clock
 *            - 10: fixed clock
 *            - 11: External clock
 */
static inline void FTM_DRV_SetClockSource(FTM_Type * const ftmBase,
                                          ftm_clock_source_t clock)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CLKS_MASK, FTM_SC_CLKS(clock));
}

/*!
 * @brief Sets the FTM clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] ps The FTM peripheral clock pre-scale divider
 */
static inline void FTM_DRV_SetClockPs(FTM_Type * const ftmBase,
                                      ftm_clock_ps_t ps)
{
    FTM_RMW_SC(ftmBase, FTM_SC_PS_MASK, FTM_SC_PS(ps));
}

/*!
 * @brief Reads the FTM clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM clock pre-scale divider
 */
static inline uint8_t FTM_DRV_GetClockPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_PS_MASK) >> FTM_SC_PS_SHIFT);
}

/*!
 * @brief  Enables the FTM peripheral timer overflow interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] state - true : Overflow interrupt enabled
 *                  - false: Overflow interrupt disabled
 */
static inline void FTM_DRV_SetTimerOverflowInt(FTM_Type * const ftmBase,
                                               bool state)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOIE_MASK, FTM_SC_TOIE(state));
}

/*!
 * @brief Enable PWM channel Outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM channel

 */
static inline void FTM_DRV_EnablePwmChannelOutputs(FTM_Type * const ftmBase,
                                                   uint8_t channel)
{
    FTM_RMW_SC(ftmBase, (1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)), (1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
}

/*!
 * @brief Disable PWM channel Outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM channel
 */
static inline void FTM_DRV_DisablePwmChannelOutputs(FTM_Type * const ftmBase,
                                                    uint8_t channel)
{
    uint32_t regValue = ((ftmBase)->SC);
    regValue = regValue & (~(1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
    ((ftmBase)->SC) = (regValue);
}

/*!
 * @brief Clears the timer overflow interrupt flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearTimerOverflow(FTM_Type * const ftmBase)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOF_MASK, FTM_SC_TOF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->SC;
#endif
}

/*!
 * @brief Reads the bit that controls enabling the FTM timer overflow interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of Timer Overflow Interrupt
 *         - true : Overflow interrupt is enabled
 *         - false: Overflow interrupt is disabled
 */
static inline bool FTM_DRV_IsOverflowIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_TOIE_MASK) >> FTM_SC_TOIE_SHIFT) != 0U;
}

/*!
 * @brief Returns the FTM peripheral timer overflow interrupt flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of Timer Overflow Flag
 *         - true : FTM counter has overflowed
 *         - false: FTM counter has not overflowed
 */
static inline bool FTM_DRV_HasTimerOverflowed(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_TOF_MASK) >> FTM_SC_TOF_SHIFT) != 0U;
}

/*!
 * @brief Sets the FTM count direction bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode The Center-Aligned PWM selection
 *                 - 1U: Up counting mode
 *                 - 0U: Up down counting mode
 */
static inline void FTM_DRV_SetCpwms(FTM_Type * const ftmBase,
                                    bool mode)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CPWMS_MASK, FTM_SC_CPWMS(mode));
}

/*!
 * @brief Gets the FTM count direction bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The Center-Aligned PWM selection
 *         - 1U: Up counting mode
 *         - 0U: Up down counting mode
 */
static inline bool FTM_DRV_GetCpwms(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_CPWMS_MASK) >> FTM_SC_CPWMS_SHIFT) != 0U;
}

/*!
 * @brief Set the FTM reload interrupt enable.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable - true : Reload interrupt is  enabled
 *                   - false: Reload interrupt is disabled
 */
static inline void FTM_DRV_SetReIntEnabledCmd(FTM_Type * const ftmBase,
                                              bool enable)
{
    FTM_RMW_SC(ftmBase, FTM_SC_RIE_MASK, FTM_SC_RIE(enable));
}

/*!
 * @brief Get the state whether the FTM counter reached a reload point.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of reload point
 *         - true : FTM counter reached a reload point
 *         - false: FTM counter did not reach a reload point
 */
static inline bool FTM_DRV_GetReloadFlag(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_RF_MASK) >> FTM_SC_RF_SHIFT) != 0U;
}

/*!
 * @brief Clears the reload flag bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearReloadFlag(FTM_Type * const ftmBase)
{
    FTM_RMW_SC(ftmBase, FTM_SC_RF_MASK, FTM_SC_RF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->SC;
#endif
}

/*!
 * @brief Return true/false whether the reload interrupt was enabled or not
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline bool FTM_DRV_IsReloadIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_RIE_MASK) >> FTM_SC_RIE_SHIFT) != 0U;
}

/*!
 * @brief Reads the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM clock source selection
 *          - 00: No clock
 *          - 01: system clock
 *          - 10: fixed clock
 *          - 11: External clock
 */
static inline uint8_t FTM_DRV_GetClockSource(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_CLKS_MASK) >> FTM_SC_CLKS_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral current counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The FTM timer counter value to be set
 */
static inline void FTM_DRV_SetCounter(FTM_Type * const ftmBase,
                                      uint16_t value)
{
    FTM_RMW_CNT(ftmBase, FTM_CNT_COUNT_MASK, FTM_CNT_COUNT(value));
}

/*!
 * @brief Sets the FTM peripheral timer modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the timer modulo
 */
static inline void FTM_DRV_SetMod(FTM_Type * const ftmBase,
                                  uint16_t value)
{
    FTM_RMW_MOD(ftmBase, FTM_MOD_MOD_MASK, FTM_MOD_MOD(value));
}

/*!
 * @brief Sets the FTM peripheral timer counter initial value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value initial value to be set
 */
static inline void FTM_DRV_SetCounterInitVal(FTM_Type * const ftmBase,
                                             uint16_t value)
{
    FTM_RMW_CNTIN(ftmBase, FTM_CNTIN_INIT_MASK, FTM_CNTIN_INIT(value));
}

/*!
 * @brief Sets the FTM peripheral timer channel mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] selection The mode to be set valid value MSnB:MSnA :00, 01, 10, 11
 */
static inline void FTM_DRV_SetChnMSnBAMode(FTM_Type * const ftmBase,
                                           uint8_t channel,
                                           uint8_t selection)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* write MSA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_MSA_MASK, FTM_CnSC_MSA((uint32_t)selection & 0x01U));

    /* write MSB bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_MSB_MASK, FTM_CnSC_MSB(((uint32_t)selection & 0x02U) >> 1U));
}

/*!
 * @brief Sets the FTM peripheral timer channel edge level.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] level ELSnB:ELSnA :00, 01, 10, 11
 */
static inline void FTM_DRV_SetChnEdgeLevel(FTM_Type * const ftmBase,
                                           uint8_t channel,
                                           uint8_t level)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* write ELSA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ELSA_MASK, FTM_CnSC_ELSA((uint32_t)level & 0x01U));

    /* write ELSB bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ELSB_MASK, FTM_CnSC_ELSB(((uint32_t)level & 0x02U) >> 1U));
}

/*!
 * @brief Enables the FTM peripheral timer channel(n) interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 */
static inline void FTM_DRV_EnableChnInt(FTM_Type * const ftmBase,
                                        uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(1U));
}

/*!
 * @brief Disables the FTM peripheral timer channel(n) interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 */
static inline void FTM_DRV_DisableChnInt(FTM_Type * const ftmBase,
                                         uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(0U));
}

/*!
 * @brief Clear the channel flag by writing a 0 to the CHF bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 */
static inline void FTM_DRV_ClearChnEventFlag(FTM_Type * const ftmBase,
                                             uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHF_MASK, FTM_CnSC_CHF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->CONTROLS[channel].CnSC;
#endif
}

/*FTM channel control*/
/*!
 * @brief Sets the FTM peripheral timer channel counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] value Counter value to be set
 */
static inline void FTM_DRV_SetChnCountVal(FTM_Type * const ftmBase,
                                          uint8_t channel,
                                          uint16_t value)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->CONTROLS[channel].CnV) = value;
}

/*!
 * @brief Get FTM channel(n) interrupt enabled or not.
 * @param[in] ftmBase FTM module base address
 * @param[in] channel The FTM peripheral channel number
 */
static inline bool FTM_DRV_IsChnIntEnabled(const FTM_Type * ftmBase,
                                           uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIE_MASK) != 0U;
}

/*!
 * @brief Returns whether any event for the FTM peripheral timer channel has occurred.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return State of channel flag
 *         - true : Event occurred
 *         - false: No event occurred.
 */
static inline bool FTM_DRV_HasChnEventOccurred(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHF_MASK) != 0U;
}

/*!
 * @brief Gets the FTM peripheral timer channel mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return The MSnB:MSnA mode value, will be 00, 01, 10, 11
 *
 */
static inline uint8_t FTM_DRV_GetChnMode(const FTM_Type * ftmBase,
                                         uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint8_t retValue;

    retValue = (uint8_t)((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSA_MASK) >> FTM_CnSC_MSA_SHIFT);

    retValue |= (uint8_t)(((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSB_MASK) >> FTM_CnSC_MSB_SHIFT) << 1U);

    return retValue;
}

/*!
 * @brief Writes the provided value to the OUTMASK register.
 *
 * This function will mask/unmask multiple channels.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] regVal Value to be written to the register
 */
static inline void FTM_DRV_SetOutmaskReg(FTM_Type * const ftmBase,
                                         uint32_t regVal)
{
    ((ftmBase)->OUTMASK) = regVal;
}

/*!
 * @brief Sets the FTM peripheral timer channel output polarity.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] polarity The polarity to be set
 *            - true : The channel polarity is active low
 *            - false  : The channel polarity is active high
 */
static inline void FTM_DRV_SetChnOutputPolarityCmd(FTM_Type * const ftmBase,
                                                   uint8_t channel,
                                                   bool polarity)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (true != polarity)
    {
        ((ftmBase)->POL) &= ~(1UL << channel);
    }
    else
    {
        ((ftmBase)->POL) |= (1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel fault input polarity.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] fltChannel The FTM peripheral channel number
 * @param[in] polarity The polarity to be set
 *                - false : The fault input polarity is active high
 *                - true: The fault input polarity is active low
 */
static inline void FTM_DRV_SetChnFaultInputPolarityCmd(FTM_Type * const ftmBase,
                                                       uint8_t fltChannel,
                                                       bool polarity)
{
    DEV_ASSERT(fltChannel < FTM_FEATURE_FAULT_CHANNELS);

    if (true == polarity)
    {
        ((ftmBase)->FLTPOL) &= ~(1UL << fltChannel);
    }
    else
    {
        ((ftmBase)->FLTPOL) |= (1UL << fltChannel);
    }
}

/*!
 * @brief Enables/disables the FTM peripheral timer fault interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] state Timer fault interrupt state
 *            - true : Fault control interrupt is enable
 *            - false: Fault control interrupt is disabled
 */
static inline void FTM_DRV_SetFaultInt(FTM_Type * const ftmBase,
                                       bool state)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(state));
}

/*!
 * @brief Return true/false whether the Fault interrupt was enabled or not
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline bool FTM_DRV_IsFaultIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->MODE & FTM_MODE_FAULTIE_MASK) >> FTM_MODE_FAULTIE_SHIFT) != 0U;
}

/*!
 * @brief Clears all fault interrupt flags that are active.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearFaultsIsr(FTM_Type * const ftmBase)
{
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF0_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF0(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF1_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF1(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF2_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF2(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF3_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF3(0U) | FTM_FMS_FAULTF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->FMS;
#endif
}

/*!
 * @brief Defines the FTM fault control mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Fault control mode value
 * - FTM_FAULT_CONTROL_DISABLED: Fault control disabled
 * - FTM_FAULT_CONTROL_MAN_EVEN: Fault control enabled for even channel (0, 2, 4, 6) and manual fault clearing.
 * - FTM_FAULT_CONTROL_MAN_ALL : Fault control enabled for all channels and manual fault clearing is enabled.
 * - FTM_FAULT_CONTROL_AUTO_ALL: Fault control enabled for all channels and automatic fault clearing is enabled.
 */
static inline void FTM_DRV_SetFaultControlMode(FTM_Type * const ftmBase,
                                               uint32_t mode)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTM_MASK, FTM_MODE_FAULTM(mode));
}

/*!
 * @brief Initializes the channels output.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Initialize the channels output
 *                   - true : The channels output is initialized according to the state of OUTINIT reg
 *                   - false: No effect
 */
static inline void FTM_DRV_SetInitChnOutputCmd(FTM_Type * const ftmBase,
                                               bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_INIT_MASK, FTM_MODE_INIT(enable));
}

/*!
 * @brief Gets the FTM detected fault input.
 *
 * This function reads the status for all fault inputs
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The fault byte
 *         - 0 : No fault condition was detected.
 *         - 1 : A fault condition was detected.
 */
static inline bool FTM_DRV_GetDetectedFaultInput(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_FAULTF_MASK) != 0U;
}

/*!
 * @brief Enables or disables the FTM write protection.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable The FTM write protection selection
 *                   - true : Write-protection is enabled
 *                   - false: Write-protection is disabled
 */
static inline void FTM_DRV_SetWriteProtectionCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    if (enable)
    {
        ftmBase->FMS = (ftmBase->FMS & ~FTM_FMS_WPEN_MASK) | FTM_FMS_WPEN(1U);
    }
    else
    {
        ftmBase->MODE = (ftmBase->MODE & ~FTM_MODE_WPDIS_MASK) | FTM_MODE_WPDIS(1U);
    }
}

/*!
 * @brief Enables the FTM peripheral timer group.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable FTM mode selection
 *                   - true : All registers including FTM-specific registers are available
 *                   - false: Only the TPM-compatible registers are available
 */
static inline void FTM_DRV_Enable(FTM_Type * const ftmBase,
                                  bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FTMEN_MASK, FTM_MODE_FTMEN(enable));
}

/*!
 * @brief Sets the FTM peripheral timer sync mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable PWM synchronization mode
 *                   - false: No restriction both software and hardware triggers can be used
 *                   - true : Software trigger can only be used for MOD and CnV synchronization,
 *                            hardware trigger only for OUTMASK and FTM counter synchronization.
 */
static inline void FTM_DRV_SetPwmSyncMode(FTM_Type * const ftmBase,
                                          bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_PWMSYNC_MASK, FTM_MODE_PWMSYNC(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] enable Software trigger selection
 *                   - true : Software trigger is selected
 *                   - false: Software trigger is not selected
 */
static inline void FTM_DRV_SetSoftwareTriggerCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SWSYNC_MASK, FTM_SYNC_SWSYNC(enable));
}

/*!
 * @brief Sets the FTM hardware synchronization trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] trigger_num Number of trigger
 *                        - 0U: trigger 0
 *                        - 1U: trigger 1
 *                        - 2U: trigger 2
 * @param[in] enable State of trigger
 *                   - true : Enable hardware trigger from field trigger_num for PWM synchronization
 *                   - false: Disable hardware trigger from field trigger_num for PWM synchronization
 */
static inline void FTM_DRV_SetHardwareSyncTriggerSrc(FTM_Type * const ftmBase,
                                                     uint8_t trigger_num,
                                                     bool enable)
{
    DEV_ASSERT(trigger_num < 3U);

    if (enable)
    {
        ((ftmBase)->SYNC) |= ((uint32_t)(FTM_SYNC_TRIG0_MASK) << trigger_num);
    }
    else
    {
        ((ftmBase)->SYNC) &= ~((uint32_t)(FTM_SYNC_TRIG0_MASK) << trigger_num);
    }
}

/*!
 * @brief Determines when the OUTMASK register is updated with the value of its buffer.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Output Mask synchronization selection
 *                   - true : OUTMASK register is updated only by PWM synchronization
 *                   - false: OUTMASK register is updated in all rising edges of the system clock
 */
static inline void FTM_DRV_SetOutmaskPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SYNCHOM_MASK, FTM_SYNC_SYNCHOM(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer maximum loading points.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Maximum loading point selection
 *                   - true : To enable maximum loading point
 *                   - false: To disable
 */
static inline void FTM_DRV_SetMaxLoadingCmd(FTM_Type * const ftmBase,
                                            bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMAX_MASK, FTM_SYNC_CNTMAX(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer minimum loading points.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Minimum loading point selection
 *                   - true : To enable minimum loading point
 *                   - false: To disable
 */
static inline void FTM_DRV_SetMinLoadingCmd(FTM_Type * const ftmBase,
                                            bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMIN_MASK, FTM_SYNC_CNTMIN(enable));
}

/*!
 * @brief Enables the FTM peripheral timer channel modified combine mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair outputs modified combine
 *                   - true : To enable modified combine
 *                   - false: To disable modified combine
 */
static inline void FTM_DRV_SetDualChnMofCombineCmd(FTM_Type * const ftmBase,
                                                   uint8_t chnlPairNum,
                                                   bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_MCOMBINE0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_MCOMBINE0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer channel pair fault control.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair fault control
 *                   - true : To enable fault control
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnFaultCmd(FTM_Type * const ftmBase,
                                              uint8_t chnlPairNum,
                                              bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_FAULTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_FAULTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair counter PWM sync.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair counter PWM sync
 *                   - true : To enable PWM synchronization
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnPwmSyncCmd(FTM_Type * const ftmBase,
                                                uint8_t chnlPairNum,
                                                bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_SYNCEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_SYNCEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disabled the FTM peripheral timer channel pair deadtime insertion.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair deadtime insertion
 *                   - true : To enable deadtime insertion
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnDeadtimeCmd(FTM_Type * const ftmBase,
                                                 uint8_t chnlPairNum,
                                                 bool enable)
{
    DEV_ASSERT(chnlPairNum < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel dual edge capture.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel dual edge capture
 *                   - true : To enable dual edge capture mode
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnDecapCmd(FTM_Type * const ftmBase,
                                              uint8_t chnlPairNum,
                                              bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DECAP0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DECAP0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of dual edge capture mode
 *                   - true : To enable dual edge capture
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualEdgeCaptureCmd(FTM_Type * const ftmBase,
                                                 uint8_t chnlPairNum,
                                                 bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Dual edge capture mode status
 *         - true : To enable dual edge capture
 *         - false: To disable
 */
static inline bool FTM_DRV_GetDualEdgeCaptureBit(const FTM_Type * ftmBase,
                                                 uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & ((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output complement mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] polarity State of channel pair output complement mode
 *            - true : The channel (n+1) output is the complement of the channel (n) output
 *            - false: The channel (n+1) output is the same as the channel (n) output
 */
static inline void FTM_DRV_SetDualChnCompCmd(FTM_Type * const ftmBase,
                                             uint8_t chnlPairNum,
                                             bool polarity)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (polarity == true)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMP0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMP0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output combine mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair output combine mode
 *                   - true : Channels pair are combined
 *                   - false: Channels pair are independent
 */
static inline void FTM_DRV_SetDualChnCombineCmd(FTM_Type * const ftmBase,
                                                uint8_t chnlPairNum,
                                                bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Verify if an channels pair is used in combine mode or not.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Channel pair output combine mode status
 *         - true : Channels pair are combined
 *         - false: Channels pair are independent
 */
static inline bool FTM_DRV_GetDualChnCombineCmd(const FTM_Type * ftmBase,
                                                uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & (FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Verify if an channels pair is used in the modified combine mode or not.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Channel pair output combine mode status
 *         - true : Channels pair are combined
 *         - false: Channels pair are independent
 */
static inline bool FTM_DRV_GetDualChnMofCombineCmd(const FTM_Type * ftmBase,
                                                   uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & (FTM_COMBINE_MCOMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Sets the FTM extended dead-time value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The FTM peripheral extend pre-scale divider
 */
static inline void FTM_DRV_SetExtDeadtimeValue(FTM_Type * const ftmBase,
                                               uint8_t value)
{
    DEV_ASSERT(value < 16U);

    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVALEX_MASK, FTM_DEADTIME_DTVALEX(value));
}

/*!
 * @brief Sets the FTM dead time divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] divider The FTM peripheral pre-scaler divider
 *                    - FTM_DEADTIME_DIVID_BY_1 : Divide by 1
 *                    - FTM_DEADTIME_DIVID_BY_4 : Divide by 4
 *                    - FTM_DEADTIME_DIVID_BY_16: Divide by 16
 */
static inline void FTM_DRV_SetDeadtimePrescale(FTM_Type * const ftmBase,
                                               ftm_deadtime_ps_t divider)
{
    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTPS_MASK, FTM_DEADTIME_DTPS((uint8_t)divider));
}

/*!
 * @brief Sets the FTM deadtime value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] count The FTM peripheral pre-scaler divider
 *                  - 0U : no counts inserted
 *                  - 1U : 1 count is inserted
 *                  - 2U : 2 count is inserted
 *                  - ... up to a possible 63 counts
 */
static inline void FTM_DRV_SetDeadtimeCount(FTM_Type * const ftmBase,
                                            uint8_t count)
{
    DEV_ASSERT(count < 64U);

    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVAL_MASK, FTM_DEADTIME_DTVAL(count));
}

/*FTM external trigger */
/*!
 * @brief Enables or disables the generation of the trigger when the FTM counter is equal
 * to the CNTIN register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of initialization trigger
 *                   - true : To enable
 *                   - false: To disable
 */
static inline void FTM_DRV_SetInitTriggerCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    ftmBase->EXTTRIG = (ftmBase->EXTTRIG & ~FTM_EXTTRIG_INITTRIGEN_MASK) | FTM_EXTTRIG_INITTRIGEN(enable);
}

/*!
 * @brief Checks whether any channel trigger event has occurred.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @return Channel trigger status
 *         - true : If there is a channel trigger event
 *         - false: If not.
 */
static inline bool FTM_DRV_IsChnTriggerGenerated(const FTM_Type * ftmBase)
{
    return (ftmBase->EXTTRIG & FTM_EXTTRIG_TRIGF_MASK) != 0U;
}

/*!
 * @brief Clear the channel trigger flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearChnTriggerFlag(FTM_Type * const ftmBase)
{
    FTM_RMW_EXTTRIG_REG(ftmBase, FTM_EXTTRIG_TRIGF_MASK, FTM_EXTTRIG_TRIGF(0UL));
}

/* Quadrature decoder control */
/*!
 * @brief Enables the channel quadrature decoder.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of channel quadrature decoder
 *                  - true : To enable
 *                  - false: To disable
 */
static inline void FTM_DRV_SetQuadDecoderCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_QUADEN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_QUADEN_SHIFT);
    }
}

/*!
 * @brief Enables or disables the phase A input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of phase A input filter
 *                   - true : Enables the phase input filter
 *                   - false: Disables the filter
 */
static inline void FTM_DRV_SetQuadPhaseAFilterCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
}

/*!
 * @brief Selects polarity for the quadrature decode phase A input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Phase A input polarity selection
 *                - 0U: Normal polarity
 *                - 1U: Inverted polarity
 */
static inline void FTM_DRV_SetQuadPhaseAPolarity(FTM_Type * const ftmBase,
                                                 uint8_t mode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHAPOL_MASK, FTM_QDCTRL_PHAPOL(mode));
}

/*!
 * @brief Selects polarity for the quadrature decode phase B input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Phase B input polarity selection
 *                - 0U: Normal polarity
 *                - 1U: Inverted polarity
 */
static inline void FTM_DRV_SetQuadPhaseBPolarity(FTM_Type * const ftmBase,
                                                 uint8_t mode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHBPOL_MASK, FTM_QDCTRL_PHBPOL(mode));
}

/*!
 * @brief Sets the encoding mode used in quadrature decoding mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] quadMode Quadrature decoder mode selection
 *                     - 0U: Phase A and Phase B encoding mode
 *                     - 1U: Count and direction encoding mode
 */
static inline void FTM_DRV_SetQuadMode(FTM_Type * const ftmBase,
                                       uint8_t quadMode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_QUADMODE_MASK, FTM_QDCTRL_QUADMODE(quadMode));
}

/*!
 * @brief Gets the FTM counter direction in quadrature mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The counting direction
 *         - 1U: if counting direction is increasing
 *         - 0U: if counting direction is decreasing
 */
static inline bool FTM_DRV_GetQuadDir(const FTM_Type * ftmBase)
{
    return (ftmBase->QDCTRL & FTM_QDCTRL_QUADIR_MASK) != 0U;
}

/*!
 * @brief Gets the Timer overflow direction in quadrature mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The timer overflow direction
 *         - 1U: if TOF bit was set on the top of counting
 *         - 0U: if TOF bit was set on the bottom of counting
 */
static inline bool FTM_DRV_GetQuadTimerOverflowDir(const FTM_Type * ftmBase)
{
    return (ftmBase->QDCTRL & FTM_QDCTRL_TOFDIR_MASK) != 0U;
}

/*!
 * @brief Enables or disables the phase B input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of phase B input filter
 *                   - true : Enables the phase input filter
 *                   - false: Disables the filter
 */
static inline void FTM_DRV_SetQuadPhaseBFilterCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
}

/*!
 * @brief Sets the fault input filter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value Fault input filter value
 */
static inline void FTM_DRV_SetFaultInputFilterVal(FTM_Type * const ftmBase,
                                                  uint32_t value)
{
    FTM_RMW_FLTCTRL(ftmBase, FTM_FLTCTRL_FFVAL_MASK, FTM_FLTCTRL_FFVAL(value));
}

/*!
 * @brief Enables or disables the fault input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] inputNum Fault input to be configured, valid value 0, 1, 2, 3
 * @param[in] enable State of fault input filter
 *                   - true : To enable fault input filter
 *                   - false: To disable fault input filter
 */
static inline void FTM_DRV_SetFaultInputFilterCmd(FTM_Type * const ftmBase,
                                                  uint8_t inputNum,
                                                  bool enable)
{
    DEV_ASSERT(inputNum < CHAN4_IDX);

    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
}

/*!
 * @brief Clears the entire content value of the Fault control register.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearFaultControl(FTM_Type * const ftmBase)
{
    ((ftmBase)->FLTCTRL) =  0U;
}

/*!
 * @brief Enables or disables the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] inputNum fault input to be configured, valid value 0, 1, 2, 3
 * @param[in] enable State of fault input
 *                   - true : To enable fault input
 *                   - false: To disable fault input
 */
static inline void FTM_DRV_SetFaultInputCmd(FTM_Type * const ftmBase,
                                            uint8_t inputNum,
                                            bool enable)
{
    DEV_ASSERT(inputNum < CHAN4_IDX);

    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << inputNum);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << inputNum);
    }
}

/*!
 * @brief Configures the behavior of the PWM outputs when a fault is detected
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of fault output
 *                   - true : Output pins are set tri-state,
 *                   - false: Pins are set to a safe state determined by POL bits
 */
static inline void FTM_DRV_SetPwmFaultBehavior(FTM_Type * const ftmBase,
                                               bool enable)
{
    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << FTM_FLTCTRL_FSTATE_SHIFT);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << FTM_FLTCTRL_FSTATE_SHIFT);
    }
}

/*!
 * @brief Writes the provided value to the Inverting control register.
 *
 * This function is enable/disable inverting control on multiple channel pairs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] regVal Value to be written to the register
 */
static inline void FTM_DRV_SetInvctrlReg(FTM_Type * const ftmBase,
                                         uint32_t regVal)
{
    ((ftmBase)->INVCTRL) = regVal;
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.The
 * main difference between this function and FTM_HAL_SetChnSoftwareCtrlCmd
 * is that this can configure all the channels in the same time.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelsMask Channels to be enabled or disabled
 */
static inline void FTM_DRV_SetAllChnSoftwareCtrlCmd(FTM_Type * const ftmBase,
                                                    uint8_t channelsMask)
{
    uint32_t mask = FTM_SWOCTRL_CH0OC_MASK | FTM_SWOCTRL_CH1OC_MASK | FTM_SWOCTRL_CH2OC_MASK |
                    FTM_SWOCTRL_CH3OC_MASK | FTM_SWOCTRL_CH4OC_MASK | FTM_SWOCTRL_CH5OC_MASK |
                    FTM_SWOCTRL_CH6OC_MASK | FTM_SWOCTRL_CH7OC_MASK;
    ((ftmBase)->SWOCTRL) = (((ftmBase)->SWOCTRL) & (~(mask))) | channelsMask;
}

/*!
 * @brief Sets the channel software output control value.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] channelsValues The values which will overwrite the output channels
 */
static inline void FTM_DRV_SetAllChnSoftwareCtrlVal(FTM_Type * const ftmBase,
                                                    uint8_t channelsValues)
{
    uint32_t mask = FTM_SWOCTRL_CH0OCV_MASK | FTM_SWOCTRL_CH1OCV_MASK | FTM_SWOCTRL_CH2OCV_MASK |
                        FTM_SWOCTRL_CH3OCV_MASK | FTM_SWOCTRL_CH4OCV_MASK | FTM_SWOCTRL_CH5OCV_MASK |
                        FTM_SWOCTRL_CH6OCV_MASK | FTM_SWOCTRL_CH7OCV_MASK;
   ((ftmBase)->SWOCTRL) = (((ftmBase)->SWOCTRL) & (~(mask))) | ((uint32_t)channelsValues << FTM_SWOCTRL_CH0OCV_SHIFT);
}

/*!
 * @brief Enable and Force the software control of channels output at once.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] value The values which enables and force the software control of channels output
 */
static inline void FTM_DRV_SoftwareOutputControl(FTM_Type * const ftmBase,
                                                 uint16_t value)
{
   ((ftmBase)->SWOCTRL) = value;
}

/*!
 * @brief Sets the BDM mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val The FTM behavior in BDM mode
 *                - FTM_BDM_MODE_00: FTM counter stopped, CH(n)F bit can be set, FTM channels
 *                                   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
 *                                   the register buffers
 *                - FTM_BDM_MODE_01: FTM counter stopped, CH(n)F bit is not set, FTM channels
 *                                   outputs are forced to their safe value , writes to MOD,CNTIN and
 *                                   C(n)V registers bypass the register buffers
 *                - FTM_BDM_MODE_10: FTM counter stopped, CH(n)F bit is not set, FTM channels
 *                                   outputs are frozen when chip enters in BDM mode, writes to MOD,
 *                                   CNTIN and C(n)V registers bypass the register buffers
 *                - FTM_BDM_MODE_11: FTM counter in functional mode, CH(n)F bit can be set,
 *                                   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
 *                                   registers is in fully functional mode
 */
static inline void FTM_DRV_SetBdmMode(FTM_Type * const ftmBase,
                                      ftm_bdm_mode_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_BDMMODE_MASK, FTM_CONF_BDMMODE(val));
}

/*FTM Synchronization configuration*/
/*!
 * @brief Sets the sync mode for the FTM SWOCTRL register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of software output control synchronization
 *                   - true : The hardware trigger activates SWOCTRL register sync
 *                   - false: The hardware trigger does not activate SWOCTRL register sync
 */
static inline void FTM_DRV_SetSwoctrlHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWSOC_MASK) | FTM_SYNCONF_HWSOC(enable);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of inverting control synchronization
 *                   - true : The hardware trigger activates INVCTRL register sync
 *                   - false: The hardware trigger does not activate INVCTRL register sync
 */
static inline void FTM_DRV_SetInvctrlHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWINVC_MASK) | FTM_SYNCONF_HWINVC(enable);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of output mask synchronization
 *                   - true : The hardware trigger activates OUTMASK register sync
 *                   - false: The hardware trigger does not activate OUTMASK register sync
 */
static inline void FTM_DRV_SetOutmaskHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWOM_MASK) | FTM_SYNCONF_HWOM(enable);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of registers synchronization
 *                   - true : The hardware trigger activates  MOD, HCR, CNTIN, and CV registers sync
 *                   - false: The hardware trigger does not activate MOD, HCR, CNTIN, and CV registers sync
 */
static inline void FTM_DRV_SetModCntinCvHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                            bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWWRBUF_MASK) | FTM_SYNCONF_HWWRBUF(enable);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of FTM counter synchronization
 *                   - true : The hardware trigger activates FTM counter sync
 *                   - false: The hardware trigger does not activate FTM counter sync
 */
static inline void FTM_DRV_SetCounterHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWRSTCNT_MASK) | FTM_SYNCONF_HWRSTCNT(enable);
}

/*!
 * @brief Sets sync mode for FTM SWOCTRL register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of software output control synchronization
 *                   - true : The software trigger activates SWOCTRL register sync
 *                   - false: The software trigger does not activate SWOCTRL register sync
 */
static inline void FTM_DRV_SetSwoctrlSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWSOC_MASK) | FTM_SYNCONF_SWSOC(enable);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of State of inverting control synchronization
 *                   - true : The software trigger activates INVCTRL register sync
 *                   - false: The software trigger does not activate INVCTRL register sync
 */
static inline void FTM_DRV_SetInvctrlSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWINVC_MASK) | FTM_SYNCONF_SWINVC(enable);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of output mask synchronization
 *                   - true : The software trigger activates OUTMASK register sync
 *                   - false: The software trigger does not activate OUTMASK register sync
 */
static inline void FTM_DRV_SetOutmaskSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWOM_MASK) | FTM_SYNCONF_SWOM(enable);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of registers synchronization
 *                   - true : The software trigger activates FTM MOD, CNTIN and CV registers sync
 *                   - false: The software trigger does not activate FTM MOD, CNTIN and CV registers sync
 */
static inline void FTM_DRV_SetModCntinCvSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                            bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWWRBUF_MASK) | FTM_SYNCONF_SWWRBUF(enable);
}

/*!
 * @brief Sets hardware trigger mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of hardware trigger mode
 *                   - true : FTM does not clear the TRIGx bit when the hardware trigger j is detected
 *                   - false: FTM clears the TRIGx bit when the hardware trigger j is detected
 */
static inline void FTM_DRV_SetHwTriggerSyncModeCmd(FTM_Type * const ftmBase,
                                                   bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWTRIGMODE_MASK) | FTM_SYNCONF_HWTRIGMODE(enable);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] update_mode State of FTM counter synchronization
 *                   - true : The software trigger activates FTM counter sync
 *                   - false: The software trigger does not activate FTM counter sync
 */
static inline void FTM_DRV_SetCounterSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         ftm_pwm_sync_mode_t update_mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWRSTCNT_MASK) | FTM_SYNCONF_SWRSTCNT(update_mode);
}

/*!
 * @brief Sets the PWM synchronization mode to enhanced or legacy.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of synchronization mode
 *                   - true : Enhanced PWM synchronization is selected
 *                   - false: Legacy PWM synchronization is selected
 */
static inline void FTM_DRV_SetPwmSyncModeCmd(FTM_Type * const ftmBase,
                                             bool mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SYNCMODE_MASK) | FTM_SYNCONF_SYNCMODE(mode);
}

/*!
 * @brief Sets the SWOCTRL register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : SWOCTRL register is updated by PWM sync
 *                   - false: SWOCTRL register is updated at all rising edges of system clock
 */
static inline void FTM_DRV_SetSwoctrlPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWOC_MASK) | FTM_SYNCONF_SWOC(mode);
}

/*!
 * @brief Sets the INVCTRL register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : INVCTRL register is updated by PWM sync
 *                   - false: INVCTRL register is updated at all rising edges of system clock
 */
static inline void FTM_DRV_SetInvctrlPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_INVC_MASK) | FTM_SYNCONF_INVC(mode);
}

/*!
 * @brief Sets the CNTIN register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : CNTIN register is updated by PWM sync
 *                   - false: CNTIN register is updated at all rising edges of system clock
 */
static inline void FTM_DRV_SetCntinPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                  ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_CNTINC_MASK) | FTM_SYNCONF_CNTINC(mode);
}

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief Sets the modulo fractional value in the PWM dithering.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the fractional value for the modulo
 */
static inline void FTM_DRV_SetModFracVal(FTM_Type * const ftmBase,
                                         uint8_t value)
{
    FTM_RMW_MOD_MIRROR(ftmBase, FTM_MOD_MIRROR_FRACMOD_MASK, FTM_MOD_MIRROR_FRACMOD(value));
}

/*!
 * @brief Sets the channel (n) match fractional value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 * @param[in] value The value to be set to the channel (n) match fractional value
 */
static inline void FTM_DRV_SetChnMatchFracVal(FTM_Type * const ftmBase,
                                              uint8_t channel,
                                              uint8_t value)
{
    FTM_RMW_CnV_MIRROR(ftmBase, channel, FTM_CV_MIRROR_FRACVAL_MASK, FTM_CV_MIRROR_FRACVAL(value));
}
#endif

/*!
 * @brief Sets the sync bit as a hardware trigger source for FTM instance.
 *
 * @param[in] simBase The SIM base address pointer.
 * @param[in] instance The instance number of the FTM module.
 * @param[in] enable Enable/Disable the sync bit.
 */
static inline void FTM_DRV_SyncBit(SIM_Type * const simBase,
                                   uint32_t instance,
                                   bool enable)
{
    uint32_t instTemp = instance;

    if (instTemp > 3U)
    {
        instTemp = instance + 7U;
    }

    if (true == enable)
    {
        simBase->FTMOPT1 |= (1U << instTemp);
    }
    else
    {
        simBase->FTMOPT1 &= ~(1U << instTemp);
    }
}

/* DRV functionality */
/*!
 * @brief Resets the FTM registers. All the register use in the driver should be
 * reset to default value of each register.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
void FTM_DRV_Reset(FTM_Type * const ftmBase);

/*!
 * @brief Initializes the FTM. This function will enable the flexTimer module
 * and selects one pre-scale factor for the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] ftmClockPrescaler The FTM peripheral clock pre-scale divider
 */
void FTM_DRV_InitModule(FTM_Type * const ftmBase,
                        ftm_clock_ps_t ftmClockPrescaler);

/*!
 * @brief Enables or disables the generation of the FTM peripheral timer channel trigger when the
 * FTM counter is equal to its initial value
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enables the generation of the channel trigger
 *                   - true : The generation of the channel trigger is enabled
 *                   - false: The generation of the channel trigger is disabled
 */
void FTM_DRV_SetChnTriggerCmd(FTM_Type * const ftmBase,
                              uint8_t channel,
                              bool enable);

/*!
 * @brief Sets the FTM peripheral timer channel input capture filter value.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number, only 0,1,2,3, channel 4, 5,6, 7 don't have
 * @param[in] value Filter value to be set
 */
void FTM_DRV_SetChnInputCaptureFilter(FTM_Type * const ftmBase,
                                      uint8_t channel,
                                      uint8_t value);

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* FTM_HW_ACCESS_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
