/*
 * Copyright 2017 NXP.
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

#ifndef PDB_HW_ACCESS_H
#define PDB_HW_ACCESS_H

#include <stddef.h>
#include "pdb_driver.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type
 * The base addresses are provided as integers so they need to be cast to pointers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer
 * The base addresses are provided as integers so they need to be cast to pointers.
 */
 
/*! @file */

/*!
 * @addtogroup pdb_hw_access
 * @{
 */

/******************************************************************************
 * Definitions
 *****************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Resets the PDB registers to a known state.
 *
 * This function resets the PDB registers to a known state. This state is
 * defined in a reference manual and is power on reset value.
 *
 * @param[in] base Register base address for the module.
 */
void PDB_Init(PDB_Type * const base);

/*!
 * @brief Configure the PDB timer.
 *
 * This function configure the PDB's basic timer.
 *
 * @param[in] base Register base address for the module.
 * @param[in] configPtr Pointer to configuration structure, see to "pdb_timer_config_t".
 */
void PDB_ConfigTimer(PDB_Type * const base,
                     const pdb_timer_config_t * const configPtr);

/*!
 * @brief Triggers the PDB by software if enabled.
 *
 * If enabled, this function triggers the PDB by using software.
 *
 * @param[in] base Register base address for the module.
 */
static inline void PDB_SetSoftTriggerCmd(PDB_Type * const base)
{
    DEV_ASSERT(base != NULL);
    REG_BIT_SET32(&(base->SC), PDB_SC_SWTRIG_MASK);
}

/*!
 * @brief Switches on to enable the PDB module.
 *
 * This function switches on to enable the PDB module.
 *
 * @param[in] base Register base address for the module.
 */
static inline void PDB_Enable(PDB_Type * const base)
{
    DEV_ASSERT(base != NULL);
    REG_BIT_SET32(&(base->SC), PDB_SC_PDBEN_MASK);
}

/*!
 * @brief Switches to disable the PDB module.
 *
 * This function switches to disable the PDB module.
 *
 * @param[in] base Register base address for the module.
 */
static inline void PDB_Disable(PDB_Type * const base)
{
    DEV_ASSERT(base != NULL);
    REG_BIT_CLEAR32(&(base->SC), PDB_SC_PDBEN_MASK);
}

/*!
 * @brief Gets the PDB delay interrupt flag.
 *
 * This function gets the PDB delay interrupt flag.
 *
 * @param[in] base Register base address for the module.
 * @return[in] Flat status, true if the flag is set.
 */
static inline bool PDB_GetTimerIntFlag(PDB_Type const * const base)
{
    DEV_ASSERT(base != NULL);

    return 1U == ((base->SC & PDB_SC_PDBIF_MASK) >> PDB_SC_PDBIF_SHIFT);
}

/*!
 * @brief Clears the PDB delay interrupt flag.
 *
 * This function clears PDB delay interrupt flag.
 *
 * @param[in] base Register base address for the module.
 */
static inline void PDB_ClearTimerIntFlag(PDB_Type * const base)
{
    DEV_ASSERT(base != NULL);
    volatile uint32_t dummy_read;
    REG_BIT_CLEAR32(&(base->SC), PDB_SC_PDBIF_MASK);

    /* This read-after-write guarantees that the write to clear operation is completed,
     * for the case when memory write buffering is enabled. */
    dummy_read = REG_READ32(&(base->SC));
    (void)dummy_read;
}

/*!
 * @brief Loads the delay registers value for the PDB module.
 *
 * This function sets the LDOK bit and loads the delay registers value.
 * Writing one  to this bit updates the internal registers MOD, IDLY, CHnDLYm and
 * POyDLY with the values written to their buffers. The MOD, IDLY,
 * CHnDLYm and POyDLY take effect according to the load mode settings.
 *
 * After one is written to the LDOK bit, the values in the buffers of above mentioned registers
 * are not effective and cannot be written until the values in the
 * buffers are loaded into their internal registers.
 * The LDOK can be written only when the the PDB is enabled or as alone with it. It is
 * automatically cleared either when the values in the buffers are loaded into the
 * internal registers or when the PDB is disabled.
 *
 * @param[in] base Register base address for the module.
 */
static inline void PDB_SetLoadValuesCmd(PDB_Type * const base)
{
    DEV_ASSERT(base != NULL);
    REG_BIT_SET32(&(base->SC), PDB_SC_LDOK_MASK);
}

/*!
 * @brief Sets the modulus value for the PDB module.
 *
 * This function sets the modulus value for the PDB module.
 * When the counter reaches the setting value, it is automatically reset to zero.
 * When in continuous mode, the counter begins to increase
 * again.
 *
 * @param[in] base Register base address for the module.
 * @param[in] value The setting value of upper limit for PDB counter.
 */
static inline void PDB_SetTimerModulusValue(PDB_Type * const base,
                                            uint16_t value)
{
    DEV_ASSERT(base != NULL);
    REG_RMW32(&(base->MOD), PDB_MOD_MOD_MASK, PDB_MOD_MOD(value));
}

/*!
 * @brief Gets the PDB counter value of PDB timer.
 *
 * This function gets the PDB counter value of PDB timer.
 *
 * @param[in] base Register base address for the module.
 * @return The current counter value.
 */
static inline uint32_t PDB_GetTimerValue(PDB_Type const * const base)
{
    DEV_ASSERT(base != NULL);

    return ((base->CNT & PDB_CNT_CNT_MASK) >> PDB_CNT_CNT_SHIFT);
}

/*!
 * @brief Sets the interrupt delay milestone of the PDB counter.
 *
 * This function sets the interrupt delay milestone of the PDB counter.
 * If enabled, a PDB interrupt is generated when the counter is equal to the
 * setting value.
 *
 * @param[in] base Register base address for the module.
 * @param[in] value The setting value for interrupt delay milestone of PDB counter.
 */
static inline void PDB_SetValueForTimerInterrupt(PDB_Type * const base,
                                                 uint16_t value)
{
    DEV_ASSERT(base != NULL);
    REG_RMW32(&(base->IDLY), PDB_IDLY_IDLY_MASK, PDB_IDLY_IDLY(value));
}

/*!
 * @brief Switches to enable the pre-trigger back-to-back mode.
 *
 * This function switches to enable the pre-trigger back-to-back mode.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 * @param[in] enable Switcher to assert the feature.
 */
void PDB_SetAdcPreTriggerBackToBackEnable(PDB_Type * const base,
                                          uint32_t chn,
                                          uint32_t preChnMask,
                                          bool enable);

/*!
 * @brief Switches to enable the pre-trigger output.
 *
 * This function switches to enable pre-trigger output.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 * @param[in] enable Switcher to assert the feature.
 */
void PDB_SetAdcPreTriggerOutputEnable(PDB_Type * const base,
                                      uint32_t chn,
                                      uint32_t preChnMask,
                                      bool enable);

/*!
 * @brief Switches to enable the pre-trigger.
 *
 * This function switches to enable the pre-trigger.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 * @param[in] enable Switcher to assert the feature.
 */
void PDB_SetAdcPreTriggerEnable(PDB_Type * const base,
                                uint32_t chn,
                                uint32_t preChnMask,
                                bool enable);

/*!
 * @brief Gets the flag which indicates whether the PDB counter has reached the pre-trigger delay value.
 *
 * This function gets the flag which indicates the PDB counter has reached the
 * pre-trigger delay value.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 * @return Flag mask. Indicated bit would be 1 if the event is asserted.
 */
static inline uint32_t PDB_GetAdcPreTriggerFlags(PDB_Type const * const base,
                                                 uint32_t chn,
                                                 uint32_t preChnMask)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);

    return preChnMask & ((base->CH[chn].S & PDB_S_CF_MASK) >> PDB_S_CF_SHIFT);
}

/*!
 * @brief Clears the flag which indicates that the PDB counter has reached the pre-trigger delay value.
 *
 * This function clears the flag which indicates that the PDB counter has reached  the
 * pre-trigger delay value.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 */
void PDB_ClearAdcPreTriggerFlags(PDB_Type * const base,
                                 uint32_t chn,
                                 uint32_t preChnMask);

/*!
 * @brief Gets the flag which indicates whether a sequence error is detected.
 *
 * This function gets the flag which indicates whether a sequence error is detected.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 * @return Flag mask. Indicated bit would be 1 if the event is asserted.
 */
static inline uint32_t PDB_GetAdcPreTriggerSeqErrFlags(PDB_Type const * const base,
                                                       uint32_t chn,
                                                       uint32_t preChnMask)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);

    return (preChnMask & ((base->CH[chn].S & PDB_S_ERR_MASK) >> PDB_S_ERR_SHIFT));
}

/*!
 * @brief Clears the flag which indicates that a sequence error has been detected.
 *
 * This function clears the flag which indicates that the sequence error has been detected.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChnMask ADC channel group index mask for trigger.
 */
void PDB_ClearAdcPreTriggerSeqErrFlags(PDB_Type * const base,
                                       uint32_t chn,
                                       uint32_t preChnMask);

/*!
 * @brief Sets the pre-trigger delay value.
 *
 * This function sets the pre-trigger delay value.
 *
 * @param[in] base Register base address for the module.
 * @param[in] chn PDB instance index for trigger.
 * @param[in] preChn ADC channel group index for trigger.
 * @param[in] value Setting value for pre-trigger's delay value.
 */
static inline void PDB_SetAdcPreTriggerDelayValue(PDB_Type * const base,
                                    uint32_t chn,
                                    uint32_t preChn,
                                    uint32_t value)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);
    DEV_ASSERT(preChn < PDB_DLY_COUNT);

    base->CH[chn].DLY[preChn] = PDB_DLY_DLY(value);
}

/*!
 * @brief Switches to enable the pulse-out trigger.
 *
 * This function switches to enable the pulse-out trigger.
 *
 * @param[in] base Register base address for the module.
 * @param[in] pulseChnMask Pulse-out channle index mask for trigger.
 * @param[in] enable Switcher to assert the feature.
 */
void PDB_SetCmpPulseOutEnable(PDB_Type * const base,
                              uint32_t pulseChnMask,
                              bool enable);

/*!
 * @brief Sets the counter delay value for the pulse-out goes high.
 *
 * This function sets the counter delay value for the pulse-out goes high.
 *
 * @param[in] base Register base address for the module.
 * @param[in] pulseChn Pulse-out channel index for trigger.
 * @param[in] value Setting value for PDB delay .
 */
static inline void PDB_SetCmpPulseOutDelayForHigh(PDB_Type * const base,
                                                  uint32_t pulseChn,
                                                  uint32_t value)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(pulseChn < PDB_POnDLY_COUNT);
    base->POnDLY[pulseChn].ACCESS16BIT.DLY1 = (uint16_t)PDB_POnDLY_ACCESS16BIT_DLY1_DLY1(value);
}

/*!
 * @brief Sets the counter delay value for the pulse-out goes low.
 *
 * This function sets the counter delay value for the pulse-out goes low.
 *
 * @param[in] base Register base address for the module.
 * @param[in] pulseChn Pulse-out channel index for trigger.
 * @param[in] value Setting value for PDB delay .
 */
static inline void PDB_SetCmpPulseOutDelayForLow(PDB_Type * const base,
                                                 uint32_t pulseChn,
                                                 uint32_t value)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(pulseChn < PDB_POnDLY_COUNT);
    base->POnDLY[pulseChn].ACCESS16BIT.DLY2 = (uint16_t)PDB_POnDLY_ACCESS16BIT_DLY2_DLY2(value);
}

#if FEATURE_PDB_HAS_INSTANCE_BACKTOBACK
/*!
 * @brief Enable PDB Back-to-Back at instance level
 *
 * This function enables Back-to-Back between PDB0 channel 0 and PDB1 channel 0
 *
 */
static inline void PDB_SIM_EnableBackToBack(void)
{
    SIM->CHIPCTL |= SIM_CHIPCTL_PDB_BB_SEL(1u);
}

/*!
 * @brief Disable PDB Back-to-Back at instance level
 *
 * This function disables Back-to-Back between PDB0 channel 0 and PDB1 channel 0
 *
 */
static inline void PDB_SIM_DisableBackToBack(void)
{
    SIM->CHIPCTL &= ~SIM_CHIPCTL_PDB_BB_SEL(1u);
}
#endif /* FEATURE_PDB_HAS_INSTANCE_BACKTOBACK */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* PDB_HW_ACCESS_H */

/******************************************************************************
 * EOF
 *****************************************************************************/
