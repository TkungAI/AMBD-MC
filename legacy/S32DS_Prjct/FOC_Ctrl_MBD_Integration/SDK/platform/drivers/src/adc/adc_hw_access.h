/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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

#ifndef ADC_HW_ACCESS_H
#define ADC_HW_ACCESS_H

#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"
#include "adc_driver.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5,
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3,
 * Expression assigned to a narrower or different essential type.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type
 * The base addresses are provided as integers so they need to be cast to pointers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer
 * The base addresses are provided as integers so they need to be cast to pointers.
 */

/*! @file adc_hw_access.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @name Converter
 * General ADC functions.
 */
/*! @{*/

/*!
 * @brief Gets the Conversion Active Flag
 *
 * This function checks whether a conversion is currently
 * taking place on the ADC module.
 *
 *
 * @param[in] baseAddr adc base pointer
 * @return Conversion Active Flag state
 */
static inline bool ADC_GetConvActiveFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ADACT_MASK) >> ADC_SC2_ADACT_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Gets the current ADC clock divider configuration.
 *
 * This function returns the configured clock divider
 * bitfield value for the ADC instance.
 *
 * @param[in] baseAddr adc base pointer
 * @return the clock divider value. Possible values:
 *        - ADC_CLK_DIVIDE_1 : Divider set to 1.
 *        - ADC_CLK_DIVIDE_2 : Divider set to 2.
 *        - ADC_CLK_DIVIDE_4 : Divider set to 4.
 *        - ADC_CLK_DIVIDE_8 : Divider set to 8.
 */
static inline adc_clk_divide_t ADC_GetClockDivide(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp = (tmp & ADC_CFG1_ADIV_MASK) >> ADC_CFG1_ADIV_SHIFT;

    /* Enum defines all possible values, so casting is safe */
    return (adc_clk_divide_t)(tmp);
}

/*!
 * @brief Sets the ADC clock divider configuration.
 *
 * This functions configures the ADC instance clock divider.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] clockDivide clk divider
 *        - ADC_CLK_DIVIDE_1 : Divider set to 1.
 *        - ADC_CLK_DIVIDE_2 : Divider set to 2.
 *        - ADC_CLK_DIVIDE_4 : Divider set to 4.
 *        - ADC_CLK_DIVIDE_8 : Divider set to 8.
 */
static inline void ADC_SetClockDivide(ADC_Type * const baseAddr,
                                      const adc_clk_divide_t clockDivide)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp &= ~(ADC_CFG1_ADIV_MASK);
    tmp |= ADC_CFG1_ADIV(clockDivide);
    baseAddr->CFG1 = tmp;
}

/*!
 * @brief Gets the Sample time in AD clock cycles
 *
 * This function gets the sample time (in AD clocks)
 * configured for the ADC. Selection of 2 to 256 ADCK is
 * possible. The value returned by this function is the
 * sample time minus 1. A sample time of 1 is not supported.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Sample Time in AD Clocks
 */
static inline uint8_t ADC_GetSampleTime(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->CFG2;
    tmp = (tmp & ADC_CFG2_SMPLTS_MASK) >> ADC_CFG2_SMPLTS_SHIFT;

    return (uint8_t)(tmp);
}

/*!
 * @brief Sets the Sample time in AD clock cycles
 *
 * This function configures the sample time for the ADC (in
 * ADCK clocks). The actual sample time will be the value
 * provided plus 1.  Selection of 2 to 256 ADCK is possible.
 * A real sample time of 1 is not supported (a parameter value of 0
 * will be automatically be changed to 1).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] sampletime Sample time in AD Clocks
 */
static inline void ADC_SetSampleTime(ADC_Type * const baseAddr,
                                     uint8_t sampletime)
{
    /* Clip sample time to minimum value */
    uint8_t rsampletime = (uint8_t)((sampletime > 0U) ? sampletime : 1U);
    uint32_t tmp = baseAddr->CFG2;
    tmp &= ~(ADC_CFG2_SMPLTS_MASK);
    tmp |= ADC_CFG2_SMPLTS(rsampletime);
    baseAddr->CFG2 = tmp;
}

/*!
 * @brief Gets the Resolution Mode configuration
 *
 * This function returns the configured resolution mode for
 * the ADC.
 *
 * @param[in] baseAddr adc base pointer
 * @return the ADC resolution mode. Possible values:
 *        - ADC_RESOLUTION_8BIT : 8-bit resolution mode.
 *        - ADC_RESOLUTION_10BIT : 10-bit resolution mode.
 *        - ADC_RESOLUTION_12BIT : 12-bit resolution mode.
 */
static inline adc_resolution_t ADC_GetResolution(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp = (tmp & ADC_CFG1_MODE_MASK) >> ADC_CFG1_MODE_SHIFT;
    adc_resolution_t retValue;
    /* Enum does not define all possible values, do a swith-case */
    switch (tmp)
    {
        case 0x00U:
            retValue = ADC_RESOLUTION_8BIT;
            break;
        case 0x01U:
            retValue = ADC_RESOLUTION_12BIT;
            break;
        case 0x02U:
            retValue = ADC_RESOLUTION_10BIT;
            break;
        default:
            retValue = ADC_RESOLUTION_8BIT;
            break;
    }

    return retValue;
}

/*!
 * @brief Sets the Resolution Mode configuration
 *
 * This function configures the ADC resolution mode.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] resolution the adc resolution mode
 *        - ADC_RESOLUTION_8BIT : 8-bit resolution mode.
 *        - ADC_RESOLUTION_10BIT : 10-bit resolution mode.
 *        - ADC_RESOLUTION_12BIT : 12-bit resolution mode.
 */
static inline void ADC_SetResolution(ADC_Type * const baseAddr,
                                     const adc_resolution_t resolution)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp &= ~(ADC_CFG1_MODE_MASK);
    tmp |= ADC_CFG1_MODE(resolution);
    baseAddr->CFG1 = tmp;
}

/*!
 * @brief Gets the AD Clock Input configuration
 *
 * This function returns the configured clock input source
 * for the ADC.
 *
 * @param[in] baseAddr adc base pointer
 * @return the input clock source. Possible values:
 *        - ADC_CLK_ALT_1 : ADC Input clock source alternative 1.
 *        - ADC_CLK_ALT_2 : ADC Input clock source alternative 2.
 *        - ADC_CLK_ALT_3 : ADC Input clock source alternative 3.
 *        - ADC_CLK_ALT_4 : ADC Input clock source alternative 4.
 */
static inline adc_input_clock_t ADC_GetInputClock(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp = (tmp & ADC_CFG1_ADICLK_MASK) >> ADC_CFG1_ADICLK_SHIFT;

    /* Enum defines all possible values, so casting is safe */
    return (adc_input_clock_t)(tmp);
}

/*!
 * @brief Sets the AD Clock Input configuration
 *
 * This function configures the clock input source for the
 * ADC.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] inputClock the new input clock source
 *        - ADC_CLK_ALT_1 : ADC Input clock source alternative 1.
 *        - ADC_CLK_ALT_2 : ADC Input clock source alternative 2.
 *        - ADC_CLK_ALT_3 : ADC Input clock source alternative 3.
 *        - ADC_CLK_ALT_4 : ADC Input clock source alternative 4.
 */
static inline void ADC_SetInputClock(ADC_Type * const baseAddr,
                                     const adc_input_clock_t inputClock)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp &= ~(ADC_CFG1_ADICLK_MASK);
    tmp |= ADC_CFG1_ADICLK(inputClock);
    baseAddr->CFG1 = tmp;
}

/*!
 * @brief Gets the ADC Trigger Mode
 *
 * This function returns the configured triggering mode
 * for the ADC. In Software Triggering Mode, the user can
 * start conversions by setting an input channel in the
 * ADC measurement channel A (index 0). When in Hardware
 * trigger mode, a conversion is started by another peripheral (
 * like PDB or TRGMUX).
 *
 * @param[in] baseAddr adc base pointer
 * @return the current trigger mode. Possible values:
 *        - ADC_TRIGGER_SOFTWARE : Software triggering.
 *        - ADC_TRIGGER_HARDWARE : Hardware triggering.
 */
static inline adc_trigger_t ADC_GetTriggerMode(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ADTRG_MASK) >> ADC_SC2_ADTRG_SHIFT;

    /* Enum defines all possible values, so casting is safe */
    return (adc_trigger_t)(tmp);
}

/*!
 * @brief Sets the ADC Trigger Mode
 *
 * This function configures the ADC triggering mode. In
 * Software Triggering Mode, the user can start conversions
 * by setting an input channel in the ADC measurement channel
 * A (index 0). When in Hardware trigger mode, a conversion
 * is started by another peripheral (like PDB or TRGMUX).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] trigger the desired trigger mode
 *        - ADC_TRIGGER_SOFTWARE : Software triggering.
 *        - ADC_TRIGGER_HARDWARE : Hardware triggering.
 */
static inline void ADC_SetTriggerMode(ADC_Type * const baseAddr,
                                      const adc_trigger_t trigger)
{
    uint32_t tmp = baseAddr->SC2;
    tmp &= ~(ADC_SC2_ADTRG_MASK);
    tmp |= ADC_SC2_ADTRG(trigger);
    baseAddr->SC2 = tmp;
}

/*!
 * @brief Gets the pretrigger source configured for an ADC instance
 *
 * This function gets the pretrigger source selected from
 * ADC Trigger Latching and Arbitration Unit,
 * affecting control channels 0-3.
 *
 * @param[in] instance  the ADC instance
 * @return  pretrigger source selected. Possible values:
 *           - ADC_PRETRIGGER_SEL_PDB     - PDB pretrigger
 *           - ADC_PRETRIGGER_SEL_TRGMUX  - TRGMUX pretrigger
 *           - ADC_PRETRIGGER_SEL_SW      - Software pretrigger
 */
static inline adc_pretrigger_sel_t ADC_GetPretriggerSelect(const uint32_t instance)
{
    const SIM_Type * const sim_base          = SIM;
    uint32_t currentVal                      = 0U;
    adc_pretrigger_sel_t returnVal           = ADC_PRETRIGGER_SEL_PDB;
    uint32_t mask[ADC_INSTANCE_COUNT] = {0U};
    uint32_t shift[ADC_INSTANCE_COUNT] = {0U};
#if (ADC_INSTANCE_COUNT == 1u)
    mask[0]  = SIM_ADCOPT_ADC0PRETRGSEL_MASK;
    shift[0] = SIM_ADCOPT_ADC0PRETRGSEL_SHIFT;
#elif (ADC_INSTANCE_COUNT == 2u)
    mask[0]  = SIM_ADCOPT_ADC0PRETRGSEL_MASK;
    mask[1]  = SIM_ADCOPT_ADC1PRETRGSEL_MASK;
    shift[0] = SIM_ADCOPT_ADC0PRETRGSEL_SHIFT;
    shift[1] = SIM_ADCOPT_ADC1PRETRGSEL_SHIFT;
#else
#error "Maximum supported value for ADC_INSTANCE_COUNT is 2."
#endif
    currentVal = (sim_base->ADCOPT & mask[instance]) >> shift[instance];
    switch(currentVal)
    {
        case ADC_PRETRIGGER_SEL_PDB:
            returnVal = ADC_PRETRIGGER_SEL_PDB;     break;
        case ADC_PRETRIGGER_SEL_TRGMUX:
            returnVal = ADC_PRETRIGGER_SEL_TRGMUX;  break;
        case ADC_PRETRIGGER_SEL_SW:
            returnVal = ADC_PRETRIGGER_SEL_SW;      break;
        default:
            DEV_ASSERT(false);                      break;
    }

    return returnVal;
}

/*!
 * @brief Sets the pretrigger select for an ADC instance
 *
 * This function sets the pretrigger source selected from
 * ADC Trigger Latching and Arbitration Unit,
 * affecting control channels 0-3.
 *
 * @param[in] instance  the ADC instance
 * @param[in] pretriggerSel  the pretrigger source to be selected
 *           - ADC_PRETRIGGER_SEL_PDB     - PDB pretrigger
 *           - ADC_PRETRIGGER_SEL_TRGMUX  - TRGMUX pretrigger
 *           - ADC_PRETRIGGER_SEL_SW      - Software pretrigger
 */
static inline void ADC_SetPretriggerSelect(const uint32_t instance,
                                           const adc_pretrigger_sel_t pretriggerSel)
{
    SIM_Type * const simBase = SIM;
    uint32_t mask[ADC_INSTANCE_COUNT] = {0U};
#if (ADC_INSTANCE_COUNT == 1u)
    mask[0] = SIM_ADCOPT_ADC0PRETRGSEL_MASK;
#elif (ADC_INSTANCE_COUNT == 2u)
    mask[0] = SIM_ADCOPT_ADC0PRETRGSEL_MASK;
    mask[1] = SIM_ADCOPT_ADC1PRETRGSEL_MASK;
#else
#error "Maximum supported value for ADC_INSTANCE_COUNT is 2."
#endif
    uint32_t intermVal = 0U;

    intermVal = simBase->ADCOPT & (~ mask[instance]);

    switch(instance)
    {
    case 0:
        intermVal |= SIM_ADCOPT_ADC0PRETRGSEL(pretriggerSel);
        break;
    case 1:
        intermVal |= SIM_ADCOPT_ADC1PRETRGSEL(pretriggerSel);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

    simBase->ADCOPT = intermVal;
}

/*!
 * @brief Gets the trigger source configured for an ADC instance
 *
 * This function gets the trigger source selected from
 * ADC Trigger Latching and Arbitration Unit.
 *
 * @param[in] instance  the ADC instance
 * @param[in] triggerSel  the trigger source to be selected
 *         - ADC_TRIGGER_SEL_PDB     - PDB trigger selected
 *         - ADC_TRIGGER_SEL_TRGMUX  - TRGMUX trigger
 */
static inline adc_trigger_sel_t ADC_GetTriggerSelect(const uint32_t instance)
{
    const SIM_Type * const sim_base          = SIM;
    uint32_t currentVal                      = 0U;
    adc_trigger_sel_t returnVal              = ADC_TRIGGER_SEL_PDB;
    uint32_t mask[ADC_INSTANCE_COUNT] = {0U};
    uint32_t shift[ADC_INSTANCE_COUNT] = {0U};
#if (ADC_INSTANCE_COUNT == 1u)
    mask[0]  = SIM_ADCOPT_ADC0TRGSEL_MASK;
    shift[0] = SIM_ADCOPT_ADC0TRGSEL_SHIFT;
#elif (ADC_INSTANCE_COUNT == 2u)
    mask[0]  = SIM_ADCOPT_ADC0TRGSEL_MASK;
    mask[1]  = SIM_ADCOPT_ADC1TRGSEL_MASK;
    shift[0] = SIM_ADCOPT_ADC0TRGSEL_SHIFT;
    shift[1] = SIM_ADCOPT_ADC1TRGSEL_SHIFT;
#else
#error "Maximum supported value for ADC_INSTANCE_COUNT is 2."
#endif
    currentVal = (sim_base->ADCOPT & mask[instance]) >> shift[instance];
    switch(currentVal)
    {
    case ADC_TRIGGER_SEL_PDB:
        returnVal = ADC_TRIGGER_SEL_PDB;    break;
    case ADC_TRIGGER_SEL_TRGMUX:
        returnVal = ADC_TRIGGER_SEL_TRGMUX; break;
    default:
        DEV_ASSERT(false);                  break;
    }

    return returnVal;
}

/*!
 * @brief Sets the trigger select for an ADC instance
 *
 * This function sets the trigger source selected from
 * ADC Trigger Latching and Arbitration Unit.
 *
 * @param[in] instance  the ADC instance
 * @param[in] triggerSel  the trigger source to be selected
 *         - ADC_TRIGGER_SEL_PDB     - PDB trigger selected
 *         - ADC_TRIGGER_SEL_TRGMUX  - TRGMUX trigger
 */
static inline void ADC_SetTriggerSelect(const uint32_t instance,
                                        const adc_trigger_sel_t triggerSel)
{
    SIM_Type * const simBase = SIM;
    uint32_t mask[ADC_INSTANCE_COUNT] = {0U};
#if (ADC_INSTANCE_COUNT == 1u)
    mask[0] = SIM_ADCOPT_ADC0TRGSEL_MASK;
#elif (ADC_INSTANCE_COUNT == 2u)
    mask[0] = SIM_ADCOPT_ADC0TRGSEL_MASK;
    mask[1] = SIM_ADCOPT_ADC1TRGSEL_MASK;
#else
#error "Maximum supported value for ADC_INSTANCE_COUNT is 2."
#endif
    uint32_t intermVal = 0U;

    intermVal = simBase->ADCOPT & (~ mask[instance]);

    switch(instance)
    {
    case 0:
        intermVal |= SIM_ADCOPT_ADC0TRGSEL(triggerSel);
        break;
    case 1:
        intermVal |= SIM_ADCOPT_ADC1TRGSEL(triggerSel);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

    simBase->ADCOPT = intermVal;
}

/*!
 * @brief Gets the DMA Enable Flag state
 *
 * This function returns the state of the DMA Enable flag.
 * DMA can be used to transfer completed conversion values
 * from the result registers to RAM without CPU intervention.
 *
 * @param[in] baseAddr adc base pointer
 * @return the DMA Enable Flag state
 */
static inline bool ADC_GetDMAEnableFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_DMAEN_MASK) >> ADC_SC2_DMAEN_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the DMA Enable Flag state
 *
 * This function configures the DMA Enable Flag. DMA can be
 * used to transfer completed conversion values from the
 * result registers to RAM without CPU intervention.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new DMA Enable Flag state
 */
static inline void ADC_SetDMAEnableFlag(ADC_Type * const baseAddr,
                                        const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_DMAEN_MASK));
    tmp |= ADC_SC2_DMAEN(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the ADC Reference Voltage selection
 *
 * This function returns the configured reference voltage
 * selection for the ADC. Reference voltage can be selected
 * between the pairs (VrefH, VrefL) and (ValtH, ValtL).
 *
 * @param[in] baseAddr adc base pointer
 * @return the voltage reference input pair. Possible values:
 *        - ADC_VOLTAGEREF_VREF : VrefL and VrefH used as voltage reference.
 *        - ADC_VOLTAGEREF_VALT : ValtL and ValtH used as voltage reference.
 */
static inline adc_voltage_reference_t ADC_GetVoltageReference(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_REFSEL_MASK) >> ADC_SC2_REFSEL_SHIFT;

    /* Enum defines all possible values, so casting is safe */
    return (adc_voltage_reference_t)(tmp);
}

/*!
 * @brief Sets the ADC Reference Voltage selection
 *
 * This function configures the ADC Reference Voltage. Reference
 * voltage can be selected between the pairs (VrefH, VrefL)
 * and (ValtH, ValtL).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] voltageRef the new voltage reference input
 *        - ADC_VOLTAGEREF_VREF : VrefL and VrefH used as voltage reference.
 *        - ADC_VOLTAGEREF_VALT : ValtL and ValtH used as voltage reference.
 */
static inline void ADC_SetVoltageReference(ADC_Type * const baseAddr,
                                           const adc_voltage_reference_t voltageRef)
{
    uint32_t tmp = baseAddr->SC2;
    tmp &= ~(ADC_SC2_REFSEL_MASK);
    tmp |= ADC_SC2_REFSEL(voltageRef);
    baseAddr->SC2 = tmp;
}

/*!
 * @brief Gets the Continuous Conversion Flag state
 *
 * This functions returns the state of the Continuous Conversion
 * Flag. This feature can be used to continuously sample a
 * single channel. When this is active, the channel cannot be
 * changed (by software or hardware trigger) until this feature
 * is turned off.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Continuous Conversion Flag state
 */
static inline bool ADC_GetContinuousConvFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    tmp = (tmp & ADC_SC3_ADCO_MASK) >> ADC_SC3_ADCO_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Continuous Conversion Flag state
 *
 * This function configures the Continuous Conversion. This
 * feature can be used to continuously sample a single channel.
 * When this is active, the channel cannot be changed (by
 * software or hardware trigger) until this feature is turned
 * off.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Continuous Conversion Flag state
 */
static inline void ADC_SetContinuousConvFlag(ADC_Type * const baseAddr,
                                             const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(ADC_SC3_ADCO_MASK);
    tmp |= ADC_SC3_ADCO(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC3 = (uint32_t)tmp;
}

/*!
 * @brief Sets the Supply Monitor Enable Flag state
 *
 * This function configures the Supply Monitor Enable Flag.
 * Supply monitoring is available only for ADC 0, and the actual supply to be
 * monitored can be selected using ADC_INPUTCHAN_SUPPLY_ enum entries.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Supply Monitor Enable Flag state
 */
static inline void ADC_SetSupplyMonitoringEnableFlag(SIM_Type * const baseAddr,
                                                  const bool state)
{
    if(state == true)
    {
        baseAddr->CHIPCTL |= SIM_CHIPCTL_ADC_SUPPLYEN_MASK;
    }
    else
    {
        baseAddr->CHIPCTL &= ~SIM_CHIPCTL_ADC_SUPPLYEN_MASK;
    }
}

/*! @}*/

/*!
 * @name Hardware Compare.
 * Functions to configure the Hardware Compare feature.
 */
/*! @{*/

/*!
 * @brief Gets the Hardware Compare Enable Flag state
 *
 * This function returns the state of the Hardware Compare
 * Enable Flag. Hardware Compare can be used to check if the
 * ADC result is within or outside of a predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Compare Enable Flag state
 */
static inline bool ADC_GetHwCompareEnableFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ACFE_MASK) >> ADC_SC2_ACFE_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Compare Enable Flag state
 *
 * This functions configures the Hardware Compare Enable Flag.
 * Hardware Compare can be used to check if the ADC result
 * is within or outside of a predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Compare Enable Flag state
 */
static inline void ADC_SetHwCompareEnableFlag(ADC_Type * const baseAddr,
                                              const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_ACFE_MASK));
    tmp |= ADC_SC2_ACFE(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the Hardware Compare Greater Than Enable Flag state
 *
 * This function returns the Hardware Compare Greater Than
 * Enable Flag. Using this feature, the ADC can be configured
 * to check if the measured value is within or outside of a
 * predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Compare Greater Than Enable Flag state
 */
static inline bool ADC_GetHwCompareGtEnableFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ACFGT_MASK) >> ADC_SC2_ACFGT_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Compare Greater Than Enable Flag state
 *
 * This function configures the Hardware Compare Greater Than
 * Enable Flag. Using this feature, the ADC can be configured
 * to check if the measured value is within or outside of a
 * predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Compare Greater Than Enable Flag state
 */
static inline void ADC_SetHwCompareGtEnableFlag(ADC_Type * const baseAddr,
                                                const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_ACFGT_MASK));
    tmp |= ADC_SC2_ACFGT(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the Hardware Compare Range Enable state
 *
 * This function returns the state of the Hardware Compare
 * Range Enable Flag. This feature allows configuration
 * of a range with two non-zero values or with a non-zero
 * and zero value.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Compare Range Enable Flag state
 */
static inline bool ADC_GetHwCompareRangeEnableFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ACREN_MASK) >> ADC_SC2_ACREN_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Compare Range Enable state
 *
 * This function configures the Hardware Compare Range
 * Enable Flag. This feature allows configuration
 * of a range with two non-zero values or with a non-zero
 * and zero value.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Compare Range Enable Flag state
 */
static inline void ADC_SetHwCompareRangeEnableFlag(ADC_Type * const baseAddr,
                                                   const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_ACREN_MASK));
    tmp |= ADC_SC2_ACREN(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the Compare Register 1 value
 *
 * This function returns the value written in the Hardware
 * Compare Register 1. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution value (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @return the Compare Register 1 value
 */
static inline uint16_t ADC_GetHwCompareComp1Value(const ADC_Type * const baseAddr)
{
    return (uint16_t)baseAddr->CV[0U];
}

/*!
 * @brief Sets the Compare Register 1 value
 *
 * This function writes a 12-bit value in the Hardware
 * Compare Register 1. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new Compare Register 1 value
 */
static inline void ADC_SetHwCompareComp1Value(ADC_Type * const baseAddr,
                                              const uint16_t value)
{
    baseAddr->CV[0U] = ADC_CV_CV(value);
}

/*!
 * @brief Gets the Compare Register 2 value
 *
 * This function returns the value written in the Hardware
 * Compare Register 2. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @return the Compare Register 2 value
 */
static inline uint16_t ADC_GetHwCompareComp2Value(const ADC_Type * const baseAddr)
{
    return (uint16_t)baseAddr->CV[1U];
}

/*!
 * @brief Sets the Compare Register 2 value
 *
 * This function writes a 12-bit value in the Hardware
 * Compare Register 2. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution value (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new Compare Register 2 value
 */
static inline void ADC_SetHwCompareComp2Value(ADC_Type * const baseAddr,
                                              const uint16_t value)
{
    baseAddr->CV[1U] = ADC_CV_CV(value);
}

/*! @}*/

/*!
 * @name Hardware Average.
 * Functions to configure the Hardware Averaging feature.
 */
/*! @{*/

/*!
 * @brief Gets the Hardware Average Enable Flag state
 *
 * This function returns the state of the Hardware Average
 * Enable Flag. Hardware averaging can be used to obtain an
 * average value over multiple consecutive conversions on
 * the same channel.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Average Enable Flag state
 */
static inline bool ADC_GetHwAverageEnableFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    tmp = (tmp & ADC_SC3_AVGE_MASK) >> ADC_SC3_AVGE_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Average Enable Flag state
 *
 * This function configures the Hardware Average Enable Flag.
 * Hardware averaging can be used to obtain an average value
 * over multiple consecutive conversions on the same channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Average Enable Flag state
 */
static inline void ADC_SetHwAverageEnableFlag(ADC_Type * const baseAddr,
                                              const bool state)
{
    uint32_t tmp = baseAddr->SC3;
    /* Clear the affected bitfield */
    tmp &= ~(ADC_SC3_AVGE_MASK);
    tmp |= ADC_SC3_AVGE(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC3 = tmp;
}

/*!
 * @brief Gets the Hardware Average Mode
 *
 * This function returns the configured Hardware Average Mode.
 * The mode selects the number of samples to average: 4, 8, 16
 * or 32.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Average Mode selection. Possible values:
 *        - ADC_AVERAGE_4 : Hardware average of 4 samples..
 *        - ADC_AVERAGE_8 : Hardware average of 8 samples.
 *        - ADC_AVERAGE_16 : Hardware average of 16 samples.
 *        - ADC_AVERAGE_32 : Hardware average of 32 samples.
 */
static inline adc_average_t ADC_GetHwAverageMode(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->SC3;
    tmp = (tmp & ADC_SC3_AVGS_MASK) >> ADC_SC3_AVGS_SHIFT;

    /* Enum defines all possible values, so casting is safe */
    return (adc_average_t)(tmp);
}

/*!
 * @brief Sets the Hardware Average Mode
 *
 * This function configures the Hardware Average Mode. The
 * mode selects the number of samples to average: 4, 8, 16
 * or 32.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] averageMode the new Hardware Average Mode.
 *        - ADC_AVERAGE_4 : Hardware average of 4 samples..
 *        - ADC_AVERAGE_8 : Hardware average of 8 samples.
 *        - ADC_AVERAGE_16 : Hardware average of 16 samples.
 *        - ADC_AVERAGE_32 : Hardware average of 32 samples.
 */
static inline void ADC_SetHwAverageMode(ADC_Type * const baseAddr,
                                        const adc_average_t averageMode)
{
    uint32_t tmp = baseAddr->SC3;
    /* Clear the affected bitfield */
    tmp &= ~(ADC_SC3_AVGS_MASK);
    tmp |= ADC_SC3_AVGS(averageMode);
    baseAddr->SC3 = tmp;
}

/*! @}*/

/*!
 * @name Automatic Calibration.
 * Functions configure and use the Automatic Calibration feature.
 */
/*! @{*/

/*!
 * @brief Gets the Calibration Active Flag state
 *
 * This function returns the state of the Calibration Active
 * Flag. This flag is set if an Auto-Calibration sequence is
 * taking place.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Calibration Active Flag state
 */
static inline bool ADC_GetCalibrationActiveFlag(const ADC_Type * const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    tmp = (tmp & ADC_SC3_CAL_MASK) >> ADC_SC3_CAL_SHIFT;

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Calibration Active Flag state
 *
 * This functions starts or aborts an Auto-Calibration
 * sequence. If this is set, it will remain set until the
 * sequence is finished.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Calibration Active Flag state
 */
static inline void ADC_SetCalibrationActiveFlag(ADC_Type * const baseAddr,
                                                const bool state)
{
    uint32_t tmp = baseAddr->SC3;
    tmp &= ~(ADC_SC3_CAL_MASK);
    tmp |= ADC_SC3_CAL(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC3 = tmp;
}

/*!
 * @brief Gets the User Gain Register value
 *
 * This function returns the value in the User Gain Register.
 * The value in this register is the amplification applied
 * to the measured data before being written in the result
 * register.
 *
 * @param[in] baseAddr adc base pointer
 * @return the User Gain Register value
 */
static inline uint16_t ADC_GetUserGainValue(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->UG;
    tmp = (tmp & ADC_UG_UG_MASK) >> ADC_UG_UG_SHIFT;

    return (uint16_t)tmp;
}

/*!
 * @brief Sets the User Gain Register value
 *
 * This function configures the User Gain Register. The value
 * in this register is the amplification applied to the
 * measured data before being written in the result register.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new User Gain Register value
 */
static inline void ADC_SetUserGainValue(ADC_Type * const baseAddr,
                                        const uint16_t value)
{
    uint16_t clp0 = (uint16_t)baseAddr->CLP0;
    uint16_t clp1 = (uint16_t)baseAddr->CLP1;
    uint16_t clp2 = (uint16_t)baseAddr->CLP2;
    uint16_t clp3 = (uint16_t)baseAddr->CLP3;
    uint16_t clps = (uint16_t)baseAddr->CLPS;
    /* Add CLP0, CLP1, CLP2, CLP3 and CLPS */
    uint16_t sum = (uint16_t)(value + clp0 + clp1 + clp2 + clp3 + clps);
    /* If OR of bits [15:11] from the sum is 1 (set), write 0xFFFFU to Gain register */
    uint16_t temp = (uint16_t)(sum & 0xF800U);
    if (temp != 0x0000U)
    {
        temp = 0xFFFFU;
    }

    baseAddr->UG = (uint32_t)value;
    baseAddr->G = (uint32_t)temp;
}

/*!
 * @brief Gets the User Offset Register value
 *
 * This function returns the value in the User Offset Register.
 * The value in this register is subtracted from the measured
 * data before being written in the result register. This value
 * is 16-bit signed value. To preserve resolution, lower-order
 * bits will be ignored in low resolution-modes.
 *
 * @param[in] baseAddr adc base pointer
 * @return the User Offset Register value
 */
static inline uint16_t ADC_GetUserOffsetValue(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->USR_OFS;
    tmp = (tmp & ADC_USR_OFS_USR_OFS_MASK) >> ADC_USR_OFS_USR_OFS_SHIFT;

    return (uint16_t)tmp;
}

/*!
 * @brief Sets the User Offset Register value
 *
 * This function configures the User Offset Register. The value
 * in this register is subtracted from the measured data before
 * being written in the result register. This value is 16-bit
 * signed value. To preserve resolution, lower-order bits
 * will be ignored in low resolution-modes.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new User Offset Register value
 */
static inline void ADC_SetUserOffsetValue(ADC_Type * const baseAddr,
                                          const uint16_t value)
{
    baseAddr->USR_OFS = ADC_USR_OFS_USR_OFS(value);
}

/*! @}*/

/*!
 * @name Converter channels.
 * Functions to configure and access each ADC converter channel.
 */
/*! @{*/

/*!
 * @brief Gets the Channel Interrupt Enable state
 *
 * This function returns the state of the Channel Interrupt
 * Enable Flag. If the flag is set, an interrupt is generated
 * when the a conversion is completed for the channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @return the Channel Interrupt Enable Flag state
 */
static inline bool ADC_GetChanInterruptEnableFlag(const ADC_Type * const baseAddr,
                                                  const uint8_t chanIndex)
{

#if FEATURE_ADC_HAS_EXTRA_NUM_REGS

    uint32_t tmp = (uint32_t)baseAddr->aSC1[chanIndex];
    tmp = (tmp & ADC_aSC1_AIEN_MASK) >> ADC_aSC1_AIEN_SHIFT;
#else

    uint32_t tmp = (uint32_t)baseAddr->SC1[chanIndex];
    tmp = (tmp & ADC_SC1_AIEN_MASK) >> ADC_SC1_AIEN_SHIFT;
#endif /* FEATURE_ADC_HAS_EXTRA_NUM_REGS */

    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Gets the configured input channel for the selected measurement channel
 *
 * This function returns the configured input channel for a
 * measurement channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @return the Input Channel selected for the Measurement Channel.
 */
static inline adc_inputchannel_t ADC_GetInputChannel(const ADC_Type * const baseAddr,
                                                     const uint8_t chanIndex)
{
#if FEATURE_ADC_HAS_EXTRA_NUM_REGS
    uint32_t tmp = baseAddr->aSC1[chanIndex];
    tmp = (tmp & ADC_aSC1_ADCH_MASK) >> ADC_aSC1_ADCH_SHIFT;
#else
    uint32_t tmp = baseAddr->SC1[chanIndex];
    tmp = (tmp & ADC_SC1_ADCH_MASK) >> ADC_SC1_ADCH_SHIFT;
#endif /* FEATURE_ADC_HAS_EXTRA_NUM_REGS */

    uint32_t supplyen = (uint32_t)SIM->CHIPCTL;
    supplyen = (supplyen & SIM_CHIPCTL_ADC_SUPPLYEN_MASK) >> SIM_CHIPCTL_ADC_SUPPLYEN_SHIFT;
    if((tmp == (uint32_t)ADC_INPUTCHAN_INT0) && (supplyen != 0UL))
    {
        tmp = (uint32_t)SIM->CHIPCTL;
        tmp = (tmp & SIM_CHIPCTL_ADC_SUPPLY_MASK) >> SIM_CHIPCTL_ADC_SUPPLY_SHIFT;
        tmp = tmp + (uint32_t)ADC_INPUTCHAN_SUPPLY_VDD;
    }
    /* Enum defines all possible values, so casting is safe */
    return (adc_inputchannel_t)(tmp);
}

/*!
 * @brief Sets the input channel and Channel Interrupt Enable as configuration for the measurement channel.
 *
 * This function configures the input channel and state of the Interrupt Enable Flag for a measurement
 * channel. In software trigger mode, configuring channel
 * A (index 0) will start a new conversion immediately.
 * If the flag is set, an interrupt is generated when
 * the a conversion is completed for the channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @param[in] inputChan the Input Channel selected for the Measurement Channel
 * @param[in] state the new Channel Interrupt Enable Flag state
 */
static inline void ADC_SetInputChannel(ADC_Type * const baseAddr,
                                       const uint8_t chanIndex,
                                       const adc_inputchannel_t inputChan,
                                       const bool state)
{
    adc_inputchannel_t inputChanDemapped;

    /* Internal supply monitor channels need special configuration */
    if((inputChan >= ADC_INPUTCHAN_SUPPLY_VDD) && (inputChan <= ADC_INPUTCHAN_SUPPLY_VDD_LV))
    {
        const uint32_t supplyMonitorIdx = (uint32_t)inputChan - (uint32_t)ADC_INPUTCHAN_SUPPLY_VDD; /* De-map ADC_INPUTCHAN_SUPPLY_ into actual index.*/

        SIM_Type * const simBase = SIM;
        DEV_ASSERT((simBase ->CHIPCTL & SIM_CHIPCTL_ADC_SUPPLYEN_MASK) != 0UL);
        simBase->CHIPCTL &= ~SIM_CHIPCTL_ADC_SUPPLY_MASK;
        simBase->CHIPCTL |= SIM_CHIPCTL_ADC_SUPPLY(supplyMonitorIdx);

        inputChanDemapped = ADC_INPUTCHAN_INT0; /* Supply monitor channels are measured on ADC internal input channel 0 */
    }
    else
    {
        inputChanDemapped = inputChan;
    }

#if FEATURE_ADC_HAS_EXTRA_NUM_REGS
    uint32_t tmp = baseAddr->aSC1[chanIndex];
    tmp &= ~(ADC_aSC1_ADCH_MASK);
    tmp |= ADC_aSC1_ADCH(inputChanDemapped);
    tmp &= ~(ADC_aSC1_AIEN_MASK);
    tmp |= ADC_aSC1_AIEN(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->aSC1[chanIndex] = tmp;
#else
    uint32_t tmp = baseAddr->SC1[chanIndex];
    tmp &= ~(ADC_SC1_ADCH_MASK);
    tmp |= ADC_SC1_ADCH(inputChanDemapped);
    tmp &= ~(ADC_SC1_AIEN_MASK);
    tmp |= ADC_SC1_AIEN(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC1[chanIndex] = tmp;
#endif /* FEATURE_ADC_HAS_EXTRA_NUM_REGS */

}

/*! @}*/

/*!
 * @name Trigger latches.
 * Functions using the trigger latch mechanism.
 */
/*! @{*/

/*!
 * @brief Clear the latched triggers
 *
 * This function clears the latched triggers, except for the one under process.
 * Before calling this function, make sure the hardware trigger source of the ADC is disabled.
 *
 * @param[in] baseAddr adc base pointer
 */
static inline void ADC_ClearLatchTriggers(ADC_Type * const baseAddr)
{
    baseAddr->CFG1 |= ADC_CFG1_CLRLTRG(0x01u);
}

/*!
 * @brief Get the trigger latch flags bits.
 *
 * This function returns the trigger latch flags.
 *
 * @param[in] baseAddr adc base pointer
 * @return trigger latch flags
 */
static inline uint32_t ADC_GetTriggerLatchFlags(const ADC_Type * const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_TRGSTLAT_MASK) >> ADC_SC2_TRGSTLAT_SHIFT;

    return tmp;
}

/*! @}*/

#if defined (__cplusplus)
}
#endif


#endif /* ADC_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
