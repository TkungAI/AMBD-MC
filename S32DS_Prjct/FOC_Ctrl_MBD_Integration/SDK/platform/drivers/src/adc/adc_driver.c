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
/*!
 * @file adc_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include <stddef.h>
#include "adc_driver.h"
#include "adc_hw_access.h"
#include "clock_manager.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The reset value for CFG2[SMPLTS] */
#define ADC_RESET_SAMPLE_TIME_VALUE (12u)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for ADC instances. */
static ADC_Type * const s_adcBase[ADC_INSTANCE_COUNT] = ADC_BASE_PTRS;

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitConverterStruct
 * Description   : This function initializes the members of the adc_converter_config_t
 * structure to default values (Reference Manual resets). This function should be called
 * on a structure before using it to configure the converter (ADC_DRV_ConfigConverter), otherwise all members
 * must be written (initialized) by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * Implements : ADC_DRV_InitConverterStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitConverterStruct(adc_converter_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->clockDivide    = ADC_CLK_DIVIDE_1;
    config->sampleTime     = (uint8_t)ADC_DEFAULT_SAMPLE_TIME;
    config->resolution     = ADC_RESOLUTION_8BIT;
    config->inputClock     = ADC_CLK_ALT_1;
    config->trigger        = ADC_TRIGGER_SOFTWARE;
    config->pretriggerSel  = ADC_PRETRIGGER_SEL_PDB;
    config->triggerSel     = ADC_TRIGGER_SEL_PDB;
    config->dmaEnable      = false;
    config->voltageRef     = ADC_VOLTAGEREF_VREF;
    config->continuousConvEnable   = false;
    config->supplyMonitoringEnable = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigConverter
 * Description   : This function configures the ADC converter with the options
 * provided in the configuration structure.
 *
 * Implements : ADC_DRV_ConfigConverter_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigConverter(const uint32_t instance,
                             const adc_converter_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    /* Some alternative clocks can be unavailable depending on the device */
    DEV_ASSERT(config->inputClock <= NUMBER_OF_ALT_CLOCKS);

    ADC_Type * const base = s_adcBase[instance];
    clock_names_t adc_clocks[ADC_INSTANCE_COUNT] = ADC_CLOCKS;
    uint32_t adc_freq = 0u;
    status_t clk_status = CLOCK_SYS_GetFreq(adc_clocks[instance], &adc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    (void) clk_status;

    adc_freq = adc_freq / (uint32_t)(1UL << ((uint32_t)(config->clockDivide)));
    DEV_ASSERT((adc_freq >= ADC_CLOCK_FREQ_MIN_RUNTIME) && (adc_freq <= ADC_CLOCK_FREQ_MAX_RUNTIME));

    ADC_SetClockDivide(base, config->clockDivide);
    ADC_SetSampleTime(base, config->sampleTime);
    ADC_SetResolution(base, config->resolution);
    ADC_SetInputClock(base, config->inputClock);
    ADC_SetTriggerMode(base, config->trigger);
    ADC_SetPretriggerSelect(instance, config->pretriggerSel);
    ADC_SetTriggerSelect(instance, config->triggerSel);
    ADC_SetDMAEnableFlag(base, config->dmaEnable);
    ADC_SetVoltageReference(base, config->voltageRef);
    ADC_SetContinuousConvFlag(base, config->continuousConvEnable);

    /* Supply monitoring is only available for ADC 0. */
    DEV_ASSERT((config->supplyMonitoringEnable == false) || (instance == 0u));
    if(instance == 0u)
    {
        SIM_Type * const simBase = SIM;
        ADC_SetSupplyMonitoringEnableFlag(simBase, config->supplyMonitoringEnable);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetConverterConfig
 * Description   : This functions returns the current converter configuration in
 * the form of a configuration structure.
 *
 * Implements : ADC_DRV_GetConverterConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetConverterConfig(const uint32_t instance,
                                adc_converter_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    const ADC_Type * const base = s_adcBase[instance];
    config->clockDivide = ADC_GetClockDivide(base);
    config->sampleTime = ADC_GetSampleTime(base);
    config->resolution = ADC_GetResolution(base);
    config->inputClock = ADC_GetInputClock(base);
    config->trigger = ADC_GetTriggerMode(base);
    config->triggerSel = ADC_GetTriggerSelect(instance);
    config->pretriggerSel = ADC_GetPretriggerSelect(instance);
    config->dmaEnable = ADC_GetDMAEnableFlag(base);
    config->voltageRef = ADC_GetVoltageReference(base);
    config->continuousConvEnable = ADC_GetContinuousConvFlag(base);

    /* Supply monitoring is only available for ADC 0. */
    if(instance == 0u)
    {
        const SIM_Type * const simBase = SIM;
        config->supplyMonitoringEnable = ((simBase->CHIPCTL & SIM_CHIPCTL_ADC_SUPPLYEN_MASK) != 0u) ? true : false;
    }
    else
    {
        config->supplyMonitoringEnable = false;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_Reset
 * Description   : This function writes all the internal ADC registers with
 * their Reference Manual reset values.
 *
 * Implements : ADC_DRV_Reset_Activity
 *END**************************************************************************/
void ADC_DRV_Reset(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const baseAddr = s_adcBase[instance];
    uint8_t idx = 0U;

    for(idx = 0U; idx < ADC_SC1_COUNT; idx++)
    {
        baseAddr->SC1[idx] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    }

    baseAddr->CFG1 = ADC_CFG1_ADICLK(ADC_CLK_ALT_1) | ADC_CFG1_MODE(ADC_RESOLUTION_8BIT) | ADC_CFG1_ADIV(ADC_CLK_DIVIDE_1);
    baseAddr->CFG2 = ADC_CFG2_SMPLTS(ADC_DEFAULT_SAMPLE_TIME);

    for(idx = 0U; idx < ADC_CV_COUNT; idx++)
    {
        baseAddr->CV[idx] = ADC_CV_CV(0U);
    }

    baseAddr->SC2 = ADC_SC2_REFSEL(ADC_VOLTAGEREF_VREF) | ADC_SC2_DMAEN(0x00U) | ADC_SC2_ACREN(0x00U) | ADC_SC2_ACFGT(0x00U) | ADC_SC2_ACFE(0x00U) |
                    ADC_SC2_ADTRG(0x00U);
    baseAddr->SC3 = ADC_SC3_AVGS(ADC_AVERAGE_4) | ADC_SC3_AVGE(0x00U) | ADC_SC3_ADCO(0x00U) | ADC_SC3_CAL(0x00U);
    baseAddr->USR_OFS = ADC_USR_OFS_USR_OFS(0U);
    baseAddr->UG = ADC_UG_UG(ADC_DEFAULT_USER_GAIN);

#if FEATURE_ADC_HAS_EXTRA_NUM_REGS
    for(idx = 0U; idx < ADC_aSC1_COUNT; idx++)
    {
        baseAddr->aSC1[idx] = ADC_aSC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_aSC1_AIEN(0x00U);
    }
#endif /* FEATURE_ADC_HAS_EXTRA_NUM_REGS */

    ADC_SetPretriggerSelect(instance, ADC_PRETRIGGER_SEL_PDB);
    ADC_SetTriggerSelect(instance, ADC_TRIGGER_SEL_PDB);
    ADC_DRV_SetSwPretrigger(instance, ADC_SW_PRETRIGGER_DISABLED);

    /* Reset ADC Supply Monitoring - available only for ADC 0 */
    if(instance == 0u)
    {
        SIM_Type * const simBase = SIM;
        ADC_SetSupplyMonitoringEnableFlag(simBase, false);

        simBase->CHIPCTL &= ~SIM_CHIPCTL_ADC_SUPPLY_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitHwCompareStruct
 * Description   : This function initializes the Hardware Compare configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Compare feature (ADC_DRV_ConfigHwCompare),
 * otherwise all members must be written by the caller. This function insures that all
 * members are written with safe values, so the user can modify the desired members.
 *
 * Implements : ADC_DRV_InitHwCompareStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitHwCompareStruct(adc_compare_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->compareEnable = false;
    config->compareGreaterThanEnable = false;
    config->compareRangeFuncEnable = false;
    config->compVal1 = 0U;
    config->compVal2 = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigHwCompare
 * Description   : This functions sets the configuration for the Hardware
 * Compare feature using the configuration structure.
 *
 * Implements : ADC_DRV_ConfigHwCompare_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigHwCompare(const uint32_t instance,
                             const adc_compare_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    ADC_Type * const base = s_adcBase[instance];
    ADC_SetHwCompareEnableFlag(base, config->compareEnable);
    ADC_SetHwCompareGtEnableFlag(base, config->compareGreaterThanEnable);
    ADC_SetHwCompareRangeEnableFlag(base, config->compareRangeFuncEnable);
    ADC_SetHwCompareComp1Value(base, config->compVal1);
    ADC_SetHwCompareComp2Value(base, config->compVal2);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetHwCompareConfig
 * Description   : This function returns the configuration for the Hardware
 * Compare feature.
 *
 * Implements : ADC_DRV_GetHwCompareConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetHwCompareConfig(const uint32_t instance,
                                adc_compare_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    const ADC_Type * const base = s_adcBase[instance];
    config->compareEnable = ADC_GetHwCompareEnableFlag(base);
    config->compareGreaterThanEnable = ADC_GetHwCompareGtEnableFlag(base);
    config->compareRangeFuncEnable = ADC_GetHwCompareRangeEnableFlag(base);
    config->compVal1 = ADC_GetHwCompareComp1Value(base);
    config->compVal2 = ADC_GetHwCompareComp2Value(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitHwAverageStruct
 * Description   : This function initializes the Hardware Average configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Average feature (ADC_DRV_ConfigHwAverage),
 * otherwise all members must be written by the caller. This function insures that all
 * members are written with safe values, so the user can modify the desired members.
 *
 * Implements : ADC_DRV_InitHwAverageStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitHwAverageStruct(adc_average_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->hwAvgEnable = false;
    config->hwAverage = ADC_AVERAGE_4;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigHwAverage
 * Description   : This function sets the configuration for the Hardware
 * Average feature.
 *
 * Implements : ADC_DRV_ConfigHwAverage_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigHwAverage(const uint32_t instance,
                             const adc_average_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    ADC_Type * const base = s_adcBase[instance];
    ADC_SetHwAverageEnableFlag(base, config->hwAvgEnable);
    ADC_SetHwAverageMode(base, config->hwAverage);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetHwAverageConfig
 * Description   : This function returns the configuration for the Hardware
 * Average feature.
 *
 * Implements : ADC_DRV_GetHwAverageConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetHwAverageConfig(const uint32_t instance,
                                adc_average_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    const ADC_Type * const base = s_adcBase[instance];
    config->hwAvgEnable = ADC_GetHwAverageEnableFlag(base);
    config->hwAverage = ADC_GetHwAverageMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitChanStruct
 * Description   : This function initializes the control channel
 * configuration structure to default values (Reference Manual resets). This function should
 * be called on a structure before using it to configure a channel (ADC_DRV_ConfigChan), otherwise
 * all members must be written by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * Implements : ADC_DRV_InitChanStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitChanStruct(adc_chan_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->interruptEnable = false;
    config->channel = ADC_INPUTCHAN_DISABLED;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigChan
 * Description   : This function sets a control channel configuration.
 *
 * When Software Trigger mode is enabled, configuring control channel index 0,
 * implicitly triggers a new conversion on the selected ADC input channel.
 * Therefore, ADC_DRV_ConfigChan can be used for sw-triggering conversions.
 *
 * Configuring any control channel while it is actively controlling a conversion
 * (sw or hw triggered) will implicitly abort the on-going conversion.
 *
 * Implements : ADC_DRV_ConfigChan_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigChan(const uint32_t instance,
                        const uint8_t chanIndex,
                        const adc_chan_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chanIndex < ADC_CTRL_CHANS_COUNT);
    DEV_ASSERT(config != NULL);

    ADC_Type * const base = s_adcBase[instance];

    /* ADC_INPUTCHAN_SUPPLY_ can only be used with ADC 0,
     * If used, the feature must be enabled separately via supplyMonitoringEnable flag in adc_converter_config_t. */
    DEV_ASSERT((instance == 0u) || ((uint32_t)config->channel < (uint32_t)ADC_INPUTCHAN_SUPPLY_VDD) || \
                                   ((uint32_t)config->channel > (uint32_t)ADC_INPUTCHAN_SUPPLY_VDD_LV));
    ADC_SetInputChannel(base, chanIndex, config->channel, config->interruptEnable);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetChanConfig
 * Description   : This function returns the current configuration for a control
 * channel.
 *
 * Implements : ADC_DRV_GetChanConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetChanConfig(const uint32_t instance,
                           const uint8_t chanIndex,
                           adc_chan_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chanIndex < ADC_CTRL_CHANS_COUNT);
    DEV_ASSERT(config != NULL);

    const ADC_Type * const base = s_adcBase[instance];
    config->interruptEnable = ADC_GetChanInterruptEnableFlag(base, chanIndex);
    config->channel = ADC_GetInputChannel(base, chanIndex);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_SetSwPretrigger
 * Description   : This function sets the software pretrigger - affects only first 4 control channels.
 *
 * Implements : ADC_DRV_SetSwPretrigger_Activity
 *END**************************************************************************/
void ADC_DRV_SetSwPretrigger(const uint32_t instance,
                             const adc_sw_pretrigger_t swPretrigger)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    SIM_Type * const simBase = SIM;
    uint32_t intermValue = 0U;
    uint32_t mask[ADC_INSTANCE_COUNT] = {0U};
#if (ADC_INSTANCE_COUNT == 1U)
    mask[0] = SIM_ADCOPT_ADC0SWPRETRG_MASK;
#elif (ADC_INSTANCE_COUNT == 2U)
    mask[0] = SIM_ADCOPT_ADC0SWPRETRG_MASK;
    mask[1] = SIM_ADCOPT_ADC1SWPRETRG_MASK;
#else
#error "Maximum supported value for ADC_INSTANCE_COUNT is 2."
#endif
    /* If SW Pretrigger Select is not enabled, the SW pretriggers will be ignored by ADC. */
    DEV_ASSERT((ADC_GetPretriggerSelect(instance) == ADC_PRETRIGGER_SEL_SW) || \
               (swPretrigger == ADC_SW_PRETRIGGER_DISABLED));

    intermValue = simBase->ADCOPT & (~ mask[instance]);
    switch(instance)
    {
    case 0:
        intermValue |= SIM_ADCOPT_ADC0SWPRETRG(swPretrigger);
        break;
    case 1:
        intermValue |= SIM_ADCOPT_ADC1SWPRETRG(swPretrigger);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

    simBase->ADCOPT = intermValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_WaitConvDone
 * Description   : This functions waits for a conversion to complete by
 * continuously polling the Conversion Active Flag.
 *
 * Implements : ADC_DRV_WaitConvDone_Activity
 *END**************************************************************************/
void ADC_DRV_WaitConvDone(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    const ADC_Type * const base = s_adcBase[instance];
    while (ADC_GetConvActiveFlag(base) == true)
    {
        /* Wait for conversion to finish */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetConvCompleteFlag
 * Description   : This function returns the state of the Conversion Complete
 * flag for a control channel. This flag is set when a conversion is complete
 * or the condition generated by the Hardware Compare feature is evaluated to true.
 *
 * Implements : ADC_DRV_GetConvCompleteFlag_Activity
 *END**************************************************************************/
bool ADC_DRV_GetConvCompleteFlag(const uint32_t instance,
                                 const uint8_t chanIndex)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chanIndex < ADC_CTRL_CHANS_COUNT);

    const ADC_Type * const base = s_adcBase[instance];

#if FEATURE_ADC_HAS_EXTRA_NUM_REGS
    uint32_t tmp = base->aSC1[chanIndex];
    tmp = (tmp & ADC_aSC1_COCO_MASK) >> ADC_aSC1_COCO_SHIFT;
#else
    uint32_t tmp = base->SC1[chanIndex];
    tmp = (tmp & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT;
#endif /* FEATURE_ADC_HAS_EXTRA_NUM_REGS */

    return (tmp != 0u) ? true : false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetChanResult
 * Description   : This function returns the conversion result from a
 * control channel.
 *
 * Implements : ADC_DRV_GetChanResult_Activity
 *END**************************************************************************/
void ADC_DRV_GetChanResult(const uint32_t instance,
                           const uint8_t chanIndex,
                           uint16_t * const result)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(result != NULL);

    const ADC_Type * const base = s_adcBase[instance];

#if FEATURE_ADC_HAS_EXTRA_NUM_REGS

    DEV_ASSERT(chanIndex < ADC_aR_COUNT);

    uint32_t tmp = base->aR[chanIndex];
    tmp = (tmp & ADC_aR_D_MASK) >> ADC_aR_D_SHIFT;
#else

    DEV_ASSERT(chanIndex < ADC_R_COUNT);

    uint32_t tmp = base->R[chanIndex];
    tmp = (tmp & ADC_R_D_MASK) >> ADC_R_D_SHIFT;
#endif /* FEATURE_ADC_HAS_EXTRA_NUM_REGS */

    *result = (uint16_t)tmp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_AutoCalibration
 * Description   : This functions executes an Auto-Calibration sequence. It
 * is recommended to run this sequence before using the ADC converter.
 * this function will check and reset clock divide based the adc frequency.
 * an error will be displayed if adc_freq too big
 * this function will set satisfy clock divide,start calibration.
 * final this function: restore adc clock divide,hardware average and trigger settings.
 *
 * Implements : ADC_DRV_AutoCalibration_Activity
 *END**************************************************************************/
void ADC_DRV_AutoCalibration(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];
    /* set hardware average to maximum and set software trigger*/
    bool hwavgen = ADC_GetHwAverageEnableFlag(base);
    adc_average_t hwavg = ADC_GetHwAverageMode(base);
    adc_trigger_t trig = ADC_GetTriggerMode(base);
    uint8_t sampletime = ADC_GetSampleTime(base);
    ADC_SetHwAverageMode(base, ADC_AVERAGE_32);
    ADC_SetHwAverageEnableFlag(base, true);
    ADC_SetTriggerMode(base, ADC_TRIGGER_SOFTWARE);
    /* Set the sample time to the reset value because it affects the
        calibration duration but not the results
     */
    ADC_SetSampleTime(base, ADC_RESET_SAMPLE_TIME_VALUE);

    base->CLPS = 0x00u;
    base->CLP3 = 0x00u;
    base->CLP2 = 0x00u;
    base->CLP1 = 0x00u;
    base->CLP0 = 0x00u;
    base->CLPX = 0x00u;
    base->CLP9 = 0x00u;

    /*Set clock divider */
    clock_names_t adc_clocks[ADC_INSTANCE_COUNT] = ADC_CLOCKS;
    uint32_t adc_freq = 0u;
    adc_clk_divide_t adc_clk_divide_res = ADC_GetClockDivide(base);
    adc_clk_divide_t adc_clk_divide = ADC_CLK_DIVIDE_1;
    status_t clk_status = CLOCK_SYS_GetFreq(adc_clocks[instance], &adc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    (void) clk_status;
    DEV_ASSERT(adc_freq >= ADC_CLOCK_FREQ_MIN_RUNTIME);
    if ((adc_freq / (uint32_t)(1UL << ((uint32_t)(adc_clk_divide_res)))) <= (ADC_CLOCK_FREQ_MAX_RUNTIME / 2U))
    {
        /* no action if adc_freq is satisfy */
    }
    else
    {
        if ((adc_freq / 2U) <= (ADC_CLOCK_FREQ_MAX_RUNTIME / 2U))
        {
            adc_clk_divide = ADC_CLK_DIVIDE_2;
        }
        else if ((adc_freq / 4U) <= (ADC_CLOCK_FREQ_MAX_RUNTIME / 2U))
        {
            adc_clk_divide = ADC_CLK_DIVIDE_4;
        }
        else if ((adc_freq / 8U) <= (ADC_CLOCK_FREQ_MAX_RUNTIME / 2U))
        {
            adc_clk_divide = ADC_CLK_DIVIDE_8;
        }
        else
        {
            /* frequency is greater than required clock for calibration */
            DEV_ASSERT(false);
        }
        ADC_SetClockDivide(base, adc_clk_divide);
    }
    /* start calibration */
    ADC_SetCalibrationActiveFlag(base, true);
    while (ADC_GetCalibrationActiveFlag(base))
    {
        /* Wait for calibration to finish */
    }

    /* restore adc clock divide*/
    ADC_SetClockDivide(base, adc_clk_divide_res);
    /* restore hardware average and trigger settings*/
    ADC_SetHwAverageEnableFlag(base, hwavgen);
    ADC_SetHwAverageMode(base, hwavg);
    ADC_SetTriggerMode(base, trig);
    ADC_SetSampleTime(base, sampletime);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitUserCalibrationStruct
 * Description   : This function initializes the User Calibration configuration
 * structure to default values (Reference Manual resets). This function should be called
 * on a structure before using it to configure the User Calibration feature (ADC_DRV_ConfigUserCalibration),
 * otherwise all members must be written by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * Implements : ADC_DRV_InitUserCalibrationStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitUserCalibrationStruct(adc_calibration_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->userGain = (uint16_t)ADC_DEFAULT_USER_GAIN;
    config->userOffset = (uint16_t)0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigUserCalibration
 * Description   : This function sets the configuration for the user calibration
 * registers.
 *
 * Implements : ADC_DRV_ConfigUserCalibration_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigUserCalibration(const uint32_t instance,
                                   const adc_calibration_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    ADC_Type * const base = s_adcBase[instance];
    ADC_SetUserGainValue(base, config->userGain);
    ADC_SetUserOffsetValue(base, config->userOffset);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetUserCalibration
 * Description   : This function returns the current user calibration
 * register values.
 *
 * Implements : ADC_DRV_GetUserCalibration_Activity
 *END**************************************************************************/
void ADC_DRV_GetUserCalibration(const uint32_t instance,
                                adc_calibration_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    const ADC_Type * const base = s_adcBase[instance];
    config->userGain = ADC_GetUserGainValue(base);
    config->userOffset = ADC_GetUserOffsetValue(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetInterruptNumber
 * Description   : This function returns the interrupt number for the specified ADC instance.
 *
 * Implements : ADC_DRV_GetInterruptNumber_Activity
 *END**************************************************************************/
IRQn_Type ADC_DRV_GetInterruptNumber(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    static const IRQn_Type adcIrqId[ADC_INSTANCE_COUNT] = ADC_IRQS;
    IRQn_Type irqId = adcIrqId[instance];

    return irqId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ClearLatchedTriggers
 * Description   : This function clears all trigger latched flags of the ADC instance.
 *
 * Implements : ADC_DRV_ClearLatchedTriggers_Activity
 *END**************************************************************************/
void ADC_DRV_ClearLatchedTriggers(const uint32_t instance,
                                  const adc_latch_clear_t clearMode)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT((clearMode == ADC_LATCH_CLEAR_WAIT) || (clearMode == ADC_LATCH_CLEAR_FORCE));

    ADC_Type * const base = s_adcBase[instance];
    if (clearMode == ADC_LATCH_CLEAR_FORCE)
    {
        ADC_ClearLatchTriggers(base);
    }

    while (ADC_GetTriggerLatchFlags(base) != 0u)
    {
        /* Wait for latched triggers to be processed */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ClearTriggerErrors
 * Description   : This function clears all trigger error flags of the ADC instance.
 *
 * Implements : ADC_DRV_ClearTriggerErrors_Activity
 *END**************************************************************************/
void ADC_DRV_ClearTriggerErrors(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];

    base->SC2 |= ADC_SC2_TRGSTERR_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetTriggerErrorFlags
 * Description   : This function returns the trigger error flags bits of the ADC instance.
 *
 * Implements : ADC_DRV_GetTriggerErrorFlags_Activity
 *END**************************************************************************/
uint32_t ADC_DRV_GetTriggerErrorFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    const ADC_Type * const base = s_adcBase[instance];

    uint32_t trig_errors = (base->SC2 & ADC_SC2_TRGSTERR_MASK) >> ADC_SC2_TRGSTERR_SHIFT;

    return trig_errors;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
