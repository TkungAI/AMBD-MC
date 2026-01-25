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
 * @file ftm_common.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver common file as external; they are needed
 * at driver initialization to install the correct interrupt handler, but are not
 * a part of the public API.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variables are defined in the source file to make transition to other
 * platforms easier.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially enum<i>' to 'essentially Boolean'. This is required by
 * the conversion of a enum type into a bool type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include "ftm_common.h"
#include "ftm_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for FTM instances. */
FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT] = FTM_BASE_PTRS;

/*! @brief Interrupt vectors for the FTM peripheral. */
const IRQn_Type g_ftmIrqId[FTM_INSTANCE_COUNT][FEATURE_FTM_CHANNEL_COUNT] = FTM_IRQS;
const IRQn_Type g_ftmFaultIrqId[FTM_INSTANCE_COUNT] = FTM_Fault_IRQS;
const IRQn_Type g_ftmOverflowIrqId[FTM_INSTANCE_COUNT] = FTM_Overflow_IRQS;
const IRQn_Type g_ftmReloadIrqId[FTM_INSTANCE_COUNT] = FTM_Reload_IRQS;

#ifdef ERRATA_E10856
bool faultDetection = false;
#endif

/*! @brief Pointer to runtime state structure. */
ftm_state_t * ftmStatePtr[FTM_INSTANCE_COUNT] = {NULL};

/*! @brief  Select external clock pin or clock source for peripheral */
static const clock_names_t g_ftmExtClockSel[FTM_INSTANCE_COUNT][2] = {{SIM_FTM0_CLOCKSEL, FTM0_CLK},
                                                                      {SIM_FTM1_CLOCKSEL, FTM1_CLK},
#if (FTM_INSTANCE_COUNT > 2U)
                                                                      {SIM_FTM2_CLOCKSEL, FTM2_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 3U)
                                                                      {SIM_FTM3_CLOCKSEL, FTM3_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 4U)
                                                                      {SIM_FTM4_CLOCKSEL, FTM4_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 5U)
                                                                      {SIM_FTM5_CLOCKSEL, FTM5_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 6U)
                                                                      {SIM_FTM6_CLOCKSEL, FTM6_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 7U)
                                                                      {SIM_FTM7_CLOCKSEL, FTM7_CLK},
#endif
                                                                      };

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Init
 * Description   : Initializes the FTM driver and get the clock frequency value
 * which select one of three possible clock sources for the FTM counter.
 *
 * Implements    : FTM_DRV_Init_Activity
 *END**************************************************************************/
status_t FTM_DRV_Init(uint32_t instance,
                      const ftm_user_config_t * info,
                      ftm_state_t * state)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(info != NULL);
    DEV_ASSERT(state != NULL);
    DEV_ASSERT((info->ftmClockSource == FTM_CLOCK_SOURCE_SYSTEMCLK) || \
               (info->ftmClockSource == FTM_CLOCK_SOURCE_FIXEDCLK) || \
               (info->ftmClockSource == FTM_CLOCK_SOURCE_EXTERNALCLK));
    FTM_Type * ftmBase = g_ftmBase[instance];
    status_t status = STATUS_SUCCESS;
    uint8_t index = 0U;

    /* Check if this instance is already initialized */
    if (ftmStatePtr[instance] != NULL)
    {
        status = STATUS_ERROR;
    }
    else
    {
        /* Configure state structure. */
        state->ftmClockSource = info->ftmClockSource;
        state->ftmMode = FTM_MODE_NOT_INITIALIZED;
        state->ftmPeriod = 0U;
        ftmStatePtr[instance] = state;

        for (index = 0U; index < FEATURE_FTM_CHANNEL_COUNT; index++)
        {
            state->measurementResults[index] = 0U;
            state->channelsCallbacksParams[index] = NULL;
            state->channelsCallbacks[index] = NULL;
            state->enableNotification[index] = false;
        }

        /* The reset operation doesn't care about write protection. FTM_DRV_Reset will
         * disable this protection.*/
        FTM_DRV_Reset(ftmBase);
        FTM_DRV_InitModule(ftmBase, info->ftmPrescaler);
        /* Get clock name used to configure the FlexTimer module */
        state->ftmSourceClockFrequency = FTM_DRV_GetFrequency(instance);
        /* Check the functional clock is selected for FTM */
        DEV_ASSERT(state->ftmSourceClockFrequency > 0U);
    }

    if (STATUS_SUCCESS == status)
    {
        /* Check if the mode operation in PWM mode */
        if ((FTM_MODE_EDGE_ALIGNED_PWM == info->ftmMode) || (FTM_MODE_CEN_ALIGNED_PWM == info->ftmMode) || (FTM_MODE_OUTPUT_COMPARE == info->ftmMode))
        {
            /* Configure sync for between registers and buffers */
            status = FTM_DRV_SetSync(instance, &(info->syncMethod));
        }

        /* Enable the generation of initialization trigger on chip module */
        FTM_DRV_SetInitTriggerCmd(ftmBase, info->enableInitializationTrigger);
        FTM_DRV_SetBdmMode(ftmBase, info->BDMMode);

        /* Check if enable interrupt in counter mode */
        if (info->isTofIsrEnabled)
        {
            FTM_DRV_SetTimerOverflowInt(ftmBase, true);
            INT_SYS_EnableIRQ(g_ftmOverflowIrqId[instance]);
        }
        else
        {
            FTM_DRV_SetTimerOverflowInt(ftmBase, false);
            INT_SYS_DisableIRQ(g_ftmOverflowIrqId[instance]);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Deinit
 * Description   : Shuts down the FTM driver.
 * First, FTM_DRV_Init must be called. Then this function will disables the FTM module.
 *
 * Implements    : FTM_DRV_Deinit_Activity
 *END**************************************************************************/
status_t FTM_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Reset all FTM register */
    FTM_DRV_Reset(ftmBase);
    ftmStatePtr[instance] = NULL;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetDefaultConfig
 * Description   : This function will get the default configuration values
 * in the structure which is used as a common use-case.
 * Return        : None
 * Implements    : FTM_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void FTM_DRV_GetDefaultConfig(ftm_user_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->syncMethod.softwareSync     = true;
    config->syncMethod.hardwareSync0    = false;
    config->syncMethod.hardwareSync1    = false;
    config->syncMethod.hardwareSync2    = false;
    config->syncMethod.maxLoadingPoint  = false;
    config->syncMethod.minLoadingPoint  = false;
    config->syncMethod.inverterSync     = FTM_SYSTEM_CLOCK;
    config->syncMethod.outRegSync       = FTM_SYSTEM_CLOCK;
    config->syncMethod.maskRegSync      = FTM_SYSTEM_CLOCK;
    config->syncMethod.initCounterSync  = FTM_SYSTEM_CLOCK;
    config->syncMethod.autoClearTrigger = true;
    config->syncMethod.syncPoint        = FTM_UPDATE_NOW;

    config->ftmMode                     = FTM_MODE_EDGE_ALIGNED_PWM;
    config->ftmPrescaler                = FTM_CLOCK_DIVID_BY_1;
    config->ftmClockSource              = FTM_CLOCK_SOURCE_SYSTEMCLK;
    config->BDMMode                     = FTM_BDM_MODE_11;
    config->isTofIsrEnabled             = false;
    config->enableInitializationTrigger = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_MaskOutputChannels
 * Description   : This function will mask the output of the channels and at match
 * events will be ignored by the masked channels.
 *
 * Implements : FTM_DRV_MaskOutputChannels_Activity
 *END**************************************************************************/
status_t FTM_DRV_MaskOutputChannels(uint32_t instance,
                                    uint32_t channelsMask,
                                    bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetOutmaskReg(ftmBase, channelsMask);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetInitialCounterValue
 * Description   : This function configure the initial counter value. The counter
 * will get this value after an overflow event.
 *
 * Implements : FTM_DRV_SetInitialCounterValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetInitialCounterValue(uint32_t instance,
                                        uint16_t counterValue,
                                        bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetCounterInitVal(ftmBase, counterValue);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetHalfCycleReloadPoint
 * Description   : This function configure the value of the counter which will
 * generates an reload point.
 *
 * Implements : FTM_DRV_SetHalfCycleReloadPoint_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance,
                                         uint16_t reloadPoint,
                                         bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetHalfCycleValue(ftmBase, reloadPoint);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSoftOutChnValue
 * Description   : This function will force the output value of a channel to a specific value.
 * Before using this function it's mandatory to mask the match events using
 * FTM_DRV_MaskOutputChannels and to enable software output control using
 * FTM_DRV_SetSoftwareOutputChannelControl.
 *
 * Implements : FTM_DRV_SetSoftOutChnValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance,
                                    uint8_t channelsValues,
                                    bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    FTM_DRV_SetAllChnSoftwareCtrlVal(ftmBase, channelsValues ^ (uint8_t)ftmBase->POL);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSoftwareOutputChannelControl
 * Description   : This function will configure which output channel can be
 * software controlled.
 *
 * Implements : FTM_DRV_SetSoftwareOutputChannelControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance,
                                                 uint8_t channelsMask,
                                                 bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    FTM_DRV_SetAllChnSoftwareCtrlCmd(ftmBase, channelsMask);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetAllChnSoftwareOutputControl
 * Description   : This function will control list of channels by software to force
 * the output to specified value.
 * Despite the odd channels are configured as HIGH/LOW, they will be inverted in the following
 * configuration: COMP bit = 1 and CH(n)OCV and CH(n+1)OCV are HIGH. Please check Software
 * output control behavior chapter from RM.
 *
 * Implements : FTM_DRV_SetAllChnSoftwareOutputControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetAllChnSoftwareOutputControl(uint32_t instance,
                                                uint8_t channelMask,
                                                uint8_t channelValueMask,
                                                bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint16_t value = 0U;

    value = (uint16_t)(((uint16_t)channelValueMask ^ (uint16_t)ftmBase->POL) << (uint16_t)8U) | (uint16_t)channelMask;

    /* Enable and force the software control of channels output */
    FTM_DRV_SoftwareOutputControl(ftmBase, value);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetInvertingControl
 * Description   : This function will configure if the second channel of a pair
 * will be inverted or not.
 *
 * Implements : FTM_DRV_SetInvertingControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetInvertingControl(uint32_t instance,
                                     uint8_t channelsPairMask,
                                     bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetInvctrlReg(ftmBase, channelsPairMask);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetModuloCounterValue
 * Description   : This function configure the maximum counter value.
 *
 * Implements : FTM_DRV_SetModuloCounterValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetModuloCounterValue(uint32_t instance,
                                       uint16_t counterValue,
                                       bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetMod(ftmBase, counterValue);
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetOutputlevel
 * Description   : This function will set the channel edge or level on the selection
 * of the channel mode.
 *
 * Implements : FTM_DRV_SetOutputlevel_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetOutputlevel(uint32_t instance,
                                uint8_t channel,
                                uint8_t level)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Sets the channel edge or level selection */
    FTM_DRV_SetChnEdgeLevel(ftmBase, channel, level);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSync
 * Description   : This function configure the synchronization for PWM register
 * (CnV, MOD, CINT, HCR, OUTMASK).If this function is used whit wrong parameters
 * it's possible to generate wrong waveform. Registers synchronization need to
 * be configured for PWM and output compare mode.
 *
 * Implements : FTM_DRV_SetSync_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSync(uint32_t instance,
                         const ftm_pwm_sync_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    status_t retStatus = STATUS_SUCCESS;
    bool hardwareSync = param->hardwareSync0 || param->hardwareSync1 || param->hardwareSync2;

    /* Software and hardware triggers are not allowed in the same time */
    if ((param->softwareSync && hardwareSync) || (true != (param->softwareSync || hardwareSync)))
    {
        retStatus = STATUS_ERROR;
    }
    else if (param->softwareSync)
    {
        /* Configure sync for OUTMASK register */
        FTM_DRV_SetOutmaskSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for INVCTRL register */
        FTM_DRV_SetInvctrlSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for SWOCTRL register */
        FTM_DRV_SetSwoctrlSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_DRV_SetModCntinCvSoftwareSyncModeCmd(ftmBase, true);
        /* Configure synchronization method (waiting next loading point or now) */
        FTM_DRV_SetCounterSoftwareSyncModeCmd(ftmBase, param->syncPoint);
    }
    else
    {
        /* Configure sync for OUTMASK register */
        FTM_DRV_SetOutmaskHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for INVCTRL register */
        FTM_DRV_SetInvctrlHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for SWOCTRL register */
        FTM_DRV_SetSwoctrlHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_DRV_SetModCntinCvHardwareSyncModeCmd(ftmBase, true);
        /* Configure synchronization method (waiting next loading point or now) */
        FTM_DRV_SetCounterHardwareSyncModeCmd(ftmBase, (bool)param->syncPoint);
    }

    if (STATUS_SUCCESS == retStatus)
    {
        /* Enhanced PWM sync is used */
        FTM_DRV_SetPwmSyncModeCmd(ftmBase, true);
        /* Configure trigger source for sync */
        FTM_DRV_SetHardwareSyncTriggerSrc(ftmBase, 0U, param->hardwareSync0);
        FTM_DRV_SetHardwareSyncTriggerSrc(ftmBase, 1U, param->hardwareSync1);
        FTM_DRV_SetHardwareSyncTriggerSrc(ftmBase, 2U, param->hardwareSync2);
        /* Configure loading points */
        FTM_DRV_SetMaxLoadingCmd(ftmBase, param->maxLoadingPoint);
        FTM_DRV_SetMinLoadingCmd(ftmBase, param->minLoadingPoint);
        /* Configure sync for OUTMASK register */
        FTM_DRV_SetOutmaskPwmSyncModeCmd(ftmBase, (bool)param->maskRegSync);
        /* Configure sync for INVCTRL register */
        FTM_DRV_SetInvctrlPwmSyncModeCmd(ftmBase, param->inverterSync);
        /* Configure sync for SWOCTRL register */
        FTM_DRV_SetSwoctrlPwmSyncModeCmd(ftmBase, param->outRegSync);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_DRV_SetCntinPwmSyncModeCmd(ftmBase, param->initCounterSync);
        /* Configure if FTM clears TRIGj (j=0,1,2) when the hardware trigger j is detected. */
        FTM_DRV_SetHwTriggerSyncModeCmd(ftmBase, param->autoClearTrigger);
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GenerateHardwareTrigger
 * Description   : This function is used to configure a trigger source for FTM instance.
 * This allow a hardware trigger input which can be used in PWM synchronization.
 * Note that the hardware trigger is implemented only on trigger 1 for each instance.
 *
 * Implements : FTM_DRV_GenerateHardwareTrigger_Activity
 *END**************************************************************************/
status_t FTM_DRV_GenerateHardwareTrigger(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    SIM_Type * simeBase = SIM_BASE_PTRS;

    FTM_DRV_SyncBit(simeBase, instance, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_EnableInterrupts
 * Description   : This function will enable the generation a list of interrupts.
 * It includes the FTM overflow interrupts, the reload point interrupt, the fault
 * interrupt and the channel (n) interrupt.
 *
 * Implements : FTM_DRV_EnableInterrupts_Activity
 *END**************************************************************************/
status_t FTM_DRV_EnableInterrupts(uint32_t instance,
                                  uint32_t interruptMask)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint32_t chnlInts = (interruptMask & 0x000000FFU);
    uint8_t channel = 0U;

    /* Enable the timer overflow interrupt */
    if ((interruptMask & (uint32_t)FTM_TIME_OVER_FLOW_INT_ENABLE) != 0x0U)
    {
        FTM_DRV_SetTimerOverflowInt(ftmBase, true);
        INT_SYS_EnableIRQ(g_ftmOverflowIrqId[instance]);
    }

    /* Enable the fault interrupt */
    if ((interruptMask & (uint32_t)FTM_FAULT_INT_ENABLE) != 0x0U)
    {
        FTM_DRV_SetFaultInt(ftmBase, true);
        INT_SYS_EnableIRQ(g_ftmFaultIrqId[instance]);
    }

    /* Enable the reload interrupt */
    if ((interruptMask & (uint32_t)FTM_RELOAD_INT_ENABLE) != 0x0U)
    {
        FTM_DRV_SetReIntEnabledCmd(ftmBase, true);
        INT_SYS_EnableIRQ(g_ftmReloadIrqId[instance]);
    }

    /* Enable the channel interrupts */
    while (chnlInts != 0U)
    {
        if ((chnlInts & 0x1U) != 0x0U)
        {
            FTM_DRV_EnableChnInt(ftmBase, channel);
            INT_SYS_EnableIRQ(g_ftmIrqId[instance][channel]);
        }
        channel++;
        chnlInts = chnlInts >> 1U;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DisableInterrupts
 * Description   : This function is used to disable some interrupts.
 *
 * Implements : FTM_DRV_DisableInterrupts_Activity
 *END**************************************************************************/
void FTM_DRV_DisableInterrupts(uint32_t instance,
                               uint32_t interruptMask)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint32_t chnlInts = (interruptMask & 0x000000FFU);
    uint8_t channel = 0U;

    /* Disable the timer overflow interrupt */
    if ((interruptMask & (uint32_t)FTM_TIME_OVER_FLOW_INT_ENABLE) != 0x0U)
    {
        FTM_DRV_SetTimerOverflowInt(ftmBase, false);
        INT_SYS_DisableIRQ(g_ftmOverflowIrqId[instance]);
    }

    /* Disable the fault interrupt */
    if ((interruptMask & (uint32_t)FTM_FAULT_INT_ENABLE) != 0x0U)
    {
        FTM_DRV_SetFaultInt(ftmBase, false);
        INT_SYS_DisableIRQ(g_ftmFaultIrqId[instance]);
    }

    /* Disable the reload interrupt */
    if ((interruptMask & (uint32_t)FTM_RELOAD_INT_ENABLE) != 0x0U)
    {
        FTM_DRV_SetReIntEnabledCmd(ftmBase, false);
        INT_SYS_DisableIRQ(g_ftmReloadIrqId[instance]);
    }

    /* Disable the channel interrupts */
    while (chnlInts != 0U)
    {
        if ((chnlInts & 0x1U) != 0x0U)
        {
            FTM_DRV_DisableChnInt(ftmBase, channel);
            INT_SYS_DisableIRQ(g_ftmIrqId[instance][channel]);
        }
        channel++;
        chnlInts = chnlInts >> 1U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetEnabledInterrupts
 * Description   : This function will get the enabled FTM interrupts.
 *
 * Implements : FTM_DRV_GetEnabledInterrupts_Activity
 *END**************************************************************************/
uint32_t FTM_DRV_GetEnabledInterrupts(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type const * ftmBase = g_ftmBase[instance];
    uint32_t enabledInterrupts = 0U;
    uint8_t channel = FEATURE_FTM_CHANNEL_COUNT;


    /* Check if timer overflow interrupt is enabled */
    if (FTM_DRV_IsOverflowIntEnabled(ftmBase) == true)
    {
        enabledInterrupts |= (uint32_t)FTM_TIME_OVER_FLOW_INT_ENABLE;
    }

    /* Check if fault interrupt is enabled */
    if (FTM_DRV_IsFaultIntEnabled(ftmBase) == true)
    {
        enabledInterrupts |= (uint32_t)FTM_FAULT_INT_ENABLE;
    }

    /* Check if the reload interrupt is enabled */
    if (FTM_DRV_IsReloadIntEnabled(ftmBase) == true)
    {
        enabledInterrupts |= (uint32_t)FTM_RELOAD_INT_ENABLE;
    }

    /* Check if the channel interrupts are enabled */
    while (channel > 0U)
    {
        channel--;
        if (FTM_DRV_IsChnIntEnabled(ftmBase, channel) == true)
        {
            enabledInterrupts |= (1UL << (uint32_t)channel);
        }
    }

    return enabledInterrupts;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetStatusFlags
 * Description   : This function will get the FTM status flags.
 *
 * Implements : FTM_DRV_GetStatusFlags_Activity
 *END**************************************************************************/
uint32_t FTM_DRV_GetStatusFlags(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type const * ftmBase = g_ftmBase[instance];
    uint8_t channel = 0U;
    uint32_t statusFlags = 0U;

    /* Check the timer flag */
    if (FTM_DRV_HasTimerOverflowed(ftmBase) == true)
    {
        statusFlags |= (uint32_t)FTM_TIME_OVER_FLOW_FLAG;
    }

    /* Check fault flag */
    if (FTM_DRV_GetDetectedFaultInput(ftmBase) == true)
    {
        statusFlags |= (uint32_t)FTM_FAULT_FLAG;
    }

    /* Check reload flag */
    if (FTM_DRV_GetReloadFlag(ftmBase) == true)
    {
        statusFlags |= (uint32_t)FTM_RELOAD_FLAG;
    }

    /* Check channel trigger flag */
    if (FTM_DRV_IsChnTriggerGenerated(ftmBase) == true)
    {
        statusFlags |= (uint32_t)FTM_CHANNEL_TRIGGER_FLAG;
    }

    /* Lower 8 bits contain the channel status flags */
    for (channel = 0U; channel < FEATURE_FTM_CHANNEL_COUNT; channel++)
    {
        if (FTM_DRV_HasChnEventOccurred(ftmBase, channel) == true)
        {
            statusFlags |= (1UL << (uint32_t)channel);
        }
    }

    return statusFlags;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_ClearStatusFlags
 * Description   : This function is used to clear the FTM status flags.
 *
 * Implements : FTM_DRV_ClearStatusFlags_Activity
 *END**************************************************************************/
void FTM_DRV_ClearStatusFlags(uint32_t instance,
                              uint32_t flagMask)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint32_t chnlMask = (flagMask & 0x000000FFU);
    uint8_t channel = 0U;

    /* Clear the timer overflow flag by writing a 0 to the bit while it is set */
    if ((flagMask & (uint32_t)FTM_TIME_OVER_FLOW_FLAG) != 0x0U)
    {
        FTM_DRV_ClearTimerOverflow(ftmBase);
    }

    /* Clear fault flag by writing a 0 to the bit while it is set */
    if ((flagMask & (uint32_t)FTM_FAULT_FLAG) != 0x0U)
    {
        FTM_DRV_ClearFaultsIsr(ftmBase);
    }

    /* Check reload flag by writing a 0 to the bit while it is set */
    if ((flagMask & (uint32_t)FTM_RELOAD_FLAG) != 0x0U)
    {
        FTM_DRV_ClearReloadFlag(ftmBase);
    }

    /* Clear channel trigger flag */
    if ((flagMask & (uint32_t)FTM_CHANNEL_TRIGGER_FLAG) != 0x0U)
    {
        FTM_DRV_ClearChnTriggerFlag(ftmBase);
    }

    /* Clear the channel status flags by writing a 0 to the bit */
    for (channel = 0U; channel < FEATURE_FTM_CHANNEL_COUNT; channel++)
    {
        if ((chnlMask & 0x00000001U) != 0x0U)
        {
            FTM_DRV_ClearChnEventStatus(ftmBase, channel);
        }
        chnlMask = chnlMask >> 1U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetFrequency
 * Description   : Retrieves the frequency of the clock source feeding the FTM counter.
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled.
 * The returned value is clock sources for the FTM counter.
 *
 * Implements    : FTM_DRV_GetFrequency_Activity
 *END**************************************************************************/
uint32_t FTM_DRV_GetFrequency(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type const * ftmBase = g_ftmBase[instance];
    status_t returnCode = STATUS_SUCCESS;
    clock_names_t ftmClkName;
    uint8_t clkPs;
    uint32_t frequency = 0U;
    const ftm_state_t * state = ftmStatePtr[instance];
    clkPs = (uint8_t)(1U << FTM_DRV_GetClockPs(ftmBase));

    switch (state->ftmClockSource)
    {
        case FTM_CLOCK_SOURCE_EXTERNALCLK:
            returnCode = CLOCK_SYS_GetFreq(g_ftmExtClockSel[instance][1], &frequency);
            if (0U == frequency)
            {
                ftmClkName = g_ftmExtClockSel[instance][0];
            }
            else
            {
                ftmClkName = g_ftmExtClockSel[instance][1];
            }

            /* Get the clock frequency value */
            returnCode = CLOCK_SYS_GetFreq(ftmClkName, &frequency);
            break;
        case FTM_CLOCK_SOURCE_FIXEDCLK:
            /* Get the clock frequency value */
            returnCode = CLOCK_SYS_GetFreq(SIM_RTCCLK_CLK, &frequency);
            break;
        case FTM_CLOCK_SOURCE_SYSTEMCLK:
            /* Get the clock frequency value */
            returnCode = CLOCK_SYS_GetFreq(CORE_CLK, &frequency);
            break;
        default:
            /* Nothing to do */
            break;
    }

    /* Checks the functional clock of FTM module */
    (void)returnCode;
    DEV_ASSERT(returnCode == STATUS_SUCCESS);

    return (uint32_t)(frequency / clkPs);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_ConvertFreqToPeriodTicks
 * Description   : This function converts the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hardware timer.
 *
 * Implements    : FTM_DRV_ConvertFreqToPeriodTicks_Activity
 *END**************************************************************************/
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                          uint32_t freqencyHz)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(freqencyHz != 0U);
    uint32_t uFTMhz;
    const ftm_state_t * state = ftmStatePtr[instance];
    uFTMhz = state->ftmSourceClockFrequency;

    return (uint16_t)(uFTMhz / freqencyHz);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterReset
 * Description   : This function will allow the FTM to restart the counter to
 * its initial counting value in the register.
 * Note that the configuration is set in the FTM_DRV_SetSync() function to make
 * sure that the FTM registers are updated by software trigger or hardware trigger.
 *
 * Implements : FTM_DRV_CounterReset_Activity
 *END**************************************************************************/
status_t FTM_DRV_CounterReset(uint32_t instance,
                              bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Updates the counter with its initial value */
    FTM_DRV_SetCounter(ftmBase, 0U);
    /* Set a software trigger or waiting a hardware trigger */
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*******************************************************************************
* EOF
******************************************************************************/
