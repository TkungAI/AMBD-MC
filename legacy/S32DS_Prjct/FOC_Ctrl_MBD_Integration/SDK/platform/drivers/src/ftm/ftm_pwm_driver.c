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
 * @file ftm_pwm_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially enum<i>' to 'essentially Boolean'
 * This is required by the conversion of a enum into a bit field value.
 */

#include "ftm_pwm_driver.h"
#include "ftm_hw_access.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitPwmIndependentChannel
 * Description   : Configures the PWM signal for the independent channel.
 *
 *END**************************************************************************/
static void FTM_DRV_InitPwmIndependentChannel(uint32_t instance,
                                              const ftm_pwm_param_t * param)
{
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t index = 0U;
    uint8_t channelId = 0U;
    uint8_t chnlPairNum = 0U;

    /* Configure independent PWM channels */
    for (index = 0U; index < param->nNumIndependentPwmChannels; index++)
    {
        channelId = param->pwmIndependentChannelConfig[index].hwChannelId;
        chnlPairNum =  (uint8_t)(channelId >> 1U);
        /* Configure POL bits for fail safe state */
        FTM_DRV_SetChnOutputPolarityCmd(ftmBase, channelId, (bool)param->pwmIndependentChannelConfig[index].safeState);

        /* Configure polarity of the PWM signal taking into consideration POL and ELSA/ELSB */
        if ((uint32_t)(param->pwmIndependentChannelConfig[index].safeState) == (uint32_t)(param->pwmIndependentChannelConfig[index].polarity))
        {
            FTM_DRV_SetChnEdgeLevel(ftmBase, channelId, (uint8_t)1U);
        }
        else
        {
            FTM_DRV_SetChnEdgeLevel(ftmBase, channelId, (uint8_t)2U);
        }

        if (param->pwmIndependentChannelConfig[index].enableSecondChannelOutput)
        {
            /* Configure dead time, and enable complementary channel. */
            FTM_DRV_SetDualChnDeadtimeCmd(ftmBase, chnlPairNum, param->pwmIndependentChannelConfig[index].deadTime);
            FTM_DRV_SetChnEdgeLevel(ftmBase, channelId + 1U, (uint8_t)2U);
            FTM_DRV_SetDualChnCompCmd(ftmBase, chnlPairNum, true);
            if (param->pwmIndependentChannelConfig[index].secondChannelPolarity == FTM_MAIN_INVERTED)
            {
                FTM_DRV_SetChnOutputPolarityCmd(ftmBase, channelId + 1U, (bool)param->pwmIndependentChannelConfig[index].safeState);
            }
            else
            {
                FTM_DRV_SetChnOutputPolarityCmd(ftmBase, channelId + 1U, !((bool)param->pwmIndependentChannelConfig[index].safeState));
            }
        }
        else
        {
            FTM_DRV_SetDualChnCompCmd(ftmBase, chnlPairNum, false);
        }

        /* Disable combined mode. */
        FTM_DRV_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
        /* Set MSB and MSA bits*/
        FTM_DRV_SetChnMSnBAMode(ftmBase, channelId, 3U);
        /* Configure fault mode */
        FTM_DRV_SetDualChnFaultCmd(ftmBase, chnlPairNum, ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED) ? true : false);
        /* Enable sync control for channels*/
        FTM_DRV_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, true);
        /* Enable the generation a trigger on chip module */
        FTM_DRV_SetChnTriggerCmd(ftmBase, channelId, param->pwmIndependentChannelConfig[index].enableExternalTrigger);

        /* Write FTMn_PWMLOAD register to enable synchronized loading points for the given channel */
        FTM_DRV_EnablePwmChannelOutputs(ftmBase, channelId);
        if (param->pwmIndependentChannelConfig[index].enableSecondChannelOutput)
        {
            FTM_DRV_EnablePwmChannelOutputs(ftmBase, channelId + 1U);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitPwmCombinedChannel
 * Description   : Configures the PWM signal for the combined channel.
 *
 *END**************************************************************************/
static void FTM_DRV_InitPwmCombinedChannel(uint32_t instance,
                                           const ftm_pwm_param_t * param)
{
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t index = 0U;
    uint8_t channelId = 0U;
    uint8_t chnlPairNum = 0U;

    /* Configure combined PWM channels */
    for (index = 0U; index < param->nNumCombinedPwmChannels; index++)
    {
        channelId = param->pwmCombinedChannelConfig[index].hwChannelId;
        chnlPairNum = (uint8_t)(channelId >> 1U);
        /* Check if the channel id is even number */
        DEV_ASSERT((channelId % 2U) == 0U);

        /* Configure POL bits for fail safe state */
        FTM_DRV_SetChnOutputPolarityCmd(ftmBase, channelId, (bool)param->pwmCombinedChannelConfig[index].mainChannelSafeState);

        /* Configure polarity of the PWM signal taking into consideration POL and ELSA/ELSB */
        if ((uint32_t)(param->pwmCombinedChannelConfig[index].mainChannelSafeState) == (uint32_t)(param->pwmCombinedChannelConfig[index].mainChannelPolarity))
        {
            FTM_DRV_SetChnEdgeLevel(ftmBase, channelId, (uint8_t)1U);
        }
        else
        {
            FTM_DRV_SetChnEdgeLevel(ftmBase, channelId, (uint8_t)2U);
        }

        FTM_DRV_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        /* Set MSB and MSA bits */
        FTM_DRV_SetChnMSnBAMode(ftmBase, channelId, 3U);

        /* Enable channel (n) output */
        FTM_DRV_EnablePwmChannelOutputs(ftmBase, channelId);
        /* Configure channel n+1 if it necessary. */
        if (param->pwmCombinedChannelConfig[index].enableSecondChannelOutput)
        {
            channelId = channelId + 1U;
            /* Configure POL bits for fail safe state */
            FTM_DRV_SetChnOutputPolarityCmd(ftmBase, channelId, (bool)param->pwmCombinedChannelConfig[index].secondChannelSafeState);
            FTM_DRV_SetChnEdgeLevel(ftmBase, channelId, (uint8_t)2U);
            /* Configure polarity of the second channel relative to main channel polarity. */
            if (param->pwmCombinedChannelConfig[index].secondChannelSafeState == param->pwmCombinedChannelConfig[index].mainChannelSafeState)
            {
                if(param->pwmCombinedChannelConfig[index].secondChannelPolarity == FTM_MAIN_DUPLICATED)
                {
                    /* If dead time is enabled and COMPx = 0 the channel n+1 is automatically disabled. */
                    DEV_ASSERT(!(param->pwmCombinedChannelConfig[index].deadTime));
                    FTM_DRV_SetDualChnCompCmd(ftmBase, chnlPairNum, false);
                }
                else
                {
                    FTM_DRV_SetDualChnCompCmd(ftmBase, chnlPairNum, true);
                }
            }
            else
            {
                if(param->pwmCombinedChannelConfig[index].secondChannelPolarity == FTM_MAIN_DUPLICATED)
                {
                    FTM_DRV_SetDualChnCompCmd(ftmBase, chnlPairNum, true);
                }
                else
                {
                    /* If dead time is enabled and COMPx = 0 the channel n+1 is automatically disabled. */
                    DEV_ASSERT(!(param->pwmCombinedChannelConfig[index].deadTime));
                    FTM_DRV_SetDualChnCompCmd(ftmBase, chnlPairNum, false);
                }
            }
            /* Enable channel (n+1) output */
            FTM_DRV_EnablePwmChannelOutputs(ftmBase, (uint8_t)(channelId));
        }

        /* Set fault control for the channel */
        FTM_DRV_SetDualChnFaultCmd(ftmBase, chnlPairNum, ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED) ? true : false);
        /* Enable sync control for channels */
        FTM_DRV_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, true);
        /* Enable the combine mode */
        FTM_DRV_SetDualChnCombineCmd(ftmBase, chnlPairNum, true);
        /* Configure the modified combine mode */
        FTM_DRV_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, param->pwmCombinedChannelConfig[index].enableModifiedCombine);
        /* Configure dead time */
        FTM_DRV_SetDualChnDeadtimeCmd(ftmBase, chnlPairNum, param->pwmCombinedChannelConfig[index].deadTime);
        /* Enable the generation a trigger on the channel (n) */
        channelId = (uint8_t)(chnlPairNum << 1U);
        FTM_DRV_SetChnTriggerCmd(ftmBase, channelId, param->pwmCombinedChannelConfig[index].enableExternalTrigger);
        /* Enable the generation a trigger on the channel (n+1) */
        channelId = channelId + 1U;
        FTM_DRV_SetChnTriggerCmd(ftmBase, channelId, param->pwmCombinedChannelConfig[index].enableExternalTriggerOnNextChn);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitPwmDutyCycleChannel
 * Description   : This function will update the duty cycle for the PWM signal
 * at the initialization.
 *
 *END**************************************************************************/
static status_t FTM_DRV_InitPwmDutyCycleChannel(uint32_t instance,
                                                const ftm_pwm_param_t * param)
{
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    status_t retVal = STATUS_SUCCESS;

    for (index = 0U; index < param->nNumIndependentPwmChannels; index++)
    {
        hwChannel = param->pwmIndependentChannelConfig[index].hwChannelId;
        /* Write CnV registers and setup duty cycle and phase values */
        retVal = FTM_DRV_UpdatePwmChannel(instance,
                                          hwChannel,
                                          FTM_PWM_UPDATE_IN_DUTY_CYCLE,
                                          param->pwmIndependentChannelConfig[index].uDutyCyclePercent,
                                          0U,
                                          false);
    }

    for (index = 0U; index < param->nNumCombinedPwmChannels; index++)
    {
        hwChannel = param->pwmCombinedChannelConfig[index].hwChannelId;
        /* Write CnV registers and setup duty cycle and phase values */
        retVal = FTM_DRV_UpdatePwmChannel(instance,
                                          hwChannel,
                                          FTM_PWM_UPDATE_IN_DUTY_CYCLE,
                                          param->pwmCombinedChannelConfig[index].firstEdge,
                                          param->pwmCombinedChannelConfig[index].secondEdge,
                                          false);
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitPwm
 * Description   : Configures duty cycle and frequency and starts outputting
 * PWM on specified channels.
 *
 * Implements    : FTM_DRV_InitPwm_Activity
 *END**************************************************************************/
status_t FTM_DRV_InitPwm(uint32_t instance,
                         const ftm_pwm_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    status_t retVal = STATUS_SUCCESS;
    uint8_t fltChannel = 0U;
    uint8_t faultChnNum = 0U;
    uint32_t tempInst = instance;
    ftm_state_t * state = ftmStatePtr[instance];
    FTM_Type * ftmBase = g_ftmBase[instance];

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        /* Disable counter clock */
        FTM_DRV_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        /* Clear the overflow flag */
        FTM_DRV_ClearTimerOverflow(ftmBase);
        /* Disable write protection */
        FTM_DRV_SetWriteProtectionCmd(ftmBase, false);
        /* Configure FTM mode */
        state->ftmMode = param->mode;
        /* Configure independent PWM channels */
        FTM_DRV_InitPwmIndependentChannel(instance, param);
        /* Configure combined PWM channels */
        FTM_DRV_InitPwmCombinedChannel(instance, param);
        /* Set enable outputs to be set to Initial/default value */
        FTM_DRV_SetInitChnOutputCmd(ftmBase, true);
        /* Enable faults (if faults were configured) */
        if ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED)
        {
            /* Configure PWM Output behavior */
            FTM_DRV_SetPwmFaultBehavior(ftmBase, ((param->faultConfig)->pwmOutputStateOnFault) ? true : false);
            /* Configure fault filter value */
            FTM_DRV_SetFaultInputFilterVal(ftmBase, ((param->faultConfig)->faultFilterValue));
            /* Check the FTM instances */
            if (tempInst <= 3U)
            {
                faultChnNum = (uint8_t)FTM_FEATURE_FAULT_CHANNELS;
            }
            else
            {
                faultChnNum = (uint8_t)(FTM_FEATURE_FAULT_CHANNELS >> 1U);
            }

            for (fltChannel = 0U; fltChannel < faultChnNum; fltChannel++)
            {
                if (true == (param->faultConfig)->ftmFaultChannelParam[fltChannel].faultChannelEnabled)
                {
                    /* Enable fault channel */
                    FTM_DRV_SetFaultInputCmd(ftmBase, fltChannel, true);
                    /* Configure fault filter */
                    FTM_DRV_SetFaultInputFilterCmd(ftmBase,
                                                   fltChannel,
                                                   ((param->faultConfig)->ftmFaultChannelParam[fltChannel].faultFilterEnabled) ? true : false);
                    /* Configure fault outputs */
                    FTM_DRV_SetChnFaultInputPolarityCmd(ftmBase,
                                                        fltChannel,
                                                        (((param->faultConfig)->ftmFaultChannelParam[fltChannel].ftmFaultPinPolarity  == FTM_POLARITY_HIGH)? true : false));
                }
            }

            /* Set fault interrupt */
            if (true == ((param->faultConfig)->pwmFaultInterrupt))
            {
                FTM_DRV_SetFaultInt(ftmBase, true);
            }

            /* Enable fault control */
            FTM_DRV_SetFaultControlMode(ftmBase, (uint32_t)(param->faultConfig)->faultMode);
        }

        /* Configure PWM mode: edge or center aligned */
        FTM_DRV_SetCpwms(ftmBase, (param->mode == FTM_MODE_CEN_ALIGNED_PWM) ? true : false);
        /* Calculate frequency of the give FTM hardware module - all channels will run at the same frequency */
        state->ftmPeriod = FTM_DRV_ConvertFreqToPeriodTicks(instance, param->uFrequencyHZ);
        /* Based on reference manual, in PWM mode CNTIN is to be set 0*/
        FTM_DRV_SetCounterInitVal(ftmBase, 0U);
        /* Write MOD register with the value of the period */
        /* For center aligned mode MOD register should be divided by 2 */
        /* For edge aligned mode period is determined by: MOD-CNTIN+1 */
        if (param->mode == FTM_MODE_CEN_ALIGNED_PWM)
        {
            FTM_DRV_SetMod(ftmBase, (uint16_t)(state->ftmPeriod >> 1U));
        }
        else
        {
            FTM_DRV_SetMod(ftmBase, (uint16_t)(state->ftmPeriod - 1U));
        }

        /* Update the duty cycle */
        retVal = FTM_DRV_InitPwmDutyCycleChannel(instance, param);

        if (STATUS_SUCCESS == retVal)
        {
            /* Configure dead time for combine mode */
            FTM_DRV_SetDeadtimeCount(ftmBase, param->deadTimeValue);
            FTM_DRV_SetDeadtimePrescale(ftmBase, param->deadTimePrescaler);
            FTM_DRV_Enable(ftmBase, true);
            FTM_DRV_SetPwmSyncMode(ftmBase, true);
            /* Set clock source to start counter */
            FTM_DRV_SetClockSource(ftmBase, state->ftmClockSource);
        }
        else
        {
            /* Restore FTM mode if initialization fails */
            state->ftmMode = FTM_MODE_NOT_INITIALIZED;
        }
    }
    else
    {
        retVal = STATUS_ERROR;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitPwm
 * Description   : Stops all PWM channels.
 *
 * Implements    : FTM_DRV_DeinitPwm_Activity
 *END**************************************************************************/
status_t FTM_DRV_DeinitPwm(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t channel;
    uint8_t chnlPairNum;
    ftm_state_t * state = ftmStatePtr[instance];
    DEV_ASSERT(state != NULL);

    /* Stop the FTM counter */
    FTM_DRV_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    for (channel = 0U; channel < FEATURE_FTM_CHANNEL_COUNT; channel++)
    {
        chnlPairNum = (uint8_t)(channel >> 1U);
        /* Disable PWM mode in hardware */
        FTM_DRV_SetChnCountVal(ftmBase, channel, 0U);
        FTM_DRV_SetChnEdgeLevel(ftmBase, channel, 0U);
        FTM_DRV_SetChnMSnBAMode(ftmBase, channel, 0U);
        FTM_DRV_SetCpwms(ftmBase, false);
        /* Configure polarity bit */
        FTM_DRV_SetChnOutputPolarityCmd(ftmBase, channel, false);
        FTM_DRV_DisablePwmChannelOutputs(ftmBase, channel);
        /* Clear the PWM synchronization */
        FTM_DRV_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, false);
        /* Clear combination for each pair of channels */
        FTM_DRV_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualChnDeadtimeCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualChnFaultCmd(ftmBase, chnlPairNum, false);
    }

    /* Clear the dead-time pre-scaler and value */
    FTM_DRV_SetExtDeadtimeValue(ftmBase, 0U);
    FTM_DRV_SetDeadtimePrescale(ftmBase, FTM_DEADTIME_DIVID_BY_1);
    FTM_DRV_SetDeadtimeCount(ftmBase, 0U);
    /* Clear fault control register */
    FTM_DRV_ClearFaultControl(ftmBase);
    /* Disable fault interrupt */
    FTM_DRV_SetFaultInt(ftmBase, false);
    /* Disable fault control */
    FTM_DRV_SetFaultControlMode(ftmBase, (uint32_t)FTM_FAULT_CONTROL_DISABLED);
    /* Clear the module value of the registers */
    FTM_DRV_SetMod(ftmBase, 0U);
    FTM_DRV_SetCounter(ftmBase, 0U);
    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmChannel
 * Description   : This function will update the duty cycle of PWM output.
 * - If the type of update in the duty cycle, this function will convert the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hardware timer. Period is calculated depending
 * on the operating mode of the FTM module and is stored in internal state structure.
 * firstEdge and secondEdge can have value between 0 - FTM_MAX_DUTY_CYCLE(0 = 0% duty
 * and FTM_MAX_DUTY_CYCLE = 100% duty). secondEdge value is used only whenFTM module is used in PWM combine mode.
 * - If the type of update in ticks, this function will get value in ticks to the hardware timer.
 * firstEdge and secondEdge variables can have value between 0 and ftmPeriod is stored in the state structure.
 * - in the modified combine mode, the firstEdge parameter is fixed value and only can modify the secondEdge variables
 * which is the initial value in the channel (n+1) match edge when the FTM counter has been ran.
 *
 * Implements    : FTM_DRV_UpdatePwmChannel_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdatePwmChannel(uint32_t instance,
                                  uint8_t channel,
                                  ftm_pwm_update_option_t typeOfUpdate,
                                  uint16_t firstEdge,
                                  uint16_t secondEdge,
                                  bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint16_t hwFirstEdge = 0U;
    uint16_t hwSecondEdge = 0U;
    uint16_t ftmPeriod = 0U;
    uint8_t chnlPairNum = (uint8_t)(channel >> 1U);
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    /* Get the newest period in the MOD register */
    ftmPeriod = FTM_DRV_GetMod(ftmBase);
    /* Check the mode operation in FTM module */
    if (state->ftmMode == FTM_MODE_CEN_ALIGNED_PWM)
    {
        ftmPeriod = (uint16_t)(ftmPeriod << 1U);
    }
    else if (state->ftmMode == FTM_MODE_EDGE_ALIGNED_PWM)
    {
        ftmPeriod = (uint16_t)(ftmPeriod + 1U);
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    /* Check the type of update for PWM */
    if (FTM_PWM_UPDATE_IN_DUTY_CYCLE == typeOfUpdate)
    {
        if ((firstEdge <= FTM_MAX_DUTY_CYCLE) && (secondEdge <= FTM_MAX_DUTY_CYCLE))
        {
            /* Calculate DutyCycle based of the previously calculated frequency*/
            /* For greater resolution the DutyCycle values are in the range [0. FTM_MAX_DUTY_CYCLE]
             *  where 0 = 0% or PWM always at Low and FTM_MAX_DUTY_CYCLE = 100% or PWM always HIGH;
             *  a value of 0x4000 is equivalent of 50% DutyCycle. */
            hwFirstEdge = (uint16_t)((ftmPeriod * firstEdge) >> FTM_DUTY_TO_TICKS_SHIFT);
            hwSecondEdge = (uint16_t)((ftmPeriod * secondEdge) >> FTM_DUTY_TO_TICKS_SHIFT);
            /* adjust DutyCycle if 100% value is to be achieved. */
            if (FTM_MAX_DUTY_CYCLE == firstEdge)
            {
                /* If expected duty is 100% then increase by 1 the value that is to be written
                 *  to Hardware so it will exceed value of period */
                hwFirstEdge = (uint16_t)(hwFirstEdge + 1U);
            }
        }
        else
        {
            retStatus = STATUS_ERROR;
        }
    }
    else
    {
        if ((firstEdge <= ftmPeriod) && (secondEdge <= ftmPeriod))
        {
            hwFirstEdge = firstEdge;
            hwSecondEdge = secondEdge;
        }
        else
        {
            retStatus = STATUS_ERROR;
        }
    }

    if (STATUS_SUCCESS == retStatus)
    {
        if (true == FTM_DRV_GetDualChnCombineCmd(ftmBase, chnlPairNum))
        {
            if (true == FTM_DRV_GetDualChnMofCombineCmd(ftmBase, chnlPairNum))
            {
                /* Check the clock source for FTM counter is disabled or not */
                if (FTM_DRV_GetClockSource(ftmBase) == 0U)
                {
                    FTM_DRV_SetChnCountVal(ftmBase, (uint8_t)(chnlPairNum * 2U), hwFirstEdge);
                }
            }
            else
            {
                FTM_DRV_SetChnCountVal(ftmBase, (uint8_t)(chnlPairNum * 2U), hwFirstEdge);
            }

            /* Modify the initial value in the channel (n+1) match edge */
            FTM_DRV_SetChnCountVal(ftmBase, (uint8_t)((chnlPairNum * 2U) + 1U), hwSecondEdge);
        }
        else
        {
            /* Channel value is divided by 2 for up down counter mode to keep same duty */
            if (true == FTM_DRV_GetCpwms(ftmBase))
            {
                FTM_DRV_SetChnCountVal(ftmBase, channel, (uint16_t)(hwFirstEdge >> 1U));
            }
            else
            {
                FTM_DRV_SetChnCountVal(ftmBase, channel, hwFirstEdge);
            }
        }

        /* Software trigger is generated to change CnV registers */
        /* Before this please configure sync mechanism to use software trigger */
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

        /* Store the PWM period in the state structure */
        state->ftmPeriod = ftmPeriod;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmPeriod
 * Description   : This function will update the new period in the frequency or
 * in the counter value into mode register which modify the period of PWM signal
 * on the channel output.
 * - If the type of update in the duty cycle which is reused in FTM_DRV_UpdatePwmChannel
 * function to convert the newValue parameters representing frequency in Hz to
 * a period value to update the MOD register. The newValue parameter must be value
 * between 1U and maximum is the frequency of the FTM counter.
 * - If the type of update in ticks, this function will get value in counting to
 * the MOD register. The newValue parameter must be value between 1U and 0xFFFFU
 *
 * Implements : FTM_DRV_UpdatePwmPeriod_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdatePwmPeriod(uint32_t instance,
                                 ftm_pwm_update_option_t typeOfUpdate,
                                 uint32_t newValue,
                                 bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(newValue != 0U);
    uint32_t ftmPeriod = 0U;
    FTM_Type * ftmBase = g_ftmBase[instance];
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    /* Check the type of update for period in PWM mode */
    if (FTM_PWM_UPDATE_IN_TICKS == typeOfUpdate)
    {
        ftmPeriod = newValue;
    }
    else
    {
        if (newValue <= state->ftmSourceClockFrequency)
        {
            ftmPeriod = (uint32_t)FTM_DRV_ConvertFreqToPeriodTicks(instance, newValue);
        }
        else
        {
            retStatus = STATUS_ERROR;
        }
    }

    if (STATUS_SUCCESS == retStatus)
    {
        /* Check the ftmPeriod is invalid */
        DEV_ASSERT(ftmPeriod <= 0xFFFFU);
        /* Check the signal operation in which PWM mode */
        DEV_ASSERT((FTM_MODE_CEN_ALIGNED_PWM == state->ftmMode) || (FTM_MODE_EDGE_ALIGNED_PWM == state->ftmMode));
        if (FTM_MODE_CEN_ALIGNED_PWM == state->ftmMode)
        {
            ftmPeriod = (ftmPeriod >> 1U);
        }
        else
        {
            ftmPeriod = (ftmPeriod - 1U);
        }
        /* Set the new modulo value into MOD register */
        FTM_DRV_SetMod(ftmBase, (uint16_t)(ftmPeriod));
        /* Software trigger is generated to change MOD registers */
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

        /* Store the PWM period in the state structure */
        state->ftmPeriod = (uint16_t)ftmPeriod;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_FastUpdatePwmChannels
 * Description   : This function will update the duty cycle of PWM output for multiple channels.
 *
 * The main differences between this function and FTM_DRV_UpdatePwmChannel is the execution speed. This
 * feature makes this function ideal for applications like motor controlling.
 * The downside is the low flexibility of the parameters (this function accept only updates in ticks).
 *
 * Implements : FTM_DRV_FastUpdatePwmChannels_Activity
 *END**************************************************************************/
status_t FTM_DRV_FastUpdatePwmChannels(uint32_t instance,
                                       uint8_t numberOfChannels,
                                       const uint8_t * channels,
                                       const uint16_t * duty,
                                       bool softwareTrigger)
{
    FTM_Type * ftmBase = g_ftmBase[instance];
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(numberOfChannels <= FEATURE_FTM_CHANNEL_COUNT);
    uint8_t i;

    for (i = 0U; i < numberOfChannels; i++)
    {
        ((ftmBase)->CONTROLS[channels[i]].CnV) = duty[i];
    }

    if (softwareTrigger)
    {
       ftmBase->SYNC |= FTM_SYNC_SWSYNC_MASK;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_ControlChannelOutput
 * Description   : This function is used to control the final logic of the
 * channel output.
 *
 * Implements : FTM_DRV_ControlChannelOutput_Activity
 *END**************************************************************************/
status_t FTM_DRV_ControlChannelOutput(uint32_t instance,
                                      uint8_t channel,
                                      bool enableChannelOutput)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    if (enableChannelOutput == true)
    {
        /* Enable the channel output */
        FTM_DRV_EnablePwmChannelOutputs(ftmBase, channel);
    }
    else
    {
        /* Disable the channel output */
        FTM_DRV_DisablePwmChannelOutputs(ftmBase, channel);
    }

    return STATUS_SUCCESS;
}

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmPeriodDither
 * Description   : This function will use in the PWM period dithering. This value
 * is added to an internal accumulator at the end of each PWM period. The value is
 * updated with its write buffer value according to the register synchronization.
 *
 * Implements : FTM_DRV_UpdatePwmPeriodDither_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdatePwmPeriodDither(uint32_t instance,
                                       uint8_t newModFracVal,
                                       bool softwareTrigger)
{
    DEV_ASSERT((instance == 1U) || (instance == 2U));
    DEV_ASSERT(newModFracVal <= 32U);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetModFracVal(ftmBase, newModFracVal);
    /* Software trigger is generated to change MOD_MIRROR registers */
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmEdgeChannelDither
 * Description   : This function will use in the PWM edge dithering. This value
 * is added to the channel (n) internal accumulator at the end of each PWM period.
 * The FRACVAL is updated with its write buffer value according to the register
 * synchronization. The PWM edge dithering is not available when the channel in the
 * input capture modes, and the channel in output compare mode.
 *
 * Implements    : FTM_DRV_UpdatePwmEdgeChannelDither_Activity
 *END**************************************************************************/
status_t FTM_DRV_UpdatePwmEdgeChannelDither(uint32_t instance,
                                            uint8_t channel,
                                            uint8_t newMatchFracVal,
                                            bool softwareTrigger)
{
    DEV_ASSERT((instance == 1U) || (instance == 2U));
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    DEV_ASSERT(newMatchFracVal <= 32U);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetChnMatchFracVal(ftmBase, channel, newMatchFracVal);
    /* Software trigger is generated to change CnV_MIRROR registers */
    FTM_DRV_SetSoftwareTriggerCmd(ftmBase, softwareTrigger);

    return STATUS_SUCCESS;
}
#endif

#ifdef ERRATA_E10856
/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_PWM_DRV_IrqHandler
 * Description   : This function is used to workaround an errata which the safe state
 * is not removed from channel outputs after fault condition ends if SWOCTRL is being
 * used to control the pin. The FTM_MODE[FAULTM] should be configured for manual fault
 * clearing (0b10)
 * This function must be used in the TOF interrupt handler when a fault is detected to
 * ensure that the outputs return to the value configured by FTM_SWOCTR
 *
 * Implements    : FTM_PWM_DRV_IrqHandler_Activity
 *END**************************************************************************/
void FTM_PWM_DRV_IrqHandler(uint32_t instance,
                            uint32_t chnOutCtrlVal)
{
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* 1. Check the value of FTM_FMS[FAULTF] */
    if (FTM_DRV_GetDetectedFaultInput(ftmBase) == true)
    {
        faultDetection = true;
        /* 2. Write the FTM_OUTMASK register to set the bit */
        FTM_DRV_SetOutmaskReg(ftmBase, chnOutCtrlVal);
        /* 3. Clear fault conditions by reading the FTM_FMS register and then writing FTM_FMS with all zeroes */
        FTM_DRV_ClearFaultsIsr(ftmBase);
        /* 4. Clear the FTM_SC[TOF] bit by reading the FTM_SC register, then writing a 0 to FTM_SC[TOF] */
        FTM_DRV_ClearTimerOverflow(ftmBase);
        /* 5. Exit the interrupt handler to skip following steps */
    }
    else if (faultDetection == true)
    {
        /* 6. Clear the FTM_SWOCTRL by writing all zeroes to it */
        FTM_DRV_SetAllChnSoftwareCtrlCmd(ftmBase, 0x00U);
        FTM_DRV_SetAllChnSoftwareCtrlVal(ftmBase, 0x00U);
        /* 7. Write FTM_SWOCTRL with the desired value again */
        uint8_t u8chnOutCtrlVal = (uint8_t)(chnOutCtrlVal & 0xFFu);
        FTM_DRV_SetAllChnSoftwareCtrlCmd(ftmBase, u8chnOutCtrlVal);
        FTM_DRV_SetAllChnSoftwareCtrlVal(ftmBase, u8chnOutCtrlVal);
        /* 8. Clear the FTM_OUTMASK bits that were set in step 2 */
        FTM_DRV_SetOutmaskReg(ftmBase, 0x00U);
        /* 9. Clear the fault variable that was set in step 1 when the fault condition was originally detected */
        faultDetection = false;
        /* 10. Clear the FTM_SC[TOF] bit by reading the FTM_SC register, then writing a 0 to FTM_SC[TOF] */
        FTM_DRV_ClearTimerOverflow(ftmBase);
    }
    else
    {
        /* Nothing to do */
    }
}
#endif

/*******************************************************************************
* EOF
******************************************************************************/
