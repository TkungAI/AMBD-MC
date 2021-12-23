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
 * @file lpit_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
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
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 */

#include <stddef.h>
#include "lpit_driver.h"
#include "lpit_hw_access.h"
#include "interrupt_manager.h"
#include "clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for LPIT instances */
static LPIT_Type * const s_lpitBase[] = LPIT_BASE_PTRS;
/* Table to save LPIT indexes in PCC register map for clock configuration */
static const clock_names_t s_lpitClkNames[LPIT_INSTANCE_COUNT] = LPIT_CLOCK_NAMES;
/* LPIT functional clock variable which will be updated in some driver functions */
static uint32_t s_lpitSourceClockFrequency[LPIT_INSTANCE_COUNT] = {0};

/******************************************************************************
 * Code
 *****************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetDefaultConfig
 * Description   : This function gets default LPIT module configuration structure.
 *
 * Implements    : LPIT_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void LPIT_DRV_GetDefaultConfig(lpit_user_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->enableRunInDebug = false;
    config->enableRunInDoze = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetDefaultChanConfig
 * Description   : This function gets default timer channel configuration structure.
 *
 * Implements    : LPIT_DRV_GetDefaultChanConfig_Activity
 *END**************************************************************************/
void LPIT_DRV_GetDefaultChanConfig(lpit_user_channel_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->timerMode = LPIT_PERIODIC_COUNTER;
    config->periodUnits = LPIT_PERIOD_UNITS_MICROSECONDS;
    config->period = 1000000U;
    config->triggerSource = LPIT_TRIGGER_SOURCE_EXTERNAL;
    config->triggerSelect = 0U;
    config->enableReloadOnTrigger = false;
    config->enableStopOnInterrupt = false;
    config->enableStartOnTrigger = false;
    config->chainChannel = false;
    config->isInterruptEnabled = true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_Init
 * Description   : Initializes LPIT module.
 * This function resets LPIT module, enables the LPIT module, configures LPIT
 * module operation in Debug and DOZE mode. The LPIT configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other LPIT driver function.
 *
 * Implements    : LPIT_DRV_Init_Activity
 *END**************************************************************************/
void LPIT_DRV_Init(uint32_t instance,
                   const lpit_user_config_t *userConfig)
{
    LPIT_Type * base;
    status_t clkErr;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(userConfig != NULL);

    /* Gets current functional clock frequency of LPIT instance */
    clkErr = CLOCK_SYS_GetFreq(s_lpitClkNames[instance], &s_lpitSourceClockFrequency[instance]);
    /* Checks the functional clock of LPIT module */
    (void)clkErr;
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    DEV_ASSERT(s_lpitSourceClockFrequency[instance] > 0U);
    /* When resetting the LPIT module, a delay of 4 peripheral clock cycles
        must be ensured. This peripheral clock and the core clock running the
        code could be very different, two distinct cases are identified:
         - core_clk > peripheral_clk. This requires a delay loop to be implemented,
            and the delay value based on the ratio between the two frequencies.
         - core_clk <= peripheral_clk. This requires a short delay, which is usually
            below the delay caused naturally by the read-modify-write operation.
     */
    uint32_t core_freq = 0u;
    clkErr = CLOCK_SYS_GetFreq(CORE_CLK, &core_freq);
    (void)clkErr;
    uint32_t lpit_freq = s_lpitSourceClockFrequency[instance];
    uint32_t core_to_per_clock_ratio = (core_freq + (lpit_freq >> 1u)) / lpit_freq;
    base = s_lpitBase[instance];
    /* Resets LPIT module */
    LPIT_Reset(base, core_to_per_clock_ratio);
    /* Enables functional clock of LPIT module*/
    LPIT_Enable(base, core_to_per_clock_ratio);
    /* Sets LPIT operation in Debug and DOZE mode*/
    LPIT_SetTimerRunInDebugCmd(base, userConfig->enableRunInDebug);
    LPIT_SetTimerRunInDozeCmd(base, userConfig->enableRunInDoze);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_Deinit
 * Description   : De-initializes LPIT module.
 * This function disables LPIT module.
 * In order to use the LPIT module again, LPIT_DRV_Init must be called.
 *
 * Implements    : LPIT_DRV_Deinit_Activity
 *END**************************************************************************/
void LPIT_DRV_Deinit(uint32_t instance)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);

    base = s_lpitBase[instance];
    /* Disables LPIT module functional clock*/
    LPIT_Disable(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_InitChannel
 * Description   : Initializes LPIT channel.
 * This function initializes the LPIT timers by using a channel, this function
 * configures timer channel chaining, timer channel mode, timer channel period,
 * interrupt generation, trigger source, trigger select, reload on trigger,
 * stop on interrupt and start on trigger.
 * The timer channel number and its configuration structure shall be passed as arguments.
 * Timer channels do not start counting by default after calling this function.
 * The function LPIT_DRV_StartTimerChannels must be called to start the timer channel counting.
 * In order to re-configures the period, call the LPIT_DRV_SetTimerPeriodByUs or
 * LPIT_DRV_SetTimerPeriodByCount.
 *
 * Implements    : LPIT_DRV_InitChannel_Activity
 *END**************************************************************************/
status_t LPIT_DRV_InitChannel(uint32_t instance,
                              uint32_t channel,
                              const lpit_user_channel_config_t * userChannelConfig)
{
    LPIT_Type * base;
    status_t reVal = STATUS_SUCCESS;
    const IRQn_Type lpitIrqId[] = LPIT_IRQS;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(userChannelConfig != NULL);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    base = s_lpitBase[instance];

    if ((channel == 0U) && (userChannelConfig->chainChannel))
    {
        reVal = STATUS_ERROR;
    }
    else
    {
        /* Setups the timer channel chaining  */
        LPIT_SetTimerChannelChainCmd(base, channel, userChannelConfig->chainChannel);
        /*  Setups the timer channel operation mode */
        LPIT_SetTimerChannelModeCmd(base, channel, userChannelConfig->timerMode);
        if (userChannelConfig->periodUnits == LPIT_PERIOD_UNITS_MICROSECONDS)
        {
            /* Setups timer channel period in microsecond unit */
            reVal = LPIT_DRV_SetTimerPeriodByUs(instance, channel, userChannelConfig->period);
        }
        else
        {
            /* Setups timer channel period in count unit */
            LPIT_DRV_SetTimerPeriodByCount(instance, channel, userChannelConfig->period);
        }

        if (reVal == STATUS_SUCCESS)
        {
            /* Setups the timer channel trigger source, trigger select, reload on trigger,
            stop on timeout, start on trigger and channel chaining */
            LPIT_SetTriggerSourceCmd(base, channel, userChannelConfig->triggerSource);
            LPIT_SetTriggerSelectCmd(base, channel, userChannelConfig->triggerSelect);
            LPIT_SetReloadOnTriggerCmd(base, channel, userChannelConfig->enableReloadOnTrigger);
            LPIT_SetStopOnInterruptCmd(base, channel, userChannelConfig->enableStopOnInterrupt);
            LPIT_SetStartOnTriggerCmd(base, channel, userChannelConfig->enableStartOnTrigger);
            /* Setups interrupt generation for timer channel */
            if (userChannelConfig->isInterruptEnabled)
            {
                /* Enables interrupt generation */
                LPIT_EnableInterruptTimerChannels(base, (uint32_t)1U << channel);
                INT_SYS_EnableIRQ(lpitIrqId[channel]);
            }
            else
            {
                /* Disables interrupt generation */
                LPIT_DisableInterruptTimerChannels(base, (uint32_t)1U << channel);
                /* Only disable channel interrupt globally if each channel has a separate interrupt line */
#if (FEATURE_LPIT_HAS_NUM_IRQS_CHANS == LPIT_TMR_COUNT)
                INT_SYS_DisableIRQ(lpitIrqId[channel]);
#endif
            }
        }
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_StartTimerChannels
 * Description   : Starts timer channel counting.
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * Implements    : LPIT_DRV_StartTimerChannels_Activity
 *END**************************************************************************/
void LPIT_DRV_StartTimerChannels(uint32_t instance,
                                 uint32_t mask)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << LPIT_TMR_COUNT));

    base = s_lpitBase[instance];
    /* Starts timer channel counting */
    LPIT_StartTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_StopTimerChannels
 * Description   : Stop timer channel from counting.
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the LPIT_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * Implements    : LPIT_DRV_StopTimerChannels_Activity
 *END**************************************************************************/
void LPIT_DRV_StopTimerChannels(uint32_t instance,
                                uint32_t mask)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << LPIT_TMR_COUNT));

    base = s_lpitBase[instance];
    /* Stops timer channel from counting */
    LPIT_StopTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodByUs
 * Description   : Sets timer channel period in microseconds unit.
 * This function sets the timer channel period in microseconds
 * when timer channel mode is 32 bit periodic or dual 16 bit counter mode.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * Implements    : LPIT_DRV_SetTimerPeriodByUs_Activity
 *END**************************************************************************/
status_t LPIT_DRV_SetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel,
                                     uint32_t periodUs)
{
    LPIT_Type * base;
    lpit_timer_modes_t timerMode;
    status_t clkErr;
    status_t reVal = STATUS_SUCCESS;
    uint64_t count;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    /* Gets current functional clock frequency of LPIT instance */
    clkErr = CLOCK_SYS_GetFreq(s_lpitClkNames[instance], &s_lpitSourceClockFrequency[instance]);
    /* Checks the functional clock of LPIT module */
    (void)clkErr;
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    DEV_ASSERT(s_lpitSourceClockFrequency[instance] > 0U);

    base = s_lpitBase[instance];
    /* Calculates the count value, assign it to timer channel counter register.*/
    count = ((uint64_t)periodUs) * s_lpitSourceClockFrequency[instance];
    count = (count / 1000000U) - 1U;
    /* Gets current timer channel operation mode */
    timerMode = LPIT_GetTimerChannelModeCmd(base, channel);
    /* Checks whether the count is valid with timer channel operation mode */
    if (count <= MAX_PERIOD_COUNT)
    {
        if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
        {
            if (count > MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE)
            {
                reVal = STATUS_ERROR;
            }
            else
            {
                if (count > MAX_PERIOD_COUNT_16_BIT)
                {
                    /* Calculates the count value for dual 16 bit periodic counter mode */
                    count = ((count - (MAX_PERIOD_COUNT_16_BIT + 1U)) << 16U)
                            | (MAX_PERIOD_COUNT_16_BIT);
                }
            }
        }
    }
    else
    {
        reVal = STATUS_ERROR;
    }
    if (reVal == STATUS_SUCCESS)
    {
        /* Sets the timer channel period in count unit */
        LPIT_SetTimerPeriodByCount(base, channel, (uint32_t)count);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodInDual16ModeByUs
 * Description   : Sets timer channel period in microseconds unit.
 * This function sets the timer channel period in microseconds
 * when timer channel mode is dual 16 bit periodic counter mode.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * Implements    : LPIT_DRV_SetTimerPeriodInDual16ModeByUs_Activity
 *END**************************************************************************/
status_t LPIT_DRV_SetTimerPeriodInDual16ModeByUs(uint32_t instance,
                                                 uint32_t channel,
                                                 uint16_t periodHigh,
                                                 uint16_t periodLow)
{
    LPIT_Type * base;
    status_t reVal = STATUS_SUCCESS;
    status_t clkErr;
    uint64_t periodHighCount;
    uint64_t periodLowCount;
    uint64_t periodCount;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    /* Gets current functional clock frequency of LPIT instance */
    clkErr = CLOCK_SYS_GetFreq(s_lpitClkNames[instance], &s_lpitSourceClockFrequency[instance]);
    /* Checks the functional clock of LPIT module */
    (void)clkErr;
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    DEV_ASSERT(s_lpitSourceClockFrequency[instance] > 0U);

    base = s_lpitBase[instance];
    /* Calculates the count value of 16 bit higher period.*/
    periodHighCount = ((uint64_t)periodHigh) * s_lpitSourceClockFrequency[instance];
    periodHighCount = (periodHighCount / 1000000U) - 1U;

    /* Calculates the count value of 16 bit lower period.*/
    periodLowCount = ((uint64_t)periodLow) * s_lpitSourceClockFrequency[instance];
    periodLowCount = (periodLowCount / 1000000U) - 1U;
    /* Checks whether the count is valid */
    if ((periodHighCount > MAX_PERIOD_COUNT_16_BIT) || (periodLowCount > MAX_PERIOD_COUNT_16_BIT))
    {
        reVal = STATUS_ERROR;
    }
    else
    {
        periodCount = (periodHighCount << 16U) | periodLowCount;
        LPIT_SetTimerPeriodByCount(base, channel, (uint32_t)periodCount);
    }

    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetTimerPeriodByUs
 * Description   : Gets the timer channel period in microseconds.
 * The returned period here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 *
 * Implements    : LPIT_DRV_GetTimerPeriodByUs_Activity
 *END**************************************************************************/
uint64_t LPIT_DRV_GetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel)
{
    const LPIT_Type * base;
    status_t clkErr;
    lpit_timer_modes_t timerMode;
    uint64_t currentPeriod;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    /* Gets current functional clock frequency of LPIT instance */
    clkErr = CLOCK_SYS_GetFreq(s_lpitClkNames[instance], &s_lpitSourceClockFrequency[instance]);
    /* Checks the functional clock of LPIT module */
    (void)clkErr;
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    DEV_ASSERT(s_lpitSourceClockFrequency[instance] > 0U);

    base = s_lpitBase[instance];
    /* Gets current timer channel period in count.*/
    currentPeriod = LPIT_GetTimerPeriodByCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        if (currentPeriod > MAX_PERIOD_COUNT_16_BIT)
        {
            currentPeriod = (((currentPeriod >> 16U) + (currentPeriod & MAX_PERIOD_COUNT_16_BIT) + 2U) * 1000000U)
                              / s_lpitSourceClockFrequency[instance];
        }
        else
        {
            /* Converts period from count unit to microseconds unit for other modes */
            currentPeriod = ((currentPeriod + 1U) * 1000000U) / s_lpitSourceClockFrequency[instance];
        }
    }
    else
    {
        /* Converts period from count unit to microseconds unit for other modes */
        currentPeriod = ((currentPeriod + 1U) * 1000000U) / s_lpitSourceClockFrequency[instance];
    }
    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetCurrentTimerUs
 * Description   : Gets current timer channel counting value in microseconds unit.
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time.
 * The return counting value here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter or 32-bit trigger input capture.
 * Need to make sure the running time will not exceed the timer channel period.
 *
 * Implements    : LPIT_DRV_GetCurrentTimerUs_Activity
 *END**************************************************************************/
uint64_t LPIT_DRV_GetCurrentTimerUs(uint32_t instance,
                                    uint32_t channel)
{
    const LPIT_Type * base;
    status_t clkErr;
    lpit_timer_modes_t timerMode;
    uint64_t currentTime = 0U;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    base = s_lpitBase[instance];
    /* Gets current timer channel counting value */
    currentTime = LPIT_GetCurrentTimerCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_GetTimerChannelModeCmd(base, channel);
    /* Gets current functional clock frequency of LPIT instance */
    clkErr = CLOCK_SYS_GetFreq(s_lpitClkNames[instance], &s_lpitSourceClockFrequency[instance]);
    (void)clkErr;
    DEV_ASSERT(s_lpitSourceClockFrequency[instance] > 0U);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        currentTime = (((currentTime >> 16U) + (currentTime & MAX_PERIOD_COUNT_16_BIT)) * 1000000U)
                          / s_lpitSourceClockFrequency[instance];
    }
    else
    {
        /* Converts counting value to microseconds unit for other modes */
        currentTime = (currentTime * 1000000U) / s_lpitSourceClockFrequency[instance];
    }

    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodByCount
 * Description   : Sets the timer channel period in count unit.
 * This function sets the timer channel period in count unit.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * Implements    : LPIT_DRV_SetTimerPeriodByCount_Activity
 *END**************************************************************************/
void LPIT_DRV_SetTimerPeriodByCount(uint32_t instance,
                                    uint32_t channel,
                                    uint32_t count)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    base = s_lpitBase[instance];
    /* Sets the timer channel period in count unit */
    LPIT_SetTimerPeriodByCount(base, channel, count);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodInDual16ModeByCount
 * Description   : Sets the timer channel period in count unit.
 * This function sets the timer channel period in count unit when timer channel
 * mode is dual 16 periodic counter mode.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * Implements    : LPIT_DRV_SetTimerPeriodInDual16ModeByCount_Activity
 *END**************************************************************************/
void LPIT_DRV_SetTimerPeriodInDual16ModeByCount(uint32_t instance,
                                                uint32_t channel,
                                                uint16_t periodHigh,
                                                uint16_t periodLow)
{
    LPIT_Type * base;
    uint32_t period;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    base = s_lpitBase[instance];

    period = ((uint32_t)periodHigh << 16U) | periodLow;
    /* Sets the timer channel period in count unit */
    LPIT_SetTimerPeriodByCount(base, channel, period);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetTimerPeriodByCount
 * Description   : Gets the current timer channel period in count unit.
 *
 * Implements    : LPIT_DRV_GetTimerPeriodByCount_Activity
 *END**************************************************************************/
uint32_t LPIT_DRV_GetTimerPeriodByCount(uint32_t instance,
                                        uint32_t channel)
{
    const LPIT_Type * base;
    lpit_timer_modes_t timerMode;
    uint32_t currentPeriod;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    base = s_lpitBase[instance];
    /* Gets current timer channel period by count.*/
    currentPeriod = LPIT_GetTimerPeriodByCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        /* Calculates the period for dual 16 bit periodic counter mode */
        currentPeriod = (currentPeriod >> 16U) + (currentPeriod & MAX_PERIOD_COUNT_16_BIT);
    }
    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetCurrentTimerCount
 * Description   : Gets the current timer channel counting value in count.
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * Implements    : LPIT_DRV_GetCurrentTimerCount_Activity
 *END**************************************************************************/
uint32_t LPIT_DRV_GetCurrentTimerCount(uint32_t instance,
                                       uint32_t channel)
{
    const LPIT_Type * base;
    lpit_timer_modes_t timerMode;
    uint32_t currentTime;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);

    base = s_lpitBase[instance];
    /* Gets current timer channel counting value */
    currentTime = LPIT_GetCurrentTimerCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        /* Calculates the current counting value for dual 16 bit periodic counter mode */
        currentTime = (currentTime >> 16U) + (currentTime & MAX_PERIOD_COUNT_16_BIT);
    }
    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_EnableTimerChannelInterrupt
 * Description   : This function allows enabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * Implements    : LPIT_DRV_EnableTimerChannelInterrupt_Activity
 *END**************************************************************************/
void LPIT_DRV_EnableTimerChannelInterrupt(uint32_t instance,
                                          uint32_t mask)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << LPIT_TMR_COUNT));

    base = s_lpitBase[instance];
    /* Enable interrupt of timer channels */
    LPIT_EnableInterruptTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_DisableTimerChannelInterrupt
 * Description   : This function allows disabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * Implements    : LPIT_DRV_DisableTimerChannelInterrupt_Activity
 *END**************************************************************************/
void LPIT_DRV_DisableTimerChannelInterrupt(uint32_t instance,
                                           uint32_t mask)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << LPIT_TMR_COUNT));

    base = s_lpitBase[instance];
    /* Disable interrupt of timer channels */
    LPIT_DisableInterruptTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetInterruptFlagTimerChannels
 * Description   : Gets the current interrupt flag of timer channels.
 * In compare modes, the flag sets to 1 at the end of the timer period.
 * In capture modes, the flag sets to 1 when the trigger asserts.
 *
 * Implements    : LPIT_DRV_GetInterruptFlagTimerChannels_Activity
 *END**************************************************************************/
uint32_t LPIT_DRV_GetInterruptFlagTimerChannels(uint32_t instance,
                                                uint32_t mask)
{
    const LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << LPIT_TMR_COUNT));

    base = s_lpitBase[instance];
    /* Gets the interrupt flag for timer channels */
    return LPIT_GetInterruptFlagTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_ClearInterruptFlagTimerChannels
 * Description   : Clears the interrupt flag of timer channels.
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 * Implements    : LPIT_DRV_ClearInterruptFlagTimerChannels_Activity
 *END**************************************************************************/
void LPIT_DRV_ClearInterruptFlagTimerChannels(uint32_t instance,
                                              uint32_t mask)
{
    LPIT_Type * base;

    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << LPIT_TMR_COUNT));

    base = s_lpitBase[instance];
    /* Clears the interrupt flag for timer channels */
    LPIT_ClearInterruptFlagTimerChannels(base, mask);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
