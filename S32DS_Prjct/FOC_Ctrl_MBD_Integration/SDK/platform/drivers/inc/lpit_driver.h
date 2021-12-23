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
 * @file lpit_driver.h
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros define the maximum of timer period in some modes and might be used by user.
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
 */

#ifndef LPIT_DRIVER_H
#define LPIT_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "status.h"

/*!
 * @addtogroup lpit_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max period in count of all operation mode except for dual 16 bit periodic counter mode */
#define MAX_PERIOD_COUNT                    (0xFFFFFFFFU)
/*! @brief Max period in count of dual 16 bit periodic counter mode                               */
#define MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE (0x1FFFEU)
/*! @brief Max count of 16 bit                                */
#define MAX_PERIOD_COUNT_16_BIT (0xFFFFU)

/*!
 * @brief Mode options available for the LPIT timer
 * Implements : lpit_timer_modes_t_Class
 */
typedef enum
{
    LPIT_PERIODIC_COUNTER      = 0x00U,  /*!< 32-bit Periodic Counter        */
    LPIT_DUAL_PERIODIC_COUNTER = 0x01U,  /*!< Dual 16-bit Periodic Counter   */
    LPIT_TRIGGER_ACCUMULATOR   = 0x02U,  /*!< 32-bit Trigger Accumulator     */
    LPIT_INPUT_CAPTURE         = 0x03U   /*!< 32-bit Trigger Input Capture   */
} lpit_timer_modes_t;

/*!
 * @brief Trigger source options.
 *
 * This is used for both internal and external trigger sources. The actual trigger
 * options available is SoC specific, user should refer to the reference manual.
 * Implements : lpit_trigger_source_t_Class
 */
typedef enum
{
    LPIT_TRIGGER_SOURCE_EXTERNAL = 0x00U, /*!< Use external trigger  */
    LPIT_TRIGGER_SOURCE_INTERNAL = 0x01U  /*!< Use internal trigger  */
}  lpit_trigger_source_t;

/*!
 * @brief Unit options for LPIT period.
 *
 * This is used to determine unit of timer period
 * Implements : lpit_period_units_t_Class
 */
typedef enum
{
    LPIT_PERIOD_UNITS_COUNTS        = 0x00U, /*!< Period value unit is count */
    LPIT_PERIOD_UNITS_MICROSECONDS  = 0x01U  /*!< Period value unit is microsecond */
} lpit_period_units_t;

/*!
 * @brief LPIT configuration structure
 *
 * This structure holds the configuration settings for the LPIT peripheral to
 * enable or disable LPIT module in DEBUG and DOZE mode
 * Implements : lpit_user_config_t_Class
 */
typedef struct
{
    bool enableRunInDebug; /*!< True: Timer channels continue to run in debug mode
                                False: Timer channels stop in debug mode            */
    bool enableRunInDoze;  /*!< True: Timer channels continue to run in doze mode
                                False: Timer channels stop in doze mode             */
} lpit_user_config_t;

/*! @brief Structure to configure the channel timer
 *
 * This structure holds the configuration settings for the LPIT timer channel
 * Implements : lpit_user_channel_config_t_Class
 */
typedef struct
{
    lpit_timer_modes_t timerMode;        /*!< Operation mode of timer channel                               */
    lpit_period_units_t periodUnits;     /*!< Timer period value units                                      */
    uint32_t period;                     /*!< Period of timer channel                                       */
    lpit_trigger_source_t triggerSource; /*!< Selects between internal and external trigger sources         */
    uint32_t triggerSelect;              /*!< Selects one trigger from the internal trigger sources
                                              this field makes sense if trigger source is internal          */
    bool enableReloadOnTrigger;          /*!< True: Timer channel will reload on selected trigger
                                              False: Timer channel will not reload on selected trigger      */
    bool enableStopOnInterrupt;          /*!< True: Timer will stop after timeout
                                              False: Timer channel does not stop after timeout              */
    bool enableStartOnTrigger;           /*!< True: Timer channel starts to decrement when rising edge
                                              on selected trigger is detected.
                                              False: Timer starts to decrement immediately based on
                                              restart condition                                             */
    bool chainChannel;                   /*!< Channel chaining enable                                       */
    bool isInterruptEnabled;             /*!< Timer channel interrupt generation enable                     */
} lpit_user_channel_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and De-initialization
 * @{
 */

/*!
 * @brief Gets the default LPIT configuration
 *
 * This function gets default LPIT module configuration structure, with the following settings:
 * - PIT runs in debug mode: Disable
 * - PIT runs in doze mode: Disable
 *
 * @param[out] config The configuration structure
 */
void LPIT_DRV_GetDefaultConfig(lpit_user_config_t * const config);

/*!
 * @brief Gets the default timer channel configuration
 *
 * This function gets the default timer channel configuration structure, with the following settings:
 * - Timer mode: 32-bit Periodic Counter
 * - Period unit: Period value unit is microsecond
 * - Period: 1000000 microseconds(1 second)
 * - Trigger sources: External trigger
 * - Trigger select: Trigger from channel 0
 * - Reload on trigger: Disable
 * - Stop on interrupt : Disable
 * - Start on trigger: Disable
 * - Channel chaining: Disable
 * - Interrupt generating: Enable
 *
 * @param[out] config The channel configuration structure
 */
void LPIT_DRV_GetDefaultChanConfig(lpit_user_channel_config_t * const config);

/*!
 * @brief Initializes the LPIT module.
 *
 * This function resets LPIT module, enables the LPIT module, configures LPIT
 * module operation in Debug and DOZE mode. The LPIT configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other LPIT driver function.
 *
 * This is an example demonstrating how to define a LPIT configuration structure:
   @code
   lpit_user_config_t lpitInit =
   {
        .enableRunInDebug = false,
        .enableRunInDoze = true
   };
   @endcode
 *
 * @param[in] instance LPIT module instance number.
 * @param[in] userConfig Pointer to LPIT configuration structure.
 */
void LPIT_DRV_Init(uint32_t instance,
                   const lpit_user_config_t * userConfig);

/*!
 * @brief De-Initializes the LPIT module.
 *
 * This function disables LPIT module.
 * In order to use the LPIT module again, LPIT_DRV_Init must be called.
 *
 * @param[in] instance LPIT module instance number
 */
void LPIT_DRV_Deinit(uint32_t instance);

/*!
 * @brief Initializes the LPIT channel.
 *
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
 * This is an example demonstrating how to define a LPIT channel configuration structure:
   @code
   lpit_user_channel_config_t lpitTestInit =
   {
    .timerMode = LPIT_PERIODIC_COUNTER,
    .periodUnits = LPTT_PERIOD_UNITS_MICROSECONDS,
    .period = 1000000U,
    .triggerSource = LPIT_TRIGGER_SOURCE_INTERNAL,
    .triggerSelect = 1U,
    .enableReloadOnTrigger = false,
    .enableStopOnInterrupt = false,
    .enableStartOnTrigger = false,
    .chainChannel = false,
    .isInterruptEnabled = true
   };
   @endcode
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to LPIT channel configuration structure
 * @return Operation status
 *         - STATUS_SUCCESS: Operation was successful.
 *         - STATUS_ERROR: The channel 0 is chained.
 *         - STATUS_ERROR: The input period is invalid.
 */
status_t LPIT_DRV_InitChannel(uint32_t instance,
                              uint32_t channel,
                              const lpit_user_channel_config_t * userChannelConfig);

/* @} */

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask Timer channels starting mask that decides which channels
 * will be started
 * - For example:
 *      - with mask = 0x01U then channel 0 will be started
 *      - with mask = 0x02U then channel 1 will be started
 *      - with mask = 0x03U then channel 0 and channel 1 will be started
 */
void LPIT_DRV_StartTimerChannels(uint32_t instance,
                                 uint32_t mask);

/*!
 * @brief Stops the timer channel counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the LPIT_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask Timer channels stopping mask that decides which channels
 * will be stopped
 * - For example:
 *      - with mask = 0x01U then channel 0 will be stopped
 *      - with mask = 0x02U then channel 1 will be stopped
 *      - with mask = 0x03U then channel 0 and channel 1 will be stopped
 */
void LPIT_DRV_StopTimerChannels(uint32_t instance,
                                uint32_t mask);

/* @} */

/*!
 * @name Timer Period
 * @{
 */

/*!
 * @brief Sets the timer channel period in microseconds.
 *
 * This function sets the timer channel period in microseconds
 * when timer channel mode is 32 bit periodic or dual 16 bit counter mode.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] periodUs Timer channel period in microseconds
 * @return Operation status
 *         - STATUS_SUCCESS: Input period of timer channel is valid.
 *         - STATUS_ERROR: Input period of timer channel is invalid.
 */
status_t LPIT_DRV_SetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel,
                                     uint32_t periodUs);

/*!
 * @brief Sets the timer channel period in microseconds.
 *
 * This function sets the timer channel period in microseconds
 * when timer channel mode is dual 16 bit periodic counter mode.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] periodHigh Period of higher 16 bit in microseconds
 * @param[in] periodLow Period of lower 16 bit in microseconds
 * @return Operation status
 *         - STATUS_SUCCESS: Input period of timer channel is valid.
 *         - STATUS_ERROR: Input period of timer channel is invalid.
 */
status_t LPIT_DRV_SetTimerPeriodInDual16ModeByUs(uint32_t instance,
                                                 uint32_t channel,
                                                 uint16_t periodHigh,
                                                 uint16_t periodLow);

/*!
 * @brief Gets the timer channel period in microseconds.
 *
 * This function gets the timer channel period in microseconds.
 * The returned period here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Timer channel period in microseconds
 */
uint64_t LPIT_DRV_GetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel);

/*!
 * @brief Gets the current timer channel counting value in microseconds.
 *
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time.
 * The return counting value here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter or 32-bit trigger input capture.
 * Need to make sure the running time will not exceed the timer channel period.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value in microseconds
 */
uint64_t LPIT_DRV_GetCurrentTimerUs(uint32_t instance,
                                    uint32_t channel);

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * This function sets the timer channel period in count unit.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 */
void LPIT_DRV_SetTimerPeriodByCount(uint32_t instance,
                                    uint32_t channel,
                                    uint32_t count);

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * This function sets the timer channel period in count unit when timer channel
 * mode is dual 16 periodic counter mode.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] periodHigh Period of higher 16 bit in count unit
 * @param[in] periodLow Period of lower 16 bit in count unit
 */
void LPIT_DRV_SetTimerPeriodInDual16ModeByCount(uint32_t instance,
                                                uint32_t channel,
                                                uint16_t periodHigh,
                                                uint16_t periodLow);

/*!
 * @brief Gets the current timer channel period in count unit.
 *
 * This function returns current period of timer channel given as argument.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Timer channel period in count unit
 */
uint32_t LPIT_DRV_GetTimerPeriodByCount(uint32_t instance,
                                        uint32_t channel);

/*!
 * @brief Gets the current timer channel counting value in count.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value in count
 */
uint32_t LPIT_DRV_GetCurrentTimerCount(uint32_t instance,
                                       uint32_t channel);
/* @} */

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Enables the interrupt generation of timer channel.
 *
 * This function allows enabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * @param[in] instance LPIT module instance number.
 * @param[in] mask The mask that decides which channels will be enabled interrupt.
 * - For example:
 *      - with mask = 0x01u then the interrupt of channel 0 will be enabled
 *      - with mask = 0x02u then the interrupt of channel 1 will be enabled
 *      - with mask = 0x03u then the interrupt of channel 0 and channel 1 will be enabled
 */
void LPIT_DRV_EnableTimerChannelInterrupt(uint32_t instance,
                                          uint32_t mask);

/*!
 * @brief Disables the interrupt generation of timer channel.
 *
 * This function allows disabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask The mask that decides which channels will be disable interrupt.
 * - For example:
 *      - with mask = 0x01u then the interrupt of channel 0 will be disable
 *      - with mask = 0x02u then the interrupt of channel 1 will be disable
 *      - with mask = 0x03u then the interrupt of channel 0 and channel 1 will be disable
 */
void LPIT_DRV_DisableTimerChannelInterrupt(uint32_t instance,
                                           uint32_t mask);

/*!
 * @brief Gets the current interrupt flag of timer channels.
 *
 * This function gets the current interrupt flag of timer channels.
 * In compare modes, the flag sets to 1 at the end of the timer period.
 * In capture modes, the flag sets to 1 when the trigger asserts.
 *
 * @param[in] instance LPIT module instance number.
 * @param[in] mask The interrupt flag getting mask that decides which channels will
 * be got interrupt flag.
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be got
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be got
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be got
 * @return Current the interrupt flag of timer channels
 */
uint32_t LPIT_DRV_GetInterruptFlagTimerChannels(uint32_t instance,
                                                uint32_t mask);

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask The interrupt flag clearing mask that decides which channels will
 * be cleared interrupt flag
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be cleared
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be cleared
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be cleared
 */
void LPIT_DRV_ClearInterruptFlagTimerChannels(uint32_t instance,
                                              uint32_t mask);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* LPIT_DRIVER_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
