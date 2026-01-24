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
 * @file ftm_pwm_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macro is used in ftm_pwm driver, but in pwm_pal the functions which use it is not called.
 *
 */

#ifndef FTM_PWM_DRIVER_H
#define FTM_PWM_DRIVER_H

#include "ftm_common.h"

/*!
 * @addtogroup ftm_pwm_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Maximum value for PWM duty cycle */
#define FTM_MAX_DUTY_CYCLE      (0x8000U)
/*! @brief Shift value which converts duty to ticks */
#define FTM_DUTY_TO_TICKS_SHIFT (15U)

/*!
 * @brief FlexTimer Configure type of PWM update in the duty cycle or in ticks
 *
 * Implements : ftm_pwm_update_option_t_Class
 */
typedef enum
{
    FTM_PWM_UPDATE_IN_DUTY_CYCLE = 0x00U,     /*!< The type of PWM update in the duty cycle/pulse or also use in frequency update */
    FTM_PWM_UPDATE_IN_TICKS      = 0x01U      /*!< The type of PWM update in ticks */
} ftm_pwm_update_option_t;

/*!
 * @brief The polarity of the channel output is configured in PWM signal
 *
 * Implements : ftm_polarity_t_Class
 */
typedef enum
{
    FTM_POLARITY_LOW  = 0x00U,  /*!< The channel polarity is active LOW which is defined again */
    FTM_POLARITY_HIGH = 0x01U   /*!< The channel polarity is active HIGH which is defined again */
} ftm_polarity_t;

/*!
 * @brief FlexTimer PWM channel (n+1) polarity for combine mode
 *
 * Implements : ftm_second_channel_polarity_t_Class
 */
typedef enum
{
    FTM_MAIN_INVERTED   = 0x01U,  /*!< The channel (n+1) output is the inverse of the
                                   *   channel (n) output  */
    FTM_MAIN_DUPLICATED = 0x00U   /*!< The channel (n+1) output is the same as the
                                   *   channel (n) output */
} ftm_second_channel_polarity_t;

/*!
 * @brief FlexTimer fault control
 *
 * Implements : ftm_fault_mode_t_Class
 */
typedef enum
{
    FTM_FAULT_CONTROL_DISABLED  = 0x00U,    /*!< Fault control is disabled for all channels */
    FTM_FAULT_CONTROL_MAN_EVEN  = 0x01U,    /*!< Fault control is enabled for even channels
                                             *   only (channels 0, 2, 4, and 6), and the selected
                                             *   mode is the manual fault clearing */
    FTM_FAULT_CONTROL_MAN_ALL   = 0x02U,    /*!< Fault control is enabled for all channels,
                                             *   and the selected mode is the manual fault clearing */
    FTM_FAULT_CONTROL_AUTO_ALL  = 0x03U     /*!< Fault control is enabled for all channels, and
                                             *   the selected mode is the automatic fault clearing */
} ftm_fault_mode_t;

/*!
 * @brief Select level of the channel (n) output at the beginning
 *
 * Implements : ftm_safe_state_polarity_t_Class
 */
typedef enum
{
    FTM_LOW_STATE = 0x00U,    /*!< When fault is detected PWM channel is low. */
    FTM_HIGH_STATE  = 0x01U   /*!< When fault is detected PWM channel is high. */
} ftm_safe_state_polarity_t;

/*!
 * @brief FlexTimer driver PWM Fault channel parameters
 *
 * Implements : ftm_pwm_ch_fault_param_t_Class
 */
typedef struct
{
    bool faultChannelEnabled;                   /*!< Fault channel state */
    bool faultFilterEnabled;                    /*!< Fault channel filter state */
    ftm_polarity_t ftmFaultPinPolarity;         /*!< Channel output state on fault */
} ftm_pwm_ch_fault_param_t;

/*!
 * @brief FlexTimer driver PWM Fault parameter
 *
 * Implements : ftm_pwm_fault_param_t_Class
 */
typedef struct
{
    bool pwmOutputStateOnFault;             /*!< Output pin state on fault (safe state or tri-state) */
    bool pwmFaultInterrupt;                 /*!< PWM fault interrupt state */
    uint8_t faultFilterValue;               /*!< Fault filter value */
    ftm_fault_mode_t faultMode;             /*!< Fault mode */
    ftm_pwm_ch_fault_param_t ftmFaultChannelParam[FTM_FEATURE_FAULT_CHANNELS]; /*!< Fault channels configuration */
} ftm_pwm_fault_param_t;

/*!
 * @brief FlexTimer driver independent PWM parameter
 *
 * Implements : ftm_independent_ch_param_t_Class
 */
typedef struct
{
    uint8_t hwChannelId;                                 /*!< Physical hardware channel ID */
    ftm_polarity_t polarity;                             /*!< Polarity of the PWM signal generated on MCU pin.*/
    uint16_t uDutyCyclePercent;                          /*!< PWM pulse width, value should be between
                                                          *   0 (0%) to FTM_MAX_DUTY_CYCLE (100%) */
    bool enableExternalTrigger;                          /*!< true: enable the generation of a trigger is used for on-chip modules
                                                          *   false: disable the generation of a trigger */
    ftm_safe_state_polarity_t safeState;                 /*!< Logical state of the PWM channel n when an fault is detected
                                                          *   and to set up the polarity of PWM signal on the channel (n+1) */
    bool enableSecondChannelOutput;                      /*!< Enable complementary mode on next channel */
    ftm_second_channel_polarity_t secondChannelPolarity; /*!< Polarity of the channel n+1 relative to channel n in the complementary mode*/
    bool deadTime;                                       /*!< Enable/disable dead time for channel */
} ftm_independent_ch_param_t;

/*!
 * @brief FlexTimer driver combined PWM parameter
 *
 * Implements : ftm_combined_ch_param_t_Class

 */
typedef struct
{
    uint8_t hwChannelId;                                 /*!< Physical hardware channel ID for channel (n) */
    uint16_t firstEdge;                                  /*!< First edge time. This time is relative to signal period. The value for this parameter is
                                                          *   between 0 and FTM_MAX_DUTY_CYCLE(0 = 0% from period and FTM_MAX_DUTY_CYCLE = 100% from period) */
    uint16_t secondEdge;                                 /*!< Second edge time. This time is relative to signal period. The value for this parameter is
                                                          *   between 0 and FTM_MAX_DUTY_CYCLE(0 = 0% from period and FTM_MAX_DUTY_CYCLE = 100% from period) */
    bool deadTime;                                       /*!< Enable/disable dead time for channel */
    bool enableModifiedCombine;                          /*!< Enable/disable the modified combine mode for channels (n) and (n+1) */
    ftm_polarity_t mainChannelPolarity;                  /*!< Polarity of the PWM signal generated on MCU pin for channel n.*/
    bool enableSecondChannelOutput;                      /*!< Select if channel (n+1)  output is enabled/disabled for the complementary mode */
    ftm_second_channel_polarity_t secondChannelPolarity; /*!< Select channel (n+1) polarity relative to channel (n) in the complementary mode */
    bool enableExternalTrigger;                          /*!< The generation of the channel (n) trigger
                                                          *   true: enable the generation of a trigger on the channel (n)
                                                          *   false: disable the generation of a trigger on the channel (n) */
    bool enableExternalTriggerOnNextChn;                 /*!< The generation of the channel (n+1) trigger
                                                          *   true: enable the generation of a trigger on the channel (n+1)
                                                          *   false: disable the generation of a trigger on the channel (n+1) */
    ftm_safe_state_polarity_t mainChannelSafeState;      /*!< The selection of the channel (n) state when fault is detected */
    ftm_safe_state_polarity_t secondChannelSafeState;    /*!< The selection of the channel (n+1) state when fault is detected
                                                          *   and set up the polarity of PWM signal on the channel (n+1) */
} ftm_combined_ch_param_t;

/*!
 * @brief FlexTimer driver PWM parameters
 *
 * Implements : ftm_pwm_param_t_Class
 */
typedef struct
{
    uint8_t nNumIndependentPwmChannels;                                     /*!< Number of independent PWM channels */
    uint8_t nNumCombinedPwmChannels;                                        /*!< Number of combined PWM channels */
    ftm_config_mode_t mode;                                                 /*!< FTM mode */
    uint8_t deadTimeValue;                                                  /*!< Dead time value in [ticks] */
    ftm_deadtime_ps_t deadTimePrescaler;                                    /*!< Dead time pre-scaler value[ticks] */
    uint32_t uFrequencyHZ;                                                  /*!< PWM period in Hz */
    ftm_independent_ch_param_t * pwmIndependentChannelConfig;               /*!< Configuration for independent PWM channels */
    ftm_combined_ch_param_t * pwmCombinedChannelConfig;                     /*!< Configuration for combined PWM channels */
    ftm_pwm_fault_param_t * faultConfig;                                    /*!< Configuration for PWM fault */
} ftm_pwm_param_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Stops all PWM channels .
 *
 * @param [in] instance The FTM peripheral instance number.
 * @return counter the current counter value
 */
status_t FTM_DRV_DeinitPwm(uint32_t instance);

/*!
 * @brief Configures the duty cycle and frequency and starts the output of the PWM on
 * all channels configured in the param structure.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] param FTM driver PWM parameter to configure PWM options.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_InitPwm(uint32_t instance,
                         const ftm_pwm_param_t * param);

/*!
 * @brief This function updates the waveform output in PWM mode (duty cycle and phase).
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channel The channel number. In combined mode, the code finds the channel.
 * @param [in] typeOfUpdate The type of PWM update in the duty cycle/pulse or in ticks.
 * @param [in] firstEdge  Duty cycle or first edge time for PWM mode. Can take value between
 *                       0 - FTM_MAX_DUTY_CYCLE(0 = 0% from period  and FTM_MAX_DUTY_CYCLE = 100% from period)
 *                       Or value in ticks for the first of the PWM mode in which can have value between 0
 *                       and ftmPeriod is stored in the state structure.
 * @param [in] secondEdge Second edge time - only for combined mode. Can take value
 *                       between 0 - FTM_MAX_DUTY_CYCLE(0 = 0% from period  and FTM_MAX_DUTY_CYCLE = 100% from period).
 *                       Or value in ticks for the second of the PWM mode in which can have value between 0
 *                       and ftmPeriod is stored in the state structure.
 * @param [in] softwareTrigger If true a software trigger is generate to update PWM parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_UpdatePwmChannel(uint32_t instance,
                                  uint8_t channel,
                                  ftm_pwm_update_option_t typeOfUpdate,
                                  uint16_t firstEdge,
                                  uint16_t secondEdge,
                                  bool softwareTrigger);

/*!
 * @brief This function will update the duty cycle of PWM output for multiple channels.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] numberOfChannels The number of channels which should be updated.
 * @param [in] channels The list of channels which should be updated.
 * @param [in] duty The list of duty cycles for selected channels.
 * @param [in] softwareTrigger If true a software trigger is generate to update PWM parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_FastUpdatePwmChannels(uint32_t instance,
                                       uint8_t numberOfChannels,
                                       const uint8_t * channels,
                                       const uint16_t * duty,
                                       bool softwareTrigger);

/*!
 * @brief This function will update the new period in the frequency or
 * in the counter value into mode register which modify the period of PWM signal
 * on the channel output
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] typeOfUpdate The type of PWM update is a period in Hz or in ticks.
 *                          - For FTM_PWM_UPDATE_IN_DUTY_CYCLE which reuse in FTM_DRV_UpdatePwmChannel function will update in Hz.
 *                          - For FTM_PWM_UPDATE_IN_TICKS will update in ticks.
 * @param [in] newValue The frequency or the counter value which will select with modified value for PWM signal.
 *                      If the type of update in the duty cycle, the newValue parameter must be value
 *                      between 1U and maximum is the frequency of the FTM counter.
 *                      If the type of update in ticks, the newValue parameter must be value between 1U and 0xFFFFU.
 * @param [in] softwareTrigger If true a software trigger is generate to update PWM parameters.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_UpdatePwmPeriod(uint32_t instance,
                                 ftm_pwm_update_option_t typeOfUpdate,
                                 uint32_t newValue,
                                 bool softwareTrigger);

/*!
 * @brief This function is used to control the final logic of the channel output.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channel The channel which is used in PWM mode.
 * @param [in] enableChannelOutput Enable/disable the channel output.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 */
status_t FTM_DRV_ControlChannelOutput(uint32_t instance,
                                      uint8_t channel,
                                      bool enableChannelOutput);

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief This function will use in the PWM period dithering. This value
 * is added to an internal accumulator at the end of each PWM period. The value is
 * updated with its write buffer value according to the register synchronization.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] newModFracVal The modulo fractional value.
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_UpdatePwmPeriodDither(uint32_t instance,
                                       uint8_t newModFracVal,
                                       bool softwareTrigger);

/*!
 * @brief This function will use in the PWM edge dithering. This value
 * is added to the channel (n) internal accumulator at the end of each PWM period.
 * The FRACVAL is updated with its write buffer value according to the register
 * synchronization. The PWM edge dithering is not available when the channel in the
 * input capture modes, and the channel in output compare modes.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channel The channel number.
 * @param [in] newMatchFracVal The channel (n) match fractional value .
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_UpdatePwmEdgeChannelDither(uint32_t instance,
                                            uint8_t channel,
                                            uint8_t newMatchFracVal,
                                            bool softwareTrigger);
#endif

#ifdef ERRATA_E10856
/*!
 * @brief This function is used to workaround an errata which the safe state
 * is not removed from channel outputs after fault condition ends if SWOCTRL is being
 * used to control the pin. The FTM_MODE[FAULTM] should be configured for manual fault
 * clearing (0b10).
 * This function must be used in the TOF interrupt handler when a fault is detected to
 * ensure that the outputs return to the value configured by FTM_SWOCTR register.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] chnOutCtrlVal The value of some channel outputs are affected by software output control.
 */
void FTM_PWM_DRV_IrqHandler(uint32_t instance,
                            uint32_t chnOutCtrlVal);

#endif /* ERRATA_E10856 */

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup ftm_pwm_driver */

#endif /* FTM_PWM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
