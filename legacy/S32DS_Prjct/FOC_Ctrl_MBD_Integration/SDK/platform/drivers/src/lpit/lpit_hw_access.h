/*
 * Copyright 2017 NXP
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

#ifndef LPIT_HW_ACCESS_H
#define LPIT_HW_ACCESS_H

#include <stdbool.h>
#include "device_registers.h"
#include "lpit_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enables the LPIT module.
 *
 * This function enables the functional clock of LPIT module (Note: this function
 * does not un-gate the system clock gating control). It should be called before
 * setup any timer channel.
 *
 * @param[in] base LPIT peripheral base address
 */
static inline void LPIT_Enable(LPIT_Type * const base, volatile uint32_t delay)
{
    base->MCR |= LPIT_MCR_M_CEN_MASK;
    /* Run this counter down to zero
        If the delay is 0, the four clock delay between setting and clearing
        the SW_RST bit is ensured by the read-modify-write operation.
    */
    while(delay != 0u)
    {
        /* Since we need a four cycle delay, we assume the decrement is one cycle
            and insert three NOP instructions. The actual delay will be larger because
            of the loop overhead and the compiler optimization.
        */
        delay--;
        NOP();
        NOP();
        NOP();
    }
}

/*!
 * @brief Disables the LPIT module.
 *
 * This function disables functional clock of LPIT module (Note: it does not
 * affect the system clock gating control).
 *
 * @param[in] base LPIT peripheral base address
 */
static inline void LPIT_Disable(LPIT_Type * const base)
{
    base->MCR &= ~LPIT_MCR_M_CEN_MASK;
}

/*!
 * @brief Resets the LPIT module.
 *
 * This function sets all LPIT registers to reset value,
 * except the Module Control Register.
 *
 * @param[in] base LPIT peripheral base address
 */
static inline void LPIT_Reset(LPIT_Type * const base, volatile uint32_t delay)
{
    base->MCR |= LPIT_MCR_SW_RST_MASK;
    /* Run this counter down to zero
        If the delay is 0, the four clock delay between setting and clearing
        the SW_RST bit is ensured by the read-modify-write operation.
    */
    while(delay != 0u)
    {
        /* Since we need a four cycle delay, we assume the decrement is one cycle
            and insert three NOP instructions. The actual delay will be larger because
            of the loop overhead and the compiler optimization.
        */
        delay--;
        NOP();
        NOP();
        NOP();
    }
    base->MCR &= ~LPIT_MCR_SW_RST_MASK;
}

/*!
  @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask Timer channels starting mask that decides which channels
 * will be started
 * - For example:
 *      - with mask = 0x01U then channel 0 will be started
 *      - with mask = 0x02U then channel 1 will be started
 *      - with mask = 0x03U then channel 0 and channel 1 will be started
 */
static inline void LPIT_StartTimerChannels(LPIT_Type * const base,
                                           uint32_t mask)
{
    base->SETTEN |= mask;
}

/*!
 * @brief Stops the timer channel from counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the LPIT_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask Timer channels stopping mask that decides which channels
 * will be stopped
 * - For example:
 *      - with mask = 0x01U then channel 0 will be stopped
 *      - with mask = 0x02U then channel 1 will be stopped
 *      - with mask = 0x03U then channel 0 and channel 1 will be stopped
 */
static inline void LPIT_StopTimerChannels(LPIT_Type * const base,
                                          uint32_t mask)
{
    base->CLRTEN |= mask;
}

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * This function sets the timer channel period in count unit.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * Timer channel begins counting from the value that is set by this function.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 */
static inline void LPIT_SetTimerPeriodByCount(LPIT_Type * const base,
                                              uint32_t channel,
                                              uint32_t count)
{
    base->TMR[channel].TVAL = count;
}

/*!
 * @brief Gets the timer channel period in count unit.
 *
 * This function returns current period of timer channel given as argument.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return Timer channel period in count unit
 */
static inline uint32_t LPIT_GetTimerPeriodByCount(const LPIT_Type * base,
                                                  uint32_t channel)
{
    return (base->TMR[channel].TVAL);
}

/*!
 * @brief Gets the current timer channel counting value.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value
 */
static inline uint32_t LPIT_GetCurrentTimerCount(const LPIT_Type * base,
                                                 uint32_t channel)
{
    return (base->TMR[channel].CVAL);
}

/*!
 * @brief Enables the interrupt generation for timer channels.
 *
 * This function allows enabling interrupt generation for timer channels simultaneously.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt enabling mask that decides which channels will
 * be enabled interrupt.
 * - For example:
 *      - with mask = 0x01u then will enable interrupt for channel 0 only
 *      - with mask = 0x02u then will enable interrupt for channel 1 only
 *      - with mask = 0x03u then will enable interrupt for channel 0 and channel 1
 */
static inline void LPIT_EnableInterruptTimerChannels(LPIT_Type * const base,
                                                     uint32_t mask)
{
    base->MIER |= mask;
}

/*!
 * @brief Disables the interrupt generation for timer channels.
 *
 * This function allows disabling interrupt generation for timer channels simultaneously.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt disabling mask that decides which channels will
 * be disabled interrupt.
 * - For example:
 *      - with mask = 0x01u then will disable interrupt for channel 0 only
 *      - with mask = 0x02u then will disable interrupt for channel 1 only
 *      - with mask = 0x03u then will disable interrupt for channel 0 and channel 1
 */
static inline void LPIT_DisableInterruptTimerChannels(LPIT_Type * const base,
                                                      uint32_t mask)
{
    base->MIER &= ~mask;
}

/*!
 * @brief Gets the interrupt flag of timer channels.
 *
 * This function gets current interrupt flag of timer channels.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt flag getting mask that decides which channels will
 * be got interrupt flag.
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be got
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be got
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be got
 * @return The interrupt flag of timer channels.
 */
static inline uint32_t LPIT_GetInterruptFlagTimerChannels(const LPIT_Type * base,
                                                          uint32_t mask)
{
    return (base->MSR) & mask;
}

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears current interrupt flag of timer channels.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt flag clearing mask that decides which channels will
 * be cleared interrupt flag.
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be cleared
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be cleared
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be cleared
 */
static inline void LPIT_ClearInterruptFlagTimerChannels(LPIT_Type * const base,
                                                        uint32_t mask)
{
    /* Write 1 to clear the interrupt flag. */
    base->MSR = mask;
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    base->MSR;
#endif
}

/*!
 * @brief Sets operation mode of timer channel
 *
 * This function sets the timer channel operation mode which control how
 * the timer channel decrements.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] mode Operation mode of timer channel that is member of lpit_timer_modes_t
 */
static inline void LPIT_SetTimerChannelModeCmd(LPIT_Type * const base,
                                               uint32_t channel,
                                               lpit_timer_modes_t mode)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_MODE_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_MODE(mode);
}

/*!
 * @brief Gets current operation mode of timer channel.
 *
 * This function gets current operation mode of the timer channel given as argument.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return Operation mode of timer channel that is one of lpit_timer_modes_t
 */
static inline lpit_timer_modes_t LPIT_GetTimerChannelModeCmd(const LPIT_Type * base,
                                                             uint32_t channel)
{
    uint32_t tmp;
    lpit_timer_modes_t mode;

    tmp = (((base->TMR[channel].TCTRL) & LPIT_TMR_TCTRL_MODE_MASK)
                                 >> LPIT_TMR_TCTRL_MODE_SHIFT);
    switch (tmp)
    {
        case 0x00U:
            mode = LPIT_PERIODIC_COUNTER;
            break;
        case 0x01U:
            mode = LPIT_DUAL_PERIODIC_COUNTER;
            break;
        case 0x02U:
            mode = LPIT_TRIGGER_ACCUMULATOR;
            break;
        case 0x03U:
            mode = LPIT_INPUT_CAPTURE;
            break;
        default:
            mode = LPIT_PERIODIC_COUNTER;
            break;
    }
    return mode;
}

/*!
 * @brief Sets internal trigger source for timer channel
 *
 * This function selects one trigger from the set of internal triggers that is
 * generated by other timer channels.
 * The selected trigger is used for starting and/or reloading the timer channel.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] triggerChannelSelect Number of the channel which is selected to be trigger source
 */
static inline void LPIT_SetTriggerSelectCmd(LPIT_Type * const base,
                                            uint32_t channel,
                                            uint32_t triggerChannelSelect)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TRG_SEL_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TRG_SEL(triggerChannelSelect);
}

/*!
 * @brief Sets trigger source of timer channel.
 *
 * This function sets trigger source of the timer channel to be internal or external trigger.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] triggerSource Trigger source of timer channel(internal or external source)
 */
static inline void LPIT_SetTriggerSourceCmd(LPIT_Type * const base,
                                            uint32_t channel,
                                            lpit_trigger_source_t triggerSource)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TRG_SRC_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TRG_SRC(triggerSource);
}

/*!
 * @brief Sets timer channel reload on trigger.
 *
 * This function sets the timer channel to reload/don't reload on trigger.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isReloadOnTrigger Timer channel reload on trigger
 *        - True : timer channel will reload on trigger
 *        - False : timer channel will not reload on trigger
 */
static inline void LPIT_SetReloadOnTriggerCmd(LPIT_Type * const base,
                                              uint32_t channel,
                                              bool isReloadOnTrigger)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TROT_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TROT(isReloadOnTrigger ? 1UL : 0UL);
}

/*!
 * @brief Sets timer channel stop on interrupt.
 *
 * This function sets the timer channel to stop or don't stop after it times out.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isStopOnInterrupt Timer channel stop on interrupt
 *        - True : Timer channel will stop after it times out
 *        - False : Timer channel will not stop after it times out
 */
static inline void LPIT_SetStopOnInterruptCmd(LPIT_Type * const base,
                                              uint32_t channel,
                                              bool isStopOnInterrupt)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TSOI_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TSOI(isStopOnInterrupt ? 1UL : 0UL);
}

/*!
 * @brief Sets timer channel start on trigger.
 *
 * This function sets the timer channel to starts/don't start on trigger.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isStartOnTrigger Timer channel start on trigger
 *        - True : Timer channel starts to decrement when rising edge on selected trigger is detected
 *        - False : Timer channel starts to decrement immediately based on restart condition
 *                      (controlled by Timer Stop On Interrupt bit)
 */
static inline void LPIT_SetStartOnTriggerCmd(LPIT_Type * const base,
                                             uint32_t channel,
                                             bool isStartOnTrigger)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TSOT_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TSOT(isStartOnTrigger ? 1UL : 0UL);
}

/*!
 * @brief Sets timer channel chaining.
 *
 * This function sets the timer channel to be chained or not chained.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number(Note: The timer channel 0 cannot be chained)
 * @param[in] isChannelChained Timer channel chaining
 *        - True : Timer channel is chained. Timer channel decrements on previous channel's timeout
 *        - False : Timer channel is not chained. Timer channel runs independently
 */
static inline void LPIT_SetTimerChannelChainCmd(LPIT_Type * const base,
                                                uint32_t channel,
                                                bool isChannelChained)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_CHAIN_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_CHAIN(isChannelChained ? 1UL : 0UL);
}

/*!
 * @brief Sets operation of LPIT in debug mode.
 *
 * When the device enters debug mode, the timer channels may or may not be frozen,
 * based on the configuration of this function. This is intended to aid software development,
 * allowing the developer to halt the processor, investigate the current state of
 * the system (for example, the timer channel values), and continue the operation.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] isRunInDebug LPIT run in debug mode
 *        - True: LPIT continue to run when the device enters debug mode
 *        - False: LPIT stop when the device enters debug mode
 */
static inline void LPIT_SetTimerRunInDebugCmd(LPIT_Type * const base,
                                              bool isRunInDebug)
{
    base->MCR &= ~LPIT_MCR_DBG_EN_MASK;
    base->MCR |= LPIT_MCR_DBG_EN(isRunInDebug ? 1UL: 0UL);
}

/*!
 * @brief Sets operation of LPIT in DOZE mode.
 *
 * When the device enters debug mode, the timer channels may or may not be frozen,
 * based on the configuration of this function. The LPIT must use an external or
 * internal clock source which remains operating during DOZE modes(low power mode).
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] isRunInDoze LPIT run in DOZE mode
 *        - True: LPIT continue to run when the device enters DOZE mode
 *        - False: LPIT channels stop when the device enters DOZE mode
 */
static inline void LPIT_SetTimerRunInDozeCmd(LPIT_Type * const base,
                                             bool isRunInDoze)
{
    base->MCR &= ~LPIT_MCR_DOZE_EN_MASK;
    base->MCR |= LPIT_MCR_DOZE_EN(isRunInDoze ? 1UL : 0UL);
}

#if defined(__cplusplus)
}
#endif

#endif /* LPIT_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
