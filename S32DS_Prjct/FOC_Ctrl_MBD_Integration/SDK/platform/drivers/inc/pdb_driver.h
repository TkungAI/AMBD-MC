/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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

#ifndef PDB_DRIVER_H
#define PDB_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "clock_manager.h"

/*! @file */

/*!
 * @addtogroup pdb_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Defines the type of value load mode for the PDB module.
 *
 * Some timing related registers, such as the MOD, IDLY, CHnDLYm, INTx and POyDLY,
 * buffer the setting values. Only the load operation is triggered.
 * The setting value is loaded from a buffer and takes effect. There are
 * four loading modes to fit different applications.
 * Implements : pdb_load_value_mode_t_Class
 */
typedef enum
{
    PDB_LOAD_VAL_IMMEDIATELY                        = 0U,
        /*!<  Loaded immediately after load operation. @internal gui name="Immediately" */
    PDB_LOAD_VAL_AT_MODULO_COUNTER                  = 1U,
        /*!< Loaded when counter hits the modulo after load operation. @internal gui name="Modulo counter" */
    PDB_LOAD_VAL_AT_NEXT_TRIGGER                    = 2U,
        /*!< Loaded when detecting an input trigger after load operation. @internal gui name="Next trigger" */
    PDB_LOAD_VAL_AT_MODULO_COUNTER_OR_NEXT_TRIGGER  = 3U
        /*!< Loaded when counter hits the modulo or detecting an input trigger after load operation. @internal gui name="Modulo counter/Next trigger" */
} pdb_load_value_mode_t;

/*!
 * @brief Defines the type of prescaler divider for the PDB counter clock.
 * Implements : pdb_clk_prescaler_div_t_Class
 */
typedef enum
{
    PDB_CLK_PREDIV_BY_1   = 0U, /*!< Counting divided by multiplication factor selected by MULT. @internal gui name="1" */
    PDB_CLK_PREDIV_BY_2   = 1U, /*!< Counting divided by multiplication factor selected by 2 times ofMULT. @internal gui name="2" */
    PDB_CLK_PREDIV_BY_4   = 2U, /*!< Counting divided by multiplication factor selected by 4 times ofMULT. @internal gui name="4" */
    PDB_CLK_PREDIV_BY_8   = 3U, /*!< Counting divided by multiplication factor selected by 8 times ofMULT. @internal gui name="8" */
    PDB_CLK_PREDIV_BY_16  = 4U, /*!< Counting divided by multiplication factor selected by 16 times ofMULT. @internal gui name="16" */
    PDB_CLK_PREDIV_BY_32  = 5U, /*!< Counting divided by multiplication factor selected by 32 times ofMULT. @internal gui name="32" */
    PDB_CLK_PREDIV_BY_64  = 6U, /*!< Counting divided by multiplication factor selected by 64 times ofMULT. @internal gui name="64" */
    PDB_CLK_PREDIV_BY_128 = 7U  /*!< Counting divided by multiplication factor selected by 128 times ofMULT. @internal gui name="128" */
} pdb_clk_prescaler_div_t;

/*!
 * @brief Defines the type of trigger source mode for the PDB.
 *
 * Selects the trigger input source for the PDB. The trigger input source can
 * be internal or the software trigger.
 * Implements : pdb_trigger_src_t_Class
 */
typedef enum
{
    PDB_TRIGGER_IN0         = 0U,    /*!< Source trigger comes from TRGMUX. @internal gui name="TRGMUX trigger" */
    PDB_SOFTWARE_TRIGGER    = 15U    /*!< Select software trigger. @internal gui name="Software trigger" */
} pdb_trigger_src_t;

/*!
 * @brief Defines the type of the multiplication source mode for PDB.
 *
 * Selects the multiplication factor of the prescaler divider for the PDB counter clock.
 * Implements : pdb_clk_prescaler_mult_factor_t_Class
 */
typedef enum
{
    PDB_CLK_PREMULT_FACT_AS_1  = 0U, /*!< Multiplication factor is 1. @internal gui name="1" */
    PDB_CLK_PREMULT_FACT_AS_10 = 1U, /*!< Multiplication factor is 10. @internal gui name="10" */
    PDB_CLK_PREMULT_FACT_AS_20 = 2U, /*!< Multiplication factor is 20. @internal gui name="20" */
    PDB_CLK_PREMULT_FACT_AS_40 = 3U  /*!< Multiplication factor is 40. @internal gui name="40" */
} pdb_clk_prescaler_mult_factor_t;

/*!
 * @brief Defines the type of structure for basic timer in PDB.
 *
 * @internal gui name="Basic configuration" id="pdbCfg"
 * Implements : pdb_timer_config_t_Class
 */
typedef struct
{
    pdb_load_value_mode_t           loadValueMode;            /*!< Select the load mode. @internal gui name="Load mode" id="LoadMode" */
    bool                            seqErrIntEnable;          /*!< Enable PDB Sequence Error Interrupt. @internal gui name="Sequence error interrupt" id="SequenceErrorInterrupt" */
    pdb_clk_prescaler_div_t         clkPreDiv;                /*!< Select the prescaler divider. @internal gui name="Divider" id="Divider" */
    pdb_clk_prescaler_mult_factor_t clkPreMultFactor;         /*!< Select multiplication factor for prescaler. @internal gui name="Multiplier" id="Multiplier" */
    pdb_trigger_src_t               triggerInput;             /*!< Select the trigger input source. @internal gui name="Trigger" id="Trigger" */
    bool                            continuousModeEnable;     /*!< Enable the continuous mode. @internal gui name="Continuous mode" id="ContinuousMode" */
    bool                            dmaEnable;                /*!< Enable the dma for timer. @internal gui name="DMA" id="DMA" */
    bool                            intEnable;                /*!< Enable the interrupt for timer. @internal gui name="Interrupt" id="Interrupt" */
    bool                            instanceBackToBackEnable; /*!< Enable the back to back mode with all of the PDB units as a ring. @internal gui name="Instance Back-To-Back mode" id="InstanceBackToBackMode"*/
} pdb_timer_config_t;

/*!
 * @brief Defines the type of structure for configuring ADC's pre_trigger.
 * @internal gui name="ADC pre-trigger configuration" id="pdbAdcTrgCfg"
 * Implements : pdb_adc_pretrigger_config_t_Class
 */
typedef struct
{
    uint32_t adcPreTriggerIdx;       /*!< Setting pre_trigger's index. */
    bool preTriggerEnable;           /*!< Enable the pre_trigger. */
    bool preTriggerOutputEnable;     /*!< Enable the pre_trigger output. @internal gui name="Trigger output" id="AdcTriggerOutput" */
    bool preTriggerBackToBackEnable; /*!< Enable the back to back mode for ADC pre_trigger. @internal gui name="Back-To-Back mode" id="AdcBackToBackMode" */
} pdb_adc_pretrigger_config_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the PDB counter and triggers input.
 *
 * This function initializes the PDB counter and triggers the input.
 * It resets PDB registers and enables the PDB clock. Therefore, it should be
 * called before any other operation. After it is initialized, the PDB can
 * act as a triggered timer, which enables other features in PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] userConfigPtr Pointer to the user configuration structure. See the "pdb_user_config_t".
 */
void PDB_DRV_Init(const uint32_t instance,
                  const pdb_timer_config_t * userConfigPtr);

/*!
 * @brief De-initializes the PDB module.
 *
 * This function de-initializes the PDB module.
 * Calling this function shuts down the PDB module and reduces the power consumption.
 *
 * @param[in] instance PDB instance ID.
 */
void PDB_DRV_Deinit(const uint32_t instance);

/*!
 * @brief Gets the default configuration structure of PDB with default settings.
 *
 * This function initializes the hardware configuration structure to default values
 * (Reference Manual Resets).
 * This function should be called before configuring the hardware feature by PDB_DRV_Init() function,
 * otherwise all members be written by user.
 * This function insures that all members are written with safe values, but the user still can
 * modify the desired members.
 *
 * @param[out] config Pointer to PDB configuration structure.
 */
void PDB_DRV_GetDefaultConfig(pdb_timer_config_t * const config);

/*!
 * @brief Enables the PDB module.
 *
 * This function enables the PDB module, counter is on.
 *
 * @param[in] instance PDB instance ID.
 */
void PDB_DRV_Enable(const uint32_t instance);

/*!
 * @brief Disables the PDB module.
 *
 * This function disables the PDB module, counter is off also.
 *
 * @param[in] instance PDB instance ID.
 */
void PDB_DRV_Disable(const uint32_t instance);

/*!
 * @brief Triggers the PDB with a software trigger.
 *
 * This function triggers the PDB with a software trigger.
 * When the PDB is set to use the software trigger as input, calling this function
 * triggers the PDB.
 *
 * @param[in] instance PDB instance ID.
 */
void PDB_DRV_SoftTriggerCmd(const uint32_t instance);

/*!
 * @brief Gets the current counter value in the PDB module.
 *
 * This function gets the current counter value.
 *
 * @param[in] instance PDB instance ID.
 * @return Current PDB counter value.
 */
uint32_t PDB_DRV_GetTimerValue(const uint32_t instance);

/*!
 * @brief Gets the PDB interrupt flag.
 *
 * This function gets the PDB interrupt flag. It is asserted if the PDB interrupt occurs.
 *
 * @param[in] instance PDB instance ID.
 * @return Assertion of indicated event.
 */
bool PDB_DRV_GetTimerIntFlag(const uint32_t instance);

/*!
 * @brief Clears the interrupt flag.
 *
 * This function clears the interrupt flag.
 *
 * @param[in] instance PDB instance ID.
 */
void PDB_DRV_ClearTimerIntFlag(const uint32_t instance);

/*!
 * @brief Executes the command of loading values.
 *
 * This function executes the command of loading values.
 *
 * @param[in] instance PDB instance ID.
 */
void PDB_DRV_LoadValuesCmd(const uint32_t instance);

/*!
 * @brief Sets the value of timer modulus.
 *
 * This function sets the value of timer modulus.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] value Setting value.
 */
void PDB_DRV_SetTimerModulusValue(const uint32_t instance,
                                  const uint16_t value);

/*!
 * @brief Sets the value for the timer interrupt.
 *
 * This function sets the value for the timer interrupt.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] value Setting value.
 */
void PDB_DRV_SetValueForTimerInterrupt(const uint32_t instance,
                                       const uint16_t value);

/*!
 * @brief Configures the ADC pre_trigger in the PDB module.
 *
 * This function configures the ADC pre_trigger in the PDB module.
 * Any pretrigger which is enabled and has the trigger output enabled ( preTriggerOutputEnable
 * and preTriggerEnable both true) must have the corresponding delay value set to a non-zero
 * value by calling ::PDB_DRV_SetAdcPreTriggerDelayValue function.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] chn PDB channel.
 * @param[in] configPtr Pointer to the user configuration structure. See ::pdb_adc_pretrigger_config_t.
 */
void PDB_DRV_ConfigAdcPreTrigger(const uint32_t instance,
                                 const uint32_t chn,
                                 const pdb_adc_pretrigger_config_t * configPtr);

/*!
 * @brief Gets the ADC pre_trigger flag in the PDB module.
 *
 * This function gets the ADC pre_trigger flags in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] chn PDB channel.
 * @param[in] preChnMask ADC pre_trigger channels mask.
 * @return Assertion of indicated flag.
 */
uint32_t PDB_DRV_GetAdcPreTriggerFlags(const uint32_t instance,
                                       const uint32_t chn,
                                       const uint32_t preChnMask);

/*!
 * @brief Clears the ADC pre_trigger flag in the PDB module.
 *
 * This function clears the ADC pre_trigger flags in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] chn PDB channel.
 * @param[in] preChnMask ADC pre_trigger channels mask.
 */
void PDB_DRV_ClearAdcPreTriggerFlags(const uint32_t instance,
                                     const uint32_t chn,
                                     const uint32_t preChnMask);

/*!
 * @brief Gets the ADC pre_trigger flag in the PDB module.
 *
 * This function gets the ADC pre_trigger flags in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] chn PDB channel.
 * @param[in] preChnMask ADC pre_trigger channels mask.
 * @return Assertion of indicated flag.
 */
uint32_t PDB_DRV_GetAdcPreTriggerSeqErrFlags(const uint32_t instance,
                                             const uint32_t chn,
                                             const uint32_t preChnMask);

/*!
 * @brief Clears the ADC pre_trigger flag in the PDB module.
 *
 * This function clears the ADC pre_trigger sequence error flags in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] chn PDB channel.
 * @param[in] preChnMask ADC pre_trigger channels mask.
 */
void PDB_DRV_ClearAdcPreTriggerSeqErrFlags(const uint32_t instance,
                                           const uint32_t chn,
                                           const uint32_t preChnMask);

/*!
 * @brief Sets the ADC pre_trigger delay value in the PDB module.
 *
 * This function sets Set the ADC pre_trigger delay value in the PDB module.
 *
 *
 * @param instance PDB instance ID.
 * @param chn ADC channel.
 * @param preChn ADC pre_channel.
 * @param value Setting value.
 */
void PDB_DRV_SetAdcPreTriggerDelayValue(const uint32_t instance,
                                        const uint32_t chn,
                                        const uint32_t preChn,
                                        const uint32_t value);

/*!
 * @brief Switches on/off the CMP pulse out in the PDB module.
 *
 * This function switches the CMP pulse on/off in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] pulseChnMask Pulse channel mask.
 * @param[in] enable Switcher to assert the feature.
 */
void PDB_DRV_SetCmpPulseOutEnable(const uint32_t instance,
                                  const uint32_t pulseChnMask,
                                  bool enable);

/*!
 * @brief Sets the CMP pulse out delay value for high in the PDB module.
 *
 * This function sets the CMP pulse out delay value for high in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] pulseChn Pulse channel.
 * @param[in] value Setting value.
 */
void PDB_DRV_SetCmpPulseOutDelayForHigh(const uint32_t instance,
                                        const uint32_t pulseChn,
                                        const uint32_t value);

/*!
 * @brief Sets the CMP pulse out delay value for low in the PDB module.
 *
 * This function sets the CMP pulse out delay value for low in the PDB module.
 *
 * @param[in] instance PDB instance ID.
 * @param[in] pulseChn Pulse channel.
 * @param[in] value Setting value.
 */
void PDB_DRV_SetCmpPulseOutDelayForLow(const uint32_t instance,
                                       const uint32_t pulseChn,
                                       const uint32_t value);

#if defined(__cplusplus)
}
#endif

/*!
 *@}
 */

#endif /* PDB_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
