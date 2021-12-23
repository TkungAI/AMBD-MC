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

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"

/*! @file adc_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
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

/*!
 * @addtogroup adc_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Clock Divider selection
 *
 * Implements : adc_clk_divide_t_Class
 */
typedef enum
{
    ADC_CLK_DIVIDE_1 = 0x00U,   /*!< Input clock divided by 1. */
    ADC_CLK_DIVIDE_2 = 0x01U,   /*!< Input clock divided by 2. */
    ADC_CLK_DIVIDE_4 = 0x02U,   /*!< Input clock divided by 4. */
    ADC_CLK_DIVIDE_8 = 0x03U    /*!< Input clock divided by 8. */
} adc_clk_divide_t;

/*!
 * @brief Conversion resolution selection
 *
 * Implements : adc_resolution_t_Class
 */
typedef enum
{
    ADC_RESOLUTION_8BIT = 0x00U,    /*!< 8-bit resolution mode */
    ADC_RESOLUTION_12BIT = 0x01U,   /*!< 12-bit resolution mode */
    ADC_RESOLUTION_10BIT = 0x02U    /*!< 10-bit resolution mode */
} adc_resolution_t;

/*!
 * @brief Input clock source selection
 *
 * Implements : adc_input_clock_t_Class
 */
typedef enum
{
    ADC_CLK_ALT_1 = 0x00U,  /*!< Input clock alternative 1. */
    ADC_CLK_ALT_2 = 0x01U,  /*!< Input clock alternative 2. */
    ADC_CLK_ALT_3 = 0x02U,  /*!< Input clock alternative 3. */
    ADC_CLK_ALT_4 = 0x03U   /*!< Input clock alternative 4. */
} adc_input_clock_t;

/*!
 * @brief Trigger type selection
 *
 * Implements : adc_trigger_t_Class
 */
typedef enum
{
    ADC_TRIGGER_SOFTWARE       = 0x00U,   /*!< Software trigger. */
    ADC_TRIGGER_HARDWARE       = 0x01U    /*!< Hardware trigger. */
} adc_trigger_t;

/*!
 * @brief Pretrigger types selectable from Trigger Latching and Arbitration Unit
 *
 * Implements : adc_pretrigger_sel_t_Class
 */
typedef enum
{
    ADC_PRETRIGGER_SEL_PDB     = 0x00U,   /*!< PDB pretrigger selected. */
    ADC_PRETRIGGER_SEL_TRGMUX  = 0x01U,   /*!< TRGMUX pretrigger selected. */
    ADC_PRETRIGGER_SEL_SW      = 0x02U    /*!< Software pretrigger selected. */
} adc_pretrigger_sel_t;

/*!
 * @brief Trigger source selectable from Trigger Latching and Arbitration Unit
 *
 * Implements : adc_trigger_sel_t_Class
 */
typedef enum
{
    ADC_TRIGGER_SEL_PDB        = 0x00U,   /*!< PDB trigger selected. */
    ADC_TRIGGER_SEL_TRGMUX     = 0x01U    /*!< TRGMUX trigger selected. */
} adc_trigger_sel_t;

/*!
 * @brief Software pretriggers which may be set from Trigger Latching and Arbitration Unit
 *
 * Implements : adc_sw_pretrigger_t_Class
 */
typedef enum
{
    ADC_SW_PRETRIGGER_DISABLED = 0x00U,   /*!< SW pretrigger disabled. */
    ADC_SW_PRETRIGGER_0        = 0x04U,   /*!< SW pretrigger 0. */
    ADC_SW_PRETRIGGER_1        = 0x05U,   /*!< SW pretrigger 1. */
    ADC_SW_PRETRIGGER_2        = 0x06U,   /*!< SW pretrigger 2. */
    ADC_SW_PRETRIGGER_3        = 0x07U    /*!< SW pretrigger 3. */
} adc_sw_pretrigger_t;

/*!
 * @brief Voltage reference selection
 *
 * Implements : adc_voltage_reference_t_Class
 */
typedef enum
{
    ADC_VOLTAGEREF_VREF = 0x00U,    /*!< VrefH and VrefL as Voltage reference. */
    ADC_VOLTAGEREF_VALT = 0x01U     /*!< ValtH and ValtL as Voltage reference. */
} adc_voltage_reference_t;

/*!
 * @brief Hardware average selection
 *
 * Implements : adc_average_t_Class
 */
typedef enum
{
    ADC_AVERAGE_4 = 0x00U,  /*!< Hardware average of 4 samples. */
    ADC_AVERAGE_8 = 0x01U,  /*!< Hardware average of 8 samples. */
    ADC_AVERAGE_16 = 0x02U, /*!< Hardware average of 16 samples. */
    ADC_AVERAGE_32 = 0x03U  /*!< Hardware average of 32 samples. */
} adc_average_t;

/*!
 * @brief Enumeration of input channels assignable to a control channel.
 * \n <b>Note 0</b>: entries in this enum are affected by ::FEATURE_ADC_NUM_EXT_CHANS, which is device dependent
 * and controlled from <i>"device_name"_features.h</i> file.
 *
 * \n <b>Note 1</b>: the actual number of external channels may differ between device packages
 * and ADC instances. Reading a channel that is not connected externally, will return a random value within the range.
 * Please refer to the Reference Manual for the maximum number of external channels for each device variant and ADC instance.
 *
 * \n <b>Note 2</b>: ADC_INPUTCHAN_SUPPLY_ select which internal supply channel to be measured.
 * They are only available for ADC0 and measured internally via internal input channel 0.
 * Please note that supply monitoring needs to be enabled separately via dedicated flag in adc_converter_config_t.
 *
 * Implements : adc_inputchannel_t_Class
 */
typedef enum
{
    ADC_INPUTCHAN_EXT0      = 0x00U,              /*!< External input channel 0 */
    ADC_INPUTCHAN_EXT1      = 0x01U,              /*!< External input channel 1 */
#if (FEATURE_ADC_HAS_CHANNEL_2)
    ADC_INPUTCHAN_EXT2      = 0x02U,              /*!< External input channel 2 */
#endif
    ADC_INPUTCHAN_EXT3      = 0x03U,              /*!< External input channel 3 */
    ADC_INPUTCHAN_EXT4      = 0x04U,              /*!< External input channel 4 */
    ADC_INPUTCHAN_EXT5      = 0x05U,              /*!< External input channel 5 */
    ADC_INPUTCHAN_EXT6      = 0x06U,              /*!< External input channel 6 */
    ADC_INPUTCHAN_EXT7      = 0x07U,              /*!< External input channel 7 */
#if (FEATURE_ADC_HAS_CHANNEL_8)
    ADC_INPUTCHAN_EXT8      = 0x08U,              /*!< External input channel 8 */
#endif
    ADC_INPUTCHAN_EXT9      = 0x09U,              /*!< External input channel 9 */
    ADC_INPUTCHAN_EXT10     = 0x0AU,              /*!< External input channel 10 */
    ADC_INPUTCHAN_EXT11     = 0x0BU,              /*!< External input channel 11 */
    ADC_INPUTCHAN_EXT12     = 0x0CU,              /*!< External input channel 12 */
    ADC_INPUTCHAN_EXT13     = 0x0DU,              /*!< External input channel 13 */
    ADC_INPUTCHAN_EXT14     = 0x0EU,              /*!< External input channel 14 */
#if (FEATURE_ADC_MAX_NUM_EXT_CHANS > 15)
    ADC_INPUTCHAN_EXT15     = 0x0FU,              /*!< External input channel 15 */
#if (FEATURE_ADC_MAX_NUM_EXT_CHANS > 16)
    ADC_INPUTCHAN_EXT16     = 0x20U,              /*!< External input channel 16 */
    ADC_INPUTCHAN_EXT17     = 0x21U,              /*!< External input channel 17 */
    ADC_INPUTCHAN_EXT18     = 0x22U,              /*!< External input channel 18 */
    ADC_INPUTCHAN_EXT19     = 0x23U,              /*!< External input channel 19 */
    ADC_INPUTCHAN_EXT20     = 0x24U,              /*!< External input channel 20 */
    ADC_INPUTCHAN_EXT21     = 0x25U,              /*!< External input channel 21 */
    ADC_INPUTCHAN_EXT22     = 0x26U,              /*!< External input channel 22 */
    ADC_INPUTCHAN_EXT23     = 0x27U,              /*!< External input channel 23 */
#if (FEATURE_ADC_MAX_NUM_EXT_CHANS > 24)
    ADC_INPUTCHAN_EXT24     = 0x28U,              /*!< External input channel 24 */
    ADC_INPUTCHAN_EXT25     = 0x29U,              /*!< External input channel 25 */
    ADC_INPUTCHAN_EXT26     = 0x2AU,              /*!< External input channel 26 */
    ADC_INPUTCHAN_EXT27     = 0x2BU,              /*!< External input channel 27 */
    ADC_INPUTCHAN_EXT28     = 0x2CU,              /*!< External input channel 28 */
    ADC_INPUTCHAN_EXT29     = 0x2DU,              /*!< External input channel 29 */
    ADC_INPUTCHAN_EXT30     = 0x2EU,              /*!< External input channel 30 */
    ADC_INPUTCHAN_EXT31     = 0x2FU,              /*!< External input channel 31 */
#endif /* (FEATURE_ADC_MAX_NUM_EXT_CHANS > 24) */
#endif /* (FEATURE_ADC_MAX_NUM_EXT_CHANS > 16) */
#endif/* (FEATURE_ADC_MAX_NUM_EXT_CHANS > 15) */

    ADC_INPUTCHAN_DISABLED  = ADC_SC1_ADCH_MASK,  /*!< Channel disabled          */

    ADC_INPUTCHAN_INT0      = 0x15,               /*!< Internal input channel 0  */
    ADC_INPUTCHAN_INT1      = 0x16,               /*!< Internal input channel 1  */
    ADC_INPUTCHAN_INT2      = 0x17,               /*!< Internal input channel 2  */
    ADC_INPUTCHAN_INT3      = 0x1C,               /*!< Internal input channel 3  */

    ADC_INPUTCHAN_TEMP      = 0x1A,               /*!< Temperature Sensor            */
    ADC_INPUTCHAN_BANDGAP   = 0x1B,               /*!< Band Gap                      */
    ADC_INPUTCHAN_VREFSH    = 0x1D,               /*!< Voltage Reference Select High */
    ADC_INPUTCHAN_VREFSL    = 0x1E,               /*!< Voltage Reference Select Low  */

    ADC_INPUTCHAN_SUPPLY_VDD            = 0xF00U,   /*!< Monitor internal supply 5 V input VDD supply.              */
    ADC_INPUTCHAN_SUPPLY_VDDA           = 0xF01U,   /*!< Monitor internal supply 5 V input analog supply.           */
    ADC_INPUTCHAN_SUPPLY_VREFH          = 0xF02U,   /*!< Monitor internal supply ADC reference supply.              */
    ADC_INPUTCHAN_SUPPLY_VDD_3V         = 0xF03U,   /*!< Monitor internal supply 3.3 V oscillator regulator output. */
    ADC_INPUTCHAN_SUPPLY_VDD_FLASH_3V   = 0xF04U,   /*!< Monitor internal supply 3.3 V flash regulator output.      */
    ADC_INPUTCHAN_SUPPLY_VDD_LV         = 0xF05U    /*!< Monitor internal supply 1.2 V core regulator output.       */
} adc_inputchannel_t;

/*!
 * @brief Defines the converter configuration
 *
 * This structure is used to configure the ADC converter
 *
 * Implements : adc_converter_config_t_Class
 */
typedef struct
{
    adc_clk_divide_t clockDivide;        /*!< Divider of the input clock for the ADC */
    uint8_t sampleTime;                  /*!< Sample time in AD Clocks */
    adc_resolution_t resolution;         /*!< ADC resolution (8,10,12 bit) */
    adc_input_clock_t inputClock;        /*!< Input clock source */
    adc_trigger_t trigger;               /*!< ADC trigger type (software, hardware) - affects only the first control channel */
    adc_pretrigger_sel_t pretriggerSel;  /*!< Pretrigger source selected from Trigger Latching and Arbitration Unit - affects only the first 4 control channels */
    adc_trigger_sel_t triggerSel;        /*!< Trigger source selected from Trigger Latching and Arbitration Unit */
    bool dmaEnable;                      /*!< Enable DMA for the ADC */
    adc_voltage_reference_t voltageRef;  /*!< Voltage reference used */
    bool continuousConvEnable;           /*!< Enable Continuous conversions */
    bool supplyMonitoringEnable;         /*!< Only available for ADC 0. Enable internal supply monitoring - enables measurement of ADC_INPUTCHAN_SUPPLY_ sources. */
} adc_converter_config_t;

/*!
 * @brief Defines the hardware compare configuration
 *
 * This structure is used to configure the hardware compare
 * feature for the ADC
 *
 * Implements : adc_compare_config_t_Class
 */
typedef struct
{
    bool compareEnable;              /*!< Enable the compare feature */
    bool compareGreaterThanEnable;   /*!< Enable Greater-Than functionality */
    bool compareRangeFuncEnable;     /*!< Enable Range functionality */
    uint16_t compVal1;               /*!< First Compare Value */
    uint16_t compVal2;               /*!< Second Compare Value */
} adc_compare_config_t;

/*!
 * @brief Defines the hardware average configuration
 *
 * This structure is used to configure the hardware average
 * feature for the ADC
 *
 * Implements : adc_average_config_t_Class
 */
typedef struct
{
    bool hwAvgEnable;        /*!< Enable averaging functionality */
    adc_average_t hwAverage; /*!< Selection for number of samples used for averaging */
} adc_average_config_t;

/*!
 * @brief Defines the control channel configuration
 *
 * This structure is used to configure a control channel
 * of the ADC
 *
 * Implements : adc_chan_config_t_Class
 */
typedef struct
{
    bool interruptEnable;       /*!< Enable interrupts for this channel */
    adc_inputchannel_t channel; /*!< Selection of input channel for measurement */
} adc_chan_config_t;

/*!
 * @brief Defines the user calibration configuration
 *
 * This structure is used to configure the user calibration
 * parameters of the ADC.
 *
 * Implements : adc_calibration_t_Class
 */
typedef struct
{
    uint16_t userGain;    /*!< User-configurable gain */
    uint16_t userOffset;  /*!< User-configurable Offset (2's complement, subtracted from result) */
} adc_calibration_t;

/*!
 * @brief Defines the trigger latch clear method
 * Implements : adc_latch_clear_t_Class
 */
typedef enum
{
    ADC_LATCH_CLEAR_WAIT, /*!< Clear by waiting all latched triggers to be processed */
    ADC_LATCH_CLEAR_FORCE /*!< Process current trigger and clear all latched */
} adc_latch_clear_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @name Converter
 */
/*! @{*/

/*!
 * @brief Initializes the converter configuration structure
 *
 * This function initializes the members of the adc_converter_config_t
 * structure to default values (Reference Manual resets). This function should
 * be called on a structure before using it to configure the converter with
 * ADC_DRV_ConfigConverter(), otherwise all members must be written
 * (initialized) by the user. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitConverterStruct(adc_converter_config_t * const config);

/*!
 * @brief Configures the converter with the given configuration structure
 *
 * This function configures the ADC converter with the options
 * provided in the provided structure.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 */
void ADC_DRV_ConfigConverter(const uint32_t instance,
                             const adc_converter_config_t * const config);

/*!
 * @brief Gets the current converter configuration
 *
 * This functions returns the configuration for converter in
 * the form of a configuration structure.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 */
void ADC_DRV_GetConverterConfig(const uint32_t instance,
                                adc_converter_config_t * const config);

/*! @}*/

/*!
 * @name Hardware Compare
 * The Hardware Compare feature of the S32K144 ADC is a versatile mechanism that
 * can be used to monitor that a value is within certain values. Measurements can
 * be monitored to be within certain ranges:
 *  - less than/ greater than a fixed value
 *  - inside or outside of a certain range
 *
 * Two compare values can be configured (the second value is used only for range
 * function mode). The compare values must be written in 12-bit resolution mode
 * regardless of the actual used resolution mode.
 *
 * Once the hardware compare feature is enabled, a conversion is considered
 * complete only when the measured value is within the allowable range set by
 * the configuration.
 */
/*! @{*/

/*!
 * @brief Initializes the Hardware Compare configuration structure
 *
 * This function initializes the Hardware Compare configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Compare feature (ADC_DRV_ConfigHwCompare),
 * otherwise all members must be written by the caller. This function insures
 * that all members are written with safe values, so the user can modify the
 * desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitHwCompareStruct(adc_compare_config_t * const config);

/*!
 * @brief Configures the Hardware Compare feature with the given configuration
 * structure
 *
 * This functions sets the configuration for the Hardware
 * Compare feature using the configuration structure.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 */
void ADC_DRV_ConfigHwCompare(const uint32_t instance,
                             const adc_compare_config_t * const config);

/*!
 * @brief Gets the current Hardware Compare configuration
 *
 * This function returns the configuration for the Hardware
 * Compare feature.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 */
void ADC_DRV_GetHwCompareConfig(const uint32_t instance,
                                adc_compare_config_t * const config);

/*! @}*/
/*!
 * @name Hardware Average
 * The Hardware Average feature of the S32K144 allows for a set of measurements
 * to be averaged together as a single conversion. The number of samples to be
 * averaged is selectable (4, 8, 16 or 32 samples).
 */
/*! @{*/

/*!
 * @brief Initializes the Hardware Average configuration structure
 *
 * This function initializes the Hardware Average configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Average feature (ADC_DRV_ConfigHwAverage),
 * otherwise all members must be written by the caller. This function insures
 * that all members are written with safe values, so the user can modify the
 * desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitHwAverageStruct(adc_average_config_t * const config);

/*!
 * @brief Configures the Hardware Average feature with the given configuration
 * structure
 *
 * This function sets the configuration for the Hardware
 * Average feature.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 */
void ADC_DRV_ConfigHwAverage(const uint32_t instance,
                             const adc_average_config_t * const config);

/*!
 * @brief Gets the current Hardware Average configuration
 *
 * This function returns the configuration for the Hardware
 * Average feature.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 */
void ADC_DRV_GetHwAverageConfig(const uint32_t instance,
                                adc_average_config_t * const config);

/*! @}*/

/*!
 * @name Channel configuration
 */
/*! @{*/

/*!
 * @brief Initializes the control channel configuration structure
 *
 * This function initializes the control channel
 * configuration structure to default values (Reference Manual resets). This
 * function should be called on a structure before using it to configure a
 * channel (ADC_DRV_ConfigChan), otherwise all members must be written by the
 * caller. This function insures that all members are written with safe values,
 * so the user can modify only the desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitChanStruct(adc_chan_config_t * const config);

/*!
 * @brief Configures the selected control channel with the given
 * configuration structure
 *
 * When Software Trigger mode is enabled, configuring control channel index 0,
 * implicitly triggers a new conversion on the selected ADC input channel.
 * Therefore, ADC_DRV_ConfigChan can be used for sw-triggering conversions.
 *
 * Configuring any control channel while it is actively controlling a conversion
 * (sw or hw triggered) will implicitly abort the on-going conversion.
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the control channel index
 * @param[in] config the configuration structure
 */
void ADC_DRV_ConfigChan(const uint32_t instance,
                        const uint8_t chanIndex,
                        const adc_chan_config_t * const config);

/*!
 * @brief Gets the current control channel configuration for the selected
 * channel index
 *
 * This function returns the configuration for a control channel
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the control channel index
 * @param[out] config the configuration structure
 */
void ADC_DRV_GetChanConfig(const uint32_t instance,
                           const uint8_t chanIndex,
                           adc_chan_config_t * const config);

/*!
 * @brief This function sets the software pretrigger - affects only first 4 control channels.
 *
 * @param[in] instance  instance number
 * @param[in] swPretrigger  the swPretrigger to be enabled
 */
void ADC_DRV_SetSwPretrigger(const uint32_t instance,
                             const adc_sw_pretrigger_t swPretrigger);

/*! @}*/
/*!
 * @name Converter
 * Converter specific methods. These are used to configure and use the A/D
 * Converter specific functionality, including:
 *  - clock input and divider
 *  - sample time in A/D clocks
 *  - resolution
 *  - trigger source
 *  - voltage reference
 *  - enable DMA
 *  - enable continuous conversion on one channel
 *
 * To start a conversion, a control channel (see \ref chan_config "Channel Configuration")
 * and a trigger source must be configured. Once a conversion is started, the user
 * application can wait for it to be finished by calling the ADC_DRV_WaitConvDone() function.
 *
 * Only the first control channel can be triggered by software. To start a
 * conversion in this case, an input channel must be written in the channel
 * selection register using the ADC_DRV_ConfigChan() method. Writing a value to
 * the control channel while a conversion is being performed on that channel
 * will start a new conversion.
 *
 */
/*! @{*/

/*!
 * @brief Resets the converter (sets all configurations to reset values)
 *
 * This function resets all the internal ADC registers to
 * reset values.
 *
 * @param[in] instance instance number
 */
void ADC_DRV_Reset(const uint32_t instance);

/*!
 * @brief Waits for a conversion/calibration to finish
 *
 * This functions waits for a conversion to complete by
 * continuously polling the Conversion Active Flag.
 *
 * @param[in] instance instance number
 */
void ADC_DRV_WaitConvDone(const uint32_t instance);

/*!
 * @brief Gets the control channel Conversion Complete Flag state
 *
 * This function returns the state of the Conversion Complete
 * flag for a control channel. This flag is set when a conversion
 * is complete or the condition generated by the Hardware
 * Compare feature is evaluated to true.
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the adc control channel index
 * @return the Conversion Complete Flag state
 */
bool ADC_DRV_GetConvCompleteFlag(const uint32_t instance,
                                 const uint8_t chanIndex);
/*! @}*/

/*!
 * @anchor chan_config
 * @name Channel configuration
 * Control register specific functions. These functions control configurations
 * for each control channel (input channel selection and interrupt enable).
 *
 * When software triggering is enabled, calling the ADC_DRV_ConfigChan() method
 * for control channel 0 starts a new conversion.
 *
 * After a conversion is finished, the result can be retrieved using the
 * ADC_DRV_GetChanResult() method.
 */
/*! @{*/

/*!
 * @brief Gets the last result for the selected control channel
 *
 * This function returns the conversion result from a
 * control channel.
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the converter control channel index
 * @param[out] result the result raw value
 */
void ADC_DRV_GetChanResult(const uint32_t instance,
                           const uint8_t chanIndex,
                           uint16_t * const result);

/*! @}*/

/*!
 * @name Automatic Calibration
 * These methods control the Calibration feature of the ADC.
 *
 * The ADC_DRV_AutoCalibration() method can be called to execute a calibration
 * sequence, or a calibration can be retrieved with the ADC_DRV_GetUserCalibration()
 * and saved to non-volatile storage, to avoid calibration on every power-on.
 * The calibration structure can be written with the ADC_DRV_ConfigUserCalibration()
 * method.
 */
/*! @{*/

/*!
 * @brief Executes an Auto-Calibration
 *
 * This functions executes an Auto-Calibration sequence. It
 * is recommended to run this sequence before using the ADC
 * converter.
 *
 * @param[in] instance instance number
 */
void ADC_DRV_AutoCalibration(const uint32_t instance);

/*!
 * @brief Initializes the User Calibration configuration structure
 *
 * This function initializes the User Calibration configuration
 * structure to default values (Reference Manual resets). This function should
 * be called on a structure before using it to configure the User Calibration
 * feature (ADC_DRV_ConfigUserCalibration), otherwise all members must be
 * written by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 * this function will check and reset clock divide based the adc frequency.
 * an error will be displayed if frequency is greater than required clock for calibration.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitUserCalibrationStruct(adc_calibration_t * const config);

/*!
 * @brief Configures the User Calibration feature with the given configuration
 * structure
 *
 * This function sets the configuration for the user calibration
 * registers.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 */
void ADC_DRV_ConfigUserCalibration(const uint32_t instance,
                                   const adc_calibration_t * const config);

/*!
 * @brief Gets the current User Calibration configuration
 *
 * This function returns the current user calibration
 * register values.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 */
void ADC_DRV_GetUserCalibration(const uint32_t instance,
                                adc_calibration_t * const config);

/*! @}*/

/*!
 * @name Interrupts
 * This method returns the interrupt number for an ADC instance, which can be used
 * to configure the interrupt, like in Interrupt Manager.
 */
/*! @{*/

/*!
 * @brief Returns the interrupt number for the ADC instance.
 *
 * This function returns the interrupt number for the specified ADC instance.
 *
 * @param[in] instance instance number of the ADC
 * @return irq_number: the interrupt number (index) of the ADC instance, used to configure the interrupt
 */
IRQn_Type ADC_DRV_GetInterruptNumber(const uint32_t instance);

/*! @}*/

/*!
 * @name Latched triggers processing
 * These functions provide basic operations for using the trigger latch mechanism.
 */
/*! @{*/

/*!
 * @brief Clear latched triggers under processing
 *
 * This function clears all trigger latched flags of the ADC instance.
 * This function must be called after the hardware trigger source for the ADC has been deactivated.
 *
 * @param[in] instance instance number of the ADC
 * @param[in] clearMode The clearing method for the latched triggers
 *        - ADC_LATCH_CLEAR_WAIT : Wait for all latched triggers to be processed.
 *        - ADC_LATCH_CLEAR_FORCE : Clear latched triggers and wait for trigger being process to finish.
 */
void ADC_DRV_ClearLatchedTriggers(const uint32_t instance,
                                  const adc_latch_clear_t clearMode);

/*!
 * @brief Clear all latch trigger error
 *
 * This function clears all trigger error flags of the ADC instance.
 *
 * @param[in] instance instance number of the ADC
 */
void ADC_DRV_ClearTriggerErrors(const uint32_t instance);

/*!
 * @brief Get the trigger error flags bits of the ADC instance
 *
 * This function returns the trigger error flags bits of the ADC instance.
 *
 * @param[in] instance instance number of the ADC
 * @return trigErrorFlags The Trigger Error Flags bit-mask
 */
uint32_t ADC_DRV_GetTriggerErrorFlags(const uint32_t instance);

/*! @}*/

#if defined (__cplusplus)
}
#endif

/*! @}*/

#endif /* ADC_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
