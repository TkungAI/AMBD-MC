/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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

#ifndef TRGMUX_DRIVER_H
#define TRGMUX_DRIVER_H

/*! @file trgmux_driver.h */

/**
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

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @addtogroup trgmux_driver
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* TRGMUX module features */
/*!
 * @brief Enumeration for trigger source module of TRGMUX
 *
 * Describes all possible inputs (trigger sources) of the TRGMUX IP
 * This enumeration depends on the supported instances in device
 *
 * Implements : trgmux_trigger_source_t_Class
 */
typedef enum trgmux_trigger_source_e trgmux_trigger_source_t;

/*!
 * @brief Enumeration for target module of TRGMUX
 *
 * Describes all possible outputs (target modules) of the TRGMUX IP
 * This enumeration depends on the supported instances in device
 *
 * Implements : trgmux_target_module_t_Class
 */
typedef enum trgmux_target_module_e trgmux_target_module_t;

/*!
 * @brief Configuration structure for pairing source triggers with target modules.
 *
 * Use an instance of this structure to define a TRGMUX link between a trigger source and a target module.
 * This structure is used by the user configuration structure.
 *
 * Implements : trgmux_inout_mapping_config_t_Class
 */
typedef struct
{
    trgmux_trigger_source_t triggerSource; /*!< selects one of the TRGMUX trigger sources */
    trgmux_target_module_t targetModule;   /*!< selects one of the TRGMUX target modules  */
    bool lockTargetModuleReg;              /*!< if true, the LOCK bit of the target module register will be
                                                set by TRGMUX_DRV_INIT(), after the current mapping is configured */
} trgmux_inout_mapping_config_t;

/*!
 * @brief User configuration structure for the TRGMUX driver.
 *
 * Use an instance of this structure with the TRGMUX_DRV_Init() function. This enables configuration of TRGMUX with the user
 * defined mappings between inputs (source triggers) and outputs (target modules), via a single function call.
 *
 * Implements : trgmux_user_config_t_Class
 */
typedef struct
{
    uint8_t numInOutMappingConfigs;                           /*!< number of in-out mappings defined in TRGMUX configuration */
    const trgmux_inout_mapping_config_t * inOutMappingConfig; /*!< pointer to array of in-out mapping structures */
} trgmux_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Initialize a TRGMUX instance for operation.
 *
 * This function first resets the source triggers of all TRGMUX target modules to their default values,
 * then configures the TRGMUX with all the user defined in-out mappings.
 * If at least one of the target modules is locked, the function will not change any of the TRGMUX target modules
 * and return error code.
 * This example shows how to set up the trgmux_user_config_t parameters and how to call the
 * TRGMUX_DRV_Init() function with the required parameters:
 *  @code
 *   trgmux_user_config_t             trgmuxConfig;
 *   trgmux_inout_mapping_config_t    trgmuxInoutMappingConfig[] =
 *   {
 *       {TRGMUX_TRIG_SOURCE_TRGMUX_IN9,     TRGMUX_TARGET_MODULE_DMA_CH0,     false},
 *       {TRGMUX_TRIG_SOURCE_FTM1_EXT_TRIG,  TRGMUX_TARGET_MODULE_TRGMUX_OUT4, true}
 *   };
 *
 *   trgmuxConfig.numInOutMappingConfigs = 2;
 *   trgmuxConfig.inOutMappingConfig     = trgmuxInoutMappingConfig;
 *
 *   TRGMUX_DRV_Init(instance, &trgmuxConfig);
 *   @endcode
 *
 * @param[in] instance          The TRGMUX instance number.
 * @param[in] trgmuxUserConfig  Pointer to the user configuration structure.
 * @return                      Execution status: \n
 *   STATUS_SUCCESS \n
 *   STATUS_ERROR    - if at least one of the target module register is locked.  */
status_t TRGMUX_DRV_Init(const uint32_t instance,
                         const trgmux_user_config_t * const trgmuxUserConfig);

/*!
 * @brief Reset to default values the source triggers corresponding to all target modules,
 * if none of the target modules is locked.
 *
 * @param[in] instance          The TRGMUX instance number.
 * @return                      Execution status: \n
 *   STATUS_SUCCESS \n
 *   STATUS_ERROR    - if at least one of the target module register is locked.
 */
status_t TRGMUX_DRV_Deinit(const uint32_t instance);

/*!
 * @brief Configure a source trigger for a selected target module.
 *
 * This function configures a TRGMUX link between a source trigger and a target module,
 * if the requested target module is not locked.
 *
 * @param[in] instance          The TRGMUX instance number.
 * @param[in] triggerSource     One of the values in the trgmux_trigger_source_t enumeration
 * @param[in] targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                      Execution status: \n
 *   STATUS_SUCCESS \n
 *   STATUS_ERROR    - if requested target module is locked  */
status_t TRGMUX_DRV_SetTrigSourceForTargetModule(const uint32_t instance,
                                                 const trgmux_trigger_source_t triggerSource,
                                                 const trgmux_target_module_t targetModule);

/*!
 * @brief Get the source trigger configured for a target module.
 *
 * This function returns the TRGMUX source trigger linked to a selected target module.
 *
 * @param[in] instance      The TRGMUX instance number.
 * @param[in] targetModule  One of the values in the trgmux_target_module_t enumeration.
 * @return                  Enum value corresponding to the trigger source configured
 *                          for the selected target module.
 */
trgmux_trigger_source_t TRGMUX_DRV_GetTrigSourceForTargetModule(const uint32_t instance,
                                                                const trgmux_target_module_t targetModule);

/*!
 * @brief Locks the TRGMUX register of a target module.
 *
 * This function sets the LK bit of the TRGMUX register corresponding to
 * the selected target module. Please note that some TRGMUX registers can contain up to 4
 * SEL bitfields, meaning that these registers can be used to configure up to 4 target
 * modules independently. Because the LK bit is only one per register, the configuration
 * of all target modules referred from that register will be locked.
 *
 * @param[in] instance          The TRGMUX instance number.
 * @param[in] targetModule      One of the values in the trgmux_target_module_t enumeration
 */
void TRGMUX_DRV_SetLockForTargetModule(const uint32_t instance,
                                       const trgmux_target_module_t targetModule);

/*!
 * @brief Get the Lock bit status of the TRGMUX register of a target module.
 *
 * This function gets the value of the LK bit from the TRGMUX register corresponding to
 * the selected target module.
 *
 * @param[in] instance          The TRGMUX instance number.
 * @param[in] targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                      true - if the selected targetModule register is locked \n
 *                              false - if the selected targetModule register is not locked
 */
bool TRGMUX_DRV_GetLockForTargetModule(const uint32_t instance,
                                       const trgmux_target_module_t targetModule);
									   
/*!
 * @brief Generate software triggers
 *
 * This function uses a SIM register in order to generate a software triggers to the target 
 * peripherals selected in TRGMUX
 *
 * @param param[in] instance          The TRGMUX instance number.
 */
void TRGMUX_DRV_GenSWTrigger(const uint32_t instance);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* TRGMUX_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
