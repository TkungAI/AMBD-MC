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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, conversion between a pointer and integer type
 * The cast from unsigned integer to (TRGMUX_Type *) is required in order to initialize
 * the table of base addresses for the TRGMUX instances.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, cast from unsigned int to pointer
 * The cast from unsigned integer to (TRGMUX_Type *) is required in order to initialize
 * the table of base addresses for the TRGMUX instances.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include <stddef.h>
#include "trgmux_driver.h"
#include "trgmux_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for TRGMUX instances. */
static TRGMUX_Type * const s_trgmuxBase[TRGMUX_INSTANCE_COUNT] = TRGMUX_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name :     TRGMUX_DRV_Init
 * Description   :     This function first resets the source triggers of all TRGMUX target modules
 * to their default values, then configures the TRGMUX with all the user defined in-out mappings.
 * If at least one of the target modules is locked, the function will not change any of the
 * TRGMUX target modules and return an error code.
 * This example shows how to set up the trgmux_user_config_t parameters and how to call the
 * TRGMUX_DRV_Init() function with the required parameters:
 *   trgmux_user_config_t             trgmuxConfig;
 *   trgmux_inout_mapping_config_t    trgmuxInoutMappingConfig[] =
 *   {
 *      {TRGMUX_TRIG_SOURCE_TRGMUX_IN9,     TRGMUX_TARGET_MODULE_DMA_CH0,     false},
 *      {TRGMUX_TRIG_SOURCE_FTM1_EXT_TRIG,  TRGMUX_TARGET_MODULE_TRGMUX_OUT4, true}
 *   };
 *   trgmuxConfig.numInOutMappingConfigs = 2;
 *   trgmuxConfig.inOutMappingConfig     = trgmuxInoutMappingConfig;
 *   TRGMUX_DRV_Init(instance, &trgmuxConfig);
 *
 * Implements    :     TRGMUX_DRV_Init_Activity
 *END**************************************************************************/
status_t TRGMUX_DRV_Init(const uint32_t instance,
                         const trgmux_user_config_t * const trgmuxUserConfig)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(trgmuxUserConfig != NULL);

    status_t status;
    TRGMUX_Type * base = s_trgmuxBase[instance];
    uint8_t count;

    /* Reset source triggers of all TRGMUX target modules to default. */
    status = TRGMUX_Init(base);

    if (status == STATUS_SUCCESS)
    {
        /* Loop through all in-out mappings in the configuration and apply them in TRGMUX */
        for (count = 0U; count < trgmuxUserConfig->numInOutMappingConfigs; count++)
        {
            TRGMUX_SetTrigSourceForTargetModule(base, trgmuxUserConfig->inOutMappingConfig[count].triggerSource,
                                                trgmuxUserConfig->inOutMappingConfig[count].targetModule);
        }

        /* Loop through all in-out mappings in the configuration and lock them if required */
        for (count = 0U; count < trgmuxUserConfig->numInOutMappingConfigs; count++)
        {
            if (trgmuxUserConfig->inOutMappingConfig[count].lockTargetModuleReg)
            {
                TRGMUX_SetLockForTargetModule(base, trgmuxUserConfig->inOutMappingConfig[count].targetModule);
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_Deinit
 * Description   : Reset to default values the source triggers corresponding to all target modules,
 * if none of the target modules is locked.
 *
 * Implements    : TRGMUX_DRV_Deinit_Activity
 *END**************************************************************************/
status_t TRGMUX_DRV_Deinit(const uint32_t instance)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);

    TRGMUX_Type * base = s_trgmuxBase[instance];
    status_t status;

    /* Reset source triggers of all TRGMUX target modules to default. */
    status = TRGMUX_Init(base);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_SetTrigSourceForTargetModule
 * Description   : This function configures a TRGMUX link between a source trigger and a target module,
 * if the requested target module is not locked.
 *
 * Implements    : TRGMUX_DRV_SetTrigSourceForTargetModule_Activity
 *END**************************************************************************/
status_t TRGMUX_DRV_SetTrigSourceForTargetModule(const uint32_t instance,
                                                 const trgmux_trigger_source_t triggerSource,
                                                 const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);

    TRGMUX_Type * base = s_trgmuxBase[instance];
    status_t status;
    bool lock;

    lock = TRGMUX_GetLockForTargetModule(base, targetModule);

    if (lock == true)
    {
        status = STATUS_ERROR;
    }
    else
    {
        /* Configure link between trigger source and target module. */
        TRGMUX_SetTrigSourceForTargetModule(base, triggerSource, targetModule);
        status = STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_GetTrigSourceForTargetModule
 * Description   : This function returns the TRGMUX source trigger linked to a selected target module.
 *
 * Implements    : TRGMUX_DRV_GetTrigSourceForTargetModule_Activity
 *END**************************************************************************/
trgmux_trigger_source_t TRGMUX_DRV_GetTrigSourceForTargetModule(const uint32_t instance,
                                                                const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);

    const TRGMUX_Type * base = s_trgmuxBase[instance];

    return TRGMUX_GetTrigSourceForTargetModule(base, targetModule);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_SetLockForTargetModule
 * Description   : This function locks the TRGMUX register of a selected target module.
 *
 * Implements    : TRGMUX_DRV_SetLockForTargetModule_Activity
 *END**************************************************************************/
void TRGMUX_DRV_SetLockForTargetModule(const uint32_t instance,
                                       const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);

    TRGMUX_Type * base = s_trgmuxBase[instance];

    TRGMUX_SetLockForTargetModule(base, targetModule);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_GetLockForTargetModule
 * Description   : This function gets the value of the LK bit from the TRGMUX register
 * corresponding to the selected target module.
 *
 * Implements    : TRGMUX_DRV_GetLockForTargetModule_Activity
 *END**************************************************************************/
bool TRGMUX_DRV_GetLockForTargetModule(const uint32_t instance,
                                       const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);

    const TRGMUX_Type * base = s_trgmuxBase[instance];

    return TRGMUX_GetLockForTargetModule(base, targetModule);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_GenSWTrigger
 * Description   : This function uses a SIM register in order to generate software
 * triggers to the target peripherals selected in TRGMUX
 *
 * Implements    : TRGMUX_DRV_GenSWTrigger_Activity
 *END**************************************************************************/
void TRGMUX_DRV_GenSWTrigger(const uint32_t instance)
{
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
	(void)instance;
	
	static SIM_Type * const s_simBase[SIM_INSTANCE_COUNT] = SIM_BASE_PTRS;

	/* The trigger is generated only when writing from 0 to 1*/
	s_simBase[0U]->MISCTRL1 = 0U;
	s_simBase[0U]->MISCTRL1 = SIM_MISCTRL1_SW_TRG(1U);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
