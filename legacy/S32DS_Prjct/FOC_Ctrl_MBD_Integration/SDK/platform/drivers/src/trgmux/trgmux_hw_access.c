/*
 * Copyright 2017 NXP.
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
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'
 * The cast is used to convert from uint32_t to enum_type returned by function.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type
 * The cast is used to convert from uint32_t to enum_type returned by function.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
 * These are generated macros used for accessing the bit-fields from registers.
 *
 */

#include <stddef.h>
#include "trgmux_hw_access.h"

/*******************************************************************************
* Definitions
*******************************************************************************/
/* Number of possible outputs (target module) for TRGMUX IP */
#define TRGMUX_NUM_TARGET_MODULES        ((uint8_t)(sizeof(s_trgmuxTargetModule) / sizeof(trgmux_target_module_t)))
/* Number of SEL bitfields in one TRGMUX register */
#define TRGMUX_NUM_SEL_BITFIELDS_PER_REG (4U)
/* Get the index of the TRGMUX register */
#define TRGMUX_IDX_REG(x)                ((uint8_t)((uint8_t)(x) / TRGMUX_NUM_SEL_BITFIELDS_PER_REG))
/* Get the index of the SEL bitfield inside TRGMUX register */
#define TRGMUX_IDX_SEL_BITFIELD_REG(x)   ((uint8_t)((uint8_t)(x) % TRGMUX_NUM_SEL_BITFIELDS_PER_REG))
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_Init
 * Description   : This function restores the TRGMUX module to reset value.
 *
 *END**************************************************************************/
status_t TRGMUX_Init(TRGMUX_Type * const base)
{
    DEV_ASSERT(base != NULL);

    /* Constant array storing the value of all TRGMUX output(target module) identifiers */
    static const trgmux_target_module_t s_trgmuxTargetModule[] = FEATURE_TRGMUX_TARGET_MODULE;
    uint8_t count = 0U;
    bool lock = false;
    status_t status = STATUS_ERROR;

    /* Check if any of the TRGMUX registers is locked */
    while ((count < TRGMUX_NUM_TARGET_MODULES) && (lock != true))
    {
        lock = TRGMUX_GetLockForTargetModule(base, s_trgmuxTargetModule[count]);
        count++;
    }

    /* Abort operations if at least one of the target module is locked. */
    if (lock == false)
    {
        /* Set all SEL bitfields of all TRGMUX registers to default value */
        for (count = 0U; count < TRGMUX_NUM_TARGET_MODULES; count++)
        {
            /* Write the TRGMUX register */
            base->TRGMUXn[TRGMUX_IDX_REG(s_trgmuxTargetModule[count])] &=
                ~((uint32_t)TRGMUX_TRGMUXn_SEL0_MASK << (TRGMUX_TRGMUXn_SEL1_SHIFT * TRGMUX_IDX_SEL_BITFIELD_REG(s_trgmuxTargetModule[count])));
        }

        status = STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_SetTrigSourceForTargetModule
 * Description   : This function configures a TRGMUX link between a source trigger
 * and a target module, if the requested target module is not locked.
 *
 *END**************************************************************************/
void TRGMUX_SetTrigSourceForTargetModule(TRGMUX_Type * const base,
                                         const trgmux_trigger_source_t triggerSource,
                                         const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);

    uint32_t tmpReg;
    /* Read value of entire TRGMUX register in a temp variable */
    tmpReg = base->TRGMUXn[TRGMUX_IDX_REG(targetModule)];
    /* Clear first the SEL bitfield inside the TRGMUX register */
    tmpReg &= ~((uint32_t)TRGMUX_TRGMUXn_SEL0_MASK << (TRGMUX_TRGMUXn_SEL1_SHIFT * TRGMUX_IDX_SEL_BITFIELD_REG(targetModule)));
    /* Configure the SEL bitfield to the desired value */
    tmpReg |=  ((uint32_t)triggerSource) << ((uint8_t)(TRGMUX_TRGMUXn_SEL1_SHIFT * TRGMUX_IDX_SEL_BITFIELD_REG(targetModule)));
    /* Write back the TRGMUX register */
    base->TRGMUXn[TRGMUX_IDX_REG(targetModule)] = tmpReg;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_GetTrigSourceForTargetModule
 * Description   : This function returns the TRGMUX source trigger linked to
 * a selected target module.
 *
 *END**************************************************************************/
trgmux_trigger_source_t TRGMUX_GetTrigSourceForTargetModule(const TRGMUX_Type * const base,
                                                            const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);

    uint32_t trigSource;
    /* Perform the update operation */
    trigSource =
        ((base->TRGMUXn[TRGMUX_IDX_REG(targetModule)] >> (TRGMUX_TRGMUXn_SEL1_SHIFT * TRGMUX_IDX_SEL_BITFIELD_REG(targetModule))) & TRGMUX_TRGMUXn_SEL0_MASK);

    return (trgmux_trigger_source_t)(trigSource);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_SetLockForTargetModule
 * Description   : This function sets the LK bit of the TRGMUX register corresponding
 * to the selected target module.
 *
 *END**************************************************************************/
void TRGMUX_SetLockForTargetModule(TRGMUX_Type * const base,
                                   const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);

    /* Perform the update operation */
    base->TRGMUXn[TRGMUX_IDX_REG(targetModule)] |= (((uint32_t)1U) << TRGMUX_TRGMUXn_LK_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_GetLockForTargetModule
 * Description   : Get the Lock bit status of the TRGMUX register of a target module.
 *
 *END**************************************************************************/
bool TRGMUX_GetLockForTargetModule(const TRGMUX_Type * const base,
                                   const trgmux_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);

    uint32_t lockVal;
    bool lock;

    /* Get the lock bit value */
    lockVal = ((base->TRGMUXn[TRGMUX_IDX_REG(targetModule)] & TRGMUX_TRGMUXn_LK_MASK) >> TRGMUX_TRGMUXn_LK_SHIFT);

    lock = (lockVal == 0U) ? false : true;

    return lock;
}

/*******************************************************************************
* EOF
*******************************************************************************/
