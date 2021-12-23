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

#ifndef TRGMUX_HW_ACCESS_H
#define TRGMUX_HW_ACCESS_H

/*! @file trgmux_hw_access.h */

#include "trgmux_driver.h"

/*!
 * trgmux_hw_access TRGMUX Hardware Access
 * @details This section describes the programming interface of the TRGMUX Hardware Access.
 * @{
 */

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
* API
*******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Restore the TRGMUX module to reset value.
 *
 * This function restores the TRGMUX module to reset value.
 *
 * @param[in] base   The TRGMUX peripheral base address
 * @return           Execution status:
 *   STATUS_SUCCESS
 *   STATUS_ERROR    If at least one of the target module register is locked.
 */
status_t TRGMUX_Init(TRGMUX_Type * const base);

/*!
 * @brief Configures a source trigger for a target module.
 *
 * This function configures a TRGMUX link between a source trigger and a target module,
 * if the requested target module is not locked.
 *
 * @param[in] base           The TRGMUX peripheral base address
 * @param[in] triggerSource  One of the values in the trgmux_trigger_source_t enumeration
 * @param[in] targetModule   One of the values in the trgmux_target_module_t enumeration
 */
void TRGMUX_SetTrigSourceForTargetModule(TRGMUX_Type * const base,
                                         const trgmux_trigger_source_t triggerSource,
                                         const trgmux_target_module_t targetModule);

/*!
 * @brief Get the source trigger configured for a target module.
 *
 * This function returns the TRGMUX source trigger linked to a selected target module.
 *
 * @param[in] base         The TRGMUX peripheral base address
 * @param[in] targetModule One of the values in the trgmux_target_module_t enumeration
 * @return                 Enum value corresponding to the trigger source configured for the given target module
 */
trgmux_trigger_source_t TRGMUX_GetTrigSourceForTargetModule(const TRGMUX_Type * const base,
                                                            const trgmux_target_module_t targetModule);

/*!
 * @brief Lock the TRGMUX register of a target module.
 *
 * This function sets the LK bit of the TRGMUX register corresponding to
 * the selected target module. Please note that some TRGMUX registers can contain up to 4
 * SEL bitfields, meaning that these registers can be used to configure up to 4 target
 * modules independently. Because the LK bit is only one per register, the configuration
 * of all target modules referred from that register will be locked.
 *
 * @param[in] base         The TRGMUX peripheral base address
 * @param[in] targetModule One of the values in the trgmux_target_module_t enumeration
 */
void TRGMUX_SetLockForTargetModule(TRGMUX_Type * const base,
                                   const trgmux_target_module_t targetModule);

/*!
 * @brief Get the Lock bit status of the TRGMUX register of a target module.
 *
 * This function gets the value of the LK bit from the TRGMUX register corresponding to
 * the selected target module.
 *
 * @param[in] base         The TRGMUX peripheral base address
 * @param[in] targetModule One of the values in the trgmux_target_module_t enumeration
 * @return                 true or false depending on the state of the LK bit
 */
bool TRGMUX_GetLockForTargetModule(const TRGMUX_Type * const base,
                                   const trgmux_target_module_t targetModule);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* TRGMUX_HW_ACCESS_H */
/*******************************************************************************
* EOF
*******************************************************************************/
