/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
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
  * @lpspi_irq.c
  *
  * @page misra_violations MISRA-C:2012 violations
  *
  * @section [global]
  * Violates MISRA 2012 Required Rule 5.1, Object/function previously declared.
  * This requirement is fulfilled since the function is declared as external in and only in 
  * one configuration C file.
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
  * @section [global]
  * Violates MISRA 2012 Required Rule 8.4, A compatible declaration shall be
  * visible when an object or function with external linkage is defined.
  *
  * @section [global]
  * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
  * Function is defined for usage by IRQs handlers and can't be defined as static
  *  
  */
 
#include <assert.h>
#include <stdbool.h>
#include "device_registers.h"
#include "lpspi_shared_function.h"


/*!
 * @addtogroup lpspi_driver Low Power Serial Peripheral Interface (LPSPI)
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

#if (LPSPI_INSTANCE_COUNT == 1U)
/*!
 * @brief This function is the implementation of LPSPI0 handler named in startup code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void LPSPI0_IRQHandler(void);
 
void LPSPI0_IRQHandler(void)
{
    LPSPI_DRV_IRQHandler(0U);
}

#elif (LPSPI_INSTANCE_COUNT == 2U)
/*!
 * @brief This function is the implementation of LPSPI0 handler named in startup code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void LPSPI0_IRQHandler(void);
 
void LPSPI0_IRQHandler(void)
{
    LPSPI_DRV_IRQHandler(0U);
}

/*!
 * @brief This function is the implementation of LPSPI1 handler named in startup code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void LPSPI1_IRQHandler(void); 
 
void LPSPI1_IRQHandler(void)
{
    LPSPI_DRV_IRQHandler(1U);
}

#else

/*!
 * @brief This function is the implementation of LPSPI0 handler named in startup code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void LPSPI0_IRQHandler(void);
 
void LPSPI0_IRQHandler(void)
{
    LPSPI_DRV_IRQHandler(0U);
}

/*!
 * @brief This function is the implementation of LPSPI1 handler named in startup code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void LPSPI1_IRQHandler(void);
 
void LPSPI1_IRQHandler(void)
{
    LPSPI_DRV_IRQHandler(1U);
}

/*!
 * @brief This function is the implementation of LPSPI2 handler named in startup code.
 *
 * It passes the instance to the shared LPSPI IRQ handler.
 */
void LPSPI2_IRQHandler(void);
 
void LPSPI2_IRQHandler(void)
{
    LPSPI_DRV_IRQHandler(2U);
}

#endif

/*! @} */

/*******************************************************************************
 * EOF
 ******************************************************************************/

