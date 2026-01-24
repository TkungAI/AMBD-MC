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
 * @file ftm_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include "ftm_hw_access.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitModule
 * Description   : Initializes the FTM. This function will enable the flexTimer module
 * and selects one pre-scale factor for the FTM clock source.
 *END**************************************************************************/
void FTM_DRV_InitModule(FTM_Type * const ftmBase,
                        ftm_clock_ps_t ftmClockPrescaler)
{
    /* Use FTM mode */
    FTM_DRV_Enable(ftmBase, true);
    FTM_DRV_SetClockPs(ftmBase, ftmClockPrescaler);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Reset
 * Description   : Resets the FTM registers. All the register use in the driver should be
 * reset to default value of each register.
 *END**************************************************************************/
void FTM_DRV_Reset(FTM_Type * const ftmBase)
{
    uint8_t chnIndex;

    /* WPDIS is set when WPEN bit is read as a 1 and then 1 is written to WPDIS */
    ((ftmBase)->FMS) &= 0U;
    /* This is the reset value for MODE register. WPDIS bit is set to disable write protection */
    ((ftmBase)->MODE) = 0x00000004U;
    ((ftmBase)->SC) &= 0U;
    ((ftmBase)->CNT) = 0U;
    ((ftmBase)->MOD) = 0U;
    ((ftmBase)->CNTIN) = 0U;
    ((ftmBase)->STATUS) &= 0U;
    ((ftmBase)->SYNC) = 0U;
    ((ftmBase)->OUTINIT) = 0U;
    ((ftmBase)->OUTMASK) = 0U;
    ((ftmBase)->COMBINE) = 0U;
    ((ftmBase)->DEADTIME) = 0U;
    ((ftmBase)->EXTTRIG) &= 0U;
    ((ftmBase)->POL) = 0U;
    ((ftmBase)->FILTER) = 0U;
    ((ftmBase)->FLTCTRL) = 0U;
    ((ftmBase)->QDCTRL) = 0U;
    ((ftmBase)->CONF) = 0U;
    ((ftmBase)->FLTPOL) = 0U;
    ((ftmBase)->SYNCONF) = 0U;
    ((ftmBase)->INVCTRL) = 0U;
    ((ftmBase)->SWOCTRL) = 0U;
    ((ftmBase)->PWMLOAD) = 0U;
    ((ftmBase)->HCR) = 0U;
    #if FEATURE_FTM_HAS_SUPPORTED_DITHERING
        #if (FTM_INSTANCE_COUNT > 2U)
        if ((ftmBase == FTM1) || (ftmBase == FTM2))
        #else
        if (ftmBase == FTM1)
        #endif
        {
            ((ftmBase)->MOD_MIRROR) = 0U;
        }
    #endif
    /* Set to reset value all CnV and CnSC registers */
    for (chnIndex = 0; chnIndex < FEATURE_FTM_CHANNEL_COUNT; chnIndex++)
    {
        ((ftmBase)->CONTROLS[chnIndex].CnSC) &= 0U;
        ((ftmBase)->CONTROLS[chnIndex].CnV) = 0U;
        #if FEATURE_FTM_HAS_SUPPORTED_DITHERING
            #if (FTM_INSTANCE_COUNT > 2U)
            if ((ftmBase == FTM1) || (ftmBase == FTM2))
            #else
            if (ftmBase == FTM1)
            #endif
            {
                ((ftmBase)->CV_MIRROR[chnIndex]) = 0U;
            }
        #endif
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetChnTriggerCmd
 * Description   : Enables or disables the generation of the FTM peripheral timer channel trigger.
 * Enables or disables the generation of the FTM peripheral timer channel trigger when the
 * FTM counter is equal to its initial value.
 *END**************************************************************************/
void FTM_DRV_SetChnTriggerCmd(FTM_Type * const ftmBase,
                              uint8_t channel,
                              bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint32_t value = 0U;

    if (channel < 2U)
    {
        value = (uint32_t)FTM_EXTTRIG_CH0TRIG_MASK << (uint32_t)channel;
    }
    else if (channel < 6U)
    {
        value = (uint32_t)FTM_EXTTRIG_CH2TRIG_MASK << ((uint32_t)(channel) - 2U);
    }
    else
    {
        value = (uint32_t)FTM_EXTTRIG_CH6TRIG_MASK << ((uint32_t)(channel) - 6U);
    }

    if (true == enable)
    {
        ((ftmBase)->EXTTRIG) |= value;
    }
    else
    {
        ((ftmBase)->EXTTRIG) &= ~value;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetChnInputCaptureFilter
 * Description   : Sets the FTM peripheral timer channel input capture filter value.
 *END**************************************************************************/
void FTM_DRV_SetChnInputCaptureFilter(FTM_Type * const ftmBase,
                                      uint8_t channel,
                                      uint8_t value)
{
    DEV_ASSERT(CHAN4_IDX > channel);

    switch (channel)
    {
        case CHAN0_IDX:
            FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH0FVAL_MASK, FTM_FILTER_CH0FVAL(value));
            break;
        case CHAN1_IDX:
            FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH1FVAL_MASK, FTM_FILTER_CH1FVAL(value));
            break;
        case CHAN2_IDX:
            FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH2FVAL_MASK, FTM_FILTER_CH2FVAL(value));
            break;
        case CHAN3_IDX:
            FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH3FVAL_MASK, FTM_FILTER_CH3FVAL(value));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
