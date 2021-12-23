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

#include "pdb_hw_access.h"

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_Init
 * Description   : Reset PDB's registers to a known state. This state is
 * defined in Reference Manual, which is power on reset value.
 *
 *END*************************************************************************/
void PDB_Init(PDB_Type * const base)
{
    DEV_ASSERT(base != NULL);
    uint32_t chn, preChn;

    base->SC    = 0U;
    PDB_Enable(base);
    base->MOD   = 0xFFFFU;
    base->IDLY  = 0xFFFFU;
    /* For ADC trigger. */
    for (chn = 0U; chn < PDB_CH_COUNT; chn++)
    {
        base->CH[chn].C1 = 0U;
        base->CH[chn].S = 0U;
        for (preChn = 0U; preChn < PDB_DLY_COUNT; preChn++)
        {
            PDB_SetAdcPreTriggerDelayValue(base, chn, preChn, 0U);
        }
    }

    /* For Pulse out trigger. */
    base->POEN = 0U;
    for (chn = 0U; chn < PDB_POnDLY_COUNT; chn++)
    {
        base->POnDLY[chn].PODLY = 0U;
    }

    /* Load the setting value. */
    PDB_SetLoadValuesCmd(base);
    PDB_Disable(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_ConfigTimer
 * Description   : Configure the PDB timer.
 *
 *END*************************************************************************/
void PDB_ConfigTimer(PDB_Type * const base,
                     const pdb_timer_config_t * configPtr)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(configPtr != NULL);
    uint32_t sc;

    sc = base->SC;
    sc &= ~((uint32_t)PDB_SC_LDMOD_MASK
          | (uint32_t)PDB_SC_PDBEIE_MASK
          | (uint32_t)PDB_SC_PRESCALER_MASK
          | (uint32_t)PDB_SC_TRGSEL_MASK
          | (uint32_t)PDB_SC_MULT_MASK
          | (uint32_t)PDB_SC_CONT_MASK
          | (uint32_t)PDB_SC_DMAEN_MASK
          | (uint32_t)PDB_SC_PDBIE_MASK);

    sc |= PDB_SC_LDMOD((uint32_t)(configPtr->loadValueMode));
    if (configPtr->seqErrIntEnable)
    {
        sc |= PDB_SC_PDBEIE_MASK;
    }
    sc |= PDB_SC_PRESCALER((uint32_t)(configPtr->clkPreDiv));
    sc |= PDB_SC_TRGSEL((uint32_t)(configPtr->triggerInput));
    sc |= PDB_SC_MULT((uint32_t)(configPtr->clkPreMultFactor));
    if (configPtr->continuousModeEnable)
    {
        sc |= PDB_SC_CONT_MASK;
    }
    if (configPtr->dmaEnable)
    {
        sc |= PDB_SC_DMAEN_MASK;
    }
    if (configPtr->intEnable)
    {
        sc |= PDB_SC_PDBIE_MASK;
    }

    base->SC = sc;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_SetAdcPreTriggerBackToBackEnable
 * Description   : Switch to enable pre-trigger's back to back mode.
 *
 *END*************************************************************************/
void PDB_SetAdcPreTriggerBackToBackEnable(PDB_Type * const base,
                                          uint32_t chn,
                                          uint32_t preChnMask,
                                          bool enable)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);

    uint32_t c1 = base->CH[chn].C1;
    if (enable)
    {
        c1 |= PDB_C1_BB(preChnMask);
    }
    else
    {
        c1 &= ~PDB_C1_BB(preChnMask);
    }

    base->CH[chn].C1 = c1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_SetAdcPreTriggerOutputEnable
 * Description   : Switch to enable pre-trigger's output.
 *
 *END*************************************************************************/
void PDB_SetAdcPreTriggerOutputEnable(PDB_Type * const base,
                                      uint32_t chn,
                                      uint32_t preChnMask,
                                      bool enable)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);

    uint32_t c1 = base->CH[chn].C1;
    if (enable)
    {
        c1 |= PDB_C1_TOS(preChnMask);
    }
    else
    {
        c1 &= ~PDB_C1_TOS(preChnMask);
    }

    base->CH[chn].C1 = c1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_SetAdcPreTriggerEnable
 * Description   : Switch to enable pre-trigger's.
 *
 *END*************************************************************************/
void PDB_SetAdcPreTriggerEnable(PDB_Type * const base,
                                uint32_t chn,
                                uint32_t preChnMask,
                                bool enable)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);

    uint32_t c1 = base->CH[chn].C1;
    if (enable)
    {
        c1 |= PDB_C1_EN(preChnMask);
    }
    else
    {
        c1 &= ~PDB_C1_EN(preChnMask);
    }

    base->CH[chn].C1 = c1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_ClearAdcPreTriggerFlags
 * Description   : Clear the flag that the PDB counter reaches to the
 * pre-trigger's delay value.
 *
 *END*************************************************************************/
void PDB_ClearAdcPreTriggerFlags(PDB_Type * const base,
                                 uint32_t chn,
                                 uint32_t preChnMask)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);

    /* Write 0 to clear. */
    uint32_t s = base->CH[chn].S;
    s &= ~PDB_S_CF(preChnMask); /* Update the change. */

    base->CH[chn].S = s;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_ClearAdcPreTriggerSeqErrFlags
 * Description   : Clear the flag that sequence error is detected.
 *
 *END*************************************************************************/
void PDB_ClearAdcPreTriggerSeqErrFlags(PDB_Type * const base,
                                       uint32_t chn,
                                       uint32_t preChnMask)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(chn < PDB_CH_COUNT);
    volatile uint32_t dummy_read;

    /* Write 0 to clear. */
    uint32_t s = base->CH[chn].S;
    s &= ~PDB_S_ERR(preChnMask);

    base->CH[chn].S = s;

    /* This read-after-write guarantees that the write to clear operation is completed,
     * for the case when memory write buffering is enabled. */
    dummy_read = base->CH[chn].S;
    (void)dummy_read;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_SetCmpPulseOutEnable
 * Description   : Switch to enable the pulse-out trigger.
 *
 *END*************************************************************************/
void PDB_SetCmpPulseOutEnable(PDB_Type * const base,
                              uint32_t pulseChnMask,
                              bool enable)
{
    DEV_ASSERT(base != NULL);
    uint32_t poen = base->POEN;
    if (enable)
    {
        poen |= PDB_POEN_POEN(pulseChnMask);
    }
    else
    {
        poen &= ~PDB_POEN_POEN(pulseChnMask);
    }

    base->POEN = poen;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
