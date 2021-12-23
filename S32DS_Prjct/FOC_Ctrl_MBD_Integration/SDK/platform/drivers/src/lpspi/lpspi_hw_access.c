/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
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

/*!
 * @lpspi_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.\
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower
 * or different essential type [MISRA 2012 Rule 10.3, required]
 * This is required by the conversion of a unsigned value of a bitfield/bit into a enum value.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially
 * unsigned' to 'essentially Boolean'.
 * This is required by the conversion of a bit into a bool.
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * This is required by the conversion of a bitfield/bit of a register into a enum.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 *(different essential type categories).
 * This is required by the conversion of a bit/bitfield of a register into boolean or a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 */

#include "lpspi_hw_access.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_Init
 * Description   : Resets the LPSPI internal logic and registers to their default settings.
 *
 * This function first performs a software reset of the LPSPI module which resets the
 * internal LPSPI logic and most registers, then proceeds to manually reset all of the
 * LPSPI registers to their default setting to ensuring these registers at programmed to
 * their default value which includes disabling the module.
 *
 *END**************************************************************************/
void LPSPI_Init(LPSPI_Type * base)
{
    /* Software reset the internal logic */
    base->CR = LPSPI_CR_RST_MASK;
    /* Now bring the LPSPI module out of reset and clear CR register */
    base->CR = (uint32_t)0x0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_Disable
 * Description   : Disables the LPSPI module.
 *
 * Note that this function returns STATUS_BUSY if it is detected that the Module Busy Flag
 * (MBF) is set, otherwise, if success, it returns STATUS_SUCCESS.
 *
 *END**************************************************************************/
status_t LPSPI_Disable(LPSPI_Type * base)
{
    uint32_t lpspi_tmp = base->SR;
    lpspi_tmp = (lpspi_tmp & LPSPI_SR_MBF_MASK) >> LPSPI_SR_MBF_SHIFT;

    if (lpspi_tmp == (uint32_t)1)
    {
        return STATUS_BUSY;
    }
    else
    {
        base->CR = base->CR & (~(LPSPI_CR_MEN_MASK));
        return STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_GetVersionId
 * Description   : Configures the LPSPI for master or slave.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
status_t LPSPI_SetMasterSlaveMode(LPSPI_Type * base, lpspi_master_slave_mode_t mode)
{
    base->CFGR1 = (base->CFGR1 & (~LPSPI_CFGR1_MASTER_MASK)) | ((uint32_t)mode << LPSPI_CFGR1_MASTER_SHIFT);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetPinConfigMode
 * Description   : Flushes the LPSPI FIFOs.
 *
 *END**************************************************************************/
void LPSPI_SetFlushFifoCmd(LPSPI_Type * base, bool flushTxFifo, bool flushRxFifo)
{
    uint32_t crValue = 0;

    crValue = ((uint32_t)flushTxFifo << LPSPI_CR_RTF_SHIFT) |
              ((uint32_t)flushRxFifo << LPSPI_CR_RRF_SHIFT);

    base->CR |= crValue;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_ClearStatusFlag
 * Description   : Clears the LPSPI status flag.
 *
 * This function clears the state of one of the LPSPI status flags as requested by
 * the user. Note, the flag must be w1c capable, if not the function returns an error.
 * w1c capable flags are:
 *   LPSPI_WORD_COMPLETE
 *   LPSPI_FRAME_COMPLETE
 *   LPSPI_TRANSFER_COMPLETE
 *   LPSPI_TRANSMIT_ERROR
 *   LPSPI_RECEIVE_ERROR
 *   LPSPI_DATA_MATCH
 *
 *END**************************************************************************/
status_t LPSPI_ClearStatusFlag(LPSPI_Type * base, lpspi_status_flag_t statusFlag)
{
    if (statusFlag == LPSPI_ALL_STATUS)
    {
        base->SR |= (uint32_t)LPSPI_ALL_STATUS;
    }
    else
    {
        base->SR |= ((uint32_t)1U << (uint32_t)statusFlag);
    }
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetPcsPolarityMode
 * Description   : Configures the desired LPSPI PCS polarity.
 *
 * This function allows the user to configure the polarity of a particular PCS signal.
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
status_t LPSPI_SetPcsPolarityMode(LPSPI_Type * base, lpspi_which_pcs_t whichPcs,
                                            lpspi_signal_polarity_t pcsPolarity)
{
    uint32_t cfgr1Value = 0;

    /* Clear the PCS polarity bit */
    cfgr1Value = (base->CFGR1) & (~((uint32_t)1U << (LPSPI_CFGR1_PCSPOL_SHIFT + (uint32_t)whichPcs)));

    /* Configure the PCS polarity bit according to the pcsPolarity setting */
    cfgr1Value |= (uint32_t)pcsPolarity << (LPSPI_CFGR1_PCSPOL_SHIFT + (uint32_t)whichPcs);

    base->CFGR1 = cfgr1Value;

    return STATUS_SUCCESS;
    
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetPinConfigMode
 * Description   : Configures the LPSPI SDO/SDI pin configuration mode.
 *
 * This function configures the pin mode of the LPSPI.
 * For the SDI and SDO pins, the user can configure these pins as follows:
 *  SDI is used for input data and SDO for output data.
 *  SDO is used for input data and SDO for output data.
 *  SDI is used for input data and SDI for output data.
 *  SDO is used for input data and SDI for output data.
 *
 * The user has the option to configure the output data as:
 *  Output data retains last value when chip select is de-asserted (default setting).
 *  Output data is tristated when chip select is de-asserted.
 *
 * Finally, the user has the option to configure the PCS[3:2] pins as:
 *  Enabled for PCS operation (default setting).
 *  Disabled - this is need if the user wishes to configure the LPSPI mode for 4-bit transfers
 *             where these pins will be used as I/O data pins.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
status_t LPSPI_SetPinConfigMode(LPSPI_Type * base,
                                          lpspi_pin_config_t pinCfg,
                                          lpspi_data_out_config_t dataOutConfig,
                                          bool pcs3and2Enable)
{
    uint32_t cfgr1Value = 0;

    cfgr1Value = base->CFGR1 &
                 ~(LPSPI_CFGR1_PINCFG_MASK|LPSPI_CFGR1_OUTCFG_MASK|LPSPI_CFGR1_PCSCFG_MASK);

    cfgr1Value |= ((uint32_t)(pinCfg) << LPSPI_CFGR1_PINCFG_SHIFT) |
                  ((uint32_t)(dataOutConfig) << LPSPI_CFGR1_OUTCFG_SHIFT) |
                  ((uint32_t)(!pcs3and2Enable) << LPSPI_CFGR1_PCSCFG_SHIFT);  /* enable = 0 */

    base->CFGR1 = cfgr1Value;
	
	return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : abs_dif
 * Description   : This is a helper function which implements absolute difference between
 * two numbers.
 *
 *END**************************************************************************/
static uint32_t abs_dif(uint32_t a, uint32_t b)
{
    if (a>b)
    {
        return(a-b);
    }
    else
    {
        return(b-a);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetBaudRate
 * Description   : Sets the LPSPI baud rate in bits per second.
 *
 * This function takes in the desired bitsPerSec (baud rate) and calculates the nearest
 * possible baud rate, and returns the calculated baud rate in bits-per-second. It requires
 * that the caller also provide the frequency of the module source clock (in Hertz).
 * Also note that the baud rate does not take into affect until the Transmit Control
 * Register (TCR) is programmed with the PRESCALE value. Hence, this function returns the
 * PRESCALE tcrPrescaleValue parameter for later programming in the TCR.  It is up to the
 * higher level peripheral driver to alert the user of an out of range baud rate input.
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 *END**************************************************************************/
uint32_t LPSPI_SetBaudRate(LPSPI_Type * base, uint32_t bitsPerSec,
                               uint32_t sourceClockInHz, uint32_t * tcrPrescaleValue)
{

    uint32_t bestFreq = 0xFFFFFFFFU;
    uint32_t bestScaler = 0U;
    uint32_t bestPrescaler = 0U;
    uint32_t freq1 = 0U;
    uint32_t freq2 = 0U;
    uint8_t scaler = 0U;
    uint8_t prescaler = 0U;
    uint32_t low, high;
    uint32_t tempBestFreq = 0U;
    uint32_t tempBestScaler = 0U;

    for (prescaler = 0; prescaler < 8U; prescaler++)
    {
        low = 0U;
        high = 256U;

        /* Implement golden section search algorithm */
        do
        {
            scaler = (uint8_t)((low + high) / 2U);
            freq1 = sourceClockInHz / (s_baudratePrescaler[prescaler] * (scaler + (uint32_t)2U));

            if (abs_dif(bitsPerSec, bestFreq) > abs_dif(bitsPerSec, freq1))
            {
                bestFreq = freq1;
            }
            if (freq1 < bitsPerSec)
            {
                high = scaler;
            }
            else
            {
                low = scaler;
            }
        }
        while((high - low) > 1U);

        /* Evaluate last 2 scaler values */
        freq1 = sourceClockInHz / (s_baudratePrescaler[prescaler] * (low + (uint32_t)2U));
        freq2 = sourceClockInHz / (s_baudratePrescaler[prescaler] * (high + (uint32_t)2U));

        if (abs_dif(bitsPerSec, freq1) > abs_dif(bitsPerSec, freq2))
        {
            tempBestFreq = freq2;
            tempBestScaler = high;
        }
        else
        {
            tempBestFreq = freq1;
            tempBestScaler = low;
        }

        if (abs_dif(bitsPerSec, bestFreq) >= abs_dif(bitsPerSec, tempBestFreq))
        {
            bestFreq = tempBestFreq;
            bestScaler = tempBestScaler;
            bestPrescaler = prescaler;
        }

        /* If current frequency is equal to target frequency  stop the search */
        if(bestFreq == bitsPerSec)
        {
            break;
        }
    }

    /* Add default values for delay between transfers, delay between sck to pcs and between pcs to sck. */
    (void)LPSPI_SetDelay(base, LPSPI_SCK_TO_PCS, bestScaler >> 2U);
    (void)LPSPI_SetDelay(base, LPSPI_PCS_TO_SCK, bestScaler >> 2U);
    (void)LPSPI_SetDelay(base, LPSPI_BETWEEN_TRANSFER, bestScaler >> 2U);

    /* Write the best baud rate scalar to the CCR.
     * Note, no need to check for error since we've already checked to make sure the module is
     * disabled and in master mode. Also, there is a limit on the maximum divider so we will not
     * exceed this.
     */
    (void)LPSPI_SetBaudRateDivisor(base, bestScaler);

    /* return the best prescaler value for user to use later */
    *tcrPrescaleValue = bestPrescaler;

    /* return the actual calculated baud rate */
    return bestFreq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetBaudRateDivisor
 * Description   : Configures the baud rate divisor manually (only the LPSPI_CCR[SCKDIV]).
 *
 * This function allows the caller to manually set the baud rate divisor in the event
 * that this divider is known and the caller does not wish to call the
 * LPSPI_SetBaudRate function. Note that this only affects the LPSPI_CCR[SCKDIV]).
 * The Transmit Control Register (TCR) is programmed separately with the PRESCALE value.
 * The valid range is 0x00 to 0xFF (255), if the user inputs outside of this range, an error
 * is returned.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 *END**************************************************************************/
status_t LPSPI_SetBaudRateDivisor(LPSPI_Type * base, uint32_t divisor)
{
    uint32_t lpspi_tmp;
	
    lpspi_tmp = base->CCR;
    lpspi_tmp &= ~(LPSPI_CCR_SCKDIV_MASK);
    lpspi_tmp |= LPSPI_CCR_SCKDIV(divisor);
    base->CCR = lpspi_tmp;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetTxCommandReg
 * Description   : Sets the Transmit Command Register (TCR) parameters.
 *
 * The Transmit Command Register (TCR) contains multiple parameters that affect
 * the transmission of data, such as clock phase and polarity, which PCS to use,
 * whether or not the PCS remains asserted at the completion of a frame, etc.
 * Any writes to this register results in an immediate push of the entire register
 * and its contents to the TX FIFO.  Hence, writes to this register should include
 * all of the desired parameters written to the register at once. Hence, the user
 * should fill in the members of the lpspi_tx_cmd_config_t data structure and pass
 * this to the function.
 *
 *END**************************************************************************/
void LPSPI_SetTxCommandReg(LPSPI_Type * base, const lpspi_tx_cmd_config_t * txCmdCfgSet)
{
     base->TCR = (((uint32_t)txCmdCfgSet->clkPolarity << LPSPI_TCR_CPOL_SHIFT) |
                         ((uint32_t)txCmdCfgSet->clkPhase << LPSPI_TCR_CPHA_SHIFT) |
                         ((uint32_t)txCmdCfgSet->preDiv << LPSPI_TCR_PRESCALE_SHIFT) |
                         ((uint32_t)txCmdCfgSet->whichPcs << LPSPI_TCR_PCS_SHIFT) |
                         ((uint32_t)txCmdCfgSet->lsbFirst << LPSPI_TCR_LSBF_SHIFT) |
                         ((uint32_t)txCmdCfgSet->byteSwap<< LPSPI_TCR_BYSW_SHIFT) |
                         ((uint32_t)txCmdCfgSet->contTransfer << LPSPI_TCR_CONT_SHIFT) |
                         ((uint32_t)txCmdCfgSet->contCmd << LPSPI_TCR_CONTC_SHIFT) |
                         ((uint32_t)txCmdCfgSet->rxMask << LPSPI_TCR_RXMSK_SHIFT) |
                         ((uint32_t)txCmdCfgSet->txMask << LPSPI_TCR_TXMSK_SHIFT) |
                         ((uint32_t)txCmdCfgSet->width << LPSPI_TCR_WIDTH_SHIFT) |
                         ((uint32_t)(txCmdCfgSet->frameSize - 1UL) << LPSPI_TCR_FRAMESZ_SHIFT));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_SetPcs
 * Description   : Sets the PCS flag to a value between 0 and 3.
 *
 * This function modifies the TCR register and sets the value of the PCS flag
 * to the value of the whichPcs parameter.
 *
 *END**************************************************************************/
void LPSPI_SetPcs(LPSPI_Type * base, lpspi_which_pcs_t whichPcs)
{
    uint32_t regVal;

    regVal = base->TCR;
    regVal &= (uint32_t)(~(LPSPI_TCR_PCS_MASK));
    regVal |= (uint32_t)((uint32_t)whichPcs << LPSPI_TCR_PCS_SHIFT);
    base->TCR = regVal;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
