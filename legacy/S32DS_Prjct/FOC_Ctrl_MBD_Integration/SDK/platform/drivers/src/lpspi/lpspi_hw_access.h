/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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
 * @lpspi_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * This increases ease of use: allows users to access the corresponding field in the register
 * using an already defined type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable 's_baudratePrescaler' at block scope
 * The variable is used in LPSPI drivers so it must remain global.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower
 * or different essential type [MISRA 2012 Rule 10.3, required]
 * This is required by the conversion of a unsigned value of a bitfield/bit into a enum value.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially
 * unsigned' to 'essentially Boolean'. This is required by the conversion of a bit into a bool.
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * This is required by the conversion of a bitfiel of a register into a enum.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 *(different essential type categories).
 * This is required by the conversion of a bit/bitfield of a register into boolean or a enum type.
 */

#ifndef LPSPI_HW_ACCESS_H
#define LPSPI_HW_ACCESS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "status.h"
#include "device_registers.h"
#include "lpspi_shared_function.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! @brief Prescaler values for LPSPI clock source.
 */
typedef enum
{
    LPSPI_DIV_1     = 0U,
    LPSPI_DIV_2     = 1U,
    LPSPI_DIV_4     = 2U,
    LPSPI_DIV_8     = 3U,
    LPSPI_DIV_16    = 4U,
    LPSPI_DIV_32    = 5U,
    LPSPI_DIV_64    = 6U,
    LPSPI_DIV_128   = 7U,
} lpspi_prescaler_t;

/*! @brief LPSPI status flags.
 */
typedef enum
{
    LPSPI_TX_DATA_FLAG      = LPSPI_SR_TDF_SHIFT, /*!< TX data flag */
    LPSPI_RX_DATA_FLAG      = LPSPI_SR_RDF_SHIFT, /*!< RX data flag */
    LPSPI_WORD_COMPLETE     = LPSPI_SR_WCF_SHIFT, /*!< Word Complete flag */
    LPSPI_FRAME_COMPLETE    = LPSPI_SR_FCF_SHIFT, /*!< Frame Complete flag */
    LPSPI_TRANSFER_COMPLETE = LPSPI_SR_TCF_SHIFT, /*!< Transfer Complete flag */
    LPSPI_TRANSMIT_ERROR    = LPSPI_SR_TEF_SHIFT, /*!< Transmit Error flag (FIFO underrun) */
    LPSPI_RECEIVE_ERROR     = LPSPI_SR_REF_SHIFT, /*!< Receive Error flag (FIFO overrun) */
    LPSPI_DATA_MATCH        = LPSPI_SR_DMF_SHIFT, /*!< Data Match flag */
    LPSPI_MODULE_BUSY       = LPSPI_SR_MBF_SHIFT, /*!< Module Busy flag */
    LPSPI_ALL_STATUS        = 0x00003F00U         /*!< Used for clearing all w1c status flags */
} lpspi_status_flag_t;

/*! @brief LPSPI master or slave configuration.
 */
typedef enum
{
    LPSPI_MASTER = 1U,     /*!< LPSPI peripheral operates in master mode. */
    LPSPI_SLAVE  = 0U      /*!< LPSPI peripheral operates in slave mode. */
} lpspi_master_slave_mode_t;

/*! @brief LPSPI pin (SDO and SDI) configuration.
 */
typedef enum
{
    LPSPI_SDI_IN_SDO_OUT = 0U,     /*!< LPSPI SDI input, SDO output. */
    LPSPI_SDI_IN_OUT     = 1U,     /*!< SDI is used for both input and output data. */
    LPSPI_SDO_IN_OUT     = 2U,     /*!< SDO is used for both input and output data. */
    LPSPI_SDI_OUT_SDO_IN = 3U      /*!< LPSPI SDO input, SDI output. */
} lpspi_pin_config_t;

/*! @brief LPSPI data output configuration.
 */
typedef enum
{
    LPSPI_DATA_OUT_RETAINED = 0U, /*!< Data out retains last value when chip select de-asserted */
    LPSPI_DATA_OUT_TRISTATE = 1U  /*!< Data out is tri-stated when chip select de-asserted */
} lpspi_data_out_config_t;

/*! @brief LPSPI transfer width configuration.
 */
typedef enum
{
    LPSPI_SINGLE_BIT_XFER = 0U, /*!< 1-bit shift at a time, data out on SDO, in on SDI (normal mode) */
    LPSPI_TWO_BIT_XFER = 1U,    /*!< 2-bits shift out on SDO/SDI and in on SDO/SDI */
    LPSPI_FOUR_BIT_XFER = 2U    /*!< 4-bits shift out on SDO/SDI/PCS[3:2] and in on SDO/SDI/PCS[3:2] */
} lpspi_transfer_width_t;

/*! @brief LPSPI Transmit Command Register configuration structure.
 *
 * This structure contains the Transmit Command Register (TCR) settings. Any writes
 * to the TCR will cause the entire TCR contents to be pushed to the TX FIFO.
 * Therefore any updates to the TCR should include updates to all of the register
 * bit fields to form a 32-bit write to the TCR.
 */
typedef struct
{
    uint32_t frameSize;              /*!< Number of bits/frame, minimum is 8-bits. */
    lpspi_transfer_width_t width;    /*!< Transfer width, single, 2-bit, or 4-bit transfer. */
    bool txMask;                     /*!< Option to mask the transmit data (won't load to FIFO). */
    bool rxMask;                     /*!< Option to mask the receive data (won't store in FIFO). */
    bool contCmd;                    /*!< Master option to change cmd word within cont transfer. */
    bool contTransfer;               /*!< Master option for continuous transfer. */
    bool byteSwap;                   /*!< Option to invoke the byte swap option in the FIFOs. */
    bool lsbFirst;                   /*!< Option to transmit LSB first. */
    lpspi_which_pcs_t whichPcs;      /*!< Selects which PCS to use. */
    uint32_t preDiv;                /*!< Selects the baud rate prescaler divider TCR bit setting. */
    lpspi_clock_phase_t clkPhase;    /*!< Selects which phase of clock to capture data. */
    lpspi_sck_polarity_t clkPolarity; /*!< Selects clock polarity. */
} lpspi_tx_cmd_config_t;

/*! @brief LPSPI initialization configuration structure.
 *
 * This structure contains parameters for the user to fill in to configure the LPSPI.
 * The user passes this structure into the LPSPI init function to configure it to
 * their desired parameters.
 * Example user code for:
    - 60MHz assumed, check ref manual for exact value
    - baudrate 500KHz
    - master mode
    - PCS is active low
   @code
    lpspi_init_config_t lpspiCfg;
    lpspiCfg.lpspiSrcClk = 60000000;
    lpspiCfg.baudRate = 500000;
    lpspiCfg.lpspiMode = LPSPI_MASTER;
    lpspiCfg.pcsPol = LPSPI_ACTIVE_LOW;
   @endcode
 */
typedef struct
{
    uint32_t lpspiSrcClk;                /*!< LPSPI module clock */
    uint32_t baudRate;                   /*!< LPSPI baudrate */
    lpspi_master_slave_mode_t lpspiMode; /*!< LPSPI master/slave mode */
    lpspi_signal_polarity_t pcsPol;      /*!< LPSPI PCS polarity */
} lpspi_init_config_t;

/*! @brief LPSPI delay type selection
 */
typedef enum
{
    LPSPI_SCK_TO_PCS = LPSPI_CCR_SCKPCS_SHIFT,     /*!< SCK to PCS Delay */
    LPSPI_PCS_TO_SCK = LPSPI_CCR_PCSSCK_SHIFT,     /*!< PCS to SCK Delay */
    LPSPI_BETWEEN_TRANSFER = LPSPI_CCR_DBT_SHIFT  /*!< Delay between transfers */
} lpspi_delay_type_t ;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Defines constant value arrays for the baud rate pre-scalar values.*/
static const uint32_t s_baudratePrescaler[] = { 1, 2, 4, 8, 16, 32, 64, 128 };

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Resets the LPSPI internal logic and registers to their default settings.
 *
 * This function first performs a software reset of the LPSPI module which resets the
 * internal LPSPI logic and most registers, then proceeds to manually reset all of the
 * LPSPI registers to their default setting to ensuring these registers at programmed to
 * their default value which includes disabling the module.
 *
 * @param Module base pointer of type LPSPI_Type.
 */
void LPSPI_Init(LPSPI_Type * base);

/*!
 * @brief Enables the LPSPI module.
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_Enable(LPSPI_Type * base)
{
    (base->CR) |= (uint32_t)1U << LPSPI_CR_MEN_SHIFT;
}

/*!
 * @brief Check if LPSPI module is enabled
 *
 * @param Module base pointer of type LPSPI_Type.
 */
static inline bool LPSPI_IsModuleEnabled(const LPSPI_Type * base)
{
    return (bool)(((base->CR) & LPSPI_CR_MEN_MASK) >> LPSPI_CR_MEN_SHIFT);
}

/*!
 * @brief Disables the LPSPI module.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @return This function returns STATUS_BUSY if it is detected that the Module Busy Flag
 *         (MBF) is set, otherwise, if success, it returns STATUS_SUCCESS.
 */
status_t LPSPI_Disable(LPSPI_Type * base);

/*!
 * @brief Configures the LPSPI for master or slave.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param mode Mode setting (master or slave) of type lpspi_master_slave_mode_t
 * @return This function returns the error condition STATUS_ERROR if the module is not
 *         disabled else it returns STATUS_SUCCESS.
 */
status_t LPSPI_SetMasterSlaveMode(LPSPI_Type * base, lpspi_master_slave_mode_t mode);

/*!
 * @brief Returns whether the LPSPI module is in master mode.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @return Returns true if LPSPI in master mode or false if in slave mode.
 */
static inline bool LPSPI_IsMaster(const LPSPI_Type * base)
{
    return (bool)((base->CFGR1 >> LPSPI_CFGR1_MASTER_SHIFT) & 1U);
}

/*!
 * @brief Gets FIFO sizes of the LPSPI module.
 *
 * @ param base Module base pointer of type LPSPI_Type.
 * @ param fifoSize The FIFO size passed back to the user
 */
static inline void LPSPI_GetFifoSizes(const LPSPI_Type * base, uint8_t * fifoSize)
{
    if (fifoSize != NULL)
    {
        *fifoSize = (uint8_t)(1U << ((base->PARAM & LPSPI_PARAM_TXFIFO_MASK) >> LPSPI_PARAM_TXFIFO_SHIFT));
    }
}

/*!
 * @brief Flushes the LPSPI FIFOs.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param flushTxFifo Flushes (true) the Tx FIFO, else do not flush (false) the Tx FIFO
 * @param flushRxFifo Flushes (true) the Rx FIFO, else do not flush (false) the Rx FIFO
 */
void LPSPI_SetFlushFifoCmd(LPSPI_Type * base, bool flushTxFifo, bool flushRxFifo);

/*!
 * @brief Sets the RX FIFO watermark values.
 *
 * This function allows the user to set the RX FIFO watermarks.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param rxWater The RX FIFO watermark value
 */
static inline void LPSPI_SetRxWatermarks(LPSPI_Type * base, uint32_t rxWater)
{
    uint32_t lpspi_tmp = base->FCR;
    lpspi_tmp &= ~(LPSPI_FCR_RXWATER_MASK);
    lpspi_tmp |= (rxWater << LPSPI_FCR_RXWATER_SHIFT);
    base->FCR = lpspi_tmp;
}

/*!
 * @brief Sets the TX FIFO watermark values.
 *
 * This function allows the user to set the TX FIFO watermarks.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param txWater The TX FIFO watermark value
 */
static inline void LPSPI_SetTxWatermarks(LPSPI_Type * base, uint32_t txWater)
{
    uint32_t lpspi_tmp = base->FCR;
    lpspi_tmp &= ~(LPSPI_FCR_TXWATER_MASK);
    lpspi_tmp |= (txWater << LPSPI_FCR_TXWATER_SHIFT);
    base->FCR = lpspi_tmp;
}

/*@}*/

/*!
 * @name Status flags and Interrupt configuration
 * @{
 */

/*!
 * @brief Gets the LPSPI status flag state.
 *
 * This function returns the state of one of the LPSPI status flags as requested by
 * the user.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param statusFlag The status flag, of type lpspi_status_flag_t
 * @return State of the status flag: asserted (true) or not-asserted (false)
 */
static inline bool LPSPI_GetStatusFlag(const LPSPI_Type * base,
                                           lpspi_status_flag_t statusFlag)
{
    return (bool)(((base->SR) >> (uint8_t)statusFlag) & 1U);
}

/*!
 * @brief Clears the LPSPI status flag.
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
 * @param base Module base pointer of type LPSPI_Type.
 * @param statusFlag The status flag, of type lpspi_status_flag_t
 * @return STATUS_SUCCESS or LPSPI_STATUS_INVALID_PARAMETER
 */
status_t LPSPI_ClearStatusFlag(LPSPI_Type * base, lpspi_status_flag_t statusFlag);

/*!
 * @brief Configures the LPSPI interrupts.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param interruptSrc The interrupt source, of type lpspi_status_flag_t
 * @param enable Enable (true) or disable (false) the interrupt source
 */
static inline void LPSPI_SetIntMode(LPSPI_Type * base,
                                        lpspi_status_flag_t interruptSrc, bool enable)
{
    if (enable == true)
    {
        base->IER |= (uint32_t)1U << (uint8_t)interruptSrc;
    }
    else
    {
        base->IER &= ~((uint32_t)1U << (uint8_t)interruptSrc);
    }
}

/*!
 * @brief Returns if the LPSPI interrupt request is enabled or disabled.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param interruptSrc The interrupt source, of type lpspi_status_flag_t
 * @return Returns if the interrupt source is enabled (true) or disabled (false)
 */
static inline bool LPSPI_GetIntMode(const LPSPI_Type * base,
                                        lpspi_status_flag_t interruptSrc)
{
    return (bool)(((base->IER) >> (uint8_t)interruptSrc) & 1U);
}

/*@}*/

/*!
 * @name DMA configuration
 * @{
 */

/*!
 * @brief Sets the LPSPI Transmit Data DMA configuration (enable or disable).
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param enable Enable (true) or disable (false) the TX DMA request
 */
static inline void LPSPI_SetTxDmaCmd(LPSPI_Type * base, bool enable)
{
    base->DER = (base->DER & (~LPSPI_DER_TDDE_MASK)) | ((uint32_t)enable << LPSPI_DER_TDDE_SHIFT);
}

/*!
 * @brief Sets the LPSPI Receive Data DMA configuration (enable or disable).
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param enable Enable (true) or disable (false) the RX DMA request
 */
static inline void LPSPI_SetRxDmaCmd(LPSPI_Type * base, bool enable)
{
    (base->DER) = (base->DER & (~LPSPI_DER_RDDE_MASK)) | ((uint32_t)enable << LPSPI_DER_RDDE_SHIFT);
}

/*!
 * @brief Manually configures a specific LPSPI delay parameter (module must be disabled to
 *        change the delay values).
 *
 * This function configures the:
 * SCK to PCS delay, or
 * PCS to SCK delay, or
 * Between transfer delay.
 *
 * These delay names are available in type lpspi_delay_type_t.
 *
 * The user passes which delay they want to configure along with the delay value.
 * This allows the user to directly set the delay values if they have
 * pre-calculated them or if they simply wish to manually increment the value.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param whichDelay The desired delay to configure, must be of type lpspi_delay_type_t
 * @param delay The 8-bit delay value 0x00 to 0xFF (255). The delay is equal to:
 *             -delay + 1 cycles of the LPSPI baud rate clock (SCK to PCS and PCS to SCK)
 *             -delay + 2 cycles of the LPSPI baud rate clock (Between transfer delay)
 * @return Either STATUS_SUCCESS, LPSPI_STATUS_OUT_OF_RANGE, or STATUS_ERROR if
 *         LPSPI is not disabled or if is not set for master mode.
 */
static inline status_t LPSPI_SetDelay(LPSPI_Type * base, lpspi_delay_type_t whichDelay, uint32_t delay)
{
    uint32_t ccrValue = 0;

    ccrValue = base->CCR & ~(0xFFUL << (uint32_t)whichDelay);
    ccrValue |= delay << (uint32_t)whichDelay;
    base->CCR = ccrValue;
    return STATUS_SUCCESS;
}

/*@}*/

/*!
 * @name SPI Bus Configuration
 * @{
 */

/*!
 * @brief Configures the desired LPSPI PCS polarity.
 *
 * This function allows the user to configure the polarity of a particular PCS signal.
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param whichPcs Select which PCS to program, of type lpspi_which_pcs_t
 * @param pcsPolarity Set PCS as active high or low, of type lpspi_signal_polarity_t
 * @return This function returns the error condition STATUS_ERROR if the module is not
 *         disabled else it returns STATUS_SUCCESS.
 */
status_t LPSPI_SetPcsPolarityMode(LPSPI_Type * base, lpspi_which_pcs_t whichPcs,
                                            lpspi_signal_polarity_t pcsPolarity);

/*!
 * @brief Configures the LPSPI SDO/SDI pin configuration mode.
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
 * @param base Module base pointer of type LPSPI_Type.
 * @param pinCfg Select configuration for the SDO/SDI pins (see lpspi_pin_config_t)
 * @param dataOutConfig Select data output config after chip select de-assertion
 * @param pcs3and2Enable Enable or disable PCS[3:2]
 * @return This function returns the error condition STATUS_ERROR if the module is not
 *         disabled else it returns STATUS_SUCCESS.
 */
status_t LPSPI_SetPinConfigMode(LPSPI_Type * base,
                                          lpspi_pin_config_t pinCfg,
                                          lpspi_data_out_config_t dataOutConfig,
                                          bool pcs3and2Enable);

/*!
 * @brief Sets the LPSPI baud rate in bits per second.
 *
 * This function takes in the desired bitsPerSec (baud rate) and calculates the nearest
 * possible baud rate without exceeding the desired baud rate, and returns the
 * calculated baud rate in bits-per-second. It requires that the caller also provide
 * the frequency of the module source clock (in Hertz). Also note that the baud rate
 * does not take into affect until the Transmit Control Register (TCR) is programmed
 * with the PRESCALE value. Hence, this function returns the PRESCALE tcrPrescaleValue
 * parameter for later programming in the TCR.  It is up to the higher level
 * peripheral driver to alert the user of an out of range baud rate input.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param bitsPerSec The desired baud rate in bits per second
 * @param sourceClockInHz Module source input clock in Hertz
 * @param tcrPrescaleValue The TCR PRESCALE value, needed by user to program the TCR
 * @return  The actual calculated baud rate. This function may also return a "0" if the
 *          LPSPI is not configued for master mode or if the LPSPI module is not disabled.
 */
uint32_t LPSPI_SetBaudRate(LPSPI_Type * base, uint32_t bitsPerSec,
                               uint32_t sourceClockInHz, uint32_t * tcrPrescaleValue);

/*!
 * @brief Configures the baud rate divisor manually (only the LPSPI_CCR[SCKDIV]).
 *
 * This function allows the caller to manually set the baud rate divisor in the event
 * that this divider is known and the caller does not wish to call the
 * LPSPI_SetBaudRate function. Note that this only affects the LPSPI_CCR[SCKDIV]).
 * The Transmit Control Register (TCR) is programmed separately with the PRESCALE value.
 * The valid range is 0x00 to 0xFF, if the user inputs outside of this range, an error
 * is returned.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param divisor Desired baud rate divisor setting (0x00 to 0xFF)
 * @return STATUS_SUCCESS or LPSPI_STATUS_OUT_OF_RANGE if divisor > 0xFF
 */
status_t LPSPI_SetBaudRateDivisor(LPSPI_Type * base, uint32_t divisor);

/*!
 * @brief Sets the PCS flag to the value of the whichPcs parameter.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param whichPcs Desired chip
 */
void LPSPI_SetPcs(LPSPI_Type * base, lpspi_which_pcs_t whichPcs);

/*@}*/

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Sets the Transmit Command Register (TCR) parameters.
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
 * @param base Module base pointer of type LPSPI_Type.
 * @param txCmdCfgSet Structure that contains the Transmit Command Register (TCR)
 *                    settings of type lpspi_tx_cmd_config_t
 */
void LPSPI_SetTxCommandReg(LPSPI_Type * base, const lpspi_tx_cmd_config_t * txCmdCfgSet);

/*!
 * @brief Writes data into the TX data buffer.
 *
 * This function writes data passed in by the user to the Transmit Data Register (TDR).
 * The user can pass up to 32-bits of data to load into the TDR. If the frame size exceeds 32-bits,
 * the user will have to manage sending the data one 32-bit word at a time.
 * Any writes to the TDR will result in an immediate push to the TX FIFO.
 * This function can be used for either master or slave mode.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param data The data word to be sent
 */
static inline void LPSPI_WriteData(LPSPI_Type * base, uint32_t data)
{
    base->TDR = data;
}

/*!
 * @brief Reads data from the data buffer.
 *
 * This function reads the data from the Receive Data Register (RDR).
 * This function can be used for either master or slave mode.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @return The data read from the data buffer
 */
static inline uint32_t LPSPI_ReadData(const LPSPI_Type * base)
{
    return (uint32_t)base->RDR;
}

/*!
 * @brief Reads TX COUNT form the FIFO Status Register.
 *
 * This function reads the TX COUNT field  from the FIFO Status Register (FSR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @return The data read from the FIFO Status Register
 */
static inline uint32_t LPSPI_ReadTxCount(const LPSPI_Type * base)
{
    return (uint32_t)(((uint32_t)(base->FSR & LPSPI_FSR_TXCOUNT_MASK)) >> LPSPI_FSR_TXCOUNT_SHIFT);
}

/*!
 * @brief Reads RX COUNT form the FIFO Status Register.
 *
 * This function reads the RX COUNT field  from the FIFO Status Register (FSR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @return The data read from the FIFO Status Register
 */
static inline uint32_t LPSPI_ReadRxCount(const LPSPI_Type * base)
{
    return (uint32_t)((((uint32_t)base->FSR & (uint32_t)LPSPI_FSR_RXCOUNT_MASK)) >> (uint32_t)LPSPI_FSR_RXCOUNT_SHIFT);
}

/*!
 * @brief Clear RXMSK bit form TCR Register.
 *
 * This function clears the RXMSK bit from the Transmit Command Register (TCR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_ClearRxmaskBit(LPSPI_Type * base)
{
    (base->TCR) = ((base->TCR) & (~LPSPI_TCR_RXMSK_MASK));
}

/*!
 * @brief Set RXMSK bit form TCR Register.
 *
 * This function set the RXMSK bit from the Transmit Command Register (TCR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_SetRxmskBit(LPSPI_Type * base)
{
    (base->TCR) = ((base->TCR) | (LPSPI_TCR_RXMSK_MASK));
}

/*!
 * @brief Clear TXMSK bit form TCR Register.
 *
 * This function clears the TXMSK bit from the Transmit Command Register (TCR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_ClearTxmaskBit(LPSPI_Type * base)
{
    (base->TCR) = ((base->TCR) & (~LPSPI_TCR_TXMSK_MASK));
}

/*!
 * @brief Set TXMSK bit form TCR Register.
 *
 * This function set the TXMSK bit from the Transmit Command Register (TCR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_SetTxmskBit(LPSPI_Type * base)
{
    (base->TCR) = ((base->TCR) | (LPSPI_TCR_TXMSK_MASK));
}

/*!
 * @brief Clear CONTC bit form TCR Register.
 *
 * This function clears the CONTC bit from the Transmit Command Register (TCR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_ClearContCBit(LPSPI_Type * base)
{
    (base->TCR) = ((base->TCR) & (~LPSPI_TCR_CONTC_MASK));
}

/*!
 * @brief Set CONTC bit form TCR Register.
 *
 * This function set the CONTC bit from the Transmit Command Register (TCR).
 *
 * @param base Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_SetContCBit(LPSPI_Type * base)
{
    (base->TCR) = ((base->TCR) | (LPSPI_TCR_CONTC_MASK));
}

/*!
 * @brief Configures the clock prescaler used for all LPSPI master logic.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param prescaler Prescaler value for master logic.
 */
static inline void LPSPI_SetClockPrescaler (LPSPI_Type * base, lpspi_prescaler_t prescaler)
{
    uint32_t lpspi_tmp = base->TCR;
    lpspi_tmp &= ~(LPSPI_TCR_PRESCALE_MASK);
    lpspi_tmp |= ((uint32_t)prescaler << LPSPI_TCR_PRESCALE_SHIFT);
    base->TCR = lpspi_tmp;
}

/*!
 * @brief Get the clock prescaler used for all LPSPI master logic.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @return Prescaler value for master logic.
 */
static inline lpspi_prescaler_t LPSPI_GetClockPrescaler (const LPSPI_Type * base)
{
    return (lpspi_prescaler_t)(((uint32_t)((base->TCR) & LPSPI_TCR_PRESCALE_MASK)) >> LPSPI_TCR_PRESCALE_SHIFT);
}

/*!
 * @brief Configures if the sample point for master devices is delayed.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param isSamplingPointDelayed Configure if the sampling point is delayed for master devices
 */
static inline void LPSPI_SetSamplingPoint (LPSPI_Type * base, bool isSamplingPointDelayed)
{
    uint32_t lpspi_tmp = base->CFGR1;
    lpspi_tmp &= ~(LPSPI_CFGR1_SAMPLE_MASK);
    lpspi_tmp |= ((uint32_t)isSamplingPointDelayed << LPSPI_CFGR1_SAMPLE_SHIFT);
    base->CFGR1 = lpspi_tmp;
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* LPSPI_HW_ACCESS_H*/

/*******************************************************************************
 * EOF
 ******************************************************************************/
