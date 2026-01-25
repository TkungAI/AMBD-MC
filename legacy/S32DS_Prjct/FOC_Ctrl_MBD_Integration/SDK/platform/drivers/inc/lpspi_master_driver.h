/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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
* Violates MISRA 2012 Required Rule 8.5, object/function previously declared.
* This requirement is fulfilled since the function is declared as external in and only in
* one configuration C file.
*/

#ifndef LPSPI_MASTER_DRIVER_H
#define LPSPI_MASTER_DRIVER_H

#include "lpspi_shared_function.h"

/*!
 * @addtogroup lpspi_driver LPSPI Driver
 * @ingroup lpspi
 * @brief Low Power Serial Peripheral Interface Peripheral Driver
 * @{
 */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Data structure containing information about a device on the SPI bus.
 *
 * The user must populate these members to set up the LPSPI master and
 * properly communicate with the SPI device.
 * Implements : lpspi_master_config_t_Class
 */
typedef struct
{
    uint32_t bitsPerSec;                 /*!< Baud rate in bits per second*/
    lpspi_which_pcs_t whichPcs;          /*!< Selects which PCS to use */
    lpspi_signal_polarity_t pcsPolarity; /*!< PCS polarity */
    bool isPcsContinuous;                /*!< Keeps PCS asserted until transfer complete */
    uint16_t bitcount;                   /*!< Number of bits/frame, minimum is 8-bits */
    uint32_t lpspiSrcClk;                /*!< Module source clock */
    lpspi_clock_phase_t clkPhase;        /*!< Selects which phase of clock to capture data */
    lpspi_sck_polarity_t clkPolarity;    /*!< Selects clock polarity */
    bool lsbFirst;                       /*!< Option to transmit LSB first */
    lpspi_transfer_type transferType;    /*!< Type of LPSPI transfer */
    uint8_t rxDMAChannel;                /*!< Channel number for DMA rx channel. If DMA mode isn't used this field will be ignored. */
    uint8_t txDMAChannel;                /*!< Channel number for DMA tx channel. If DMA mode isn't used this field will be ignored. */
    spi_callback_t callback;             /*!< Select the callback to transfer complete */
    void *callbackParam;                 /*!< Select additional callback parameters if it's necessary */
} lpspi_master_config_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and shutdown
 * @{
 */

/*!
 * @brief Return default configuration for SPI master
 *
 * Initializes a structured provided by user with the configuration
 * of an interrupt based LPSPI transfer. Source clock for LPSPI is configured to 
 * 8MHz. If the applications uses other frequency is necessary to update lpspiSrcClk field.
 *
 @param spiConfig Pointer to configuration structure which is filled with default configuration 
 */
 
 void LPSPI_DRV_MasterGetDefaultConfig(lpspi_master_config_t * spiConfig);
 
/*!
 * @brief Initializes a LPSPI instance for interrupt driven master mode operation.
 *
 * This function uses an interrupt-driven method for transferring data.
 * In this function, the term "spiConfig" is used to indicate the SPI device for which the LPSPI
 * master is communicating.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, un-gates the clock to the LPSPI module, resets the LPSPI module,
 * configures the IRQ state structure, enables the module-level interrupt to the core, and
 * enables the LPSPI module.
 * This is an example to set up the lpspi_master_state_t and call the
 * LPSPI_DRV_MasterInit function by passing in these parameters:
   @code
    lpspi_master_state_t lpspiMasterState;  <- the user  allocates memory for this structure
    lpspi_master_config_t spiConfig;  Can declare more configs for use in transfer functions
    spiConfig.bitsPerSec = 500000;
    spiConfig.whichPcs = LPSPI_PCS0;
    spiConfig.pcsPolarity = LPSPI_ACTIVE_LOW;
    spiConfig.isPcsContinuous = false;
    spiConfig.bitCount = 16;
    spiConfig.clkPhase = LPSPI_CLOCK_PHASE_1ST_EDGE;
    spiConfig.clkPolarity = LPSPI_ACTIVE_HIGH;
    spiConfig.lsbFirst= false;
    spiConfig.transferType = LPSPI_USING_INTERRUPTS;
    LPSPI_DRV_MasterInit(masterInstance, &lpspiMasterState, &spiConfig);
   @endcode
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param lpspiState The pointer to the LPSPI master driver state structure. The user
 *  passes the memory for this run-time state structure. The LPSPI master driver
 *  populates the members. This run-time state structure keeps track of the
 *  transfer in progress.
 * @param spiConfig The data structure containing information about a device on the SPI bus
 * @return An error code or STATUS_SUCCESS.
 */
status_t LPSPI_DRV_MasterInit(uint32_t instance, lpspi_state_t * lpspiState,
                                    const lpspi_master_config_t * spiConfig);

/*!
 * @brief Shuts down a LPSPI instance.
 *
 * This function resets the LPSPI peripheral, gates its clock, and disables the interrupt to
 * the core.  It first checks to see if a transfer is in progress and if so returns an error
 * status.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @return STATUS_SUCCESS The transfer has completed successfully, or
 *         STATUS_BUSY The transfer is still in progress.
 *         STATUS_ERROR if driver is error and needs to clean error.
 */
status_t LPSPI_DRV_MasterDeinit(uint32_t instance);

/*!
 * @brief Configures the LPSPI master mode bus timing delay options.
 *
 * This function involves the LPSPI module's delay options to
 * "fine tune" some of the signal timings and match the timing needs of a slower peripheral device.
 * This is an optional function that can be called after the LPSPI module has been initialized for
 * master mode. The timings are adjusted in terms of cycles of the baud rate clock.
 * The bus timing delays that can be adjusted are listed below:
 *
 * SCK to PCS Delay: Adjustable delay option between the last edge of SCK to the de-assertion
 *                   of the PCS signal.
 *
 * PCS to SCK Delay: Adjustable delay option between the assertion of the PCS signal to the
 *                   first SCK edge.
 *
 * Delay between Transfers: Adjustable delay option between the de-assertion of the PCS signal for
 *                          a frame to the assertion of the PCS signal for the next frame.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param delayBetwenTransfers Minimum delay between 2 transfers in microseconds
 * @param delaySCKtoPCS Minimum delay between SCK and PCS
 * @param delayPCStoSCK Minimum delay between PCS and SCK
 * @return STATUS_SUCCESS The transfer has completed successfully, or
 *         STATUS_ERROR if driver is error and needs to clean error.
 */
status_t LPSPI_DRV_MasterSetDelay(uint32_t instance, uint32_t delayBetwenTransfers,
                        uint32_t delaySCKtoPCS, uint32_t delayPCStoSCK);


/*@}*/

/*!
 * @name Bus configuration
 * @{
 */

/*!
 * @brief Configures the LPSPI port physical parameters to access a device on the bus when the LSPI
 *        instance is configured for interrupt operation.
 *
 * In this function, the term "spiConfig" is used to indicate the SPI device for which the LPSPI
 * master is communicating. This is an optional function as the spiConfig parameters are
 * normally configured in the initialization function or the transfer functions, where these various
 * functions would call the configure bus function.
 * This is an example to set up the lpspi_master_config_t structure
 * to call the LPSPI_DRV_MasterConfigureBus function by passing in these parameters:
   @code
    lpspi_master_config_t spiConfig1;   You can also declare spiConfig2, spiConfig3, etc
    spiConfig1.bitsPerSec = 500000;
    spiConfig1.whichPcs = LPSPI_PCS0;
    spiConfig1.pcsPolarity = LPSPI_ACTIVE_LOW;
    spiConfig1.isPcsContinuous = false;
    spiConfig1.bitCount = 16;
    spiConfig1.clkPhase = LPSPI_CLOCK_PHASE_1ST_EDGE;
    spiConfig1.clkPolarity = LPSPI_ACTIVE_HIGH;
    spiConfig1.lsbFirst= false;
    spiConfig.transferType = LPSPI_USING_INTERRUPTS;
   @endcode
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param spiConfig Pointer to the spiConfig structure. This structure contains the settings
 *  for the SPI bus configuration.  The SPI device parameters are the desired baud rate (in
 *  bits-per-sec), bits-per-frame, chip select attributes, clock attributes, and data shift
 *  direction.
 * @param calculatedBaudRate The calculated baud rate passed back to the user to determine
 *  if the calculated baud rate is close enough to meet the needs. The baud rate never exceeds
 *  the desired baud rate.
 * @return STATUS_SUCCESS The transfer has completed successfully, or
 *         STATUS_ERROR if driver is error and needs to clean error.
 */
status_t LPSPI_DRV_MasterConfigureBus(uint32_t instance,
                                            const lpspi_master_config_t * spiConfig,
                                            uint32_t * calculatedBaudRate);

/*!
* @brief Select the chip to communicate with.
*
* The main purpose of this function is to set the PCS and the appropriate polarity.
* @param instance LPSPI instance
* @param whichPcs selected chip
* @param polarity chip select line polarity
* @return STATUS_SUCCESS The transfer has completed successfully, or
*         STATUS_ERROR if driver is error and needs to clean error.
*/
status_t LPSPI_DRV_SetPcs(uint32_t instance, lpspi_which_pcs_t whichPcs, lpspi_signal_polarity_t polarity);

/*@}*/

/*!
 * @name Blocking transfers
 * @{
 */

/*!
 * @brief Performs an interrupt driven blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function does not return until the transfer is complete.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 * Depending on frame size sendBuffer and receiveBuffer must be aligned like this:
 * -1 byte if frame size <= 8 bits 
 * -2 bytes if 8 bits < frame size <= 16 bits 
 * -4 bytes if 16 bits < frame size   
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive which is equal to size of send or receive buffers
 * @param timeout A timeout for the transfer in milliseconds. If the transfer takes longer than
 *  this amount of time, the transfer is aborted and a STATUS_TIMEOUT error
 *  returned.
 * @return STATUS_SUCCESS The transfer was successful, or
 *         STATUS_BUSY Cannot perform transfer because a transfer is already in progress, or
 *         STATUS_TIMEOUT The transfer timed out and was aborted.
 */
status_t LPSPI_DRV_MasterTransferBlocking(uint32_t instance,
                                                const uint8_t * sendBuffer,
                                                uint8_t * receiveBuffer,
                                                uint16_t transferByteCount,
                                                uint32_t timeout);

/*@}*/

/*!
 * @name Non-blocking transfers
 * @{
 */

/*!
 * @brief Performs an interrupt driven non-blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function returns immediately after initiating the transfer. The user
 * needs to check whether the transfer is complete using the LPSPI_DRV_MasterGetTransferStatus
 * function.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 * Depending on frame size sendBuffer and receiveBuffer must be aligned like this:
 * -1 byte if frame size <= 8 bits 
 * -2 bytes if 8 bits < frame size <= 16 bits 
 * -4 bytes if 16 bits < frame size
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param spiConfig Pointer to the SPI configuration structure. This structure contains the settings
 *  for the SPI bus configuration in this transfer. You may pass NULL for this
 *  parameter, in which case the current bus configuration is used unmodified. The device can be
 *  configured separately by calling the LPSPI_DRV_MasterConfigureBus function.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive which is equal to size of send or receive buffers
 * @return STATUS_SUCCESS The transfer was successful, or
 *         STATUS_BUSY Cannot perform transfer because a transfer is already in progress
 */
status_t LPSPI_DRV_MasterTransfer(uint32_t instance,
                                        const uint8_t * sendBuffer,
                                        uint8_t * receiveBuffer,
                                        uint16_t transferByteCount);

/*!
 * @brief Returns whether the previous interrupt driven transfer is completed.
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function to ascertain
 * the state of the current transfer: in progress (or busy) or complete (success).
 * In addition, if the transfer is still in progress, the user can get the number of words that
 * have been transferred up to now.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param bytesRemained Pointer to a value that is filled in with the number of bytes that
 *      must be received.
 * @return STATUS_SUCCESS The transfer has completed successfully, or
 *         STATUS_BUSY The transfer is still in progress. framesTransferred is filled
 *         with the number of words that have been transferred so far.
 */
status_t LPSPI_DRV_MasterGetTransferStatus(uint32_t instance, uint32_t * bytesRemained);

/*!
 * @brief Terminates an interrupt driven asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to terminate the transfer early
 * if the transfer is still in progress.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @return STATUS_SUCCESS The transfer was successful, or
 *         LPSPI_STATUS_NO_TRANSFER_IN_PROGRESS No transfer is currently in progress.
 */
status_t LPSPI_DRV_MasterAbortTransfer(uint32_t instance);

/*!
 * @brief Interrupt handler for LPSPI master mode.
 * This handler uses the buffers stored in the lpspi_master_state_t structs to transfer data.
 *
 * @param instance The instance number of the LPSPI peripheral.
 */
void LPSPI_DRV_MasterIRQHandler(uint32_t instance);

/* @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* __LPSPI_MASTER_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
