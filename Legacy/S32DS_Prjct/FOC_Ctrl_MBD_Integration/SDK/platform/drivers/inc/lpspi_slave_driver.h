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
 
/**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Required Rule 8.5, object/function previously declared.
* This requirement is fulfilled since the function is declared as external in and only in
* one configuration C file.
*/ 

#ifndef LPSPI_SLAVE_DRIVER_H
#define LPSPI_SLAVE_DRIVER_H

#include "lpspi_shared_function.h"



/*!
 * @addtogroup lpspi_driver LPSPI Driver
 * @ingroup lpspi
 * @{
 */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  @brief User configuration structure for the SPI slave driver.
 * Implements : lpspi_slave_config_t_Class
 */
typedef struct 
{
    lpspi_signal_polarity_t pcsPolarity; /*!< PCS polarity */
    uint16_t bitcount;                   /*!< Number of bits/frame, minimum is 8-bits */
    lpspi_clock_phase_t clkPhase;        /*!< Selects which phase of clock to capture data */
    lpspi_which_pcs_t whichPcs;
    lpspi_sck_polarity_t clkPolarity;    /*!< Selects clock polarity */
    bool lsbFirst;                       /*!< Option to transmit LSB first */
    lpspi_transfer_type transferType;    /*!< Type of LPSPI transfer */
    uint8_t rxDMAChannel;                /*!< Channel number for DMA rx channel. If DMA mode isn't used this field will be ignored. */
    uint8_t txDMAChannel;                /*!< Channel number for DMA tx channel. If DMA mode isn't used this field will be ignored. */
    spi_callback_t callback;             /*!< Select the callback to transfer complete */
    void *callbackParam;                 /*!< Select additional callback parameters if it's necessary */
} lpspi_slave_config_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Return default configuration for SPI master.
 *
 * Initializes a structured provided by user with the configuration
 * of an interrupt based LPSPI transfer. 
 *
 */
 
 void LPSPI_DRV_SlaveGetDefaultConfig(lpspi_slave_config_t * spiConfig);

/*!
 * @brief Initializes a LPSPI instance for a slave mode operation, using interrupt mechanism.
 *
 * This function un-gates the clock to the LPSPI module, initializes the LPSPI for
 * slave mode. After it is  initialized, the LPSPI module is configured in slave mode and the
 * user can start transmitting and receiving data by calling send, receive, and transfer functions.
 * This function indicates LPSPI slave uses an interrupt mechanism.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param lpspiState The pointer to the LPSPI slave driver state structure.
 * @param slaveConfig The configuration structure lpspi_slave_user_config_t which
 *      configures the data bus format.
 *
 * @return An error code or STATUS_SUCCESS.
 */
status_t LPSPI_DRV_SlaveInit(uint32_t instance,
                               lpspi_state_t * lpspiState,
                               const lpspi_slave_config_t * slaveConfig);

/*!
 * @brief Shuts down an LPSPI instance interrupt mechanism.
 *
 * Disables the LPSPI module, gates its clock, and changes the LPSPI slave driver state to NonInit for the
 * LPSPI slave module which is initialized with interrupt mechanism. After de-initialization, the
 * user can re-initialize the LPSPI slave module with other mechanisms.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @return STATUS_SUCCESS if driver starts to send/receive data successfully.
 *         STATUS_ERROR if driver is error and needs to clean error.
 *         STATUS_BUSY if a transfer is in progress
 */
status_t LPSPI_DRV_SlaveDeinit(uint32_t instance);

/*!
 * @brief Transfers data on LPSPI bus using a blocking call.
 *
 * This function checks the driver status and mechanism and transmits/receives data through the LPSPI
 * bus. If the sendBuffer is NULL, the transmit process is ignored. If the receiveBuffer is NULL, the
 * receive process is ignored. If both the receiveBuffer and the sendBuffer are available, the transmit and the
 * receive progress is processed. If only the receiveBuffer is available, the receive is
 * processed. Otherwise, the transmit is processed. This function only returns when the
 * processes are completed. This function uses an interrupt mechanism.
 * Depending on frame size sendBuffer and receiveBuffer must be aligned like this:
 * -1 byte if frame size <= 8 bits 
 * -2 bytes if 8 bits < frame size <= 16 bits 
 * -4 bytes if 16 bits < frame size   
 *
 * @param instance The instance number of LPSPI peripheral
 * @param sendBuffer The pointer to data that user wants to transmit.
 * @param receiveBuffer The pointer to data that user wants to store received data.
 * @param transferByteCount The number of bytes to send and receive which is equal to size of send or receive buffers
 * @param timeout The maximum number of milliseconds that function waits before
 *              timed out reached.
 *
 * @return  STATUS_SUCCESS if driver starts to send/receive data successfully.
 *          STATUS_ERROR if driver is error and needs to clean error.
 *          STATUS_BUSY if a transfer is in progress
 *          STATUS_TIMEOUT if time out reached while transferring is in progress.
 */
status_t LPSPI_DRV_SlaveTransferBlocking(uint32_t instance,
                                           const uint8_t *sendBuffer,
                                           uint8_t *receiveBuffer,
                                           uint16_t transferByteCount,
                                           uint32_t timeout);

/*!
 * @brief Starts the transfer data on LPSPI bus using a non-blocking call.
 *
 * This function checks the driver status and mechanism and transmits/receives data through the LPSPI
 * bus. If the sendBuffer is NULL, the transmit process is ignored. If the receiveBuffer is NULL, the
 * receive process is ignored. If both the receiveBuffer and the sendBuffer are available, the transmit and the
 * receive progress is processed. If only the receiveBuffer is available, the receive is
 * processed. Otherwise, the transmit is processed. This function only returns when the
 * processes are completed. This function uses an interrupt mechanism.
 * Depending on frame size sendBuffer and receiveBuffer must be aligned like this:
 * -1 byte if frame size <= 8 bits 
 * -2 bytes if 8 bits < frame size <= 16 bits 
 * -4 bytes if 16 bits < frame size
 *
 * @param instance The instance number of LPSPI peripheral
 * @param sendBuffer The pointer to data that user wants to transmit.
 * @param receiveBuffer The pointer to data that user wants to store received data.
 * @param transferByteCount The number of bytes to send and receive which is equal to size of send or receive buffers
 *
 * @return  STATUS_SUCCESS if driver starts to send/receive data successfully.
 *          STATUS_ERROR if driver is error and needs to clean error.
 *          STATUS_BUSY if a transfer is in progress
 */
status_t LPSPI_DRV_SlaveTransfer(uint32_t instance,
                                   const uint8_t *sendBuffer,
                                   uint8_t *receiveBuffer,
                                   uint16_t transferByteCount);

/*!
 * @brief Aborts the transfer that started by a non-blocking call transfer function.
 *
 * This function stops the transfer which was started by the calling the SPI_DRV_SlaveTransfer() function.
 *
 * @param instance The instance number of SPI peripheral
 *
 * @return  STATUS_SUCCESS if everything is OK.
 *
 */
status_t LPSPI_DRV_SlaveAbortTransfer(uint32_t instance);

/*!
 * @brief Returns whether the previous transfer is finished.
 *
 * When performing an a-sync transfer, the user can call this function to ascertain
 * the state of the current transfer: in progress (or busy) or complete (success).
 * In addition, if the transfer is still in progress, the user can get the number
 * of bytes that have been transferred up to now.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param bytesRemained Pointer to value that is filled in with the number of
 *  frames that have been sent in the active transfer. A frame is defined as the
 *  number of bits per frame.
 *
 * @return STATUS_SUCCESS The transfer has completed successfully, or
 *         STATUS_BUSY The transfer is still in progress.
 *         STATUS_ERROR if driver is error and needs to clean error.
 */
status_t LPSPI_DRV_SlaveGetTransferStatus(uint32_t instance,
                                            uint32_t * bytesRemained);

/*!
 * @brief Interrupt handler for LPSPI slave mode.
 * This handler uses the buffers stored in the lpspi_master_state_t structs to transfer data.
 *
 * @param instance The instance number of the LPSPI peripheral.

 */
void LPSPI_DRV_SlaveIRQHandler(uint32_t instance);

#if defined(__cplusplus)
}
#endif


/*! @} */


#endif /* __LPSPI_SLAVE_DRIVER_H__ */


