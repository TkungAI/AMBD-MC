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
 * @lpspi_shared_function.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, A compatible declaration shall be
 * visible when an object or function with external linkage is defined.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.5, object/function previously declared.
 * This requirement is fulfilled since the function is declared as external in and only
 * in one configuration C file.
 */

#ifndef LPSPI_SHARED_FUNCTION_H
#define LPSPI_SHARED_FUNCTION_H

#include <stdbool.h>
#include "edma_driver.h"
#include "clock_manager.h"
#include "osif.h"
#include "status.h"
#include "callbacks.h"

/*!
 * @addtogroup lpspi_driver LPSPI Driver
 * @ingroup lpspi
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Extern for the LPSPI master driver's interrupt handler.*/
extern void LPSPI_DRV_MasterIRQHandler(uint32_t instance);

/* Extern for the LPSPI slave driver's interrupt handler.*/
extern void LPSPI_DRV_SlaveIRQHandler(uint32_t instance);

/*! @brief LPSPI Peripheral Chip Select (PCS) configuration (which PCS to configure).
 * Implements : lpspi_which_pcs_t_Class
 */
typedef enum
{
    LPSPI_PCS0 = 0U, /*!< PCS[0] */
    LPSPI_PCS1 = 1U, /*!< PCS[1] */
    LPSPI_PCS2 = 2U, /*!< PCS[2] */
    LPSPI_PCS3 = 3U  /*!< PCS[3] */
} lpspi_which_pcs_t;

/*! @brief LPSPI Signal (PCS and Host Request) Polarity configuration.
 * Implements : lpspi_signal_polarity_t_Class
 */
typedef enum
{
    LPSPI_ACTIVE_HIGH = 1U, /*!< Signal is Active High (idles low). */
    LPSPI_ACTIVE_LOW  = 0U  /*!< Signal is Active Low (idles high). */
} lpspi_signal_polarity_t;

/*! @brief LPSPI clock phase configuration.
 * Implements : lpspi_clock_phase_t_Class
 */
typedef enum
{
    LPSPI_CLOCK_PHASE_1ST_EDGE = 0U, /*!< Data captured on SCK 1st edge, changed on 2nd. */
    LPSPI_CLOCK_PHASE_2ND_EDGE = 1U  /*!< Data changed on SCK 1st edge, captured on 2nd. */
} lpspi_clock_phase_t;

/*! @brief LPSPI Clock Signal (SCK) Polarity configuration.
 * Implements : lpspi_sck_polarity_t_Class
 */
typedef enum
{
    LPSPI_SCK_ACTIVE_HIGH = 0U, /*!< Signal is Active High (idles low). */
    LPSPI_SCK_ACTIVE_LOW  = 1U  /*!< Signal is Active Low (idles high). */
} lpspi_sck_polarity_t;


 /*! @brief Type of LPSPI transfer (based on interrupts or DMA).
  * Implements : lpspi_transfer_type_Class
  */
typedef enum
{
    LPSPI_USING_DMA         = 0,    /*!< The driver will use DMA to perform SPI transfer */
    LPSPI_USING_INTERRUPTS,         /*!< The driver will use interrupts to perform SPI transfer */
} lpspi_transfer_type;

 /*! @brief Type of error reported by LPSPI
  */
typedef enum
{
    LPSPI_TRANSFER_OK = 0U,    /*!< Transfer OK */
    LPSPI_TRANSMIT_FAIL,       /*!< Error during transmission */
    LPSPI_RECEIVE_FAIL         /*!< Error during reception */
} transfer_status_t;

/*!
 * @brief Runtime state structure for the LPSPI master driver.
 *
 * This structure holds data that is used by the LPSPI peripheral driver to
 * communicate between the transfer function and the interrupt handler. The
 * interrupt handler also uses this information to keep track of its progress.
 * The user must pass  the memory for this run-time state structure. The
 * LPSPI master driver populates the members.
 * Implements : lpspi_state_t_Class
 */
typedef struct
{
    uint16_t bitsPerFrame;               /*!< Number of bits per frame: 8- to 4096-bits; needed for
                                              TCR programming */
    uint16_t bytesPerFrame;              /*!< Number of bytes per frame: 1- to 512-bytes */
    bool isPcsContinuous;                /*!< Option to keep chip select asserted until transfer
                                              complete; needed for TCR programming */
    bool isBlocking;                     /*!< Save the transfer type */
    uint32_t lpspiSrcClk;                /*!< Module source clock */
    volatile bool isTransferInProgress;  /*!< True if there is an active transfer */
    const uint8_t * txBuff;                      /*!< The buffer from which transmitted bytes are taken */
    uint8_t * rxBuff;                    /*!< The buffer into which received bytes are placed */
    volatile uint16_t txCount;           /*!< Number of bytes remaining to send  */
    volatile uint16_t rxCount;           /*!< Number of bytes remaining to receive */
    volatile uint16_t txFrameCnt;        /*!< Number of bytes from current frame which were already sent */
    volatile uint16_t rxFrameCnt;        /*!< Number of bytes from current frame which were already received */
    volatile bool lsb;                   /*!< True if first bit is LSB and false if first bit is MSB */
    uint8_t fifoSize;                    /*!< RX/TX fifo size */
    uint8_t rxDMAChannel;                /*!< Channel number for DMA rx channel */
    uint8_t txDMAChannel;                /*!< Channel number for DMA tx channel */
    lpspi_transfer_type transferType;    /*!< Type of LPSPI transfer */
    semaphore_t lpspiSemaphore;          /*!< The semaphore used for blocking transfers */
    transfer_status_t status;            /*!< The status of the current */
    spi_callback_t callback;             /*!< Select the callback to transfer complete */
    void *callbackParam;                 /*!< Select additional callback parameters if it's necessary */
    uint32_t dummy;                      /*!< This field is used for the cases when TX is NULL and LPSPI is in DMA mode */
} lpspi_state_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base pointers for SPI instances. */
extern LPSPI_Type * g_lpspiBase[LPSPI_INSTANCE_COUNT];

/*! @brief Table to save LPSPI IRQ enumeration numbers defined in the CMSIS header file. */
extern IRQn_Type g_lpspiIrqId[LPSPI_INSTANCE_COUNT];

/* Pointer to runtime state structure.*/
extern lpspi_state_t * g_lpspiStatePtr[LPSPI_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief The function LPSPI_DRV_IRQHandler passes IRQ control to either the master or
 * slave driver.
 *
 * The address of the IRQ handlers are checked to make sure they are non-zero before
 * they are called. If the IRQ handler's address is zero, it means that driver was
 * not present in the link (because the IRQ handlers are marked as weak). This would
 * actually be a program error, because it means the master/slave config for the IRQ
 * was set incorrectly.
 */
void LPSPI_DRV_IRQHandler(uint32_t instance);

/*!
 * @brief The function LPSPI_DRV_FillupTxBuffer writes data in TX hardware buffer
 * depending on driver state and number of bytes remained to send.
 */
void LPSPI_DRV_FillupTxBuffer(uint32_t instance);

/*!
 * @brief The function LPSPI_DRV_ReadRXBuffer reads data from RX hardware buffer and
 * writes this data in RX software buffer.
 */
void LPSPI_DRV_ReadRXBuffer(uint32_t instance);

/*!
 * @brief Disable the TEIE interrupts at the end of a transfer.
 * Disable the interrupts and clear the status for transmit/receive errors.
 */
void LPSPI_DRV_DisableTEIEInterrupts(uint32_t instance);
/*! @} */


#endif /* __LPSPI_SHARED_FUNCTION_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

