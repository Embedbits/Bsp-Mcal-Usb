/**
 * \author Mr.Nobody
 * \file Usb_Pma.c
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module packet memory (PMA) handling
 *
 * The packet memory of the USB_DRD_FS controller (2 KB) holds the buffer descriptor table
 * (8 bytes per endpoint) followed by the packet buffers of the endpoints. It is accessed by
 * 32-bit words only. This file contains the data copy between the application memory and
 * the packet memory and the allocation of the packet buffers.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usb.h"                            /* Self include                   */
#include "Usb_Types.h"                      /* Module types definitions       */
#include <string.h>                         /* Unaligned word access          */
/* ============================== TYPEDEFS ================================== */

#if defined(USB_DRD_FS)

/* ======================== FORWARD DECLARATIONS ============================ */

static volatile uint32_t * Usb_Pma_Get_WordPtr( usb_PmaAddr_t pmaAddr );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Size of the packet memory access word in bytes */
#define USB_PMA_WORD_SIZE                   ( 4u )

/** Maximal count of bytes of a packet buffer using 2-byte blocks (buffer descriptor limit) */
#define USB_PMA_SMALL_BLOCK_MAX_SIZE        ( 62u )

/** Size of the large block of the buffer descriptor in bytes */
#define USB_PMA_LARGE_BLOCK_SIZE            ( 32u )

/** Count of endpoint buffers which can be allocated (one per endpoint number and direction) */
#define USB_PMA_SLOT_CNT                    ( USB_EP_NUM_CNT * USB_EP_DIR_CNT )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Copies data from the application memory to the packet memory
 *
 * The packet memory is written by whole 32-bit words, the last incomplete word is padded by
 * zeros. The source needs no alignment.
 *
 * \param srcData [in]: Source data in the application memory
 * \param pmaAddr [in]: Destination address in the packet memory (word aligned)
 * \param dataLen [in]: Count of bytes to copy
 */
void Usb_Pma_Write( const usb_Data_t * const srcData, usb_PmaAddr_t pmaAddr, usb_DataCnt_t dataLen )
{
    volatile uint32_t * const pmaWord  = Usb_Pma_Get_WordPtr( pmaAddr );
    const usb_DataCnt_t       wordCnt  = dataLen / USB_PMA_WORD_SIZE;
    const usb_DataCnt_t       tailLen  = dataLen % USB_PMA_WORD_SIZE;

    for( usb_DataCnt_t wordIdx = 0u; wordCnt > wordIdx; wordIdx ++ )
    {
        uint32_t wordValue = 0u;

        (void)memcpy( &wordValue, &srcData[ wordIdx * USB_PMA_WORD_SIZE ], USB_PMA_WORD_SIZE );

        pmaWord[ wordIdx ] = wordValue;
    }

    if( 0u != tailLen )
    {
        uint32_t wordValue = 0u;

        (void)memcpy( &wordValue, &srcData[ wordCnt * USB_PMA_WORD_SIZE ], tailLen );

        pmaWord[ wordCnt ] = wordValue;
    }
    else
    {
        /* Data length is a multiple of the word size */
    }
}


/**
 * \brief Copies data from the packet memory to the application memory
 *
 * The packet memory is read by whole 32-bit words. The destination needs no alignment.
 *
 * \param dstData [out]: Destination in the application memory
 * \param pmaAddr  [in]: Source address in the packet memory (word aligned)
 * \param dataLen  [in]: Count of bytes to copy
 */
void Usb_Pma_Read( usb_Data_t * const dstData, usb_PmaAddr_t pmaAddr, usb_DataCnt_t dataLen )
{
    volatile uint32_t * const pmaWord  = Usb_Pma_Get_WordPtr( pmaAddr );
    const usb_DataCnt_t       wordCnt  = dataLen / USB_PMA_WORD_SIZE;
    const usb_DataCnt_t       tailLen  = dataLen % USB_PMA_WORD_SIZE;

    for( usb_DataCnt_t wordIdx = 0u; wordCnt > wordIdx; wordIdx ++ )
    {
        const uint32_t wordValue = pmaWord[ wordIdx ];

        (void)memcpy( &dstData[ wordIdx * USB_PMA_WORD_SIZE ], &wordValue, USB_PMA_WORD_SIZE );
    }

    if( 0u != tailLen )
    {
        const uint32_t wordValue = pmaWord[ wordCnt ];

        (void)memcpy( &dstData[ wordCnt * USB_PMA_WORD_SIZE ], &wordValue, tailLen );
    }
    else
    {
        /* Data length is a multiple of the word size */
    }
}


/**
 * \brief Clears the buffer descriptor table of the packet memory
 *
 * Addresses and counts of all endpoint buffers become zero.
 */
void Usb_Pma_Reset( void )
{
    volatile uint32_t * const pmaWord = Usb_Pma_Get_WordPtr( 0u );

    for( usb_DataCnt_t wordIdx = 0u; ( USB_PMA_BTABLE_SIZE / USB_PMA_WORD_SIZE ) > wordIdx; wordIdx ++ )
    {
        pmaWord[ wordIdx ] = 0u;
    }
}


/**
 * \brief Waits until the controller finished the packet memory accesses of a received packet
 *
 * Device errata ES0561 section 2.15.1 (STM32H503): the correct transfer interrupt is triggered a
 * little before the last accesses of the controller to the packet memory (data and buffer
 * descriptor) are completed, so the contents read at once can be incomplete. The software has to
 * wait 800 ns in full speed mode (6.4 us in low speed mode) before it reads the received packet.
 * The wait is a fixed count of reads of the control register (no timer is needed).
 *
 * \param periphReg [in]: USB controller registers
 */
void Usb_Pma_Wait_Settle( const USB_DRD_TypeDef * const periphReg )
{
    for( usb_TimeoutCnt_t readIdx = 0u; USB_PMA_SETTLE_READ_CNT > readIdx; readIdx ++ )
    {
        (void)periphReg->CNTR;
    }
}


/**
 * \brief Calculates the size of the packet buffer reserved in the packet memory for an endpoint
 *
 * The receive buffer size is described by the buffer descriptor in blocks of 2 bytes (up to
 * 62 bytes) or 32 bytes (larger buffers), so the buffer is rounded up to the block size.
 * The buffer address of the next buffer is word aligned.
 *
 * \param maxPacketSize [in]: Maximal packet size of the endpoint in bytes
 *
 * \return Size of the packet buffer in bytes
 */
usb_PmaSize_t Usb_Pma_Get_BufferSize( usb_EpMaxPacket_t maxPacketSize )
{
    usb_PmaSize_t bufSize = 0u;

    if( USB_PMA_SMALL_BLOCK_MAX_SIZE < maxPacketSize )
    {
        const usb_PmaSize_t blockCnt = (usb_PmaSize_t)( ( maxPacketSize + USB_PMA_LARGE_BLOCK_SIZE - 1u ) / USB_PMA_LARGE_BLOCK_SIZE );

        bufSize = (usb_PmaSize_t)( blockCnt * USB_PMA_LARGE_BLOCK_SIZE );
    }
    else
    {
        const usb_PmaSize_t wordCnt = (usb_PmaSize_t)( ( maxPacketSize + USB_PMA_ALIGN - 1u ) / USB_PMA_ALIGN );

        bufSize = (usb_PmaSize_t)( wordCnt * USB_PMA_ALIGN );
    }

    return ( bufSize );
}


/**
 * \brief Allocates a packet buffer in the free area of the packet memory
 *
 * First fit: the lowest address behind the buffer descriptor table where the buffer does not
 * overlap the buffer of another initialized endpoint. Buffers are released by closing the
 * endpoint (size of its buffer becomes zero).
 *
 * \param usbContext [in]: Context of the USB controller holding the allocated buffers
 * \param bufSize    [in]: Size of the buffer in bytes (multiple of word size)
 * \param pmaAddr   [out]: Address of the allocated buffer in the packet memory
 *
 * \return State of request execution. Returns "OK" if the buffer was allocated, otherwise
 *         returns error (invalid parameter or no free area of the required size).
 */
usb_RequestState_t Usb_Pma_Alloc( const usb_Context_t * const usbContext, usb_PmaSize_t bufSize, usb_PmaAddr_t * const pmaAddr )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( ( USB_NULL_PTR != usbContext ) &&
        ( USB_NULL_PTR != pmaAddr    ) &&
        ( 0u            < bufSize    )    )
    {
        usb_DataCnt_t   candidate = USB_PMA_BTABLE_SIZE;
        usb_FlagState_t overlap   = USB_FLAG_ACTIVE;
        usb_DataCnt_t   passIdx   = 0u;

        /* Every pass moves the candidate behind at least one allocated buffer, so the passes are limited by their count */
        while( ( ( USB_PMA_SLOT_CNT + 1u ) >  passIdx ) &&
               ( USB_FLAG_ACTIVE           == overlap )    )
        {
            overlap = USB_FLAG_INACTIVE;
            passIdx ++;

            for( usb_EpNum_t epNum = 0u; USB_EP_NUM_CNT > epNum; epNum ++ )
            {
                for( usb_DataCnt_t dirIdx = 0u; USB_EP_DIR_CNT > dirIdx; dirIdx ++ )
                {
                    const usb_EpContext_t * const epContext = &usbContext->Ep[ epNum ][ dirIdx ];
                    const usb_DataCnt_t           slotStart = epContext->PmaAddr;
                    const usb_DataCnt_t           slotEnd   = slotStart + epContext->PmaSize;

                    if( ( USB_FUNCTION_ACTIVE == epContext->Opened ) &&
                        ( candidate           < slotEnd            ) &&
                        ( ( candidate + bufSize ) > slotStart      )    )
                    {
                        candidate = slotEnd;
                        overlap   = USB_FLAG_ACTIVE;
                    }
                    else
                    {
                        /* Buffer of the endpoint does not collide with the candidate */
                    }
                }
            }
        }

        if( ( USB_FLAG_INACTIVE == overlap                          ) &&
            ( USB_PMA_TOTAL_SIZE >= ( candidate + bufSize )         )    )
        {
            *pmaAddr = (usb_PmaAddr_t)candidate;
            retState = USB_REQUEST_OK;
        }
        else
        {
            /* No free area of the required size */
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Returns pointer to a word of the packet memory
 *
 * \param pmaAddr [in]: Address in the packet memory (byte offset from the packet memory start)
 *
 * \return Pointer to the word at the address
 */
static volatile uint32_t * Usb_Pma_Get_WordPtr( usb_PmaAddr_t pmaAddr )
{
    return ( (volatile uint32_t *)(uintptr_t)( USB_DRD_PMAADDR + (uintptr_t)pmaAddr ) );
}

#endif /* USB_DRD_FS */
