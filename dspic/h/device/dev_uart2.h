//======================================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives exclusively with
// Microchip products. It is your responsibility to comply with third party license terms applicable to your use of
// third-party software (including open source software) that may accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO
// THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE,
// COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY
// ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE
// PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//======================================================================================================================

//======================================================================================================================
// @file dev_uart2.h
//
// @brief uart2 driver
//
//======================================================================================================================

#ifndef _DEV_UART2_H_
#define _DEV_UART2_H_


#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif
        
//======================================================================================================================
// @brief UART2 Driver Hardware Status Flags
// @description
//      This type specifies the status of the hardware receive or transmit.
//      More than one of these values may be OR'd together to create a complete status value.
//      To test a value of this type, the bit of interest must be AND'ed with value and
//      checked to see if the result is non-zero.
//======================================================================================================================
typedef enum
{
    // Indicates that Receive buffer has overflowed
    UART2_RX_OVERRUN_ERROR              = (1 << 1),
    // Indicates that Receive break interrupt was received
    UART2_RX_BREAK_INT_FLAG             = (1 << 2),
    // Indicates that Framing error has been detected for the current character
    UART2_FRAMING_ERROR                 = (1 << 3),
    // Indicates that Auto-Baud rate acquisition interrupt was received
    UART2_AUTOBAUD_RATE_ACQ_INT_FLAG    = (1 << 5),
    // Indicates that Parity error has been detected for the current character
    UART2_PARITY_ERROR                  = (1 << 6),
    // Indicates that Transmit shifter empty interrupt was received
    UART2_TX_SHIFTER_EMPTY_INT_FLAG     = (1 << 7),
    // Indicates that Receive buffer is full
    UART2_RX_BUFFER_FULL                = (1UL << 16),
    // Indicates that Receive buffer is empty
    UART2_RX_BUFFER_EMPTY               = (1UL << 17),
    // Indicates that Receiver is Idle
    UART2_RECEIVER_IDLE                 = (1UL << 19),
    // Indicates that Transmit buffer is full
    UART2_TX_BUFFER_FULL                = (1UL << 20),
    // Indicates that Transmit buffer is empty
    UART2_TX_BUFFER_EMPTY               = (1UL << 21),
    // Indicates that Stop bit detection mode is set
    UART2_STOP_DETECT_MODE              = (1UL << 22),
    // Indicates that TX Write transmit error has been detected for the current character
    UART2_TX_WRITE_TX_ERROR             = (1UL << 23),
} DEV_UART2_STATUS;


//======================================================================================================================
// @brief UART2 Driver Transfer Flags
// @description
//      This type specifies the status of the receive or transmit operation.
//      More than one of these values may be OR'd together to create a complete status value.
//      To test a value of this type, the bit of interest must be AND'ed with value and
//      checked to see if the result is non-zero.
//======================================================================================================================
typedef enum
{
    // Indicates that the core driver buffer is full
    UART2_TRANSFER_STATUS_RX_FULL           = (1 << 0),
    // Indicates that at least one byte of Data has been received
    UART2_TRANSFER_STATUS_RX_DATA_PRESENT   = (1 << 1),
    // Indicates that the core driver receiver buffer is empty
    UART2_TRANSFER_STATUS_RX_EMPTY          = (1 << 2),
    // Indicates that the core driver transmitter buffer is full
    UART2_TRANSFER_STATUS_TX_FULL           = (1 << 3),
    // Indicates that the core driver transmitter buffer is empty
    UART2_TRANSFER_STATUS_TX_EMPTY          = (1 << 4)
} DEV_UART2_TRANSFER_STATUS;


//======================================================================================================================
// @brief   initializes the UART2 Driver
// @note    call this function at boot up to initialize the UART2 Driver
//======================================================================================================================
void Dev_UART2_Init(void);


//======================================================================================================================
// @brief   UART2 Read function for one byte
// @note    use this function to read one byte from the UART2 Receive Queue
// @note    before using this function you have to make sure that the Receive Queue is not empty
//======================================================================================================================
uint8_t Dev_UART2_Read(void);


//======================================================================================================================
// @brief   UART2 Read function for multiple bytes
// @note    use this function to read some bytes from the UART2 Receive Queue
// @param   buffer - Buffer into which the data read from the UART2
// @param   numbytes - Total number of bytes that need to be read from the UART2,
//          (must be equal to or less than the size of the buffer)
// @returns the number of bytes transferred
// @preconditions   UART2_Initializer function should have been called before calling this function
//======================================================================================================================
unsigned int Dev_UART2_ReadBuffer(uint8_t *buffer,  const unsigned int numbytes);



//======================================================================================================================
// @brief   UART2 Write function for one byte
// @note    use this function to write one byte to the UART2 Transfer Queue
// @note    before using this function, make sure the buffer is not full
// @param   byte - Data byte to write to the UART2
// @preconditions
//          UART2_Initializer function should have been called before calling this function.
//          The transfer status should be checked to see if transmitter is not full before calling this function.
//======================================================================================================================
void Dev_UART2_Write( const uint8_t byte);


//======================================================================================================================
// @brief   UART2 Write function for multiple bytes
// @note    use this function to write multiple bytes to the UART2 Transfer Queue
// @returns the number of bytes transferred into the UART2 Transfer Queue
// @preconditions   UART2_Initializer function should have been called before calling this function
//======================================================================================================================
unsigned int Dev_UART2_WriteBuffer(const uint8_t *buffer ,const unsigned int numbytes);


//======================================================================================================================
// @brief   Returns the Transfer status of the UART2 TX and RX Queue
// @description
//  This returns the transmitter and receiver transfer status. The returned status may contain a value with more
//  than one of the bits specified in the DEV_UART2_TRANSFER_STATUS enumeration set.
//  The caller should perform an "AND" with the bit of interest and verify if the result is non-zero
//  to verify the desired status bit.
//  @returns    A DEV_UART2_TRANSFER_STATUS value describing the current status of the transfer.
//  @preconditions  UART2_Initializer function should have been called before calling this function
//======================================================================================================================
DEV_UART2_TRANSFER_STATUS Dev_UART2_TransferStatusGet (void );


//======================================================================================================================
// @brief   Returns the character from the UART2 RX Queue with the provided offset without extracting it
// @param   offset - UART2 RX buffer peek position.
//          Offset input range should be 0 to (UART2_CONFIG_RX_BYTEQ_LENGTH - 1)
// @note    Make sure that for the given offset there are enough bytes in the RX Queue
//======================================================================================================================
uint8_t Dev_UART2_Peek(uint16_t offset);


//======================================================================================================================
// @brief   Returns the character from the UART2 RX Queue with the provided offset without extracting it
// @note    this function validates all the possible conditions for getting the character wanted
// @param   
//  dataByte - Data byte to be read from UART2 RX buffer based on offset position.
//  offset   - UART2 RX buffer peek position. Offset input range is should be 0 to (UART2_CONFIG_RX_BYTEQ_LENGTH - 1)
// @returns true, if there was a character retrieved in dataByte
//======================================================================================================================
bool Dev_UART2_PeekSafe(uint8_t *dataByte, uint16_t offset);


//======================================================================================================================
// @brief   Returns the amount of characters in the receive buffer
// @returns amount of characters in the receive buffer
//======================================================================================================================
unsigned int Dev_UART2_ReceiveBufferSizeGet(void);


//======================================================================================================================
// @brief   Returns the amount of characters in the transmit buffer
// @returns amount of characters in the transmit buffer
//======================================================================================================================
unsigned int Dev_UART2_TransmitBufferSizeGet(void);


//======================================================================================================================
// @brief   Returns if the Receive Buffer is empty or not
// @returns
//  True if the receive buffer is empty
//  False if the receive buffer is not empty
//======================================================================================================================
bool Dev_UART2_ReceiveBufferIsEmpty(void);


//======================================================================================================================
// @brief   Returns if the Transmit Buffer is empty or not
// @Returns
//  True if the transmit buffer is full
//  False if the transmit buffer is not full
//======================================================================================================================
bool Dev_UART2_TransmitBufferIsFull(void);


//======================================================================================================================
// @brief   Returns the 32 bit transmitter and receiver hardware status of the UART2
// @note    The caller should perform an "AND" with the bit of interest and verify if the result
//          is non-zero (as shown in the example) to verify the desired status bit.
// @returns 32 bit value describing the current status of the transfer
// @preconditions   UART2_Initializer function should have been called before calling this function
//======================================================================================================================
uint32_t Dev_UART2_StatusGet(void);



#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif  // _DEV_UART2_H_
