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
// @file dev_uart.h
//
// @brief uart structures
//
//======================================================================================================================

#ifndef _DEV_UART_H_
#define _DEV_UART_H_

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif


//======================================================================================================================
// @brief UART Driver Hardware Status Flags
// @description
//      This type specifies the status of the hardware receive or transmit.
//      More than one of these values may be OR'd together to create a complete status value.
//      To test a value of this type, the bit of interest must be AND'ed with value and
//      checked to see if the result is non-zero.
//======================================================================================================================
typedef enum
{
    // Indicates that Receive buffer has overflowed
    UART_RX_OVERRUN_ERROR              = (1 << 1),
    // Indicates that Receive break interrupt was received
    UART_RX_BREAK_INT_FLAG             = (1 << 2),
    // Indicates that Framing error has been detected for the current character
    UART_FRAMING_ERROR                 = (1 << 3),
    // Indicates that Auto-Baud rate acquisition interrupt was received
    UART_AUTOBAUD_RATE_ACQ_INT_FLAG    = (1 << 5),
    // Indicates that Parity error has been detected for the current character
    UART_PARITY_ERROR                  = (1 << 6),
    // Indicates that Transmit shifter empty interrupt was received
    UART_TX_SHIFTER_EMPTY_INT_FLAG     = (1 << 7),
    // Indicates that Receive buffer is full
    UART_RX_BUFFER_FULL                = (1UL << 16),
    // Indicates that Receive buffer is empty
    UART_RX_BUFFER_EMPTY               = (1UL << 17),
    // Indicates that Receiver is Idle
    UART_RECEIVER_IDLE                 = (1UL << 19),
    // Indicates that Transmit buffer is full
    UART_TX_BUFFER_FULL                = (1UL << 20),
    // Indicates that Transmit buffer is empty
    UART_TX_BUFFER_EMPTY               = (1UL << 21),
    // Indicates that Stop bit detection mode is set
    UART_STOP_DETECT_MODE              = (1UL << 22),
    // Indicates that TX Write transmit error has been detected for the current character
    UART_TX_WRITE_TX_ERROR             = (1UL << 23),
} DEV_UART_STATUS;


//======================================================================================================================
// @brief UART Driver Transfer Flags
// @description
//      This type specifies the status of the receive or transmit operation.
//      More than one of these values may be OR'd together to create a complete status value.
//      To test a value of this type, the bit of interest must be AND'ed with value and
//      checked to see if the result is non-zero.
//======================================================================================================================
typedef enum
{
    // Indicates that the core driver buffer is full
    UART_TRANSFER_STATUS_RX_FULL           = (1 << 0),
    // Indicates that at least one byte of Data has been received
    UART_TRANSFER_STATUS_RX_DATA_PRESENT   = (1 << 1),
    // Indicates that the core driver receiver buffer is empty
    UART_TRANSFER_STATUS_RX_EMPTY          = (1 << 2),
    // Indicates that the core driver transmitter buffer is full
    UART_TRANSFER_STATUS_TX_FULL           = (1 << 3),
    // Indicates that the core driver transmitter buffer is empty
    UART_TRANSFER_STATUS_TX_EMPTY          = (1 << 4)
} DEV_UART_TRANSFER_STATUS;


//======================================================================================================================
// @brief UART Driver Queue Status
//======================================================================================================================
typedef union
{
    struct
    {
        uint8_t full:1;
        uint8_t empty:1;
        uint8_t reserved:6;
    }s;
    uint8_t status;
} UART_BYTEQ_STATUS;


//======================================================================================================================
// @brief UART Driver Hardware Instance Object
//======================================================================================================================
typedef struct
{
    uint8_t     *rxTail ;   // pointer to the tail of the RX Byte Q
    uint8_t     *rxHead ;   // pointer to the head of the RX Byte Q
    uint8_t     *txTail ;   // pointer to the tail of the TX Byte Q
    uint8_t     *txHead ;   // pointer to the head of the TX Byte Q
    UART_BYTEQ_STATUS                        rxStatus ;
    UART_BYTEQ_STATUS                        txStatus ;
} UART_OBJECT;


#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif  // _DEV_UART_H_
