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
// @file dev_uart1.c
//
// @brief contains the uart1 driver
//
//======================================================================================================================

#include <stdbool.h>
#include <stdint.h>
#include "device/dev_uart.h"
#include "device/dev_uart2.h"


//======================================================================================================================
// @brief UART2 Driver Queue Length, defines the length of the Transmit and Receive Buffers
//======================================================================================================================
#define UART2_CONFIG_TX_BYTEQ_LENGTH 32
#define UART2_CONFIG_RX_BYTEQ_LENGTH 32


static UART_OBJECT uart2_obj ;

//======================================================================================================================
// @brief UART2 Driver Queue, defines the Transmit and Receive Buffers
//======================================================================================================================
static uint8_t uart2_txByteQ[UART2_CONFIG_TX_BYTEQ_LENGTH];
static uint8_t uart2_rxByteQ[UART2_CONFIG_RX_BYTEQ_LENGTH];

static inline void Dev_UART2_Init_PeripheralPinMapping(void)
{
//    RPINR19bits.U2RXR = 0x003B;   //RC11->UART2:U2RX;
//    RPOR13bits.RP58R = 0x0003;   //RC10->UART2:U2TX;
    _U2RXR = 59;    // Uart2 RX connects to Port Pin RP59 / RC11
    _RP58R = 3;     // Port Pin RPRP58 / RC10 connects to Uart2 TX
}

//======================================================================================================================
// @brief   initializes the UART2 Driver
// @note    call this function at boot up to initialize the UART2 Driver
//======================================================================================================================
void Dev_UART2_Init(void)
{
    Dev_UART2_Init_PeripheralPinMapping();
        
    IPC47bits.U2EVTIP = 1;      // UART2 Event Interrupt Priority 1
    IPC12bits.U2EIP = 1;        // UART2 Error Interrupt Priority 1
    IPC6bits.U2RXIP = 1;        // UART2 RX Interrupt Priority 1
    IPC7bits.U2TXIP = 1;        // UART2 TX Interrupt Priority 1

    // URXEN disabled; RXBIMD RXBKIF flag when Break makes low-to-high transition after being low for at least 23/11 bit periods; UARTEN enabled; MOD Asynchronous 8-bit UART; UTXBRK disabled; BRKOVR TX line driven by shifter; UTXEN disabled; USIDL disabled; WAKE disabled; ABAUD disabled; BRGH enabled; 
    // Data Bits = 8; Parity = None; Stop Bits = 1 Stop bit sent, 1 checked at RX;
    U2MODE = (0x8080 & ~(1<<15));  // disabling UARTEN bit
    // STSEL 1 Stop bit sent, 1 checked at RX; BCLKMOD disabled; SLPEN disabled; FLO Off; BCLKSEL FOSC; C0EN disabled; RUNOVF disabled; UTXINV disabled; URXINV disabled; HALFDPLX disabled; 
    U2MODEH = 0x400;
    // OERIE disabled; RXBKIF disabled; RXBKIE disabled; ABDOVF disabled; OERR disabled; TXCIE disabled; TXCIF disabled; FERIE disabled; TXMTIE disabled; ABDOVE disabled; CERIE disabled; CERIF disabled; PERIE disabled; 
    U2STA = 0x00;
    // URXISEL RX_ONE_WORD; UTXBE enabled; UTXISEL TX_BUF_EMPTY; URXBE enabled; STPMD disabled; TXWRE disabled; 
    U2STAH = 0x22;
    
	
    U2BRG = 0xD8;   // BaudRate = 230400; Frequency = 200000000 Hz; BRG 216; 
    U2BRGH = 0x00;  // BRG 0;
    U2P1 = 0x00;    // P1 0;
    U2P2 = 0x00;    // P2 0;
    U2P3 = 0x00;    // P3 0;
    U2P3H = 0x00;   // P3H 0; 
    U2TXCHK = 0x00; // TXCHK 0; 
    U2RXCHK = 0x00; // RXCHK 0;
    U2SCCON = 0x00; // T0PD 1 ETU; PTRCL disabled; TXRPT Retransmit the error byte once; CONV Direct logic;
    // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; BTCIE disabled; BTCIF disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
    U2SCINT = 0x00;
    U2INT = 0x00;   // ABDIF disabled; WUIF disabled; ABDIE disabled; 
    IEC1bits.U2RXIE = 1;

    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U2MODEbits.UARTEN = 1;  // enabling UARTEN bit
    U2MODEbits.UTXEN = 1; 
    U2MODEbits.URXEN = 1;  

    uart2_obj.txHead = uart2_txByteQ;
    uart2_obj.txTail = uart2_txByteQ;
    uart2_obj.rxHead = uart2_rxByteQ;
    uart2_obj.rxTail = uart2_rxByteQ;
    uart2_obj.rxStatus.s.empty = true;
    uart2_obj.txStatus.s.empty = true;
    uart2_obj.txStatus.s.full = false;
    uart2_obj.rxStatus.s.full = false;
}


//======================================================================================================================
// @brief   UART2 TX Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2TXInterrupt ( void )
{
    if(uart2_obj.txStatus.s.empty)
    {
        IEC1bits.U2TXIE = false;
        return;
    }
    IFS1bits.U2TXIF = false;
    while(!(U2STAHbits.UTXBF == 1))
    {
        U2TXREG = *uart2_obj.txHead;
        uart2_obj.txHead++;
        if(uart2_obj.txHead == (uart2_txByteQ + UART2_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart2_obj.txHead = uart2_txByteQ;
        }
        uart2_obj.txStatus.s.full = false;
        if(uart2_obj.txHead == uart2_obj.txTail)
        {
            uart2_obj.txStatus.s.empty = true;
            break;
        }
    }
}


//======================================================================================================================
// @brief   UART2 RX Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2RXInterrupt( void )
{
    while(!(U2STAHbits.URXBE == 1))    //Check for the RX Buffer not empty
    {
        *uart2_obj.rxTail = U2RXREG;
        uart2_obj.rxTail++;
        if(uart2_obj.rxTail == (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH))
        {
            uart2_obj.rxTail = uart2_rxByteQ;
        }
        uart2_obj.rxStatus.s.empty = false;
        if(uart2_obj.rxTail == uart2_obj.rxHead)
        {
            //Sets the flag RX full
            uart2_obj.rxStatus.s.full = true;
            break;
        }   
    }
    IFS1bits.U2RXIF = false;
}


//======================================================================================================================
// @brief   UART2 Error Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2EInterrupt( void )
{
    if ((U2STAbits.OERR == 1))
        U2STAbits.OERR = 0;
    IFS3bits.U2EIF = false;
}


//======================================================================================================================
// @brief   UART2 Event Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2EVTInterrupt ( void )
{
    // Add your own handling for UART events here
    IFS11bits.U2EVTIF = false;
}

//======================================================================================================================
// @brief   UART2 Read function for one byte
// @note    use this function to read one byte from the UART2 Receive Queue
// @note    before using this function you have to make sure that the Receive Queue is not empty
//======================================================================================================================
uint8_t Dev_UART2_Read(void)
{
    uint8_t data = 0;
    data = *uart2_obj.rxHead;
    uart2_obj.rxHead++;
    if (uart2_obj.rxHead == (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH))
        uart2_obj.rxHead = uart2_rxByteQ;
    if (uart2_obj.rxHead == uart2_obj.rxTail)
        uart2_obj.rxStatus.s.empty = true;
    uart2_obj.rxStatus.s.full = false;
    return data;
}


//======================================================================================================================
// @brief   UART2 Read function for multiple bytes
// @note    use this function to read some bytes from the UART2 Receive Queue
// @param   buffer - Buffer into which the data read from the UART2
// @param   numbytes - Total number of bytes that need to be read from the UART2,
//          (must be equal to or less than the size of the buffer)
// @returns the number of bytes transferred
// @preconditions   UART2_Initializer function should have been called before calling this function
//======================================================================================================================
unsigned int Dev_UART2_ReadBuffer(uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart2_obj.rxStatus.s.empty)
            break;
        else
            buffer[numBytesRead++] = Dev_UART2_Read();
    }
    return numBytesRead ;
}


//======================================================================================================================
// @brief   UART2 Write function for one byte
// @note    use this function to write one byte to the UART2 Transfer Queue
// @note    before using this function, make sure the buffer is not full
// @param   byte - Data byte to write to the UART2
// @preconditions
//          UART2_Initializer function should have been called before calling this function.
//          The transfer status should be checked to see if transmitter is not full before calling this function.
//======================================================================================================================
void Dev_UART2_Write(const uint8_t byte)
{
    IEC1bits.U2TXIE = false;
    *uart2_obj.txTail = byte;
    uart2_obj.txTail++;
    if (uart2_obj.txTail == (uart2_txByteQ + UART2_CONFIG_TX_BYTEQ_LENGTH))
        uart2_obj.txTail = uart2_txByteQ;
    uart2_obj.txStatus.s.empty = false;
    if (uart2_obj.txHead == uart2_obj.txTail)
        uart2_obj.txStatus.s.full = true;
    IEC1bits.U2TXIE = true;
}


//======================================================================================================================
// @brief   UART2 Write function for multiple bytes
// @note    use this function to write multiple bytes to the UART2 Transfer Queue
// @returns the number of bytes transferred into the UART2 Transfer Queue
// @preconditions   UART2_Initializer function should have been called before calling this function
//======================================================================================================================
unsigned int Dev_UART2_WriteBuffer(const uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesWritten = 0 ;
    while ( numBytesWritten < ( bufLen ))
    {
        if((uart2_obj.txStatus.s.full))
            break;
        else
            Dev_UART2_Write(buffer[numBytesWritten++]);
    }
    return numBytesWritten ;
}


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
DEV_UART2_TRANSFER_STATUS Dev_UART2_TransferStatusGet(void)
{
    DEV_UART2_TRANSFER_STATUS status = 0;
    if(uart2_obj.txStatus.s.full)
        status |= UART2_TRANSFER_STATUS_TX_FULL;
    if(uart2_obj.txStatus.s.empty)
        status |= UART2_TRANSFER_STATUS_TX_EMPTY;
    if(uart2_obj.rxStatus.s.full)
        status |= UART2_TRANSFER_STATUS_RX_FULL;
    if(uart2_obj.rxStatus.s.empty)
        status |= UART2_TRANSFER_STATUS_RX_EMPTY;
    else
        status |= UART2_TRANSFER_STATUS_RX_DATA_PRESENT;
    return status;
}


//======================================================================================================================
// @brief   Returns the character from the UART2 RX Queue with the provided offset without extracting it
// @param   offset - UART2 RX buffer peek position.
//          Offset input range should be 0 to (UART2_CONFIG_RX_BYTEQ_LENGTH - 1)
// @note    Make sure that for the given offset there are enough bytes in the RX Queue
//======================================================================================================================
uint8_t Dev_UART2_Peek(uint16_t offset)
{
    if( (uart2_obj.rxHead + offset) >= (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH))
        return uart2_rxByteQ[offset - (uart2_rxByteQ + UART2_CONFIG_RX_BYTEQ_LENGTH - uart2_obj.rxHead)];
    else
        return *(uart2_obj.rxHead + offset);
}


//======================================================================================================================
// @brief   Returns the character from the UART2 RX Queue with the provided offset without extracting it
// @note    this function validates all the possible conditions for getting the character wanted
// @param   
//  dataByte - Data byte to be read from UART2 RX buffer based on offset position.
//  offset   - UART2 RX buffer peek position. Offset input range is should be 0 to (UART2_CONFIG_RX_BYTEQ_LENGTH - 1)
// @returns true, if there was a character retrieved in dataByte
//======================================================================================================================
bool Dev_UART2_PeekSafe(uint8_t *dataByte, uint16_t offset)
{
    uint16_t index = 0;
    bool status = true;
    
    if((offset >= UART2_CONFIG_RX_BYTEQ_LENGTH) || (uart2_obj.rxStatus.s.empty) || (!dataByte))
    {
        status = false;
    }
    else
    {
        //Compute the offset buffer overflow range
        index = ((uart2_obj.rxHead - uart2_rxByteQ) + offset) % UART2_CONFIG_RX_BYTEQ_LENGTH;
        
        // Check for offset input value range is valid or invalid.
        // If the range is invalid, then status set to false else true.
        if(uart2_obj.rxHead < uart2_obj.rxTail) 
        {
            if((uart2_obj.rxHead + offset) > (uart2_obj.rxTail - 1))
                status = false;
        }
        else if(uart2_obj.rxHead > uart2_obj.rxTail)
        {
            if((uart2_rxByteQ + index) > (uart2_obj.rxTail - 1))
                status = false;
        }
        if(status == true)
            *dataByte = Dev_UART2_Peek(index);
    }
    return status;
}


//======================================================================================================================
// @brief   Returns the amount of characters in the receive buffer
// @returns amount of characters in the receive buffer
//======================================================================================================================
unsigned int Dev_UART2_ReceiveBufferSizeGet(void)
{
    if(!uart2_obj.rxStatus.s.full)
    {
        if(uart2_obj.rxHead > uart2_obj.rxTail)
            return(uart2_obj.rxHead - uart2_obj.rxTail);
        else
            return(UART2_CONFIG_RX_BYTEQ_LENGTH - (uart2_obj.rxTail - uart2_obj.rxHead));
    }
    return 0;
}


//======================================================================================================================
// @brief   Returns the amount of characters in the transmit buffer
// @returns amount of characters in the transmit buffer
//======================================================================================================================
unsigned int DEV_UART2_TransmitBufferSizeGet(void)
{
    if(!uart2_obj.txStatus.s.full)
    { 
        if(uart2_obj.txHead > uart2_obj.txTail)
            return(uart2_obj.txHead - uart2_obj.txTail);
        else
            return(UART2_CONFIG_TX_BYTEQ_LENGTH - (uart2_obj.txTail - uart2_obj.txHead));
    }
    return 0;
}


//======================================================================================================================
// @brief   Returns if the Receive Buffer is empty or not
// @returns
//  True if the receive buffer is empty
//  False if the receive buffer is not empty
//======================================================================================================================
bool Dev_UART2_ReceiveBufferIsEmpty (void)
{
    return((bool) uart2_obj.rxStatus.s.empty);
}


//======================================================================================================================
// @brief   Returns if the Transmit Buffer is empty or not
// @Returns
//  True if the transmit buffer is full
//  False if the transmit buffer is not full
//======================================================================================================================
bool Dev_UART2_TransmitBufferIsFull(void)
{
    return((bool) uart2_obj.txStatus.s.full);
}


//======================================================================================================================
// @brief   Returns the 32 bit transmitter and receiver hardware status of the UART2
// @note    The caller should perform an "AND" with the bit of interest and verify if the result
//          is non-zero (as shown in the example) to verify the desired status bit.
// @returns 32 bit value describing the current status of the transfer
// @preconditions   UART2_Initializer function should have been called before calling this function
//======================================================================================================================
uint32_t Dev_UART2_StatusGet (void)
{
    uint32_t statusReg = U2STAH;
    return ((statusReg << 16 ) | U2STA);
}

