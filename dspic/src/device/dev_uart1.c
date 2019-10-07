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
#include <string.h>
#include "device/dev_uart.h"
#include "device/dev_uart1.h"


//======================================================================================================================
// @brief UART1 Driver Queue Length, defines the length of the Transmit and Receive Buffers
//======================================================================================================================
#define UART1_CONFIG_TX_BYTEQ_LENGTH 350
#define UART1_CONFIG_RX_BYTEQ_LENGTH 8


static UART_OBJECT uart1_obj;

//======================================================================================================================
// @brief UART1 Driver Queue, defines the Transmit and Receive Buffers
//======================================================================================================================
static uint8_t uart1_txByteQ[UART1_CONFIG_TX_BYTEQ_LENGTH];
static uint8_t uart1_rxByteQ[UART1_CONFIG_RX_BYTEQ_LENGTH];

static inline void Dev_UART1_Init_PeripheralPinMapping(void)
{
//    RPINR18bits.U1RXR = 0x003D;   //RC13->UART1:U1RX;
//    RPOR14bits.RP60R = 0x0001;   //RC12->UART1:U1TX;
    _U1RXR = 61;    // Uart1 RX connects to Port Pin RP61 / RC13
    _RP60R = 1;     // Port Pin RP60 / RC12 connects to Uart1 TX
}

//======================================================================================================================
// @brief   initializes the UART1 Driver
// @note    call this function at boot up to initialize the UART1 Driver
//======================================================================================================================
void Dev_UART1_Init(void)
{
    Dev_UART1_Init_PeripheralPinMapping();

    IPC47bits.U1EVTIP = 1;      // UART1 Event Interrupt Priority 1
    IPC12bits.U1EIP = 1;        // UART1 Error Interrupt Priority 1
    IPC2bits.U1RXIP = 1;        // UART1 RX Interrupt Priority 1
    IPC3bits.U1TXIP = 1;        // UART1 TX Interrupt Priority 1

    // URXEN disabled; RXBIMD RXBKIF flag when Break makes low-to-high transition after being low for at least 23/11 bit periods; UARTEN enabled; MOD Asynchronous 8-bit UART; UTXBRK disabled; BRKOVR TX line driven by shifter; UTXEN disabled; USIDL disabled; WAKE disabled; ABAUD disabled; BRGH enabled; 
    // Data Bits = 8; Parity = None; Stop Bits = 1 Stop bit sent, 1 checked at RX;
    U1MODE = (0x8080 & ~(1<<15));  // disabling UARTEN bit
    // STSEL 1 Stop bit sent, 1 checked at RX; BCLKMOD disabled; SLPEN disabled; FLO Off; BCLKSEL FOSC; C0EN disabled; RUNOVF disabled; UTXINV disabled; URXINV disabled; HALFDPLX disabled; 
    U1MODEH = 0x400;
    // OERIE disabled; RXBKIF disabled; RXBKIE disabled; ABDOVF disabled; OERR disabled; TXCIE disabled; TXCIF disabled; FERIE disabled; TXMTIE disabled; ABDOVE disabled; CERIE disabled; CERIF disabled; PERIE disabled; 
    U1STA = 0x00;
    // URXISEL RX_ONE_WORD; UTXBE enabled; UTXISEL TX_BUF_EMPTY; URXBE enabled; STPMD disabled; TXWRE disabled; 
    U1STAH = 0x22;
	
    U1BRG = 0x1B1;  // BaudRate = 115200; Frequency = 200000000 Hz; BRG 433; 
    U1BRGH = 0x00;  // BRG 0;
    U1P1 = 0x00;    // P1 0;
    U1P2 = 0x00;    // P2 0;
    U1P3 = 0x00;    // P3 0;
    U1P3H = 0x00;   // P3H 0;
    U1TXCHK = 0x00; // TXCHK 0;
    U1RXCHK = 0x00; // RXCHK 0; 
    U1SCCON = 0x00; // T0PD 1 ETU; PTRCL disabled; TXRPT Retransmit the error byte once; CONV Direct logic; 
    // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; BTCIE disabled; BTCIF disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
    U1SCINT = 0x00;
    U1INT = 0x00;   // ABDIF disabled; WUIF disabled; ABDIE disabled;
    IEC0bits.U1RXIE = 1;
            
     //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U1MODEbits.UARTEN = 1;  // enabling UARTEN bit
    U1MODEbits.UTXEN = 1; 
    U1MODEbits.URXEN = 1;  

    uart1_obj.txHead = uart1_txByteQ;
    uart1_obj.txTail = uart1_txByteQ;
    uart1_obj.rxHead = uart1_rxByteQ;
    uart1_obj.rxTail = uart1_rxByteQ;
    uart1_obj.rxStatus.s.empty = true;
    uart1_obj.txStatus.s.empty = true;
    uart1_obj.txStatus.s.full = false;
    uart1_obj.rxStatus.s.full = false;
}


//======================================================================================================================
// @brief   UART1 TX Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{ 
    if(uart1_obj.txStatus.s.empty)
    {
        IEC0bits.U1TXIE = false;
        return;
    }
    IFS0bits.U1TXIF = false;
    while(!(U1STAHbits.UTXBF == 1))
    {
        U1TXREG = *uart1_obj.txHead;
        uart1_obj.txHead++;
        if(uart1_obj.txHead == (uart1_txByteQ + UART1_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart1_obj.txHead = uart1_txByteQ;
        }
        uart1_obj.txStatus.s.full = false;
        if(uart1_obj.txHead == uart1_obj.txTail)
        {
            uart1_obj.txStatus.s.empty = true;
            break;
        }
    }
}


//======================================================================================================================
// @brief   UART1 RX Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
    while(!(U1STAHbits.URXBE == 1))    //Check for the RX Buffer not empty
    {
        *uart1_obj.rxTail = U1RXREG;
        uart1_obj.rxTail++;
        if(uart1_obj.rxTail == (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
        {
            uart1_obj.rxTail = uart1_rxByteQ;
        }
        uart1_obj.rxStatus.s.empty = false;
        if(uart1_obj.rxTail == uart1_obj.rxHead)    //check if RX Queue is full
        {
            uart1_obj.rxStatus.s.full = true;       //Sets the flag RX full
            break;
        }   
    }
    IFS0bits.U1RXIF = false;
}


//======================================================================================================================
// @brief   UART1 Error Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1EInterrupt( void )
{
    if ((U1STAbits.OERR == 1))
        U1STAbits.OERR = 0;
    IFS3bits.U1EIF = false;
}


//======================================================================================================================
// @brief   UART1 Event Interrupt routine
//======================================================================================================================
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1EVTInterrupt ( void )
{
    // Add your own handling for UART events here
    IFS11bits.U1EVTIF = false;
}

//======================================================================================================================
// @brief   UART1 Read function for one byte
// @note    use this function to read one byte from the UART1 Receive Queue
// @note    before using this function you have to make sure that the Receive Queue is not empty
//======================================================================================================================
uint8_t Dev_UART1_Read(void)
{
    uint8_t data = 0;
    data = *uart1_obj.rxHead;
    uart1_obj.rxHead++;
    if (uart1_obj.rxHead == (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
        uart1_obj.rxHead = uart1_rxByteQ;
    if (uart1_obj.rxHead == uart1_obj.rxTail)
        uart1_obj.rxStatus.s.empty = true;
    uart1_obj.rxStatus.s.full = false;
    return data;
}


//======================================================================================================================
// @brief   UART1 Read function for multiple bytes
// @note    use this function to read some bytes from the UART1 Receive Queue
// @param   buffer - Buffer into which the data read from the UART1
// @param   numbytes - Total number of bytes that need to be read from the UART1,
//          (must be equal to or less than the size of the buffer)
// @returns the number of bytes transferred
// @preconditions   UART1_Initializer function should have been called before calling this function
//======================================================================================================================
unsigned int Dev_UART1_ReadBuffer(uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart1_obj.rxStatus.s.empty)
            break;
        else
            buffer[numBytesRead++] = Dev_UART1_Read();
    }
    return numBytesRead;
}


//======================================================================================================================
// @brief   UART1 Write function for one byte
// @note    use this function to write one byte to the UART1 Transfer Queue
// @note    before using this function, make sure the buffer is not full
// @param   byte - Data byte to write to the UART1
// @preconditions
//          UART1_Initializer function should have been called before calling this function.
//          The transfer status should be checked to see if transmitter is not full before calling this function.
//======================================================================================================================
void Dev_UART1_Write(const uint8_t byte)
{
    IEC0bits.U1TXIE = false;
    *uart1_obj.txTail = byte;
    uart1_obj.txTail++;
    if (uart1_obj.txTail == (uart1_txByteQ + UART1_CONFIG_TX_BYTEQ_LENGTH))
        uart1_obj.txTail = uart1_txByteQ;
    uart1_obj.txStatus.s.empty = false;
    if (uart1_obj.txHead == uart1_obj.txTail)
        uart1_obj.txStatus.s.full = true;
    IEC0bits.U1TXIE = true;
}


//======================================================================================================================
// @brief   UART1 Write function for multiple bytes
// @note    use this function to write multiple bytes to the UART1 Transfer Queue
// @returns the number of bytes transferred into the UART1 Transfer Queue
// @preconditions   UART1_Initializer function should have been called before calling this function
//======================================================================================================================
unsigned int Dev_UART1_WriteBuffer(const uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesWritten = 0 ;
    while ( numBytesWritten < ( bufLen ))
    {
        if((uart1_obj.txStatus.s.full))
            break;
        else
            Dev_UART1_Write(buffer[numBytesWritten++]);
    }
    return numBytesWritten;
}


//======================================================================================================================
// @brief   UART1 Write function for multiple bytes
// @note    use this function to write multiple bytes to the UART1 Transfer Queue
// @note    if the write queue is full this function will block until all data bytes are written into the Transfer Queue
// @preconditions   UART1_Initializer function should have been called before calling this function
//======================================================================================================================
void Dev_UART1_WriteBufferBlocking(const uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesWritten = 0 ;
    while ( numBytesWritten < ( bufLen ))
    {
        while((uart1_obj.txStatus.s.full));     //wait until there is space for the next byte
        Dev_UART1_Write(buffer[numBytesWritten++]);
    }
}

//======================================================================================================================
// @brief   Blocking UART1 Write function for a string
// @note    use this function to write multiple bytes to the UART1 Transfer Queue
// @note    if the write queue is full this function will block until all data bytes are written into the Transfer Queue
// @preconditions   UART1_Initializer function should have been called before calling this function
//======================================================================================================================
void Dev_UART1_WriteStringBlocking(const char *str)
{
    Dev_UART1_WriteBufferBlocking((uint8_t*)str, strlen(str));
}


//======================================================================================================================
// @brief   Returns the Transfer status of the UART1 TX and RX Queue
// @description
//  This returns the transmitter and receiver transfer status. The returned status may contain a value with more
//  than one of the bits specified in the DEV_UART1_TRANSFER_STATUS enumeration set.
//  The caller should perform an "AND" with the bit of interest and verify if the result is non-zero
//  to verify the desired status bit.
//  @returns    A DEV_UART1_TRANSFER_STATUS value describing the current status of the transfer.
//  @preconditions  UART1_Initializer function should have been called before calling this function
//======================================================================================================================
DEV_UART1_TRANSFER_STATUS Dev_UART1_TransferStatusGet (void)
{
    DEV_UART1_TRANSFER_STATUS status = 0;
    if(uart1_obj.txStatus.s.full)
        status |= UART1_TRANSFER_STATUS_TX_FULL;
    if(uart1_obj.txStatus.s.empty)
        status |= UART1_TRANSFER_STATUS_TX_EMPTY;
    if(uart1_obj.rxStatus.s.full)
        status |= UART1_TRANSFER_STATUS_RX_FULL;
    if(uart1_obj.rxStatus.s.empty)
        status |= UART1_TRANSFER_STATUS_RX_EMPTY;
    else
        status |= UART1_TRANSFER_STATUS_RX_DATA_PRESENT;
    return status;
}


//======================================================================================================================
// @brief   Returns the character from the UART1 RX Queue with the provided offset without extracting it
// @param   offset - UART1 RX buffer peek position.
//          Offset input range should be 0 to (UART1_CONFIG_RX_BYTEQ_LENGTH - 1)
// @note    Make sure that for the given offset there are enough bytes in the RX Queue
//======================================================================================================================
uint8_t Dev_UART1_Peek(uint16_t offset)
{
    if( (uart1_obj.rxHead + offset) >= (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
        return uart1_rxByteQ[offset - (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH - uart1_obj.rxHead)];
    else
        return *(uart1_obj.rxHead + offset);
}


//======================================================================================================================
// @brief   Returns the character from the UART1 RX Queue with the provided offset without extracting it
// @note    this function validates all the possible conditions for getting the character wanted
// @param   
//  dataByte - Data byte to be read from UART1 RX buffer based on offset position.
//  offset   - UART1 RX buffer peek position. Offset input range is should be 0 to (UART1_CONFIG_RX_BYTEQ_LENGTH - 1)
// @returns true, if there was a character retrieved in dataByte
//======================================================================================================================
bool Dev_UART1_PeekSafe(uint8_t *dataByte, uint16_t offset)
{
    uint16_t index = 0;
    bool status = true;
    
    if((offset >= UART1_CONFIG_RX_BYTEQ_LENGTH) || (uart1_obj.rxStatus.s.empty) || (!dataByte))
    {
        status = false;
    }
    else
    {
        //Compute the offset buffer overflow range
        index = ((uart1_obj.rxHead - uart1_rxByteQ) + offset) % UART1_CONFIG_RX_BYTEQ_LENGTH;
        
        // Check for offset input value range is valid or invalid.
        // If the range is invalid, then status set to false else true.
        if(uart1_obj.rxHead < uart1_obj.rxTail) 
        {
            if((uart1_obj.rxHead + offset) > (uart1_obj.rxTail - 1))
                status = false;
        }
        else if(uart1_obj.rxHead > uart1_obj.rxTail)
        {
            if((uart1_rxByteQ + index) > (uart1_obj.rxTail - 1))
                status = false;
        }
        if(status == true)
            *dataByte = Dev_UART1_Peek(index);
    }
    return status;
}


//======================================================================================================================
// @brief   Returns the amount of characters in the receive buffer
// @returns amount of characters in the receive buffer
//======================================================================================================================
unsigned int Dev_UART1_ReceiveBufferSizeGet(void)
{
    if(!uart1_obj.rxStatus.s.full)
    {
        if(uart1_obj.rxHead > uart1_obj.rxTail)
            return(uart1_obj.rxHead - uart1_obj.rxTail);
        else
            return(UART1_CONFIG_RX_BYTEQ_LENGTH - (uart1_obj.rxTail - uart1_obj.rxHead));
    }
    return 0;
}


//======================================================================================================================
// @brief   Returns the amount of characters in the transmit buffer
// @returns amount of characters in the transmit buffer
//======================================================================================================================
unsigned int Dev_UART1_TransmitBufferSizeGet(void)
{
    if(!uart1_obj.txStatus.s.full)
    { 
        if(uart1_obj.txHead > uart1_obj.txTail)
            return(uart1_obj.txHead - uart1_obj.txTail);
        else
            return(UART1_CONFIG_TX_BYTEQ_LENGTH - (uart1_obj.txTail - uart1_obj.txHead));
    }
    return 0;
}


//======================================================================================================================
// @brief   Returns if the Receive Buffer is empty or not
// @returns
//  True if the receive buffer is empty
//  False if the receive buffer is not empty
//======================================================================================================================
bool Dev_UART1_ReceiveBufferIsEmpty(void)
{
    return((bool) uart1_obj.rxStatus.s.empty);
}


//======================================================================================================================
// @brief   Returns if the Transmit Buffer is empty or not
// @Returns
//  True if the transmit buffer is full
//  False if the transmit buffer is not full
//======================================================================================================================
bool Dev_UART1_TransmitBufferIsFull(void)
{
    return((bool) uart1_obj.txStatus.s.full);
}


//======================================================================================================================
// @brief   Returns the 32 bit transmitter and receiver hardware status of the UART1
// @note    The caller should perform an "AND" with the bit of interest and verify if the result
//          is non-zero (as shown in the example) to verify the desired status bit.
// @returns 32 bit value describing the current status of the transfer
// @preconditions   UART1_Initializer function should have been called before calling this function
//======================================================================================================================
uint32_t Dev_UART1_StatusGet(void)
{
    uint32_t statusReg = U1STAH;
    return ((statusReg << 16 ) | U1STA);
}

