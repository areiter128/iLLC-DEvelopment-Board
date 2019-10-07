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
// @file app_proto24.c
//
// @brief communication application with the pic24
//
//======================================================================================================================

/* Including header files */
#include <xc.h>
#include <stdbool.h>
#include <string.h>
#include "misc/delay.h"
#include "device/dev_uart2.h"
#include "app/app_proto24.h"
#include "misc/interrupt.h"

#define RX_TIMEOUT  100  // ticks of 1ms = 100ms

static volatile bool    timeout_flag;
static volatile uint8_t timeout_counter;
static volatile bool    new_data_is_avaiable = false;
protocol_data_t         proto24data;


void App_Proto24_Init(void)
{
    INTERRUPT_GlobalDisable();
    Dev_UART2_Init();
    timeout_flag = false;
    timeout_counter = 0;
    memset(&proto24data, 0, PROTO_P24_DSP_LEN);
    INTERRUPT_GlobalEnable();
}

// returns true if something new has been received over UART, otherwise false
bool App_Proto24_IsNewDataAvailable(void)
{
    return new_data_is_avaiable;
}

void App_Proto24_GetData(protocol_data_t * newdata)
{
    memcpy(newdata, &proto24data, sizeof(proto24data));
    new_data_is_avaiable = false;
}

void App_Proto24_Send(protocol_event_t p24_event)
{
    Dev_UART2_Write(PROTO_SOF);
    Dev_UART2_WriteBuffer((uint8_t *)&p24_event, PROTO_DSP_P24_LEN);
}

void App_Proto24_Task_1ms(void)
{
    protocol_data_t temp_proto24data;
    uint8_t buff_len, transfer_len;
    static bool WSOF = true;

    //first we check if we ran into a timeout
    if(timeout_counter) // started
    {
        if(timeout_counter >= RX_TIMEOUT) // timer expired
        {
            timeout_flag = true; // trigger the flag
            timeout_counter = 0; // and clear counter
        }
        else
        {
            timeout_counter++;
        }
    }

    if(WSOF) // waiting for SOF - start of frame
    {
        if(!Dev_UART2_ReceiveBufferIsEmpty())
        {
            uint8_t byte = Dev_UART2_Read();
            if(byte == PROTO_SOF)
            {
                WSOF = false; // no longer waiting for SOF, now can load the packet
                timeout_counter++; // start timer for timeout
            }
        }
    }
    else
    {
        buff_len = 32 - (uint8_t)Dev_UART2_ReceiveBufferSizeGet(); // 32 must be replaced by UART2 API call
        if(buff_len >= PROTO_P24_DSP_LEN)
        {
            transfer_len = Dev_UART2_ReadBuffer((uint8_t *)&temp_proto24data, PROTO_P24_DSP_LEN);
            WSOF = true; // wait again for SOF
            if(transfer_len == PROTO_P24_DSP_LEN)
            {
                memcpy(&proto24data, &temp_proto24data, sizeof(proto24data));
                new_data_is_avaiable = true;
            }
        }
        // if not accumulated enough bytes in the RX uart buffer, then try again next Proto24Check cycle
    }
    if(timeout_flag)
    {
        WSOF = true;
        timeout_flag = false;
    }
}
