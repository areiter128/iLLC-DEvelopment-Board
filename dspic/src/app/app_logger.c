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
// @file app_logger.c
//
// @brief app for sending some data to the pc for testing and visualisation
//
//======================================================================================================================

#include <stdio.h>
#include "device/dev_uart1.h"
#include "misc/global.h"
#include "misc/helpers.h"

void App_Logger_Init(void)
{
    Dev_UART1_Init();
    PrintSerial("\033[2J");
}


void App_Logger_LogData(void)
{
    double volt2;
    PrintSerial("\033[1;1H");
    PrintSerial("==================================\n\r");
    PrintSerial("Packet count = %d\n\r", global_data.pic24_packet_counter);
    PrintSerial("Vbuck  =  %2.2f V\n\r", global_data.buck.output_voltage);
    PrintSerial("Vboost = %2.2f V\n\r", global_data.boost.output_voltage);
    PrintSerial("Vinput =  %2.2f V\n\r", global_data.board.input_voltage);
    PrintSerial("Temp   = %d deg C\n\r",  global_data.board.temperature);
    volt2 = global_data.buck.output_voltage * global_data.buck.output_voltage;            //TODO: seems to be wrong to me, P = U*I
    PrintSerial("P buck = %1.2f W, ", volt2 * global_data.buck.load);
    PrintSerial("step = %1.2f W\n\r", volt2 * global_data.buck.step_load);
    volt2 = global_data.boost.output_voltage * global_data.boost.output_voltage;          //TODO: seems to be wrong to me, P = U*I
    PrintSerial("Pboost = %1.2f W, ", volt2 * global_data.boost.load);
    PrintSerial("step = %1.2f W\n\r", volt2 * global_data.boost.step_load);
    PrintSerial("Buck faults:  OCP %d, OVP %d, REG %d\n\r",
        global_data.buck.fault_overcurrentprotection, global_data.buck.fault_overvoltageprotection, global_data.buck.fault_reg);
    PrintSerial("Boost faults: OCP %d, OVP %d, REG %d\n\r",
        global_data.boost.fault_overcurrentprotection, global_data.boost.fault_overvoltageprotection, global_data.boost.fault_reg);
}


void App_Logger_LogProto24(protocol_data_t *pData)
{
    uint8_t *pBin = (uint8_t *)pData;
    int i;
    PrintSerial("Raw PIC24 incoming data:");
    for(i = 0; i < sizeof(protocol_data_t); i++)
        PrintSerial(" %02x", pBin[i]);
    PrintSerial("\n\r");
}
