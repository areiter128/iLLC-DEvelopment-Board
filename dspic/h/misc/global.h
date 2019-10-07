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
// @file global.h
//
// @brief global defines, structures and variables
//
//======================================================================================================================

#ifndef _GLOBAL_H_
#define	_GLOBAL_H_

#include <stdint.h>
#include "../../../common/h/proto_data.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t    output_current;     // this is the output current of the buck/boost converter
//    uint16_t    reference_voltage;  // this is the target voltage for the control loop
    double      output_voltage;     // this is the output voltage of the buck/boost converter
    double      load;               // data is coming from the pic24 communication
    double      step_load;          // data is coming from the pic24 communication
    uint8_t     fault_overvoltageprotection:1;  // data is coming from the pic24 communication
    uint8_t     fault_overcurrentprotection:1;  // data is coming from the pic24 communication
    uint8_t     fault_reg:1;
    uint8_t     :5;
} ControlLoopData_t;


typedef struct
{
    double      input_voltage;      // input voltage on the board connector
    uint8_t     temperature;        // board temperature - coming from the pic24 communication
} BoardData_t;


typedef struct
{
    ControlLoopData_t buck;
    ControlLoopData_t boost;
    BoardData_t board;
    uint8_t pic24_packet_counter;
    uint8_t :2;
} GlobalData_t;

typedef struct {
    volatile uint16_t vout_boost; 
    volatile uint16_t boost_vref;
//    volatile uint16_t vout_buck;
    volatile uint16_t buck_vref;
}MY_DATA_POINTS_t;

extern volatile MY_DATA_POINTS_t data;
extern GlobalData_t     global_data;
extern protocol_data_t  global_proto24data;

void Global_UpdateBoardData(void);

void Global_UpdateProto24Data(void);


#ifdef	__cplusplus
}
#endif

#endif  // _GLOBAL_H_