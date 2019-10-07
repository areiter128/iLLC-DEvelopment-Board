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
// @file global.c
//
// @brief global defines, structures and variables
//
//======================================================================================================================

#include <stdbool.h>
#include "misc/global.h"
#include "misc/helpers.h"
#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_buck_custom.h"
#include "driver/power_controllers/drv_power_controller_boost_custom.h"

GlobalData_t    global_data;
protocol_data_t global_proto24data;
//uint16_t        global_debug_uart2_rx_counter = 0;
//uint16_t        global_debug_uart2_tx_counter = 0;

volatile MY_DATA_POINTS_t data;

//returns 1/ohms, because the loads are in parallel
double GetLoadBuck(uint8_t bits)
{
    double retval = 0.0;
    if(bits & 4)
        retval = 1.0/6.65;
    if(bits & 2)
        retval = retval + 1.0/8.25;
    if(bits & 1)
        retval = retval + 1.0/33.0;
    return retval;
}

//returns 1/ohms
double GetLoadBoost(uint8_t bits)
{
    double retval = 0.0;
    if(bits & 4)
        retval = 1.0/150.0;
    if(bits & 2)
        retval = retval + 1.0/215.0;
    if(bits & 1)
        retval = retval + 1.0/499.0;
    return retval;
}

void Global_UpdateBoardData(void)
{
    global_data.buck.output_voltage  = Drv_PowerControllerBuck1_GetOutputVoltage();
    global_data.boost.output_voltage = Drv_PowerControllerBoost1_GetOutputVoltage();
    global_data.board.input_voltage = Drv_PowerControllers_GetInputVoltage();
//    global_data.fault_reg_buck  = buck_event; 
//    global_data.fault_reg_boost = boost_event;
    global_data.buck.fault_reg  = false;             //TODO: fake, just for testing
    global_data.boost.fault_reg = false;             //TODO: fake, just for testing
}

void Global_UpdateProto24Data(void)
{
    global_data.board.temperature       = global_proto24data.temperature;
    global_data.buck.load               = GetLoadBuck(global_proto24data.load_status.buck_still);
    global_data.buck.step_load          = GetLoadBuck(global_proto24data.load_status.buck_blink);
    global_data.boost.load              = GetLoadBoost(global_proto24data.load_status.boost_still);
    global_data.boost.step_load         = GetLoadBoost(global_proto24data.load_status.boost_blink);
    global_data.buck.fault_overcurrentprotection    = global_proto24data.fault_status.fault_ocp_buck;
    global_data.buck.fault_overvoltageprotection    = global_proto24data.fault_status.fault_ovp_buck;
    global_data.boost.fault_overcurrentprotection   = global_proto24data.fault_status.fault_ocp_boost;
    global_data.boost.fault_overvoltageprotection   = global_proto24data.fault_status.fault_ovp_boost;
    global_data.pic24_packet_counter++;
}
