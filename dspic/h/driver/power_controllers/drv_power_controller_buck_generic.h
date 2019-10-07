//======================================================================================================================
// Copyright(c) 2019 Microchip Technology Inc. and its subsidiaries.
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
// @file drv_power_controller_buck_generic.h
//
// @brief power controller functions for buck converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date July 9, 2019, 1:10 PM
//======================================================================================================================

#ifndef _DRV_POWER_CONTROLLER_BUCK_GENERIC_H_
#define	_DRV_POWER_CONTROLLER_BUCK_GENERIC_H_

#include <stdint.h>
#include <stdbool.h>
//#include "driver/power_controllers/drv_power_controllers.h"

#ifdef	__cplusplus
extern "C" {
#endif // __cplusplus


//======================================================================================================================
// @brief   Initializes all peripherals and data structures of the buck controller like PWM, ADC, DAC, CMP etc.
// @note    call this during booting up the system before you call anything else or the Power Controller
//          you must have an object with initialized data!
//======================================================================================================================
extern void Drv_PowerControllerBuck_Init(POWER_CONTROLLER_DATA_t*  pPCBuckDataObj, bool autostart);

//======================================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//======================================================================================================================
extern void Drv_PowerControllerBuck_SetOutputVoltageReference(POWER_CONTROLLER_DATA_t* pPCData, double newVoltRef);

//======================================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//======================================================================================================================
extern void Drv_PowerControllerBuck_SetOutputVoltageReference_mV(POWER_CONTROLLER_DATA_t* pPCData, uint32_t newVoltRef_mV);

//======================================================================================================================
// @brief   returns the raw ADC value for the Output Voltage
//======================================================================================================================
extern uint16_t Drv_PowerControllerBuck_GetOutputVoltageRaw(POWER_CONTROLLER_DATA_t* pPCData);

//======================================================================================================================
// @brief   Task that runs all the necessary things to do like soft start and voltage monitoring
// @note    call this every 100us from your main scheduler to ensure the right timing
//======================================================================================================================
extern void     Drv_PowerControllerBuck_Task_100us(POWER_CONTROLLER_DATA_t* pPCData);

#ifdef	__cplusplus
}
#endif // __cplusplus
#endif	// _DRV_POWER_CONTROLLER_BUCK_GENERIC_H_

