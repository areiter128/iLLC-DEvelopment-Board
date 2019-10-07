//=======================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives
// exclusively with Microchip products. It is your responsibility to comply with third party license
// terms applicable to your use of third-party software (including open source software) that may
// accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
// APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND
// FITNESS FOR A PARTICULAR PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
// LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
// MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT
// ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
// EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//=======================================================================================================

//=======================================================================================================
// @file drv_power_controller_boost_custom.h
//
// @brief power controller functions for boost converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date September 4, 2019, 1:10 PM
//=======================================================================================================


#ifndef _DRV_POWER_CONTROLLER_BOOST_CUSTOM_H_
#define	_DRV_POWER_CONTROLLER_BOOST_CUSTOM_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif // __cplusplus

extern POWER_CONTROLLER_DATA_t pwrCtrlBoost1_Data;      // data instance for the boost converter

//=======================================================================================================
// @brief   returns the Input Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBoost1_GetInputVoltage();

//=======================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBoost1_GetOutputVoltage();

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//=======================================================================================================
void Drv_PowerControllerBoost1_SetOutputVoltageReference(double newVoltRef);

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//=======================================================================================================
void Drv_PowerControllerBoost1_SetOutputVoltageReference_mV(uint32_t newVoltRef_mV);
    
//=======================================================================================================
// @brief   Initializes the Boost Power Converter - Instance 1
// @note    In this routine all the application specific custom functions are implemented
//=======================================================================================================
void Drv_PowerControllerBoost1_Init(bool autostart);
    
//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_InitPWM(void);    
 
//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_LaunchPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_InitAuxiliaryPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_LaunchAuxiliaryPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_InitACMP(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_LaunchOPA(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_LaunchACMP(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost1_InitADC(void);


void Drv_PowerControllerBoost2_Init(bool autostart);
    
//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost2_InitPWM(void);    
 
//=======================================================================================================
volatile uint16_t Drv_PowerControllerBoost2_LaunchPWM(void);


#ifdef	__cplusplus
}
#endif // __cplusplus 

#endif	// _DRV_POWER_CONTROLLER_BOOST_CUSTOM_H_

