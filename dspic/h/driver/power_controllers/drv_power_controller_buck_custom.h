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
// @file drv_power_controller_buck_custom.h
//
// @brief power controller functions for buck converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date July 9, 2019, 1:10 PM
//=======================================================================================================

#ifndef _DRV_POWER_CONTROLLER_BUCK_CUSTOM_H_
#define	_DRV_POWER_CONTROLLER_BUCK_CUSTOM_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif // __cplusplus
    


extern POWER_CONTROLLER_DATA_t pwrCtrlBuck1_Data;      // data instance for the buck converter

//=======================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBuck1_GetOutputVoltage();

//=======================================================================================================
// @brief   sets the Initial Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed initial reference voltage 
//=======================================================================================================
void Drv_PowerControllerBoost1_SetInitialOutputVoltageReference(double newVoltRef);

//=======================================================================================================
// @brief   sets the Initial Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed initial reference voltage
//=======================================================================================================
void Drv_PowerControllerBoost1_SetInitialOutputVoltageReference_mV(uint32_t newVoltRef_mV);

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//=======================================================================================================
void Drv_PowerControllerBuck1_SetOutputVoltageReference(double newVoltRef);

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//=======================================================================================================
void Drv_PowerControllerBuck1_SetOutputVoltageReference_mV(uint32_t newVoltRef_mV);

//=======================================================================================================
// @brief   Initializes the Buck Power Converter - Instance 1
// @note    In this routine all the application specific custom functions are implemented
//=======================================================================================================
void Drv_PowerControllerBuck1_Init(bool autostart);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_InitPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_InitAuxiliaryPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_InitACMP(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_InitADC(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_LaunchPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_LaunchAuxiliaryPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck1_LaunchACMP(void);


void Drv_PowerControllerBuck2_Init(bool autostart);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck2_InitPWM(void);

//=======================================================================================================
volatile uint16_t Drv_PowerControllerBuck2_LaunchPWM(void);


           
#ifdef	__cplusplus
}
#endif // __cplusplus
#endif	// _DRV_POWER_CONTROLLER_BUCK_CUSTOM_H_

