//=======================================================================================================
// Copyright(c) 2019 Microchip Technology Inc. and its subsidiaries.
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
// @file app_fault_handling.h
//
// @brief fault monitoring and handling
//
// @author M52409
//
// @date 2019-09-24
//
//=======================================================================================================


#ifndef _APP_FAULT_HANDLING_H_
#define _APP_FAULT_HANDLING_H_

#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif

//======================================================================================================================
//  @brief  these defines are the fault bits that are combined in a uint16 value
//  @note   use the function App_Fault_Handling_GetFaults to retrieve the whole uint16 value
//======================================================================================================================
#define FAULT_GENERAL                   1
#define FAULT_SUPPLY_OVERVOLTAGE        2
#define FAULT_SUPPLY_UNDERVOLTAGE       4
#define FAULT_OVERTEMPERATURE           8
#define FAULT_BUCK_OVERVOLTAGE          16
#define FAULT_BUCK_UNDERVOLTAGE         32
#define FAULT_BUCK_OVERCURRENT          32
#define FAULT_BOOST_OVERVOLTAGE         64
#define FAULT_BOOST_UNDERVOLTAGE        128
#define FAULT_BOOST_OVERCURRENT         256

//======================================================================================================================
//  @brief  this function initializes the fault handling
//  @note   
//======================================================================================================================
void App_Fault_Handling_Init(void);

//======================================================================================================================
//  @brief  this function returns the combined fault bits
//  @note   use the defined fault bit defines above to filter out the bit of interest
//======================================================================================================================
uint16_t App_Fault_Handling_GetFaults(void);

//======================================================================================================================
//  @brief  this function does the fault handling every 1 ms
//  @note   call this function in your main scheduler every 1ms
//======================================================================================================================
void App_Fault_Handling_Task_1ms(void);

//======================================================================================================================
//  @brief  this function does the fault handling every 1 ms
//  @note   call this function in your main scheduler every 1ms
//======================================================================================================================
void App_Fault_Handling_Task_100us(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif  //_APP_FAULT_HANDLING_H_