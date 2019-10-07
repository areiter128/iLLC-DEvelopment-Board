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
// @file app_display.h
//
// @brief main statemachine for buttons and display
//
//======================================================================================================================

#ifndef _APP_DISPLAY_H_
#define	_APP_DISPLAY_H_

#include <stdint.h>
#include "device/dev_lcd.h"

#ifdef	__cplusplus
extern "C" {
#endif

//======================================================================================================================
//  @brief  this function contains the display task and needs to be called from outside every 100 ms
//  @note   the 100 ms interval does not need to be exact. it should be roughly 100 ms
//======================================================================================================================
void App_Display_Task_100ms(void);


#ifdef	__cplusplus
}
#endif

#endif // _APP_DISPLAY_H_