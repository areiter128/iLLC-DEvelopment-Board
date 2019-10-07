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
// @file helpers.c
//
// @brief misc functions that are helpful for the project and there is no suitable place elsewhere
//
//======================================================================================================================

//#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include "device/dev_lcd.h"
#include "device/dev_uart1.h"


/*
void PrintLcd(uint8_t linenumber, char *format, ...)
{
    char tmpstr[TEMPSTR_LCD_SIZE];
    va_list args;

    va_start(args, format);
    sprintf(tmpstr, format, args);
    va_end(args);

    Dev_Lcd_WriteStringXY(0, linenumber, tmpstr);
}
*/

/*

void PrintSerialInit()
{
    //UartSendText("\n\r");
    Dev_UART1_WriteStringBlocking("\n\r"); 
}

void PrintSerial( char *format, ...)
{
    char tmpstr[TEMPSTR_SERIAL_SIZE];
    //sprintf(tmpstr, __VA_ARGS__);
    va_list args;

    va_start(args, format);
    sprintf(tmpstr, format, args);
    va_end(args);

    Dev_UART1_WriteStringBlocking(tmpstr); 
}
*/
