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
// @file main.c
//
// @brief contains the main function for the Digital Power Starter Kit Version 3 (aka DPSK3)
//
// @author M91406
// @author M91281
// @author M52409
//
// @date July 8, 2019, 1:52 PM
//=======================================================================================================

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "misc/system.h"
#include "device/dev_lcd.h"
#include "device/dev_button.h"
#include "device/dev_uart1.h"
#include "driver/power_controllers/drv_power_controllers.h"
#include "app/app_display.h"
#include "app/app_proto24.h"
#include "app/app_logger.h"
#include "app/app_fault_handling.h"
#include "os/os_scheduler.h"

#ifdef TEST_ENABLED
#include "app/app_test.h"
#endif  // TEST_ENABLED
#include "misc/delay.h"
#include "misc/global.h"

#include "driver/power_controllers/drv_power_controller_buck_custom.h"
#include "driver/power_controllers/drv_power_controller_boost_custom.h"

int main(void)
{
    System_Init();      //Init System: Oscillators and GPIOs
    
//    Dev_Lcd_Init();
//    Dev_Button_Init();
//    App_Logger_Init();
//    App_Fault_Handling_Init();
//    App_Proto24_Init();
//    App_Proto24_GetData(&global_proto24data);
//    OS_Scheduler_Init();
    //TODO: is that delay for sending the SYS_RESET information after about one second to make sure that the pic24 is also ready?
    //TODO: this could also be integrated in the App_Proto24_Task_10ms function where we have a more accurate timing
    __delay_ms(200); // due to heavy workload on ISR, delays take ~3.7 times longer than normal

    Drv_PowerControllers_Init();    // Initialize all Power Controllers (Buck Converter and Boost Converter)
    
    Drv_PowerControllers_LaunchADC();   // Start ADC Module; Note if
                                        // ADC Module is already turned on,
    
    // Launch PG4
    Drv_PowerControllerBoost2_LaunchPWM();// this routine will be skipped
    // Launch PG2
    Drv_PowerControllerBuck2_LaunchPWM();
    // Launch PG3
    Drv_PowerControllerBoost1_LaunchPWM();
    // Launch PG1
    Drv_PowerControllerBuck1_LaunchPWM();
    
    // Enable PG1 outputs
    Drv_PowerControllerBuck1_EnableControlLoop(); 
    // Enable PG3 outputs
    Drv_PowerControllerBoost1_EnableControlLoop(); 
    // Enable PG2 outputs
    Drv_PowerControllerBuck2_EnableControlLoop(); 
    // Enable PG4 outputs
    Drv_PowerControllerBoost2_EnableControlLoop(); 
    
    

//    Global_UpdateBoardData();
//    global_data.pic24_packet_counter = 0;

//#ifdef TEST_ENABLED
//    App_Test();
//#else
//    OS_Scheduler_RunForever();
//#endif  // TEST_ENABLED
    
    while(1)
    {
        Nop();
        Nop();
        Nop();
        
        MPER = (uint16_t)(800 - (double)((unsigned long)potentiometer_adc * 400/4095));
        MDC = MPER >> 1;
        PG1TRIGC = MPER >> 1;
        PG3TRIGC = (MPER >> 1) + 40;
        PG2TRIGC = MPER >> 1;
        
        while(PG1STATbits.UPDATE || PG3STATbits.UPDATE || PG2STATbits.UPDATE || PG4STATbits.UPDATE);
        PG1STATbits.UPDREQ = 1; // Update all PWM registers
        PG3STATbits.UPDREQ = 1; // Update all PWM registers
        PG2STATbits.UPDREQ = 1; // Update all PWM registers
        PG4STATbits.UPDREQ = 1; // Update all PWM registers
        __delay_ms(100);
        
        Nop();
        Nop();
        Nop();
    }
    
    return (0);
}

