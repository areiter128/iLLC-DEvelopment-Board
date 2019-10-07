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
// @file drv_power_controller_boost_custom.c
//
// @brief power controller functions for boost converter
//
// @author M91406
// @author M52409
// @author M91281
//
// @date September 4, 2019, 2:45 PM
//=======================================================================================================

#include <stdbool.h>
#include <stdint.h>
#include "xc.h"

#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_boost_generic.h"
#include "driver/power_controllers/drv_power_controller_boost_custom.h"
#include "driver/power_controllers/npnz16b.h"
#include "driver/power_controllers/c2P2Z_boost.h"

//=======================================================================================================
// local defines
//=======================================================================================================
// Boost instance 1 specific defines:
#define BOOST1_ADC_REFERENCE     3.3             // 3.3 Volts ==> maximum ADC-Value
#define BOOST1_ADC_RESOLUTION    4095UL          // 12 bits
#define BOOST1_FEEDBACK_GAIN     0.1253          // 1k /(1k+6.98k)
#define BOOST1_IIN_FEEDBACK_GAIN 1.0             // 1 V/A
#define BOOST1_VIN_GAIN 0.1253                   // 1k /(1k+6.98k)


#define INIT_DACDATH_BOOST            0  // DAC value for the boost the slope starts from
#define INIT_DACDATL_BOOST            0  // Set this to minimum in Slope mode

#define BOOST_INIT_PCMC_CLAMP         0  // [A]; Minimum (and initial maximum) clamping value for boost converter input current
#define BOOST_FINAL_PCMC_CLAMP        2  // [A]; Maximum (and final maximum) clamping value for boost converter input current  

#define BOOST_IN_PCMC_CL      (uint16_t)(BOOST_INIT_PCMC_CLAMP * BOOST1_IIN_FEEDBACK_GAIN * BOOST1_ADC_RESOLUTION / BOOST1_ADC_REFERENCE)
#define BOOST_FN_PCMC_CL      (uint16_t)(BOOST_FINAL_PCMC_CLAMP * BOOST1_IIN_FEEDBACK_GAIN * BOOST1_ADC_RESOLUTION / BOOST1_ADC_REFERENCE)
#define BOOST_RP_CL_PER       (uint16_t)((BOOST1_CLAMP_RAMPUP_PERIOD / MAIN_EXECUTION_PERIOD)-1.0)             
#define BOOST_RP_CL_STEP      (uint16_t)((BOOST_FN_PCMC_CL - BOOST_IN_PCMC_CL)/(BOOST_RP_CL_PER + 1))
#define BOOST_RP_VREF_PER     (uint16_t)((BOOST1_VREF_RAMPUP_PERIOD / MAIN_EXECUTION_PERIOD)-1.0)
#define BOOST_DAC_SLOPE_RATE  (uint16_t)((16.0 * (BOOST1_SLEW_RATE / DAC_GRAN) / (1.0e-6/DACCLK)) + 1.0) // SLOPE DATA in [DAC-ticks/CLK-tick]

POWER_CONTROLLER_DATA_t pwrCtrlBoost1_Data, pwrCtrlBoost2_Data;      // data instance for the boost converter


//=======================================================================================================
// @brief   returns the Input Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBoost1_GetInputVoltage()
{
    return (double)(((unsigned long)voltage_input_adc * BOOST1_ADC_REFERENCE) / (BOOST1_VIN_GAIN * BOOST1_ADC_RESOLUTION));
}

//=======================================================================================================
// @brief   returns the Output Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllerBoost1_GetOutputVoltage()
{
    return (double)(((unsigned long)pwrCtrlBoost1_Data.voltageOutput * BOOST1_ADC_REFERENCE) / (BOOST1_FEEDBACK_GAIN * BOOST1_ADC_RESOLUTION));
}

//=======================================================================================================
// @brief   sets the Initial Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed initial reference voltage 
//=======================================================================================================
void Drv_PowerControllerBoost1_SetInitialOutputVoltageReference(double newVoltRef)
{
    pwrCtrlBoost1_Data.voltageRef_softStart = ((newVoltRef * BOOST1_ADC_RESOLUTION * BOOST1_FEEDBACK_GAIN) / BOOST1_ADC_REFERENCE);
}

//=======================================================================================================
// @brief   sets the Initial Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed initial reference voltage
//=======================================================================================================
void Drv_PowerControllerBoost1_SetInitialOutputVoltageReference_mV(uint32_t newVoltRef_mV)
{
    pwrCtrlBoost1_Data.voltageRef_softStart = ((((uint32_t)newVoltRef_mV * BOOST1_ADC_RESOLUTION) * BOOST1_FEEDBACK_GAIN) / BOOST1_ADC_REFERENCE) / 1000;
}


//=======================================================================================================
// @brief   sets the Output Voltage Reference in Volts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage 
//=======================================================================================================
void Drv_PowerControllerBoost1_SetOutputVoltageReference(double newVoltRef)
{
    pwrCtrlBoost1_Data.voltageRef_softStart = ((newVoltRef * BOOST1_ADC_RESOLUTION * BOOST1_FEEDBACK_GAIN) / BOOST1_ADC_REFERENCE);
}

//=======================================================================================================
// @brief   sets the Output Voltage Reference in Millivolts
// @note    call this function after calling the Init function to tell power controller the needed reference voltage
//=======================================================================================================
void Drv_PowerControllerBoost1_SetOutputVoltageReference_mV(uint32_t newVoltRef_mV)
{
    pwrCtrlBoost1_Data.voltageRef_softStart = ((((uint32_t)newVoltRef_mV * BOOST1_ADC_RESOLUTION) * BOOST1_FEEDBACK_GAIN) / BOOST1_ADC_REFERENCE) / 1000;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Drv_PowerControllerBoost1_EnableControlLoop(void)
{
    PG3IOCONLbits.OVRENH = 0;           // User override enabled for PWMxH Pin
    PG3IOCONLbits.OVRENL = 0;           // User override disabled for PWMxL Pin
}

void Drv_PowerControllerBoost2_EnableControlLoop(void)
{
    PG4IOCONLbits.OVRENH = 0;           // User override enabled for PWMxH Pin
    PG4IOCONLbits.OVRENL = 0;           // User override disabled for PWMxL Pin
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void Drv_PowerControllerBoost1_DisableControlLoop(void)
{
    PG3IOCONLbits.OVRENH = 1;           // User override enabled for PWMxH Pin
    PG3IOCONLbits.OVRENL = 1;           // User override enabled for PWMxL Pin
}
void Drv_PowerControllerBoost2_DisableControlLoop(void)
{
    PG4IOCONLbits.OVRENH = 1;           // User override enabled for PWMxH Pin
    PG4IOCONLbits.OVRENL = 1;           // User override enabled for PWMxL Pin
}

void Drv_PowerControllerBoost1_LaunchPeripherals(void)
{
    Drv_PowerControllerBoost1_LaunchPWM();           // Start PWM
}
void Drv_PowerControllerBoost2_LaunchPeripherals(void)
{
    Drv_PowerControllerBoost2_LaunchPWM();           // Start PWM
}

//=======================================================================================================
// @brief   Initializes the Boost Power Converter - Instance 1
// @note    In this routine all the application specific custom functions are implemented
//=======================================================================================================
void Drv_PowerControllerBoost1_Init(bool autostart)
{
    pwrCtrlBoost1_Data.ftkEnableControlLoop  = Drv_PowerControllerBoost1_EnableControlLoop;
    pwrCtrlBoost1_Data.ftkDisableControlLoop = Drv_PowerControllerBoost1_DisableControlLoop;
    pwrCtrlBoost1_Data.ftkLaunchPeripherals  = Drv_PowerControllerBoost1_LaunchPeripherals;
      
    Drv_PowerControllerBoost1_InitPWM();          // Set up primary PWM 
    Drv_PowerControllerBoost1_InitADC();          // Set up ADC (voltage feedback only)
}

void Drv_PowerControllerBoost2_Init(bool autostart)
{
    pwrCtrlBoost2_Data.ftkEnableControlLoop  = Drv_PowerControllerBoost2_EnableControlLoop;
    pwrCtrlBoost2_Data.ftkDisableControlLoop = Drv_PowerControllerBoost2_DisableControlLoop;
    pwrCtrlBoost2_Data.ftkLaunchPeripherals  = Drv_PowerControllerBoost2_LaunchPeripherals;

    Drv_PowerControllerBoost2_InitPWM();          // Set up primary PWM 
}

volatile uint16_t Drv_PowerControllerBoost1_InitPWM(void)
{
   // PWM GENERATOR x CONTROL REGISTERS
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG3CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG3CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator x
    PG3CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG3CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode
    
    PG3CONHbits.MDCSEL = 1; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG3CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG3CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG3CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG3CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG3CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG3CONHbits.SOCS = 1; // Start-of-Cycle Selection: Trigger output selected by PG1

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER LOW
    PG3IOCONL = 0x0000;
    
    PG3IOCONLbits.SWAP = 0;    // Swap PWM Signals to PWMxH and PWMxL Device Pins: PWMxH/L signals are mapped to their respective pins
    PG3IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG3IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin
    PG3IOCONLbits.OVRDAT = 0b00; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAT1
    PG3IOCONLbits.OSYNC = 0b00; // User Output Override Synchronization Control: User output overrides via the OVRENH/L and OVRDAT[1:0] bits are synchronized to the local PWM time base (next Start-of-Cycle)
    
    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG3IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG3IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG3IOCONHbits.PMOD = 0b00; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Complementary mode
    PG3IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG3IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG3IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG3IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR x STATUS REGISTER
    PG3STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR x EVENT REGISTER LOW 
    PG3EVTLbits.ADTR1PS     = 0b00001;      // ADC Trigger 1 Postscaler Selection = 1:2
    PG3EVTLbits.ADTR1EN1    = 0b0;          // PG3TRIGA  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN2    = 0b0;          // PG3TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.ADTR1EN3    = 0b0;          // PG3TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG3EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG3EVTLbits.PGTRGSEL    = 0b011;        // PGxTRIGC compare event is the PWM Generator trigger => This serves as SOC trigger for PG2
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG3EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG3EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG3EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG3EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG3EVTHbits.IEVTSEL     = 0b11;         // Interrupt Event Selection: Time base interrupts are disabled
    PG3EVTHbits.ADTR2EN1    = 0b0;          // PG3TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN2    = 0b0;          // PG3TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR2EN3    = 0b0;          // PG3TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG3EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = No offset
    
    // PCI function for current limitation is not used
    PG3CLPCIH       = 0x0000;           // PWM GENERATOR CL PCI REGISTER HIGH
    PG3CLPCIL       = 0x0000;           // PWM GENERATOR CL PCI REGISTER LOW
      
    // Reset further PCI control registers
    PG3FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG3FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG3FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG3FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG3SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG3SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // Leading edge blanking is not used
    PG3LEBH         = 0x0000;
    PG3LEBL         = 0x0000;
    
        
    // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    PG3PHASE    = 0;
    
    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    PG3DC       = MAX_DUTY_CYCLE;     
    
    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG3DCA      =  0x0000;      
    
    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG3PER      = 0;     // Master defines the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG3TRIGA    = 0;  // ToDo: Check this value on oscilloscope
    
    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG3TRIGB    = 0;  
    
    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG3TRIGC    = (MPER >> 1) + 40; 
//    PG3TRIGC    = 0;
    
    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG3DTL      = PWM_DEAD_TIME_FALLING;
    
    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG3DTH      = PWM_DEAD_TIME_RISING;
            
//  PG3CAP      = 0x0000;   // Read only register
   
    return(1);
}

volatile uint16_t Drv_PowerControllerBoost1_LaunchPWM(void)
{
    Nop();
    Nop();
    Nop();
    
    PG3CONLbits.ON = 1; // PWM Generator #2 Enable: PWM Generator is enabled
    
    while(PG3STATbits.UPDATE);
    PG3STATbits.UPDREQ = 1; // Update all PWM registers

    PG3IOCONHbits.PENH = 1; // PWMxH Output Port Enable: Disabled
    PG3IOCONHbits.PENL = 1; // PWMxL Output Port Enable: PWM generator controls the PWMxL output pin
    
    return(1);
}


volatile uint16_t Drv_PowerControllerBoost2_InitPWM(void)
{
   // PWM GENERATOR x CONTROL REGISTERS
    PG4CONLbits.ON = 0; // PWM Generator #4 Enable: PWM Generator is not enabled
    PG4CONLbits.TRGCNT = 0b000; // Trigger Count Select: PWM Generator produces one PWM cycle after triggered
    PG4CONLbits.HREN = 0; // High-Resolution mode is not enabled for PWM Generator x
    PG4CONLbits.CLKSEL = 0b01; // Clock Selection: PWM Generator uses Master clock selected by the MCLKSEL[1:0] (PCLKCON[1:0]) control bits
    PG4CONLbits.MODSEL = 0b000; // PWM Mode Selection: Independent Edge PWM mode
    
    PG4CONHbits.MDCSEL = 1; // Master Duty Cycle Register Selection: PWM Generator uses PGxDC register
    PG4CONHbits.MPERSEL = 1; // Master Period Register Selection: PWM Generator uses MPER register
    PG4CONHbits.MPHSEL = 0; // Master Phase Register Selection: PWM Generator uses PGxPHASE register
    PG4CONHbits.MSTEN = 0; // Master Update Enable: PWM Generator does not broadcast the UPDREQ status bit state or EOC signal
    PG4CONHbits.UPDMOD = 0b000; // PWM Buffer Update Mode Selection: SOC update
    PG4CONHbits.TRGMOD = 0; // PWM Generator Trigger Mode Selection: PWM Generator operates in single trigger mode
    PG4CONHbits.SOCS = 2; // Start-of-Cycle Selection: Trigger output selected by PG2

    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER LOW
    PG4IOCONL = 0x0000;
    
    PG4IOCONLbits.SWAP = 0;    // Swap PWM Signals to PWMxH and PWMxL Device Pins: PWMxH/L signals are mapped to their respective pins
    PG4IOCONLbits.OVRENH = 1;  // User Override Enable for PWMxH Pin: OVRDAT1 provides data for output on the PWMxH pin
    PG4IOCONLbits.OVRENL = 1;  // User Override Enable for PWMxL Pin: OVRDAT0 provides data for output on the PWMxL pin
    PG4IOCONLbits.OVRDAT = 0b00; // Data for PWMxH/PWMxL Pins if Override Event is Active: PWMxL=OVRDAT0, PWMxH=OVRDAT1
    PG4IOCONLbits.OSYNC = 0b00; // User Output Override Synchronization Control: User output overrides via the OVRENH/L and OVRDAT[1:0] bits are synchronized to the local PWM time base (next Start-of-Cycle)
    
    // PGxIOCONH: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    PG4IOCONHbits.CAPSRC = 0b000;  // Time Base Capture Source Selection: No hardware source selected for time base capture ? software only
    PG4IOCONHbits.DTCMPSEL = 0; // Dead-Time Compensation Selection: Dead-time compensation is controlled by PCI Sync logic
    PG4IOCONHbits.PMOD = 0b00; // PWM Generator Output Mode Selection: PWM Generator outputs operate in Complementary mode
    PG4IOCONHbits.PENH = 0; // PWMxH Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxH output pin
    PG4IOCONHbits.PENL = 0; // PWMxL Output Port Enable: GPIO registers TRISx, LATx, Rxx registers control the PWMxL output pin
    PG4IOCONHbits.POLH = 0; // PWMxH Output Port Enable: Output pin is active-high
    PG4IOCONHbits.POLL = 0; // PWMxL Output Port Enable: Output pin is active-high
    
    // PWM GENERATOR x STATUS REGISTER
    PG4STAT = 0x0000;   // Reset to default
    
    // PWM GENERATOR x EVENT REGISTER LOW 
    PG4EVTLbits.ADTR1PS     = 0b00001;      // ADC Trigger 1 Postscaler Selection = 1:2
    PG4EVTLbits.ADTR1EN1    = 0b0;          // PG4TRIGA  Compare Event is disabled as trigger source for ADC Trigger 1
    PG4EVTLbits.ADTR1EN2    = 0b0;          // PG4TRIGB  Compare Event is disabled as trigger source for ADC Trigger 1
    PG4EVTLbits.ADTR1EN3    = 0b0;          // PG4TRIGC  Compare Event is disabled as trigger source for ADC Trigger 1
    PG4EVTLbits.UPDTRG      = 0b00;         // User must set the UPDATE bit (PG1STAT<4>) manually
    PG4EVTLbits.PGTRGSEL    = 0b000;        // EOC serves as trigger output
    
    // PWM GENERATOR x EVENT REGISTER HIGH
    PG4EVTHbits.FLTIEN      = 0b0;          // PCI Fault interrupt is disabled
    PG4EVTHbits.CLIEN       = 0b0;          // PCI Current-Limit interrupt is disabled
    PG4EVTHbits.FFIEN       = 0b0;          // PCI Feed-Forward interrupt is disabled
    PG4EVTHbits.SIEN        = 0b0;          // PCI Sync interrupt is disabled
    PG4EVTHbits.IEVTSEL     = 0b11;         // Interrupt Event Selection: Time base interrupts are disabled
    PG4EVTHbits.ADTR2EN1    = 0b0;          // PG4TRIGA register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR2EN2    = 0b0;          // PG4TRIGB register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR2EN3    = 0b0;          // PG4TRIGC register compare event is disabled as trigger source for ADC Trigger 2
    PG4EVTHbits.ADTR1OFS    = 0b00000;      // ADC Trigger 1 offset = No offset
    
    // PCI function for current limitation is not used
    PG4CLPCIH       = 0x0000;           // PWM GENERATOR CL PCI REGISTER HIGH
    PG4CLPCIL       = 0x0000;           // PWM GENERATOR CL PCI REGISTER LOW
      
    // Reset further PCI control registers
    PG4FPCIH        = 0x0000;          // PWM GENERATOR F PCI REGISTER HIGH
    PG4FPCIL        = 0x0000;          // PWM GENERATOR F PCI REGISTER LOW
    PG4FFPCIH       = 0x0000;          // PWM GENERATOR FF PCI REGISTER HIGH
    PG4FFPCIL       = 0x0000;          // PWM GENERATOR FF PCI REGISTER LOW
    PG4SPCIH        = 0x0000;          // PWM GENERATOR S PCI REGISTER HIGH
    PG4SPCIL        = 0x0000;          // PWM GENERATOR S PCI REGISTER LOW
    
    // Leading edge blanking is not used
    PG4LEBH         = 0x0000;
    PG4LEBL         = 0x0000;
    
        
    // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    PG4PHASE    = 0;
    
    // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    PG4DC       = MAX_DUTY_CYCLE;     
    
    // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER
    PG4DCA      =  0x0000;      
    
    // PGxPER: PWM GENERATOR x PERIOD REGISTER        
    PG4PER      = 0;     // Master defines the period

    // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    PG4TRIGA    = 0;  // ToDo: Check this value on oscilloscope
    
    // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER       
    PG4TRIGB    = 0;  
    
    // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER        
    PG4TRIGC    = 0;  
    
    // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW        
    PG4DTL      = PWM_DEAD_TIME_FALLING;
    
    // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    PG4DTH      = PWM_DEAD_TIME_RISING;
            
//  PG4CAP      = 0x0000;   // Read only register
   
    return(1);
}

volatile uint16_t Drv_PowerControllerBoost2_LaunchPWM(void)
{
    Nop();
    Nop();
    Nop();
    
    PG4CONLbits.ON = 1; // PWM Generator #4 Enable: PWM Generator is enabled
    
    while(PG4STATbits.UPDATE);
    PG4STATbits.UPDREQ = 1; // Update all PWM registers

    PG4IOCONHbits.PENH = 1; // PWMxH Output Port Enable: Disabled
    PG4IOCONHbits.PENL = 1; // PWMxL Output Port Enable: PWM generator controls the PWMxL output pin
    
    return(1);
}


volatile uint16_t Drv_PowerControllerBoost1_InitADC(void)
{
    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELDbits.ANSELD10 = 1; // Analog input is enabled and digital input is disabled for RD10 (Boost converter output voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGHbits.LVLEN18 = 0; // Input trigger is edge-sensitive
    
    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD1Lbits.DIFF18 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD1Lbits.SIGN18 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEH: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIEHbits.EIEN18 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIEHbits.IE18 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel

    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG4Hbits.TRGSRC18 = 0b01010; // Trigger Source Selection for Corresponding Analog Inputs: PWM4 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP2CONbits.CHNL = 18; // Input Channel Number: 13=AN13
    ADCMP2CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP2CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP2CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP2CONbits.HIHI = 0; // High/High Comparator Event: Disabled
    ADCMP2CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP2CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP2CONbits.LOLO = 0; // Low/Low Comparator Event: Disabled
    
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP2ENHbits.CMPEN18 = 0; // Comparator Enable for Corresponding Input Channels: AN18 Disabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP2LO = 1399; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 9Vin=1399 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP2HI = 2799; // R1=6.98kOhm, R2=1kOhm, G=0.751879699; 18Vin=2799 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL2CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL2CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL2CONbits.OVRSAM = 0b001; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL2CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL2CONbits.FLCHSEL = 18; // Oversampling Filter Input Channel Selection: 18=AN18
    
    return(1);
}

//=======================================================================================================
// @brief   Interrupt routine for calling the boost c2p2z compensator and sampling the Output Voltage
//=======================================================================================================
void __attribute__((__interrupt__, auto_psv, context)) _ADCAN18Interrupt(void)
{
//    c2P2Z_boost_Update(&c2P2Z_boost);     //call the compensator as soon as possible
    // the readout of the ADC register is mandatory to make the reset of the interrupt flag stick
    // if we would not read from the ADC register then the interrupt flag would be set immediately after resetting it
    pwrCtrlBoost1_Data.voltageOutput =  ADCBUF18;
    pwrCtrlBoost1_Data.flags.bits.adc_active = true;
    _ADCAN18IF = 0;  // Clear the ADCANx interrupt flag. read from ADCBUFx first to make it stick
    
    //TODO: discuss, if we should call the boost_Update routine at first or after resetting the interrupt flag?
}

