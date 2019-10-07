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
// @file drv_power_controllers.c
//
// @brief instances of the various power controllers that are needed in the project
//
// @ntoe    in this application we have two power converters
//          1. Buck converter
//          2. Boost converter
//          all the instances of the data structures for that converters are placed in this file
//
// @author M52409
//
// @date 2019-08-06
//=======================================================================================================

#include <stdbool.h>
#include <xc.h>

#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_buck_generic.h"
#include "driver/power_controllers/drv_power_controller_buck_custom.h"
#include "driver/power_controllers/drv_power_controller_boost_generic.h"
#include "driver/power_controllers/drv_power_controller_boost_custom.h"

//=======================================================================================================
// local defines
//=======================================================================================================
// 
#define VIN_ADC_REFERENCE           3.3     // 3.3 Volts ==> maximum ADC-Value
#define VIN_ADC_RESOLUTION          4095UL  // 12 bits
#define VIN_FEEDBACK_GAIN           0.1253  // 1k /(1k+6.98k)
#define VIN_ACD_REF_DIV_GAIN_1000   26334   // (3.3*1000) / (1k /(1k+6.98k))

volatile uint16_t voltage_input_adc = 0;    // Input Voltage measured by the ADC
volatile uint16_t potentiometer_adc = 0;    // Voltage measured at the potentiometer input

volatile uint16_t Drv_PowerControllers_InitPWM(void);
volatile uint16_t Drv_PowerControllers_InitACMP(void);
volatile uint16_t Drv_PowerControllers_InitADC(void);
volatile uint16_t Drv_PowerControllers_InitVinADC(void);


//=======================================================================================================
// @brief   Initializes the power controllers
// @note    Call this function from your main.c
//=======================================================================================================
void Drv_PowerControllers_Init(void)
{
    // Basic setup of common power controller peripheral modules
    Drv_PowerControllers_InitPWM();    // Set up PWM module (basic module configuration)
    Drv_PowerControllers_InitADC();    // Set up Analog-To-Digital converter module
    Drv_PowerControllers_InitVinADC(); // Initialize ADC Channel to measure input voltage
    
    // Init all Buck Converter instances
    Drv_PowerControllerBuck1_Init(true);                       // Init Buck Converter 1
    Drv_PowerControllerBuck2_Init(true);                       // Init Buck Converter 2

    // Init all Boost Converter instances
    Drv_PowerControllerBoost1_Init(true);                       // Init Boost Convert 1
    Drv_PowerControllerBoost2_Init(true);                       // Init Boost Convert 2
}


//=======================================================================================================
// @brief   returns the Input Voltage in Volts as a double
//=======================================================================================================
double Drv_PowerControllers_GetInputVoltage(void)
{
    return (double)(((unsigned long)voltage_input_adc * VIN_ADC_REFERENCE) / (VIN_FEEDBACK_GAIN * VIN_ADC_RESOLUTION));
}

//=======================================================================================================
// @brief   returns the Input Voltage in Millivolts
//=======================================================================================================
uint32_t Drv_PowerControllers_GetInputVoltage_mV(void)
{
//    return ((uint32_t)voltage_input_adc * VIN_ADC_REFERENCE*1000) / (VIN_FEEDBACK_GAIN * VIN_ADC_RESOLUTION);
    return ((uint32_t)voltage_input_adc * VIN_ACD_REF_DIV_GAIN_1000) / VIN_ADC_RESOLUTION;
}

//=======================================================================================================
// @brief   returns the value of the DACxDATH register for the buck regulator
//=======================================================================================================
uint16_t GetDacBuck(void)
{
    return DAC1DATH;
}

//=======================================================================================================
// @brief   returns the value of the DACxDATH register for the boost regulator
//=======================================================================================================
uint16_t GetDacBoost(void)
{
    return DAC2DATH;
}


//=======================================================================================================
// @brief   power controller task
// @note    call this every 100 µs from your task loop
//=======================================================================================================
void Drv_PowerControllers_Task_100us(void)
{
    // Boost converter PWM generators are triggered by auxiliary PWM of the buck regulator,
    // therefore the boost PWM is the first to be enabled. They start running once the buck
    // PWMs get enabled.
    Drv_PowerControllerBoost_Task_100us(&pwrCtrlBoost1_Data);
    Drv_PowerControllerBuck_Task_100us(&pwrCtrlBuck1_Data);

    // Reading input voltage
    if (_AN12RDY)
    {
        voltage_input_adc = ADCBUF12;
        _ADCAN12IF = 0;
        //pwrCtrlBuck1_Data.voltageInput  = voltage_input_adc;
        //pwrCtrlBoost1_Data.voltageInput = voltage_input_adc;
    }
}


volatile uint16_t Drv_PowerControllers_InitPWM(void)
{
    // Make sure power to the peripheral is enabled
    PMD1bits.PWMMD = 0; // PWM Module Disable: PWM module is enabled
    
    // PWM GENERATOR ENABLE
    PG1CONLbits.ON = 0; // PWM Generator #1 Enable: PWM Generator is not enabled
    PG2CONLbits.ON = 0; // PWM Generator #2 Enable: PWM Generator is not enabled
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG4CONLbits.ON = 0; // PWM Generator #4 Enable: PWM Generator is not enabled
    PG5CONLbits.ON = 0; // PWM Generator #5 Enable: PWM Generator is not enabled
    PG6CONLbits.ON = 0; // PWM Generator #6 Enable: PWM Generator is not enabled
    PG7CONLbits.ON = 0; // PWM Generator #7 Enable: PWM Generator is not enabled
    PG8CONLbits.ON = 0; // PWM Generator #8 Enable: PWM Generator is not enabled
    
    // PWM CLOCK CONTROL REGISTER
    PCLKCONbits.LOCK = 0;       // Lock bit: Write-protected registers and bits are unlocked
    PCLKCONbits.DIVSEL = 0b00;  // PWM Clock Divider Selection: Divide ratio is 1:2
    PCLKCONbits.MCLKSEL = 0b11; // PWM Master Clock Selection: Auxiliary PLL post-divider output
    
    // FREQUENCY SCALE REGISTER & FREQUENCY SCALING MINIMUM PERIOD REGISTER
    FSCL = 0x0000;      // Reset frequency scaling register
    FSMINPER = 0x0000;  // Reset frequency scaling minimum register
    
    // MASTER PHASE, DUTY CYCLE AND PERIOD REGISTERS
    MPHASE = 0;           // Reset master phase
    MDC = 0x0000;         // Reset master duty cycle
    MPER = PWM_PERIOD;    // Master period PWM_PERIOD
    
    // LINEAR FEEDBACK SHIFT REGISTER
    LFSR = 0x0000;      // Reset linear feedback shift register
    
    // COMBINATIONAL TRIGGER REGISTERS
    CMBTRIGLbits.CTA1EN = 0; // Disable Trigger Output from PWM Generator #1 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA2EN = 0; // Disable Trigger Output from PWM Generator #2 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA3EN = 0; // Disable Trigger Output from PWM Generator #3 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA4EN = 0; // Disable Trigger Output from PWM Generator #4 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA5EN = 0; // Disable Trigger Output from PWM Generator #5 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA6EN = 0; // Disable Trigger Output from PWM Generator #6 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA7EN = 0; // Disable Trigger Output from PWM Generator #7 as Source for Combinational Trigger A
    CMBTRIGLbits.CTA8EN = 0; // Disable Trigger Output from PWM Generator #8 as Source for Combinational Trigger A
    
    CMBTRIGHbits.CTB1EN = 0; // Disable Trigger Output from PWM Generator #1 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB2EN = 0; // Disable Trigger Output from PWM Generator #2 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB3EN = 0; // Disable Trigger Output from PWM Generator #3 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB4EN = 0; // Disable Trigger Output from PWM Generator #4 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB5EN = 0; // Disable Trigger Output from PWM Generator #5 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB6EN = 0; // Disable Trigger Output from PWM Generator #6 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB7EN = 0; // Disable Trigger Output from PWM Generator #7 as Source for Combinational Trigger B
    CMBTRIGHbits.CTB8EN = 0; // Disable Trigger Output from PWM Generator #8 as Source for Combinational Trigger B

    // COMBINATORIAL PWM LOGIC A CONTROL REGISTERS A-F
    LOGCONAbits.PWMS1A = 0b0000; // Combinatorial PWM Logic Source #1 Selection: PWM1H
    LOGCONAbits.S1APOL = 0;      // Combinatorial PWM Logic Source #1 Polarity: Input is positive logic
    LOGCONAbits.PWMS2A = 0b0010; // Combinatorial PWM Logic Source #2 Selection: PWM2H
    LOGCONAbits.S2APOL = 0;      // Combinatorial PWM Logic Source #2 Polarity: Input is positive logic
    LOGCONAbits.PWMLFA = 0b01;   // Combinatorial PWM Logic Function Selection: PWMS1y & PWMS2y (AND)
    LOGCONAbits.PWMLFAD = 0b000; // Combinatorial PWM Logic Destination Selection: No assignment, combinatorial PWM logic function is disabled
    
    // Reset further combinatorial logic registers
    LOGCONB = 0x0000; // LOGCONB: COMBINATORIAL PWM LOGIC CONTROL REGISTER B
    LOGCONC = 0x0000; // LOGCONC: COMBINATORIAL PWM LOGIC CONTROL REGISTER C
    LOGCOND = 0x0000; // LOGCOND: COMBINATORIAL PWM LOGIC CONTROL REGISTER D
    LOGCONE = 0x0000; // LOGCONE: COMBINATORIAL PWM LOGIC CONTROL REGISTER E
    LOGCONF = 0x0000; // LOGCONF: COMBINATORIAL PWM LOGIC CONTROL REGISTER F
    
    // PWM EVENT OUTPUT CONTROL REGISTERS A-F
    PWMEVTAbits.EVTAOEN = 0;    // PWM Event Output Enable: Event output signal is internal only
    PWMEVTAbits.EVTAPOL = 0;    // PWM Event Output Polarity: Event output signal is active-high
    PWMEVTAbits.EVTASTRD = 0;   // PWM Event Output Stretch Disable: Event output signal is stretched to eight PWM clock cycles minimum
    PWMEVTAbits.EVTASYNC = 0;   // PWM Event Output Sync: Event output is not synchronized to the system clock
    PWMEVTAbits.EVTASEL = 0b0000; // PWM Event Selection: Source is selected by the PGTRGSEL[2:0] bits
    PWMEVTAbits.EVTAPGS = 0b000;  // PWM Event Source Selection: PWM Generator 1
    
    // Reset further PWM event output registers
    PWMEVTB = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER B
    PWMEVTC = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER C
    PWMEVTD = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER D
    PWMEVTE = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER E
    PWMEVTF = 0x0000;   // PWM EVENT OUTPUT CONTROL REGISTER F
    
    return(1);
}


volatile uint16_t Drv_PowerControllers_InitACMP(void)
{
    // Make sure power is turned off to comparator modules
    PMD7bits.CMP1MD = 0; // Comparator 1 Module Power Disable: Comparator 1 module is disabled
    PMD7bits.CMP2MD = 0; // Comparator 1 Module Power Disable: Comparator 1 module is disabled
    PMD7bits.CMP3MD = 0; // Comparator 3 Module Power Disable: Comparator 3 module is disabled
    
    // Turn off all Comparator/DAC modules during configuration
    DACCTRL1Lbits.DACON = 0; // Common DAC Module Enable: Disables all DAC modules
    DAC1CONLbits.DACEN = 0; // Individual DAC1 Module Enable: Disables DAC1 module during configuration
    DAC2CONLbits.DACEN = 0; // Individual DAC2 Module Enable: Disables DAC2 module during configuration
    DAC3CONLbits.DACEN = 0; // Individual DAC3 Module Enable: Disables DAC3 module during configuration

    // VREGCON: VOLTAGE REGULATOR CONTROL REGISTER
    VREGCONbits.LPWREN = 0; // Low-Power Mode Enable: Voltage regulators are in Full Power mode
    VREGCONbits.VREG1OV = 0b00; // Regulator #1 Low-Power Mode Enable: VOUT = 1.5 * VBG = 1.2V
    VREGCONbits.VREG2OV = 0b00; // Regulator #2 Low-Power Mode Enable: VOUT = 1.5 * VBG = 1.2V
    VREGCONbits.VREG3OV = 0b00; // Regulator #3 Low-Power Mode Enable: VOUT = 1.5 * VBG = 1.2V

    // DACCTRL1L: DAC CONTROL 1 LOW REGISTER
    DACCTRL1Lbits.DACSIDL = 0; // DAC Stop in Idle Mode: Continues module operation in Idle mode
    DACCTRL1Lbits.CLKSEL = 0b10; // DAC Clock Source Selection: AFPLLO
    DACCTRL1Lbits.CLKDIV = 0b00; // DAC Clock Divider: Divider = 1:1
    DACCTRL1Lbits.FCLKDIV = 0b000; // Comparator Filter Clock Divider: Divider = 1:1
    
    // DACCTRL2H/L: DAC CONTROL 2 HIGH and DAC CONTROL 2 LOW REGISTER
    // Settings = 2 x 
    DACCTRL2Lbits.TMODTIME = (DAC_TMODTIME  & 0x03FF); // Transition Mode Duration (default 0x55 = 340ns @ 500 MHz)
    DACCTRL2Hbits.SSTIME = (DAC_SSTIME & 0x0FFF); // Time from Start of Transition Mode until Steady-State Filter is Enabled (default 0x8A = 552ns @ 500 MHz)
    
    return(1);
}


volatile uint16_t Drv_PowerControllers_InitADC(void)
{
     // Make sure power to peripheral is enabled
    PMD1bits.ADC1MD = 0; // ADC Module Power Disable: ADC module power is enabled
    
    // ADCON1L: ADC CONTROL REGISTER 1 LOW
    ADCON1Lbits.ADON = 0; // ADC Enable: ADC module is off during configuration
    ADCON1Lbits.ADSIDL = 0; // ADC Stop in Idle Mode: Continues module operation in Idle mode
    
    // ADCON1H: ADC CONTROL REGISTER 1 HIGH
    ADCON1Hbits.SHRRES = 0b11; // Shared ADC Core Resolution Selection: 12-bit resolution ADC resolution = 12-bit (0...4095 ticks)
    ADCON1Hbits.FORM = 0; // Fractional Data Output Format: Integer

    // ADCON2L: ADC CONTROL REGISTER 2 LOW
    ADCON2Lbits.REFCIE = 0;; // Band Gap and Reference Voltage Ready Common Interrupt Enable: Common interrupt is disabled for the band gap ready event
    ADCON2Lbits.REFERCIE = 0; // Band Gap or Reference Voltage Error Common Interrupt Enable: Disabled
    ADCON2Lbits.EIEN = 1; // Early Interrupts Enable: The early interrupt feature is enabled
    ADCON2Lbits.PTGEN = 0; // External Conversion Request Interface: Disabled
    ADCON2Lbits.SHREISEL = 0b111; // Shared Core Early Interrupt Time Selection: Early interrupt is set and interrupt is generated 8 TADCORE clocks prior to when the data are ready
    ADCON2Lbits.SHRADCS = 0b0000001; // Shared ADC Core Input Clock Divider: 2:1 (minimum)

    // ADCON2H: ADC CONTROL REGISTER 2 HIGH
    ADCON2Hbits.SHRSAMC = 8; // Shared ADC Core Sample Time Selection: 8x TADs sampling time 
    ADCON2Hbits.REFERR = 0; // reset error flag
    ADCON2Hbits.REFRDY = 0; // reset bandgap status bit

    // ADCON3L: ADC CONTROL REGISTER 3 LOW
    ADCON3Lbits.REFSEL = 0b000; // ADC Reference Voltage Selection: AVDD-toAVSS
    ADCON3Lbits.SUSPEND = 0; // All ADC Core Triggers Disable: All ADC cores can be triggered
    ADCON3Lbits.SUSPCIE = 0; // Suspend All ADC Cores Common Interrupt Enable: Common interrupt is not generated for suspend ADC cores
    ADCON3Lbits.SUSPRDY = 0; // All ADC Cores Suspended Flag: ADC cores have previous conversions in progress
    ADCON3Lbits.SHRSAMP = 0; // Shared ADC Core Sampling Direct Control: use hardware trigger
    ADCON3Lbits.CNVRTCH = 0; // Software Individual Channel Conversion Trigger: Next individual channel conversion trigger can be generated (not used)
    ADCON3Lbits.SWLCTRG = 0; // Software Level-Sensitive Common Trigger: No software, level-sensitive common triggers are generated (not used)
    ADCON3Lbits.SWCTRG = 0; // Software Common Trigger: Ready to generate the next software common trigger (not used)
    ADCON3Lbits.CNVCHSEL = 0; // Channel Number Selection for Software Individual Channel Conversion Trigger: AN0 (not used)
    
    // ADCON3H: ADC CONTROL REGISTER 3 HIGH
    ADCON3Hbits.CLKSEL = 0b10; // ADC Module Clock Source Selection: AFVCODIV
    ADCON3Hbits.CLKDIV = 0b000000; // ADC Module Clock Source Divider: 1 Source Clock Period
    ADCON3Hbits.SHREN = 0; // Shared ADC Core Enable: Shared ADC core is disabled
    ADCON3Hbits.C0EN = 0; // Dedicated ADC Core 0 Enable: Dedicated ADC Core 0 is disabled
    ADCON3Hbits.C1EN = 0; // Dedicated ADC Core 1 Enable: Dedicated ADC Core 1 is disabled
    
    // ADCON4L: ADC CONTROL REGISTER 4 LOW
    ADCON4Lbits.SAMC0EN = 0;  // Dedicated ADC Core 0 Conversion Delay Enable: Immediate conversion
    ADCON4Lbits.SAMC1EN = 0;  // Dedicated ADC Core 1 Conversion Delay Enable: Immediate conversion
    
    // ADCON4H: ADC CONTROL REGISTER 4 HIGH
    ADCON4Hbits.C0CHS = 0b00; // Dedicated ADC Core 0 Input Channel Selection: AN0
    ADCON4Hbits.C1CHS = 0b01; // Dedicated ADC Core 1 Input Channel Selection: ANA1

    // ADCON5L: ADC CONTROL REGISTER 5 LOW
    // ADCON5Lbits.SHRRDY: Shared ADC Core Ready Flag (read only)
    // ADCON5Lbits.C0RDY: Dedicated ADC Core 0 Ready Flag (read only)
    // ADCON5Lbits.C1RDY: Dedicated ADC Core 1 Ready Flag (read only)
    ADCON5Lbits.SHRPWR = 0; // Shared ADC Core Power Enable: ADC core is off
    ADCON5Lbits.C0PWR = 0; // Dedicated ADC Core 0 Power Enable: ADC core is off
    ADCON5Lbits.C1PWR = 0; // Dedicated ADC Core 1 Power Enable: ADC core is off
  
    // ADCON5H: ADC CONTROL REGISTER 5 HIGH
    ADCON5Hbits.WARMTIME = 0b1111; // ADC Dedicated Core x Power-up Delay: 32768 Source Clock Periods
    ADCON5Hbits.SHRCIE = 0; // Shared ADC Core Ready Common Interrupt Enable: Common interrupt is disabled for an ADC core ready event
    ADCON5Hbits.C0CIE = 0; // C1CIE: Dedicated ADC Core 0 Ready Common Interrupt Enable: Common interrupt is disabled
    ADCON5Hbits.C1CIE = 0; // C1CIE: Dedicated ADC Core 1 Ready Common Interrupt Enable: Common interrupt is disabled
    
    // ADCORExL: DEDICATED ADC CORE x CONTROL REGISTER LOW
    ADCORE1Lbits.SAMC = 0b0000000000;   // Dedicated ADC Core 1 Conversion Delay Selection: 2 TADCORE (minimum)
    ADCORE0Lbits.SAMC = 0b0000000000;   // Dedicated ADC Core 0 Conversion Delay Selection: 2 TADCORE (minimum)

    // ADCORExH: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE0Hbits.RES = 0b11; // ADC Core x Resolution Selection: 12 bit
    ADCORE0Hbits.ADCS = 0b0000000; // ADC Core x Input Clock Divider: 2 Source Clock Periods
    ADCORE0Hbits.EISEL = 0b111; // Early interrupt is set and an interrupt is generated 8 TADCORE clocks prior

    ADCORE1Hbits.RES = 0b11; // ADC Core x Resolution Selection: 12 bit
    ADCORE1Hbits.ADCS = 0b0000000; // ADC Core x Input Clock Divider: 2 Source Clock Periods
    ADCORE1Hbits.EISEL = 0b111; // Early interrupt is set and an interrupt is generated 8 TADCORE clocks prior
    
    return(1);
}


volatile uint16_t Drv_PowerControllers_InitVinADC(void)
{
    // ANSELx: ANALOG SELECT FOR PORTx REGISTER
    ANSELCbits.ANSELC0 = 1; // Analog input is enabled and digital input is disabled for RC0 (Buck converter input voltage feedback)
    
    // ADLVLTRGL: ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGLbits.LVLEN12 = 0; // Input trigger is edge-sensitive

    // ADMOD0L: ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD0Hbits.DIFF13 = 0; // Differential-Mode for Corresponding Analog Inputs: Channel is single-ended
    ADMOD0Hbits.SIGN13 = 0; // Output Data Sign for Corresponding Analog Inputs: Channel output data are unsigned
    
    // ADEIEL: ADC EARLY INTERRUPT ENABLE REGISTER LOW
    ADEIELbits.EIEN12 = 1; // Early interrupt is enabled for the channel
    
    // ADIEL: ADC INTERRUPT ENABLE REGISTER LOW
    ADIELbits.IE12 = 1; // Common Interrupt Enable: Common and individual interrupts are disabled for the corresponding channel
    
    // ADTRIGnL/ADTRIGnH: ADC CHANNEL TRIGGER n(x) SELECTION REGISTERS LOW AND HIGH
    ADTRIG3Lbits.TRGSRC12 = 0b00100; // Trigger Source Selection for Corresponding Analog Inputs: PWM1 Trigger 1
    
    // ADCMPxCON: ADC DIGITAL COMPARATOR x CONTROL REGISTER
    ADCMP0CONbits.CHNL = 12; // Input Channel Number: 12=AN12
    ADCMP0CONbits.CMPEN = 0; // Comparator Enable: Comparator is disabled
    ADCMP0CONbits.IE = 0; // Comparator Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the comparator
    ADCMP0CONbits.BTWN = 0; // Between Low/High Comparator Event: Disabled
    ADCMP0CONbits.HIHI = 1; // High/High Comparator Event: Enabled
    ADCMP0CONbits.HILO = 0; // High/Low Comparator Event: Disabled
    ADCMP0CONbits.LOHI = 0; // Low/High Comparator Event: Disabled
    ADCMP0CONbits.LOLO = 1; // Low/Low Comparator Event: Enabled
    
    // ADCMPxENL: ADC DIGITAL COMPARATOR x CHANNEL ENABLE REGISTER LOW
    ADCMP0ENLbits.CMPEN12 = 1; // Comparator Enable for Corresponding Input Channels: AN11 Enabled
    
    // ADCMPxLO: ADC COMPARARE REGISTER LOWER THRESHOLD VALUE REGISTER
    ADCMP0LO = 933; // R1=6.98kOhm, R2=1kOhm, G=0.1253; 6Vin=933 ADC ticks

    // ADCMPxHI: ADC COMPARARE REGISTER UPPER THRESHOLD VALUE REGISTER
    ADCMP0HI = 2146; // R1=6.98kOhm, R2=1kOhm, G=0.1253; 13.8Vin=2146 ADC ticks
    
    // ADFLxCON: ADC DIGITAL FILTER x CONTROL REGISTER
    ADFL0CONbits.FLEN = 0; // Filter Enable: Filter is disabled
    ADFL0CONbits.MODE = 0b11; // Filter Mode: Averaging mode (always 12-bit result 7 in oversampling mode 12-16bit wide)
    ADFL0CONbits.OVRSAM = 0b011; // Filter Averaging/Oversampling Ratio: 16x (result in the ADFLxDAT)
    ADFL0CONbits.IE = 0; // Filter Common ADC Interrupt Enable: Common ADC interrupt will not be generated for the filter
    ADFL0CONbits.FLCHSEL = 12; // Oversampling Filter Input Channel Selection: 12=AN12
    
    return(1);
}


volatile uint16_t Drv_PowerControllers_LaunchADC(void)
{
    volatile uint16_t timeout=0;
    
    // If ADC Module is already turned on, ADC is running => skip launch_adc())
    if(ADCON1Lbits.ADON) return(1); 
    
    ADCON1Lbits.ADON = 1; // ADC Enable: ADC module is enabled first

    ADCON5Lbits.SHRPWR = 1; // Enabling Shared ADC Core analog circuits power
    while((!ADCON5Lbits.SHRRDY) && (timeout++<ADC_POWRUP_TIMEOUT));
    if((!ADCON5Lbits.SHRRDY) || (timeout>=ADC_POWRUP_TIMEOUT)) return(0);
    ADCON3Hbits.SHREN  = 1; // Enable Shared ADC digital circuitry
        
    ADCON5Lbits.C0PWR = 0; // Dedicated ADC Core 0 Power Enable: ADC core is off
//    while((!ADCON5Lbits.C0RDY) && (timeout++<ADC_POWRUP_TIMEOUT));
//    if((!ADCON5Lbits.C0RDY) || (timeout>=ADC_POWRUP_TIMEOUT)) return(0);
    ADCON3Hbits.C0EN  = 0; // Dedicated Core 0 is not enabled

    ADCON5Lbits.C1PWR = 0; // Dedicated ADC Core 1 Power Enable: ADC core is off
//    while((!ADCON5Lbits.C1RDY) && (timeout++<ADC_POWRUP_TIMEOUT));
//    if((!ADCON5Lbits.C1RDY) || (timeout>=ADC_POWRUP_TIMEOUT)) return(0);
    ADCON3Hbits.C1EN  = 0; // Dedicated Core 1 is not enabled

    // INITIALIZE AN12 INTERRUPTS (Board Input Voltage)
    IPC25bits.ADCAN12IP = 5;   // Interrupt Priority Level 0
    IFS6bits.ADCAN12IF = 0;    // Reset Interrupt Flag Bit
    IEC6bits.ADCAN12IE = 1;    // Disable ADCAN12 Interrupt 

     // INITIALIZE AN13 INTERRUPTS (Buck Output Voltage)
    IPC26bits.ADCAN13IP = 0;   // Interrupt Priority Level 5
    IFS6bits.ADCAN13IF = 0;    // Reset Interrupt Flag Bit
    IEC6bits.ADCAN13IE = 0;    // Disable ADCAN13 Interrupt 
    
    // INITIALIZE AN18 INTERRUPTS (Boost Output Voltage)
    IPC27bits.ADCAN18IP = 0;   // Interrupt Priority Level 5
    IFS6bits.ADCAN18IF = 0;    // Reset Interrupt Flag Bit
    IEC6bits.ADCAN18IE = 0;    // Disable ADCAN18 Interrupt 
    
    return(1);
}

//=======================================================================================================
// @brief   Interrupt routine for sampling the frequency tuning input
//=======================================================================================================
void __attribute__((__interrupt__, auto_psv, context)) _ADCAN12Interrupt(void)
{
    // the readout of the ADC register is mandatory to make the reset of the interrupt flag stick
    // if we would not read from the ADC register then the interrupt flag would be set immediately after resetting it
    
    potentiometer_adc = ADCBUF12;
         
    _ADCAN12IF = 0;  // Clear the ADCANx interrupt flag. read from ADCBUFx first to make it stick
    
    //TODO: discuss, if we should call the buck_Update routine at first or after resetting the interrupt flag?
}