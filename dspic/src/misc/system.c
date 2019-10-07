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
// @file system.c
//
// @brief functions to initialize the system
//
// @author M91406
// @author M52409
// @author M91281
//
// @date 2019-09-09 1:10 PM
//=======================================================================================================/*

#include <xc.h> // include processor files - each processor file is guarded.
#include "misc/system.h"

#define TIMEOUT_LIMIT   5000    // timeout counter maximum



// Configuration bits: selected in the GUI
//-------------------------------------------------------------------------------------------------------
// FICD

#ifdef DPSK3_R30
    #pragma config ICS = PGD1    //ICD Communication Channel Select bits->Communicate on PGC2 and PGD2
#endif
#ifdef MA330048_R30
    #pragma config ICS = PGD2    //ICD Communication Channel Select bits->Communicate on PGC2 and PGD2
#endif

#pragma config JTAGEN = OFF    //JTAG Enable bit->JTAG is disabled
#pragma config NOBTSWP = DISABLED    //BOOTSWP instruction disable bit->BOOTSWP instruction is disabled

//-------------------------------------------------------------------------------------------------------
// FALTREG
#pragma config CTXT1 = IPL5    //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits->Not Assigned
#pragma config CTXT2 = OFF    //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits->Not Assigned
#pragma config CTXT3 = OFF    //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits->Not Assigned
#pragma config CTXT4 = OFF    //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits->Not Assigned

//-------------------------------------------------------------------------------------------------------
// FSEC
#pragma config BWRP = OFF    //Boot Segment Write-Protect bit->Boot Segment may be written
#pragma config BSS = DISABLED    //Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSEN = OFF    //Boot Segment Control bit->No Boot Segment
#pragma config GWRP = OFF    //General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = DISABLED    //General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF    //Configuration Segment Write-Protect bit->Configuration Segment may be written
#pragma config CSS = DISABLED    //Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = OFF    //Alternate Interrupt Vector Table bit->Disabled AIVT

// FBSLIM
#pragma config BSLIM = 8191    //Boot Segment Flash Page Address Limit bits->8191

// FOSCSEL
#pragma config FNOSC = FRC    //Oscillator Source Selection->Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) 
#pragma config IESO = OFF    //Two-speed Oscillator Start-up Enable bit->Start up with user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE    //Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFNC = ON    //OSC2 Pin Function bit->OSC2 is general purpose digital I/O pin
#pragma config FCKSM = CSECMD    //Clock Switching Mode bits->Clock switching is enabled,Fail-safe Clock Monitor is disabled
#pragma config PLLKEN = ON    //PLL Lock Status Control->PLL lock signal will be used to disable PLL clock output if lock is lost
#pragma config XTCFG = G3    //XT Config->24-32 MHz crystals
#pragma config XTBST = ENABLE    //XT Boost->Boost the kick-start

// FWDT
#pragma config RWDTPS = PS2147483648    //Run Mode Watchdog Timer Post Scaler select bits->1:2147483648
#pragma config RCLKSEL = LPRC    //Watchdog Timer Clock Select bits->Always use LPRC
#pragma config WINDIS = OFF    //Watchdog Timer Window Enable bit->Watchdog Timer in Window mode
#pragma config WDTWIN = WIN25    //Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config SWDTPS = PS2147483648    //Sleep Mode Watchdog Timer Post Scaler select bits->1:2147483648
#pragma config FWDTEN = ON_SW    //Watchdog Timer Enable bit->WDT controlled via SW, use WDTCON.ON bit

// FPOR
#pragma config BISTDIS = DISABLED    //Memory BIST Feature Disable->mBIST on reset feature disabled

// FDMTIVTL
#pragma config DMTIVTL = 0    //Dead Man Timer Interval low word->0

// FDMTIVTH
#pragma config DMTIVTH = 0    //Dead Man Timer Interval high word->0

// FDMTCNTL
#pragma config DMTCNTL = 0    //Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF)->0

// FDMTCNTH
#pragma config DMTCNTH = 0    //Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF)->0

// FDMT
#pragma config DMTDIS = OFF    //Dead Man Timer Disable bit->Dead Man Timer is Disabled and can be enabled by software

// FDEVOPT
#pragma config ALTI2C1 = OFF    //Alternate I2C1 Pin bit->I2C1 mapped to SDA1/SCL1 pins
#pragma config ALTI2C2 = OFF    //Alternate I2C2 Pin bit->I2C2 mapped to SDA2/SCL2 pins
#pragma config ALTI2C3 = OFF    //Alternate I2C3 Pin bit->I2C3 mapped to SDA3/SCL3 pins
#pragma config SMBEN = SMBUS    //SM Bus Enable->SMBus input threshold is enabled
#pragma config SPI2PIN = PPS    //SPI2 Pin Select bit->SPI2 uses I/O remap (PPS) pins

// FBTSEQ
#pragma config BSEQ = 4095    //Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active->4095
#pragma config IBSEQ = 4095    //The one's complement of BSEQ; must be calculated by the user and written during device programming.->4095

// FBOOT
#pragma config BTMODE = SINGLE    //Device Boot Mode Configuration->Device is in Single Boot (legacy) mode



//=======================================================================================================
// @brief Set up system oscillator for 100 MIPS operation
//=======================================================================================================
volatile uint16_t System_FOsc_Init(void)
{
    volatile uint16_t timeout=0;
    
    //Temporarily switch to FRC (without PLL), so we can safely change the PLL settings,
    //in case we had previously been already running from the PLL.
    if(OSCCONbits.COSC != 0b000)
    {
        // NOSC = 0b000 = FRC without divider or PLL
        __builtin_write_OSCCONH(0b000);  // Fast RC Oscillator, no PLL 
        // Clear CLKLOCK and assert OSWEN = 1 to initiate switch-over
        __builtin_write_OSCCONL((OSCCON & 0x7E) | 0x01); 
        //Wait for switch over to complete.
        while((OSCCONbits.COSC != OSCCONbits.NOSC) && (timeout++ < TIMEOUT_LIMIT)); 
        if (timeout >= TIMEOUT_LIMIT)
            return(0);
    }
    
    OSCTUNbits.TUN = 0;     // Set FRC tuning register to 8.000 MHz (default)
    
    // Configure PLL prescaler, both PLL postscalers, and PLL feedback divider
    CLKDIVbits.PLLPRE = 1; // N1=1
    PLLFBDbits.PLLFBDIV = 200; // M = 125
    PLLDIVbits.POST1DIV = 4; // N2=5
    PLLDIVbits.POST2DIV = 1; // N3=1
    PLLDIVbits.VCODIV = 0; // VCO Output divider is set to Fvco/4
    
    // Initiate Clock Switch to FRC Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0b001);  // Fast RC Oscillator with PLL 
    if(OSCCONbits.COSC != OSCCONbits.NOSC)
    {
        // Assert OSWEN and make sure CLKLOCK is clear, to initiate the switching operation
        __builtin_write_OSCCONL((OSCCON & 0x7F) | 0x01);    
        // Wait for clock switch to finish
        while((OSCCONbits.COSC != OSCCONbits.NOSC) && (timeout++ < TIMEOUT_LIMIT));  
        if ((OSCCONbits.COSC != OSCCONbits.NOSC) || (timeout >= TIMEOUT_LIMIT))
            return(0);
    }

    // Lock registers against accidental changes
    OSCCONbits.CLKLOCK = 1;
    
    while((OSCCONbits.LOCK != 1) && (timeout++ < TIMEOUT_LIMIT)); // Wait n while loops for PLL to Lock
	if ((OSCCONbits.LOCK != 1) || (timeout >= TIMEOUT_LIMIT)) // Error occurred? 
        return(0);  // => If so, return error code
   
    // Return Success/Failure
    return((1 - OSCCONbits.CF));				// Return oscillator fail status bit
}


//=======================================================================================================
// @brief Set up Auxiliary PLL for 400 MHz (source clock to PWM module)
//=======================================================================================================
volatile uint16_t System_AClk_Init(void)
{
    volatile uint16_t timeout=0;

    // Clear Enable-bit of Auxiliary PLL during configuration
    ACLKCON1bits.APLLEN = 0;

    // Set AVCO divider of Auxiliary PLL 
    APLLDIV1bits.AVCODIV   = 0b01;  // AVCO Scaler = AFVCO/3

    // Configure APLL pre-scaler, APLL post-scaler, APLL divisor
    ACLKCON1bits.APLLPRE   = 1;     // N1 (non zero)
	APLLFBD1bits.APLLFBDIV = 100;   // M  = APLLFBD 
    APLLDIV1bits.APOST1DIV = 2;     // N2 (non zero)
    APLLDIV1bits.APOST2DIV = 1;     // N3 (non zero)

    // Select clock input source (either primary oscillator or internal FRC)
    ACLKCON1bits.FRCSEL = 1;        // FRC is the clock source for APLL
//    ACLKCON1bits.ASRCSEL = 0;       // ?? unknown bit from device header file
	
    // Set Enable-bit of Auxiliary PLL 
    ACLKCON1bits.APLLEN = 1;

    // if user has not enabled the APLL module, exit here
    if(!ACLKCON1bits.APLLEN)
        return(0);
        
    // Wait 5000 while loops for APLL to Lock
    while((ACLKCON1bits.APLLCK != 1) && (timeout++<TIMEOUT_LIMIT));		
	if ((ACLKCON1bits.APLLCK != 1) || (timeout++ >= TIMEOUT_LIMIT))	// PLL still not locked in? 
        return (0);     // => If so, return error code
    else
        return(ACLKCON1bits.APLLCK);

    return(1);
}

//=======================================================================================================
// @brief Initialize common device GPIOs
//=======================================================================================================
volatile uint16_t Systen_GPIO_Init(void)
{   
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    
    TRISA = 0;  // RA set to output
    TRISB = 0;  // RB set to output
    TRISC = 0;  // RC set to output
    TRISD = 0;  // RD set to output
    
    ODCA = 0xFF;    // Open-drain is enabled for PORTA
    ODCB = 0xCF;    // Open-drain is enabled for PORTB
    ODCC = 0xFF;    // Open-drain is enabled for PORTC
    ODCD = 0xFC;    // Open-drain is enabled for PORTD
    
    LATA = 0xFF;   
    LATB = 0xCF;
    LATC = 0xFF;
    LATD = 0xFC;    // RD0 & RD1 to set 0
    
    
    
    DBGLED_INIT;
//    DBGPIN_1_INIT;
//    DBGPIN_2_INIT;
//    DBGPIN_3_INIT;
    
    return(1);
}


//=======================================================================================================
// @brief Initialize the system components (Config Bits, Oscillators, GPIO)
//=======================================================================================================
void System_Init(void)
{
    System_FOsc_Init();
    System_AClk_Init();
    Systen_GPIO_Init();
}
        