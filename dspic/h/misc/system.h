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
// @file system.h
//
// @brief functions to initialize the system
//
// @author M91406
// @author M52409
// @author M91281
//
// @date 2019-09-09 1:10 PM
//=======================================================================================================/*

#ifndef _SYSTEM_H_
#define	_SYSTEM_H_


//=======================================================================================================
// @brief Initialize the system components (Config Bits, Oscillators, GPIO)
//=======================================================================================================
void System_Init(void);

#ifdef DPSK3_R30
// DPSK3 Debug Pins 
    #define DBGLED_SET		{ _LATB6 = 1; }
    #define DBGLED_CLEAR	{ _LATB6 = 0; }
    #define DBGLED_TOGGLE	{ _LATB6 ^= 1; }
    #define DBGLED_INIT		{ _LATB6 = 0; _TRISB6 = 0; }

    // TP50
    #define DBGPIN_1_SET	{ _LATB5 = 1; }
    #define DBGPIN_1_CLEAR	{ _LATB5 = 0; }
    #define DBGPIN_1_TOGGLE	{ _LATB5 ^= 1; }
    #define DBGPIN_1_INIT	{ _LATB5 = 0; _TRISB5 = 0; }

    // TP52
    #define DBGPIN_2_SET	{ _LATB11 = 1; }
    #define DBGPIN_2_CLEAR	{ _LATB11 = 0; }
    #define DBGPIN_2_TOGGLE	{ _LATB11 ^= 1; }
    #define DBGPIN_2_INIT	{ _LATB11 = 0; _TRISB11 = 0; }

    // TP53
    #define DBGPIN_3_SET	{ _LATB12 = 1; }
    #define DBGPIN_3_CLEAR	{ _LATB12 = 0; }
    #define DBGPIN_3_TOGGLE	{ _LATB12 ^= 1; }
    #define DBGPIN_3_INIT	{ _LATB12 = 0; _TRISB12 = 0; }

#endif

#ifdef MA330048_R30
// dsPIC33CK DP PIM Debug Pins
    #define DBGLED_SET		{ _LATD15 = 1; }
    #define DBGLED_CLEAR	{ _LATD15 = 0; }
    #define DBGLED_TOGGLE	{ _LATD15 ^= 1; }
    #define DBGLED_INIT		{ _LATD15 = 0; _TRISD15 = 0; }

    #define DBGPIN_SET		{ _LATD12 = 1; }
    #define DBGPIN_CLEAR	{ _LATD12 = 0; }
    #define DBGPIN_TOGGLE	{ _LATD12 ^= 1; }
    #define DBGPIN_INIT		{ _LATD12 = 0; _TRISD12 = 0; }
#endif


#ifdef	__cplusplus
extern "C" {
#endif  // __cplusplus


    
#ifdef	__cplusplus
}
#endif  // __cplusplus

#endif	// _SYSTEM_H_

