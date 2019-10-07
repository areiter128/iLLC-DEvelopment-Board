/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   pwr_control.h
 * Author: M91406
 * Comments: power controller functions for buck converter
 * Revision history: 
 * 1.0  initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INITIALIZE_BOOST_POWER_CONTROL_H
#define	INITIALIZE_BOOST_POWER_CONTROL_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "c2P2Z_boost.h"


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum {
    BOOST_STAT_OFF    = 0,  // Converter Status OFF
    BOOST_STAT_START  = 1,  // Converter STatus Startup
    BOOST_STAT_ON     = 2,  // Converter Status Active and Running
    BOOST_STAT_FAULT  = 3   // Converter Status FAULT
}BOOST_CONVERTER_STATUS_e;

typedef enum {
    BOOST_SS_INIT            = 0,  // Soft-Start Phase Initialization
    BOOST_SS_PWR_ON_DELAY    = 1,  // Soft-Start Phase Power On Delay
    BOOST_SS_RAMP_UP         = 2,  // Soft-Start Phase Output Ramp Up 
    BOOST_SS_PWR_GOOD_DELAY  = 3,  // Soft-Start Phase Power Good Delay
    BOOST_SS_COMPLETE        = 4   // Soft-Start Phase Complete
}BOOST_SOFT_START_STATUS_e;

typedef struct {
    volatile uint16_t reference;        // Soft-Start target reference value
    volatile uint16_t pwr_on_delay;     // Soft-Start POwer On Delay
    volatile uint16_t ramp_period;      // Soft-Start Ramp-Up Duration
    volatile uint16_t pwr_good_delay;   // Soft-Start Power Good Delay
    volatile uint16_t counter;          // Soft-Start Execution Counter
    volatile uint16_t phase;            // Soft-Start Phase Index
}BOOST_SOFT_START_t;

extern volatile BOOST_SOFT_START_t boost_soft_start;
    
extern volatile uint16_t init_boost_pwr_control(void);
extern volatile uint16_t launch_boost_pwr_control(void);
extern volatile uint16_t exec_boost_pwr_control(void);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* INITIALIZE_BOOST_POWER_CONTROL_H */

