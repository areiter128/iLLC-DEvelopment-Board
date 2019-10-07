/*
 * File:   pwr_control.c
 * Author: M91406
 *
 * Created on July 9, 2019, 1:10 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "npnz16b.h"
#include "boost_pwr_control.h"
#include "init/init_pwm.h"
#include "init/init_acmp.h"
#include "init/init_adc.h"
#include "misc/global.h"

volatile BOOST_SOFT_START_t boost_soft_start;

volatile uint16_t init_boost_pwr_control(void) {

    init_boost_trig_pwm();  // Initialize auxiliary PWM generator for the boost controller
    init_boost_pwm();       // Set up boost converter PWM
    init_boost_acmp();      // Set up boost converter peak current comparator/DAC
    init_boost_adc();       // Set up boost converter ADC (voltage feedback only)
    
    boost_soft_start.counter = 0;            // Reset Soft-Start Counter
    boost_soft_start.phase = BOOST_SS_INIT;  // Reset Soft-Start Phase to Initialization
    boost_soft_start.pwr_good_delay = 999;   // Soft-Start Power-On Delay = 100 ms
    boost_soft_start.ramp_period = 499;      // Soft-Start Ramp Period = 500 ms
    boost_soft_start.pwr_good_delay = 1999;  // Soft-Start Power Good Delay = 200 ms
    boost_soft_start.reference = 2047;       // Soft-Start Target Reference = 3.3V
    
    c2P2Z_boost_Init();
    
    c2P2Z_boost.ADCTriggerOffset = 80;
    c2P2Z_boost.ptrADCTriggerRegister = &PG4TRIGA;
    c2P2Z_boost.InputOffset = 0;
    c2P2Z_boost.ptrControlReference = &data.boost_vref;
    c2P2Z_boost.ptrSource = &ADCBUF18;
    c2P2Z_boost.ptrTarget = &DAC2DATH;
    c2P2Z_boost.status.flag.enable = 0;
    
    return(1);
}

volatile uint16_t launch_boost_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller

    launch_adc();           // Start ADC Module
    launch_boost_acmp();    // Start analog comparator/DAC module
    launch_boost_trig_pwm();// Start auxiliary PWM
    launch_boost_pwm();     // Start PWM
    
    
    
    return(1);
}

volatile uint16_t exec_boost_pwr_control(void) {

    // Run enable-sequence of all peripherals used by this power controller
    
    // ToDo: ADD BOOST CONVERTER EXECUTION CODE
    
    return(1);
}

void __attribute__((__interrupt__, auto_psv)) _ADCAN18Interrupt(void)
{
//    volatile uint16_t dummy=0;
    
//    dummy = ADCBUF18;

    data.vout_boost = ADCBUF18;
    c2P2Z_boost_Update(&c2P2Z_boost);
            
    _ADCAN18IF = 0;  // Clear the ADCANx interrupt flag 
    
}

