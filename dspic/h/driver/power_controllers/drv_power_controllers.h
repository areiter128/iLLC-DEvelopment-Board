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
// @file drv_power_controllers.h
//
// @brief contains the generic and custom parts of the power controller(s)
//
// @version v1.0
// @date 2019-08-06
// @author M52409
//
//=======================================================================================================

#ifndef _DRV_POWER_CONTROLLERS_H_
#define _DRV_POWER_CONTROLLERS_H_

#include <stdbool.h>
#include <stdint.h>


//=======================================================================================================
// @brief   Initializes the power controllers
// @note    Call this function from your main.c
//=======================================================================================================
extern void Drv_PowerControllers_Init(void);


//=======================================================================================================
// @brief   power controller task
// @note    call this every 100 탎 from your task loop
//=======================================================================================================
extern void Drv_PowerControllers_Task_100us(void);


//=======================================================================================================
// @brief   returns the Input Voltage in Volts as a double
//=======================================================================================================
extern double Drv_PowerControllers_GetInputVoltage(void);


//=======================================================================================================
// @brief   returns the Input Voltage in Millivolts
//=======================================================================================================
extern uint32_t Drv_PowerControllers_GetInputVoltage_mV(void);







#define BUCK1_VREF               3.3   // [V]; Voltage reference for boost converter
#define BUCK1_SLEW_RATE          0.3 // Compensation ramp in [V/usec] (SLPxDAT is calculated below)
#define BUCK1_VREF_RAMPUP_PERIOD 100e-3  // [s]; Vref ramp-up period for buck converter

#define BOOST1_VREF              15.0   // [V]; Voltage reference for boost converter
#define BOOST1_SLEW_RATE          0.2 // Compensation ramp in [V/usec] (SLPxDAT is calculated below)
#define BOOST1_CLAMP_RAMPUP_PERIOD 20e-3  // [s]; Current clamp ramp-up period for boost converter
#define BOOST1_VREF_RAMPUP_PERIOD 100e-3  // [s]; Vref ramp-up period for boost converter
  
#define BUCK1_VREF_mV         (uint32_t)(1000*(double)(BUCK1_VREF))
#define BOOST1_VREF_mV        (uint32_t)(1000*(double)(BOOST1_VREF))

/*!Device Clock Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for device clock settings
 * 
 * Description:
 * This section is used to define device specific parameters related to the core clock and 
 * auxiliary clock used to drive PWM, ADC and DAC.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

#define CPU_FREQUENCY       100000000   // CPU frequency in [Hz]
#define AUX_FREQUENCY       400000000   // Auxiliary Clock Frequency in [Hz]
#define PWM_FREQUENCY       400000000   // PWM Generator Base Clock Frequency in [Hz]

#define MAIN_EXECUTION_PERIOD    100e-6     // main state machine pace period in [sec]

#define ADC_POWRUP_TIMEOUT         5000




/*!ACMP Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device DAC
 * 
 * Description:
 * This section is used to define device specific parameters of ADC reference, resolution,
 * granularity and slope timer frequency to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

//-------    
#define DAC_REF         (double)3.300           // DAC reference voltage (usually AVDD)
#define DAC_RES         (double)12.00           // DAC resolution in [bit]
#define DAC_GRAN        (double)(DAC_REF / pow(2, DAC_RES))  // DAC granularity in [V/tick]
#define FDAC            (double)AUX_FREQUENCY   // DAC input clock in Hz
#define DACCLK          (double)(2.0/FDAC)      // DAC input clock (period) selected in [sec]

//-------    
#define DAC_CBLANK_TIME 100e-9  // Comparator Blanking Period in [ns] applied when DAC reference changes 
#define DAC_T_RESET     300e-9  // Transition Mode Duration
#define DAC_T_SETTLING  350e-9  // Time from Start of Transition Mode until Steady-State Filter is Enabled

// Device-specific DAC settings
#define DAC_TMCB        (uint16_t)((DAC_CBLANK_TIME * FDAC)/2.0)    // Leading edge period for the comparator when slope re-settles to its initial value
#define DAC_TMODTIME    (uint16_t)((DAC_T_RESET * FDAC)/2.0)            // Transition Mode Duration
#define DAC_SSTIME      (uint16_t)((DAC_T_SETTLING * FDAC)/2.0)         // Time from Start of Transition Mode until Steady-State Filter is Enabled


/*!PWM Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device PWM
 * 
 * Description:
 * This section is used to define device specific parameters of PWM frequency, may duty ratio, 
 * leading edge blanking, slope compensation and ADC triggers.
 * granularity and slope timer frequency to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/
#define SWITCHING_FREQUENCY         850e+3      // Power Supply Switching Frequency in [Hz]
    
//------ macros
#define SWITCHING_PERIOD            (1.0/SWITCHING_FREQUENCY)   // Power Supply Switching Period in [sec]
#define PWM_RES                     (1.0/AUX_FREQUENCY)         // PWM Resolution
#define PWM_PERIOD                  (uint16_t)(SWITCHING_PERIOD / PWM_RES)      // Measured in [tick = 2.5ns]
//------ 

#define PG3_TO_PG1_OFFSET           0.50    // Boost offset with respect to the buck in proportion of the PWM period 
#define MAXIMUM_DUTY_RATIO          0.50    // Maximum Duty Ratio in [%]
#define LEB_PERIOD                  100e-9  // Leading Edge Blanking period in [sec]
#define SLOPE_START_DELAY           100e-9  // Delay in {sec] until the slope compensation ramp starts
#define SLOPE_STOP_DELAY            0.80    // Delay in [%] until the slope compensation ramp stops
#define VOUT_ADC_TRIGGER_DELAY      (SWITCHING_PERIOD - 1300e-9) // ADC trigger delay in [sec] used to sample output voltage
//#define VOUT_ADC_TRIGGER_DELAY      (uint16_t)(MAXIMUM_DUTY_RATIO * SWITCHING_PERIOD) // ADC trigger delay in [sec] used to sample output voltage
#define PWM_MASTER_PHASE_SHIFT      0e-9  // Switching frequency phase shift in [sec]
#define PWM_AUXILIARY_PHASE_SHIFT   100e-9  // Switching frequency phase shift in [sec]

//------ macros
#define P3_TO_P1_OFFSET             (uint16_t)(PWM_PERIOD * PG3_TO_PG1_OFFSET);  // This sets the offset for the boost converter with respect to the buck
#define MAX_DUTY_CYCLE              (uint16_t)(PWM_PERIOD * MAXIMUM_DUTY_RATIO)     // This sets the maximum duty cycle
#define PWM_LEB_PERIOD              (uint16_t)(LEB_PERIOD / PWM_RES)  // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define PWM_MSTR_PHASE_SHIFT        (uint16_t)(PWM_MASTER_PHASE_SHIFT / PWM_RES)   // Master PWM Phase Shift
#define PWM_AUX_PHASE_SHIFT         (uint16_t)(PWM_AUXILIARY_PHASE_SHIFT / PWM_RES)   // Auxiliary PWM Phase Shift
    
#define VOUT_ADCTRIG                (uint16_t)(VOUT_ADC_TRIGGER_DELAY / PWM_RES)    // ADC trigger delay in [ticks] used to sample output voltage
#define SLP_TRIG_START              (uint16_t)(SLOPE_START_DELAY / PWM_RES)         // Delay in {sec] until the slope compensation ramp starts
#define SLP_TRIG_STOP               (uint16_t)(PWM_PERIOD * SLOPE_STOP_DELAY) // Delay in {sec] until the slope compensation ramp stops

#define PWM_DEAD_TIME_RISING        20   // Rising edge dead time [2.5ns]
#define PWM_DEAD_TIME_FALLING       20   // Falling edge dead time [2.5ns]

typedef enum
{
    PCS_STARTUP_PERIPHERALS      = 1,    // Fire up all the peripherals that are involved
    PCS_STANDBY                  = 2,    // Soft-Start Standby (wait for GO command)
    PCS_WAIT_FOR_POWER_IN_GOOD   = 3,    // Soft-Start wait some time to guarantee a stable power supply
    PCS_WAIT_FOR_ADC_ACTIVE      = 4,    // wait until ADC is running
    PCS_MEASURE_INPUT_VOLTAGE    = 5,    // measure input voltage
    PCS_RAMP_UP_VOLTAGE          = 6,    // Soft-Start Ramp Up Output Voltage
    PCS_WAIT_FOR_POWER_OUT_GOOD  = 7,    // Soft-Start wait to stabilize
    PCS_UP_AND_RUNNING           = 8     // Soft-Start is complete, power is up and running
}PWR_CTRL_STATE_INT_e;


typedef enum
{
    STATE_OFF     = 0b000,  // PwrCtrl Status Off:      Everything is inactive incl. peripherals
    STATE_STANDBY = 0b001,  // PwrCtrl Status Standby:  Peripherals are running but controller and PWM outputs are off
    STATE_START   = 0b010,  // PwrCtrl Status Startup:  Converter is executing its startup procedure
    STATE_ON      = 0b011,  // PwrCtrl Status On:       Power Controller is Active and Running
    STATE_FAULT   = 0b100   // PwrCtrl Status Fault:    Power Controller has been shut down waiting for restart attempt
} PWR_CTRL_STATE_e;

typedef struct
{
    volatile PWR_CTRL_STATE_e op_status :3; // Bit <0:2> operation status
    volatile unsigned : 5;                              // Bit <3:7> (reserved)
    volatile bool pwm_active  :1;                       // Bit  8: Status bit indicating that the PWM outputs have been enabled
    volatile bool adc_active  :1;                       // Bit  9: Status bit indicating that the ADC has been started and is sampling data
    volatile bool fault_active  :1;                     // Bit 10: Status bit indicating that a critical fault condition has been detected
    volatile bool overvoltage_fault  :1;                // Bit 11: Status bit indicating that a critical fault condition has been detected
    volatile bool undervoltage_fault  :1;               // Bit 12: Status bit indicating that a critical fault condition has been detected
    volatile bool GO :1;                                // Bit 13: POWER SUPPLY START bit (will trigger startup procedure when set)
    volatile bool auto_start :1;                        // Bit 14: Auto-Start will automatically enable the converter and set the GO bit when ready
    volatile bool enabled :1;                           // Bit 15: Enable-bit (when disabled, power supply will reset in STANDBY mode)
} __attribute__((packed))POWER_CONTROLLER_FLAGBITS_t;

typedef union {
	volatile uint16_t value;                 // buffer for 16-bit word read/write operations
	volatile POWER_CONTROLLER_FLAGBITS_t bits; // data structure for single bit addressing operations
} POWER_CONTROLLER_FLAGS_t;                  // SEPIC operation status bits

typedef void (*pCallback_t)(void);

typedef struct
{
    //TODO: is the reference always voltage or sometimes current?
    volatile uint16_t voltageRef_rampPeriod_100us;  // amount of 100탎 ticks it takes for the reference to ramp up
    volatile uint16_t powerInputOk_waitTime_100us;  // amount of 100탎 ticks to wait for the input power to be stable
    volatile uint16_t powerOutputOk_waitTime_100us; // amount of 100탎 ticks to wait for the output power to be stable
//    volatile uint16_t precharge_delay;      // Soft-Start Bootstrap Capacitor pre-charge delay if necessary
    volatile uint16_t rampUp_period;            // Soft-Start Ramp-Up Duration
    volatile uint16_t averageCounter;           // average value calculation counter
    volatile uint16_t timeCounter;              // Soft-Start Execution Counter
    volatile PWR_CTRL_STATE_INT_e pc_state_internal;   // state of the power control state machine
    volatile uint16_t voltageOutput;            // Output Voltage measured by the ADC
    volatile uint16_t voltageRef_softStart;     // target voltage reference value for soft start
    volatile uint16_t voltageRef_compensator;   // voltage reference for the compensator
    volatile uint16_t voltageRef_rampStep;      // Soft-Start Single Reference Increment per Step
    volatile int16_t currentClamp_rampStep;    // Soft-Start Single Current Clamp Increment per Step
    volatile uint16_t compMaxOutput;
    volatile uint16_t compMinOutput;
    volatile uint16_t OverVoltageLimit;         // Overvoltage Limit, gets calculated automatically
    volatile uint16_t UnderVoltageLimit;        // Overvoltage Limit, gets calculated automatically
    volatile POWER_CONTROLLER_FLAGS_t flags;    // status flags
    volatile pCallback_t ftkEnableControlLoop;  // Controller calls this function to enable the control Loop
    volatile pCallback_t ftkDisableControlLoop; // Controller calls this function to disable the control Loop
    volatile pCallback_t ftkLaunchPeripherals;  // Controller calls this function to launch the peripherals
    volatile int16_t *compClampMax;
   }POWER_CONTROLLER_DATA_t;                       // power control soft-start settings and variables

extern volatile uint16_t Drv_PowerControllers_LaunchADC(void);


//TODO: change names of that functions and move it to the proper place
//double GetVoltageBoost(void);
//double GetVoltageInput(void);
uint16_t GetDacBuck(void);
uint16_t GetDacBoost(void);

extern volatile uint16_t voltage_input_adc;    // Input/Supply Voltage measured by the ADC
extern volatile uint16_t potentiometer_adc;

#endif  //_DRV_POWER_CONTROLLERS_H_
