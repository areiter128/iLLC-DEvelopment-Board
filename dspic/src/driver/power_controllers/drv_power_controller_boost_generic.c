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
// @file drv_power_controller_boost_generic.c
//
// @brief   power controller functions for boost converter
// @note    in this file are only generic functions for the boost converter,
//          every application specific customized code is in the file drv_power_controller_custom_boost.c
//
// @author M91406
// @author M52409
// @author M91281
//
// @date September 4, 2019, 5:57 PM
//=======================================================================================================

#include <stdbool.h>
#include <stdint.h>
#include <xc.h>         // include processor specific defines and functions

#include "driver/power_controllers/drv_power_controllers.h"
#include "driver/power_controllers/drv_power_controller_boost_generic.h"

#include <misc/system.h>
//=======================================================================================================
// local defines
//=======================================================================================================

//======================================================================================================================
// adjust these defines to have the right under- and overvoltage borders for monitoring the output voltage
//======================================================================================================================
#define BOOST_OVERVOLTAGE_PERCENT     105
#define BOOST_UNDERVOLTAGE_PERCENT     95

volatile static uint16_t vin_avg = 0;      // Averaging buffer for Vin measurement

static inline void Drv_PowerControllerBoost_CalculateVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData);
static inline void Drv_PowerControllerBoost_MonitorVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData);

//=======================================================================================================
// @brief   returns the raw ADC value for the Output Voltage
//=======================================================================================================
uint16_t Drv_PowerControllerBoost_GetOutputVoltageRaw(POWER_CONTROLLER_DATA_t* pPCData)
{
    return pPCData->voltageOutput;
}

//=======================================================================================================
// @brief   internal helping function for the state machine to set the new state
// @note    the also resets the timer automatically
//=======================================================================================================
void boostPC_GotoState(POWER_CONTROLLER_DATA_t* pPCData, PWR_CTRL_STATE_e newState)
{
    pPCData->averageCounter = 0;
    pPCData->timeCounter = 0;
    pPCData->pc_state_internal = newState;
}

//=======================================================================================================
// @brief   Initializes all peripherals and data structures of the boost controller like PWM, ADC, DAC, CMP etc.
// @note    call this during booting up the system before you call anything else or the Power Controller
//=======================================================================================================
void Drv_PowerControllerBoost_Init(POWER_CONTROLLER_DATA_t* pPCData, bool autostart)
{
    pPCData->voltageRef_compensator = 0;                // 2047 for 3.3V
    pPCData->voltageRef_softStart = 0;                  // 2047 for 3.3V
    pPCData->voltageOutput = 0;
    //pPCData->voltageInput  = 0;
    pPCData->flags.value = 0;                           // reset everything
    pPCData->flags.bits.adc_active = false;
    pPCData->flags.bits.auto_start = autostart;
    boostPC_GotoState(pPCData, PCS_STARTUP_PERIPHERALS); // reset state machine
    
    Drv_PowerControllerBoost_CalculateVoltageLimits(pPCData);
}


//=======================================================================================================
// @brief   Task that runs all the necessary things to do like soft start and voltage monitoring
// @note    call this every 100us from your main scheduler to ensure the right timing
//=======================================================================================================
void Drv_PowerControllerBoost_Task_100us(POWER_CONTROLLER_DATA_t* pPCData)
{
    //TODO Monitor Over Undervoltage every time before calling the statemachine
    switch (pPCData->pc_state_internal)
    {
        // Fire up all peripherals used by this power controller
        case PCS_STARTUP_PERIPHERALS:
              
            Drv_PowerControllers_LaunchADC();   // Start ADC Module; Note if
                                                // ADC Module is already turned on,
                                                // this routine will be skipped
            pPCData->ftkLaunchPeripherals();

            //status.flags.op_status = SEPIC_STAT_OFF; // Set SEPIC status to OFF
            boostPC_GotoState(pPCData, PCS_WAIT_FOR_POWER_IN_GOOD);           //Goto next state
            break;
        
        // In this step the soft-start procedure continues with counting up until the defined power-on
        // delay period has expired.
        // PWM1H is kept low, while PWM1L is kept high to pre-charge the half-bridge bootstrap cap.
        // At the end of this phase, PWM1 output user overrides are disabled and the control is enabled.
        case PCS_WAIT_FOR_POWER_IN_GOOD:
            if(++pPCData->timeCounter >= pPCData->powerInputOk_waitTime_100us)
            {
               pPCData->ftkEnableControlLoop();

                boostPC_GotoState(pPCData, PCS_WAIT_FOR_ADC_ACTIVE);
            }
            break;
                 
        case PCS_WAIT_FOR_ADC_ACTIVE:    // wait until the power controller is active
            if (pPCData->flags.bits.adc_active)
            {
                boostPC_GotoState(pPCData, PCS_MEASURE_INPUT_VOLTAGE);
            }
            break;
        case PCS_MEASURE_INPUT_VOLTAGE:  // take 8 samples of input voltage and calculate average value  
        {    
                vin_avg += voltage_input_adc;
                
                if (++(pPCData->averageCounter) == 8) 
                {
                    vin_avg = 0;    // Reset averaging buffer

                    // Recently established input voltage value serves as initial reference value for the output
                    pPCData->voltageRef_compensator = vin_avg >> 3;;
                    
                    // Establishing magnitude of single step for the upcoming ramp-up phase
                    pPCData->voltageRef_rampStep = (uint16_t)((pPCData->voltageRef_softStart - pPCData->voltageRef_compensator)/(pPCData->voltageRef_rampPeriod_100us + 1));
                    if(pPCData->voltageRef_rampStep == 0) {         // Protecting startup settings against 
                        pPCData->voltageRef_rampStep = 1;           // ZERO settings
                    }               
                    boostPC_GotoState(pPCData, PCS_RAMP_UP_VOLTAGE);
                }
            break;
        }         
        case PCS_RAMP_UP_VOLTAGE: // Increasing the voltage reference and compensator clamp for boost every scheduler cycle
        {
            int16_t newClamp;
            uint16_t newReference;
            
            Drv_PowerControllerBoost_MonitorVoltageLimits(pPCData);
           
            newClamp = *(pPCData->compClampMax) + pPCData->currentClamp_rampStep;

            if (newClamp < pPCData->compMaxOutput)
            {
                *(pPCData->compClampMax) = newClamp;
            }
            else
            {
                 *(pPCData->compClampMax) = pPCData->compMaxOutput;
            }
            
            newReference = pPCData->voltageRef_compensator + pPCData->voltageRef_rampStep;
            if (newReference < pPCData->voltageRef_softStart)
            {
                pPCData->voltageRef_compensator = newReference;
            }
            else
            {
                pPCData->voltageRef_compensator = pPCData->voltageRef_softStart;
                boostPC_GotoState(pPCData, PCS_WAIT_FOR_POWER_OUT_GOOD);
            }
            Drv_PowerControllerBoost_CalculateVoltageLimits(pPCData);    // Update the VoltageLimits
            break; 
        }

        case PCS_WAIT_FOR_POWER_OUT_GOOD:    // wait some time for the caps to charge and the power to be stable
            Drv_PowerControllerBoost_MonitorVoltageLimits(pPCData);
            if (++pPCData->timeCounter >= pPCData->powerOutputOk_waitTime_100us)
            {
                boostPC_GotoState(pPCData, PCS_UP_AND_RUNNING);
            }
            break;
                 
        case PCS_UP_AND_RUNNING:   // Soft start is complete, system is running
          
            // Checking output voltage limits
            Drv_PowerControllerBoost_MonitorVoltageLimits(pPCData);
            break;

        default: // If something is going wrong, reset entire PWR controller
            boostPC_GotoState(pPCData, PCS_STARTUP_PERIPHERALS);
            break;
    }
}

static inline void Drv_PowerControllerBoost_CalculateVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData)
{
    pPCData->OverVoltageLimit  = (uint32_t) (pPCData->voltageRef_compensator * BOOST_OVERVOLTAGE_PERCENT) / 100;
    pPCData->UnderVoltageLimit = (uint32_t) (pPCData->voltageRef_compensator * BOOST_UNDERVOLTAGE_PERCENT) / 100;
}

static inline void Drv_PowerControllerBoost_MonitorVoltageLimits(POWER_CONTROLLER_DATA_t* pPCData)
{
    if (pPCData->voltageOutput >= pPCData->OverVoltageLimit)
        pPCData->flags.bits.overvoltage_fault = true;
    else
        pPCData->flags.bits.overvoltage_fault = false;
    if (pPCData->voltageOutput <= pPCData->UnderVoltageLimit)
        pPCData->flags.bits.undervoltage_fault = true;
    else
        pPCData->flags.bits.undervoltage_fault = false;
}
