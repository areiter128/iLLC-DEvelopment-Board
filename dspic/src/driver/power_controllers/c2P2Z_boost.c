/* ***************************************************************************************
 * z-Domain Control Loop Designer Version 0.9.0.60.
 * ***************************************************************************************
 * 2p2z compensation filter coefficients derived for following operating conditions:
 * ***************************************************************************************
 *
 * 	Controller Type:	2P2Z - Basic Current Mode Compensator
 * 	Sampling Frequency:	250000 Hz 
 * 	Fixed Point Format:	15
 * 	Scaling Mode:		3 - Dual Bit-Shift Scaling
 * 	Input Gain:			1
 * 
 * ***************************************************************************************/

#include "driver/power_controllers/c2P2Z_boost.h"

/* ***************************************************************************************
 * Data Arrays:
 * The cNPNZ_t data structure contains a pointer to derived coefficients in X-space and
 * other pointers to controller and error history in Y-space.
 * This source file declares the default parameters of the z-domain compensation filter.
 * These declarations are made publicly accessible through defines in c2p2z_boost.h
 * ***************************************************************************************/

	volatile C2P2Z_BOOST_CONTROL_LOOP_COEFFICIENTS_t __attribute__((space(xmemory), near)) c2P2Z_boost_coefficients; // A/B-Coefficients 
	volatile uint16_t c2P2Z_boost_ACoefficients_size = (sizeof(c2P2Z_boost_coefficients.ACoefficients)/sizeof(c2P2Z_boost_coefficients.ACoefficients[0])); // A-coefficient array size
	volatile uint16_t c2P2Z_boost_BCoefficients_size = (sizeof(c2P2Z_boost_coefficients.BCoefficients)/sizeof(c2P2Z_boost_coefficients.BCoefficients[0])); // B-coefficient array size

	volatile C2P2Z_BOOST_CONTROL_LOOP_HISTORIES_t __attribute__((space(ymemory), far)) c2P2Z_boost_histories; // Control/Error Histories 
	volatile uint16_t c2P2Z_boost_ControlHistory_size = (sizeof(c2P2Z_boost_histories.ControlHistory)/sizeof(c2P2Z_boost_histories.ControlHistory[0])); // Control history array size
	volatile uint16_t c2P2Z_boost_ErrorHistory_size = (sizeof(c2P2Z_boost_histories.ErrorHistory)/sizeof(c2P2Z_boost_histories.ErrorHistory[0])); // Error history array size

/* ***************************************************************************************
 * 	Pole&Zero Placement:
 * ***************************************************************************************
 *
 * 	fP0:	650 Hz 
 * 	fP1:	30000 Hz 
 * 	fZ1:	3000 Hz 
 *
 * ***************************************************************************************
 * 	Filter Coefficients and Parameters:
 * ***************************************************************************************/

	volatile fractional c2P2Z_boost_ACoefficients [2] = 
	{
		0x5CF5,	// Coefficient A1 will be multiplied with controller output u(n-1)
		0xE30C	// Coefficient A2 will be multiplied with controller output u(n-2)
	};

	volatile fractional c2P2Z_boost_BCoefficients [3] = 
	{
		0x7E11,	// Coefficient B0 will be multiplied with error input e(n)
		0x0929,	// Coefficient B1 will be multiplied with error input e(n-1)
		0x8B19	// Coefficient B2 will be multiplied with error input e(n-2)
	};


	volatile int16_t c2P2Z_boost_pre_scaler = 3;
	volatile int16_t c2P2Z_boost_post_shift_A = -1;
	volatile int16_t c2P2Z_boost_post_shift_B = 4;
	volatile fractional c2P2Z_boost_post_scaler = 0x0000;

	volatile cNPNZ16b_t c2P2Z_boost; // user-controller data object

/* ***************************************************************************************/

uint16_t c2P2Z_boost_Init(void)
{
	volatile uint16_t i = 0;

	// Initialize controller data structure at runtime with pre-defined default values
	c2P2Z_boost.status.flags = CONTROLLER_STATUS_CLEAR;  // clear all status flag bits (will turn off execution))

	c2P2Z_boost.ptrACoefficients = &c2P2Z_boost_coefficients.ACoefficients[0]; // initialize pointer to A-coefficients array
	c2P2Z_boost.ptrBCoefficients = &c2P2Z_boost_coefficients.BCoefficients[0]; // initialize pointer to B-coefficients array
	c2P2Z_boost.ptrControlHistory = &c2P2Z_boost_histories.ControlHistory[0]; // initialize pointer to control history array
	c2P2Z_boost.ptrErrorHistory = &c2P2Z_boost_histories.ErrorHistory[0]; // initialize pointer to error history array
	c2P2Z_boost.normPostShiftA = c2P2Z_boost_post_shift_A; // initialize A-coefficients/single bit-shift scaler
	c2P2Z_boost.normPostShiftB = c2P2Z_boost_post_shift_B; // initialize B-coefficients/dual/post scale factor bit-shift scaler
	c2P2Z_boost.normPostScaler = c2P2Z_boost_post_scaler; // initialize control output value normalization scaling factor
	c2P2Z_boost.normPreShift = c2P2Z_boost_pre_scaler; // initialize A-coefficients/single bit-shift scaler

	c2P2Z_boost.ACoefficientsArraySize = c2P2Z_boost_ACoefficients_size; // initialize A-coefficients array size
	c2P2Z_boost.BCoefficientsArraySize = c2P2Z_boost_BCoefficients_size; // initialize A-coefficients array size
	c2P2Z_boost.ControlHistoryArraySize = c2P2Z_boost_ControlHistory_size; // initialize control history array size
	c2P2Z_boost.ErrorHistoryArraySize = c2P2Z_boost_ErrorHistory_size; // initialize error history array size


	// Load default set of A-coefficients from user RAM into X-Space controller A-array
	for(i=0; i<c2P2Z_boost.ACoefficientsArraySize; i++)
	{
		c2P2Z_boost_coefficients.ACoefficients[i] = c2P2Z_boost_ACoefficients[i];
	}

	// Load default set of B-coefficients from user RAM into X-Space controller B-array
	for(i=0; i<c2P2Z_boost.BCoefficientsArraySize; i++)
	{
		c2P2Z_boost_coefficients.BCoefficients[i] = c2P2Z_boost_BCoefficients[i];
	}

	// Clear error and control histories of the 2P2Z controller
	c2P2Z_boost_Reset(&c2P2Z_boost);

	return(1);
}


