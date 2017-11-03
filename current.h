#ifndef CURRENT_H_
#define CURRENT_H_
/// @file current.h
/// @brief Controls the inner current loop and the PWM signal
/// @author Your Name 
/// @version 1.0
/// @date 2014-03-01
/// Implements the PI current controller.  Also allows for directly setting PWM values.
#define FULL_DUTY 1999
int currentref;
/// @brief Initializes the current.c module and the peripherals it uses
void current_init(void);

/// @brief Specifies the reference PWM value, as a signed duty cycle percentage
///	   The percentage is the magnitude and the sign the direction
///	   This will be the PWM value when core_state == PWM, otherwise it is ignored.
/// @post  This function will not change core_state
void current_pwm_set(int duty_percent);

/// @brief Specifies the current reference, in mA.  
///	   The current loop will attempt to maintain this current when state == TRAJECTORY
/// @post  
///	   This function will set the current amps reference, subject to a saturation condition.
///	   Saturate the current at +/- 2000 mA so we do not command too much current
///   	   This function will not change core_state
void current_amps_set(int amps);

/// @brief Reads the current from the ADC, in mA
/// @return The motor current, in mA.
short current_amps_get();

/// @brief Writes the kp and ki gains to the buffer
///
/// @param buffer [out] Write the gains to this buffer. 
/// @pre 		The buffer has a minimum length of 100 characters.
/// @post 		The buffer will contain two integers separated by a ' ' (space)
void current_gains_sprintf(char * buffer);


/// @brief Reads the kp and ki gains from the buffer and sets them accordingly
//
/// @param buffer String containing the kp and ki gains as two integers separated by a ' ' (space )
/// @post  	  The kp and ki gains will be set according to the values read from the buffer.
void current_gains_sscanf(const char * buffer);
#endif
