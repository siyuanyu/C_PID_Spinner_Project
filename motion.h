#ifndef MOTION_H_
#define MOTION_H_
/// @file motion.h
/// @brief Handles motion control
/// @author Your name 
/// @version 1.0
/// @date 2014-03-01

/// @brief Initialize the motion control module
void motion_init(void);

/// @brief Get the angle of the motor 
/// @return the angle of the motor, in degrees
int motion_angle(void);

/// @brief Set the motion trajectory
///
/// @param traj	 The desired angle
/// @param index The index into the trajectory array
/// @return 1 on success, 0 if the index is out of range
/// @post    If the index is in range:
///	     	the trajectory at position index should be the angle
///	     	the length of the current trajectory should be set to index + 1
///	     If the index is out of range, no changes should be made
///		you can also set the error integral to zero
int motion_trajectory_set(int angle, unsigned int index);


/// modes for how to reset the motion controller
// an enumeration is essentially just a list of constants
// so you declare a variable
// as 
// enum ResetMode mode;
// mode can be assigned the values, NOW, LAST, or ANGLE
enum ResetMode {
	        NOW,	//reset to hold the current position
		LAST,   //reset to hold the last position in the trajectory
		ANGLE   //reset to hold a specified angle
		};

/// @brief Resets the motion trajectory to the beginning and updates the holding angle
/// @param mode - the type of reset mode to perform 
///		   if mode == NOW the hold angle is updated to the current motor angle
///		   if mode == LAST the hold angle is updated to the last angle in the trajectory array
///		   if mode == ANGLE the hold angle is updated to the angle specified by the angle parameter
/// @param angle - the new hold angle.  Only used if mode == ANGLE
/// @post - After this function is called:
///		The holding angle should be set appropriately, according to mode
///		The current position in the motion trajectory array should be 0
///		This is called before any new control reference is given.
///		Therefore, it is the place to do tasks that you only want executed when you start trying to match
///		a new reference signal (such as resetting the error integral to zero)		   
void motion_trajectory_reset(enum ResetMode mode,int angle);


/// @brief Writes the motion gains to the buffer.  
///	   You define the gains you need.  Make sure to update the matlab code with the correct format string.
/// @param buffer [out] Write the gains to this buffer
/// @pre   The buffer has a minimum length of 100 characters
/// @post  The buffer will contain the values used as gains separated by ' ' (spaces)
void motion_gains_sprintf(char * buffer);


/// @brief Reads the motion gains from the buffer and sets them accordingly
///
/// @param buffer String containing the gains as numbers separated by ' ' (spaces)
/// @post  The gains that you need for motion control should be set according to
///	   values read from the buffer
void motion_gains_sscanf(const char * buffer);

#endif
