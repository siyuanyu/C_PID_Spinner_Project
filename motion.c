#include "motion.h"
#include "core.h"
#include "NU32.h"
#include "current.h"
#include "streaming.h"

#define MAX_TRAJ_LEN 1000

//TODO: define variables for:
//		gains (you define what gains you will use)
//		and anything else you may need to run trajectories
static int kp = 700, ki = 10, kd = 20000;
static int eprev = 0, eint = 0, edot = 0, u = 0;
static int traj_length = 0;          // The length of the current trajectory
static int trajectory[MAX_TRAJ_LEN]; // The current trajectory
static int curr_traj = 0; 	     // The current trajectory index
static int hold_angle = 0;	     // The angle to maintain in the HOLD state, in degrees

static void timer2_init(void);

//TODO: define the motion ISR.
//	It should have a similar form to the current.c ISR.
//	Use streaming_record() to send the reference, sensor and control effort to the PC
//	when you are in the TRACK or HOLD states.

void __ISR(_TIMER_2_VECTOR,IPL6SOFT) Motion_Control_Interrupt(void) {
    
    LATEINV = 0b10;
    
    switch (core_state)
	{
		case IDLE:
		{
			break;
		}
        case TRACK:
		{
            int r, s, e;
            r = trajectory[curr_traj];
            s = motion_angle();
            e = r-s;
            edot = (e - eprev);
            eint = eint + e;
            
            if (eint > 200) {
                eint = 200;
            }
            else if (eint < -200) {
                eint = -200;
            }
            else {
                eint = eint;
            }
            
            u = (kp*e + ki*eint + kd*edot)/100;  // calculate the control (current)
            current_amps_set(u);                // send the current to the motor
            streaming_record(r,s,u);
            eprev = e;
            if (curr_traj == traj_length-1) {
                break;
            }
            else {
                curr_traj = curr_traj+1;
            }
            //TODO:
			// in tracking mode the motion.c code sets a current reference
			//using current_amps_set().  THerefore, here, we should make sure
			//that the motion control loop gets the current it requests
            
            //tracking trajectory (stepping to next element of array)
			break;
		}
		case HOLD:
		{
            int r, s, e;
            r = hold_angle;
            s = motion_angle();
            e = r-s;
            edot = (e - eprev);
            eint = eint + e;
            
            if (eint > 200) {
                eint = 200;
            }
            else if (eint < -200) {
                eint = -200;
            }
            else {
                eint = eint;
            }
            
            u = (kp*e + ki*eint + kd*edot)/100;  // calculate the control (current)
            current_amps_set(u);                // send the current to the motor
            streaming_record(r,s,u);
            eprev = e;
            //TODO:
			// make sure that the motion control loop gets the current
			// it has requested via current_amps_set()
            
            //holding track trajectory (single variable)
            curr_traj = 0;
            break;
		}
		default:
		{
			break;
		}
    }
    IFS0bits.T2IF = 0;
}

void motion_init(void)
{
	//TODO: setup E1 for digital output.  This will be used to verify the loop
	//	frequency on the scope.  
	//	The effect of the nScope bug will be very large here so you will
	//	probably see a period that appears too short even when your frequency is correct
	T2CONbits.TCS = 0;
	T2CONbits.TCKPS = 0b011;
	PR2 = 49999;
	TMR2 = 0;
	IPC2bits.T2IP = 6;
	IPC2bits.T2IS = 0;
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
	T2CONbits.ON  = 1;
    //TODO:
	//setup a timer to interrupt at 200Hz.  This is your motion control loop
	//it should be at a lower priority than the current loop
	//register the gains
	// reset the encoder
    
    TRISEbits.TRISE1 = 0;

    core_encoder_reset();
	//TODO: TO save your gains to flash when the save command is issued
	//use core_register_int and core_register_float as appropriate.
	//setup E1 for digital output
    core_register_int(&kp);
	core_register_int(&ki);
    core_register_int(&kd);
}


int motion_angle()
{	
	int angle, encodercount;
    
    encodercount = core_encoder_read();
    angle = (encodercount-32768)*360/396;
    // make sure that increasing PWM increases the angle
	// TODO:
	// use core_encoder_reader() to read the encoder
	// and convert that to an angle.
	// NOTE: 
	// Do not have the angle wrap around.  i.e 0, 360, 720. -360
	// are all considered different position{
	// make sure that increasing PWM increases the angle
	return angle;
}

int motion_trajectory_set(int angle, unsigned int index)
{
    int check;
    
    if ((index+1) <= MAX_TRAJ_LEN) {
        
        trajectory[index] = angle;
        traj_length = index + 1;
        check = 1;
    }
    // the angle is a reference angle
	// and the index is a position in the trajectory array.
	// TODO:
	// FIRST make sure that the index fits inside the array.
	// if the index is out of bounds, return 1.
	// otherwise, set the trajectory reference at the index
	// to the provided angle
	// and set the length of the current trajectory to index + 1
	else {
        check = 0;
    }
    return check;
}

void motion_trajectory_reset(enum ResetMode mode,int angle)
{	
	if (mode == NOW) {
        hold_angle = motion_angle();
    }
    else if (mode == LAST) {
        hold_angle = trajectory[traj_length-1];
    }
    else if (mode == ANGLE) {
        hold_angle = angle;
    }
    eprev = 0;
    eint = 0;
    curr_traj = 0;
    //TODO: see motion.h
    //	based on the mode you will need to
	//	set the holding angle to either
	//	the current position, the end of the last trajectory
	//	or to whatever the parameter angle is.
	//	you should also reset the current trajectory position to 0.
	//	This function is called before a different holding reference
	//	or trajectory is about to be run, so you
	//	can do things that you want done only at the beginning
	//	of a new reference (like setting the accumulated integral
	//	to 0
}

void motion_gains_sprintf(char * buffer)
{	
	//TODO: this is like the current_gains_sprintf,
	//print the gains you use for motion control to the buffer.
	//note: You should edit gains.m so that the format in matlab
	//matches the format here
    sprintf(buffer,"%d %d %d",kp,ki,kd);
}

void motion_gains_sscanf(const char * buffer)
{
    //TODO: read the gains from the buffer using sscanf and store them
    //in the variables you defined to hold the
    sscanf(buffer,"%d %d %d",&kp,&ki,&kd);
}
