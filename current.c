#include "current.h"
#include "core.h"
#include "streaming.h"
#include "NU32.h"
#include "motion.h"

#define FULL_DUTY 1999
#define WAVEFORM_SAMPS 50

/// @file current.c
/// @brief Implements the inner current control loop
/// @author Siyuan Yu
/// @version 3.2014
/// @date 2014-03-19

// The proportional and integral gains
// "static" means that these variables are visible only inside current.c
// thus, if you declare similar variables in motion.c as static,
// they can also be named kp and ki and there will be no issue
// (thus these variables are global within the module, but not throughout the 
// whole program)
int currentref = 0;
static int kp = 100, ki = 100, pwmref;
static int waveform[WAVEFORM_SAMPS], waveformcount = 0;
static int eint = 0, u = 0;

/// @brief Setup Timer 1, which runs the current control loop
/// @post  Timer 1 and its interrupt are enabled.  
///        Its frequency is 5 kHz and its interrupt priority should be 7
///// NOTE: static just means that this function is only visible within current.c
///	   code in other files cannot call this function
static void timer1_init(void);

int set_u(int u);

void makeWaveform();
// TODO: this is setup for interrupt priority of 1.
//       You will need to change this.
//       Remember that interrupt priority 7 uses the shadow register set (SRS)
void __ISR(_TIMER_1_VECTOR,IPL7SRS) Current_Control_Interrupt(void)
{
	//TODO: invert E0 so we can see when the interrupt is triggered
    LATEINV = 0x1;
	//the switch stament examines the core_state.
	//it then jumps to the appropriate case (so if core_state = PWM,
	//the switch statement will jump to the PWM case.)
	//every case should end with a break; statement.
	//If you do not end it with a break; statement, the code will fall through
	//to the subsequent case.
	// See core.h for more information about the states.
	//
	// We manage the states for you (via menu.c, take a look if you want)
	// (modifying core state is probably not a good idea as it will make
	// the state machine logic more complicated
	// our demo program only modifies core_state in menu.c 

	// NOTE: Your motion control interrupt will have a similar structure to this
	
	//TODO: when you are ready, remove this statement.
	//	it is only here to make the matlab menu function until
	//	you get your own code up and running.
	//	You will be calling this when core_state == TUNE to record
	//	the response of the current controller to the test reference
	//	(see below).
	//	
	// TODO: this statement to be removed whe you are ready

	switch (core_state)
	{
		case IDLE:
		{
            OC1RS = 0;
            OC2RS = 0;
            eint = 0;
			break;
		}
		case PWM:
		{
			// TODO: set the pwm according to the pwm reference value
            if (pwmref > 0) {
                OC1RS = FULL_DUTY;
                OC2RS = FULL_DUTY-pwmref;
            }
            else if (pwmref < 0) {
                OC1RS = (FULL_DUTY+pwmref);
                OC2RS = FULL_DUTY;
            }
            else if (pwmref == 0) {
                OC1RS = 0;
                OC2RS = 0;
            }
			break;
		}
		case TUNE:
		{
            int r, s, e, newu;
            //TODO:
			// record the sensor, reference, and control effort
			// all you need to do is call the follwoing function here, and it will be streamed to the PC
			//  by code in streaming.c and menu.c.  
			//	streaming_record(reference signal , sensor reading, control effort);
			//
			//	you will also want to use streaming_record in your motion interrupt service routine
			//	when you are in the TRACK or HOLD state

			// in tune mode you are tracking a -200mA to 200mA square wave at 100 Hz.
            //streaming_record( , , control effort);
            r = waveform[waveformcount];
            s = current_amps_get();
            e = r-s;
            eint = e+eint;
            u = (kp*e + ki*eint)/100;
            newu = set_u(u);
            
            if (newu > 0) {
                OC1RS = FULL_DUTY;
                OC2RS = FULL_DUTY-((FULL_DUTY*newu)/2000);
            }
            else if (newu < 0) {
                OC1RS = FULL_DUTY+((FULL_DUTY*newu)/2000);
                OC2RS = FULL_DUTY;
            }
            else {
                OC1RS = 0;
                OC2RS = 0;
            }

            streaming_record(r,s,u);
            
            if (waveformcount < WAVEFORM_SAMPS) {
                waveformcount = waveformcount + 1;
            }
            if (waveformcount == WAVEFORM_SAMPS) {
                waveformcount = 0;
            }
            break;
		}
		case TRACK:
		{
            int r, s, e, newu;
            r = currentref;
            s = current_amps_get();
            e = r-s;
            eint = eint + e;
            u = (kp*e + ki*eint)/100;
            newu = set_u(u);
            
            if (newu > 0) {
                OC1RS = FULL_DUTY;
                OC2RS = FULL_DUTY-((FULL_DUTY*newu)/2000);
            }
            else if (newu < 0) {
                OC1RS = FULL_DUTY+((FULL_DUTY*newu)/2000);
                OC2RS = FULL_DUTY;
            }
            else {
                OC1RS = 0;
                OC2RS = 0;
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
            int r, s, e, newu;
            r = currentref;
            s = current_amps_get();
            e = r-s;
            eint = eint + e;
            u = (kp*e + ki*eint)/100;
            newu = set_u(u);

            if (newu > 0) {
                OC1RS = FULL_DUTY;
                OC2RS = FULL_DUTY-((FULL_DUTY*newu)/2000);
            }
            else if (newu < 0) {
                OC1RS = FULL_DUTY+((FULL_DUTY*newu)/2000);
                OC2RS = FULL_DUTY;
            }
            else {
                OC1RS = 0;
                OC2RS = 0;
            }
            //TODO:
			// make sure that the motion control loop gets the current
			// it has requested via current_amps_set()
            
            //holding track trajectory (single variable)
			break;
		}
		default:	
		{	
			OC1RS = FULL_DUTY;
            OC2RS = FULL_DUTY;
			break;
		}
	}

	IFS0bits.T1IF = 0; // clear the interrupt flag
}

void current_init(void)
{
	//TODO: Uh Oh, somebody made the timer run too fast and at the
	//wrong priority.  FIXME
	// setup the current loop timer for for 5 kHz operation
	timer1_init();
	//TODO: setup the appropriate output compare pins and a timer for
	// 20 kHz PWM operation
    
    T3CONbits.TCKPS = 0b001; // Timer 3 pre-scaler N = 2 (1:2)
    PR3 = FULL_DUTY;
    TMR3 = 0; // Set the initial timer count to 0
    OC1CONbits.OCM = 0b110; // PWM mode without the failsafe for OC1
    OC1CONbits.OCTSEL = 1; // use timer 3
    OC1RS = 0; // Next duty duty cycle is 0
    OC1R = 0; // Initial duty cycle of 0
    OC1CONbits.ON = 1; // Turn on output compare 1
    
    OC2CONbits.OCM = 0b110; // PWM mode without the failsafe for OC2
    OC2CONbits.OCTSEL = 1; // use timer 3
    OC2RS = 0; // Next duty duty cycle is 0
    OC2R = 0; // Initial duty cycle of 0
    T3CONbits.ON = 1; // Turn on timer 3
    OC2CONbits.ON = 1; // Turn on output compare 2
	
    makeWaveform();
    //TODO: setup pin e0 for digital I/O. this is just so that we can
	//verify the control loop frequency on the nscope.
    TRISEbits.TRISE0 = 0;
    
	//NOTE: due to a bug in the nscope firmware, the frequency displayed
	//	may not exactly match what you specify here, but it should be close

	//We register the gains. This allows core.c to handle saving them
	core_register_int(&kp);
	core_register_int(&ki);
}


void current_pwm_set(int duty_percent)
{
    //TODO:
	//this is called by the menu when the user wants to specify
	//pwm duty cycles directly.
	// convert the duty_percent into a duty cycle
	// and assign it to a variable that stores the pwm reference.
	// Your interrupt will set this pwm mode when in the PWM state
    pwmref = (duty_percent/100.00)*FULL_DUTY;
}

void current_amps_set(int amps)
{
    //TODO:
    // set the amp reference
	// saturate at +/- 2000 mA
    if (amps > 2000) {
        currentref = 2000;
    }
    else if (amps < -2000) {
        currentref = -2000;
    }
    else {
        currentref = amps;
    }
}

short current_amps_get()
{
	int adcValue;
    short current;
    
    adcValue = core_adc_read();
    current = 1500*(((float)(adcValue-512))/512);
    // TODO: read from the ADC and convert
	// the tics into mA
	return current;
}

void current_gains_sprintf(char * buffer)
{
	// TODO: sprintf the gains ki and kp into the buffer
	//For Now we return return a dummy buffer with fixed gains
	//you will need to remove this
    sprintf(buffer,"%d %d",kp,ki);
}

void current_gains_sscanf(const char * buffer)
{
	//TODO: buffer will contain the gains as two numbers separated by 
	// a space.  scanf them from buffer and store the gains in
	// kp and ki
    sscanf(buffer,"%d %d",&kp,&ki);
}

void makeWaveform() {
    
    int i, max = 200, min = -200;
    for (i = 0; i < WAVEFORM_SAMPS; i++) {
        if (i < WAVEFORM_SAMPS/2) {
            waveform[i] = min;
        }
        else {
            waveform[i] = max;
        }
    }
}

static void timer1_init(void)
{
	// setup timer 1
	// TODO: Make the timer run at 5kHz
	// whoever coded this is fired!
	T1CONbits.TCS = 0;    	  
	T1CONbits.TCKPS = 0b01;
	PR1 = 1999;
	TMR1 = 0;   	  
	IPC1bits.T1IP = 7;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	T1CONbits.ON  = 1;
}

int set_u(int uvalue) {
    int newu;
    
    if (uvalue >= 2000) {
        newu = 1999;
    }
    else if (uvalue <= -2000) {
        newu = -1999;
    }
    else {
        newu = uvalue;
    }
    return newu;
}