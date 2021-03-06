#include "menu.h"
#include "core.h"
#include "current.h"
#include "streaming.h"
#include "motion.h"
#include "NU32.h"

static char buffer[200]; // used for storing incoming and outgoing requests
static const unsigned int BUF_SIZE = sizeof(buffer)/sizeof(buffer[0]);

static const char assert_fail[] = "\a%s:%d Assertion failed. %s "; // format string for failed assertions

/// @brief The sub-menu related to current control
static void current_menu(void);


/// @brief The sub-menu related to motion control
static void motion_menu(void);


/// @brief The sub-menu related to diagnostics
static void diagnostic_menu(void);


/// @brief Sends a response back to the PC
///	   The response to send is stored in buffer.
///	   "\r\n" will be sent regardless of whether buf ends with "\r\n"
static void send_response(const char * buffer);

void menu_run(void)
{
	core_gains_load();
	while(1)
	{
		NU32_ReadUART1(buffer,BUF_SIZE);	//we expect the next character to be the menu command
		switch (buffer[0])
		{
			case 'i':
			{
				current_menu();
				break;
			}
			case 'm':
			{
				motion_menu();
				break;
			}
			case 'd':
			{
				diagnostic_menu();
				break;
			}
			case 's':
			{
				core_gains_save();
				break;
			}
			case 'l':
			{
				core_gains_load();
				break;
			}
			default: 
			{
				// The final status message starts with a '\a' to indicate an error
				NU32_WriteUART1("\aUnrecognized Command.");
				break;					   
			}
		}
		// there are certain states we should never be in after running a menu command.
		// catch these errors and report them.  These are programming errors and indicate
		// a bug in the code if they are ever triggered.
		if (core_state == TRACK)
		{
			core_state = IDLE;
			sprintf(buffer,assert_fail,__FILE__,__LINE__,"Invalid state TRACK");
			NU32_WriteUART1(buffer);
		}
		else if(core_state == TUNE)
		{
			core_state = IDLE;
			sprintf(buffer,assert_fail,__FILE__,__LINE__,"Invalid state TUNE");
			NU32_WriteUART1(buffer);
		}
		NU32_WriteUART1("\r\n"); //send the acknowledgment that Matlab expects
	}
}


static void current_menu(void)
{
	NU32_ReadUART1(buffer,BUF_SIZE);
	switch (buffer[0])
	{
		case 'k':	// get and set the controller gains
		{
			core_state = IDLE;	// don't change the gains while we could potentially be executing a controller

			// get the current gains as a string and write them to serial
			current_gains_sprintf(buffer);
			send_response(buffer);

			// receive the new gains from serial and set them
			NU32_ReadUART1(buffer,BUF_SIZE);
			current_gains_sscanf(buffer);
			break;
		}
		case 'r':  
		{
			core_state = IDLE;			//stop whatever we were doing
			unsigned int nsamps = 50;		// the number of samples to record
			NU32_ReadUART1(buffer,BUF_SIZE);	// read the number of samples from the uart
			sscanf(buffer,"%d",&nsamps);
			
			streaming_begin(nsamps); 		// setup data streaming

			core_state = TUNE;			// start tuning mode
			streaming_write();			// write the data as it is generated
			core_state = IDLE;			// stop moving
			break;
		}
		case 'b':
		{
			current_pwm_set(0);
			core_state = PWM;
			break;
		}
		case 'c':
		{
			core_state = IDLE;
			break;
		}
		default:	//ignore unrecognized commands
		{
			NU32_WriteUART1("\acurrent_menu: Unrecognized Command.");
			break;
		}
	}
}

static void motion_menu(void)
{
	NU32_ReadUART1(buffer,BUF_SIZE);
	static  int length = 0; //keep track of the loaded motion trajectory length
	switch(buffer[0])
	{
		case 'k':
		{
			core_state = IDLE;
			
			// get the motion controller gains as a string and write them to serial
			motion_gains_sprintf(buffer);
			send_response(buffer);

			NU32_ReadUART1(buffer,BUF_SIZE);
			motion_gains_sscanf(buffer);
			break;
		}
		case 'l': // load trajectory
		{
			//read in the length of the trajectory
			int new_length = 0;
			NU32_ReadUART1(buffer,BUF_SIZE);
			sscanf(buffer,"%d",&new_length);
			if (new_length < 0)
			{
				; // trajectory loading aborted, do nothing
			}
			else if (!motion_trajectory_set(0,new_length-1))
			{
				sprintf(buffer,"\amotion_menu:l Trajectory too long");
				NU32_WriteUART1(buffer);
			}
			else if (new_length > 0)
			{
				length = new_length;
				NU32_WriteUART1("\r\n"); //signal to the pc to start sending the data
				//load the trajectory
				int i = 0, angle = 0;	
				for(i = 0; i != length; ++i)
				{
					NU32_ReadUART1(buffer,BUF_SIZE);
					sscanf(buffer,"%d",&angle);
					motion_trajectory_set(angle,i);
				}
			}
			break;
		}
		case 'x': // execute trajectory
		{
			if (length <= 0)
			{
				//we are expecting the number of extra samples to be sent, so read that
				NU32_ReadUART1(buffer,BUF_SIZE);
				//however, we cannot execute the trajectory
				NU32_WriteUART1("\aCannot Execute, No Trajectory Loaded");
			}
			else
			{
				int xtra = 0;
				NU32_ReadUART1(buffer,BUF_SIZE);
				sscanf(buffer,"%d",&xtra);	//the number of extra samples
				motion_trajectory_reset(LAST,0);//start the trajectory from the beginning, hold at the end
				streaming_begin(length+xtra);	// setup the number of data samples	
				core_state = TRACK;		// track the trajectory
				streaming_write();		// stream the data to the PC
				core_state = HOLD;		// hold the last position
			}
			break;
		}
		case 'h': // hold the current position
		{
			int nsamples = 0;
			//read the number of samples
			NU32_ReadUART1(buffer,BUF_SIZE);
			sscanf(buffer,"%d",&nsamples);
			
			motion_trajectory_reset(NOW,0); // hold at the current angle
			streaming_begin(nsamples);  // setup the number of data samples to stream
			core_state = HOLD;	    // begin holding
			streaming_write();	    // stream the data to the PC
			break;
		}
		case 'g': // goto a position
		{
			int abort = -1;
			NU32_ReadUART1(buffer,BUF_SIZE);
			sscanf(buffer,"%d",&abort);
			if(abort < 0) //the command has been aborted
			{
				;
			}
			else
			{
				//read the angle
				int angle = 0;
				NU32_ReadUART1(buffer,BUF_SIZE);
				sscanf(buffer,"%d",&angle);

				// read the number of samples
				int nsamples = 0;
				NU32_ReadUART1(buffer,BUF_SIZE);
				sscanf(buffer,"%d",&nsamples);

				// set the holding angle
				motion_trajectory_reset(ANGLE,angle);
				//begin streaming and start the goto
				streaming_begin(nsamples);
				core_state = HOLD;
				streaming_write();
			}
			break;
		}
		case 's': // stop trajectory
		{
			core_state = IDLE;
			break;
		}
		default:
		{
			NU32_WriteUART1("\amotion_menu: Unrecognized Command"); 
			break;
		}
	}
}

static void diagnostic_menu(void)
{
	NU32_ReadUART1(buffer,BUF_SIZE);
	switch(buffer[0])
	{
		case 'e':
		{
			//read the encoder count
			sprintf(buffer,"%d\r\n",core_encoder_read());
			NU32_WriteUART1(buffer);
			break;
		}
		case 'd':
		{
			sprintf(buffer,"%d\r\n",motion_angle());
			NU32_WriteUART1(buffer);
			break;
		}
		case 'r':
		{
			//reset the encoder count
			core_encoder_reset();
			break;
		}
		case 'p':
		{
			int duty = 0;
			NU32_ReadUART1(buffer,BUF_SIZE);
			sscanf(buffer,"%d",&duty);
			if (duty < -100 || duty > 100)
			{
				NU32_WriteUART1("\adiagnostic_menu: Enter a duty cycle between -100 and 100");
			}
			else
			{
				current_pwm_set(duty);
				core_state = PWM;
			}
			break;
		}
		case 'a': // read the adc ticks and return them
		{
			sprintf(buffer,"%d\r\n",core_adc_read());
			NU32_WriteUART1(buffer);
			break;
		}
		case 'i': //read the current in mA and return it
		{
			sprintf(buffer,"%d\r\n",current_amps_get());
			NU32_WriteUART1(buffer);
			break;
		}
		case 'x':
		{
			sprintf(buffer,"%d\r\n",core_state);
			NU32_WriteUART1(buffer);
			break;
		}
		default:
		{
			NU32_WriteUART1("\adiagnostic_menu: Unrecognized Command.");
			break;
		}
	}
}


void send_response(const char * buf)
{
	NU32_WriteUART1(buf);
	
	// Make this work regardless of whether buffer ends with "\r\n". Only print it if missing.
	if(buf[strlen(buf) - 1] != '\n')
	{
		NU32_WriteUART1("\r\n");
	}
}
