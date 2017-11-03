#include "NU32.h"          // plib.h, config bits, constants, funcs for startup and UART
#include "core.h"
#include "current.h"  	   // the current module contains functions related to controlling the current.
#include "motion.h"	   // the motion controller module
#include "menu.h"

int main() 
{
	NU32_Startup(); 			// cache on, min flash wait, interrupts on, LED/button init, UART init
	INTDisableInterrupts(); 		// turn off interrupts
	core_init();				// initialize the core system
	current_init();				// initialize the current control module
	motion_init();
	INTEnableSystemMultiVectoredInt();	// enable interrupts
	
	menu_run();
	return 0;
}


