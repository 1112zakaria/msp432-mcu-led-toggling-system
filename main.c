/*
#	Date: November 22, 2021
#
#	Team #: 17
#	Team Members:
#		Daniah Mohammed - 101145902
#		Ethan Leir - 101146422
#		Zakaria Ismail - 101143497
#	Lab Section: L2
*/


#include "msp.h"
#include "toggle_leds.h"


int main() {
	
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;            // Halt the WDT
	run();	// run toggle_leds.c program

	return 0;
}


