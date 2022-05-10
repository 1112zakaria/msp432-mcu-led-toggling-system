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

// Is this allowed?
#include "msp.h"

#ifndef TOGGLE_LEDS_H
	#define TOGGLE_LEDS_H
		
	#define DELAY_VALUE 65535
	#define INITIAL_LED_STATE 0
	#define INITIAL_LED 0
	
	typedef enum {
		SWITCH_LED_STATE			=	0,
		SWITCH_SELECTED_LED		= 1
	} LED_Op_Type;
	
	typedef enum {
		STOP				=	TIMER_A_CTL_MC__STOP >> 1,
		UP					=	TIMER_A_CTL_MC__UP >> 1,
		CONTINUOUS	=	TIMER_A_CTL_MC__CONTINUOUS >> 1,
		UPDOWN			=	TIMER_A_CTL_MC__UPDOWN >> 1
	} Timer_Mode_Type;
	
	// toggle_leds.c header declarations
	void configure_IO(void);
	void initialize_IO(void);
	void run(void);
	// LED functions
	void perform_LED_operation(LED_Op_Type);
	void switch_selected_LED(uint8_t*, uint8_t*);
	void switch_LED_state(uint8_t*, uint8_t*);
	void set_LEDs(uint8_t, uint8_t);
	// Timer functions
	void reset_timer(uint16_t);
	void set_timer_mode(uint16_t, Timer_Mode_Type);
	
#endif

