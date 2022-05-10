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

#include "toggle_leds.h"
#include "msp.h"

/**
*	Configures switches P1.1 & P1.4
*	as PULLUP INPUT pins and LEDs
*	P1.0, P2.0, P2.1, and P2.2 as
*	OUTPUT pins.
*
*	@param	none
*	@return	none
*/
void configure_IO(void) {
	
	// Set Function Select registers for General Purpose I/O
	P1->SEL0 &= (uint8_t) (~(BIT0 | BIT1 | BIT4));	// Clear BIT0, BIT1, BIT4
	P1->SEL1 &= (uint8_t) (~(BIT0 | BIT1 | BIT4));	// Clear BIT0, BIT1, BIT4
	P2->SEL0 &= (uint8_t) (~(BIT0 | BIT1 | BIT2));	// Clear BIT0, BIT1, BIT2
	
	// Configure switches P1.1 & P1.4 as PULLUP INPUT
	P1->DIR &= (uint8_t) (~(BIT1 | BIT4));	// Clear BIT1, BIT4
	P1->OUT |= (uint8_t) (BIT1 | BIT4);			// Set BIT1, BIT4
	P1->REN |= (uint8_t) (BIT1 | BIT4);			// Set BIT1, BIT4 
	
	// Configure LEDs P1.0, P2.0, P2.1, P2.2 as OUTPUT
	P1->DIR |= (uint8_t) (BIT0);								// Set BIT0
	P2->DIR |= (uint8_t) (BIT0 | BIT1 | BIT2);	// Set BIT0, BIT1, BIT2
	
	// Configure device interrupts
	P1->IE |= (uint8_t) (BIT1 | BIT4);		// Enable P1.1 and P1.4 interrupt
	P1->IE &= (uint8_t) (BIT1 | BIT4);		// Disable all other port interrupts
	P1->IES |= (uint8_t) (BIT1 | BIT4);		// Set P1.1 and P1.4 to falling-edge interrupt
	P1->IFG &= (uint8_t) 0;								// Clear Port 1 Interrupt Flag Register
	
	// Configure Timer A0
	TA0CTL &= ~(uint16_t)(BIT4 | BIT5);												// Stop Timer
	TA0CTL = (TA0CTL & (uint16_t)0) | (uint16_t)(BIT1|BIT2);	// Enable interrupt and clear timer
	TA0CTL |= (uint16_t) (BIT8);															// Select ACLK source and no divide
	TA0CCR0 = (uint16_t)8192;																	// Set capture/compare to trigger at 0.3 seconds
	
	// Configure Timer A1
	TA1CTL &= ~(uint16_t)(BIT4 | BIT5);												// Stop Timer
	TA1CTL = (TA1CTL & (uint16_t)0) | (uint16_t)(BIT1|BIT2);	// Enable interrupt and clear timer
	TA1CTL |= (uint16_t) (BIT8);															// Set ACLK source and no divide
	TA1CCR0 = (uint16_t)8192;																	// Set capture/compare to trigger at 0.3 seconds
	
	// Configure Timer A2
	TA2CTL &= ~(uint16_t)(BIT4 | BIT5);												// Stop Timer
	TA2CTL = (TA2CTL & (uint16_t)0) | (uint16_t)(BIT1|BIT2);	// Enable interrupt and clear timer
	TA2CTL |= (uint16_t) (BIT8);															// Set ACLK source and no divide
	TA2CTL |= (uint16_t) BIT4;																// Set TA2 to up mode
	TA2CCR0 = (uint16_t)32768;																// Set capture/compare to trigger at 1 seconds
	
	// Configure NVIC registers
	// For PORT1
	// IRQ # FOR PORT 1 = 35
	NVIC_ClearPendingIRQ((IRQn_Type)PORT1_IRQn);						// Clear pending interrupts from PORT 1 in NVIC
	NVIC_SetPriority((IRQn_Type)PORT1_IRQn, (uint32_t)4);		// Setting Port 1 Interrupt Priority
	NVIC_EnableIRQ((IRQn_Type)PORT1_IRQn);									// Enable Port 1 Interrupts on NVIC
	// For TIMER TA0 N interrupts
	
	NVIC_ClearPendingIRQ((IRQn_Type)TA0_N_IRQn);
	NVIC_SetPriority((IRQn_Type)TA0_N_IRQn, (uint32_t)5);
	NVIC_EnableIRQ((IRQn_Type)TA0_N_IRQn);
	
	// For TIMER TA1 N interrupts
	NVIC_ClearPendingIRQ((IRQn_Type)TA1_N_IRQn);
	NVIC_SetPriority((IRQn_Type)TA1_N_IRQn, 5);
	NVIC_EnableIRQ((IRQn_Type)TA1_N_IRQn);
	
	// For TIMER TA2 N interrupts
	NVIC_ClearPendingIRQ((IRQn_Type)TA2_N_IRQn);
	NVIC_SetPriority((IRQn_Type)TA2_N_IRQn, 6);
	NVIC_EnableIRQ((IRQn_Type)TA2_N_IRQn);
	
	// Enable interrupts globally inside the processor
	__ASM("CPSIE I");
	
}

/**
*	Initializes the LED states
*	as off.
*	
*	@return	none
*/
void initialize_IO() {

	// set the LED registers. Initialize to Red and OFF
	set_LEDs((uint8_t)INITIAL_LED, (uint8_t)INITIAL_LED_STATE);
	
}
/**
* Handles PORT1 device interrupts.
*/
void PORT1_IRQHandler(void) {

	NVIC_ClearPendingIRQ((IRQn_Type)PORT1_IRQn);
	if ( (P1->IFG & (uint8_t)BIT1) != (uint8_t)0 ) {
		// P1.1 triggered the interrupt
		perform_LED_operation(SWITCH_SELECTED_LED);
		
		// Question: should I use TACLR bit or can I directly
		//	read and write to TA0R?
		P1->IE &= (uint8_t) (~BIT1);	// Disable P1.1 interrupt
		P1->IFG &= (uint8_t)~BIT1;
		TA0R = (uint16_t)0;	// Set to TA0 counter to 0
		//reset_timer(TA0R);
		TA0CTL |= (uint16_t) BIT4;	// Set TA0 to up mode
		//set_timer_mode(TA0CTL, (Timer_Mode_Type)UP);
		
		TA2R = (uint16_t)0;		// Reset the interval timer
		//reset_timer(TA2R);
	} else if ( (P1->IFG & (uint8_t)BIT4) != (uint8_t)0) {
		// P1.4 triggered the interrupt		
		P1->IE &= (uint8_t) (~BIT4);	// Disable P1.4 interrupt
		P1->IFG &= (uint8_t)~BIT4;
		TA1R = (uint16_t)0;	// Set to TA0 counter to 0
		//reset_timer(TA1R);
		TA1CTL |= (uint16_t) BIT4;	// Set TA1 to up mode
		//set_timer_mode(TA1CTL, (Timer_Mode_Type)UP);
		
		TA2CTL ^= (uint16_t) BIT1;		// Toggle timer interrupt enable
		if ( TA2CTL & (uint16_t)BIT1 ) {
			// if toggle timer interrupt enabled	
			TA2CTL |= (uint16_t) BIT4;	// Set TA1 to up mode
			//set_timer_mode(TA2CTL, (Timer_Mode_Type)UP);
		} else {
			TA2CTL &= ~(uint16_t)(BIT4 | BIT5);	// Stop the timer
			//set_timer_mode(TA2CTL, (Timer_Mode_Type)STOP);
		}
		TA2R = (uint16_t)0;						// Set timer to 0;
		//reset_timer(TA2R);
		
	}
		
}

/**
* Handles TIMER A0 N interrupts.
* Used to time P1.1 debouncing.
*/

void TA0_N_IRQHandler(void) {
	
	NVIC_ClearPendingIRQ((IRQn_Type)TA0_N_IRQn);

	TA0CTL &= (uint16_t)~BIT0;		// lower the pending interrupt
	TA0CTL &= ~(uint16_t)(BIT4 | BIT5);	// Stop the timer
	//set_timer_mode(TA0CTL, (Timer_Mode_Type)STOP);
	TA0R = (uint16_t)0;	// Set the timer to 0
	//reset_timer(TA0R);
	P1->IE |= (uint8_t) (BIT1);	// Enable P1.1 interrupt
	
	//TA2R = (uint16_t)0;		// Reset the interval timer

}


/**
* Handles TIMER A1 N interrupts.
* Used to time P1.4 debouncing.
*/

void TA1_N_IRQHandler(void) {
	NVIC_ClearPendingIRQ((IRQn_Type)TA1_N_IRQn);

	TA1CTL &= (uint16_t)~BIT0;		// lower the pending interrupt
	TA1CTL &= ~(uint16_t)(BIT4 | BIT5);	// Stop the timer
	//set_timer_mode(TA1CTL, (Timer_Mode_Type)STOP);
	//reset_timer(TA1R);
	TA1R = (uint16_t)0;	// Set the timer to 0
	P1->IE |= (uint8_t) (BIT4);	// Enable P1.4 interrupt

}

/**
* Handles TIMER A2 N interrupts.
* Used to toggle LED state every second.
*/

void TA2_N_IRQHandler(void) {
	NVIC_ClearPendingIRQ((IRQn_Type)TA2_N_IRQn);

	TA2CTL &= (uint16_t)~BIT0;		// lower the pending interrupt
	//reset_timer(TA2R);
	TA2R = (uint16_t)0;	// Set the timer to 0
	perform_LED_operation(SWITCH_LED_STATE);
}

/**
* "Wrapper" function for LED functions.
* Stores LED state variables on stack.
*/
void perform_LED_operation(LED_Op_Type operation) {
	static uint8_t current_LED = INITIAL_LED;
	static uint8_t current_state = INITIAL_LED_STATE;
	
	if (operation == SWITCH_SELECTED_LED) {
		switch_selected_LED(&current_LED, &current_state);
	} else if (operation == SWITCH_LED_STATE) {
		switch_LED_state(&current_LED, &current_state);
	}
}

/**
*	Switches to the LED not in use,
*	either the Red or the RGB LED.
*	
*	@param	curr_LED		uint8_t*, pointer to current LED value
*	@param	curr_state	uint8_t*, pointer to current state value
*	@return	none
*/
void switch_selected_LED(uint8_t* curr_LED, uint8_t* curr_state) {
	
	// Switch to the other LED (This could be simplified)
	*curr_LED ^= (uint8_t)1;
	// Set the state to 1 (This could be simplified)
	*curr_state = (uint8_t)1;
	
	// Set the LED registers
	set_LEDs(*curr_LED, *curr_state);
}

/**
*	Changes LED state to the next
*	state.
*
*	@param	curr_LED		uint8_t*, pointer to current LED value
*	@param	curr_state	uint8_t*, pointer to current state value
*	@return	none
*/
void switch_LED_state(uint8_t* curr_LED, uint8_t* curr_state) {
	/*
	*	Formula for toggling to the next
	*	state:
	*		next_state = (current_state + 1) % num_states
	*/
	uint8_t num_states;
	
	// Get num states (2 for Red, 8 for RGB)
	num_states = *curr_LED == (uint8_t)0 ? (uint8_t)2 : (uint8_t)8;
	// Set current state to the next state
	*curr_state = (*curr_state + (uint8_t)1) % num_states;
	
	// Set LED registers
	set_LEDs(*curr_LED, *curr_state);
	
}

/**
*	Sets the LEDs on and off corresponding
*	to the values of curr_LED and curr_state
*
*	@param	curr_LED		uint8_t, current LED value
*	@param	curr_state	uint8_t, current state value	
*/
void set_LEDs(uint8_t curr_LED, uint8_t curr_state) {
	
	// Set all LEDs off (P1.0, P2.0, P2.1, P2.2)
	P1->OUT &= (uint8_t) (~(BIT0));								// Clear BIT0
	P2->OUT &= (uint8_t) (~(BIT0 | BIT1 | BIT2));	// Clear BIT0, BIT1, BIT2
	
	// Set LED corresponding to curr_LED and curr_state
	if (curr_LED == (uint8_t)0) {
		// Red LED
		P1->OUT |= curr_state;		// Set BIT0 w/ curr_state
	} else if (curr_LED == (uint8_t)1) {
		// RGB Led
		P2->OUT |= curr_state;		// Set BIT0, BIT1, BIT2 w/ curr_state
	}
	
}

/**
* Resets the timer to 0.
*/
void reset_timer(uint16_t timer_counter) {
	// I hope this doesn't break everything haha
	timer_counter = (uint16_t)0;
}

/**
* Sets the timer's mode.
*/
void set_timer_mode(uint16_t timer_control, Timer_Mode_Type mode) {
	timer_control &= ~(BIT5 | BIT4);	// Clear MC bits
	timer_control |= mode;	// YOU DONT WANT TO SET ALL THE OTHER BITS IDIOT
}

/**
*	Runs the infinite loop
*	waiting for switch input
*	and behaving accordingly.
*	
*	@param	none
*	@return	none
*/
void run(void) {
	
	configure_IO();
	initialize_IO();
	
	while (1) {
		__ASM("WFI");
	}
		
	
}



