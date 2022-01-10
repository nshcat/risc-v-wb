#include "rv32.h"
#include "device.h"

uint32_t pwm_val = 127;

const uint32_t animation[10] = {
	0, 30, 70, 120, 180, 230, 180, 120, 70, 30
};

uint32_t counter;

void handle_tim1()
{
	LED_STATE = ~LED_STATE;
	
	TIMER1_CMPV1 = animation[counter];
	
	++counter;
	
	if(counter >= 10)
		counter = 0;

	return;
}

int main()
{
	LED_STATE = 0b1010;
	counter = 0;
	
	// Setup timer 1. It will be used to change brightness every 200 ms.
	TIMER1_PRETH = 24999;		                    	// Prescaler: 25MHz / 25000 => 1 KHz
	TIMER1_CNTTH = 199;			                    	// Counter: 1KHz / 200 = 5 Hz
	TIMER1_CONTROL = TIMER_ENABLE | TIMER_ENABLE_IRQ;	// Enable timer 1 and its interrupts
	
	// Setup timer 2. It will be used to implement PWM.
	TIMER2_PRETH = 249;			                    	// Prescaler: 25MHz / 250 = 100 KHz
	TIMER2_CNTTH = 255;			                    	// 256 steps of PWM resolution
	TIMER2_CMPV1 = 0;		                        	// 0% initial brightness
	TIMER2_CONTROL = TIMER_ENABLE | TIMER_ENABLE_CMP1;	// Enable timer 2 and its first comparator output
	
	// Enable interrupt handling for timer interrupt 1
	EIC_MASK = 0b1;
	
	while(1);
	
	return 0;
}

void exc_handler(uint32_t cause, uint32_t details)
{
	while(1);
}