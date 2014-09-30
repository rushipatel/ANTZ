#include "antz.h"
#include "buttons.h"

/*
  Combine the various buttons into a single value with one bit per button
  staring from the least significant position. Up to 8 buttons can be handled
  simultaneously.
*/
#define BUTTONS ((BTN_UP<<B_UP)|(BTN_DN<<B_DN)|(BTN_GO<<B_GO))

/*
  global button state variables set here. These are the processed, 
  debounced button states, not to be confused with the actual state 
  of the pins. 
*/
volatile unsigned char  keyPressed;  // true if any button is pressed
volatile unsigned char  key_GO;      // true if the GO button is pressed
volatile unsigned char  key_UP;      // true if the UP button is pressed
volatile unsigned char  key_DN;      // true if the DN button is pressed

unsigned char buttonState;
unsigned char clock_A,clock_B,delta;
unsigned char changes;
unsigned char debounced_state; 
// assume interval is timer ticks (1.6 ms each)
#define DEBOUNCEINTERVAL 7
unsigned char debounceCounter;

void buttonsInit(void)
{
	// make the pins inputs
	BTN_UP_TRIS |= (1<<BTN_UP_BIT);
	BTN_DN_TRIS |= (1<<BTN_DN_BIT);
	BTN_GO_TRIS |= (1<<BTN_GO_BIT);
	BTN_UP_PU = 1;
	BTN_DN_PU = 1;
	
	key_GO = 0;
	key_UP = 0;
	key_DN = 0;
	keyPressed =  (key_GO) || (key_UP) || (key_DN);
}
  
/*
  simply return the current button state with no processing or debouncing
  also forces the state of the global flags. 
  Use this only if you are not using the timer driven debouncer below
  and want to examine the button hardware directly.
*/
unsigned char readButtons(void)
{
	unsigned char buttons;
	buttons = ~BUTTONS;
	key_GO  = buttons & (1<<B_GO);
	key_UP  = buttons & (1<<B_UP);
	key_DN  = buttons & (1<<B_DN);
	keyPressed =  (key_GO) || (key_UP) || (key_DN);
	return (buttons);
}
/*
  Reading a single button requires the number of the button to read.
  the state returned is that of the corresponding pin. There is no 
  debounce processing. Buttons are numbered 0 to 7.
*/
unsigned char readSingleButton(unsigned char buttonNumber)
{
	return (~BUTTONS & (1 <<buttonNumber));
}

void waitForKeyPress(void)
{
	while (!keyPressed);
	while( keyPressed);
}  

/*
  Button processing uses a vertical counter technique to track changes of state.
  It is hard to say where this idea comes from, it appears in many places on the 
  web and I have not tracked it to its source.
  The technique uses a couple of logical operations to effectively create a two bit
  counter for each button. The function doButtons would normally be called at intervals
  determined by the system clock - perhaps every millisecond. A DEBOUNCEINTERVAL of about 
  10ms would do for most buttons but you could vary that for your hardware.
*/  
void doButtons(void)
{   
	if (++debounceCounter > DEBOUNCEINTERVAL) //
	{
		debounceCounter = 0;
		buttonState = ~BUTTONS;
		delta = buttonState ^ debounced_state;   //Find all of the changes
		// This is a parallel two-bit adder
		clock_A ^=  clock_B;                     //Increment the counters
		clock_B  = ~clock_B;
		
		clock_A &= delta;                       //Reset the counters if no changes
		clock_B &= delta;                       //were detected.
		
		changes = ~(~delta | clock_A | clock_B);
		debounced_state ^= changes;
		key_GO    = debounced_state & (1<<B_GO);
		key_UP    = debounced_state & (1<<B_UP);
		key_DN    = debounced_state & (1<<B_DN);
		keyPressed =  (key_GO) || (key_UP) || (key_DN);
	}
}

