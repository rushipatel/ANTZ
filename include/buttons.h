
#ifndef BUTTONS_H
#define BUTTONS_H


extern volatile unsigned char  keyPressed;
extern volatile unsigned char  key_GO;
extern volatile unsigned char  key_UP;
extern volatile unsigned char  key_DN;

void buttonsInit(void);
unsigned char readButtons(void);
unsigned char readSingleButton(unsigned char buttonNumber);
void waitForKeyPress(void);
void doButtons(void);

#endif
