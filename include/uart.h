
#ifndef UART_H
#define UART_H

void serialInit(void);
int putUART(char c);
int getUART(void);
int putsUART(char * s);

#endif
