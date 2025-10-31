#include "uart.h"

void UART_init( void )
{
    UART0_BRD = 26;
    UART0_FBRD = 3;
    UART0_CTRL = 1;
}

void UART_close( void )
{
    UART0_CTRL = 0;
}

void UART_printf(const char *s) {
    while(*s != '\0') {
        UART0_DATA = (unsigned int)(*s);
        s++;
    }
}
