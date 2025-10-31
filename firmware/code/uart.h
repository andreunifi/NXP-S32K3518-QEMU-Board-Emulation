#ifndef __PRINTF__
#define __PRINTF__

#include "FreeRTOS.h"

#define UART0_ADDRESS                         ( 0x40328000UL )
#define UART0_DATA                            ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 0UL ) ) ) )
#define UART0_STATE                           ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 0x004 ) ) ) )
#define UART0_CTRL                            ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 0x030 ) ) ) )
#define UART0_BRD                             ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 0x024 ) ) ) )
#define UART0_FBRD                            ( *( ( ( volatile uint32_t * ) ( UART0_ADDRESS + 0x028 ) ) ) )


void UART_init(void);
void UART_close(void);
void UART_printf(const char *s);


#endif
