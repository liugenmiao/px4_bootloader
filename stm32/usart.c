/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * LICENSE NOTE FOR EXTERNAL LIBOPENCM3 LIBRARY:
 *
 *   The PX4 development team considers libopencm3 to be
 *   still GPL, not LGPL licensed, as it is unclear if
 *   each and every author agreed to the LGPS -> GPL change.
 *
 ***********************************************************************/

/*
 * USART interface for the bootloader.
 */

#include "hw_config.h"

#include <libopencm3/cm3/nvic.h>
# include <libopencm3/stm32/rcc.h>
# include <libopencm3/stm32/gpio.h>

#include <libopencm3/stm32/usart.h>

#include "bl.h"
#include "uart.h"


#if !defined(USART_SR)
#define USART_SR USART_ISR
#endif
#include "bl.h"
#include "uart.h"
#include <stdio.h>
#include <stdarg.h>
#include "print.h"
#include <string.h>

uint32_t usart;
uint32_t usart_debug;

void uart_buf_put(uint8_t b);
int uart_buf_get(void);

//int heq_printf(const char *fmt,...)
int heq_printf(const char *fmt)
{

	for (int i = 0; i < strlen(fmt); i++)
	{
		usart_send_blocking(usart_debug, fmt[i]);
		//usart_send(usart_debug, fmt[i]);
	}
#if 0
	//void myprintf(char *fmt,...)
    	//va_list ap;
    	//char string[256];

    	//va_start(ap,fmt);
    	//myvsprintf(string,fmt,ap);
    	//Uart_SendString(string);
    	//va_end(ap);
	
	va_list args1;
	va_list args2;
	va_start(args1, fmt);
	va_copy(args2, args1);
	char buf[1+vsnprintf(NULL, 0, fmt, args1)];
	
	va_end(args1);
    	//myvsprintf(string,fmt,ap);
    	myvsprintf(buf,fmt,args2);
	//vsnprintf(buf, sizeof buf, fmt, args2);
	va_end(args2);

	for (int i = 0;  i < sizeof buf; i++)
	{
		usart_send_blocking(usart_debug, buf[i]);
	}
	
#endif
	return 0;
}


/*
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

//预处理实现Linux与Windows的兼容，不同的操作系统函数名不一样
PUTCHAR_PROTOTYPE
{
	usart_send_blocking(usart_debug, (uint8_t)ch);
	return ch;
}
*/

void
uart_debug_cinit(void *config)
{
        usart_debug = (uint32_t)config;

        /* board is expected to do pin and clock setup */

        /* do usart setup */
        //USART_CR1(usart) |= (1 << 15);        /* because libopencm3 doesn't know the OVER8 bit */
	//usart_set_baudrate(usart_debug, USART_BAUDRATE);
	usart_set_baudrate(usart_debug, 921600);
        usart_set_databits(usart_debug, 8);
        usart_set_stopbits(usart_debug, USART_STOPBITS_1);
        usart_set_mode(usart_debug, USART_MODE_TX_RX);
        usart_set_parity(usart_debug, USART_PARITY_NONE);
        usart_set_flow_control(usart_debug, USART_FLOWCONTROL_NONE);

        /* and enable */
        usart_enable(usart_debug);


#if 0
        usart_send_blocking(usart_debug, 'A');
	usart_send_blocking(usart, 'O');
        usart_send_blocking(usart_debug, 'B');
	usart_send_blocking(usart, 'K');
        usart_send_blocking(usart_debug, 'C');
        usart_send_blocking(usart_debug, 'D');

        while (true) {
		usart_send_blocking(usart, 'H');
                usart_recv_blocking(usart_debug);
                //usart_send_blocking(usart_debug, c);
		//
        	heq_printf("Hello World\r\n");
        	//printf("Hello World\r\n");
	}

#endif
}


void
uart_cinit(void *config)
{
	usart = (uint32_t)config;

	/* board is expected to do pin and clock setup */

	nvic_enable_irq(NVIC_USART2_IRQ);
	/* do usart setup */
	//USART_CR1(usart) |= (1 << 15);	/* because libopencm3 doesn't know the OVER8 bit */
	usart_set_baudrate(usart, USART_BAUDRATE);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_mode(usart, USART_MODE_TX_RX);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(usart);

	/* and enable */
	usart_enable(usart);


#if 0
	usart_send_blocking(usart, 'A');
	usart_send_blocking(usart, 'B');
	usart_send_blocking(usart, 'C');
	usart_send_blocking(usart, 'D');

	//while (true) {
	//	int c;
	//	c = usart_recv_blocking(usart);
	//	usart_send_blocking(usart, c);
	//}
#endif

#if  defined(TARGET_HW_PX4_FMU_V3)  
	uart_debug_cinit((void *)BOARD_USART_DEBUG);
	heq_printf("console init success\r\n");
#endif
}

void
uart_cfini(void)
{
	usart_disable(usart);
}

int
uart_cin(void)
{
	int c = -1;

	//if (USART_SR(usart) & USART_FLAG_RXNE) {
	//	c = usart_recv(usart);
	//}
	c = uart_buf_get();
	/*if (c > 0) {
		usart_send_blocking(usart,c);
	}*/
	return c;
}

void
uart_cout(uint8_t *buf, unsigned len)
{
	while (len--) {
		usart_send_blocking(usart, *buf++);
	}
}



void usart2_isr(void)
{
	static uint8_t data = 'A';

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART2);
		uart_buf_put(data);
	}
}




