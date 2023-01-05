
/******************************************************************************
* File Name:   me310g1-ww.c
*
* Description: This is the source code for RDK2 Rutronik development kit Application
*              for ModusToolbox.
*
*
*  Created on: 2022-12-20
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Authors: BAS
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "stdio.h"
#include "stdint.h"
#include "me310g1-ww.h"

#define BAUD_RATE       115200
#define DEFAULT_TIMEOUT	5000
#define STR_BUF_SIZE    256
#define INT_PRIORITY    3

cyhal_uart_t modem_uart_obj;
cyhal_uart_t terminal_uart_obj;
uint32_t actualbaud;
uint8_t str_buf[STR_BUF_SIZE];
_Bool modem_uart_flag=false; // modem uart flag
_Bool terminal_uart_flag=false; // terminal uart flag
uint8_t modem_rx_buf[STR_BUF_SIZE];
uint8_t mrb_i=0; // modem ring buffer index
uint8_t crb_i=0; // command ring buffer index
uint32_t timer_i=0;
cyhal_timer_t timer_obj;
char command_buf[128];


cy_rslt_t timer_init(void)
{
	 const cyhal_timer_cfg_t timer_cfg =
	 {
	     .compare_value = 0,                 /* Timer compare value, not used */
	     .period = 9,                        /* Defines the timer period */
	     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
	     .is_compare = false,                /* Don't use compare mode */
	     .is_continuous = true,              /* Run the timer indefinitely */
	     .value = 0                          /* Initial value of counter */
	 };

	 cy_rslt_t result;

	 result = cyhal_timer_init(&timer_obj, NC, NULL);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_configure(&timer_obj, &timer_cfg);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_set_frequency(&timer_obj, 10000);
	 if (result != CY_RSLT_SUCCESS)
	 {CY_ASSERT(0);}

	 cyhal_timer_register_callback(&timer_obj, timer_isr, NULL);
	 cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

	 return result;
}

void timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;
    timer_i++;
    /*Is called every ten milliseconds*/
    //SCP_Tick(10);
}

void stop_timer(cyhal_timer_t* timer_obj)
{
	cyhal_timer_stop(timer_obj);
	cyhal_timer_reset(timer_obj);
	timer_i=0;
}

void terminal_uart_init(void)
{
	/* Initialize the UART configuration structure */
	const cyhal_uart_cfg_t terminal_uart_config =
	{
		.data_bits = 8,
		.stop_bits = 1,
		.parity = CYHAL_UART_PARITY_NONE,
		.rx_buffer = NULL,
		.rx_buffer_size = 0
	};
	cy_rslt_t result;
	result = cyhal_uart_init(&terminal_uart_obj, KITPROG_TX, KITPROG_RX, NC, NC,  NULL, &terminal_uart_config);
	if (result != CY_RSLT_SUCCESS)
	{CY_ASSERT(0);}
	/* Set the baud rate */
	result = cyhal_uart_set_baud(&terminal_uart_obj, BAUD_RATE, &actualbaud);
	/* The UART callback handler registration */
	cyhal_uart_register_callback(&terminal_uart_obj, terminal_uart_event_handler, NULL);
	/* Enable required UART events */
	cyhal_uart_enable_event(&terminal_uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_TX_DONE), INT_PRIORITY, true);
}

void modem_uart_init(void)
{
	/* Initialize the UART configuration structure */
	const cyhal_uart_cfg_t modem_uart_config =
	{
		.data_bits = 8,
		.stop_bits = 1,
		.parity = CYHAL_UART_PARITY_NONE,
		.rx_buffer = NULL,
		.rx_buffer_size = 0
	};
	cy_rslt_t result;
	result = cyhal_uart_init(&modem_uart_obj, ARDU_TX, ARDU_RX, NC, NC,  NULL, &modem_uart_config); //&modem_uart_config
	if (result != CY_RSLT_SUCCESS)
	{CY_ASSERT(0);}
	/* Set the baud rate */
	result = cyhal_uart_set_baud(&modem_uart_obj, BAUD_RATE, &actualbaud);
	/* The UART callback handler registration */
	cyhal_uart_register_callback(&modem_uart_obj, modem_uart_event_handler, NULL);
	/* Enable required UART events */
	cyhal_uart_enable_event(&modem_uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY), INT_PRIORITY, true);
}

/* Event handler callback function */
void modem_uart_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    if ((event & CYHAL_UART_IRQ_RX_NOT_EMPTY) == CYHAL_UART_IRQ_RX_NOT_EMPTY)
    {
    	cyhal_uart_getc(&modem_uart_obj, &modem_rx_buf[mrb_i], 0);
    	mrb_i++;
    	if(mrb_i==255)
    		mrb_i=0;
    	modem_uart_flag=true;
    }
}

/* Event handler callback function */
void terminal_uart_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    if ((event & CYHAL_UART_IRQ_TX_DONE) == CYHAL_UART_IRQ_TX_DONE)
    {
    	terminal_uart_flag=true;
    }
}

void modem_init(void)
{
	cy_rslt_t result;
    /*Initialize ME310G1 Modem GPIOs*/
    result = cyhal_gpio_init(ARDU_IO3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1); /*Power Control for SMPS*/
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init(ARDU_IO4, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0); /*USB PD Alarm Input*/
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init(ARDU_IO5, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*USB PD Reset Control*/
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init(ARDU_IO6, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*Modem Reset Control*/
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init(ARDU_IO7, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*Modem Wake-Up Control*/
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init(ARDU_IO8, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0); /*Modem ON/OFF Control*/
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    //POWER-ON sequence
	cyhal_gpio_write(ARDU_IO3, 0);
	CyDelay(100);

	if(!send_to_modem("AT\r\n", 20, "OK"))
	{
		cyhal_gpio_write(ARDU_IO8, 1);
		CyDelay(5000);
		cyhal_gpio_write(ARDU_IO8, 0);
	}
}

void modem_connect(void)
{
    if(!send_to_modem("AT#SGACT?\r\n", DEFAULT_TIMEOUT, "1,1"))   //Checks/connects to LTE/2G network
    {
    	send_to_modem("AT#SGACT=1,1\r\n", DEFAULT_TIMEOUT*2, "OK");
    }

    if(!send_to_modem("AT#LWM2MENA?\r\n", DEFAULT_TIMEOUT, "1"))	//Checks/connects to LWM2M network
    {
    	send_to_modem("AT#LWM2MENA=1\r\n", DEFAULT_TIMEOUT*2, "OK");
    }
//    if(!send_to_modem("AT#LWM2MMON?\r\n", DEFAULT_TIMEOUT, "33001"))  //Checks/activates LWM2M object monitoring if updates are made from TELIT server side
//    {
//    	send_to_modem("AT#LWM2MMON=1,33001\r\n", DEFAULT_TIMEOUT*2, "OK");
//    }
    modem_uart_flag=0;
}

_Bool send_to_modem(char* command, uint32_t timeout, char* reply)
{
	_Bool state=0;
	char reply_buf[64];
	int16_t sb_i=0;

	sprintf((char *__restrict)reply_buf, reply);
    sb_i=sprintf((char *__restrict)str_buf, (const char *__restrict)command);
	cyhal_uart_write_async(&modem_uart_obj, str_buf, sb_i);
	stop_timer(&timer_obj);
    cyhal_timer_start(&timer_obj);

	while(timer_i<=timeout+1)
	{
		if(strstr((const char *)modem_rx_buf, (const char *)reply_buf) != NULL)
		{
			state=1;
			break;
		}
		if(timeout==0)
			timer_i=0;
	}
	if(timer_i>timeout)
		state=0;

	stop_timer(&timer_obj);
	clear_buffer((char*)modem_rx_buf, mrb_i);
	mrb_i=0;
	CyDelay(20);//minimum delay before sending another command to the modem
	return state;
}

void send_to_terminal(char* command)
{
    crb_i=sprintf((char *__restrict)command_buf, command);
    cyhal_uart_write_async(&terminal_uart_obj, command_buf, crb_i);
    while(!terminal_uart_flag)
    	cyhal_system_delay_ms(10);
    clear_buffer(command_buf, crb_i);
}

void clear_buffer(char *buffer, int16_t index)
{
	if(index<0)
		CY_ASSERT(0);
	while(index>0)
	{
		buffer[index]=0x00;
		index--;
	}
	buffer[index]=0x00;
}
