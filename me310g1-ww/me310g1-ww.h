/******************************************************************************
* File Name:   me310g1-ww.h
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

#ifndef ME310G1_WW_ME310G1_WW_H_
#define ME310G1_WW_ME310G1_WW_H_

/*
 * timer_init() - Initializes the timer for timing events
 *
 * @note Used for delays and waiting time
 *
 * @param none
 *
 * @return CY_RSLT_SUCCESS on success, an error code otherwise
 */
cy_rslt_t timer_init(void);

/*
 * timer_isr() - Controls timer ticks
 *
 * @note Used for counting
 *
 * @param callback_arg
 *
 * @param event - interrupt triggers
 *
 * @return CY_RSLT_SUCCESS on success, an error code otherwise
 */
void timer_isr(void *callback_arg, cyhal_timer_event_t event);

/*
 * modem_uart_init() - Initializes the UART peripheral for terminal
 *
 * @note This UART is used for communication with modem
 *
 * @return none
 */
void terminal_uart_init(void);

/*
 * terminal_uart_init() - Initializes the UART peripheral for terminal
 *
 * @note This UART is used for communication with terminal
 *
 * @return none
 */
void modem_uart_init(void);

/*
 * modem_uart_event_handler() - ISR for handling modem UART event
 *
 * @note Used to detect communication from modem side
 *
 * @param handler_arg - handler arg
 *
 * @param event - event to service
 *
 * @return none
 */
void terminal_uart_event_handler(void *handler_arg, cyhal_uart_event_t event);

/*
 * terminal_uart_event_handler() - ISR for handling terminal UART event
 *
 * @note Used to detect TX done events
 *
 * @param handler_arg - handler arg
 *
 * @param event - event to service
 *
 * @return none
 */
void modem_uart_event_handler(void *handler_arg, cyhal_uart_event_t event);

/*
 * modem_init() - Initializes the modem
 *
 * @note POWER ON sequence using GPIO pins
 *
 * @param none
 *
 * @return none
 */
void modem_init(void);

/*
 * timer_isr() - ISR for handling timer events
 *
 * @note Used to time events
 *
 * @param handler_arg - handler arg
 *
 * @param event - event to service
 *
 * @return none
 */
void timer_isr(void *callback_arg, cyhal_timer_event_t event);

/*
 * stop_timer() - stops and resets the timer
 *
 * @note stop and reset to 0
 *
 * @param timer_obj - pointer to timer to stop
 *
 * @return none
 */
void stop_timer(cyhal_timer_t* timer_obj);

/*
 * modem_connect() - establishes connection through LWM2M to Telit cloud via modem
 *
 * @note Through LTE/2G network with a SIM card
 *
 * @param none
 *
 * @return none
 */
void modem_connect(void);

/*
 * clear_buffer() - cleans buffer
 *
 * @note only the values up to and including the index
 *
 * @param buffer - buffer to clean
 *
 * @param index - index to clean from backwards
 *
 * @return none
 */
void clear_buffer(char* buffer, int16_t index);

/*
 * send_to_modem() - function to send a string to modem
 *
 * @note through UART
 *
 * @param command - string to send
 *
 * @param timeout - waiting time for reply, 0 for infinity
 *
 * @param reply - expected reply
 *
 * @return true or false if the expected reply was received
 */
_Bool send_to_modem(char* command, uint32_t timeout, char* reply);

/*
 * send_to_terminal() - function to send a string to terminal
 *
 * @note through UART
 *
 * @param command - string to send
 *
 * @return true none
 */
void send_to_terminal(char* command);

#endif /* ME310G1_WW_ME310G1_WW_H_ */
