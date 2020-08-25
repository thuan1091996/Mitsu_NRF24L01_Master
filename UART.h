/*
 * UART.h
 *
 *  Created on: Jan 12, 2019
 *      Author: Itachi
 */

#ifndef UART_H_
#define UART_H_

#define RX1_PIN          GPIO_PIN_0
#define TX1_PIN          GPIO_PIN_1
extern char Data_Recv_SIM[100];             //String store data receive from RX FIFO
extern int  count_recv_SIM;                 //variable count number of data receive from SIM

/* --------UART1 Initialization-----------------------
 * UART1 - RX pin PB0 - TX pin PB1
 * To communicate with SIM800a
 * In order to change, change the PORT or pin or different configuration for UART and UART_ISR
 * Input:  No
 * Output: No
-------------------------------------------------------*/
void UART_Init(void);

/* -------UART 1 INTERRUPT HANDLER--------------------
 * Function: Receive data from UART driver then store in Data_Recv_SIM_UART
 * Input: No
 * Output: No
 * Affect to variable:
    - String Data_Recv_SIM - Store data receive from module SIM
    - count_recv_SIM       - Number of character receive
*/
void Receive_ISR(void);

/* UART_TransmitCommand(char *command)
 * Function: Transmit string command to UART device
 * Input: *command - Pointer to command
 * Output: No
*/
void UART_TransmitCommand(char *command);

/* str_flush(char *string, unsigned int length)
 * Function: Clear the assign string
 * Input: *string - Pointer to string
          *length - The length of string
 * Output: No
*/
void str_flush(char *string, unsigned int length);

#endif /* UART_H_ */
