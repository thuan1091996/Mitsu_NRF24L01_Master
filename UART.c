/*
 * UART.c
 *
 *  Created on: Jan 12, 2019
 *      Author: Itachi
 */
#include "include.h"
#include "UART.h"

char Data_Recv_SIM[100]={0};           //String store data receive from RX FIFO
int  count_recv_SIM=0;                 //variable count number of data receive from SIM

/* UART 1 INTERRUPT HANDLER
 * Function: Receive data from UART driver then store in Data_Recv_SIM_UART
 * Input: No
 * Output: No
 * Affect to variable:
    - String Data_Recv_SIM - Store data receive from module SIM
    - count_recv_SIM       - Number of character receive
*/
void Receive_ISR(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART1_BASE, true);     //get interrupt status /value true because they're masked interrupt (they are enabled by code)
    UARTIntClear(UART1_BASE, ui32Status);             //clear the asserted interrupts
    while(UARTCharsAvail(UART1_BASE)==1)
    {
        Data_Recv_SIM[count_recv_SIM]=UARTCharGet(UART1_BASE);  //Store data
        count_recv_SIM++;                                       //go to next place holder
    }
}
/* --------UART1 Initialization-----------------------
 * UART1 - RX pin PB0 - TX pin PB1
 * To communicate with SIM800a
 * In order to change, change the PORT or pin or different configuration for UART and UART_ISR
 * Input:  No
 * Output: No
-------------------------------------------------------*/
void UART_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);            //PIN mux for UART
    GPIOPinConfigure(GPIO_PB1_U1TX);            //PIN mux for UART
    GPIOPinTypeUART(GPIO_PORTB_BASE, RX1_PIN |TX1_PIN);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART1);                                //Enable interrupt
    UARTIntRegister(UART1_BASE, Receive_ISR);            //If don't have timeout, system will interrupt after receive 8 bits data
    UARTIntEnable(UART1_BASE, UART_INT_RX| UART_INT_RT); //Receive and receive timeout in case of data don't come after 30 cycles


}
/* UART_TransmitCommand(char *command)
 * Function: Transmit string command to UART device
 * Input: *command - Pointer to command
 * Output: No
*/
void UART_TransmitCommand(char *command)
{
        while(UARTBusy(UART1_BASE));
        while(*command != '\0')
        {
            UARTCharPut(UART1_BASE, *command++);
        }
        UARTCharPut(UART1_BASE,0x0D);                        //Send extra in order for SIM respond
}
/* str_flush(char *string, unsigned int length)
 * Function: Clear the assign string
 * Input: *string - Pointer to string
          *length - The length of string
 * Output: No
*/
void str_flush(char *string, unsigned int length)
{
    int count_str_flush=0;
    for(;count_str_flush<length;count_str_flush++) string[count_str_flush]=0;
}
