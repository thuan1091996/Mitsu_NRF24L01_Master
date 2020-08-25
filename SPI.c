/*
 * SPI.c
 *
 *  Created on: Oct 26, 2018
 *      Author: Itachi
 */

#include "include.h"
#include "SPI.h"

/* SSI (SPI) Initialization
 * Function: Initialization SSI0 with 2MHz, Motorola Mode0, Master, Band width 8 bits
 * Input: No
 * Output: No
 * Change this function:
       + Change SSI Port by changing GPIO port, SSI base (SysCtlEn & SysCtlReady,SSI Enable)
         then change appropriate pins (PinConfigure,PinTypeSSI)
       + Changing Other SSI characteristic (SSIConfigSetExpClk)
 * Associate with
       + PA2,PA3,PA4,PA5
       + No variable
*/
void SSI0_Init(void)
{
    //Enable peripheral and wait until ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);         //Enable the SSI0 peripheral
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));  //Wait for the SSI0 module to be ready
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        //PORTA for SSI0 pins
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    }
    //PORT pin configure for SSI
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);                 //PA2 CLK
    GPIOPinConfigure(GPIO_PA4_SSI0RX);                  //PA5 TX
    GPIOPinConfigure(GPIO_PA5_SSI0TX);                  //PA4 RX
    GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_2);
    //Configure for SSI -> Clock=2 000 000, 8bit data, master
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 2000000, 8);
    SSIEnable(SSI0_BASE);
}

/* SSI (SPI) COMunication
 * Function: Transfer 1 byte and also receive 1 byte from SPI
 * Input:    Unsigned char data to send (byte)
 * Output:   Unsigned char receive from SPI
 * Change this function:
       + Change SSI Port by changing SSI base in these functions (SSIDataPut,SSIBusy,SSIDataGet)
 * Associate with
       + No gobal variable
*/
unsigned char SSI_COM(unsigned char byte)
{
    uint32_t data_recv;
    SSIDataPut(SSI0_BASE, byte);
    while(SSIBusy(SSI0_BASE));                          //Wait until new data receive
    SSIDataGet(SSI0_BASE, &data_recv);
    return data_recv;                                   //Collect and return data value
}
