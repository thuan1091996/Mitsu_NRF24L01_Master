/*
 * SPI.h
 *
 *  Created on: Oct 26, 2018
 *      Author: Itachi
 */

#ifndef SPI_H_
#define SPI_H_

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
void SSI0_Init(void);

/* SSI (SPI) COMunication
 * Function: Transfer 1 byte and also receive 1 byte from SPI
 * Input:    Unsigned char data to send (byte)
 * Output:   Unsigned char receive from SPI
 * Change this function:
       + Change SSI Port by changing SSI base in these functions (SSIDataPut,SSIBusy,SSIDataGet)
 * Associate with
       + No gobal variable
*/
unsigned char SSI_COM(unsigned char byte);

#endif /* SPI_H_ */
