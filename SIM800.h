/*
 * SIM800.h
 *
 *  Created on: Jan 12, 2019
 *      Author: Itachi
 */

#ifndef SIM800_H_
#define SIM800_H_

//#define IP_Adress        "113.22.133.71"
#define IP_Adress        "171.252.118.212"


#define PORT             "1996"
#define PHONE            "+84377365720"
#define HOST_SMS         "+84980200030"
extern unsigned short Error_SIM800A;         //Cannot send command to SIM800A
extern unsigned short Error_TCP;             //Cannot connect send data to TCP server
extern char SIM_startTCP_command[50];
extern char OK[4];
extern char Newline[4];


bool SIM_Init(void);
bool SIM_TCPSend(unsigned char *datatcp);
bool SIM_Respond(char *correct_resp);
bool SIM_SendCMD_Comp(char *command,
                      char *correct_res,
                      int   time_try);

bool SIM_SMS(char *phonenumber,
             char *sms_message);


#endif /* SIM800_H_ */
