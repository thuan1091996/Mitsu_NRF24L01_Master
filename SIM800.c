/*
 * SIM800.c
 *
 *  Created on: Jan 12, 2019
 *      Author: Itachi
 */
#include "include.h"
#include "SIM800.h"
char SIM_startTCP_command[50]={"AT+CIPSTART=TCP,"};
char OK[4]={'O','K',0x0D,0x0A};
char Newline[4]={0x0D,0x0A};
unsigned short Error_SIM800A=0;         //Cannot send command to SIM800A
unsigned short Error_TCP=0;             //Cannot connect send data to TCP server
/* SIM_Init(void)
 * Function: Initialization for SIM800 and TCP connection
 * Input:    None
 * Output:   None
*/
bool SIM_Init(void)
{
   uint8_t send_try=0;
   SIM_SendCMD_Comp("AT",OK,10);          //Checking module operation and synchronize Speed -> OK
   SysCtlDelay(SysCtlClockGet()/30000);
   SIM_SendCMD_Comp("ATE0", OK, 2);       //Disable Echo back
   SysCtlDelay(SysCtlClockGet()/30000);   //Delay after send command successfully (100us)
   SIM_SendCMD_Comp("AT+CIPMUX=0",OK,2);  //Single connection -> OK
   SysCtlDelay(SysCtlClockGet()/30000);                            //Delay after send command successfully (100us)
   SIM_SendCMD_Comp("AT+CPIN?",OK,2);     //Check for Password ->OK
   SysCtlDelay(SysCtlClockGet()/30000);                            //Delay after send command successfully (100us)
   SIM_SendCMD_Comp("AT+CREG?",OK,2);     //Check register network -> OK
   SysCtlDelay(SysCtlClockGet()/30000);                            //Delay after send command successfully (100us)
   SIM_SendCMD_Comp("AT+CGATT=1",OK,2);   //Attach GPRS       -> OK
   SysCtlDelay(SysCtlClockGet()/30000);                            //Delay after send command successfully (100us)
   SIM_SendCMD_Comp("AT+CIPSHUT",OK,2);   //Close all previous connection -> SHUT "OK"
   SysCtlDelay(SysCtlClockGet()/30000);
   strcat(SIM_startTCP_command, IP_Adress); //creating start TCP connection message
   strcat(SIM_startTCP_command, ",");
   strcat(SIM_startTCP_command, PORT);
   for(send_try=0;send_try<20;send_try++)
   {
       if(SIM_SendCMD_Comp(SIM_startTCP_command,"ALREADY CONNECT",5)==1) break;  //If connect OK
   }
//   for(send_try=0;send_try<20;send_try++)
//   {
//       if(SIM_TCPSend("CONNECTED")==1)  return 1;
//       SysCtlDelay(SysCtlClockGet()/1000);
//   }
   return 0;
}

/* SIM_Respond(char *correct_resp)
 * Function: Check the respond from SIM count_correct increase 1 every time to character is correct
 * Input: *correct_resp - the correct respond from SIM
 * Output:
             -1:The Data_Recv_SIM contain correct_resp
             -0:Received wrong information
*/
bool SIM_Respond(char *correct_resp)
{
    int8_t count_correct=0;                        //increase 1 each time if both character is the same
    count_recv_SIM=0;                              //reset count receive UART to compare
    while(Data_Recv_SIM[count_recv_SIM]!=0)        //receive string hasn't end yet
    {
        if (correct_resp[count_correct] == Data_Recv_SIM[count_recv_SIM]) count_correct++;
        count_recv_SIM++;
        if(count_correct>=strlen(correct_resp))
        return 1;
    }
    return 0;
}

/* SIM_SendCMD_Comp(char *command,
                    char *correct_res,
                    int  time_try)
 * Function: Send command to SIM800 and check for respond if false re-send with limit time try
     0. Clear the receive string and its place holder
     1. Check to see if reach the limit time try
     2. If not and hasn't receive appropriate respond then send command and wait for respond
 * Input:  *command         - command to send to SIM
           *correct_res     - expected respond
            time_try        - max time re-send command
 * Output:
           1: send and receive correct respond
           0: if not
 * Change function by changing the delay time
 * Affect variable:  Error_SIM_send_cmd++s
*/
bool SIM_SendCMD_Comp(char *command,
                      char *correct_res,
                      int  time_try)
{
    uint16_t count_send_cmd=0;  //re-send command if SIM doesn't respond
    for(count_send_cmd=0;count_send_cmd<time_try;count_send_cmd++)   //Send maximum time_try times and wait for "OK" respond
    {
        count_recv_SIM=0;                                    //clear receive string and counter receive
        str_flush(Data_Recv_SIM,strlen(Data_Recv_SIM));      //in order to receive clean value and delete old things
        UART_TransmitCommand(command);
        SysCtlDelay(SysCtlClockGet()/1000);                      //Delay after send command successfully
        if(SIM_Respond(correct_res)==1)
        {
            Error_SIM800A=0;
            return 1;
        }
    }
    Error_SIM800A++;
    return 0;
}

/* SIM_TCPSend(char *datatcp)
 * Function: Send datatcp to TCP server
     1. Connect to TCP
     2. Send data to TCP
 * Input:  char *datatcp - pointer to data to send
 * Output: No
 * Change function by changing the max times try in each SIM_CMD_COMP and delay time to wait for respond
*/
bool SIM_TCPSend(unsigned char *datatcp)
{
    int  count_try=0;
    if(SIM_SendCMD_Comp(SIM_startTCP_command,"ALREADY CONNECT",2)==1)  //If connect OK
    {
        Error_SIM800A=0;                                               //Can connect to SIM800A
        if(SIM_SendCMD_Comp("AT+CIPSEND",">",3)==1)                    //Send and wait for ">"
        {
                Error_SIM800A=0;
                count_recv_SIM=0;                                      //clear receive string and counter receive
                str_flush(Data_Recv_SIM,strlen(Data_Recv_SIM));        //in order to receive clean value and delete old things
                UART_TransmitCommand(datatcp);
                UARTCharPut(UART1_BASE, 0X1A);
                for(count_try=0;count_try<1000;count_try++)
                {
                    SysCtlDelay(SysCtlClockGet()/10000);

                    {
                        Error_TCP=0;
                        Error_SIM800A=0;
                        return 1;
                    }
                }
                Error_TCP++;
        }
        else Error_SIM800A++;
    }
    Error_TCP++;
    Error_SIM800A++;
    return 0;
}


bool SIM_SMS(char *phonenumber,
             char *sms_message)
{
    int  count_try=0;
    char phone_char[30]={0x22};
    char SIM_sms_start[30]="AT+CMGS=";

    char host_message[30]={0x22};
    char SIM_host_start[30]="AT+CSCA=";
// PRocessing data
    strcat(phone_char,phonenumber);
    phone_char[13]=0x22;
    strcat(SIM_sms_start,phone_char);

    strcat(host_message,HOST_SMS);
    host_message[13]=0x22;
    strcat(SIM_host_start,host_message);

//    SIM_SendCMD_Comp("AT",OK,10);          //Checking module operation and synchronize Speed -> OK
//    SysCtlDelay(SysCtlClockGet()/3000);
//    SIM_SendCMD_Comp("AT+CIPSHUT",OK,2);   //Close all previous connection -> SHUT "OK"
//    SysCtlDelay(SysCtlClockGet()/30000);
//    SIM_SendCMD_Comp("AT+CGATT=0",OK,2);   //Attach GPRS       -> OK
//    SysCtlDelay(SysCtlClockGet()/30000);
//    SIM_SendCMD_Comp("AT+CREG=0;",OK,2);     //Check register network -> OK
//    SysCtlDelay(SysCtlClockGet()/30000);
    for(count_try=0;count_try<200;count_try++)
    {
    if(SIM_SendCMD_Comp("AT+CMGF?","OK",5)==1)
        {
            SysCtlDelay(SysCtlClockGet()/100);
            if(SIM_SendCMD_Comp(SIM_host_start,OK,5)==1) //Trung tam tin nhan ngan han
            {
                SysCtlDelay(SysCtlClockGet()/100);
                if(SIM_SendCMD_Comp(SIM_sms_start,">",5)==1)
                {
                    count_recv_SIM=0;                                      //clear receive string and counter receive
                    str_flush(Data_Recv_SIM,strlen(Data_Recv_SIM));        //in order to receive clean value and delete old things
                    UART_TransmitCommand(sms_message);
                    SysCtlDelay(SysCtlClockGet()/1000);
                    UARTCharPut(UART1_BASE, 0X1A);
                    for(count_try=0;count_try<100000;count_try++)
                    {
                        SysCtlDelay(SysCtlClockGet()/1000);
                        if(SIM_Respond("OK")==1)
                        {
                            Error_SIM800A=0;
                            return 1;
                        }

                   }
                }
            }
        }
    }
    return 0;
}
