
/*
 * PLCMITSU.c
 *
 *  Created on: Nov 2, 2018
 *      Author: Itachi
 */
#include "include.h"
#include "PLC_MITSU.h"

/* --------------Read_D(uint16_t Start_addr)--------------------
 * Operating: Read D memory of PLC FX series
 * Read from D address (Start_addr)
 * up to D1023 (maximum)
 * Data frame structure collect from Autobase
 * Input:  Start address (Address want to read)
 * Output: Data value at Start_addr (uint16)
---------------------------------------------------------------*/
uint16_t Read_D(uint16_t Start_addr){
/*-----Variables for PLC Read------- */
    uint16_t data_d=0;
    uint32_t csum=0;
    uint8_t  csum_low,csum_high;
    uint8_t  Addr,Amount;  //Change if want to configure different protocol
    uint8_t  addr_low,addr_high;
    uint8_t  amount_low,amount_high;
    uint8_t  start_addr_b0,start_addr_b1,start_addr_b2;
    uint8_t  send_count=0;

    //Data processing
    Addr= D_MEM_READ;               //Change if want to connect to different memory address
    addr_low    =   Convert_2Char(Addr&0x0F);
    addr_high   =   Convert_2Char((Addr>>4)&0x000F);

    Start_addr=     Start_addr*2;   //*2 because 2 byte (autobase data showing that)
    start_addr_b0=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4; //2 address per unit
    start_addr_b1=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4;
    start_addr_b2=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4;

    Amount=0x01*2;   //Read 1 word by default, (2 byte-autobase data showing that)
    amount_low    =   Convert_2Char(Amount&0x000F);
    amount_high   =   Convert_2Char((Amount>>4)&0x000F);

    csum = addr_high+addr_low+start_addr_b2+start_addr_b1+start_addr_b0+amount_high+amount_low+ETX;
    csum =      csum&0x00FF;                          // Collect 2 last nibbles for sum check
    csum_low=   Convert_2Char(csum&0x000F);           // Last digit have  4 bits
    csum_high=  Convert_2Char((csum>>4)&0x000F);
    //Package the data
    data_send_2plc[0]=STX;
    data_send_2plc[1]=addr_high;
    data_send_2plc[2]=addr_low;
    data_send_2plc[3]=start_addr_b2;
    data_send_2plc[4]=start_addr_b1;
    data_send_2plc[5]=start_addr_b0;
    data_send_2plc[6]=amount_high;
    data_send_2plc[7]=amount_low;
    data_send_2plc[8]=ETX;
    data_send_2plc[9]=csum_high;
    data_send_2plc[10]=csum_low;
    // Send data_send_2plc to UART transmitter and send to PLC
    for(send_count=0;send_count<11;send_count++){
        UARTCharPut(UART1_BASE, data_send_2plc[send_count]);
        delay_us(50);
    }
    delay_us(10000);
    data_d=Correct_Process(D_MEM_READ);
    return data_d;
}

/* ------------------Read_M(uint16_t Start_addr)----------------
 * Operating: Read M memory of PLC FX series
 * Read from address (Start_addr) (bit)
 * Data frame structure collect from Autobase
 * Input:  Start address (Address want to read)
 * Output: Data value at Start_addr (bool)
---------------------------------------------------------------*/
bool Read_M(uint16_t Start_addr){
        //Calculate variables (SEND)
        uint8_t  Addr;
        uint8_t  addr_low,addr_high;
        uint32_t base_addr=0;
        uint8_t  start_addr_b0,start_addr_b1,start_addr_b2;
        uint8_t  amount,amount_low,amount_high;
        uint32_t csum=0;
        uint8_t  csum_low,csum_high;
        uint8_t  send_count=0;
        //Calculate variables (RECEIVE)
        uint8_t  offset=0;
        uint8_t  return_type=0;
        uint8_t  Bit_shift=0;         //Bit shift  for Read_M_MEM
        bool     Bit_return=0;        //Bit return for Read_M_MEM
        uint16_t ui16Data_Read_M=0;

        //******************  SEND DATA TO PLC *********************//

        Addr= M_MEM_READ;
        addr_low    =   Convert_2Char(Addr&0x0F);
        addr_high   =   Convert_2Char((Addr>>4)&0x000F);
        //Convert Start_add to 3 bytes of character to send to PLC
        //Offset definition
        if          (( Start_addr/10)<4)                            offset=0;
        else if     (((Start_addr/10)>=4)  && ((Start_addr/10)<8))  offset=1;
        else if     (((Start_addr/10)>=8)  && ((Start_addr/10)<12)) offset=2;
        else if     (((Start_addr/10)>=12) && ((Start_addr/10)<16)) offset=3;
        else if     (((Start_addr/10)>=16) && ((Start_addr/10)<20)) offset=4;
        else if     (((Start_addr/10)>=20) && ((Start_addr/10)<22)) offset=5;
        base_addr=  (( Start_addr/10)|0x100)+offset;
        start_addr_b0=  Convert_2Char(base_addr&0x0F);     base_addr>>=4;
        start_addr_b1=  Convert_2Char(base_addr&0x0F);     base_addr>>=4;
        start_addr_b2=  Convert_2Char(base_addr&0x0F);     base_addr>>=4;
        //Size of data
        if ( ( (Start_addr/10)==12 ) || ((Start_addr/10)==19)) amount=0x03;
        else              amount=0x02;
        amount_low    =   Convert_2Char(amount&0x000F);
        amount_high   =   Convert_2Char((amount>>4)&0x000F);
        //Check sum
        csum = addr_high+addr_low+start_addr_b2+start_addr_b1+start_addr_b0+amount_high+amount_low+ETX;
        csum =      csum&0x00FF;                          // Collect 2 last nibbles for sum check
        csum_low=   Convert_2Char(csum&0x000F);           // Last digit have  4 bits
        csum_high=  Convert_2Char((csum>>4)&0x000F);
        //Package the data
        data_send_2plc[0]=STX;
        data_send_2plc[1]=addr_high;
        data_send_2plc[2]=addr_low;
        data_send_2plc[3]=start_addr_b2;
        data_send_2plc[4]=start_addr_b1;
        data_send_2plc[5]=start_addr_b0;
        data_send_2plc[6]=amount_high;
        data_send_2plc[7]=amount_low;
        data_send_2plc[8]=ETX;
        data_send_2plc[9]=csum_high;
        data_send_2plc[10]=csum_low;
        // Send data_send_2plc to UART transmitter and send to PLC
        for(send_count=0;send_count<11;send_count++)
        {
            UARTCharPut(UART1_BASE, data_send_2plc[send_count]);
            delay_us(10);
        }
        delay_us(10000);
        //****************** RECEIVE DATA FROM PLC *********************//
        // When finish collect data from Process there will be a
        // right shift to collect corresponding bit
        return_type= (Start_addr/10)%4; //There are 4 type of return
        Bit_shift=    Start_addr%10;         //Bit shift will be 0-9
        ui16Data_Read_M=Correct_Process(M_MEM_READ);
//        type0 - real data in bit 0-9 of 16bits
        if (return_type==0)
        {
            ui16Data_Read_M=ui16Data_Read_M&0x3FF;
        }
        else if (return_type==1)
        {
            ui16Data_Read_M=(ui16Data_Read_M>>2)&0x3FF;
        }
        else if (return_type==2)
        {
            ui16Data_Read_M=(ui16Data_Read_M>>4)&0x3FF;
        }
        else if (return_type==3)
        {
            ui16Data_Read_M=(ui16Data_Read_M>>6)&0x3FF;
        }
        else;
        Bit_return=(ui16Data_Read_M>>Bit_shift)&0x01;
        return Bit_return;
}

/* ---------Write_D(uint16_t Start_addr, uint16_t Value)---------
 * Operating: Write D memory of PLC FX series
 * write to D address  (word-2 bytes)
 * Data frame structure collect from Autobase
 * Input:  uint16_t Start_add(Address want to write), uint16_t Value(value to write)
 * Output: None
 * Write D address is 0x11
----------------------------------------------------------------*/
void Write_D(uint16_t Start_addr, uint16_t Value){
    /*-----Local Variables ------ */
        uint32_t csum=0;
        uint8_t  csum_low,csum_high;
        uint8_t  Addr,Amount;  //Change if want to configure different protocol
        uint8_t  addr_low,addr_high;
        uint8_t  amount_low,amount_high;
        uint8_t  start_addr_b0,start_addr_b1,start_addr_b2;
        uint8_t  send_count=0;
        uint8_t  value_b0,value_b1,value_b2,value_b3;


        //    Data processing
        Addr        =   D_MEM_WRITE; //Change if want to connect to different memory address
        addr_low    =   Convert_2Char(Addr&0x0F);
        addr_high   =   Convert_2Char((Addr>>4)&0x000F);

        Start_addr=     Start_addr*2;    //*2 because 2 byte (autobase data showing that)
        start_addr_b0=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4; //2 address per unit
        start_addr_b1=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4;
        start_addr_b2=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4;

        Amount=0x01*2;   //Read 1 word by default, (2 byte-autobase data showing that)
        amount_low    =   Convert_2Char(Amount&0x000F);
        amount_high   =   Convert_2Char((Amount>>4)&0x000F);

        value_b0= Convert_2Char(Value&0x0F);     Value>>=4;
        value_b1= Convert_2Char(Value&0x0F);     Value>>=4;
        value_b2= Convert_2Char(Value&0x0F);     Value>>=4;
        value_b3= Convert_2Char(Value&0x0F);     Value>>=4;

        csum = addr_high+addr_low+start_addr_b2+start_addr_b1+start_addr_b0+amount_high+amount_low+value_b0+value_b1+value_b2+value_b3+ETX;
        csum =      csum&0x00FF;                          // Collect 2 last nibbles for sum check
        csum_low=   Convert_2Char(csum&0x000F);           // Last digit have  4 bits
        csum_high=  Convert_2Char((csum>>4)&0x000F);
        //Pack data and send
        data_send_2plc[0]=STX;
        data_send_2plc[1]=addr_high;
        data_send_2plc[2]=addr_low;
        data_send_2plc[3]=start_addr_b2;
        data_send_2plc[4]=start_addr_b1;
        data_send_2plc[5]=start_addr_b0;
        data_send_2plc[6]=amount_high;
        data_send_2plc[7]=amount_low;
        data_send_2plc[8]=value_b1;
        data_send_2plc[9]=value_b0;
        data_send_2plc[10]=value_b3;
        data_send_2plc[11]=value_b2;
        data_send_2plc[12]=ETX;
        data_send_2plc[13]=csum_high;
        data_send_2plc[14]=csum_low;
        for(send_count=0;send_count<15;send_count++)
        {
            UARTCharPut(UART1_BASE, data_send_2plc[send_count]);
            delay_us(10);
        }
        delay_us(10000);
    }

/* ------------Write_M(uint16_t Start_addr, uint8_t Type)------
 * Operating: Write M memory of PLC FX series
 * write to M address (Start_addr) (bit)
 * Data frame structure collect from Autobase
 * Input:  uint16_t Start_add(Address want to write), uint8_t can be either SET or RESET
 * Output: None
 * Write M address is 0x0800
---------------------------------------------------------------*/
void Write_M(uint16_t Start_addr, uint8_t Type){
            uint32_t csum=0;
            uint8_t  csum_low,csum_high;
            uint8_t  start_addr_b0,start_addr_b1,start_addr_b2,start_addr_b3;
            uint8_t  send_count=0;
            // Data processing
            // Type can be SET or RESET
            Type=Convert_2Char(Type);

            Start_addr   =  Start_addr+0x0800;      // Write M address is 0x0800
            start_addr_b0=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4; //2 address per unit
            start_addr_b1=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4;
            start_addr_b2=  Convert_2Char(Start_addr&0x0F);     Start_addr>>=4;
            start_addr_b3=  Convert_2Char(Start_addr&0x0F);
            csum = Type+start_addr_b3+start_addr_b2+start_addr_b1+start_addr_b0+ETX;
            csum =      csum&0x00FF;                          // Collect 2 last nibbles for sum check
            csum_low=   Convert_2Char(csum&0x000F);           // Last digit have  4 bits
            csum_high=  Convert_2Char((csum>>4)&0x000F);
            //Package the data
            data_send_2plc[0]=STX;
            data_send_2plc[1]=Type;
            data_send_2plc[2]=start_addr_b1;
            data_send_2plc[3]=start_addr_b0;
            data_send_2plc[4]=start_addr_b3;
            data_send_2plc[5]=start_addr_b2;
            data_send_2plc[6]=ETX;
            data_send_2plc[7]=csum_high;
            data_send_2plc[8]=csum_low;
            // Send data_send_2plc to UART transmitter and send to PLC
            for(send_count=0;send_count<9;send_count++)
            {
                UARTCharPut(UART1_BASE, data_send_2plc[send_count]);
                delay_us(50);
            }
            delay_us(10000);
}

/* ----Calculate_Data(char byte0,char byte1,char byte2,char byte3)---
 * Operating: Convert data receive from PLC to real value
 * Input:  Byte 0,1,2,3 of real data
 * Output: 16-bit Real data
--------------------------------------------------------------------*/
uint16_t Calculate_Data(char byte0,char byte1,char byte2,char byte3){
    uint16_t cal_data=0;
    byte0=Convert_2Numb(byte0);
    byte1=Convert_2Numb(byte1);
    byte2=Convert_2Numb(byte2);
    byte3=Convert_2Numb(byte3);
    cal_data=(byte3<<12)+(byte2<<8)+(byte1<<4)+byte0;
    return cal_data;
}

/* ----------Convert_2Numb(char char_in)-----------------
 * Operating: Convert character to number
 * Input:  character want to convert to number
 * Output: corresponding number
 * '0' - '9' -> 0 - 9
 * 'A' - 'F' -> 0x0A - 0x0F
-------------------------------------------------------*/
uint8_t  Convert_2Numb(char char_in){
    uint8_t numb_out;
    if(char_in<'A')  numb_out=char_in-0x30; //'0-9' return 0-9
    else             numb_out=char_in-55;   //'A'-'F' return 0x0A-0x0F
    return numb_out;
}

/* ---------Convert_2Char(uint8_t numb_in)--------------
 * Operating: Convert number to character
 * Input:  Number want to convert to char
 * Output: corresponding char character
 * 0 - 9 -> '0' -> '9'
 * 0x0A  ->0x0F return 'A'->'F'
-------------------------------------------------------*/
char     Convert_2Char(uint8_t numb_in){
    char char_out;
    if(numb_in<0x0A)  char_out=numb_in+0x30; //0-9 return '0'-'9'
    else              char_out=numb_in+55;   //0x0A-0x0F return 'A'-'F'
    return char_out;
}

uint16_t ProcessData(uint8_t Mem_Type)
{
    uint16_t data_value=0;
    char    recv_temp=0;         //variable to collect data from UART
    uint8_t Start_process=0;     //processing running flag
    uint8_t byte_count=0 ;       //increase after collect 1 byte data
    uint8_t byte_end=0;          //assign the last value of data frame "ETX"
    uint8_t csum_count_recv=0;   //increase after collect 1 byte checksum data
    uint8_t csum_count_cal=0;    //use to calculate check sum
    uint8_t sum_cal=0;
    uint8_t sum_low,sum_high;
    uint8_t data_correct=0;       //data receive flag

    //Stay here as long as there are data in receive FIFO
    while(UARTCharsAvail(UART1_BASE))
    {
        recv_temp=UARTCharGet(UART1_BASE);
        //Detect START signal
        if(recv_temp==STX)
        {
            byte_count=0;                           //Reset
            Start_process=1;                        //Start the processing
            data_recv[byte_count]=recv_temp;        //Store the STX signal (1st element)
            byte_count++;                           //Next byte
            delay_us(100);
        }
        //If it is not the start or stop bit but start bit already appear
        //Store data value
        else if( (Start_process==1) && (recv_temp!=ETX) )
        {
            data_recv[byte_count]=recv_temp;
            byte_count++;
            delay_us(5000);
        }
        //Detect STOP signal, start ending process, check sum and store value
        else if( (Start_process==1) && (recv_temp==ETX))
        {
            data_recv[byte_count]=recv_temp;  //Store ETX signal (0x03) in DATA[byte_count]
            byte_end=byte_count;              //Detect how many bytes received
            byte_count++;
            delay_us(5000);
            // Collect check sum data from PLC
            for(csum_count_recv=0;csum_count_recv<2;csum_count_recv++)
            {
                //Careful with infinite loop below
                while(UARTCharsAvail(UART1_BASE)==0); //If no data stay in loop, do next instruction when have data
                data_recv[byte_count]=UARTCharGet(UART1_BASE);
                byte_count++;
            }
            // Calculate sum by receiving data
            for(csum_count_cal=1;csum_count_cal<=byte_end;csum_count_cal++)  sum_cal+=data_recv[csum_count_cal];   //Sum=data_recv[1->byte_end]
            sum_low =Convert_2Char(sum_cal&0x0F);             //sum_low (char)
            sum_high=Convert_2Char((sum_cal>>4)&0x0F);        //sum_high (char)
            // Compare data from PLC VS receiving data
            if( (data_recv[byte_end+1]==sum_high) && (data_recv[byte_end+2]==sum_low))    data_correct=1;
            else                                                                          data_correct=0;
        }
        // Ending process
        if(data_correct==1)
            {
                // Store received data (correct data)
                data_value=Calculate_Data(data_recv[2], data_recv[1], data_recv[4], data_recv[3]);
                return data_value;
            }
        }
    return 0;
}

/* ---------uint16_t Correct_Process(uint16_t Mem_type)-------------
 * Operating: Find the true value of Process function
 * Because Process usually return 0, try (n=200) time if none zero return
 * After 200 time but there no data except 0-> real data =0 return 0
 * Input:  uint16_t Data memory type
 * Output: True value
 * Change: Change the time n to test, n bigger test more time
 *         -> Increase accuracy but take more time to execute
-------------------------------------------------------*/
uint16_t Correct_Process(uint16_t Mem_type)
{
    unsigned long temp=0;
    unsigned long temp1=0;
    unsigned int ui16Data=0;
    unsigned long n=250;
    //Try 250 time to see if the data != 0, if not -> true data =0, if there is data =0
    //escape the loop and return true value
    for(temp=0;((temp<n)&&(temp1==0));temp++)
        {
           temp1=ProcessData(Mem_type);
           if (temp1!=0) ui16Data=temp1;
        }
    return ui16Data;
}



