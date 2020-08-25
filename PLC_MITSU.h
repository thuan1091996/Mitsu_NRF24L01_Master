/*
 * PLC_MITSU.h
 *
 *  Created on: Nov 2, 2018
 *      Author: Itachi
 */

#ifndef PLC_MITSU_H_
#define PLC_MITSU_H_


/*--------------------------\\//------------------------------- */
/*-------------------------Define value------------------------*/
#define RX1_PIN         GPIO_PIN_0
#define TX1_PIN         GPIO_PIN_1
#define STX             2                 //Start of TeXt
#define ETX             3                 //End of TeXt
#define M_MEM_READ      0x00              //From Autobase
#define M_SET           0x07              //From Autobase
#define M_RESET         0x08              //From Autobase
#define D_MEM_READ      0x01              //From Autobase
#define D_MEM_WRITE     0x11              //From Autobase


/*---------------------Assign variables------------------------*/
static char     data_recv[15]={0};
static char     data_send_2plc[15];

/* ---------Write_D(uint16_t Start_addr, uint16_t Value)---------
 * Operating: Write D memory of PLC FX series
 * write to D address  (word-2 bytes)
 * Data frame structure collect from Autobase
 * Input:  uint16_t Start_add(Address want to write), uint16_t Value(value to write)
 * Output: None
 * Write D address is 0x11
----------------------------------------------------------------*/
void        Write_D(uint16_t Start_addr, uint16_t Value);

/* ------------Write_M(uint16_t Start_addr, uint8_t Type)------
 * Operating: Write M memory of PLC FX series
 * write to M address (Start_addr) (bit)
 * Data frame structure collect from Autobase
 * Input:  uint16_t Start_add(Address want to write), uint8_t can be either SET or RESET
 * Output: None
 * Write M address is 0x0800
---------------------------------------------------------------*/
void        Write_M(uint16_t Start_addr, uint8_t Type);

/* --------------Read_D(uint16_t Start_addr)--------------------
 * Operating: Read D memory of PLC FX series
 * Read from D address (Start_addr)
 * up to D1023 (maximum)
 * Data frame structure collect from Autobase
 * Input:  Start address (Address want to read)
 * Output: Data value at Start_addr (uint16)
---------------------------------------------------------------*/
uint16_t Read_D(uint16_t Start_addr);

/* ------------------Read_M(uint16_t Start_addr)----------------
 * Operating: Read M memory of PLC FX series
 * Read from address (Start_addr) (bit)
 * Data frame structure collect from Autobase
 * Input:  Start address (Address want to read)
 * Output: Data value at Start_addr (bool)
---------------------------------------------------------------*/
bool        Read_M(uint16_t Start_addr);

uint16_t    ProcessData(uint8_t Mem_Type);

/* ----Calculate_Data(char byte0,char byte1,char byte2,char byte3)---
 * Operating: Convert data receive from PLC to real value
 * Input:  Byte 0,1,2,3 of real data
 * Output: 16-bit Real data
--------------------------------------------------------------------*/
uint16_t    Calculate_Data(char byte0,char byte1,char byte2,char byte3);

/* ----------Convert_2Numb(char char_in)-----------------
 * Operating: Convert character to number
 * Input:  character want to convert to number
 * Output: corresponding number
 * '0' - '9' -> 0 - 9
 * 'A' - 'F' -> 0x0A - 0x0F
-------------------------------------------------------*/
uint8_t     Convert_2Numb(char char_in);   //convert char to uint8

/* ---------Convert_2Char(uint8_t numb_in)--------------
 * Operating: Convert number to character
 * Input:  Number want to convert to char
 * Output: corresponding char character
 * 0 - 9 -> '0' -> '9'
 * 0x0A  ->0x0F return 'A'->'F'
-------------------------------------------------------*/
char        Convert_2Char(uint8_t numb_in);//convert uint8 to char

/* ---------uint16_t Correct_Process(void)--------------
 * Operating: Find the true value of Process function
 * Because Process usually return 0, try (n=200) time if none zero return
 * After 200 time but there no data except 0-> real data =0 return 0
 * Input:  NO
 * Output: True value
 * Change: Change the time n to test, n bigger test more time
 *         -> Increase accuracy but take more time to execute
-------------------------------------------------------*/
uint16_t Correct_Process(uint16_t Mem_type);

#endif /* PLC_MITSU_H_ */
