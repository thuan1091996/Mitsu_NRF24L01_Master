/* --------0.Project information--------------------
 * Mitsubishi FX communication Master
 * Protocol communication: SPI (SSI0), UART (UART1)
 * Debug through LEDs
 * Author : TRAN MINH THUAN
---------------------------------------------------*/

/* ------------------------------1.System requirement-------------------------------
 * 1.Create a simple SSI communication system send and receive 1 byte(SPI.c)
 * 2.Delay function using SYSTICK timer
 * 3.Monitor the system through LEDs
 * 4.Create a protocol to send data through a Master NRF24L01 to NRF24L01 slave
     using Enhanced ShockBurst with IRQ pin Falling edge interrupt
   5.System operating
     * Master - TX - Click Button->Send 0x01 then monitor IRQ Flags(TX_DS or RT_MAX)
     * Slave  - RX - Monitor IRQ Flag (RX_DR)
 * 6.Debugging Notes
6.0- Register information  (get_all --> True register in datasheet)
     + 0x00        ---> 0x00            (CONFIG)
     + 0x01        ---> 0x01
     + 0x02 - 0x06 ---> 0x02 -> 0x06
     + 0x07        ---> 0x07
     + 0x08        ---> 0x08
     + 0x09        ---> 0x09
     + 0x0A - 0x0E ---> 0x0A
     + 0x18 - 0x1C ---> 0x10
     + 0x1D        ---> 0x11
     + 0x23        ---> 0x17
6.1- Can't read the TX_DR and MAX_RT flags in GPIO (IRQ) interrupt
Solution: Change nrf24l01_CONFIG_DEFAULT_VAL 0x08 - 0x28 to disable TX_DR interrupt
6.2- When read multiple from FIFO, the data collected represent just 1st byte
Solution: Read data in RX interrupt
6.3- Master send request to one slave but other slave acknowledge -> wrong if Master immediately
change to RX device
Solution: Master continuously send requests (many times) to slave in an amount of time
 to make sure all slave receive request then change to RX
 (Careful if the amount of time is too much, because then Slave will change back to RX device
 after timeout)
7. Backup history
7.1: 15-12-2018: System work but quite slow and function declaration is not correct, data lost is about 4%
( (Slave_Error + Receive_Error) / Complete *0.01 ~4)
7.2: 20-12-2018: System work, data transfer reach 5% rate (error_slave+error_recv/complete).
7.3: 14-1-2019 : Complete implementation of SIM800A, the system now can communication with 1 Slave and then
upload data to sever
7.4: 28-1-2019 : Add feature to send packet of PLC data to TCP server
7.5: 29-1-2019 : Add new feature - Watchdog-timer will reset system after 100s without any complete to prevent system hangs
-----------------------------------------------------------------------------------*/

/* -----------------2.Pre-processor Directives Section-------------------*/
#include "include.h"
//-------------------Debugging LEDs-----------------------
#define LEDs_DATA_R    (*((volatile unsigned long *)0x40025038)) //LEDs Data Register Addressing
#define LEDs_PORT      SYSCTL_PERIPH_GPIOF
#define LEDs_BASE      GPIO_PORTF_BASE
#define RED_PIN        GPIO_PIN_1
#define BLUE_PIN       GPIO_PIN_2
#define GREEN_PIN      GPIO_PIN_3
//-----------------System States--------------------------
#define RX_STATE       0x00
#define TX_STATE       0x01
//-----------Color from mixing RBG LEDs-------------------
#define RED            0x02
#define GREEN          0x08
#define YELLOW         0x0A
#define BLUE           0x04
#define WHITE          0x0E
#define SKY_BLUE       0x0C
#define PINK           0x06

//-----------------Slave Address---------------------------
#define SLAVE1  1
#define SLAVE2  2
#define SLAVE3  3
//-------------------------------------------------------------//

/* -------------3.Global Declarations Section---------------------*/
//-----------------------Variable for monitor system----------------//
typedef enum System_State System_State; enum System_State {State_Normal, State_Error_nRF24L01, State_Error_Data, State_Error_SIM800A, State_Error_TCP};
unsigned short State_disp=0;            //System state display
unsigned short State=0;                 //System state nrf24l01
unsigned short Error_Data[10]={0};      //Send request complete but doesn't receive respond from Slave
unsigned short Error_Slave[10]={0};     //Send request but slave doesn't respond
unsigned short Complete=0;              //Number of times transmit successfully
//------------------------Variable for nRF24L01--------------------//
unsigned int   ui16_Data_Receive_Length=0;
unsigned short Error_Trans=0;
unsigned char  TX_complete=0;     //NRF24L01 transmission flag
unsigned char  RX_complete=0;     //NRF24L01 transmission flag
unsigned char  ui8Data_RECV_SLAVE[50]={0}; //Data receive from Slaves but don't know if it is correct
unsigned char  ui8Data_RECV_temp[50]={0};  //String hold value to process
unsigned char  ui8Data_Request[12]={0};    //Hold request data to send to Slaves
char  Data_NRF[50];               //Debugging NRF24L01 Registers

//-------------------------Variable for data PLC-------------------//
short Data_D_Slave1[10]={0};      //Data D memory from slave 1
short Data_D_Slave2[10]={0};      //Data D memory from slave 2
short Data_D_Slave3[10]={0};      //Data D memory from slave 3
char  Data_D0[5]={0};
//-----------------------------Variable for TCP connection---------//
unsigned char data_sendtcp[50]={0};
unsigned int relay=0;
unsigned int contactor=0;
unsigned int tanso=0;


//Function declaration
void NRF24L01_Init();
void Monitor_Init(void);
void Monitor(void);
void Change_State_RX(void);
void Change_State_TX(void);
void Data_Packet( unsigned char   *ui8Location,
                  unsigned short  ui16Slave_Addr,
                  unsigned char   ui8Funct,
                  unsigned short  ui16Start_Addr,
                  unsigned char   ui8Size,
                  unsigned short  ui16WriteValue);

void Send_Data_Request(void);

void StrCopy(unsigned char  *Dest,
             unsigned char  *Source,
             unsigned short  length);
void StrCopyINT(unsigned short *Dest,
                unsigned short *Source,
                unsigned short length);

void numbdouble_2char(unsigned short i16numb, char *save_location);

void Data_Collect(unsigned char *ui8Pack_Location);

bool CheckPacket(unsigned char *Location);

bool Master_to_Slave(unsigned short  ui16Slave_Addr,
                     unsigned char   ui8Funct,
                     unsigned short  ui16Start_Addr,
                     unsigned char   ui8Size,
                     unsigned short  ui16WriteValue,
                     unsigned long   ui32Time_100ms);

bool Slave_Responded(unsigned long ui32Limit_time);

void double_2char(unsigned char *ui8save_loc,
                  unsigned short ui16data);
void Packet_TCP(unsigned char *ui8Save_loc,
                unsigned char ui8Slavenumb,
                unsigned char ui8Function,
                unsigned int  ui16Startaddr,
                unsigned int  ui8Size);

bool Timeout(unsigned long Timeout_100ms, unsigned char Number);
/*------------------------------------------------------------------*/

/* ---------------------------4. Subroutines Section--------------------------*/
void main(void)
{
    //-----------------Initialize System------------------------
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
    Systick_Init();                             //1us interrupt
    Monitor_Init();                             //LEDs from port F  (PF1,2,3)
    SSI0_Init();                                //Master,8bit,2Mhz,Mode0
    NRF24L01_Init();                            //PA3-CSN,PA6-CE,PA7-ISR, TX         EN_AA
    UART_Init();                                //UART initialization for connect with SIM800A
    SIM_Init();                                 //SIM initialization (include configuration for TCP)
    Watchdog_Init(100);                         //WDT initialization
    IntMasterEnable();

    //----------------Infinite Loop-------------------------------
    while(true)
    {
        unsigned int Slave_numb=0;
        Slave_numb=SLAVE1;
        if (Master_to_Slave(Slave_numb, D_MEM_READ, 0, 10, 0, 5)==1)        //Send request to Slave and wait for respond
            {
                Error_Slave[Slave_numb]=0;                                  //Slave respond -> clear error history
                if (Slave_Responded(3)==1)
                {
                    Error_Data[Slave_numb]=0;
                    Packet_TCP(data_sendtcp, 1, 3, 0, 10);
                    strcat(data_sendtcp, Newline);
                    if(SIM_TCPSend(data_sendtcp)==1)
                    {
                        Complete++;
                        Watchdog_Feed(100);
                    }
                    str_flush(data_sendtcp, strlen(data_sendtcp));          //flush string to and wait for new
                }
                else Error_Data[Slave_numb]++;  //send request complete but slave doesn't send respond
            }
        else   Error_Slave[Slave_numb]+=1;      //Slave doesn't respond
        delay_us(100000);
    }
}
/* NRF ISR Handler
 * Caused by NRF24L01 PIN PF 7 (FALLING EDGE)
 * Interrupt when IRQ PIN CHANGE FROM HIGH  -> LOW (Falling Edge)
                     +MASTER: TX_DS     - Transmit complete
                              MAX_RT    - Fail to transmit
                     +SLAVE:  RX_DR     - Receive complete
 *Usage: Use to acknowledge the NRF24L01 transmission status. When transmission complete
 then clear all data in FIFO, interrupt flag and then acknowledge interrupt
 * unsigned char transmit_times is used because there a chance when Master send
request to one Slave but other Slave may acknowledge then Master change to RX even though
the true Slave still in RX mode
*/
void NRF_ISRHandler()
    {
    static unsigned char transmit_times=0;
    if      ( (State==TX_STATE) && (nrf24l01_irq_tx_ds_active()) ) //Master sent request complete
            {
                    transmit_times++;
                    if(transmit_times>=1)
                    {
                        TX_complete=1;
                        transmit_times=0;
                    }
            }
    else if ((State==TX_STATE) && (nrf24l01_irq_max_rt_active()) )  //Master fail to send requests
            {
                    Error_Trans++;
            }
    else if ((State==RX_STATE) && (nrf24l01_irq_rx_dr_active()) )   //Master receive data from slave complete
            {
                    nrf24l01_read_rx_payload(&ui8Data_RECV_SLAVE[0],ui16_Data_Receive_Length);
                    RX_complete=1;
            }
    nrf24l01_clear_flush();
    GPIOIntClear(IRQ_BASE,IRQ_MASK);             //Acknowledge interrupt
}

/* NRF24L01 Initialization
 * Function:
         1. Settings for Input pin (IRQ) and Output pins (CE,CSN)
         2. Setup for IRQ interrupt
         3. Setup NRF24L01 Registers
 * Input: No
 * Output: No
 * Change:
         - Change the IRQ Interrupt handler
         - Change another configure for NRF registers
*/
void NRF24L01_Init(){
    //-------1. Setup for GPIO (CSN,CE,IRQ Pins)-----------
    //------------------CE Pin-----------------------
    if(!SysCtlPeripheralReady(CE_PORT))           //Check Clock peripheral
    {
        SysCtlPeripheralEnable(CE_PORT);          //Enable if not ready (busy or disable)
        while (!SysCtlPeripheralReady(CE_PORT));  //Wait for peripheral ready
    }
    GPIOPinTypeGPIOOutput(CE_BASE, CE_PIN);       //CE OUTPUT
    //------------------CSN Pin-----------------------
    if(!SysCtlPeripheralReady(CSN_PORT))          //Check Clock peripheral
    {
        SysCtlPeripheralEnable(CSN_PORT);         //Enable if not ready (busy or disable)
        while (!SysCtlPeripheralReady(CSN_PORT)); //Wait for peripheral ready
    }
    GPIOPinTypeGPIOOutput(CSN_BASE, CSN_PIN);     //CSN OUTPUT
    //------------------IRQ Pin-----------------------
    if(!SysCtlPeripheralReady(IRQ_PORT))          //Check Clock peripheral
    {
        SysCtlPeripheralEnable(IRQ_PORT);         //Enable if not ready (busy or disable)
        while (!SysCtlPeripheralReady(IRQ_PORT)); //Wait for peripheral ready
    }
    GPIOPinTypeGPIOInput(IRQ_BASE, IRQ_PIN);      //IRQ INPUT
//    //----------------2. Setup IRQ Interrupt --------------
    GPIOIntTypeSet(IRQ_BASE, IRQ_PIN, GPIO_LOW_LEVEL);    // NRF24L01 ISR Active "LOW"
    GPIOIntRegister(IRQ_BASE, NRF_ISRHandler);            // Assign interrupt handler
    GPIOIntEnable(IRQ_BASE, IRQ_MASK);                    // Enable interrupt
    //---------------3.Setup NRF Register----------------------
    nrf24l01_initialize_debug(false, 12, true);            //TX,9 Byte, Enhanced ShockBurst
    State=TX_STATE;
//    LEDs_DATA_R=0;
    nrf24l01_irq_clear_all();
    nrf24l01_flush_rx();
    nrf24l01_flush_tx();
}

/* Monitor Debugging System Initialization
 * Function: Initialize I/O for LED pins
 * Input: No
 * Output: No
*/
void Monitor_Init(void)
{
        if(!SysCtlPeripheralReady(LEDs_PORT))        //Check Clock peripheral
        {
            SysCtlPeripheralEnable(LEDs_PORT);       //Enable if not ready (busy or disable)
            while(!SysCtlPeripheralReady(LEDs_PORT));//Wait for peripheral ready
        }
        GPIOPinTypeGPIOOutput(LEDs_BASE, RED_PIN|BLUE_PIN|GREEN_PIN);
}

/* Monitor Debugging System
 * Function: Show the status of the system
               GREEN:             OK
               RED  :             NRF24L01 AFTER LOST CONNECT ABOUT 30S
               YELLOW:            SIM800A  AFTER LOST CONNECT ABOUT 30S
               PINK:               System stop AFTER LOST CONNECT ABOUT 30S
 * Input: No
 * Output: No
 * Change:
          - Change the color of LEDs
          - Add more system states
          - Change monitor time
 * Affect global variable:
          1. "State_disp" - system state variable
          2. "Tick_monitor"
 */
void Monitor(void)
{
    static unsigned int prev_state=0;
    //---------------------------------Change state---------------------------------
    int count_slave_check=1;
//    int count_sms=0;
    if(Error_TCP>=100)            State_disp=State_Error_TCP;
    if(Error_SIM800A>=200)        State_disp=State_Error_SIM800A;
    for(count_slave_check=1;count_slave_check<=1;count_slave_check++)
    {
        if((Error_TCP<100) && (Error_SIM800A<200) &&  (Error_Slave[count_slave_check]<50))  State_disp=State_Normal;
        if(Error_Slave[count_slave_check]>=50)          State_disp=State_Error_nRF24L01;
    }
    //---------------------------------send message-----------------------------------------
//    if((prev_state==State_Normal) && (State_disp==State_Error_TCP))
//    {
//        SIM_Init();
//        for(count_sms=0;count_sms<20;count_sms++)
//        {
//            if(SIM_SMS(PHONE, "Error TCP Connection")==1) break;
//        }
//    }
//    if((prev_state==State_Normal) && (State_disp==State_Error_SIM800A))
//    {
//        SIM_Init();
//        for(count_sms=0;count_sms<20;count_sms++)
//        {
//            SIM_Init();
//            SIM_SMS(PHONE, "Error SIM800A connection");
//        }
//    }
//    if((prev_state==State_Normal) && (State_disp==State_Error_nRF24L01))
//    {
//        for(count_sms=0;count_sms<20;count_sms++)
//        {
//            if(SIM_SMS(PHONE, "Error nRF24L01 connection")==1) break;
//        }
//    }
    //---------------------------------update display colors---------------------------------
    if(State_disp==State_Normal)
    {
        LEDs_DATA_R=0;
        LEDs_DATA_R=GREEN;
    }
    if(State_disp==State_Error_TCP)
    {
        LEDs_DATA_R=0;
        LEDs_DATA_R=WHITE;
    }
    if(State_disp==State_Error_SIM800A)
    {
        LEDs_DATA_R=0;
        LEDs_DATA_R=PINK;

    }
    if(State_disp==State_Error_nRF24L01)
    {
        LEDs_DATA_R=0;
        LEDs_DATA_R=RED;
    }
//    if((Error_SIM800A>300) || (Error_TCP>300)) SIM_Init();
    Tick_monitor=0;
    prev_state=State_disp;  //backup state
}

/* StrCopy(unsigned char *Dest, unsigned char *Source, unsigned short length)
 * Function: Copy "length" byte data in "Source" to "Dest"
 * Input: *Dest      - Destination string
 *        *Source    - Source string
 *        Length    - String length
 * Output: No
*/
void StrCopy(unsigned char *Dest,
             unsigned char *Source,
             unsigned short length)
{
    uint8_t count=0;
    for(count=0;count<length;count++)
    Dest[count]=Source[count];
}

/* StrCopyINT(unsigned short *Dest, unsigned short *Source, unsigned short length)
 * Function: Copy "length" byte data in "Source" to "Dest"
 * Input: *Dest      - Destination string
 *        *Source    - Source string
 *        Length    - String length
 * Output: No
*/
void StrCopyINT(unsigned short *Dest,
                unsigned short *Source,
                unsigned short length)
{
    uint8_t count=0;
    for(count=0;count<length;count++)
    Dest[count]=Source[count];
}

/* numbdouble_2char (unsigned short numb, char *save_location)
 * Function: Convert double -2^15 -> 2^15. If there is a meaningless "0" then it will be delete
 * Input:  ui16numb          - double number to convert
           *save_location    - string location to store value after convert
 * Output: No
*/
void numbdouble_2char(unsigned short i16numb, char *save_location)
{
    uint8_t count_conv=0;
    uint16_t start_div=10000; //max 32768 -> /10000 to get the ten thousand
//-------------------------------Convert number to character--------------------------//
    for(count_conv=0;count_conv<5;count_conv++)
    {
        save_location[count_conv]=i16numb/start_div;
        i16numb=i16numb%(save_location[count_conv]*start_div);
        start_div/=10;
    }
//-------------------------------store value after convert---------------------------//

    for(count_conv=0;count_conv<5;count_conv++)
    {
        save_location[count_conv]=Convert_2Char(save_location[count_conv]);
    }

//-------------------------------Zero elimination------------------------------------//
    if(save_location[0]==0x30)
    {
        save_location[0]=' ';
        if(save_location[1]==0x30)
        {
            save_location[1]=' ';
            if(save_location[2]==0x30)
            {
                save_location[2]=' ';
                if(save_location[3]==0x30)
                {
                    save_location[3]=' ';
                }
            }
        }
    }
//------------------------------------------------------------------------------------//

}

/* bool CheckPacket(unsigned char *Location)
 * Function: Check the package just arrived in RX FIFO to see if it have the right "Frame Data"
by calculate 2 byte CRC then checking the  "STX,ETX, CRC1,CRC0"
   This function using global variable "ui16_Data_Receive_Length" to detect where is the "ETX" byte
in the data frame receive in RX FIFO
 * Input: *Location - Pointer to array location
 * Output: True if data correct (correct data frame) and false if it doesn't
*/
bool CheckPacket(unsigned char *Location)
{
    unsigned long crc_sum=0;
    unsigned char crc_sum_b0,crc_sum_b1;
    unsigned char i;
    unsigned short data_length=ui16_Data_Receive_Length;
    unsigned char etx_location=data_length-3;   //ETX location in array = data length receive - 3
    //Sum calculation
    for(i=0;i<=etx_location;i++) crc_sum+=Location[i];
    crc_sum_b0=crc_sum&0xFF;
    crc_sum=crc_sum>>8;
    crc_sum_b1=crc_sum&0xFF;
    if(     (Location[0] == STX)&&
            (Location[etx_location] == ETX) &&
            (Location[etx_location+1] == crc_sum_b1) &&
            (Location[etx_location+2] == crc_sum_b0)
      )
        return 1;
    else
        return 0;
}

/* void Data_Collect(unsigned char *ui8Pack_Location)
 * Function: - Collect data from SLAVE in RX FIFO when the package has correct data frame structure
             - Use variable "ui16_Data_Receive_Length" to detect how many bytes received then process
             data received and store in correct place (depend on SLAVE number)
 * Input: unsigned char *ui8Pack_Location  - Location of packet to process data
 * Output: None
 * Affect global variables: SLAVE data arrays
 */
void Data_Collect(unsigned char *ui8Pack_Location)
{
   //Number of data bytes receive = Data length - 2 (STX,ETX) - 2 (CRC) - 2 (Start_Addr) - 2 (Slave addr) - 1 Func
   unsigned char  num_of_bytes=(ui16_Data_Receive_Length-9)/2; //divided by 2 because data in WORDs structure
   unsigned short ui16Slave_addr_recv=0;
   unsigned short ui16Start_addr_recv=0;
   unsigned char  start_addr_data=6; //Fix because first is ETX, 2 byte Start addr, 2 byte Slave addr, 1 byte function
   unsigned char  temp_count=0;
   //Start collecting data from SLAVE
   ui16Slave_addr_recv= (ui8Pack_Location[1]<<8)|ui8Pack_Location[2];   //Slave address
   ui16Start_addr_recv= (ui8Pack_Location[4]<<8)|ui8Pack_Location[5];   //Start address of data memory
   if (ui16Slave_addr_recv==1)  //If this is SLAVE 1 ->Store in "Data_D_Slave1"
   {
       for(temp_count=0;temp_count<num_of_bytes;temp_count++)
       {
          Data_D_Slave1[ui16Start_addr_recv]=ui8Pack_Location[start_addr_data]<<8|ui8Pack_Location[start_addr_data+1];
          start_addr_data+=2; //D data in WORD = 2 bytes
          ui16Start_addr_recv++;
       }
   }
   //Store Slave's Data
   if (ui16Slave_addr_recv==2)  //If this is SLAVE 2 ->Store in "Data_D_Slave2"
   {
      for(temp_count=0;temp_count<num_of_bytes;temp_count++)
      {
         Data_D_Slave2[ui16Start_addr_recv]=ui8Pack_Location[start_addr_data]<<8|ui8Pack_Location[start_addr_data+1];
         start_addr_data+=2; //D data in WORD = 2 bytes
         ui16Start_addr_recv++;
      }
   }
   if (ui16Slave_addr_recv==3)  //If this is SLAVE 2 ->Store in "Data_D_Slave2"
   {
         for(temp_count=0;temp_count<num_of_bytes;temp_count++)
         {
            Data_D_Slave3[ui16Start_addr_recv]=ui8Pack_Location[start_addr_data]<<8|ui8Pack_Location[start_addr_data+1];
            start_addr_data+=2; //D data in WORD = 2 bytes
            ui16Start_addr_recv++;
         }
   }
}

/* void Data_Packet( unsigned char   *ui8Location,
                     unsigned short  ui16Slave_Addr,
                     unsigned char   ui8Funct,
                     unsigned short  ui16Start_Addr,
                     unsigned char   ui8Size,
                     unsigned short  ui16WriteValue)
 * Function: Packet the data in data frame structure of 12 bytes
 * Input:
             . *ui8Location    - Pointer to the variable location that the Packet will be saved
             . ui16Slave_Addr  - Slave address (Define in top off main.c)
             . ui8Funct        - Define what the slave need to do
                                     + M_SET        -> Set   M memory
                                     + M_RESET      -> Reset M Memory
                                     + D_MEM_READ   -> Read  D Memory
             . ui16Start_Addr  - Memory address to execute
             . ui8Size         - The number of unit to control
             . ui16WriteValue  - Use if want to WRITE to D memory
 * Output: No
 * Affect global variable "ui16_Data_Receive_Length" to define the length of receive data (very important)
*/
void Data_Packet( unsigned char   *ui8Location,
                  unsigned short  ui16Slave_Addr,
                  unsigned char   ui8Funct,
                  unsigned short  ui16Start_Addr,
                  unsigned char   ui8Size,
                  unsigned short  ui16WriteValue)
{
    unsigned long crc_sum=0;
    unsigned char crc_sum_b0,crc_sum_b1;
    unsigned char slave_addr_b1,slave_addr_b0;
    unsigned char start_addr_b1,start_addr_b0;

    slave_addr_b0=ui16Slave_Addr&0xFF;
    slave_addr_b1=(ui16Slave_Addr>>8)&0xFF;
    start_addr_b0=ui16Start_Addr&0xFF;
    start_addr_b1=(ui16Start_Addr>>8)&0xFF;

    ui8Location[0]=STX;              //Start Signal
    ui8Location[1]=slave_addr_b1;    //Slave address
    ui8Location[2]=slave_addr_b0;    //Slave address
    ui8Location[3]=ui8Funct;
    ui8Location[4]=start_addr_b1;    //Slave function
    ui8Location[5]=start_addr_b0;    //Data
    ui8Location[6]=ui8Size;
    ui8Location[9]=ETX;              //End signal
    unsigned char i;
    for(i=0;i<10;i++)
    crc_sum+=ui8Location[i];
    crc_sum_b0=crc_sum&0xFF;        crc_sum=crc_sum>>8;
    crc_sum_b1=crc_sum&0xFF;
    ui8Location[10]=crc_sum_b1;
    ui8Location[11]=crc_sum_b0;
    ui16_Data_Receive_Length=ui8Size*2+9; //6 bit start, 2*size data + 3 byte end
}

/*void Send_Data_Request(void)
 * Function: Send the packet to SLAVE it's very important to delay between each time sending the packet
 * Input:  None, all data information done in Data_Packet
*  Output: No
*/
void Send_Data_Request(void)
{
    nrf24l01_write_tx_payload(&ui8Data_Request[0],12,true); //Write packet to slave
    delay_us(5000);          //15 (Number of times try to retransmit)*250us (auto retransmit delay) give 5000us so the receive always have result of transmit
}

/* void Change_State_RX(void)
 * Function: Change to RX State include changes State variable and LEDs status
 * Input:  No
 * Output: No
 * Affect global variable: State, LEDs_DATA_R
 */
void Change_State_RX(void)
{
    nrf24l01_set_as_rx(true);     //Change NRF24L01 State
    delay_us(200);                //Wait for state changes
    State=RX_STATE;               //Change state for indicator
//    LEDs_DATA_R=0;
}

/* void Change_State_TX(void)
 * Function: Change to TX State include changes in State variable and LEDs status
 * Input:  No
 * Output: No
 * Affect global variable: State, LEDs_DATA_R
 */
void Change_State_TX(void)
{
    nrf24l01_set_as_tx();         //Change NRF24L01 State
    delay_us(200);                //Wait for state changes
    State=TX_STATE;               //Change state for indicator
//    LEDs_DATA_R=0;
}

/* bool Master_to_Slave( unsigned short  ui16Slave_Addr,
                         unsigned char   ui8Funct,
                         unsigned short  ui16Start_Addr,
                         unsigned char   ui8Size,
                         unsigned short  ui16WriteValue,
                         unsigned long   ui32Time_100ms)
 * Function: Send packet to slave and wait for slave respond within a limited time
 *           Time respond must <= ui32Time_100ms if not return false
 * Input:  unsigned short  ui16Slave_Addr  - Slave Address
           unsigned char   ui8Funct        - Type of communication with slaves
           unsigned short  ui16Start_Addr  - Start Memory address to execute
           unsigned char   ui8Size         - The number of unit to control
           unsigned long   ui16WriteValue  - Use if want to WRITE to D memory
           unsigned long   ui32Time_100ms  - Timeout 1
 * Output: True if slave respond with time limit
           False if don't receive respond from slave
 */
bool Master_to_Slave(unsigned short  ui16Slave_Addr,
                     unsigned char   ui8Funct,
                     unsigned short  ui16Start_Addr,
                     unsigned char   ui8Size,
                     unsigned short  ui16WriteValue,
                     unsigned long   ui32Time_100ms)
{
    nrf24l01_set_rx_pw(12, 0);    //Reset PAYLOAD length to 12 to send request
    delay_us(50);                 //Delay to wait for stable
    Change_State_TX();
    nrf24l01_clear_flush();
    Data_Packet(&ui8Data_Request[0], ui16Slave_Addr, ui8Funct, ui16Start_Addr,ui8Size,ui16WriteValue);
    //If there time run < timeout, this while is valid and wait for respond from slave
    while(Timeout(ui32Time_100ms, 1)==0)    //Timeout1
    {
            if (State==TX_STATE)            //Master is TX device
            {
                if(TX_complete==0)          //If the slave haven't respond ->Send request to slave
                {
                    Send_Data_Request();    //Already have delay inside
                }
                else if (TX_complete==1)    //If the slave receive ->State = RX
                {
                    //Change PAYLOAD width to receive data from slave (receive length depend on request)
                    nrf24l01_set_rx_pw(ui16_Data_Receive_Length, 0);    //Change payload width depend on the size of data will receive
                    delay_us(50);    //Delay wait for stable
                    Change_State_RX();
                    TX_complete=0;
                    //// // // // // // // // // // // // // // // // // // // // //
                    delay_us(1000);  //Change to improve performance
                    //// // // // // // // // // // // // // // // // // // // // //
                    return 1;
                }
            }
    }
    return 0;
}

/* bool Slave_Responded(unsigned long ui32Limit_time)
 * Function: While in RX mode, wait for Slave respond
 * Input:  unsigned long ui32Limit_time - Limit time if Slave hasn't respond within this time -> return 0
 * Output: True if slave respond with time limit
 *         False if don't receive respond from slave
 */
bool Slave_Responded(unsigned long ui32Limit_time)
{
    bool data_correct=0;    //Flag indicate receive data from slave complete and correct
    //This while loop will end if change state or timeout come true
    while( (State==RX_STATE) && (Timeout(ui32Limit_time,0)==0) )
        {
            if(RX_complete==1)      //Receive data before timeout
            {
                GPIOIntDisable(IRQ_BASE,IRQ_MASK);          //Disarm until store new data complete
                StrCopy(&ui8Data_RECV_temp[0], &ui8Data_RECV_SLAVE[0], ui16_Data_Receive_Length);
                if(CheckPacket(&ui8Data_RECV_temp[0])==1)   //If have correct data frame structure
                {
                    Data_Collect(&ui8Data_RECV_temp[0]);    //Collect data
                    RX_complete=0;
                    data_correct=1;
                    //// // // // // // // // // // // // // // // // // // // // //
                    delay_us(1000);
                    //// // // // // // // // // // // // // // // // // // // // //
                    break;
                }
                //if have wrong data, clear flag and then wait for another data come in
                RX_complete=0;   //Receive from slave but data frame is wrong, clear flag and wait for new data
                nrf24l01_irq_clear_all();                    //Clear all interrupt flags in the 24L01
                GPIOIntEnable(IRQ_BASE,IRQ_MASK);
            }
        }
        //after receive true data or timeout, change back to TX device and check to see if received true data
        // return (1) if not return (0)
        if(State==RX_STATE)
        {
            Change_State_TX();
            nrf24l01_clear_flush();
            GPIOIntEnable(IRQ_BASE,IRQ_MASK);
        }
        if (data_correct) return 1;
        else              return 0;
}

/* void double_2char(unsigned char *ui8save_loc,
                     unsigned short ui16data)
 * Function: Convert a double number to 4 character LSB in the last cell
 * Input:  ui16numb          - double number to convert
           *save_location    - string location to store value after convert
 * Output: None
*/
void double_2char(unsigned char *ui8save_loc,
                  unsigned short ui16data)
{
    unsigned int    count_conv=0;
    for(count_conv=0;count_conv<4;count_conv++)
    {
        ui8save_loc[count_conv]=Convert_2Char((ui16data&0xF000)>>12);
        ui16data<<=4;
    }
}

/*void Packet_TCP(  unsigned char *ui8Save_loc,
                    unsigned char ui8Slavenumb,
                    unsigned char ui8Function,
                    unsigned int  ui16Startaddr,
                    unsigned int  ui8Size)
 * Function: Creating packet TCP to send data to TCP server
 * Input: unsigned char *ui8Save_loc    -   The address location to store packet
          unsigned char ui8Slavenumb    -   Slave addresses
          unsigned char ui8Function     -   The function to do
          unsigned int  ui16Startaddr   -   Start address
          unsigned int  ui8Size         -   Size
 * Output: None
 */
void Packet_TCP(unsigned char *ui8Save_loc,
                unsigned char ui8Slavenumb,
                unsigned char ui8Function,
                unsigned int  ui16Startaddr,
                unsigned int  ui8Size)
{
    uint8_t     count_save=0;
    uint8_t     count_data=0;
    uint8_t     sizeb0=0,sizeb1=0;
    count_save=0;
    sizeb0=ui8Size&0x0F;
    sizeb1=(ui8Size>>4)&0x0F;
    ui8Save_loc[count_save]=Convert_2Char(ui8Slavenumb);  count_save++;
    ui8Save_loc[count_save]=Convert_2Char(ui8Function);   count_save++;
    double_2char(&ui8Save_loc[count_save],ui16Startaddr); count_save+=4;
    ui8Save_loc[count_save]=Convert_2Char(sizeb1);        count_save++;
    ui8Save_loc[count_save]=Convert_2Char(sizeb0);        count_save++;
    // 4 Byte each word
    for(count_data=0;count_data<ui8Size;count_data++)
    {
        double_2char(&ui8Save_loc[count_save],Data_D_Slave1[ui16Startaddr]);
        count_save+=4;
        ui16Startaddr++;
    }
}
/*-------------------------------------------------------------------------------*/

