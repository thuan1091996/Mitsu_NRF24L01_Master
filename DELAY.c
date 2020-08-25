/*
 * DELAY.c
 *
 *  Created on: Oct 26, 2018
 *      Author: Itachi
 */
#include "include.h"
#include "DELAY.h"

unsigned long  Tick=0;
unsigned long  Tick_monitor=0;
unsigned long  tick_Timeout[5]={0};
unsigned long  time_Timeout[5]={0};
unsigned long  active_Timeout[5]={0};

/* SYSTICK Initialization
 * Function:- Initialization for Systick timer then load 80  ->(80 * 1/(80*10^6) = 1us)
            - STAY IN LOOP until delay enough amount of time except when Systick interrupt execute
            - Use "HWREG" to change register value (more info in HW_type.h)
            - Enable SYSTICK Interrupt
 * Input: No
 * Output: No
 * Change this function:
            + Change the Load value to create different time delay interval
            + Change the interrupt handler by "SystickIntRegister" function
*/
void Systick_Init(){
    SysTickIntRegister(Systick_ISR);    //Define  Systick ISR
    HWREG(NVIC_ST_CTRL)&=~0x07;         //Disable Systick Timer during initialization
    HWREG(NVIC_ST_RELOAD)=80-1;
    HWREG(NVIC_ST_CURRENT)=0;           //Write any value to clear the counter
    HWREG(NVIC_ST_CTRL)|=0x07;          //Enable Systick Timer and ARM for Systick interrupt
}

/* Delay us second by SYSTICK
 * Function: Delay the amount of "microseconds" micro second
 * Input: The amount of microseconds to delay
 * Output: No
 * Change: Use flag so the system is able to jump to another thread
           rather than stuck in 1 line code
-------------------------------------------------------*/
void delay_us(unsigned int microseconds) {
    Tick=0;                             //Reset count value
    while(Tick < microseconds);
}

/* SYSTICK Overflow Interrupt Handler
 * The SYSTICK timer automatically load value from RELOAD_R so there no need to update new value
 * Interrupt after each 1us
 * After 100ms: Execute monitor to indicate system state
                Increase Tick_timeout
 * Input:  No
 * Output: No
 * Global variable affect:
                 - "Tick"
                 - "Tick_monitor"
                 - "active_Timeout[]"
                 - "tick_Timeout[]"
*/
void Systick_ISR()

{
    unsigned char count_Timeout=0;
    Tick++;             //Increase every 1 us corresponding to Reload value
    Tick_monitor++;
    if(Tick_monitor>=100000)
    {
            SysTickDisable();
            for(count_Timeout=0;count_Timeout<5;count_Timeout++)
            {
                if(active_Timeout[count_Timeout]==1)  tick_Timeout[count_Timeout]++;
            }
            Monitor();
            SysTickEnable();
    }
};

/* Timeout(unsigned long Timeout_100ms)
 * Function: Create a loop after "Timeout_100ms" will break that loop and return 1
 * Input: - unsigned long "Timeout_100ms": time in microseconds
          - unsigned char "Number": the Timeout block use because more than one Timeout can be used at a time
 * Output: 1 if enough time in loop
 * Change: Change time in loop, change the Number to using more Timeout block, then must add more code in Systick_ISR
and the definition of 2 arrays using in code bellow because the code is using just 5 Timeout block
 * Affect global variable:
          1. "Timeout_active"
          2. "Tick_timeout"
 */
bool Timeout(unsigned long Timeout_100ms, unsigned char Number)
{
    time_Timeout[Number]=Timeout_100ms;
    active_Timeout[Number]=1;
    if(tick_Timeout[Number]>=time_Timeout[Number])
        {
            tick_Timeout[Number]=0;
            active_Timeout[Number]=0;
            return 1;
        }
        else return 0;
}
