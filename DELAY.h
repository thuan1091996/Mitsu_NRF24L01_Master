/*
 * delay.h
 *
 *  Created on: Oct 26, 2018
 *  Author: Itachi
 */

#ifndef DELAY_H_
#define DELAY_H_

extern unsigned long Tick;          //Reference from main.c
extern unsigned long Tick_monitor;  //Reference from main.c
extern unsigned long Tick_timeout;  //Reference from main.c
extern bool Timeout_active;


/* SYSTICK Initialization
 * Function:- Initialization for Systick timer then load 80  ->(80 * 1/(80*10^6) = 1us)
            - STAY IN LOOP until delay enough amount of time except when Systick interrupt execute
            - Use "HWREG" to change register value (more info in HW_type.h)
 * Input: No
 * Output: No
 * Change this function:
            + Change the Load value to create different time delay interval
            + Change the interrupt handler by "SystickIntRegister" function
*/
void Systick_Init();

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
void Systick_ISR();

/* Delay us second by SYSTICK
 * Function: Delay micro second corresponding to "microseconds"
 * Input: The amount of microseconds to delay
 * Output: No
 * Change: Use flag so the system is able to jump to another thread
           rather than stick in 1 line code
-------------------------------------------------------*/
void delay_us(unsigned int microseconds);

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
bool Timeout(unsigned long Timeout_100ms, unsigned char Number);

#endif /* DELAY_H_ */
