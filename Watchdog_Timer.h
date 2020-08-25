/*
 * Watchdog_Timer.h
 *
 *  Created on: Jan 29, 2019
 *      Author: Itachi
 */

#ifndef WATCHDOG_TIMER_H_
#define WATCHDOG_TIMER_H_

/* Watchdog_Init(unsigned char max_timeout_sec)
 * Function: Initialize for Watchdog-Timer to reset system after max_timeout_sec without WDT interrupt
 * Input:  unsigned char max_timeout_sec - The number of second to reset if the dog didn't feed
           (Maximum 107 second with 80Mhz)
 * Output: None
*/
void Watchdog_Init(unsigned char max_timeout_sec);


/* Watchdog_Feed(unsigned char max_timeout_sec)
 * Function: Reload value for WDT (prevent from RESET system)
 * Input:  unsigned char max_timeout_sec - The number of second to reset if the dog haven't feeded
           (Maximum 107 second with 80Mhz)
 * Output: None
*/
void Watchdog_Feed(unsigned char max_timeout_sec);


#endif /* WATCHDOG_TIMER_H_ */
