/*
 * Watchdog_Timer.c
 *
 *  Created on: Jan 29, 2019
 *      Author: Itachi
 */
#include "include.h"
#include "Watchdog_Timer.h"

/* Watchdog_Init(unsigned char max_timeout_sec)
 * Function: Initialize for Watchdog-Timer to reset system after max_timeout_sec without WDT interrupt
 * Input:  unsigned char max_timeout_sec - The number of second to reset if the dog didn't feed
           (Maximum 107 second with 80Mhz)
 * Output: None
*/
void Watchdog_Init(unsigned char max_timeout_sec)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);    //Enable WATCHDOG-timer peripheral
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0));
    if(WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);             //Enable WATCHDOG-timer write access
    }
    WatchdogReloadSet(WATCHDOG0_BASE, (SysCtlClockGet()/2*max_timeout_sec));    //Configure the reload value
    WatchdogResetEnable(WATCHDOG0_BASE);            //Enable reset system feature
    WatchdogEnable(WATCHDOG0_BASE);                 //Enable WATCHDOG-timer
}

/* Feed_WDT(unsigned char max_timeout_sec)
 * Function: Reload value for WDT (prevent from RESET system)
 * Input:  unsigned char max_timeout_sec - The number of second to reset if the dog haven't feeded
           (Maximum 107 second with 80Mhz)
 * Output: None
*/
void Watchdog_Feed(unsigned char max_timeout_sec)
{
    WatchdogReloadSet(WATCHDOG0_BASE,  (SysCtlClockGet()/2*max_timeout_sec));
}
