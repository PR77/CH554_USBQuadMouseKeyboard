/********************************** (C) COPYRIGHT *******************************
* File Name     : SYSTEM.C
* Author        : Zhiyuan Wan
* License       : MIT
* Version       : V1.0
* Date          : 2018/03/17
* Description   : 8051 System Functions
********************************************************************************
* Modified      : Paul Raspa (PR77)
* License       : MIT
* Version       : V2.0
* Date          : 2024/12/17
*******************************************************************************/

#include <stdint.h>
#include <compiler.h>
#include "ch554.h"
#include "system.h"

/*******************************************************************************
* Function Name  : system_CfgFsys( )
* Description    : CH554 clock selection and configuration function.
*******************************************************************************/
void system_CfgFsys(void)  {
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;

#if FREQ_SYS == 32000000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x07; // 32MHz
#elif FREQ_SYS == 24000000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x06; // 24MHz
#elif FREQ_SYS == 16000000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x05; // 16MHz
#elif FREQ_SYS == 12000000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x04; // 12MHz
#elif FREQ_SYS == 6000000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x03; // 6MHz
#elif FREQ_SYS == 3000000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x02; // 3MHz
#elif FREQ_SYS == 750000
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x01; // 750KHz
#elif FREQ_SYS == 187500
    CLOCK_CFG = CLOCK_CFG & ~MASK_SYS_CK_SEL | 0x00; // 187.5KHz
#else
#warning FREQ_SYS invalid or not set
#endif

    SAFE_MOD = 0x00;
}

inline void system_enableGlobalInterupts(void) {

    if (EA == 0) {
        EA = 1;                     // only if already disabled, then enable global interrupts
    }
}

inline void system_disableGlobalInterupts(void) {

    if (EA == 1) {
        EA = 0;                     // only if already enabled, then disable global interrupts
    }
}

/*******************************************************************************
* Function Name  : system_mDelayuS(UNIT16 n)
* Description    : us delay function
* Input          : UNIT16 n
* Output         : None
* Return         : None
*******************************************************************************/
void system_mDelayuS(uint16_t n) {
#ifdef FREQ_SYS
#if FREQ_SYS <= 6000000
    n >>= 2;
#endif
#if FREQ_SYS <= 3000000
    n >>= 2;
#endif
#if FREQ_SYS <= 750000
    n >>= 4;
#endif
#endif
    while (n) {                     // total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
        ++SAFE_MOD;                 // 2 Fsys cycles, for higher Fsys, add operation here
#ifdef FREQ_SYS
#if FREQ_SYS >= 14000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 16000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 18000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 20000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 22000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 24000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 26000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 28000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 30000000
        ++SAFE_MOD;
#endif
#if FREQ_SYS >= 32000000
        ++SAFE_MOD;
#endif
#endif
        --n;
    }
}

/*******************************************************************************
* Function Name  : system_mDelayms(UNIT16 n)
* Description    : ms delay function
* Input          : UNIT16 n
* Output         : None
* Return         : None
*******************************************************************************/
void system_mDelaymS(uint16_t n) {

    while (n) {
#ifdef DELAY_MS_HW
        while ((TKEY_CTRL & bTKC_IF) == 0)
            ;
        while (TKEY_CTRL & bTKC_IF)
            ;
#else
        system_mDelayuS(1000);
#endif
        --n;
    }
}

/*******************************************************************************
* Function Name  : CH554WDTModeSelect(uint8_t mode)
* Description    : CH554 watchdog mode selection
* Input          : uint8_t mode
                   0  timer
                   1  watchDog
* Output         : None
* Return         : None
*******************************************************************************/
inline void system_WDTModeSelect(uint8_t mode) {
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                // Enter Safe Mode

    if (mode) {
        GLOBAL_CFG |= bWDOG_EN;     // Start watchdog reset
    } else {
        GLOBAL_CFG &= ~bWDOG_EN;    // Start watchdog only as a timer
    }
    
    SAFE_MOD = 0x00;                // Exit safe Mode
    WDOG_COUNT = 0;                 // Watchdog assignment initial value
}

/*******************************************************************************
* Function Name  : CH554WDTFeed(uint8_t tim)
* Description    : CH554 watchdog timer time setting
* Input          : uint8_t tim watchdog reset time setting

                   00H(6MHz)=2.8s
                   80H(6MHz)=1.4s
* Output         : None
* Return         : None
*******************************************************************************/
inline void system_WDTFeed(uint8_t timerTime) {
    WDOG_COUNT = timerTime;         // Watchdog counter assignment
}

/*******************************************************************************
* Function Name  : system_coldReboot(void)
* Description    : CH554 perform cold reboot via bSW_RESET
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
inline void system_coldReboot(void) {
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG |= bSW_RESET;
}