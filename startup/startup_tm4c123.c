//*****************************************************************************
//
// Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2011-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//*****************************************************************************

#include <stdint.h>

//*****************************************************************************
//   /* 2. Initialize PWM driver */
    //PWM_Init(MOTOR_A);  /* Right motor - PB6 */
  //  PWM_Init(MOTOR_B);  /* Left motor - PB4 */
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);
extern void UART1_Handler(void);
extern void CAN0_Handler(void);

//*****************************************************************************
//
// EXCEPTION SLOTS 11 / 14 / 15 - SVCall, PendSV, SysTick.
//
// Under FreeRTOS the KERNEL owns all three. The port does NOT define
// SysTick_Handler/PendSV_Handler/SVC_Handler - it defines vPortSVCHandler,
// xPortPendSVHandler and xPortSysTickHandler and leaves the routing to the
// application (lib/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c:138-140). We use
// DIRECT ROUTING: the three kernel symbols go straight into the table below.
//
// This is the routing the port RECOMMENDS (port.c:316-325) and the only one it
// can VALIDATE - xPortStartScheduler() checks slots 11 and 14 against the live
// VTOR (port.c:343-344), so a mistake in those two announces itself as a
// configASSERT at startup instead of misbehaving.
//
// The alternative "indirect" idiom - #define xPortSysTickHandler
// SysTick_Handler in FreeRTOSConfig.h - is DELIBERATELY NOT USED: it would
// rename the kernel's handler onto a symbol src/systick.c:477 already defines,
// giving a duplicate-symbol link error.
//
// ⚠️ SLOT 15 IS THE DANGEROUS ONE, and the port checks it deliberately NOT AT
// ALL (port.c:331-334, because an application may drive the tick from a
// different timer via the weak vPortSetupTimerInterrupt()). If our
// SysTick_Handler were left here under FreeRTOS, xPortStartScheduler() would
// reprogram SysTick anyway (port.c:826-830), OUR handler would run,
// xTaskIncrementTick() would never be called, and every vTaskDelay() would block
// forever - with no assert and no fault. The system would just stop. That silent
// hang is the specific failure this guard exists to prevent.
//
// EVERY NON-RTOS ENV IS UNAFFECTED. USE_FREERTOS is defined by exactly one
// PlatformIO env (rtos_bringup); the production env and the 17 other bench
// harnesses keep their own SysTick_Handler and IntDefaultHandler, byte for byte.
//
//*****************************************************************************
#ifdef USE_FREERTOS
extern void vPortSVCHandler(void);        // -> vector slot 11 (SVCall)
extern void xPortPendSVHandler(void);     // -> vector slot 14 (PendSV)
extern void xPortSysTickHandler(void);    // -> vector slot 15 (SysTick)
#else
extern void SysTick_Handler(void);        // src/systick.c - our 1 ms timebase
#endif
//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
//extern void _c_int00(void);
extern int main(void);
extern uint32_t _etext;
extern uint32_t _sdata, _edata;
extern uint32_t _sbss, _ebss;
//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern uint32_t __STACK_TOP;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
// To be added by user

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((uint32_t)&__STACK_TOP),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
#ifdef USE_FREERTOS
    vPortSVCHandler,                        // SVCall handler   (FreeRTOS)
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    xPortPendSVHandler,                     // The PendSV handler  (FreeRTOS)
    xPortSysTickHandler,                    // The SysTick handler (FreeRTOS)
#else
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
    SysTick_Handler,                      // The SysTick handler
#endif
    IntDefaultHandler,                      // GPIO Port A
    IntDefaultHandler,                      // GPIO Port B
    IntDefaultHandler,                      // GPIO Port C
    IntDefaultHandler,                      // GPIO Port D
    IntDefaultHandler,                      // GPIO Port E
    IntDefaultHandler,                      // UART0 Rx and Tx
    UART1_Handler,                      // UART1 Rx and Tx
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // I2C0 Master and Slave
    IntDefaultHandler,                      // PWM Fault
    IntDefaultHandler,                      // PWM Generator 0
    IntDefaultHandler,                      // PWM Generator 1
    IntDefaultHandler,                      // PWM Generator 2
    IntDefaultHandler,                      // Quadrature Encoder 0
    IntDefaultHandler,                      // ADC Sequence 0
    IntDefaultHandler,                      // ADC Sequence 1
    IntDefaultHandler,                      // ADC Sequence 2
    IntDefaultHandler,                      // ADC Sequence 3
    IntDefaultHandler,                      // Watchdog timer
    IntDefaultHandler,                      // Timer 0 subtimer A
    IntDefaultHandler,                      // Timer 0 subtimer B
    IntDefaultHandler,                      // Timer 1 subtimer A
    IntDefaultHandler,                      // Timer 1 subtimer B
    IntDefaultHandler,                      // Timer 2 subtimer A
    IntDefaultHandler,                      // Timer 2 subtimer B
    IntDefaultHandler,                      // Analog Comparator 0
    IntDefaultHandler,                      // Analog Comparator 1
    IntDefaultHandler,                      // Analog Comparator 2
    IntDefaultHandler,                      // System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // FLASH Control
    IntDefaultHandler,                      // GPIO Port F
    IntDefaultHandler,                      // GPIO Port G
    IntDefaultHandler,                      // GPIO Port H
    IntDefaultHandler,                      // UART2 Rx and Tx
    IntDefaultHandler,                      // SSI1 Rx and Tx
    IntDefaultHandler,                      // Timer 3 subtimer A
    IntDefaultHandler,                      // Timer 3 subtimer B
    IntDefaultHandler,                      // I2C1 Master and Slave
    IntDefaultHandler,                      // Quadrature Encoder 1
    CAN0_Handler,                           // CAN0
    IntDefaultHandler,                      // CAN1
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // Hibernate
    IntDefaultHandler,                      // USB0
    IntDefaultHandler,                      // PWM Generator 3
    IntDefaultHandler,                      // uDMA Software Transfer
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // ADC1 Sequence 0
    IntDefaultHandler,                      // ADC1 Sequence 1
    IntDefaultHandler,                      // ADC1 Sequence 2
    IntDefaultHandler,                      // ADC1 Sequence 3
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port J
    IntDefaultHandler,                      // GPIO Port K
    IntDefaultHandler,                      // GPIO Port L
    IntDefaultHandler,                      // SSI2 Rx and Tx
    IntDefaultHandler,                      // SSI3 Rx and Tx
    IntDefaultHandler,                      // UART3 Rx and Tx
    IntDefaultHandler,                      // UART4 Rx and Tx
    IntDefaultHandler,                      // UART5 Rx and Tx
    IntDefaultHandler,                      // UART6 Rx and Tx
    IntDefaultHandler,                      // UART7 Rx and Tx
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C2 Master and Slave
    IntDefaultHandler,                      // I2C3 Master and Slave
    IntDefaultHandler,                      // Timer 4 subtimer A
    IntDefaultHandler,                      // Timer 4 subtimer B
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // Timer 5 subtimer A
    IntDefaultHandler,                      // Timer 5 subtimer B
    IntDefaultHandler,                      // Wide Timer 0 subtimer A
    IntDefaultHandler,                      // Wide Timer 0 subtimer B
    IntDefaultHandler,                      // Wide Timer 1 subtimer A
    IntDefaultHandler,                      // Wide Timer 1 subtimer B
    IntDefaultHandler,                      // Wide Timer 2 subtimer A
    IntDefaultHandler,                      // Wide Timer 2 subtimer B
    IntDefaultHandler,                      // Wide Timer 3 subtimer A
    IntDefaultHandler,                      // Wide Timer 3 subtimer B
    IntDefaultHandler,                      // Wide Timer 4 subtimer A
    IntDefaultHandler,                      // Wide Timer 4 subtimer B
    IntDefaultHandler,                      // Wide Timer 5 subtimer A
    IntDefaultHandler,                      // Wide Timer 5 subtimer B
    IntDefaultHandler,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C4 Master and Slave
    IntDefaultHandler,                      // I2C5 Master and Slave
    IntDefaultHandler,                      // GPIO Port M
    IntDefaultHandler,                      // GPIO Port N
    IntDefaultHandler,                      // Quadrature Encoder 2
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port P (Summary or P0)
    IntDefaultHandler,                      // GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    IntDefaultHandler,                      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
    IntDefaultHandler,                      // PWM 1 Generator 0
    IntDefaultHandler,                      // PWM 1 Generator 1
    IntDefaultHandler,                      // PWM 1 Generator 2
    IntDefaultHandler,                      // PWM 1 Generator 3
    IntDefaultHandler                       // PWM 1 Fault
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
//*****************************************************************************
//
// Coprocessor Access Control Register - enables the Cortex-M4F FPU.
// CP10/CP11 = bits 23:20; 0xF there is "full access" for both.
//
//*****************************************************************************
#define CPACR_REG   (*((volatile uint32_t *)0xE000ED88))
#define CPACR_CP10_CP11_FULL_ACCESS   (0xFUL << 20)

//*****************************************************************************
//
// System clock: run the core from the 16 MHz main crystal (MOSC), not PIOSC.
// C6-4 — see docs/can/REVIEW_06_can_ringbuffer.md.
//
//*****************************************************************************
#define SYSCTL_RIS_REG      (*((volatile uint32_t *)0x400FE050))
#define SYSCTL_MISC_REG     (*((volatile uint32_t *)0x400FE058))
#define SYSCTL_RCC_REG      (*((volatile uint32_t *)0x400FE060))

#define RCC_MOSCDIS         (1UL << 0)      /* 1 = main oscillator powered down */
#define RCC_BYPASS          (1UL << 11)     /* 1 = bypass the PLL               */
#define RCC_USESYSDIV       (1UL << 22)     /* 0 = no system divider            */
#define RCC_OSCSRC_MASK     (3UL << 4)
#define RCC_OSCSRC_MOSC     (0UL << 4)      /* 0 = MOSC, 1 = PIOSC (reset)      */
#define RIS_MOSCPUPRIS      (1UL << 8)      /* MOSC powered up and stable       */

/* Bounded wait for crystal start-up. A healthy 16 MHz crystal settles in well
 * under a millisecond; this cap is ~an order of magnitude beyond that at the
 * 16 MHz PIOSC we are still running on when it executes. */
#define MOSC_STARTUP_SPINS  (200000UL)

/**
 * Switch the core clock from the reset-default PIOSC to the 16 MHz crystal.
 *
 * WHY: at reset the part runs on the internal PIOSC, specified at +/-1% and
 * measured ~1.1% fast on this board, while every *_cfg.h (timer_pwm, systick,
 * qei, uart, can) hardcodes exactly 16 000 000 Hz. That made the "20 ms" QEI
 * velocity window really ~20.2 ms, biasing measured wheel velocity ~1% low into
 * the PID. Running from the crystal makes the hardcoded constant TRUE rather
 * than approximate - so no *_cfg.h value changes.
 *
 * The board was verified to have a working 16 MHz crystal before this was
 * written: powering MOSC over the debug probe set MOSCPUPRIS with no MOFRIS,
 * and switching OSCSRC to it moved the measured CAN TX rates from
 * 98.9/99.0/9.9 Hz to 100.02/99.99/10.00 Hz.
 *
 * SAFETY: the crystal-ready wait is BOUNDED. If MOSC never stabilises (crystal
 * absent or damaged on some other board), we power it back down and stay on
 * PIOSC - timing is then +/-1% but the board still boots. A missing crystal
 * must never brick the reset path.
 *
 * PLL is not used: BYPASS stays 1 and USESYSDIV stays 0, so the core clock is
 * the oscillator directly. RCC2 is left alone (USERCC2 = 0 at reset, so RCC
 * governs).
 */
static void SystemClock_Init(void)
{
    uint32_t rcc;
    uint32_t spins;

    /* Clear any stale MOSC power-up status before waiting on it. */
    SYSCTL_MISC_REG = RIS_MOSCPUPRIS;

    /* Power up MOSC, keep the PLL bypassed and the system divider off. Still
     * running on PIOSC at this point.
     *
     * NOTE: RCC.XTAL is deliberately NOT written. That field only tells the PLL
     * its input frequency, and BYPASS=1 means the PLL is never used here - the
     * core is driven by the oscillator directly. (An earlier version of this
     * function did set XTAL=0x15/16 MHz; the field was observed to still read
     * its 0x0B reset value afterwards, which is unexplained but has no effect
     * with the PLL bypassed. Rather than keep a write that claims to configure
     * something it does not, it is left out. If a PLL is ever introduced, XTAL
     * MUST be set correctly first.) */
    rcc  = SYSCTL_RCC_REG;
    rcc |=  RCC_BYPASS;
    rcc &= ~RCC_USESYSDIV;
    rcc &= ~RCC_MOSCDIS;
    SYSCTL_RCC_REG = rcc;

    /* Bounded wait for the crystal to stabilise. */
    spins = MOSC_STARTUP_SPINS;
    while (((SYSCTL_RIS_REG & RIS_MOSCPUPRIS) == 0UL) && (spins > 0UL))
    {
        spins--;
    }

    if (spins == 0UL)
    {
        /* No usable crystal - power MOSC back down and stay on PIOSC. */
        SYSCTL_RCC_REG |= RCC_MOSCDIS;
        return;
    }

    /* Crystal is good: switch the core onto it. */
    SYSCTL_RCC_REG = (SYSCTL_RCC_REG & ~RCC_OSCSRC_MASK) | RCC_OSCSRC_MOSC;
}

void ResetISR(void)
{
    /* Enable the FPU (T5-3) - FIRST, before ANY code that could touch a float.
     *
     * The build is compiled -mfpu=fpv4-sp-d16 -mfloat-abi=hard, so the compiler
     * is free to emit VFP instructions anywhere, including in the .data copy
     * below if it ever vectorises it. Executing a VFP instruction while CP10/
     * CP11 are disabled raises a UsageFault (NOCP) - so this write has to come
     * before everything else in the reset path.
     *
     * DSB then ISB per the ARM architecture requirement: the DSB ensures the
     * CPACR write has completed, the ISB flushes the pipeline so the very next
     * instruction is fetched with the FPU already enabled. Without them the
     * enable may not be visible to an immediately-following float instruction.
     *
     * Lazy stacking (FPCCR.ASPEN/LSPEN) is left at its reset default = enabled:
     * an exception only pays the FPU-frame cost if the handler actually uses
     * float. Production ISRs (SysTick_Handler, CAN0_Handler) are float-free, so
     * this costs them nothing. */
    CPACR_REG |= CPACR_CP10_CP11_FULL_ACCESS;
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");

    /* Core clock -> 16 MHz crystal (C6-4). Must run BEFORE main() and therefore
     * before every peripheral init that derives its timing from the system
     * clock (SysTick, PWM, timer_pwm, QEI, UART, CAN all do). Touches only
     * SYSCTL registers, so it is safe this early - before .data/.bss. */
    SystemClock_Init();

    /* Copy .data from FLASH to RAM */
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* Zero .bss */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    /* Jump to main */
    (void)main();

    while (1) { }
}


//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}