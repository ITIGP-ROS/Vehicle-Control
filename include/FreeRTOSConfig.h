/******************************************************************************
 * FreeRTOSConfig.h - kernel configuration for the TM4C123GH6PM (Cortex-M4F)
 *
 * Kernel: FreeRTOS-Kernel v11.1.0, MIT, port GCC/ARM_CM4F (hard-FP).
 * Vendored at B1a into lib/FreeRTOS-Kernel/ (see docs/RTOS_PORT_PHASE_B_PLAN.md
 * section 4.1).
 *
 * ⚠️ THIS FILE IS ONLY REACHED BY ENVS THAT DEFINE -DUSE_FREERTOS.
 * Today that is exactly one env: `rtos_bringup`. The production env
 * (lptm4c123gh6pm) and the 17 other bench envs do NOT define it, do NOT compile
 * the kernel, and are unaffected by anything in here. The startup vector table
 * is guarded by the same symbol - see startup/startup_tm4c123.c.
 *
 * Every value below is justified in docs/RTOS_TIMING_RESOURCE_BUDGET.md; where
 * a number came from that analysis the section is cited inline. Do not change
 * one here without changing it there - the budget is the reason, this is only
 * the encoding of it.
 *****************************************************************************/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*=============================================================================
 * 1. CLOCK AND TICK
 *
 * ⚠️ 16 MHz, NOT 80 MHz. The core runs from the 16 MHz crystal with the PLL
 * BYPASSED (C6-4: MOSC direct, confirmed on silicon; startup_tm4c123.c
 * SystemClock_Init, systick_cfg.h:20). Raising the clock is explicitly OUT OF
 * SCOPE for this port (RTOS_PORT_PHASE_B_PLAN section 4.3) because it would move
 * every PWM / QEI / timer / UART divisor at once and destroy the port's whole
 * verification method ("is the candump unchanged?").
 *
 * If configCPU_CLOCK_HZ is wrong the tick period is wrong by exactly that
 * ratio, silently - the heartbeat would simply print at the wrong rate. That is
 * the first thing to suspect if B1's 1 Hz measures as something else.
 *
 * 1000 Hz tick: the 20 ms control period and the 100 ms telemetry period become
 * EXACTLY 20 and 100 ticks, pdMS_TO_TICKS() is the identity function, and the
 * super-loop's wrap-safe millisecond arithmetic transfers verbatim when its body
 * becomes a task at B3. Cost is ~1.5-2 % of the CPU at 16 MHz (budget section 3.1),
 * which is the single largest consumer in the whole system - and accepted.
 *===========================================================================*/
#define configCPU_CLOCK_HZ                          ( 16000000UL )
#define configTICK_RATE_HZ                          ( ( TickType_t ) 1000 )

/* The port derives the SysTick reload from configSYSTICK_CLOCK_HZ, which
 * defaults to configCPU_CLOCK_HZ (port.c:110-111). Our SysTick runs from the
 * core clock, not PIOSC/4 (SYSTICK_USE_SYSTEM_CLOCK == 1 in systick_cfg.h), so
 * the two are equal and the default is correct. Left undefined deliberately. */

/*=============================================================================
 * 2. MEMORY MODEL - STATIC ONLY, NO HEAP
 *
 * DECIDED in RTOS_PORT_PHASE_B_PLAN section 4.2 / budget section 1. Three reasons:
 *  1. No runtime allocation failure is possible. Insufficient RAM stops being a
 *     pvPortMalloc() returning NULL somewhere in the vehicle and becomes a
 *     LINKER ERROR on the bench - linker/tm4c123.ld:62 already asserts that >=4K
 *     of RAM remains between .bss and the stack top, and static allocation is
 *     what puts the task stacks under that assertion.
 *  2. Determinism: no heap => no fragmentation => allocation contributes nothing
 *     to any WCET. This is the MISRA-C:2012 Dir 4.12 / AUTOSAR "no dynamic
 *     memory after init" principle.
 *  3. It costs nothing: every task is created at boot and none is ever deleted.
 *
 * heap_1.c IS vendored but is deliberately NOT in any build_src_filter. With
 * configSUPPORT_DYNAMIC_ALLOCATION 0 there is no pvPortMalloc in the image at
 * all, so a stray xTaskCreate() (dynamic) fails to LINK rather than failing to
 * allocate - which is the whole point.
 *===========================================================================*/
#define configSUPPORT_STATIC_ALLOCATION             1
#define configSUPPORT_DYNAMIC_ALLOCATION            0

/* v11.1.0 offers configKERNEL_PROVIDED_STATIC_MEMORY, which supplies the Idle
 * and Timer tasks' static memory so the application need not write
 * vApplicationGetIdleTaskMemory() / vApplicationGetTimerTaskMemory() by hand
 * (tasks.c:8575-8637). It was tried first and REJECTED - see below.
 *
 * ⚠️ FINDING (B1, measured against this kernel): the kernel-provided
 * vApplicationGetTimerTaskMemory() at tasks.c:8616-8637 is gated ONLY on
 * (configSUPPORT_STATIC_ALLOCATION && configKERNEL_PROVIDED_STATIC_MEMORY &&
 * !MPU) - it is NOT gated on configUSE_TIMERS. So with timers disabled it still
 * fails to compile without configTIMER_TASK_STACK_DEPTH, and once that is
 * defined it instantiates
 *      static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];
 * inside a function that is never called.
 *
 * On a normal project that array would be stripped. NOT HERE: per CLAUDE.md,
 * --gc-sections is NOT actually applied to this build (PlatformIO silently
 * ignores the `link_flags` key), so unreferenced statics still occupy .bss.
 * Kernel-provided memory would therefore cost ~512 bytes of permanently dead
 * RAM to support a timer task we deliberately do not have.
 *
 * DECISION: configKERNEL_PROVIDED_STATIC_MEMORY = 0, and the application
 * provides vApplicationGetIdleTaskMemory() itself (test/rtos_bringup/main.c for
 * B1; app.c from B11). Only the IDLE callback is needed - the timer callback's
 * sole caller is timers.c, which is compiled out entirely by configUSE_TIMERS 0.
 * One hand-written function, zero wasted RAM. */
#define configKERNEL_PROVIDED_STATIC_MEMORY         0

/*=============================================================================
 * 3. SCHEDULER
 *
 * configMAX_PRIORITIES = 11 => usable priorities 0 (idle) .. 10.
 *
 * ⚠️ THE LADDER IS NOT DEFINED HERE. It lives in ONE place, include/app_priorities.h,
 * with a justification per level. Raised from 6 to 10 on 2026-08-06 when the ladder
 * was settled: 6 levels could not give every interfering pair a DISTINCT level, and
 * sharing one is what produced the BUDGET-5 clash (tBattery parked on tVelocity's
 * slot). Cost of the extra 4 levels is ~80 bytes of ready-list array.
 *
 * Summary (see app_priorities.h for why each sits where it does):
 *     10  tSafety      9  tVelocity   8  tRosRx     7  tBattery
 *      6  tCanTx       5  tBusHealth  4  tRosTx     3  tClusterTx
 *      2  tOdo         1  tHeartbeat  0  idle
 *
 * ⚠️ This is a CRITICALITY-monotonic assignment, NOT a rate-monotonic one -
 * tSafety has a 20 ms period yet outranks tRosTx at 10 ms. That is correct and
 * intentional, and it is why the budget uses response-time analysis (the exact
 * test, which assumes nothing about how priorities were assigned) rather than
 * the Liu & Layland utilisation bound as its proof. See budget section 3.2.
 *
 * B1 uses exactly one task at priority 1, but the ceiling is set to its final
 * value now so no later step has to touch this file for it.
 *===========================================================================*/
#define configUSE_PREEMPTION                        1
#define configMAX_PRIORITIES                        ( 11 )
#define configUSE_TIME_SLICING                      1
#define configIDLE_SHOULD_YIELD                     1

/* Cortex-M4 has CLZ, so the port can select the highest ready priority in
 * constant time instead of scanning the ready lists. Requires
 * configMAX_PRIORITIES <= 32 (portmacro.h:148-162 #errors otherwise); ours is 11. */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1

/* configMINIMAL_STACK_SIZE is in WORDS, not bytes: 128 words = 512 bytes. It
 * sizes the Idle task. Per-task stacks are sized individually in budget
 * section 7.1 and refined from uxTaskGetStackHighWaterMark measurements. */
#define configMINIMAL_STACK_SIZE                    ( ( configSTACK_DEPTH_TYPE ) 128 )

#define configMAX_TASK_NAME_LEN                     ( 12 )

/* 32-bit TickType_t. At 1 kHz it wraps every 49.7 days; every tick comparison
 * in this codebase already uses wrap-safe unsigned subtraction, which the
 * super-loop has done since day one.
 * ⚠️ configUSE_16_BIT_TICKS is the DEPRECATED spelling and FreeRTOS.h:71
 * #errors if BOTH are defined. Use only the new one. */
#define configTICK_TYPE_WIDTH_IN_BITS               TICK_TYPE_WIDTH_32_BITS

/*=============================================================================
 * 4. FPU
 *
 * configUSE_TASK_FPU_SUPPORT 1 (the default, set explicitly) = EVERY task gets
 * a lazy-stacked FPU context. Worst case +132 bytes of stack per task, already
 * included in the budget's stack table.
 *
 * ⚠️ DO NOT set this to 0 and sprinkle portTASK_USES_FLOATING_POINT() to save
 * RAM. Seven of the ten planned tasks touch float (PID, SoC LUT, IR
 * compensation, unit conversions, servo angle maths), and ONE missed task means
 * silent floating-point register corruption - a bug that presents as
 * intermittent wrong numbers with no fault, which is the most expensive kind to
 * chase. The RAM margin (~10 KB net, budget section 7.1) does not require the risk.
 *
 * Note the FPU is enabled in ResetISR (CPACR CP10/CP11) BEFORE anything else,
 * and lazy stacking (FPCCR.ASPEN/LSPEN) is left at its reset default.
 *===========================================================================*/
#define configUSE_TASK_FPU_SUPPORT                  1

/*=============================================================================
 * 5. INTERRUPT PRIORITIES  ****  READ THIS BEFORE TOUCHING AN NVIC PRIORITY ****
 *
 * The TM4C123 implements 3 NVIC priority bits, held in the TOP of each 8-bit
 * priority field (bits 7:5). So there are 8 levels, 0 = most urgent.
 *
 *   configKERNEL_INTERRUPT_PRIORITY     = 7 << 5 = 0xE0  (PendSV + SysTick;
 *                                                         lowest, as required)
 *   configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 5 = 0xA0
 *
 * ==> NVIC priorities 0..4 are NEVER masked by the kernel (BASEPRI does not
 *     reach them) and therefore may NOT call ANY FreeRTOS "...FromISR" API.
 * ==> NVIC priorities 5..7 ARE maskable and MAY call the FromISR API.
 *
 * 🔴 BUDGET-2 (docs/RTOS_TIMING_RESOURCE_BUDGET.md section 6.2): CAN0 is
 * currently at NVIC priority 1 (can_cfg.h:185), i.e. in the "never masked, may
 * not call the kernel" band - while step B8's entire design is
 * xQueueSendFromISR() from the CAN0 ISR. Left as-is, FreeRTOS's
 * vPortValidateInterruptPriority() would configASSERT on the FIRST received
 * frame. B8 MUST lower CAN_CFG_NVIC_PRIORITY from 1 to 5. The cost is ~4 us of
 * masked latency instead of 0.75 us, which is 1.8 % of one 222 us CAN frame -
 * irrelevant.
 *
 * The ceiling is set to its final value HERE, in B1, precisely so that B8
 * inherits a correct constraint instead of discovering one.
 *
 * The general rule, worth remembering: an ISR may be FAST or it may TALK TO THE
 * KERNEL - not both. Anything genuinely latency-critical can live at 0..4 and
 * keep the 0.75 us hardware latency, provided it communicates only through
 * plain volatile variables. We have no such interrupt today.
 *===========================================================================*/
#define configPRIO_BITS                             3

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     7
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* The port validates, on every ...FromISR call, that the calling interrupt's
 * priority is legal (port.c:860-890). Keep this ON for the whole port - it is
 * the mechanism that would catch BUDGET-2 immediately rather than as a rare
 * corruption. It can be reconsidered only once every ISR priority is settled. */
#define configASSERT_DEFINED                        1

/*=============================================================================
 * 6. FEATURES - lean by default; each is turned on by the step that needs it
 *===========================================================================*/

/* Software timers OFF. B1 does not use them, and the failsafe deliberately does
 * NOT use them either: a timer callback runs in the timer daemon task, behind
 * every other timer's callback, on one shared priority - rejected in
 * RTOS_PORT_PHASE_B_PLAN section 2.1 in favour of a dedicated tSafety task.
 * timers.c is still compiled (it validates the vendoring) but the ENTIRE file
 * is inside #if (configUSE_TIMERS == 1) at timers.c:56..1340, so it contributes
 * ZERO bytes. With it off, no timer-task memory callback is required. */
#define configUSE_TIMERS                            0

/* Needed from B6 (CAN TX queue) and B10 (battery status mutex). Enabling them
 * now costs only what queue.c already links; queue.c backs all three.
 * FreeRTOS mutexes carry PRIORITY INHERITANCE by default - which is why the
 * battery status uses a mutex and the CAN TX-complete signal uses a BINARY
 * SEMAPHORE (a signal, not a lock: it wants no inheritance). */
#define configUSE_MUTEXES                           1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_RECURSIVE_MUTEXES                 0
#define configQUEUE_REGISTRY_SIZE                   0

#define configUSE_TASK_NOTIFICATIONS                1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       1

#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0
#define configUSE_MALLOC_FAILED_HOOK                0   /* no heap exists */
#define configUSE_TICKLESS_IDLE                     0
#define configUSE_TRACE_FACILITY                    0
#define configUSE_STATS_FORMATTING_FUNCTIONS        0
#define configUSE_APPLICATION_TASK_TAG              0
#define configUSE_NEWLIB_REENTRANT                  0
#define configENABLE_BACKWARD_COMPATIBILITY         0

/*=============================================================================
 * 7. DIAGNOSTICS - loud during bring-up, by design
 *
 * configCHECK_FOR_STACK_OVERFLOW 2 = the pattern-checking method, which catches
 * overflows that method 1 (stack-pointer bounds only) misses. It costs a few
 * cycles per context switch and is to be kept at 2 for the WHOLE port; it may
 * only be reduced once every task's high-water mark has been stable across
 * several representative missions (budget section 8.2).
 *
 * INCLUDE_uxTaskGetStackHighWaterMark is not optional here - it is the
 * measurement instrument for the budget's stack table. Every [E] stack estimate
 * becomes an [M] through this function.
 *===========================================================================*/
#define configCHECK_FOR_STACK_OVERFLOW              2
#define configRECORD_STACK_HIGH_ADDRESS             1

/* Run-time stats: OFF for B1 (this env does not init timer0). Turned on later
 * with portGET_RUN_TIME_COUNTER_VALUE() = Timer0_GetTicks() - already a
 * free-running 16 MHz counter read by a single tear-free LDR, so CPU-load
 * measurement needs no new hardware. Budget section 8.3. */
#define configGENERATE_RUN_TIME_STATS               0

/*=============================================================================
 * 8. INCLUDE_ - the API actually used
 *===========================================================================*/
#define INCLUDE_vTaskPrioritySet                    0
#define INCLUDE_uxTaskPriorityGet                   0
#define INCLUDE_vTaskDelete                         0   /* nothing is ever deleted */
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskDelay                          1
/* ⚠️ INCLUDE_vTaskDelayUntil is the DEPRECATED spelling and FreeRTOS.h:223
 * #errors if both are defined. xTaskDelayUntil is the one to use anyway: it
 * RETURNS whether it actually delayed, which is the signal that a task overran
 * its period - directly relevant to tVelocity's early-firing hazard
 * (budget section 4.2). */
#define INCLUDE_xTaskDelayUntil                     1
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_xTaskGetIdleTaskHandle              0
#define INCLUDE_eTaskGetState                       0
#define INCLUDE_xTimerPendFunctionCall              0
#define INCLUDE_xSemaphoreGetMutexHolder            0
#define INCLUDE_xTaskAbortDelay                     0

/*=============================================================================
 * 9. configASSERT - loud halt, never silent
 *
 * A failed assertion here is almost always a CONFIGURATION error (an illegal
 * ISR priority, a FromISR call from the wrong context, a stack overflow), and
 * those are exactly the failures that are otherwise invisible. So: disable
 * interrupts and spin, leaving the file/line in registers for the debugger, and
 * leave the machine in the faulted state rather than limping on.
 *
 * The caller-visible signature is what the kernel expects: configASSERT(x).
 * vAssertCalled() lives in the application (test/rtos_bringup/main.c for B1,
 * app.c from B11) so it can also print - printing is not safe to do from this
 * header, which is included by the kernel itself.
 *===========================================================================*/
#ifndef __ASSEMBLER__
    extern void vAssertCalled( const char *pcFile, unsigned long ulLine );
    #define configASSERT( x ) \
        if( ( x ) == 0 ) { vAssertCalled( __FILE__, __LINE__ ); }
#endif

/*=============================================================================
 * 10. VECTOR ROUTING - DIRECT, done in the startup file, NOT here
 *
 * ⚠️ Note what is ABSENT from this file, deliberately:
 *
 *     #define vPortSVCHandler     SVC_Handler        <-- NOT USED
 *     #define xPortPendSVHandler  PendSV_Handler     <-- NOT USED
 *     #define xPortSysTickHandler SysTick_Handler    <-- NOT USED
 *
 * That "indirect routing" idiom is the one route that BREAKS on this codebase:
 * our vector table already names SysTick_Handler, and src/systick.c:477 already
 * DEFINES it, so the third #define would rename the kernel's handler onto an
 * existing symbol and produce a genuine duplicate-symbol link error.
 *
 * Instead we use DIRECT ROUTING: startup/startup_tm4c123.c places
 * vPortSVCHandler / xPortPendSVHandler / xPortSysTickHandler straight into
 * vector slots 11 / 14 / 15, guarded by #ifdef USE_FREERTOS. This is also the
 * routing the port itself validates at startup (port.c:343-344) and the one it
 * recommends (port.c:316-325).
 *
 * ⚠️ The SysTick slot is the dangerous one and it is UNCHECKED BY DESIGN
 * (port.c:331-334 - an application may drive the tick from another timer via
 * the weak vPortSetupTimerInterrupt()). If our SysTick_Handler were left in
 * slot 15, xPortStartScheduler() would reprogram SysTick anyway
 * (port.c:826-830), our handler would run, xTaskIncrementTick() would never be
 * called, and every vTaskDelay would block forever - with NO assert and NO
 * fault. The system would simply stop. That silent hang is the single failure
 * mode B1 exists to defeat.
 *===========================================================================*/

#endif /* FREERTOS_CONFIG_H */
