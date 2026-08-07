/******************************************************************************
 * app.h - Tier 4: APPLICATION / ORCHESTRATION.
 *
 * ARCHITECTURE_app_layer.md section 2 named this layer before any of it existed:
 *
 *     "app.c - creates all FreeRTOS tasks, sets priorities, owns the
 *      queues/mutexes, wires services<->comm. NO control math, NO register
 *      access. This is the ONLY module that knows both 'the services' and
 *      'the CAN link'."
 *
 * B11 is where that becomes true. Everything above the HAL now runs as a task,
 * and this module is the single place that knows the task SET - what exists, at
 * what priority, with what stack. main.c is reduced to hardware init plus one
 * call to App_Start().
 *
 * ⚠️ THE INVARIANT TO KEEP: app.c is pure WIRING. It creates tasks and hands
 * them their buffers. It contains no control math, no protocol packing and no
 * register access - if logic ever starts accumulating here, it belongs in a
 * service instead.
 *****************************************************************************/

#ifndef APP_H_
#define APP_H_

#include "Platform_Types.h"
#include "Std_Types.h"

/**
 * @brief  Create the full task set and start the scheduler. NEVER RETURNS.
 *
 * @param  batteryOk  the init block's verdict on the A3 battery chain. FALSE
 *                    suppresses tBattery and the 0x210 alternation, mirroring
 *                    the old super-loop's "skip both battery slots" containment.
 * @param  imuOk      whether the I2C bus came up, so tImu has something to talk
 *                    to. ⚠️ This is NOT "the MPU6050 answered" - deliberately.
 *                    imu_service retries an absent sensor on its own backoff and
 *                    publishes nothing until a real sample exists, so a sensor
 *                    that is missing or late at boot must NOT suppress the task
 *                    the way a dead battery chain does. Only a dead BUS does,
 *                    because that leaves nothing to retry.
 *
 * @note   Call ONCE, LAST, from main() - after every hardware Init has run and
 *         in particular after Can_Init(), since the CAN TX queue creates the
 *         ISR's TX-complete semaphore.
 * @note   Returns only if vTaskStartScheduler() itself fails, which with static
 *         allocation can only mean the Idle task could not be created. There is
 *         no heap to exhaust. main() halts loudly on return.
 * @note   FreeRTOS builds only. In a non-RTOS build this is a no-op returning
 *         E_NOT_OK, and main() runs the legacy super-loop instead.
 */
Std_ReturnType App_Start(boolean batteryOk, boolean imuOk);

#endif /* APP_H_ */
