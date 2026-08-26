/******************************************************************************
 * SYSTEM MAIN — hand-rolled cooperative super-loop (full-stack integration)
 *
 * First main that runs ALL layers together: MCAL (CAN/PWM/QEI/ADC/WTIMER0) +
 * HAL (Motor/Encoder/Servo/ServoFb) + CONTROL (velocity/steering) + COMM
 * (jetson_comm RX/TX + cluster_comm 0x200). Jetson is the command source over
 * CAN; this board closes the velocity loop and streams feedback back.
 *
 *   RX (Jetson -> us), applied by JetsonComm_Poll() every iteration:
 *     0x100 VelocityCommand -> VelocityControl_SetSetpoint (RPM)
 *     0x120 SteeringCommand -> SteeringControl_SetAngle (rad, +left)
 *   TX (us -> Jetson/cluster), one frame per 1 ms slot, never colliding:
 *     0x110 VelocityFeedback @100 Hz (now%10==0)
 *     0x130 SteeringFeedback @100 Hz (now%10==5, 5 ms after 0x110)
 *     0x200 VehicleStatus    @ 10 Hz (now%100==9,  own slot)
 *     0x210 BatteryStatus    @ 10 Hz (now%100==47, own slot)
 *   CONTROL: VelocityControl_Update() exactly once per 20 ms (QEI window / Ts).
 *   TELEMETRY: BatteryService_Update() @10 Hz (now%100==43) - I2C read, NO TX.
 *
 * WHY a hand-rolled super-loop (not scheduler.c): scheduler.c is unproven dead
 * code and cannot guarantee <=1 Can_Transmit per tick (SCHEDULER_AUDIT S1/S3).
 * This loop guarantees it BY CONSTRUCTION: the TX slots (0,5 mod 10; 9 and 47
 * mod 100) are mutually exclusive, and a per-millisecond edge guard fires each
 * slot at most once, so at most one Can_Transmit happens in any millisecond.
 *
 *   PROOF for the two A3 slots: 43 mod 10 = 3 and 47 mod 10 = 7, so neither can
 *   ever coincide with the 0-or-5 mod 10 slots; 43 != 9 and 47 != 9, so neither
 *   collides with 0x200; and 43 != 47. Slot 43 issues NO Can_Transmit at all -
 *   it is the I2C read - so its ~480 us has the millisecond to itself.
 *
 * ---------------------------------------------------------------------------
 *  SAFETY COVERAGE. Corrected 2026-08-27 (CC_PROMPT_117) - this block used to
 *  say "there is NO RX command-loss failsafe/watchdog (deferred pending WDT
 *  MCAL)". That was written before B10 and is now WRONG IN BOTH DIRECTIONS:
 *  the RX failsafe DOES exist, and the watchdog is not merely deferred, it is
 *  absent by decision. The accurate statement, to be used verbatim in any
 *  writeup:
 *
 *      No hardware watchdog is enabled. Command loss is covered in software by
 *      tSafety, measured at 155-177 ms over three runs. A hung task is NOT
 *      covered.
 *
 *  Concretely:
 *    - COVERED: the Jetson stops sending -> tSafety (prio 11, 10 ms period)
 *      stops the drive and latches vc_inhibit. Measured 155-177 ms, 3/3
 *      (CC_PROMPT_110), and re-exercised on the deep path 2026-08-27. Budget is
 *      CMD_TIMEOUT_MS + one 33 ms control cycle = 183 ms, NOT a bare 150.
 *      ⚠️ The TRIP path costs 522 us of execution against tSafety's 41 us
 *      steady body, and consumes 32 more words of stack than an idle run - so
 *      it is the branch that sizes this task, and it is NON-RECURRENT (once per
 *      episode, not once per 10 ms period).
 *    - NOT COVERED: a hung or deadlocked task, tSafety included. src/wdt.c is
 *      compiled into the image but has ZERO callers (verified by grep and by
 *      objdump: the only branch into WDT code is WDT_ClearInterrupt from the
 *      driver's own ISRs, which can never fire because the peripheral is never
 *      started). It is dead code, not a safety mechanism.
 *
 *  Enabling the WDT is a deliberate, TESTED change - never a pre-demo switch-on.
 *  An untested feed path reboots the ECU mid-drive.
 *  Steering holds its last angle in every case (open loop, no failsafe centre).
 *  First power-on: wheels OFF the ground.
 * ---------------------------------------------------------------------------
 *
 * INIT ORDER (SYSTEM_INTEGRATION_AUDIT.md §2, derived from header prereqs):
 *   Port -> PWM -> SysTick -> UART -> Motor -> Encoder -> Can
 *        -> VelocityControl -> SteeringControl -> ClusterComm -> JetsonComm
 *        -> Timer0 -> I2C -> BatteryService.
 *
 * ⚠️ Timer0_FreeRunning_Init() MUST precede I2C_Init(). Every I2C command is
 * bounded by the free-running TIMER0 tick, NOT SysTick; with TIMER0 stopped
 * GPTMTAR is static, the cap never elapses, and the FIRST I2C read HANGS
 * FOREVER - a bus fault, not a graceful degrade. This is the single hardest
 * requirement in A3 (REVIEW 25 §0).
 *
 * ⚠️ ESSENTIAL vs AUXILIARY init, a deliberate distinction. The drive stack
 * (Port/PWM/SysTick/UART/Motor/Encoder) halts on failure - a vehicle that
 * cannot read its encoders must not run. The battery telemetry chain
 * (Timer0/I2C/BatteryService) does NOT halt: it is a fuel gauge, and turning a
 * missing telemetry sensor into a total vehicle failure would be a far worse
 * outcome than driving without a battery reading. On failure the loop simply
 * skips the two battery slots and says so on the console; battery_service is
 * built to degrade (last-good sample, flags, IsHealthy false).
 * SteeringControl_Init OWNS the steering HAL (it calls Servo_Init -> TimerPWM_Init
 * and ServoFb_Init -> Adc_Init internally, then self-centers); this main must NOT
 * call Servo_Init / TimerPWM_Init / Adc_Init directly (double-init hazard).
 * Prior main (distance_control bench test) preserved as main.c.orig.<timestamp>.
 ******************************************************************************/

#include "Platform_Types.h"
#include "PORT.h"
#include "PWM.h"
#include "systick.h"
#include "uart.h"
#include "Motor.h"
#include "encoder.h"
#include "can.h"
#include "velocity_control.h"
#include "steering_control.h"
#include "cluster_comm.h"
#include "node_ping.h"
#include "odo.h"
#include "eeprom.h"
#include "jetson_comm.h"
#include "can_tx_queue.h"    /* B6: the single CAN transmit path (Tier 3)      */
#include "app.h"             /* B11: Tier-4 orchestration - App_Start()        */
#include "app_cfg.h"         /* B11: shared timing/sizing (was defined here)   */
#include "timer0.h"          /* free-running us tick - the I2C timeout basis    */
#include "i2c.h"             /* I2C0 (PB2/PB3) INA226 + I2C1 (PA6/PA7) MPU6050  */
#include "battery_service.h" /* hybrid coulomb/voltage fuel gauge over ina226   */
#include "imu_service.h"     /* B13: MPU6050 service - 0x150/0x160 source       */
#include "mpu6050.h"         /* MPU6050_GetLastWhoAmI() for the boot diagnostic */
#include "ina226.h"          /* DIAGNOSTIC ONLY: Ina226_GetLastI2cError() for a
                              * decoded boot message. BatteryService owns the
                              * driver's lifecycle - main never calls
                              * Ina226_Init/ReadAll itself. */


/*----------------------------------------------------------------------------
 * The loop's millisecond timebase - the ONE substantive edit inside the body.
 *
 * Both sources are a free-running uint32 of MILLISECONDS, so every
 * `(now - last) >= period` wrap-safe subtraction and every `now % 10` /
 * `now % 100` slot residue below is numerically IDENTICAL either way. That
 * equivalence is not a happy accident - it is exactly why configTICK_RATE_HZ
 * was chosen as 1000 (RTOS_PORT_PHASE_B_PLAN.md 4.3).
 *
 * ⚠️ Under USE_FREERTOS our SysTick driver is NOT initialised at all (see the
 * guard at init step 3). FreeRTOS owns the SysTick hardware from
 * vTaskStartScheduler() onward, and letting SysTick_Init() program STRELOAD/
 * STCTRL as well would leave the two fighting over the same registers with
 * whoever ran last winning. Verified safe by grep: NO production module calls
 * any SysTick_* function - only this file and the uncalled, dead scheduler.c.
 * systick.c still compiles into the image and still serves the 18 non-RTOS
 * bench envs completely unchanged.
 *--------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * REMOVED HERE: the temporary LEFT-actuator RAM-buffer diagnostic (DiagBuf /
 * Diag_DumpLeftBuffer, ~90 lines).
 *
 * Two independent reasons, either sufficient:
 *   1. It called VelocityControl_GetLeftSetpointRpm(), which FIX 26 DELETED
 *      (V9-6: a setpoint getter that returned a value the caller had just
 *      written). This file therefore no longer compiled as committed - the
 *      "build landmine" recorded in docs/velocity/FIX_26_velocity_control.md
 *      §2 and in the A3 design. Restoring this main REQUIRED resolving it.
 *   2. Its own header said "Remove this whole block ... once done", and it IS
 *      done: the LEFT-wheel jitter it existed to capture was root-caused to a
 *      QEI velocity SIGN bug (windowed magnitude paired with an instantaneous
 *      direction bit) and fixed in qei.c, then verified over 5 boots.
 *
 * Removing it - rather than re-adding the deleted getter - is the resolution
 * both records already point to.
 *--------------------------------------------------------------------------*/

#ifndef USE_FREERTOS
/* ⚠️ B11: THE SUPER-LOOP EXISTS ONLY IN THE NON-RTOS BUILD NOW.
 *
 * Under USE_FREERTOS it is not declared, not defined, and not compiled - the
 * ten tasks in app.c replace it entirely. It is kept for the rollback build,
 * which is the byte-identical reference the whole B-series was verified
 * against, and which still produces the pre-B3 firmware. */
static void SuperLoop_Run(boolean batteryOk);
#endif



int main(void)
{
    boolean batteryOk;               /* A3: FALSE disables the two battery slots */
    boolean imuOk;                   /* B13: FALSE = dead I2C bus, so no tImu    */

    /* ---- Init: order matters; violations fail SILENTLY on hardware ---- */
    Port_Init(&Port_Configuration);                 /* 1. mux ALL pins; first    */
    PWM_Init();                                      /* 2. PWM0; before Motor     */

#ifndef USE_FREERTOS
    if (SysTick_Init(1U) != SYSTICK_OK) { while (1) { } }   /* 3. 1 ms timebase  */
#else
    /* 3. NO SysTick_Init UNDER FreeRTOS - the kernel owns the SysTick hardware
     * and programs STRELOAD/STCTRL itself in vPortSetupTimerInterrupt()
     * (port.c:814-830). Initialising ours too would leave the two fighting over
     * the same registers, with whoever ran last winning.
     *
     * Nothing in the init block below needs a millisecond tick to run: no
     * production module calls any SysTick_* function (verified by grep across
     * every src/*.c that this env links), and the one init that DOES need a
     * timebase - I2C - caps on TIMER0, not SysTick, which is precisely why
     * Timer0_FreeRunning_Init() must precede I2C_Init() (REVIEW 25 section 0).
     *
     * So between here and vTaskStartScheduler() there is deliberately NO
     * millisecond tick at all, and nothing misses it. */
#endif
    if (UART_Init()      != UART_OK)    { while (1) { } }   /* 4. debug console  */

    if (Motor_Init() != MOTOR_OK)                   /* 5. needs PWM_Init done     */
    {
        UART_SendString(DIAG_UART, "# Motor_Init FAILED\r\n");
        while (1) { }
    }
    if (Encoder_Init() != ENCODER_OK)               /* 6. needs Port_Init done     */
    {
        UART_SendString(DIAG_UART, "# Encoder_Init FAILED\r\n");
        while (1) { }
    }

    Can_Init();                 /* 7. needs Port_Init (PE4/PE5); enables IRQ39     */
    VelocityControl_Init();     /* 8. needs Port/PWM/Motor/Encoder up              */
    SteeringControl_Init();     /* 9. OWNS steering HAL (Servo+TimerPWM, ServoFb+Adc), self-centers */
    ClusterComm_Init();         /* 10. no-op today; after Encoder+CAN              */

    /* 10b. Persistent odometer. Restores the lifetime distance from EEPROM, so
     * it must run BEFORE the first ClusterComm_SendVehicleStatus() publishes
     * odo_m. NOT a halt on failure - a dead EEPROM costs persistence, not the
     * vehicle: the odometer keeps counting in RAM and simply will not survive
     * the next reset. */
    if (Odo_Init() != E_OK)
    {
        UART_SendString(DIAG_UART,
            "# WARN: Odo_Init FAILED (eeprom err=");
        UART_SendInteger(DIAG_UART, (sint32)Eeprom_GetErrorCount());
        UART_SendString(DIAG_UART,
            ") - odometer is RAM-ONLY this run, it will NOT persist\r\n");
    }
    else
    {
        UART_SendString(DIAG_UART, "# odometer: ");
        UART_SendInteger(DIAG_UART, (sint32)Odo_GetMetres());
        UART_SendString(DIAG_UART, " m restored from EEPROM (next ring slot ");
        UART_SendInteger(DIAG_UART, (sint32)Odo_GetNextSlot());
        UART_SendString(DIAG_UART, ")\r\n");
    }
    JetsonComm_Init();          /* 11. no-op; after CAN+velocity+steering          */

    /* ---- 12-14. A3 battery telemetry chain. Placed LAST because it depends on
     * Port (12: pin mux for PB2/PB3) and on nothing else here, and because
     * nothing above depends on IT - so a failure in this chain can be contained
     * without leaving a half-built drive stack behind. ---- */

    batteryOk = TRUE;
    imuOk     = TRUE;

    /* 12. ⚠️ MUST precede I2C_Init - every I2C per-command timeout caps on the
     * TIMER0 tick. With TIMER0 stopped the cap never elapses and the first read
     * hangs forever (REVIEW 25 §0).
     *
     * T25-2: the return is now CHECKED. This used to be `void`, so a TIMER0
     * that failed to come up was indistinguishable from one that did - and the
     * consequence was not a degraded gauge but a hung super-loop. Reporting it
     * here is belt-and-braces: I2C_Init() below independently refuses to
     * initialise on a dead timebase (T25-1), so batteryOk would end up FALSE
     * either way. The explicit check exists to name the ROOT cause in the log
     * rather than blaming I2C for TIMER0's failure. */
    if (Timer0_FreeRunning_Init() != E_OK)
    {
        UART_SendString(DIAG_UART,
            "# WARN: Timer0_FreeRunning_Init FAILED (expiries=");
        UART_SendInteger(DIAG_UART, (sint32)Timer0_GetInitExpiryCount());
        UART_SendString(DIAG_UART,
            ") - TIMER0 dead, so I2C has no timeout base; battery AND IMU telemetry DISABLED\r\n");
        batteryOk = FALSE;
        imuOk     = FALSE;
    }

    /* 13. ⚠️ I2C_Init() TAKES NO ARGUMENT AND BRINGS UP *BOTH* BUSES - I2C0
     * (PB2/PB3, the INA226) and I2C1 (PA6/PA7, the MPU6050), per the enable
     * flags in i2c_cfg.h. This comment used to read "I2C0, PB2/PB3 - the INA226
     * bus", which was true of the only consumer that existed at the time and
     * became misleading at B13 when the IMU was wired in.
     *
     * ℹ️ Neither bus's pins are muxed by the PORT driver: i2c.c self-muxes them
     * from its own I2C_Module[] table (PINOUT.md documents this as the one
     * deliberate exception to "the PORT driver owns pin assignment"). So there
     * is no PORT_PBCFG.c entry to add for the IMU. */
    if (I2C_Init() != I2C_OK)
    {
        /* NOT a halt: see the ESSENTIAL vs AUXILIARY note in the file header. A
         * fuel-gauge bus that will not come up must not ground the vehicle. */
        UART_SendString(DIAG_UART,
            "# WARN: I2C_Init FAILED - battery telemetry DISABLED, 0x210 will not be sent, "
            "IMU DISABLED, 0x150/0x160 will not be sent\r\n");
        batteryOk = FALSE;
        imuOk     = FALSE;
    }
    else
    {
        /* 14. BatteryService OWNS the INA226 (it calls Ina226_Init internally),
         * exactly as SteeringControl owns Servo/TimerPWM/Adc. main must NOT
         * call Ina226_Init itself - that would be a double-init. */
        BatteryService_Init();

        /* One Update here both PROVES the sensor answered and takes the boot
         * anchor while the vehicle is still stationary - which is the best
         * moment for it, since the anchor is the one the rest-detector cannot
         * wait for. IsHealthy() needs a sample, so it is only meaningful after
         * this call. */
        BatteryService_Update(0.0f);

        if (BatteryService_IsHealthy() == FALSE)
        {
            UART_SendString(DIAG_UART,
                "# WARN: BatteryService not healthy after init (last i2c err=");
            UART_SendInteger(DIAG_UART, (sint32)Ina226_GetLastI2cError());
            UART_SendString(DIAG_UART,
                ") - telemetry DISABLED. I2C_OK(0) here means the part answered "
                "but MANUF_ID/CONFIG/CAL verify failed.\r\n");
            batteryOk = FALSE;
        }
        else
        {
            BatteryStatusType boot;
            (void)BatteryService_GetStatus(&boot);
            UART_SendString(DIAG_UART, "# battery: ");
            UART_SendInteger(DIAG_UART, boot.voltage_mV);
            UART_SendString(DIAG_UART, " mV, SoC ");
            UART_SendFloat(DIAG_UART, boot.soc_pct, 1U);
            UART_SendString(DIAG_UART, " % (boot anchor, IR-compensated estimate)\r\n");
        }
    }

    /* ---- 15. B13: the IMU service (MPU6050 on I2C1/PA6-PA7). ----
     *
     * Prerequisites are the SAME chain the battery already needed and they have
     * all just run in order: Port_Init -> Timer0_FreeRunning_Init -> I2C_Init
     * (imu_service.h states this explicitly; Timer0 must precede I2C or the
     * first read's cap never elapses and it hangs forever).
     *
     * ⚠️ A SENSOR THAT DOES NOT ANSWER HERE IS NOT A FAILURE TO CONTAIN, which
     * is why this looks different from the battery block above. ImuService_Init
     * cannot hang and cannot trap on an absent part - it reports unhealthy and
     * the service retries on its own backoff. So we log what happened and leave
     * imuOk TRUE: tImu is still created, publishes nothing until a real sample
     * exists, and picks the sensor up if it appears later. The only thing that
     * clears imuOk is a dead BUS, handled above, because that leaves nothing to
     * retry against. */
    if (imuOk != FALSE)
    {
        ImuService_Init();

        if (ImuService_IsHealthy() != FALSE)
        {
            UART_SendString(DIAG_UART,
                "# imu: MPU6050 up on I2C1 (WHO_AM_I ok) - 0x150/0x160 at 50 Hz\r\n");
        }
        else
        {
            UART_SendString(DIAG_UART,
                "# WARN: MPU6050 did not answer (who_am_i=0x");
            UART_SendInteger(DIAG_UART, (sint32)MPU6050_GetLastWhoAmI());
            UART_SendString(DIAG_UART,
                ") - tImu WILL RETRY on backoff; 0x150/0x160 stay SILENT until a "
                "real sample exists (never zeroes)\r\n");
        }
    }

    UART_SendString(DIAG_UART,
        "\r\n# SYSTEM MAIN up: CAN 0x100/0x120 in, "
        "0x110/0x130/0x150/0x160/0x200/0x210 out.\r\n"
        "# RX command-loss failsafe ACTIVE: no accepted 0x100/0x120 for ");
    UART_SendInteger(DIAG_UART, (sint32)CMD_TIMEOUT_MS);
    UART_SendString(DIAG_UART,
        " ms -> zero velocity, HOLD steering.\r\n"
        "# !! TIMEOUT IS PROVISIONAL (Jetson cadence never measured) - VERIFY "
        "BEFORE GROUND USE. Wheels-up until then.\r\n");

#ifdef USE_FREERTOS
    /*------------------------------------------------------------------------
     * B11 - hand everything to the application layer. THE PORT ENDS HERE.
     *
     * main() is now exactly what ARCHITECTURE_app_layer.md section 2 specified
     * before any of this existed: "a THIN shell: hardware Init in the required
     * order, then App_Start() + vTaskStartScheduler()."
     *
     * App_Start() creates all ten tasks and never returns. There is no
     * super-loop under the RTOS - SuperLoop_Run is compiled only for the
     * non-RTOS rollback build below.
     *----------------------------------------------------------------------*/
    (void)App_Start(batteryOk, imuOk);

    /* App_Start returns only if vTaskStartScheduler() itself failed, which with
     * static allocation can only mean the Idle task could not be created - a
     * configuration error, not heap exhaustion, because there is no heap. */
    UART_SendString(DIAG_UART,
        "# App_Start RETURNED - scheduler did not start - HALTED\r\n");
    while (1) { }
#else
    /* Original behaviour, unchanged: run the legacy super-loop inline, forever.
     * This is the ROLLBACK and the reference the whole B-series was verified
     * against. */
    SuperLoop_Run(batteryOk);
#endif

    /* unreachable */
    return 0;
}

#ifndef USE_FREERTOS
static void SuperLoop_Run(boolean batteryOk)
{
    uint32 now;
    uint32 lastTickMs;    /* per-millisecond edge guard (one TX slot per ms) */
    uint32 lastVelMs;     /* wrap-safe, phase-locked 20 ms control tick      */
    uint32 lastBeatMs;    /* debug heartbeat                                 */
    uint32 lastCanChkMs;  /* C6-1 bus-health / bus-off recovery cadence       */
    uint32 canRecoverAttempts = 0U;  /* observability for C6-1                */

#ifndef USE_FREERTOS
    /* A4-1 RX command-loss failsafe state.
     * ⚠️ B10: under FreeRTOS this state lives in tSafety (file-scope, so the
     * heartbeat can report it). These locals exist only for the non-RTOS
     * build's inline failsafe. */
    uint32  lastCmdActivityMs;       /* when a command was last ACCEPTED        */
    uint32  lastCmdSeen;             /* snapshot of the accepted-command counts */
    boolean cmdFailsafeActive = FALSE;
    boolean cmdEverSeen       = FALSE; /* has the host EVER been heard from?    */
    uint32  cmdFailsafeCount  = 0U;  /* sticky, observable on the heartbeat     */
#endif

    now        = SYS_TICK_MS();
    lastTickMs = now;
    lastVelMs    = now;
    lastBeatMs   = now;
    lastCanChkMs = now;

#ifndef USE_FREERTOS
    /* Seed the deadline at boot rather than at zero (see cmdEverSeen below). */
    lastCmdActivityMs = now;
    lastCmdSeen       = JetsonComm_GetVelocityCommandCount() +
                        JetsonComm_GetSteeringCommandCount();
#endif

    for (;;)
    {
        /* ---- RX drain ----
         * ⚠️ B8: MOVED OUT to tRosRx, which blocks on the ISR's "ring non-empty"
         * edge instead of being polled every loop iteration. The routing itself
         * (JetsonComm_Poll and its handlers) is unchanged - only its execution
         * context moved, and tRosRx is now the single writer of the control
         * setpoints (V9-R3).
         *
         * ⚠️ The A4-1 failsafe below still reads the accepted-command counters,
         * which tRosRx now writes - see the note at the failsafe for why that
         * boundary is coherent without any lock. */
#ifndef USE_FREERTOS
        JetsonComm_Poll();
#endif

        now = SYS_TICK_MS();

        /* ---- Velocity control @>=20 ms: wrap-safe subtraction, re-reference
         * (lastVelMs = now). This GUARANTEES every VelocityControl_Update() is
         * >= 20 ms after the previous one. That is REQUIRED: QEISPEED (the QEI
         * hardware velocity sample) only refreshes once per 20 ms window, so
         * calling Update faster than that re-reads the SAME stale sample and
         * breaks the closed loop regardless of gains (qei_cfg.h / QEI_WINDOW_
         * AUDIT.md) -> motor jitter. A fixed-increment (+= 20) would fire a
         * <20 ms "catch-up" update after any loop overshoot (100 Hz TX,
         * heartbeat, RX) and trip exactly that; re-referencing to `now` cannot.
         * The ~1 ms phase drift this reintroduces is harmless (polling slower
         * than the window is fine). Matches the proven main_control_layer.c. ---- */
#ifndef USE_FREERTOS
        if ((now - lastVelMs) >= VEL_PERIOD_MS)
        {
            VelocityControl_Update();
            lastVelMs = now;
        }
#else
        /* ⚠️ B9: MOVED OUT to tVelocity (prio 8). The >= 20 ms re-referencing
         * above is reproduced there as an explicit guard on top of
         * xTaskDelayUntil - see the long note at VELOCITY_TASK_* for why
         * xTaskDelayUntil alone is not sufficient. */
#endif

        /* ---- C6-1: CAN bus health. The ISR only OBSERVES bus-off (recovery
         * involves a rejoin delay that must not run in interrupt context), so
         * the recovery call belongs here in the loop.
         *
         * SAFETY POLICY on bus-off: command ZERO VELOCITY. There is no RX
         * command-loss failsafe (jetson_comm documents this as deferred), so
         * without this the last velocity setpoint persists and the vehicle
         * keeps driving with its comms dead - "recovered the bus but kept
         * driving on a stale command" is only half a fix.
         *
         * Steering is deliberately NOT re-centred: snapping the wheels straight
         * mid-corner is its own hazard, the same reasoning that made
         * SteeringControl_SetAngle HOLD the last valid angle when it rejects a
         * non-finite command (R3-1). Stop driving, hold the wheel.
         *
         * The zero setpoint is re-applied on every check while bus-off
         * persists, so a stale command cannot creep back. */
        if ((now - lastCanChkMs) >= CAN_HEALTH_PERIOD_MS)
        {
            lastCanChkMs = now;

            if (Can_GetLastBusError() == CAN_ERROR_BUS_OFF)
            {
                /* R8-1: VelocityControl_Stop(), NOT SetSetpoint(0,0). Proven on
                 * hardware: a PID latched in FAULT_NAN ignores the setpoint and
                 * keeps returning its last output, so SetSetpoint(0,0) left the
                 * wheels driving at 31%/35% duty while the setpoint read 0.000.
                 * Stop() calls Motor_Stop on both wheels AND PID_Reset on both
                 * handles, so it halts immediately and clears any latch. It is
                 * also the better semantic for a comms-loss action: an immediate
                 * stop rather than a closed-loop ramp from a charged integrator. */
                VelocityControl_Stop();                    /* safe state first */
                (void)Can_RecoverBusOff();                 /* then rejoin      */
                canRecoverAttempts++;
            }

            /* ---- A4-1: RX COMMAND-LOSS FAILSAFE ----
             * ⚠️ B10: MOVED OUT to tSafety (prio 9, the HIGHEST in the system),
             * where nothing can starve it. The logic is verbatim; only its
             * execution context changed. It also gained the V9-R4 inhibit latch,
             * which is what lets its Stop() survive preempting tVelocity
             * mid-Update() - see velocity_control.c.
             *
             * The C6-1 bus-off check ABOVE stays here deliberately: it answers a
             * different question ("did the BUS die?" vs "did the HOST die?"),
             * and it is what still gives tSuperLoop a reason to hold its 100 ms
             * cadence.
             *
             * ℹ️ The bus-off path also calls VelocityControl_Stop(), so it now
             * sets the inhibit latch too - an IMPROVEMENT that falls out for
             * free: previously an Update() between two 100 ms bus-off
             * re-assertions could drive the motors again; now it cannot.
             *
             * The non-RTOS build keeps the failsafe inline, exactly as before. */
#ifndef USE_FREERTOS
            /* ---- A4-1: RX COMMAND-LOSS FAILSAFE (non-RTOS build) ----
             * The other half of C6-1. Bus-off catches "the bus died"; this
             * catches "the bus is fine and the HOST died" - which the hardware
             * cannot report, because a silent Jetson is electrically identical
             * to an idle one. Until now that was the gap that kept every test
             * wheels-up: liveness was ONE-DIRECTIONAL (the host can detect a
             * dead Tiva via 0x7A0->0x7A1; the Tiva could not detect a dead
             * host).
             *
             * MECHANISM: jetson_comm publishes monotonic counts of ACCEPTED
             * commands. We watch for CHANGE, not for a rate - so this stays
             * correct whatever cadence the Jetson turns out to use, which
             * matters because that cadence is still unmeasured (CMD_TIMEOUT_MS).
             *
             * ⚠️ WHY THE TWO CHANNELS ARE COMBINED. A4-1 asks for the
             * timestamps to be kept separate, and they are (jetson_comm exposes
             * them per channel). The TRIP RULE deliberately fires only when
             * NEITHER has advanced, i.e. on "the host is gone" rather than "one
             * channel went quiet". Tripping per-channel would demand that BOTH
             * streams be periodic - an assumption we cannot make about an
             * unmeasured host, and getting it wrong means stopping a vehicle
             * whose controller is demonstrably alive and still steering it.
             * Revisit once the real cadence of each channel is known.
             *
             * ⚠️ STEERING IS NOT TOUCHED. That IS the action: HOLD the last
             * angle. Policy decided in S10-5 and identical to the bus-off path
             * three lines up - snapping the wheels straight mid-corner is its
             * own hazard. SteeringControl_Center() must NOT be called here. */
            {
                /* ⚠️ B8: THIS READ IS NOW CROSS-TASK, and it is coherent without
                 * a lock. tRosRx (prio 7) writes these counters; this failsafe
                 * check reads them from tSuperLoop (prio 4), so tRosRx can
                 * preempt BETWEEN the two reads and the sum can mix "velocity
                 * count at T1" with "steering count at T2".
                 *
                 * THAT TEAR IS HARMLESS HERE, and the reason is worth stating
                 * because it is the opposite conclusion to A4-4:
                 *   - each counter is a plain uint32, so each individual read is
                 *     ATOMIC on Cortex-M4 (no half-value is possible);
                 *   - both counters are MONOTONIC NON-DECREASING;
                 *   - this code only ever asks "is the sum DIFFERENT from last
                 *     time?", never what the sum's absolute value means.
                 * A torn sum can therefore only be >= the previous sum, and it
                 * differs if and only if at least one counter advanced - which
                 * is exactly the question being asked. The answer is correct
                 * under every interleaving.
                 *
                 * CONTRAST WITH A4-4 (the 0x110 tick pair), which looked like
                 * the same shape and was NOT harmless: there the pair fed an
                 * INTEGRATOR, so a tear became a permanent odometry step. The
                 * distinction is not "two reads" - it is what the reader does
                 * with them. Do not generalise either way without checking. */
                uint32 cmdSeen = JetsonComm_GetVelocityCommandCount() +
                                 JetsonComm_GetSteeringCommandCount();

                if (cmdSeen != lastCmdSeen)
                {
                    /* Fresh accepted command: re-arm. The setpoint it carried
                     * has ALREADY been applied by JetsonComm_Poll, so recovery
                     * needs no action here beyond clearing the latch - and
                     * certainly no power-cycle. */
                    lastCmdSeen       = cmdSeen;
                    lastCmdActivityMs = now;
                    cmdFailsafeActive = FALSE;
                    cmdEverSeen       = TRUE;
                }
                /* ⚠️ `cmdEverSeen` GATES THE TRIP, AND IT IS NOT COSMETIC.
                 * Without it the failsafe fires 500 ms after EVERY boot on any
                 * bench with no host attached - observed on hardware, first
                 * run. The vehicle is already stopped at that point (Init
                 * leaves the setpoint at zero and the motors stopped), so the
                 * ACTION was harmless; what it wrecked was the SIGNAL. `cmd_fs`
                 * is meant to answer "did we ever lose the host?", and a count
                 * of 1 on a machine that was never commanded makes the only
                 * interesting value - zero - unreachable.
                 *
                 * A failsafe counts LOSS. You cannot lose what you never had,
                 * so "never commanded" is a distinct state, not an episode:
                 * boot into stopped-and-waiting silently, and start policing
                 * the deadline from the first accepted command onward.
                 *
                 * Safe by construction: nothing can be driving before the first
                 * accepted command, because only JetsonComm_Poll ever writes a
                 * non-zero setpoint. */
                else if ((cmdEverSeen != FALSE) &&
                         (cmdFailsafeActive == FALSE) &&
                         ((now - lastCmdActivityMs) >= CMD_TIMEOUT_MS))
                {
                    /* R8-1 again: Stop(), not SetSetpoint(0,0) - a PID latched
                     * in FAULT_NAN ignores the setpoint and keeps driving.
                     * Un-gated by design (V9-3): a failsafe must fire even if a
                     * module never initialised. */
                    VelocityControl_Stop();

                    cmdFailsafeActive = TRUE;
                    if (cmdFailsafeCount < 0xFFFFFFFFUL) { cmdFailsafeCount++; }

                    /* Latched, so this runs ONCE per loss episode rather than
                     * every 100 ms. Unlike bus-off - where the bus may still
                     * deliver a stale frame and the zero must be re-asserted -
                     * nothing can rewrite the setpoint while no commands are
                     * being accepted, so re-applying would only re-reset the
                     * PIDs and spam the log for no benefit. */
                    UART_SendString(DIAG_UART,
                        "# FAILSAFE: no accepted 0x100/0x120 for ");
                    UART_SendInteger(DIAG_UART, (sint32)CMD_TIMEOUT_MS);
                    UART_SendString(DIAG_UART,
                        " ms -> velocity ZEROED, steering HELD\r\n");
                }
                else
                {
                    /* armed and fresh, or already latched - nothing to do */
                }
            }
#endif
        }

        /* ---- Per-millisecond slot body ----
         *
         * ⚠️ B7: THE TX SLOT RESIDUES ARE RETIRED under FreeRTOS. This block
         * used to carry all five senders on mutually-exclusive residues
         * (0 and 5 mod 10; 9, 47, 51 mod 100) with a written proof in this
         * file's header that no two could coincide - the hand-timed
         * "<=1 Can_Transmit per ms" invariant.
         *
         * That is now STRUCTURAL instead of arithmetic, in two layers:
         *   1. tCanTx (B6) is the only Can_Transmit caller and sleeps on the
         *      hardware TX-complete edge, so two transmits cannot overlap
         *      whatever anyone adds later;
         *   2. the GLOBAL PHASE PLAN (see the top of this file) keeps the
         *      producers' wakes on distinct tick residues, so two frames are
         *      not even POSTED in the same millisecond.
         * The residues were kept at B6 purely as pacing while all five senders
         * were still here; B7 moves them to tRosTx/tClusterTx and retires them.
         *
         * ⚠️ WHAT DOES **NOT** RETIRE: tBattery's 43 and tOdo's 53 phase-locks.
         * Those are not TX pacing - they keep those tasks' wakes off the
         * producers' ticks, which is what BUDGET-6 was about. They stay.
         *
         * What remains in this loop: the RX drain, the 20 ms control update,
         * the C6-1/A4-1 checks above, and the heartbeat below. All of it moves
         * out at B8/B9/B10, after which tSuperLoop disappears (B11).
         *
         * The non-RTOS build keeps every slot inline, exactly as before. */
        if (now != lastTickMs)
        {
            lastTickMs = now;

#ifndef USE_FREERTOS
            if ((now % 10U) == 0U)                    /* 0x110 VelocityFeedback, 100 Hz */
            {
                (void)JetsonComm_SendVelocityFeedback();
            }
            else if ((now % 10U) == 5U)               /* 0x130 SteeringFeedback, 100 Hz */
            {
                (void)JetsonComm_SendSteeringFeedback();
            }

            if ((now % 100U) == 9U)                   /* 0x200 VehicleStatus, 10 Hz     */
            {
                (void)ClusterComm_SendVehicleStatus();
            }

            if ((now % 100U) == PING_TX_SLOT_MS)
            {
                (void)NodePing_MainFunction();
            }

            if ((now % 100U) == ODO_SLOT_MS)
            {
                Odo_MainFunction();
            }

            if (batteryOk != FALSE)
            {
                if ((now % 100U) == BATT_UPDATE_SLOT_MS)
                {
                    float32 vLeft  = Encoder_GetLinearVelocityM(ENCODER_ID_LEFT);
                    float32 vRight = Encoder_GetLinearVelocityM(ENCODER_ID_RIGHT);
                    float32 speedMps = (vLeft + vRight) * 0.5f;

                    if (speedMps < 0.0f) { speedMps = -speedMps; }

                    BatteryService_Update(speedMps);
                }
                else if ((now % 100U) == BATT_TX_SLOT_MS)
                {
                    (void)ClusterComm_SendBatteryStatus();           /* 0x210 */
                }
                else
                {
                    /* no battery work this millisecond */
                }
            }
#endif /* !USE_FREERTOS */

            /* ---- Debug heartbeat (UART only; NEVER a Can_Transmit) ---- */
            if ((now - lastBeatMs) >= HEARTBEAT_MS)
            {
                lastBeatMs = now;
                UART_SendString(DIAG_UART, "# alive t=");
                UART_SendInteger(DIAG_UART, (sint32)now);
                UART_SendString(DIAG_UART, " bus_err=");
                UART_SendInteger(DIAG_UART, (sint32)Can_GetLastBusError());
                UART_SendString(DIAG_UART, " rx_overrun=");
                UART_SendInteger(DIAG_UART, (sint32)Can_GetRxOverrunCount());
                UART_SendString(DIAG_UART, " can_recov=");
                UART_SendInteger(DIAG_UART, (sint32)canRecoverAttempts);

                /* A4-1: sticky failsafe count + current latch state. `fs=` is
                 * the number of loss EPISODES since boot (not a level), so a
                 * non-zero count with FS:0 means "it happened and recovered" -
                 * which is exactly the history a bench operator needs and which
                 * a bare boolean would have thrown away. */
                UART_SendString(DIAG_UART, " cmd_fs=");
#ifdef USE_FREERTOS
                UART_SendInteger(DIAG_UART, (sint32)sf_failsafeCount);
#else
                UART_SendInteger(DIAG_UART, (sint32)cmdFailsafeCount);
#endif
                /* FS:W = never commanded (waiting, deadline not policed yet)
                 * FS:0 = armed, host fresh
                 * FS:1 = failsafe LATCHED, velocity zeroed, steering held
                 * The three are distinct on purpose: "W" and "0" would
                 * otherwise be indistinguishable on a bench, and they mean very
                 * different things about whether the vehicle is being watched. */
#ifdef USE_FREERTOS
                if      (sf_everSeen       == FALSE) { UART_SendString(DIAG_UART, " FS:W"); }
                else if (sf_failsafeActive != FALSE) { UART_SendString(DIAG_UART, " FS:1"); }
                else                                 { UART_SendString(DIAG_UART, " FS:0"); }
                /* B10 / V9-R4: the LATCH itself, so a bench operator can see the
                 * stop is being HELD rather than merely having been issued once. */
                UART_SendString(DIAG_UART,
                    (VelocityControl_IsInhibited() != FALSE) ? " INH:1" : " INH:0");
                UART_SendString(DIAG_UART, " sf_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)sf_hwmWords);
#else
                if      (cmdEverSeen       == FALSE) { UART_SendString(DIAG_UART, " FS:W"); }
                else if (cmdFailsafeActive != FALSE) { UART_SendString(DIAG_UART, " FS:1"); }
                else                                 { UART_SendString(DIAG_UART, " FS:0"); }
#endif

                if (batteryOk != FALSE)
                {
                    BatteryStatusType batt;
                    (void)BatteryService_GetStatus(&batt);
                    UART_SendString(DIAG_UART, " batt=");
                    UART_SendInteger(DIAG_UART, batt.voltage_mV);
                    UART_SendString(DIAG_UART, "mV/");
                    UART_SendInteger(DIAG_UART, batt.current_mA);
                    UART_SendString(DIAG_UART, "mA soc=");
                    UART_SendFloat(DIAG_UART, batt.soc_pct, 1U);
                    UART_SendString(DIAG_UART, "% flags=");
                    UART_SendInteger(DIAG_UART, (sint32)batt.flags);
                }
                else
                {
                    UART_SendString(DIAG_UART, " batt=DISABLED");
                }
#ifdef USE_FREERTOS
                /* Diagnostic only, no logic. Minimum stack WORDS ever free on
                 * this task - the measurement that turns the budget's [E]
                 * estimate into an [M] (budget 8.2).
                 * ⚠️ Read it after the loop has been round several times: at B1
                 * the first sample under-reported by 28 words because the
                 * deepest path had not run yet. A 1 Hz heartbeat means the
                 * first line is already ~1000 iterations in, so it is settled. */
                UART_SendString(DIAG_UART, " hwm=");
                UART_SendInteger(DIAG_UART,
                                 (sint32)uxTaskGetStackHighWaterMark(NULL));
                /* B4: tBattery's own numbers, published for observability.
                 * bt_hwm = its stack high-water; bt_wcet = the WORST
                 * BatteryService_Update() seen since boot, TIMER0-measured. */
                if (batteryOk != FALSE)
                {
                    UART_SendString(DIAG_UART, " bt_hwm=");
                    UART_SendInteger(DIAG_UART, (sint32)bt_hwmWords);
                    UART_SendString(DIAG_UART, " bt_wcet=");
                    UART_SendInteger(DIAG_UART, (sint32)bt_maxUpdateUs);
                    UART_SendString(DIAG_UART, "us");
                }
                /* B5: tOdo's stack, plus the two numbers that prove the EEPROM
                 * writer still functions as a task - saves completed and the
                 * wear-ring position. */
                UART_SendString(DIAG_UART, " od_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)od_hwmWords);
                UART_SendString(DIAG_UART, " odo=");
                UART_SendInteger(DIAG_UART, (sint32)Odo_GetMetres());
                UART_SendString(DIAG_UART, "m saves=");
                UART_SendInteger(DIAG_UART, (sint32)Odo_GetSaveCount());
                UART_SendString(DIAG_UART, "/slot");
                UART_SendInteger(DIAG_UART, (sint32)Odo_GetNextSlot());

                /* B6 / A4-3: the CAN TX queue's health. Three DISTINCT failure
                 * causes, deliberately not lumped:
                 *   qfull - a producer outran the drainer (SIZING)
                 *   fail  - Can_Transmit refused past the C6-2 limit (BUS)
                 *   tmo   - accepted but no TX-complete edge (nobody ACKing)
                 * peak is the deepest the queue has ever been - the number that
                 * justifies (or shrinks) CANTX_QUEUE_DEPTH from evidence. */
                UART_SendString(DIAG_UART, " ct_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetStackHwm());
                UART_SendString(DIAG_UART, " tx=");
                UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetTxCount());
                UART_SendString(DIAG_UART, " peak=");
                UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetPeakDepth());
                UART_SendString(DIAG_UART, " qfull=");
                UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetQueueFullDrops());
                UART_SendString(DIAG_UART, " txfail=");
                UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetTxFailCount());
                UART_SendString(DIAG_UART, " txtmo=");
                UART_SendInteger(DIAG_UART, (sint32)CanTxQueue_GetTxTimeoutCount());
                UART_SendString(DIAG_UART, " rt_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)rt_hwmWords);
                UART_SendString(DIAG_UART, " cl_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)cl_hwmWords);
                UART_SendString(DIAG_UART, " rr_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)rr_hwmWords);
                /* B9: ★ vel_min is THE proof the QEI guard works - it must
                 * never read below 20. vel_skip counts refused early wakes. */
                UART_SendString(DIAG_UART, " vt_hwm=");
                UART_SendInteger(DIAG_UART, (sint32)vt_hwmWords);
                UART_SendString(DIAG_UART, " vel_ivl=");
                UART_SendInteger(DIAG_UART, (sint32)vt_minIntervalMs);
                UART_SendString(DIAG_UART, "/");
                UART_SendInteger(DIAG_UART, (sint32)vt_maxIntervalMs);
                UART_SendString(DIAG_UART, "ms skip=");
                UART_SendInteger(DIAG_UART, (sint32)vt_skipped);
                UART_SendString(DIAG_UART, " vel_us=");
                UART_SendInteger(DIAG_UART, (sint32)vt_minUpdateUs);
                UART_SendString(DIAG_UART, "/");
                UART_SendInteger(DIAG_UART, (sint32)vt_maxUpdateUs);
#endif
                UART_SendString(DIAG_UART, "\r\n");
            }
        }

#ifdef USE_FREERTOS
        /*--------------------------------------------------------------------
         * The ONLY other body edit: yield for one tick.
         *
         * Pre-B3 this loop spun freely, iterating thousands of times per
         * millisecond, and the `now != lastTickMs` edge guard above filtered
         * that down to one slot-body per millisecond. Under a preemptive
         * scheduler a free-spinning task at any priority would starve
         * everything below it, so it must block.
         *
         * WHY vTaskDelay(1) AND NOT xTaskDelayUntil - this is the opposite of
         * the choice tVelocity will make later, and the difference matters:
         *
         *   vTaskDelay(1) wakes when the tick counter reaches (tick-at-call + 1).
         *   As long as one iteration finishes inside its millisecond, the task
         *   runs EXACTLY ONCE PER TICK and no value of `now` is ever skipped -
         *   so every slot residue (0,5 mod 10; 9,43,47,51,53 mod 100) is
         *   observed exactly as before. If an iteration ever overruns, the
         *   delay is measured from the CALL, so the loop simply resumes on the
         *   next tick and self-corrects.
         *
         *   xTaskDelayUntil would instead try to CATCH UP after an overrun,
         *   firing back-to-back iterations to make up lost time - which for a
         *   residue-based slot scheme means double-firing a slot. Drift-free
         *   is the wrong property here; not-skipping and not-repeating is the
         *   right one.
         *
         * ⚠️ THE INVARIANT THIS RESTS ON: one iteration must stay under 1 ms.
         * The heaviest is the ~480 us battery slot (43). If a millisecond is
         * ever skipped, its slot is MISSED and the symptom is a dropped frame -
         * directly visible as a rate below 100.0/10.0 Hz in candump, which is
         * exactly what B3's standing criterion measures.
         *
         * (b) in the plan - xTaskDelayUntil - is noted there as a later
         * refinement. It is not one: it is the wrong primitive for this loop.
         * The right refinement is B4-onward, which removes the loop entirely.
         *------------------------------------------------------------------*/
        vTaskDelay(1U);
#endif
    }

    /* unreachable - SuperLoop_Run never returns in either build mode */
}
#endif /* !USE_FREERTOS */
