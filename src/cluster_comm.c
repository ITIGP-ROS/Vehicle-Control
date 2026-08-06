/******************************************************************************
 *
 * Module: ClusterComm
 *
 * File Name: cluster_comm.c
 *
 * Description: Tiva -> instrument-cluster display traffic. Converts Encoder
 *              HAL data into the VehicleStatus (0x200) display frame and
 *              transmits it via the CAN MCAL.
 *
 ******************************************************************************/

#include "cluster_comm.h"
#include "encoder.h"   /* HAL: Encoder_GetLinearVelocityM()                  */
#include "battery_service.h" /* service: BatteryService_GetStatus() for 0x210    */
#include "odo.h"        /* service: the PERSISTENT lifetime odometer (0x200)  */
#include "robot.h"     /* cantools: robot_vehicle_status_pack(), 0x200 ids   */
#include "can.h"       /* MCAL transport (types, Can_Receive)                */
#include "can_tx_queue.h" /* B6: ALL transmits go through the TX queue now.
                          * Non-RTOS builds pass straight through to
                          * Can_Transmit(), so this module needs no #ifdef. */

/*******************************************************************************
 *                          Local Configuration                                *
 *******************************************************************************/

/* Gear enum on the wire (matches DBC comment: 0=N, 1=D, 2=R). */
#define CLUSTER_GEAR_NEUTRAL        (0U)
#define CLUSTER_GEAR_DRIVE          (1U)
#define CLUSTER_GEAR_REVERSE        (2U)

/* Below this speed the vehicle is reported as Neutral (no meaningful dir).
 * 8 m/min = 0.133 m/s = 0.48 km/h, i.e. the same physical threshold as the
 * 0.5 km/h it replaced - rounded to a whole m/min so it reads naturally in the
 * new unit. Kept small on purpose: this only decides N vs D/R, and the drive
 * cannot hold a speed this low anyway. */
#define CLUSTER_GEAR_NEUTRAL_MPM    (8.0f)

/* DBC "speed" is uint16 with 1 LSB = 1 m/min (bytes 0-1 of 0x200), so the value
 * packed is WHOLE m/min and the ceiling is the uint16 span.
 * History: uint8 whole km/h (capped 178) -> uint16 x0.1 km/h (2026-07-31,
 * resolution) -> uint16 x1 m/min (2026-08-05, UNIT change - see below). */
#define CLUSTER_SPEED_MPM_MAX       (65535)

/* m/s -> m/min. UNIT CHANGED 2026-08-05 from CLUSTER_MPS_TO_KMH (3.6f).
 * This robot runs 0.7-2 m/s. In km/h that is 2-7 - a display that looks parked.
 * In m/min it is 40-120, which fills the gauge with real numbers. Whole m/min
 * is enough resolution: 1 m/min = 0.017 m/s, already finer than the drive can
 * hold, so tenths would only render noise. */
#define CLUSTER_MPS_TO_MPM          (60.0f)

/* TRIP deadband, in metres per 10 Hz sample. One encoder count is
 * QEI_CM_PER_COUNT ~= 0.0083 cm = 83 um, so a stationary wheel dithering by a
 * count or two must not accumulate. 1 mm per sample is ~12 counts - far above
 * dither, and at the 8 m/min neutral threshold a real sample moves 13 mm, so
 * genuine motion is never discarded. Without this, 10 Hz x hours of parked
 * dither would slowly invent distance. */
#define CLUSTER_TRIP_DEADBAND_M     (0.001f)

/* uint16 ceiling of the DBC trip_m signal. Saturate, never wrap. */
#define CLUSTER_TRIP_MAX_M          (65535)

/*******************************************************************************
 *                          Private Helpers                                    *
 *******************************************************************************/

/* Signed average forward velocity of the two driven wheels, in m/s (+fwd).
 * Encoder_GetLinearVelocityM() already applies the qei_cfg wheel radius
 * (65 mm dia = 0.0325 m) and the direction sign, so the wheel constant lives
 * in exactly one place (qei_cfg.h). */
/* TRIP accumulator state. This module was stateless before 2026-08-05; a trip
 * meter is inherently cumulative, so it needs exactly these two words.
 *
 * ⚠️ WHY ACCUMULATE |delta| RATHER THAN PUBLISH THE ENCODER DISTANCE DIRECTLY.
 * Encoder_GetAverageDistanceCm() is SIGNED net displacement, so driving forward
 * 10 m and back 10 m returns to 0 - and a uint16 signal would WRAP if it ever
 * went negative. A trip meter measures TOTAL PATH LENGTH: reversing adds. So we
 * integrate the absolute increment. Cost: it can never be reconstructed from a
 * single encoder read, which is exactly why the state lives here. */
static float32 cluster_tripMetres   = 0.0f;
static float32 cluster_lastDistM    = 0.0f;
static boolean cluster_tripSeeded   = FALSE;

static float32 ClusterComm_VehicleVelMps(void)
{
    float32 vLeft  = Encoder_GetLinearVelocityM(ENCODER_ID_LEFT);
    float32 vRight = Encoder_GetLinearVelocityM(ENCODER_ID_RIGHT);
    return (vLeft + vRight) * 0.5f;
}

/* Advance the TRIP accumulator from the encoder's cumulative distance.
 * Called once per published frame (10 Hz). */
static void ClusterComm_UpdateTrip(void)
{
    float32 nowM = Encoder_GetAverageDistanceCm() * 0.01f;   /* cm -> m */
    float32 delta;

    /* First call after boot/reset: adopt the reading as the origin rather than
     * treating it as distance already travelled. */
    if (cluster_tripSeeded == FALSE)
    {
        cluster_lastDistM  = nowM;
        cluster_tripSeeded = TRUE;
        return;
    }

    delta = nowM - cluster_lastDistM;
    if (delta < 0.0f) { delta = -delta; }        /* total path, not net displacement */

    if (delta >= CLUSTER_TRIP_DEADBAND_M)
    {
        cluster_tripMetres += delta;
        cluster_lastDistM   = nowM;              /* only advance on a real move, so
                                                  * sub-deadband creep still sums */
        if (cluster_tripMetres > (float32)CLUSTER_TRIP_MAX_M)
        {
            cluster_tripMetres = (float32)CLUSTER_TRIP_MAX_M;   /* saturate, never wrap */
        }

        /* ⚠️ ODO IS FED FROM THE SAME |delta|, ON PURPOSE. Deriving the lifetime
         * odometer from its own origin would give it a second copy of the
         * encoder-origin problem - and a fatal one, because 0x140 ResetCommand
         * zeroes the encoder counters underneath us. TRIP survives that by
         * re-seeding its origin (ClusterComm_ResetTrip); ODO cannot re-seed,
         * because it must not lose its total.
         *
         * Feeding ODO pure INCREMENTS removes the problem entirely: it holds no
         * origin, so there is nothing for a counter reset to invalidate. The
         * reset re-seeds TRIP's origin and ODO simply never notices. */
        Odo_AddMetres(delta);
    }
}

void ClusterComm_ResetTrip(void)
{
    /* 0x140 ResetCommand. Zero the trip and DROP THE ORIGIN: the caller has
     * just zeroed the encoder counters, so cluster_lastDistM now refers to a
     * distance that no longer exists. Without re-seeding, the next sample would
     * compute a |delta| of the entire pre-reset distance and add it to both
     * TRIP and ODO in one step. Clearing the seeded flag makes the next call
     * adopt the new (zeroed) reading as the origin instead.
     *
     * ODO IS DELIBERATELY UNTOUCHED - it is the lifetime total. */
    cluster_tripMetres = 0.0f;
    cluster_lastDistM  = 0.0f;
    cluster_tripSeeded = FALSE;
}

/* Fill a VehicleStatus struct from the current encoder state. */
static void ClusterComm_BuildVehicleStatus(struct robot_vehicle_status_t *st)
{
    float32 vAvg = ClusterComm_VehicleVelMps();          /* signed m/s, +fwd */
    float32 mag  = (vAvg < 0.0f) ? -vAvg : vAvg;         /* |m/s|, no libm   */
    float32 mpm  = mag * CLUSTER_MPS_TO_MPM;
    sint32  rounded;

    /* Convert to the DBC's whole-m/min units, round to nearest, then clamp to
     * the uint16 range the signal can carry. */
    rounded = (sint32)(mpm + 0.5f);
    if (rounded < 0)                     { rounded = 0; }
    if (rounded > CLUSTER_SPEED_MPM_MAX) { rounded = CLUSTER_SPEED_MPM_MAX; }
    st->speed = (uint16)rounded;

    /* Gear from speed magnitude + direction (sign of the signed average). */
    if (mpm < CLUSTER_GEAR_NEUTRAL_MPM) { st->gear = CLUSTER_GEAR_NEUTRAL; }
    else if (vAvg >= 0.0f)              { st->gear = CLUSTER_GEAR_DRIVE;   }
    else                                { st->gear = CLUSTER_GEAR_REVERSE; }

    ClusterComm_UpdateTrip();
    st->trip_m = (uint16)(cluster_tripMetres + 0.5f);

    /* Lifetime odometer, bytes 5-7. Already whole metres and already saturated
     * at the 24-bit signal maximum by the service, so no scaling or clamping is
     * repeated here - odo_cfg.h's ODO_MAX_M IS the DBC signal's maximum. */
    st->odo_m = Odo_GetMetres();
}

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

void ClusterComm_Init(void)
{
    /* No private hardware/state yet. Encoder and CAN are initialised by their
     * own drivers in main(). Kept for API symmetry and future per-frame
     * scheduling state (BatteryStatus / PowerStatus). */
}

/*----------------------------------------------------------------------------
 * BatteryStatus (0x210). Fill the DBC struct from ONE BatteryService snapshot -
 * never from several getters, because `power` is voltage x current and two
 * getters could pair values from different instants (S10-1 / V9-R1).
 *
 * ✅ RESOLVED 2026-08-05 (DBC v2, the coordinated two-node regen).
 * Power was scaled at 0.1 kW = 100 W/LSB, so this robot's whole 3-220 W range
 * mapped to 0-2 counts - the signal was blind, and A3 step 4a confirmed it on
 * the wire (4.06 W transmitted as 0.0 kW). The DBC is a TWO-NODE CONTRACT, so
 * it was not changed unilaterally then; it has now been changed on BOTH sides
 * together. Power is 1 W/LSB and range is 1 m/LSB, so the code below passes
 * watts and metres STRAIGHT THROUGH - the /1000.0 that used to throw the value
 * away is gone.
 *
 * ⚠️ The cluster MUST be running the regenerated DBC. With a v1 decoder against
 * v2 frames every field is wrong - not just rescaled but MISALIGNED, because
 * `voltage` narrowed to one byte and everything after it shifted down.
 *--------------------------------------------------------------------------*/
static void ClusterComm_BuildBatteryStatus(struct robot_battery_status_t *st,
                                           const BatteryStatusType *s)
{
    /* The generated *_encode() helpers own the scaling; doing the division here
     * would duplicate the DBC and silently diverge from it at the next regen. */
    st->voltage = robot_battery_status_voltage_encode((double)s->voltage_mV / 1000.0);
    st->current = robot_battery_status_current_encode((double)s->current_mA / 1000.0);
    st->soc     = robot_battery_status_soc_encode((double)s->soc_pct);
    st->power   = robot_battery_status_power_encode((double)s->power_W);   /* W, 1 LSB = 1 W */
    st->range   = robot_battery_status_range_encode((double)s->range_m);   /* m, 1 LSB = 1 m */
}

Std_ReturnType ClusterComm_SendBatteryStatus(void)
{
    struct robot_battery_status_t st;
    BatteryStatusType             s;
    uint8 data[ROBOT_BATTERY_STATUS_LENGTH];
    int   packed;

    if (BatteryService_GetStatus(&s) != E_OK)
    {
        return E_NOT_OK;
    }

    ClusterComm_BuildBatteryStatus(&st, &s);

    packed = robot_battery_status_pack(data, &st, (size_t)sizeof(data));
    if (packed < 0)
    {
        return E_NOT_OK;
    }

    if (CanTxQueue_Post(ROBOT_BATTERY_STATUS_FRAME_ID, data, (uint8)packed) != CAN_OK)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

Std_ReturnType ClusterComm_SendVehicleStatus(void)
{
    struct robot_vehicle_status_t st;
    uint8 data[ROBOT_VEHICLE_STATUS_LENGTH];
    int   packed;

    ClusterComm_BuildVehicleStatus(&st);

    packed = robot_vehicle_status_pack(data, &st, (size_t)sizeof(data));
    if (packed < 0)
    {
        return E_NOT_OK;   /* serialization failed (buffer too small) */
    }

    if (CanTxQueue_Post(ROBOT_VEHICLE_STATUS_FRAME_ID, data, (uint8)packed) != CAN_OK)
    {
        return E_NOT_OK;   /* TX_BUSY / INVALID_DLC -> caller may retry next tick */
    }

    return E_OK;
}

void ClusterComm_MainFunction(void)
{
    (void)ClusterComm_SendVehicleStatus();
    /* NOTE: BatteryStatus is deliberately NOT dispatched from here. 0x210 has
     * its own phased slot in the super-loop (DESIGN §6.4: update at
     * now%100==43, transmit at ==47) to preserve the one-Can_Transmit-per-ms
     * invariant. Sending it from this runnable would put two transmits in the
     * same millisecond. Wiring is main.c's job - A3 step 4. */
}
