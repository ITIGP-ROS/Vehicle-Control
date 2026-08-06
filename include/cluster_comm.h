/******************************************************************************
 *
 * Module: ClusterComm
 *
 * File Name: cluster_comm.h
 *
 * Description: Application-layer module for ALL Tiva -> instrument-cluster
 *              display CAN traffic. Consumes the Encoder HAL, converts raw
 *              data to display form, serializes via the cantools-generated
 *              packers (robot.h), and hands raw bytes to the CAN MCAL (can.h).
 *
 *              Currently handles ONLY VehicleStatus (0x200). BatteryStatus /
 *              PowerStatus will slot into ClusterComm_MainFunction() later,
 *              following the same build -> pack -> transmit pattern.
 *
 *              Layer: application. Includes encoder.h + robot.h + can.h.
 *              NEVER touches hardware registers directly (that is MCAL's job).
 *
 ******************************************************************************/

#ifndef CLUSTER_COMM_H_
#define CLUSTER_COMM_H_

#include "Std_Types.h"

/**
 * @brief  Initialize the cluster-comm module.
 * @note   No private hardware today (Encoder and CAN are initialised by their
 *         own drivers in main()). Present for API symmetry and future
 *         per-frame scheduling state.
 */
void ClusterComm_Init(void);

/**
 * @brief  Build, serialize and transmit one VehicleStatus (0x200) frame from
 *         the current encoder state.
 * @return E_OK if the frame was staged on the CAN controller, else E_NOT_OK.
 */
Std_ReturnType ClusterComm_SendVehicleStatus(void);

/**
 * @brief  Build, serialize and transmit one BatteryStatus (0x210) frame from
 *         the current BatteryService snapshot.
 * @return E_OK if the frame was staged on the CAN controller, else E_NOT_OK.
 * @note   Call from its OWN super-loop slot, not from ClusterComm_MainFunction:
 *         0x210 is phased separately (DESIGN §6.4) so that at most one
 *         Can_Transmit happens per millisecond.
 * @note   The DBC's power signal is 0.1 kW/LSB, which cannot resolve this
 *         vehicle (3-220 W -> 0-2 counts). The agreed 1 W/LSB change is a
 *         two-node contract change pending cluster coordination; this packs per
 *         the current DBC until then. See cluster_comm.c.
 */
Std_ReturnType ClusterComm_SendBatteryStatus(void);

/**
 * @brief  Periodic runnable for all cluster traffic. Call at the cluster
 *         refresh rate (see CLUSTER_COMM_PERIOD_MS in main). Today it sends
 *         VehicleStatus; future display frames are dispatched from here.
 */
/**
 * @brief  Zero the TRIP distance and drop its origin (0x140 ResetCommand).
 * @note   The caller MUST have just zeroed the encoder counters
 *         (Encoder_ResetAllDistances) - this re-seeds against the new reading.
 * @note   The LIFETIME ODOMETER IS NOT AFFECTED. TRIP is reset-relative; ODO is
 *         permanent and has no CAN path that can clear it, by design.
 */
void ClusterComm_ResetTrip(void);

void ClusterComm_MainFunction(void);

#endif /* CLUSTER_COMM_H_ */
