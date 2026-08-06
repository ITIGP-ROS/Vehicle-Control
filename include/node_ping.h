/******************************************************************************
 *
 * Module: NodePing (COMM)
 *
 * File Name: node_ping.h
 *
 * Description: Node liveness ping. The HOST sends 0x7A0 NodePingRequest to all
 *              nodes ~1 Hz; this node echoes the seq byte back on its OWN
 *              response id, 0x7A1 NodePingRespTiva. That is the whole protocol.
 *
 *              0x7A2 (Jetson) and 0x7A3 (ESP32) are those nodes' responsibility
 *              and are NOT implemented here - they exist in the DBC only so the
 *              bus definition is complete.
 *
 * -----------------------------------------------------------------------------
 *  ⚠️ THIS MODULE ANSWERS UNCONDITIONALLY - AND THAT IS THE WHOLE POINT
 * -----------------------------------------------------------------------------
 *  The ping proves exactly one thing: THIS NODE IS POWERED AND ITS CAN STACK IS
 *  ALIVE. It does not claim the node is healthy, calibrated, or ready to drive.
 *
 *  So the reply must go out even when everything else is broken - a failed
 *  BatteryService_Init, a dead I2C bus, a faulted PID, services not yet
 *  initialised at all. A node that WITHHOLDS the reply because it considers
 *  itself "not ready" is indistinguishable on the bus from a node that has lost
 *  power, and the HOST reports it as DEAD. Silence is the one answer that means
 *  something else entirely, so it must never be given voluntarily.
 *
 *  🔶 THIS IS THE ONE RECEIVE PATH IN THE SYSTEM DELIBERATELY EXEMPT FROM THE
 *  INIT GATING ADDED IN FIX 31 (T25-1/2/3, V9-3, S10-3). Everywhere else, a
 *  pre-init call is rejected and counted; here, rejecting would defeat the
 *  purpose. The exemption is safe because the ping COMMANDS NOTHING - it reads
 *  no sensor, touches no actuator, and consults no service.
 *
 *  That property is STRUCTURAL, not a promise: this module includes only can.h
 *  and the generated robot.h. It has no dependency on velocity, steering,
 *  battery, encoder, servo or ADC, so a fault in any of them CANNOT suppress the
 *  liveness reply. Keep it that way - do not add a service include here.
 *
 *  There is deliberately NO NodePing_Init(). Nothing needs initialising, so
 *  there is nothing that could be "not yet initialised" to gate on; the
 *  file-static state is BSS-zeroed and correct from reset.
 * -----------------------------------------------------------------------------
 *
 *  NOT SecOC-PROTECTED, deliberately (unlike the OTA gate). The ping carries no
 *  command, and a forged response can only make a DEAD node look ALIVE - it
 *  cannot cause an action. Authentication would buy nothing.
 *
 ******************************************************************************/

#ifndef NODE_PING_H_
#define NODE_PING_H_

#include "Platform_Types.h"
#include "Std_Types.h"

/**
 * @brief  Handle a received 0x7A0 NodePingRequest: latch its seq and arm the
 *         reply. Called from the CAN RX dispatch, in main context.
 *
 * @param  data  frame payload (may be NULL_PTR only if dlc is 0)
 * @param  dlc   payload length, expected 1
 *
 * @note   DOES NOT TRANSMIT. The reply is sent later from
 *         NodePing_MainFunction() in its own super-loop slot, because
 *         transmitting the instant the request arrives could put two
 *         Can_Transmit calls in the same millisecond and break the
 *         one-transmit-per-ms invariant.
 * @note   A malformed request (dlc != 1) is still answered, with seq 0. The
 *         node is demonstrably alive either way, and liveness is the only thing
 *         this frame reports - refusing to answer would say "dead", which is a
 *         worse lie than echoing a zero.
 */
void NodePing_OnRequest(const uint8 *data, uint8 dlc);

/**
 * @brief  Transmit 0x7A1 NodePingRespTiva if a request is pending. Call from
 *         ONE dedicated super-loop slot that is mutually exclusive with every
 *         other Can_Transmit slot.
 *
 * @return E_OK      a reply was transmitted
 *         E_NOT_OK  nothing pending, or the CAN MCAL refused the frame
 *
 * @note   The pending flag is cleared only on a SUCCESSFUL transmit, so a
 *         TX_BUSY refusal retries on the next slot instead of dropping the
 *         reply. At 1 Hz of requests against a 10 Hz slot there is an order of
 *         magnitude of headroom for that.
 */
Std_ReturnType NodePing_MainFunction(void);

/** @brief Requests received (saturating). Non-zero proves the RX filter admits
 *         0x7A0 - i.e. that the dedicated message object works. */
uint32 NodePing_GetRequestCount(void);

/** @brief Replies successfully transmitted (saturating). Should track the
 *         request count; a gap means the MCAL is refusing transmits. */
uint32 NodePing_GetResponseCount(void);

#endif /* NODE_PING_H_ */
