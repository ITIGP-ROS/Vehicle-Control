/******************************************************************************
 *
 * Module: NodePing (COMM)
 *
 * File Name: node_ping.c
 *
 * Description: Echo 0x7A0 NodePingRequest.seq back on 0x7A1 NodePingRespTiva.
 *              See node_ping.h for why this path answers UNCONDITIONALLY and is
 *              the one receive path exempt from the FIX 31 init gating.
 *
 * ⚠️ DEPENDENCY DISCIPLINE IS LOAD-BEARING HERE. The includes below are the
 * complete list and must stay that way: the CAN transport and the generated
 * frame definition, nothing else. No velocity, steering, battery, encoder,
 * servo or ADC header belongs in this file. That is what makes "a fault in any
 * service cannot suppress the liveness reply" a structural fact rather than a
 * promise - there is simply no service this code could fail on.
 *
 ******************************************************************************/

#include "node_ping.h"
#include "robot.h"     /* cantools: robot_node_ping_resp_tiva_pack(), 0x7A1 id */
#include "can.h"       /* MCAL transport (types)                               */
#include "can_tx_queue.h" /* B6: ALL transmits go through the TX queue now      */

/*******************************************************************************
 *                          Module State                                       *
 *
 * File-static, therefore BSS-zeroed: the reset state (no request pending, seq 0,
 * counters 0) is correct with no runtime initialiser, which is precisely why
 * this module needs no Init and therefore has no "not yet initialised" state to
 * gate on. Deliberate - see the header.
 *
 * NOT volatile, and that is checked rather than assumed: NodePing_OnRequest is
 * called from the CAN RX DISPATCH, which runs in main context (JetsonComm_Poll
 * pops the ring buffer that the ISR pushed into), and NodePing_MainFunction is
 * called from the super-loop. Both are main context, so there is no ISR race.
 * If a future change ever calls OnRequest from the CAN ISR itself, these two
 * become a genuine cross-context pair and would need re-examining together.
 *******************************************************************************/

static boolean g_Ping_Pending  = FALSE;   /* a request is waiting to be answered */
static uint8   g_Ping_Seq      = 0U;      /* the seq byte to echo back           */
static uint32  g_Ping_RequestCount  = 0U;
static uint32  g_Ping_ResponseCount = 0U;

/*******************************************************************************
 *                          Public Functions                                   *
 *******************************************************************************/

void NodePing_OnRequest(const uint8 *data, uint8 dlc)
{
    /* NO GATE OF ANY KIND. Not on an init flag, not on a service being healthy,
     * not on the DLC. The only precondition for answering is "a 0x7A0 arrived",
     * and that has already happened by the time we are called. */

    if ((data != NULL_PTR) && (dlc >= (uint8)ROBOT_NODE_PING_REQUEST_LENGTH))
    {
        g_Ping_Seq = data[0];
    }
    else
    {
        /* Malformed request - answer anyway, with seq 0. The node is
         * demonstrably alive (a frame reached us and we are executing), and
         * liveness is the ONLY thing this exchange reports. Staying silent
         * would assert "dead", which is a worse lie than echoing a zero. */
        g_Ping_Seq = 0U;
    }

    g_Ping_Pending = TRUE;

    if (g_Ping_RequestCount < 0xFFFFFFFFUL)
    {
        g_Ping_RequestCount++;
    }

    /* If a previous reply had not gone out yet, the newer seq REPLACES it: the
     * host wants to know we are alive NOW, and the freshest seq answers the
     * freshest question. Unreachable in practice - requests arrive at ~1 Hz and
     * the reply slot comes round every 100 ms, two orders of magnitude of
     * headroom - but defined rather than left to chance. */
}

Std_ReturnType NodePing_MainFunction(void)
{
    struct robot_node_ping_resp_tiva_t resp;
    uint8 data[ROBOT_NODE_PING_RESP_TIVA_LENGTH];
    int   packed;

    if (g_Ping_Pending == FALSE)
    {
        return E_NOT_OK;              /* nothing to answer - not an error */
    }

    resp.seq = g_Ping_Seq;            /* echo the LATCHED seq, byte-for-byte.
                                       * NOT a counter of our own: the host
                                       * matches the reply to the request that
                                       * caused it. */

    packed = robot_node_ping_resp_tiva_pack(data, &resp, (size_t)sizeof(data));
    if (packed < 0)
    {
        return E_NOT_OK;
    }

    if (CanTxQueue_Post(ROBOT_NODE_PING_RESP_TIVA_FRAME_ID, data, (uint8)packed) != CAN_OK)
    {
        /* TX_BUSY: leave g_Ping_Pending SET so the next slot retries. Dropping a
         * liveness reply would read as a missed heartbeat on the host side, and
         * at 1 Hz in / 10 Hz out there is ample room to retry. */
        return E_NOT_OK;
    }

    g_Ping_Pending = FALSE;           /* cleared only on a SUCCESSFUL transmit */

    if (g_Ping_ResponseCount < 0xFFFFFFFFUL)
    {
        g_Ping_ResponseCount++;
    }

    return E_OK;
}

uint32 NodePing_GetRequestCount(void)  { return g_Ping_RequestCount;  }
uint32 NodePing_GetResponseCount(void) { return g_Ping_ResponseCount; }
