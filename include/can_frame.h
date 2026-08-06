/******************************************************************************
 *
 * Module: CAN (MCAL)
 *
 * File Name: can_frame.h
 *
 * Description: One raw Classic-CAN frame as carried by the MCAL transport
 *              layer. No meaning - just id/dlc/bytes. Stored by value in the
 *              RX ring buffer (RING_BUFFER.*) and produced/consumed by can.c.
 *              Kept in this small neutral header so the ring buffer stays
 *              decoupled from can_private.h (register-only, can.c-only).
 *
 ******************************************************************************/

#ifndef CAN_FRAME_H_
#define CAN_FRAME_H_

#include "Platform_Types.h"

typedef struct {
    uint32 id;        /* 11-bit standard identifier (0..0x7FF)        */
    uint8  dlc;       /* data length 0..8                              */
    uint8  data[8];   /* payload; byte 0 is first on the bus           */
} Can_FrameType;

#endif /* CAN_FRAME_H_ */
