/******************************************************************************
 *
 * Module: CommData (ROS comms data-prep boundary)
 *
 * File Name: comm_data_cfg.h
 *
 * Description: Compile-time configuration constants for the comm_data
 *              boundary layer. Currently only the rad/s <-> RPM scale
 *              factors live here. NO transport configuration (no CAN IDs,
 *              no UART baud, no framing) belongs in this file - this layer
 *              is data-preparation only.
 *
 ******************************************************************************/

#ifndef COMM_DATA_CFG_H_
#define COMM_DATA_CFG_H_

/*******************************************************************************
 *                       rad/s <-> RPM scale factors                           *
 *
 * Defined ONCE here, with derivation, so both directions share a single
 * source of truth and the f-suffix literal style matches the rest of the
 * codebase.
 *
 * RPM    = rad/s * (1 rev / 2*pi rad) * (60 s / 1 min)
 *        = rad/s * 60 / (2*pi)
 *        = rad/s * 9.549296585513720...   (60 / 6.283185307179586)
 *
 * rad/s  = RPM   * (2*pi rad / 1 rev) * (1 min / 60 s)
 *        = RPM   * 2*pi / 60
 *        = RPM   * 0.104719755119660...   (6.283185307179586 / 60)
 *
 * The two factors are exact reciprocals; both are stored explicitly so
 * neither direction pays a runtime divide.
 ******************************************************************************/

/* rad/s -> RPM : multiply by 60/(2*pi) */
#define COMM_DATA_RADPS_TO_RPM      (9.549296585513720f)

/* RPM -> rad/s : multiply by 2*pi/60 (inverse of the above) */
#define COMM_DATA_RPM_TO_RADPS      (0.104719755119660f)

#endif /* COMM_DATA_CFG_H_ */
