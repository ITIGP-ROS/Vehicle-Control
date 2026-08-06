#ifndef INA226_PRIVATE_H_
#define INA226_PRIVATE_H_

/******************************************************************************
 *
 * Module: INA226 (MCAL)
 *
 * File Name: ina226_private.h
 *
 * Description: Internals shared only within the INA226 module - the register
 *              map and the 16-bit register accessors.
 *
 *              NOT a public header. Application and service code includes
 *              ina226.h only; nothing outside this module has any business
 *              touching a register number.
 *
 ******************************************************************************/

#include "Platform_Types.h"
#include "ina226_types.h"

/*******************************************************************************
 *                          Register map (datasheet)                           *
 *******************************************************************************/

#define INA226_REG_CONFIG       (0x00U)  /* 16-bit configuration                */
#define INA226_REG_SHUNT_V      (0x01U)  /* SIGNED int16, LSB 2.5 uV            */
#define INA226_REG_BUS_V        (0x02U)  /* UNSIGNED,     LSB 1.25 mV           */
#define INA226_REG_POWER        (0x03U)  /* NOT USED - CAL-dependent            */
#define INA226_REG_CURRENT      (0x04U)  /* NOT USED - CAL-dependent            */
#define INA226_REG_CALIB        (0x05U)  /* 16-bit calibration                  */
#define INA226_REG_MASK_EN      (0x06U)  /* alert/flags; READING IT CLEARS CVRF  */
#define INA226_REG_MANUF_ID     (0xFEU)  /* expect 0x5449                       */

/* MASK_EN bits. Read in Init only - never in the hot path (see ina226.h). */
#define INA226_MASK_CVRF        (0x0008U)  /* bit 3: conversion ready           */
#define INA226_MASK_OVF         (0x0004U)  /* bit 2: CURRENT/POWER math overflow */

/* CONFIG bit 15: device is self-resetting if this reads back set. */
#define INA226_CONFIG_RST_BIT   (0x8000U)

/*******************************************************************************
 *                          Register access                                    *
 *                                                                             *
 * WIRE FORMAT: INA226 registers are 16-bit BIG-ENDIAN (MSB first) and the I2C  *
 * driver performs NO byte swapping, so these two pack/unpack explicitly. Both  *
 * were proven on hardware in main_ina226_calib.c before being lifted here.     *
 *******************************************************************************/

/**
 * @brief  Write a 16-bit register, MSB first.
 * @return I2C_OK, or the transport error (also cached for GetLastI2cError).
 */
Ina226_I2cErrorType Ina226_WriteReg16(uint8 reg, uint16 val);

/**
 * @brief  Read a 16-bit register into *out, MSB first.
 * @note   *out is left UNTOUCHED unless the read returns I2C_OK - a failed read
 *         never fabricates a value.
 */
Ina226_I2cErrorType Ina226_ReadReg16(uint8 reg, uint16 *out);

#endif /* INA226_PRIVATE_H_ */
