/******************************************************************************
 *
 * Module: SysTick
 *
 * File Name: systick_private.h
 *
 * Description: Private definitions for TM4C123GH6PM SysTick driver
 *              Contains register addresses and bit definitions
 *
 ******************************************************************************/

#ifndef SYSTICK_PRIVATE_H_
#define SYSTICK_PRIVATE_H_

#include "Std_Types.h"

/*******************************************************************************
 *                          SysTick Register Addresses                         *
 *******************************************************************************/

#define SYSTICK_BASE_ADDR               (0xE000E000UL)

#define SYSTICK_CTRL_REG                (*((volatile uint32 *)(SYSTICK_BASE_ADDR + 0x010)))
#define SYSTICK_RELOAD_REG              (*((volatile uint32 *)(SYSTICK_BASE_ADDR + 0x014)))
#define SYSTICK_CURRENT_REG             (*((volatile uint32 *)(SYSTICK_BASE_ADDR + 0x018)))

/*******************************************************************************
 *                          SysTick Control Register Bits                      *
 *******************************************************************************/

/* Bit 0: ENABLE - SysTick enable */
#define SYSTICK_CTRL_ENABLE_POS         (0U)
#define SYSTICK_CTRL_ENABLE_MASK        (1UL << SYSTICK_CTRL_ENABLE_POS)

/* Bit 1: INTEN - Interrupt enable */
#define SYSTICK_CTRL_INTEN_POS          (1U)
#define SYSTICK_CTRL_INTEN_MASK         (1UL << SYSTICK_CTRL_INTEN_POS)

/* Bit 2: CLK_SRC - Clock source (0=PIOSC/4, 1=System clock) */
#define SYSTICK_CTRL_CLKSRC_POS         (2U)
#define SYSTICK_CTRL_CLKSRC_MASK        (1UL << SYSTICK_CTRL_CLKSRC_POS)

/* Bit 16: COUNT - Count flag (read-only, cleared on read) */
#define SYSTICK_CTRL_COUNT_POS          (16U)
#define SYSTICK_CTRL_COUNT_MASK         (1UL << SYSTICK_CTRL_COUNT_POS)

/*******************************************************************************
 *                          Interrupt Control Macros                           *
 *******************************************************************************/

/* Enable global interrupts */
#define SYSTICK_ENABLE_INTERRUPTS()     __asm(" CPSIE I ")

/* Disable global interrupts */
#define SYSTICK_DISABLE_INTERRUPTS()    __asm(" CPSID I ")

/*******************************************************************************
 *                          Helper Macros                                      *
 *******************************************************************************/

/* Check if reload value is valid (24-bit max) */
#define SYSTICK_IS_VALID_RELOAD(val)    ((val) <= 0x00FFFFFFUL)

#endif /* SYSTICK_PRIVATE_H_ */
