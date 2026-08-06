 /******************************************************************************
 *
 * Module: Common - Macros
 *
 * File Name: Common_Macros.h
 *
 * Description: Commonly used Macros
 *
 * Author: Mohamed Tarek
 *
 *******************************************************************************/

#ifndef Common_Macros_H
#define Common_Macros_H

/* NOTE (T5-2): every argument below is parenthesised, and the shifted 1 is
 * UNSIGNED (1UL). Both matter:
 *   - Unparenthesised args mis-parse a compound argument. GET_BIT(r, a+1) used
 *     to expand to  ( r & (1<<(a+1)) ) >> a + 1 , and since + binds tighter
 *     than >> that evaluated as ((...) >> a) + 1 - silently wrong. SET_BIT and
 *     friends survived the same argument only because + also binds tighter
 *     than <<, which was luck rather than design.
 *   - 1<<BIT is a signed int, so 1<<31 was undefined behaviour. 1UL<<(BIT) is
 *     unsigned and well-defined across the whole 32-bit register width.
 * Behaviour is unchanged for every existing caller (all pass a simple lvalue);
 * verified by byte-identical disassembly of PORT.o, the only user. */

/* Set a certain bit in any register */
#define SET_BIT(REG,BIT) ((REG)|=(1UL<<(BIT)))

/* Clear a certain bit in any register */
#define CLEAR_BIT(REG,BIT) ((REG)&=(~(1UL<<(BIT))))

/* Toggle a certain bit in any register */
#define TOGGLE_BIT(REG,BIT) ((REG)^=(1UL<<(BIT)))

/* Rotate right the register value with specific number of rotates */
#define ROR(REG,num) ( (REG) = ((REG)>>(num)) | ((REG) << ((sizeof(REG) * 8)-(num))) )

/* Rotate left the register value with specific number of rotates */
#define ROL(REG,num) ( (REG) = ((REG)<<(num)) | ((REG) >> ((sizeof(REG) * 8)-(num))) )

/* Check if a specific bit is set in any register and return true if yes */
#define BIT_IS_SET(REG,BIT) ( (REG) & (1UL<<(BIT)) )

/* Check if a specific bit is cleared in any register and return true if yes */
#define BIT_IS_CLEAR(REG,BIT) ( !((REG) & (1UL<<(BIT))) )

/* Macro to get value of a specific bit */
#define GET_BIT(REG,BIT) ( ( (REG) & (1UL<<(BIT)) ) >> (BIT) )

#endif
