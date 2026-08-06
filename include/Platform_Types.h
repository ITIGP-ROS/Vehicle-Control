/******************************************************************************
 *
 * Module: Platform Types
 *
 * File Name: Platform_Types.h
 *
 * Description: Platform types for ARM Cortex-M4 TM4C123GH6PM
 *
 *******************************************************************************/

#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

/*******************************************************************************
 *                              Boolean Types                                  *
 *******************************************************************************/

#ifndef FALSE
#define FALSE       (0U)
#endif

#ifndef TRUE
#define TRUE        (1U)
#endif

#ifndef NULL
#define NULL        ((void *)0)
#endif

/*******************************************************************************
 *                              Integer Types                                  *
 *******************************************************************************/

typedef unsigned char       uint8;
typedef signed char         sint8;
typedef unsigned short      uint16;
typedef signed short        sint16;
typedef unsigned long       uint32;
typedef signed long         sint32;
typedef unsigned long long  uint64;
typedef signed long long    sint64;

typedef unsigned char       boolean;

/*******************************************************************************
 *                              Float Types                                    *
 *******************************************************************************/

typedef float               float32;
typedef double              float64;

#endif /* PLATFORM_TYPES_H */
