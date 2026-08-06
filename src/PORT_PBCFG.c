 /******************************************************************************
 *
 * Module: Port 
 * File Name: Port_PBcfg.c
 *
 * Description: Source file for Port Post Build configurations
 *              on TM4C123GH6PM Microcontroller
 *
 * Author: ABDELRAHMAN MOHAEMED
 *
 *******************************************************************************/

/*******************************************************************************
 *                                Version Definitions                          *
 *******************************************************************************/
/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Software Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION           (1U)
#define PORT_PBCFG_SW_MINOR_VERSION           (0U)
#define PORT_PBCFG_SW_PATCH_VERSION           (0U)

/*******************************************************************************
 *                                   INCLUDES                                  *
 *******************************************************************************/
/* AUTOSAR Port header file */
#include "PORT.h"

/* Checking AUTOSAR Release compitability between Port.h and Port_Cfg.h */
#if ((PORT_AR_RELEASE_MAJOR_VERSION != PORT_CFG_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_AR_RELEASE_MINOR_VERSION != PORT_CFG_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_AR_RELEASE_PATCH_VERSION != PORT_CFG_AR_RELEASE_PATCH_VERSION))
      #error "The AR version of Port.h does not match the expected version"
#endif

/* Checking Software compitability between Port.h and Port_Cfg.h */
#if ((PORT_SW_MAJOR_VERSION != PORT_CFG_SW_MAJOR_VERSION)\
 ||  (PORT_SW_MINOR_VERSION != PORT_CFG_SW_MINOR_VERSION)\
 ||  (PORT_SW_PATCH_VERSION != PORT_CFG_SW_PATCH_VERSION))
      #error "The AR version of Port.h does not match the expected version"
#endif


/*******************************************************************************
 *                           Initialization Structure                          *
 *******************************************************************************/
/* 
 * Note:
 *      The name of each pin is specified at first, then the following features
 *      are configured in the following order:
 *        1. Pin number
 *        2. Port number
 *        3. Pin mode
 *        4. Pin direction
 *        5. Pin initial value
 *        6. Internal pull-up/down resistor control                
 *        7. Pin direction changeable
 *        8. Pin mode changeable
 *
 */
const Port_ConfigType Port_Configuration =
{
 /* PORT_A_PIN_0 */
  PIN_0,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_1 */
  PIN_1,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_2 */
  PIN_2,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_3 */
  PIN_3,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_4 */
  PIN_4,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_5 */
  PIN_5,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_6 */
  PIN_6,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_A_PIN_7 */
  PIN_7,
  PORT_A,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
/* PORT_B_PIN_0 - UART1 RX from Pi */
  PIN_0,
  PORT_B,
  PORT_PB0_U1Rx_MODE,      /* <-- Change from PORT_DIGITAL_IO */
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
/* PORT_B_PIN_1 - UART1 TX to Pi */
  PIN_1,
  PORT_B,
  PORT_PB1_U1Tx_MODE,      /* <-- Change from PORT_DIGITAL_IO */
  PORT_PIN_OUT,            /* <-- Change from PORT_PIN_IN */
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_B_PIN_2 */
  PIN_2,
  PORT_B,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_B_PIN_3 */
  PIN_3,
  PORT_B,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_B_PIN_4 */
  PIN_4,
  PORT_B,
  PORT_PB4_M0PWM2_MODE,
  PORT_PIN_OUT,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_B_PIN_5 */
  PIN_5,
  PORT_B,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_B_PIN_6 */
  PIN_6,
  PORT_B,
  PORT_PB6_M0PWM0_MODE,
  PORT_PIN_OUT,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_B_PIN_7 */
  PIN_7,
  PORT_B,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_0 */
  PIN_0,
  PORT_C,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_1 */
  PIN_1,
  PORT_C,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_2 */
  PIN_2,
  PORT_C,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_3 */
  PIN_3,
  PORT_C,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_4 - WT0CCP0 (Wide Timer 0, TimerA) -> 50Hz servo PWM (timer_pwm.c MCAL) */
  PIN_4,
  PORT_C,
  PORT_PC4_WT0CCP0_MODE,   /* <-- alt-func mode 7, drives the servo CCP output */
  PORT_PIN_OUT,            /* <-- output, mirroring the PB4/PB6 motor-PWM pins  */
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_5 */
  PIN_5,
  PORT_C,
  PORT_PC5_PhA1_MODE,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_6 */
  PIN_6,
  PORT_C,
  PORT_PC6_PhB1_MODE,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_C_PIN_7 */
  PIN_7,
  PORT_C,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
 
 /* PORT_D_PIN_0 */
  PIN_0,
  PORT_D,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_1 */
  PIN_1,
  PORT_D,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_2 */
  PIN_2,
  PORT_D,
  PORT_DIGITAL_IO,
  PORT_PIN_OUT,
  STD_HIGH,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_3 */
  PIN_3,
  PORT_D,
  PORT_DIGITAL_IO,
  PORT_PIN_OUT,
  STD_HIGH,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_4 */
  PIN_4,
  PORT_D,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_5 */
  PIN_5,
  PORT_D,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_6 */
  PIN_6,
  PORT_D,
  PORT_PD6_PhA0_MODE ,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_D_PIN_7 */
  PIN_7,
  PORT_D,
  PORT_PD7_PhB0_MODE ,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_E_PIN_0 */
  PIN_0,
  PORT_E,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_E_PIN_1 */
  PIN_1,
  PORT_E,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_E_PIN_2 */
  PIN_2,
  PORT_E,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_E_PIN_3 - AIN0 -> steering-pot ADC feedback (adc.c). Analog mode:
  * Port_Init() clears DEN and sets AMSEL for this pin (see PORT.c analog branch). */
  PIN_3,
  PORT_E,
  PORT_ANALOG,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
  /* PORT_E_PIN_4 - CAN0Rx (alt-func mode 8) -> CAN MCAL (can.c) */
  PIN_4,
  PORT_E,
  PORT_PE4_CAN0Rx_MODE,    /* <-- alt-func mode 8, CAN0 receive */
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,

 /* PORT_E_PIN_5 - CAN0Tx (alt-func mode 8) -> CAN MCAL (can.c) */
  PIN_5,
  PORT_E,
  PORT_PE5_CAN0Tx_MODE,    /* <-- alt-func mode 8, CAN0 transmit */
  PORT_PIN_OUT,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_F_PIN_0 */
  PIN_0,
  PORT_F,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_F_PIN_1 */
  PIN_1,
  PORT_F,
  PORT_DIGITAL_IO,
  PORT_PIN_OUT,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_F_PIN_2 */
  PIN_2,
  PORT_F,
  PORT_DIGITAL_IO,
  PORT_PIN_OUT,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_F_PIN_3 */
  PIN_3,
  PORT_F,
  PORT_DIGITAL_IO,
  PORT_PIN_OUT,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF,
  
 /* PORT_F_PIN_4 */
  PIN_4,
  PORT_F,
  PORT_DIGITAL_IO,
  PORT_PIN_IN,
  STD_LOW,
  PORT_PIN_INTERNAL_RESISTOR_OFF,
  PIN_DIRECTION_CHANGEABLE_OFF,
  PIN_MODE_CHANGEABLE_OFF
};
