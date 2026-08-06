 /******************************************************************************
	 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: ABDELRAHMAN MOHAEMED
 *
 *******************************************************************************/
#ifndef Port_Cfg_H
#define Port_Cfg_H
/*******************************************************************************
 *                                Version Definitions                          *
 *******************************************************************************/
/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Software Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION           (1U)
#define PORT_CFG_SW_MINOR_VERSION           (0U)
#define PORT_CFG_SW_PATCH_VERSION           (0U)

/*******************************************************************************
 *                                Pre-compile Options                          *
 *******************************************************************************/
 /* 
	$ Options :-
	
		1-  (STD_ON)    // ENABLE  //
		2-  (STD_OFF)  // DISABLE //
		
*/
/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Enable/Disable the Port_SetPinDirection service */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/* Pre-compile option for Enable/Disable the Port_SetPinMode service */
#define PORT_SET_MODE_API                    (STD_ON)
   
/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)          
/*******************************************************************************
 *                               MCU Ports Definitions                         *
 *******************************************************************************/
/* Ports Number */
#define PORT_PORTS_NUM          (6U)
 
/* PORTS symbolic names */
#define PORT_A                  (0U)
#define PORT_B                  (1U)
#define PORT_C                  (2U)
#define PORT_D                  (3U)
#define PORT_E                  (4U)
#define PORT_F                  (5U) 
 
/*******************************************************************************
 *                               MCU Pins Definitions                          *
 *******************************************************************************/
 /* Pins number */
#define PORT_PINS_NUM           (43U)
   
/* Pins within a port */
#define PIN_0                   (0U)
#define PIN_1                   (1U)
#define PIN_2                   (2U)
#define PIN_3                   (3U)   
#define PIN_4                   (4U)
#define PIN_5                   (5U)
#define PIN_6                   (6U)
#define PIN_7                   (7U)
   
/* Port A pins symbolic names*/
#define PORT_A_PIN_0            (0U)
#define PORT_A_PIN_1            (1U)
#define PORT_A_PIN_2            (2U)
#define PORT_A_PIN_3            (3U)
#define PORT_A_PIN_4            (4U)
#define PORT_A_PIN_5            (5U)
#define PORT_A_PIN_6            (6U)
#define PORT_A_PIN_7            (7U)

/* Port B pins symbolic names*/
#define PORT_B_PIN_0            (8U)
#define PORT_B_PIN_1            (9U)
#define PORT_B_PIN_2            (10U)
#define PORT_B_PIN_3            (11U)
#define PORT_B_PIN_4            (12U)
#define PORT_B_PIN_5            (13U)
#define PORT_B_PIN_6            (14U)
#define PORT_B_PIN_7            (15U)
   
/* Port C pins symbolic names*/
#define PORT_C_PIN_0            (16U)
#define PORT_C_PIN_1            (17U)
#define PORT_C_PIN_2            (18U)
#define PORT_C_PIN_3            (19U)
#define PORT_C_PIN_4            (20U)
#define PORT_C_PIN_5            (21U)
#define PORT_C_PIN_6            (22U)
#define PORT_C_PIN_7            (23U)
   
/* Port D pins symbolic names*/
#define PORT_D_PIN_0            (24U)
#define PORT_D_PIN_1            (25U)
#define PORT_D_PIN_2            (26U)
#define PORT_D_PIN_3            (27U)
#define PORT_D_PIN_4            (28U)
#define PORT_D_PIN_5            (29U)
#define PORT_D_PIN_6            (30U)
#define PORT_D_PIN_7            (31U)

/* Port E pins symbolic names*/
#define PORT_E_PIN_0            (32U)
#define PORT_E_PIN_1            (33U)
#define PORT_E_PIN_2            (34U)
#define PORT_E_PIN_3            (35U)
#define PORT_E_PIN_4            (36U)
#define PORT_E_PIN_5            (37U)

/* Port F pins symbolic names*/
#define PORT_F_PIN_0            (38U)
#define PORT_F_PIN_1            (39U)
#define PORT_F_PIN_2            (40U)
#define PORT_F_PIN_3            (41U)
#define PORT_F_PIN_4            (42U)
 
 
/*******************************************************************************
 *                               MCU Modes Definitions                         *
 *******************************************************************************/
/* Maximum Mode Number */
#define MAX_MODE_NUMBER                 (16U)

/* DIGITAL I/O MODE */
#define PORT_DIGITAL_IO                 (0U)

/* ANALOG MODE */
#define PORT_ANALOG                     (16U)

/* Mode Mask */
#define PIN_MODE_MASK       0x0000000F
#define SHIFT_4                 (4U)
						
/* PA0 Modes */           
#define PORT_PA0_U0RX_MODE        (1U)                    
#define PORT_PA0_CAN1Rx_MODE      (8U)
						
/* PA1 Modes */           
#define PORT_PA1_U0Tx_MODE        (1U)           
#define PORT_PA1_CAN1Tx_MODE      (8U)
						
/* PA2 Modes */           
#define PORT_PA2_SSI0Clk_MODE      (2U)
						
/* PA3 Modes */           
#define PORT_PA3_SSI0Fss_MODE      (2U)
						
/* PA4 Modes */           
#define PORT_PA4_SSI0Rx_MODE      (2U)
						
/* PA5 Modes */          
#define PORT_PA5_SSI0Tx_MODE      (2U)
						
/* PA6 Modes */           
#define PORT_PA6_I2C1SCL_MODE      (3U)
#define PORT_PA6_M1PWM2_MODE      (5U)
						
/* PA7 Modes */           
#define PORT_PA7_I2C1SDA_MODE      (3U)
#define PORT_PA7_M1PWM3_MODE      (5U)
					
/* PB0 Modes */          
#define PORT_PB0_U1Rx_MODE        (1U)
#define PORT_PB0_T2CCP0_MODE      (7U)
						
/* PB1 Modes */          
#define PORT_PB1_U1Tx_MODE        (1U)
#define PORT_PB1_T2CCP1_MODE      (7U)
						
/* PB2 Modes */           
#define PORT_PB2_I2C0SCL_MODE      (3U)
#define PORT_PB2_T3CCP0_MODE      (7U)
						
/* PB3 Modes */           
#define PORT_PB3_I2C0SDA_MODE      (3U)
#define PORT_PB3_T3CCP1_MODE      (7U)
						
/* PB4 Modes */           
#define PORT_PB4_SSI2Clk_MODE      (2U)
#define PORT_PB4_M0PWM2_MODE      (4U)
#define PORT_PB4_T1CCP0_MODE      (7U)
#define PORT_PB4_CAN0Rx_MODE      (8U)
						
/* PB5 Modes */           
#define PORT_PB5_SSI2Fss_MODE      (2U)
#define PORT_PB5_M0PWM3_MODE      (4U)
#define PORT_PB5_T1CCP1_MODE      (7U)
#define PORT_PB5_CAN0Tx_MODE      (8U)
						
/* PB6 Modes */           
#define PORT_PB6_SSI2Rx_MODE      (2U)
#define PORT_PB6_M0PWM0_MODE      (4U)
#define PORT_PB6_T0CCP0_MODE      (7U)
						
/* PB7 Modes */           
#define PORT_PB7_SSI2Tx_MODE      (2U)
#define PORT_PB7_M0PWM1_MODE      (4U)
#define PORT_PB7_T0CCP1_MODE      (7U)
						
/* PC0 Modes */           
#define PORT_PC0_TCK_SWCL_MODE      (1U)
#define PORT_PC0_T4CCP0_MODE         (7U)
						
/* PC1 Modes */           
#define PORT_PC1_TMS_SWDI_MODE      (1U)
#define PORT_PC1_T4CCP1_MODE        (7U)
						
/* PC2 Modes */           
#define PORT_PC2_TDI_MODE         (1U)
#define PORT_PC2_T5CCP0_MODE      (7U)
						
/* PC3 Modes */           
#define PORT_PC3_TDO_SWO_MODE      (1U)
#define PORT_PC3_T5CCP1_MODE      (7U)
						
/* PC4 Modes */          
#define PORT_PC4_U4Rx_MODE         (1U)
#define PORT_PC4_U1Rx_MODE         (2U)
#define PORT_PC4_M0PWM6_MODE       (4U)
#define PORT_PC4_IDX1_MODE         (6U)
#define PORT_PC4_WT0CCP0_MODE      (7U)
#define PORT_PC4_U1RTS  _MODE      (8U)
						
/* PC5 Modes */           
#define PORT_PC5_U4Tx_MODE         (1U)
#define PORT_PC5_U1Tx_MODE         (2U)
#define PORT_PC5_M0PWM7_MODE      (4U)
#define PORT_PC5_PhA1_MODE         (6U)
#define PORT_PC5_WT0CCP1_MODE      (7U)
#define PORT_PC5_U1CTS_MODE        (8U)
						
/* PC6 Modes */           
#define PORT_PC6_U3Rx_MODE          (1U)
#define PORT_PC6_PhB1_MODE          (6U)
#define PORT_PC6_WT1CCP0_MODE       (7U)
#define PORT_PC6_USB0EPEN_MODE      (8U)
						
/* PC7 Modes */           
#define PORT_PC7_U3Tx_MODE          (1U)
#define PORT_PC7_WT1CCP1_MODE       (7U)
#define PORT_PC7_USB0PFLT_MODE      (8U)
						
/* PD0 Modes */           
#define PORT_PD0_SSI3Clk_MODE      (1U)
#define PORT_PD0_SSI1Clk_MODE      (2U)
#define PORT_PD0_I2C3SCL_MODE      (3U)
#define PORT_PD0_M0PWM6_MODE      (4U)
#define PORT_PD0_M1PWM0_MODE      (5U)
#define PORT_PD0_WT2CCP0_MODE      (7U)
						
/* PD1 Modes */           
#define PORT_PD1_SSI3Fss_MODE      (1U)
#define PORT_PD1_SSI1Fss_MODE      (2U)
#define PORT_PD1_I2C3SDA_MODE      (3U)
#define PORT_PD1_M0PWM7_MODE      (4U)
#define PORT_PD1_M1PWM1_MODE      (5U)
#define PORT_PD1_WT2CCP1_MODE      (7U)
						
/* PD2 Modes */           
#define PORT_PD2_SSI3Rx_MODE        (1U)
#define PORT_PD2_SSI1Rx_MODE        (2U)
#define PORT_PD2_M0FAULT0_MODE      (4U)
#define PORT_PD2_WT3CCP0_MODE       (7U)
#define PORT_PD2_USB0EPEN_MODE      (8U)
						
/* PD3 Modes */           
#define PORT_PD3_SSI3Tx_MODE       (1U)
#define PORT_PD3_SSI1Tx_MODE       (2U)
#define PORT_PD3_IDX0_MODE         (6U)
#define PORT_PD3_WT3CCP1_MODE      (7U)
#define PORT_PD3_USB0PFLT_MODE     (8U)
						
/* PD4 Modes */           
#define PORT_PD4_U6Rx_MODE         (1U)
#define PORT_PD4_WT4CCP0_MODE      (7U)
						
/* PD5 Modes */           
#define PORT_PD5_U6Tx_MODE         (1U)
#define PORT_PD5_WT4CCP1_MODE      (7U)
						
/* PD6 Modes */           
#define PORT_PD6_U2Rx_MODE          (1U)
#define PORT_PD6_M0FAULT0_MODE      (4U)
#define PORT_PD6_PhA0_MODE          (6U)
#define PORT_PD6_WT5CCP0_MODE       (7U)
						
/* PD7 Modes */           
#define PORT_PD7_U2Tx_MODE         (1U)
#define PORT_PD7_PhB0_MODE         (6U)
#define PORT_PD7_WT5CCP1_MODE      (7U)
#define PORT_PD7_NMI_MODE          (8U)
						
/* PE0 Modes */           
#define PORT_PE0_U7Rx_MODE      (1U)
					
/* PE1 Modes */      
#define PORT_PE1_U7Tx_MODE      (1U)
						
/* PE4 Modes */           
#define PORT_PE4_U5Rx_MODE        (1U)
#define PORT_PE4_I2C2SC_MODE      (3U)
#define PORT_PE4_M0PWM4_MODE      (4U)
#define PORT_PE4_M1PWM2_MODE      (5U)
#define PORT_PE4_CAN0Rx_MODE      (8U)
						
/* PE5 Modes */           
#define PORT_PE5_U5Tx_MODE        (1U)
#define PORT_PE5_I2C2SDA_MODE     (3U)
#define PORT_PE5_M0PWM5_MODE      (4U)
#define PORT_PE5_M1PWM3_MODE      (5U)
#define PORT_PE5_CAN0Tx_MODE      (8U)
				
/* PF0 Modes */           
#define PORT_PF0_U1RTS_MODE       (1U)
#define PORT_PF0_SSI1Rx_MODE      (2U)
#define PORT_PF0_CAN0Rx_MODE      (3U)
#define PORT_PF0_M1PWM4_MODE      (5U)
#define PORT_PF0_PhA0_MODE        (6U)
#define PORT_PF0_T0CCP0_MODE      (7U)
#define PORT_PF0_NMI_MODE         (8U)
#define PORT_PF0_C0o_MODE         (9U)
					
/* PF1 Modes */           
#define PORT_PF1_U1CTS_MODE         (1U)
#define PORT_PF1_SSI1Tx_MODE        (2U)
#define PORT_PF1_M1PWM5_MODE        (5U)
#define PORT_PF1_PhB0_MODE          (6U)
#define PORT_PF1_T0CCP1_MODE        (7U)
#define PORT_PF1_C1o_MODE           (9U)
#define PORT_PF1_TRD1_MODE          (14U)
						
/* PF2 Modes */           
#define PORT_PF2_SSI1Clk_MODE       (2U)
#define PORT_PF2_M0FAULT0_MODE      (4U)
#define PORT_PF2_M1PWM6_MODE        (5U)
#define PORT_PF2_T1CCP0_MODE        (7U) 
#define PORT_PF2_TRD0_MODE          (14U)
						
/* PF3 Modes */           
#define PORT_PF3_SSI1Fss_MODE       (2U)
#define PORT_PF3_CAN0Tx_MODE        (3U)
#define PORT_PF3_M1PWM7_MODE        (5U)
#define PORT_PF3_T1CCP1_MODE        (7U)
#define PORT_PF3_TRCLK_MODE         (14U)
						
/* PF4 Modes */           
#define PORT_PF4_M1FAULT0_MODE      (5U)
#define PORT_PF4_IDX0_MODE          (6U)
#define PORT_PF4_T2CCP0_MODE        (7U)
#define PORT_PF4_USB0EPEN_MODE      (8U)

#endif /* Port_Cfg_H  */
