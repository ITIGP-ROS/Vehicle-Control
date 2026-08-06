#ifndef TM4C123GH6PM_REGISTERS
#define TM4C123GH6PM_REGISTERS

#include "Std_Types.h"

/*****************************************************************************
GPIO registers (PORTA)
*****************************************************************************/
#define GPIO_PORTA_DATA_REG       (*((volatile uint32 *)0x400043FC))
#define GPIO_PORTA_DIR_REG        (*((volatile uint32 *)0x40004400))
#define GPIO_PORTA_AFSEL_REG      (*((volatile uint32 *)0x40004420))
#define GPIO_PORTA_PUR_REG        (*((volatile uint32 *)0x40004510))
#define GPIO_PORTA_PDR_REG        (*((volatile uint32 *)0x40004514))
#define GPIO_PORTA_DEN_REG        (*((volatile uint32 *)0x4000451C))
#define GPIO_PORTA_LOCK_REG       (*((volatile uint32 *)0x40004520))
#define GPIO_PORTA_CR_REG         (*((volatile uint32 *)0x40004524))
#define GPIO_PORTA_AMSEL_REG      (*((volatile uint32 *)0x40004528))
#define GPIO_PORTA_PCTL_REG       (*((volatile uint32 *)0x4000452C))
/* ODR added for I2C1SDA (PA7) open-drain config - GPIOODR offset 0x50C,
 * Table 10-6 GPIO Register Map, datasheet p.659 */
#define GPIO_PORTA_ODR_REG        (*((volatile uint32 *)0x4000450C))

/* PORTA External Interrupts Registers */
#define GPIO_PORTA_IS_REG         (*((volatile uint32 *)0x40004404))
#define GPIO_PORTA_IBE_REG        (*((volatile uint32 *)0x40004408))
#define GPIO_PORTA_IEV_REG        (*((volatile uint32 *)0x4000440C))
#define GPIO_PORTA_IM_REG         (*((volatile uint32 *)0x40004410))
#define GPIO_PORTA_RIS_REG        (*((volatile uint32 *)0x40004414))
#define GPIO_PORTA_ICR_REG        (*((volatile uint32 *)0x4000441C))

/*****************************************************************************
GPIO registers (PORTB)
*****************************************************************************/
#define GPIO_PORTB_DATA_REG       (*((volatile uint32 *)0x400053FC))
#define GPIO_PORTB_DIR_REG        (*((volatile uint32 *)0x40005400))
#define GPIO_PORTB_AFSEL_REG      (*((volatile uint32 *)0x40005420))
#define GPIO_PORTB_PUR_REG        (*((volatile uint32 *)0x40005510))
#define GPIO_PORTB_PDR_REG        (*((volatile uint32 *)0x40005514))
#define GPIO_PORTB_DEN_REG        (*((volatile uint32 *)0x4000551C))
#define GPIO_PORTB_LOCK_REG       (*((volatile uint32 *)0x40005520))
#define GPIO_PORTB_CR_REG         (*((volatile uint32 *)0x40005524))
#define GPIO_PORTB_AMSEL_REG      (*((volatile uint32 *)0x40005528))
#define GPIO_PORTB_PCTL_REG       (*((volatile uint32 *)0x4000552C))
/* ODR added for I2C0SDA (PB3) open-drain config - GPIOODR offset 0x50C,
 * Table 10-6 GPIO Register Map, datasheet p.659 */
#define GPIO_PORTB_ODR_REG        (*((volatile uint32 *)0x4000550C))

/* PORTB External Interrupts Registers */
#define GPIO_PORTB_IS_REG         (*((volatile uint32 *)0x40005404))
#define GPIO_PORTB_IBE_REG        (*((volatile uint32 *)0x40005408))
#define GPIO_PORTB_IEV_REG        (*((volatile uint32 *)0x4000540C))
#define GPIO_PORTB_IM_REG         (*((volatile uint32 *)0x40005410))
#define GPIO_PORTB_RIS_REG        (*((volatile uint32 *)0x40005414))
#define GPIO_PORTB_ICR_REG        (*((volatile uint32 *)0x4000541C))

/*****************************************************************************
GPIO registers (PORTC)
*****************************************************************************/
#define GPIO_PORTC_DATA_REG       (*((volatile uint32 *)0x400063FC))
#define GPIO_PORTC_DIR_REG        (*((volatile uint32 *)0x40006400))
#define GPIO_PORTC_AFSEL_REG      (*((volatile uint32 *)0x40006420))
#define GPIO_PORTC_PUR_REG        (*((volatile uint32 *)0x40006510))
#define GPIO_PORTC_PDR_REG        (*((volatile uint32 *)0x40006514))
#define GPIO_PORTC_DEN_REG        (*((volatile uint32 *)0x4000651C))
#define GPIO_PORTC_LOCK_REG       (*((volatile uint32 *)0x40006520))
#define GPIO_PORTC_CR_REG         (*((volatile uint32 *)0x40006524))
#define GPIO_PORTC_AMSEL_REG      (*((volatile uint32 *)0x40006528))
#define GPIO_PORTC_PCTL_REG       (*((volatile uint32 *)0x4000652C))
/* ODR added for symmetry with the other 5 ports (unused by I2C - PORTC has
 * no I2C pins) - GPIOODR offset 0x50C, Table 10-6, datasheet p.659 */
#define GPIO_PORTC_ODR_REG        (*((volatile uint32 *)0x4000650C))

/* PORTC External Interrupts Registers */
#define GPIO_PORTC_IS_REG         (*((volatile uint32 *)0x40006404))
#define GPIO_PORTC_IBE_REG        (*((volatile uint32 *)0x40006408))
#define GPIO_PORTC_IEV_REG        (*((volatile uint32 *)0x4000640C))
#define GPIO_PORTC_IM_REG         (*((volatile uint32 *)0x40006410))
#define GPIO_PORTC_RIS_REG        (*((volatile uint32 *)0x40006414))
#define GPIO_PORTC_ICR_REG        (*((volatile uint32 *)0x4000641C))

/*****************************************************************************
GPIO registers (PORTD)
*****************************************************************************/
#define GPIO_PORTD_DATA_REG       (*((volatile uint32 *)0x400073FC))
#define GPIO_PORTD_DIR_REG        (*((volatile uint32 *)0x40007400))
#define GPIO_PORTD_AFSEL_REG      (*((volatile uint32 *)0x40007420))
#define GPIO_PORTD_PUR_REG        (*((volatile uint32 *)0x40007510))
#define GPIO_PORTD_PDR_REG        (*((volatile uint32 *)0x40007514))
#define GPIO_PORTD_DEN_REG        (*((volatile uint32 *)0x4000751C))
#define GPIO_PORTD_LOCK_REG       (*((volatile uint32 *)0x40007520))
#define GPIO_PORTD_CR_REG         (*((volatile uint32 *)0x40007524))
#define GPIO_PORTD_AMSEL_REG      (*((volatile uint32 *)0x40007528))
#define GPIO_PORTD_PCTL_REG       (*((volatile uint32 *)0x4000752C))
/* ODR added for I2C3SDA (PD1) open-drain config - GPIOODR offset 0x50C,
 * Table 10-6 GPIO Register Map, datasheet p.659 */
#define GPIO_PORTD_ODR_REG        (*((volatile uint32 *)0x4000750C))

/* PORTD External Interrupts Registers */
#define GPIO_PORTD_IS_REG         (*((volatile uint32 *)0x40007404))
#define GPIO_PORTD_IBE_REG        (*((volatile uint32 *)0x40007408))
#define GPIO_PORTD_IEV_REG        (*((volatile uint32 *)0x4000740C))
#define GPIO_PORTD_IM_REG         (*((volatile uint32 *)0x40007410))
#define GPIO_PORTD_RIS_REG        (*((volatile uint32 *)0x40007414))
#define GPIO_PORTD_ICR_REG        (*((volatile uint32 *)0x4000741C))

/*****************************************************************************
GPIO registers (PORTE)
*****************************************************************************/
#define GPIO_PORTE_DATA_REG       (*((volatile uint32 *)0x400243FC))
#define GPIO_PORTE_DIR_REG        (*((volatile uint32 *)0x40024400))
#define GPIO_PORTE_AFSEL_REG      (*((volatile uint32 *)0x40024420))
#define GPIO_PORTE_PUR_REG        (*((volatile uint32 *)0x40024510))
#define GPIO_PORTE_PDR_REG        (*((volatile uint32 *)0x40024514))
#define GPIO_PORTE_DEN_REG        (*((volatile uint32 *)0x4002451C))
#define GPIO_PORTE_LOCK_REG       (*((volatile uint32 *)0x40024520))
#define GPIO_PORTE_CR_REG         (*((volatile uint32 *)0x40024524))
#define GPIO_PORTE_AMSEL_REG      (*((volatile uint32 *)0x40024528))
#define GPIO_PORTE_PCTL_REG       (*((volatile uint32 *)0x4002452C))
/* ODR added for I2C2SDA (PE5) open-drain config - GPIOODR offset 0x50C,
 * Table 10-6 GPIO Register Map, datasheet p.659. NOTE: PE4/PE5 are
 * electrically committed to CAN0Rx/CAN0Tx on this board (PORT_PBCFG.c) -
 * I2C2 is NOT usable; this register is defined only for table symmetry and
 * is never touched by i2c.c (I2C2_ENABLED is compile-guarded off, see
 * i2c_cfg.h). */
#define GPIO_PORTE_ODR_REG        (*((volatile uint32 *)0x4002450C))

/* PORTE External Interrupts Registers */
#define GPIO_PORTE_IS_REG         (*((volatile uint32 *)0x40024404))
#define GPIO_PORTE_IBE_REG        (*((volatile uint32 *)0x40024408))
#define GPIO_PORTE_IEV_REG        (*((volatile uint32 *)0x4002440C))
#define GPIO_PORTE_IM_REG         (*((volatile uint32 *)0x40024410))
#define GPIO_PORTE_RIS_REG        (*((volatile uint32 *)0x40024414))
#define GPIO_PORTE_ICR_REG        (*((volatile uint32 *)0x4002441C))

/*****************************************************************************
GPIO registers (PORTF)
*****************************************************************************/
#define GPIO_PORTF_DATA_REG       (*((volatile uint32 *)0x400253FC))
#define GPIO_PORTF_DIR_REG        (*((volatile uint32 *)0x40025400))
#define GPIO_PORTF_AFSEL_REG      (*((volatile uint32 *)0x40025420))
#define GPIO_PORTF_PUR_REG        (*((volatile uint32 *)0x40025510))
#define GPIO_PORTF_PDR_REG        (*((volatile uint32 *)0x40025514))
#define GPIO_PORTF_DEN_REG        (*((volatile uint32 *)0x4002551C))
#define GPIO_PORTF_LOCK_REG       (*((volatile uint32 *)0x40025520))
#define GPIO_PORTF_CR_REG         (*((volatile uint32 *)0x40025524))
#define GPIO_PORTF_AMSEL_REG      (*((volatile uint32 *)0x40025528))
#define GPIO_PORTF_PCTL_REG       (*((volatile uint32 *)0x4002552C))
/* ODR added for symmetry with the other 5 ports (unused by I2C - PORTF has
 * no I2C pins) - GPIOODR offset 0x50C, Table 10-6, datasheet p.659 */
#define GPIO_PORTF_ODR_REG        (*((volatile uint32 *)0x4002550C))

/* PORTF External Interrupts Registers */
#define GPIO_PORTF_IS_REG         (*((volatile uint32 *)0x40025404))
#define GPIO_PORTF_IBE_REG        (*((volatile uint32 *)0x40025408))
#define GPIO_PORTF_IEV_REG        (*((volatile uint32 *)0x4002540C))
#define GPIO_PORTF_IM_REG         (*((volatile uint32 *)0x40025410))
#define GPIO_PORTF_RIS_REG        (*((volatile uint32 *)0x40025414))
#define GPIO_PORTF_ICR_REG        (*((volatile uint32 *)0x4002541C))

/*****************************************************************************
Systick Timer Registers
*****************************************************************************/
#define SYSTICK_CTRL_REG          (*((volatile uint32 *)0xE000E010))
#define SYSTICK_RELOAD_REG        (*((volatile uint32 *)0xE000E014))
#define SYSTICK_CURRENT_REG       (*((volatile uint32 *)0xE000E018))

/*****************************************************************************
NVIC Registers
*****************************************************************************/
#define NVIC_PRI0_REG             (*((volatile uint32 *)0xE000E400))
#define NVIC_PRI1_REG             (*((volatile uint32 *)0xE000E404))
#define NVIC_PRI2_REG             (*((volatile uint32 *)0xE000E408))
#define NVIC_PRI3_REG             (*((volatile uint32 *)0xE000E40C))
#define NVIC_PRI4_REG             (*((volatile uint32 *)0xE000E410))
#define NVIC_PRI5_REG             (*((volatile uint32 *)0xE000E414))
#define NVIC_PRI6_REG             (*((volatile uint32 *)0xE000E418))
#define NVIC_PRI7_REG             (*((volatile uint32 *)0xE000E41C))
#define NVIC_PRI8_REG             (*((volatile uint32 *)0xE000E420))
#define NVIC_PRI9_REG             (*((volatile uint32 *)0xE000E424))
#define NVIC_PRI10_REG            (*((volatile uint32 *)0xE000E428))
#define NVIC_PRI11_REG            (*((volatile uint32 *)0xE000E42C))
#define NVIC_PRI12_REG            (*((volatile uint32 *)0xE000E430))
#define NVIC_PRI13_REG            (*((volatile uint32 *)0xE000E434))
#define NVIC_PRI14_REG            (*((volatile uint32 *)0xE000E438))
#define NVIC_PRI15_REG            (*((volatile uint32 *)0xE000E43C))
#define NVIC_PRI16_REG            (*((volatile uint32 *)0xE000E440))
#define NVIC_PRI17_REG            (*((volatile uint32 *)0xE000E444))
#define NVIC_PRI18_REG            (*((volatile uint32 *)0xE000E448))
#define NVIC_PRI19_REG            (*((volatile uint32 *)0xE000E44C))
#define NVIC_PRI20_REG            (*((volatile uint32 *)0xE000E450))
#define NVIC_PRI21_REG            (*((volatile uint32 *)0xE000E454))
#define NVIC_PRI22_REG            (*((volatile uint32 *)0xE000E458))
#define NVIC_PRI23_REG            (*((volatile uint32 *)0xE000E45C))
#define NVIC_PRI24_REG            (*((volatile uint32 *)0xE000E460))
#define NVIC_PRI25_REG            (*((volatile uint32 *)0xE000E464))
#define NVIC_PRI26_REG            (*((volatile uint32 *)0xE000E468))
#define NVIC_PRI27_REG            (*((volatile uint32 *)0xE000E46C))
#define NVIC_PRI28_REG            (*((volatile uint32 *)0xE000E470))
#define NVIC_PRI29_REG            (*((volatile uint32 *)0xE000E474))
#define NVIC_PRI30_REG            (*((volatile uint32 *)0xE000E478))
#define NVIC_PRI31_REG            (*((volatile uint32 *)0xE000E47C))
#define NVIC_PRI32_REG            (*((volatile uint32 *)0xE000E480))
#define NVIC_PRI33_REG            (*((volatile uint32 *)0xE000E484))
#define NVIC_PRI34_REG            (*((volatile uint32 *)0xE000E488))

#define NVIC_EN0_REG              (*((volatile uint32 *)0xE000E100))
#define NVIC_EN1_REG              (*((volatile uint32 *)0xE000E104))
#define NVIC_EN2_REG              (*((volatile uint32 *)0xE000E108))
#define NVIC_EN3_REG              (*((volatile uint32 *)0xE000E10C))
#define NVIC_EN4_REG              (*((volatile uint32 *)0xE000E110))
#define NVIC_DIS0_REG             (*((volatile uint32 *)0xE000E180))
#define NVIC_DIS1_REG             (*((volatile uint32 *)0xE000E184))
#define NVIC_DIS2_REG             (*((volatile uint32 *)0xE000E188))
#define NVIC_DIS3_REG             (*((volatile uint32 *)0xE000E18C))
#define NVIC_DIS4_REG             (*((volatile uint32 *)0xE000E190))

/*****************************************************************************
System Control Block Registers
*****************************************************************************/
#define NVIC_SYSTEM_PRI1_REG      (*((volatile uint32 *)0xE000ED18))
#define NVIC_SYSTEM_PRI2_REG      (*((volatile uint32 *)0xE000ED1C))
#define NVIC_SYSTEM_PRI3_REG      (*((volatile uint32 *)0xE000ED20))
#define NVIC_SYSTEM_SYSHNDCTRL    (*((volatile uint32 *)0xE000ED24))
#define NVIC_SYSTEM_INTCTRL       (*((volatile uint32 *)0xE000ED04))
#define NVIC_SYSTEM_CFGCTRL       (*((volatile uint32 *)0xE000ED14))

/*****************************************************************************
MPU Registers
*****************************************************************************/
#define MPU_TYPE_REG              (*((volatile uint32 *)0xE000ED90))
#define MPU_CTRL_REG              (*((volatile uint32 *)0xE000ED94))
#define MPU_NUMBER_REG            (*((volatile uint32 *)0xE000ED98))
#define MPU_BASE_REG              (*((volatile uint32 *)0xE000ED9C))
#define MPU_ATTR_REG              (*((volatile uint32 *)0xE000EDA0))
#define MPU_BASE1_REG             (*((volatile uint32 *)0xE000EDA4))
#define MPU_ATTR1_REG             (*((volatile uint32 *)0xE000EDA8))
#define MPU_BASE2_REG             (*((volatile uint32 *)0xE000EDAC))
#define MPU_ATTR2_REG             (*((volatile uint32 *)0xE000EDB0))
#define MPU_BASE3_REG             (*((volatile uint32 *)0xE000EDB4))
#define MPU_ATTR3_REG             (*((volatile uint32 *)0xE000EDB8))

/*****************************************************************************
System Control Registers
*****************************************************************************/
#define SYSCTL_DID0_REG           (*((volatile uint32 *)0x400FE000))
#define SYSCTL_DID1_REG           (*((volatile uint32 *)0x400FE004))
#define SYSCTL_DC0_REG            (*((volatile uint32 *)0x400FE008))
#define SYSCTL_DC1_REG            (*((volatile uint32 *)0x400FE010))
#define SYSCTL_DC2_REG            (*((volatile uint32 *)0x400FE014))
#define SYSCTL_DC3_REG            (*((volatile uint32 *)0x400FE018))
#define SYSCTL_DC4_REG            (*((volatile uint32 *)0x400FE01C))
#define SYSCTL_DC5_REG            (*((volatile uint32 *)0x400FE020))
#define SYSCTL_DC6_REG            (*((volatile uint32 *)0x400FE024))
#define SYSCTL_DC7_REG            (*((volatile uint32 *)0x400FE028))
#define SYSCTL_DC8_REG            (*((volatile uint32 *)0x400FE02C))
#define SYSCTL_PBORCTL_REG        (*((volatile uint32 *)0x400FE030))
#define SYSCTL_SRCR0_REG          (*((volatile uint32 *)0x400FE040))
#define SYSCTL_SRCR1_REG          (*((volatile uint32 *)0x400FE044))
#define SYSCTL_SRCR2_REG          (*((volatile uint32 *)0x400FE048))
#define SYSCTL_RIS_REG            (*((volatile uint32 *)0x400FE050))
#define SYSCTL_IMC_REG            (*((volatile uint32 *)0x400FE054))
#define SYSCTL_MISC_REG           (*((volatile uint32 *)0x400FE058))
#define SYSCTL_RESC_REG           (*((volatile uint32 *)0x400FE05C))
#define SYSCTL_RCC_REG            (*((volatile uint32 *)0x400FE060))
#define SYSCTL_GPIOHBCTL_REG      (*((volatile uint32 *)0x400FE06C))
#define SYSCTL_RCC2_REG           (*((volatile uint32 *)0x400FE070))
#define SYSCTL_MOSCCTL_REG        (*((volatile uint32 *)0x400FE07C))
#define SYSCTL_RCGC0_REG          (*((volatile uint32 *)0x400FE100))
#define SYSCTL_RCGC1_REG          (*((volatile uint32 *)0x400FE104))
#define SYSCTL_RCGC2_REG          (*((volatile uint32 *)0x400FE108))
#define SYSCTL_SCGC0_REG          (*((volatile uint32 *)0x400FE110))
#define SYSCTL_SCGC1_REG          (*((volatile uint32 *)0x400FE114))
#define SYSCTL_SCGC2_REG          (*((volatile uint32 *)0x400FE118))
#define SYSCTL_DCGC0_REG          (*((volatile uint32 *)0x400FE120))
#define SYSCTL_DCGC1_REG          (*((volatile uint32 *)0x400FE124))
#define SYSCTL_DCGC2_REG          (*((volatile uint32 *)0x400FE128))
#define SYSCTL_DSLPCLKCFG_REG     (*((volatile uint32 *)0x400FE144))
#define SYSCTL_SYSPROP_REG        (*((volatile uint32 *)0x400FE14C))
#define SYSCTL_PIOSCCAL_REG       (*((volatile uint32 *)0x400FE150))
#define SYSCTL_PIOSCSTAT_REG      (*((volatile uint32 *)0x400FE154))
#define SYSCTL_PLLFREQ0_REG       (*((volatile uint32 *)0x400FE160))
#define SYSCTL_PLLFREQ1_REG       (*((volatile uint32 *)0x400FE164))
#define SYSCTL_PLLSTAT_REG        (*((volatile uint32 *)0x400FE168))
#define SYSCTL_DC9_REG            (*((volatile uint32 *)0x400FE190))
#define SYSCTL_NVMSTAT_REG        (*((volatile uint32 *)0x400FE1A0))
#define SYSCTL_PPWD_REG           (*((volatile uint32 *)0x400FE300))
#define SYSCTL_PPTIMER_REG        (*((volatile uint32 *)0x400FE304))
#define SYSCTL_PPGPIO_REG         (*((volatile uint32 *)0x400FE308))
#define SYSCTL_PPDMA_REG          (*((volatile uint32 *)0x400FE30C))
#define SYSCTL_PPHIB_REG          (*((volatile uint32 *)0x400FE314))
#define SYSCTL_PPUART_REG         (*((volatile uint32 *)0x400FE318))
#define SYSCTL_PPSSI_REG          (*((volatile uint32 *)0x400FE31C))
#define SYSCTL_PPI2C_REG          (*((volatile uint32 *)0x400FE320))
#define SYSCTL_PPUSB_REG          (*((volatile uint32 *)0x400FE328))
#define SYSCTL_PPCAN_REG          (*((volatile uint32 *)0x400FE334))
#define SYSCTL_PPADC_REG          (*((volatile uint32 *)0x400FE338))
#define SYSCTL_PPACMP_REG         (*((volatile uint32 *)0x400FE33C))
#define SYSCTL_PPPWM_REG          (*((volatile uint32 *)0x400FE340))
#define SYSCTL_PPQEI_REG          (*((volatile uint32 *)0x400FE344))
#define SYSCTL_PPEEPROM_REG       (*((volatile uint32 *)0x400FE358))
#define SYSCTL_PPWTIMER_REG       (*((volatile uint32 *)0x400FE35C))
#define SYSCTL_SRWD_REG           (*((volatile uint32 *)0x400FE500))
#define SYSCTL_SRTIMER_REG        (*((volatile uint32 *)0x400FE504))
#define SYSCTL_SRGPIO_REG         (*((volatile uint32 *)0x400FE508))
#define SYSCTL_SRDMA_REG          (*((volatile uint32 *)0x400FE50C))
#define SYSCTL_SRHIB_REG          (*((volatile uint32 *)0x400FE514))
#define SYSCTL_SRUART_REG         (*((volatile uint32 *)0x400FE518))
#define SYSCTL_SRSSI_REG          (*((volatile uint32 *)0x400FE51C))
#define SYSCTL_SRI2C_REG          (*((volatile uint32 *)0x400FE520))
#define SYSCTL_SRUSB_REG          (*((volatile uint32 *)0x400FE528))
#define SYSCTL_SRCAN_REG          (*((volatile uint32 *)0x400FE534))
#define SYSCTL_SRADC_REG          (*((volatile uint32 *)0x400FE538))
#define SYSCTL_SRACMP_REG         (*((volatile uint32 *)0x400FE53C))
#define SYSCTL_SRPWM_REG          (*((volatile uint32 *)0x400FE540))
#define SYSCTL_SRQEI_REG          (*((volatile uint32 *)0x400FE544))
#define SYSCTL_SREEPROM_REG       (*((volatile uint32 *)0x400FE558))
#define SYSCTL_SRWTIMER_REG       (*((volatile uint32 *)0x400FE55C))
#define SYSCTL_RCGCWD_REG         (*((volatile uint32 *)0x400FE600))
#define SYSCTL_RCGCTIMER_REG      (*((volatile uint32 *)0x400FE604))
#define SYSCTL_RCGCGPIO_REG       (*((volatile uint32 *)0x400FE608))
#define SYSCTL_RCGCDMA_REG        (*((volatile uint32 *)0x400FE60C))
#define SYSCTL_RCGCHIB_REG        (*((volatile uint32 *)0x400FE614))
#define SYSCTL_RCGCUART_REG       (*((volatile uint32 *)0x400FE618))
#define SYSCTL_RCGCSSI_REG        (*((volatile uint32 *)0x400FE61C))
#define SYSCTL_RCGCI2C_REG        (*((volatile uint32 *)0x400FE620))
#define SYSCTL_RCGCUSB_REG        (*((volatile uint32 *)0x400FE628))
#define SYSCTL_RCGCCAN_REG        (*((volatile uint32 *)0x400FE634))
#define SYSCTL_RCGCADC_REG        (*((volatile uint32 *)0x400FE638))
#define SYSCTL_RCGCACMP_REG       (*((volatile uint32 *)0x400FE63C))
#define SYSCTL_RCGCPWM_REG        (*((volatile uint32 *)0x400FE640))
#define SYSCTL_RCGCQEI_REG        (*((volatile uint32 *)0x400FE644))
#define SYSCTL_RCGCEEPROM_REG     (*((volatile uint32 *)0x400FE658))
#define SYSCTL_RCGCWTIMER_REG     (*((volatile uint32 *)0x400FE65C))
#define SYSCTL_SCGCWD_REG         (*((volatile uint32 *)0x400FE700))
#define SYSCTL_SCGCTIMER_REG      (*((volatile uint32 *)0x400FE704))
#define SYSCTL_SCGCGPIO_REG       (*((volatile uint32 *)0x400FE708))
#define SYSCTL_SCGCDMA_REG        (*((volatile uint32 *)0x400FE70C))
#define SYSCTL_SCGCHIB_REG        (*((volatile uint32 *)0x400FE714))
#define SYSCTL_SCGCUART_REG       (*((volatile uint32 *)0x400FE718))
#define SYSCTL_SCGCSSI_REG        (*((volatile uint32 *)0x400FE71C))
#define SYSCTL_SCGCI2C_REG        (*((volatile uint32 *)0x400FE720))
#define SYSCTL_SCGCUSB_REG        (*((volatile uint32 *)0x400FE728))
#define SYSCTL_SCGCCAN_REG        (*((volatile uint32 *)0x400FE734))
#define SYSCTL_SCGCADC_REG        (*((volatile uint32 *)0x400FE738))
#define SYSCTL_SCGCACMP_REG       (*((volatile uint32 *)0x400FE73C))
#define SYSCTL_SCGCPWM_REG        (*((volatile uint32 *)0x400FE740))
#define SYSCTL_SCGCQEI_REG        (*((volatile uint32 *)0x400FE744))
#define SYSCTL_SCGCEEPROM_REG     (*((volatile uint32 *)0x400FE758))
#define SYSCTL_SCGCWTIMER_REG     (*((volatile uint32 *)0x400FE75C))
#define SYSCTL_DCGCWD_REG         (*((volatile uint32 *)0x400FE800))
#define SYSCTL_DCGCTIMER_REG      (*((volatile uint32 *)0x400FE804))
#define SYSCTL_DCGCGPIO_REG       (*((volatile uint32 *)0x400FE808))
#define SYSCTL_DCGCDMA_REG        (*((volatile uint32 *)0x400FE80C))
#define SYSCTL_DCGCHIB_REG        (*((volatile uint32 *)0x400FE814))
#define SYSCTL_DCGCUART_REG       (*((volatile uint32 *)0x400FE818))
#define SYSCTL_DCGCSSI_REG        (*((volatile uint32 *)0x400FE81C))
#define SYSCTL_DCGCI2C_REG        (*((volatile uint32 *)0x400FE820))
#define SYSCTL_DCGCUSB_REG        (*((volatile uint32 *)0x400FE828))
#define SYSCTL_DCGCCAN_REG        (*((volatile uint32 *)0x400FE834))
#define SYSCTL_DCGCADC_REG        (*((volatile uint32 *)0x400FE838))
#define SYSCTL_DCGCACMP_REG       (*((volatile uint32 *)0x400FE83C))
#define SYSCTL_DCGCPWM_REG        (*((volatile uint32 *)0x400FE840))
#define SYSCTL_DCGCQEI_REG        (*((volatile uint32 *)0x400FE844))
#define SYSCTL_DCGCEEPROM_REG     (*((volatile uint32 *)0x400FE858))
#define SYSCTL_DCGCWTIMER_REG     (*((volatile uint32 *)0x400FE85C))
#define SYSCTL_PRWD_REG           (*((volatile uint32 *)0x400FEA00))
#define SYSCTL_PRTIMER_REG        (*((volatile uint32 *)0x400FEA04))
#define SYSCTL_PRGPIO_REG         (*((volatile uint32 *)0x400FEA08))
#define SYSCTL_PRDMA_REG          (*((volatile uint32 *)0x400FEA0C))
#define SYSCTL_PRHIB_REG          (*((volatile uint32 *)0x400FEA14))
#define SYSCTL_PRUART_REG         (*((volatile uint32 *)0x400FEA18))
#define SYSCTL_PRSSI_REG          (*((volatile uint32 *)0x400FEA1C))
#define SYSCTL_PRI2C_REG          (*((volatile uint32 *)0x400FEA20))
#define SYSCTL_PRUSB_REG          (*((volatile uint32 *)0x400FEA28))
#define SYSCTL_PRCAN_REG          (*((volatile uint32 *)0x400FEA34))
#define SYSCTL_PRADC_REG          (*((volatile uint32 *)0x400FEA38))
#define SYSCTL_PRACMP_REG         (*((volatile uint32 *)0x400FEA3C))
#define SYSCTL_PRPWM_REG          (*((volatile uint32 *)0x400FEA40))
#define SYSCTL_PRQEI_REG          (*((volatile uint32 *)0x400FEA44))
#define SYSCTL_PREEPROM_REG       (*((volatile uint32 *)0x400FEA58))
#define SYSCTL_PRWTIMER_REG       (*((volatile uint32 *)0x400FEA5C))

/*****************************************************************************
UART0 Registers
*****************************************************************************/
#define UART0_DR_REG              (*((volatile uint32 *)0x4000C000))
#define UART0_RSR_REG             (*((volatile uint32 *)0x4000C004))
#define UART0_ECR_REG             (*((volatile uint32 *)0x4000C004))
#define UART0_FR_REG              (*((volatile uint32 *)0x4000C018))
#define UART0_ILPR_REG            (*((volatile uint32 *)0x4000C020))
#define UART0_IBRD_REG            (*((volatile uint32 *)0x4000C024))
#define UART0_FBRD_REG            (*((volatile uint32 *)0x4000C028))
#define UART0_LCRH_REG            (*((volatile uint32 *)0x4000C02C))
#define UART0_CTL_REG             (*((volatile uint32 *)0x4000C030))
#define UART0_IFLS_REG            (*((volatile uint32 *)0x4000C034))
#define UART0_IM_REG              (*((volatile uint32 *)0x4000C038))
#define UART0_RIS_REG             (*((volatile uint32 *)0x4000C03C))
#define UART0_MIS_REG             (*((volatile uint32 *)0x4000C040))
#define UART0_ICR_REG             (*((volatile uint32 *)0x4000C044))
#define UART0_DMACTL_REG          (*((volatile uint32 *)0x4000C048))
#define UART0_9BITADDR_REG        (*((volatile uint32 *)0x4000C0A4))
#define UART0_9BITAMASK_REG       (*((volatile uint32 *)0x4000C0A8))
#define UART0_PP_REG              (*((volatile uint32 *)0x4000CFC0))
#define UART0_CC_REG              (*((volatile uint32 *)0x4000CFC8))

/*****************************************************************************
Micro Direct Memory Access Registers (UDMA)
*****************************************************************************/
#define UDMA_STAT_REG             (*((volatile uint32 *)0x400FF000))
#define UDMA_CFG_REG              (*((volatile uint32 *)0x400FF004))
#define UDMA_CTLBASE_REG          (*((volatile uint32 *)0x400FF008))
#define UDMA_ALTBASE_REG          (*((volatile uint32 *)0x400FF00C))
#define UDMA_WAITSTAT_REG         (*((volatile uint32 *)0x400FF010))
#define UDMA_SWREQ_REG            (*((volatile uint32 *)0x400FF014))
#define UDMA_USEBURSTSET_REG      (*((volatile uint32 *)0x400FF018))
#define UDMA_USEBURSTCLR_R      (*((volatile uint32 *)0x400FF01C))
#define UDMA_REQMASKSET_REG       (*((volatile uint32 *)0x400FF020))
#define UDMA_REQMASKCLR_REG       (*((volatile uint32 *)0x400FF024))
#define UDMA_ENASET_REG           (*((volatile uint32 *)0x400FF028))
#define UDMA_ENACLR_REG           (*((volatile uint32 *)0x400FF02C))
#define UDMA_ALTSET_REG           (*((volatile uint32 *)0x400FF030))
#define UDMA_ALTCLR_REG           (*((volatile uint32 *)0x400FF034))
#define UDMA_PRIOSET_REG          (*((volatile uint32 *)0x400FF038))
#define UDMA_PRIOCLR_REG          (*((volatile uint32 *)0x400FF03C))
#define UDMA_ERRCLR_REG           (*((volatile uint32 *)0x400FF04C))
#define UDMA_CHASGN_REG           (*((volatile uint32 *)0x400FF500))
#define UDMA_CHIS_REG             (*((volatile uint32 *)0x400FF504))
#define UDMA_CHMAP0_REG           (*((volatile uint32 *)0x400FF510))
#define UDMA_CHMAP1_REG           (*((volatile uint32 *)0x400FF514))
#define UDMA_CHMAP2_REG           (*((volatile uint32 *)0x400FF518))
#define UDMA_CHMAP3_REG           (*((volatile uint32 *)0x400FF51C))

/*****************************************************************************
Flash Registers
*****************************************************************************/
#define FLASH_FMA_REG             (*((volatile uint32 *)0x400FD000))
#define FLASH_FMD_REG             (*((volatile uint32 *)0x400FD004))
#define FLASH_FMC_REG             (*((volatile uint32 *)0x400FD008))
#define FLASH_FCRIS_REG           (*((volatile uint32 *)0x400FD00C))
#define FLASH_FCIM_REG            (*((volatile uint32 *)0x400FD010))
#define FLASH_FCMISC_REG          (*((volatile uint32 *)0x400FD014))
#define FLASH_FMC2_REG            (*((volatile uint32 *)0x400FD020))
#define FLASH_FWBVAL_REG          (*((volatile uint32 *)0x400FD030))
#define FLASH_FWBN_REG            (*((volatile uint32 *)0x400FD100))
#define FLASH_FSIZE_REG           (*((volatile uint32 *)0x400FDFC0))
#define FLASH_SSIZE_REG           (*((volatile uint32 *)0x400FDFC4))
#define FLASH_ROMSWMAP_REG        (*((volatile uint32 *)0x400FDFCC))
#define FLASH_RMCTL_REG           (*((volatile uint32 *)0x400FE0F0))
#define FLASH_BOOTCFG_REG         (*((volatile uint32 *)0x400FE1D0))
#define FLASH_USERREG0_REG        (*((volatile uint32 *)0x400FE1E0))
#define FLASH_USERREG1_REG        (*((volatile uint32 *)0x400FE1E4))
#define FLASH_USERREG2_REG        (*((volatile uint32 *)0x400FE1E8))
#define FLASH_USERREG3_REG        (*((volatile uint32 *)0x400FE1EC))
#define FLASH_FMPRE0_REG          (*((volatile uint32 *)0x400FE200))
#define FLASH_FMPRE1_REG          (*((volatile uint32 *)0x400FE204))
#define FLASH_FMPRE2_REG          (*((volatile uint32 *)0x400FE208))
#define FLASH_FMPRE3_REG          (*((volatile uint32 *)0x400FE20C))
#define FLASH_FMPPE0_REG          (*((volatile uint32 *)0x400FE400))
#define FLASH_FMPPE1_REG          (*((volatile uint32 *)0x400FE404))
#define FLASH_FMPPE2_REG          (*((volatile uint32 *)0x400FE408))
#define FLASH_FMPPE3_REG          (*((volatile uint32 *)0x400FE40C))
/*****************************************************************************
PWM0 registers 
*****************************************************************************/
#define PWM0_CTL       (*((volatile uint32*) 0x40028000))
#define PWM0_SYNC      (*((volatile uint32*) 0x40028004))
#define PWM0_ENABLE    (* ((volatile uint32*) 0x40028008))
#define PWM0_INVERT    (* ((volatile uint32*) 0x4002800C))
#define PWM0_FAULT     (* ((volatile uint32*) 0x40028010))
#define PWM0_INTEN     (* ((volatile uint32*) 0x40028014))
#define PWM0_RIS       (* ((volatile uint32*) 0x40028018))
#define PWM0_ISC       (* ((volatile uint32*) 0x4002801C))
#define PWM0_STATUS    (* ((volatile uint32*) 0x40028020))
#define PWM0_FAULTVAL  (* ((volatile uint32*) 0x40028024))
#define PWM0_ENUPD     (* ((volatile uint32*) 0x40028028))
#define PWM0_CTL0      (* ((volatile uint32*) 0x40028040))
#define PWM0_CTL1      (* ((volatile uint32*) 0x40028080))
#define PWM0_CTL2      (* ((volatile uint32*) 0x400280C0))
#define PWM0_CTL3      (* ((volatile uint32*) 0x40028100))
#define PWM0_INTEN0    (* ((volatile uint32*) 0x40028044))
#define PWM0_INTEN1    (* ((volatile uint32*) 0x40028084))
#define PWM0_INTEN2    (* ((volatile uint32*) 0x400280C4))
#define PWM0_INTEN3    (* ((volatile uint32*) 0x40028104))
#define PWM0_RIS0      (* ((volatile uint32*) 0x40028048))
#define PWM0_RIS1      (* ((volatile uint32*) 0x40028088))
#define PWM0_RIS2      (* ((volatile uint32*) 0x400280C8))
#define PWM0_RIS3      (* ((volatile uint32*) 0x40028108))
#define PWM0_ISC0      (* ((volatile uint32*) 0x4002804C))
#define PWM0_ISC1      (* ((volatile uint32*) 0x4002808C))
#define PWM0_ISC2      (* ((volatile uint32*) 0x400280CC))
#define PWM0_ISC3      (* ((volatile uint32*) 0x4002810C))
#define PWM0_LOAD0     (* ((volatile uint32*) 0x40028050))
#define PWM0_LOAD1     (* ((volatile uint32*) 0x40028090))
#define PWM0_LOAD2     (* ((volatile uint32*) 0x400280D0))
#define PWM0_LOAD3     (* ((volatile uint32*) 0x40028110))
#define PWM0_COUNT0    (* ((volatile uint32*) 0x40028054))
#define PWM0_COUNT1    (* ((volatile uint32*) 0x40028094))
#define PWM0_COUNT2    (* ((volatile uint32*) 0x400280D4))
#define PWM0_COUNT3    (* ((volatile uint32*) 0x40028114))
#define PWM0_CMPA0     (* ((volatile uint32*) 0x40028058))
#define PWM0_CMPA1     (* ((volatile uint32*) 0x40028098))
#define PWM0_CMPA2     (* ((volatile uint32*) 0x400280D8))
#define PWM0_CMPA3     (* ((volatile uint32*) 0x40028118))
#define PWM0_CMPB0     (* ((volatile uint32*) 0x4002805C))
#define PWM0_CMPB1     (* ((volatile uint32*) 0x4002809C))
#define PWM0_CMPB2     (* ((volatile uint32*) 0x400280DC))
#define PWM0_CMPB3     (* ((volatile uint32*) 0x4002811C))
#define PWM0_GENA0     (* ((volatile uint32*) 0x40028060))
#define PWM0_GENA1     (* ((volatile uint32*) 0x400280A0))
#define PWM0_GENA2     (* ((volatile uint32*) 0x400280E0))
#define PWM0_GENA3     (* ((volatile uint32*) 0x40028120))
#define PWM0_GENB0     (* ((volatile uint32*) 0x40028064))
#define PWM0_GENB1     (* ((volatile uint32*) 0x400280A4))
#define PWM0_GENB2     (* ((volatile uint32*) 0x400280E4))
#define PWM0_GENB3     (* ((volatile uint32*) 0x40028124))


/*****************************************************************************
QEI0 registers 
*****************************************************************************/
#define QEI0_CTL_REG          (*((volatile uint32 *)0x4002C000))
#define QEI0_STAT_REG         (*((volatile uint32 *)0x4002C004))
#define QEI0_POS_REG          (*((volatile uint32 *)0x4002C008))
#define QEI0_MAXPOS_REG       (*((volatile uint32 *)0x4002C00C))
#define QEI0_LOAD_REG         (*((volatile uint32 *)0x4002C010))
#define QEI0_TIME_REG         (*((volatile uint32 *)0x4002C014))
#define QEI0_COUNT_REG        (*((volatile uint32 *)0x4002C018))
#define QEI0_SPEED_REG        (*((volatile uint32 *)0x4002C01C))
#define QEI0_INTEN_REG        (*((volatile uint32 *)0x4002C020))
#define QEI0_RIS_REG          (*((volatile uint32 *)0x4002C024))
#define QEI0_ISC_REG          (*((volatile uint32 *)0x4002C028))
/*****************************************************************************
QEI0 registers 
*****************************************************************************/
#define QEI1_CTL_REG          (*((volatile uint32 *)0x4002D000))
#define QEI1_STAT_REG         (*((volatile uint32 *)0x4002D004))
#define QEI1_POS_REG          (*((volatile uint32 *)0x4002D008))
#define QEI1_MAXPOS_REG       (*((volatile uint32 *)0x4002D00C))
#define QEI1_LOAD_REG         (*((volatile uint32 *)0x4002D010))
#define QEI1_TIME_REG         (*((volatile uint32 *)0x4002D014))
#define QEI1_COUNT_REG        (*((volatile uint32 *)0x4002D018))
#define QEI1_SPEED_REG        (*((volatile uint32 *)0x4002D01C))
#define QEI1_INTEN_REG        (*((volatile uint32 *)0x4002D020))
#define QEI1_RIS_REG          (*((volatile uint32 *)0x4002D024))
#define QEI1_ISC_REG          (*((volatile uint32 *)0x4002D028))

/*****************************************************************************
WTIMER0 (Wide General-Purpose Timer 0) registers

Added for the Timer-PWM MCAL driver (timer_pwm.c). Base address 0x40036000.
ADDITIVE ONLY - no existing definitions above were modified. The wide timer's
TimerA half drives WT0CCP0 (PC4) in PWM mode at 50Hz. A wide (32/64-bit) timer
is used so the 20ms period (320,000 counts at 16MHz) fits a 32-bit split-mode
half directly, with no 8-bit-prescaler-as-extension needed (see timer_pwm_cfg.h).
GPTM register offsets per TM4C123GH6PM datasheet Table 11-6.
*****************************************************************************/
#define WTIMER0_CFG_REG       (*((volatile uint32 *)0x40036000))  /* GPTMCFG     */
#define WTIMER0_TAMR_REG      (*((volatile uint32 *)0x40036004))  /* GPTMTAMR    */
#define WTIMER0_CTL_REG       (*((volatile uint32 *)0x4003600C))  /* GPTMCTL     */
#define WTIMER0_TAILR_REG     (*((volatile uint32 *)0x40036028))  /* GPTMTAILR   */
#define WTIMER0_TAMATCHR_REG  (*((volatile uint32 *)0x40036030))  /* GPTMTAMATCHR*/
#define WTIMER0_TAPR_REG      (*((volatile uint32 *)0x40036038))  /* GPTMTAPR    */
#define WTIMER0_TAPMR_REG     (*((volatile uint32 *)0x40036040))  /* GPTMTAPMR   */

/*****************************************************************************
ADC0 registers

Added for the steering-pot ADC MCAL (adc.c). Base address 0x40038000.
ADDITIVE ONLY - no existing definitions above were modified. Only the registers
this driver touches are declared: a single channel (AIN0/PE3) on Sample
Sequencer 3 (SS3, depth-1 FIFO), software-triggered. Offsets per TM4C123GH6PM
datasheet Table 13-2.
*****************************************************************************/
#define ADC0_ACTSS_REG    (*((volatile uint32 *)0x40038000))  /* Active SS         */
#define ADC0_RIS_REG      (*((volatile uint32 *)0x40038004))  /* Raw Int Status    */
#define ADC0_ISC_REG      (*((volatile uint32 *)0x4003800C))  /* Int Status/Clear  */
#define ADC0_EMUX_REG     (*((volatile uint32 *)0x40038014))  /* Event Mux (trig)  */
#define ADC0_PSSI_REG     (*((volatile uint32 *)0x40038028))  /* Proc SS Initiate  */
#define ADC0_SSMUX3_REG   (*((volatile uint32 *)0x400380A0))  /* SS3 Input Mux     */
#define ADC0_SSCTL3_REG   (*((volatile uint32 *)0x400380A4))  /* SS3 Control       */
#define ADC0_SSFIFO3_REG  (*((volatile uint32 *)0x400380A8))  /* SS3 Result FIFO   */

/*****************************************************************************
I2C0..I2C3 Master registers

Added for the I2C MCAL (i2c.c). ADDITIVE ONLY - no existing definitions above
were modified. Only the 5 registers a polling master driver needs are
declared (I2CMSA/I2CMCS/I2CMDR/I2CMTPR/I2CMCR) - interrupt-related registers
(I2CMIMR/I2CMRIS/I2CMMIS/I2CMICR) are intentionally NOT declared (this driver
polls, see i2c.c header comment), and I2CMCLKOCNT/I2CMBMON/I2CMCR2 are
intentionally NOT declared (optional features not used - see
I2C_MCAL_DESIGN_PREP_AUDIT.md Step A note on why the hardware clock-low
timeout isn't used here).

Base addresses (Table 2-4 Memory Map, datasheet p.92, and repeated on every
I2C register's own page, e.g. I2CMCS p.1019):
  I2C0 = 0x4002.0000   I2C1 = 0x4002.1000   I2C2 = 0x4002.2000   I2C3 = 0x4002.3000
Offsets (Table 16-4 "I2C Interface Register Map", datasheet p.1016):
  I2CMSA=0x000(p.1018)  I2CMCS=0x004(p.1019)  I2CMDR=0x008(p.1024)
  I2CMTPR=0x00C(p.1025)  I2CMCR=0x020(p.1030)
*****************************************************************************/
#define I2C0_MSA_REG          (*((volatile uint32 *)0x40020000))
#define I2C0_MCS_REG          (*((volatile uint32 *)0x40020004))
#define I2C0_MDR_REG          (*((volatile uint32 *)0x40020008))
#define I2C0_MTPR_REG         (*((volatile uint32 *)0x4002000C))
#define I2C0_MCR_REG          (*((volatile uint32 *)0x40020020))

#define I2C1_MSA_REG          (*((volatile uint32 *)0x40021000))
#define I2C1_MCS_REG          (*((volatile uint32 *)0x40021004))
#define I2C1_MDR_REG          (*((volatile uint32 *)0x40021008))
#define I2C1_MTPR_REG         (*((volatile uint32 *)0x4002100C))
#define I2C1_MCR_REG          (*((volatile uint32 *)0x40021020))

/* I2C2 - PE4/PE5 are committed to CAN0Rx/CAN0Tx on this board and NOT
 * available for I2C use (see I2C_MCAL_DESIGN_PREP_AUDIT.md SS2). Registers
 * defined here only for address-space completeness/symmetry with I2C0/1/3;
 * i2c_cfg.h compile-guards I2C2_ENABLED so this module can never actually be
 * brought up by I2C_Init(). */
#define I2C2_MSA_REG          (*((volatile uint32 *)0x40022000))
#define I2C2_MCS_REG          (*((volatile uint32 *)0x40022004))
#define I2C2_MDR_REG          (*((volatile uint32 *)0x40022008))
#define I2C2_MTPR_REG         (*((volatile uint32 *)0x4002200C))
#define I2C2_MCR_REG          (*((volatile uint32 *)0x40022020))

#define I2C3_MSA_REG          (*((volatile uint32 *)0x40023000))
#define I2C3_MCS_REG          (*((volatile uint32 *)0x40023004))
#define I2C3_MDR_REG          (*((volatile uint32 *)0x40023008))
#define I2C3_MTPR_REG         (*((volatile uint32 *)0x4002300C))
#define I2C3_MCR_REG          (*((volatile uint32 *)0x40023020))

/*****************************************************************************
TIMER0 registers (16/32-bit General-Purpose Timer 0)

Added for the free-running microsecond-counter MCAL (timer0.c). Base address
0x40030000. ADDITIVE ONLY - no existing definitions above were modified. This
is the 16/32-bit GPTM block (distinct from the WTIMER0 wide-timer block above,
which the servo owns via SYSCTL_RCGCWTIMER; TIMER0 is clocked separately via
SYSCTL_RCGCTIMER, so there is zero overlap). Used in 32-bit concatenated,
periodic, up-count mode as a monotonic tick source - only the registers this
driver touches are declared. GPTM register offsets per TM4C123GH6PM datasheet
Table 11-12 (p.725).
*****************************************************************************/
#define TIMER0_CFG_REG        (*((volatile uint32 *)0x40030000))  /* GPTMCFG   */
#define TIMER0_TAMR_REG       (*((volatile uint32 *)0x40030004))  /* GPTMTAMR  */
#define TIMER0_CTL_REG        (*((volatile uint32 *)0x4003000C))  /* GPTMCTL   */
#define TIMER0_TAILR_REG      (*((volatile uint32 *)0x40030028))  /* GPTMTAILR */
#define TIMER0_TAR_REG        (*((volatile uint32 *)0x40030048))  /* GPTMTAR (RO) */

#endif
