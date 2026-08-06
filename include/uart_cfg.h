/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart_cfg.h
 *
 * Description: Configuration header for TM4C123GH6PM UART driver
 *              Edit this file to configure UART modules for your application
 *
 ******************************************************************************/

#ifndef UART_CFG_H_
#define UART_CFG_H_

/*******************************************************************************
 *                          System Configuration                               *
 *******************************************************************************/

/* System clock frequency in Hz */
#define UART_SYSTEM_CLOCK           (16000000UL)    /* 16MHz default, change to 80000000UL if using PLL */

/*******************************************************************************
 *                          UART Module Enable                                 *
 *******************************************************************************/

/* Set to 1 to enable, 0 to disable */
#define UART0_ENABLED               (0U)
#define UART1_ENABLED               (1U)
#define UART2_ENABLED               (0U)
#define UART3_ENABLED               (0U)
#define UART4_ENABLED               (0U)
#define UART5_ENABLED               (0U)
#define UART6_ENABLED               (0U)
#define UART7_ENABLED               (0U)

/*******************************************************************************
 *                          UART0 Configuration                                *
 *******************************************************************************/
#if (UART0_ENABLED == 1U)

#define UART0_BAUDRATE              (115200UL)
#define UART0_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART0_PARITY                (UART_PARITY_NONE)
#define UART0_STOP_BITS             (UART_STOP_BITS_1)
#define UART0_FIFO_ENABLE           (1U)            /* 1 = Enable 16-byte FIFO, 0 = Disable */
#define UART0_RX_BUFFER_SIZE        (128U)
#define UART0_TX_BUFFER_SIZE        (128U)

/* Interrupt Configuration */
#define UART0_RX_INT_ENABLE         (1U)
#define UART0_TX_INT_ENABLE         (1U)
#define UART0_RX_TIMEOUT_INT_ENABLE (1U)            /* Useful for variable length messages */
#define UART0_ERROR_INT_ENABLE      (1U)
#define UART0_INT_PRIORITY          (2U)            /* 0 = highest, 7 = lowest */

#endif /* UART0_ENABLED */

/*******************************************************************************
 *                          UART1 Configuration (Pi Communication)             *
 *******************************************************************************/
#if (UART1_ENABLED == 1U)

#define UART1_BAUDRATE              (115200UL)
#define UART1_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART1_PARITY                (UART_PARITY_NONE)
#define UART1_STOP_BITS             (UART_STOP_BITS_1)
#define UART1_FIFO_ENABLE           (1U)
#define UART1_RX_BUFFER_SIZE        (128U)
#define UART1_TX_BUFFER_SIZE        (512U)  /* Increased from 128 to accommodate banner (316+ chars) */

/* Interrupt Configuration */
#define UART1_RX_INT_ENABLE         (1U)
#define UART1_TX_INT_ENABLE         (1U)
#define UART1_RX_TIMEOUT_INT_ENABLE (1U)
#define UART1_ERROR_INT_ENABLE      (1U)
#define UART1_INT_PRIORITY          (2U)

#endif /* UART1_ENABLED */

/*******************************************************************************
 *                          UART2 Configuration                                *
 *******************************************************************************/
#if (UART2_ENABLED == 1U)

#define UART2_BAUDRATE              (115200UL)
#define UART2_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART2_PARITY                (UART_PARITY_NONE)
#define UART2_STOP_BITS             (UART_STOP_BITS_1)
#define UART2_FIFO_ENABLE           (1U)
#define UART2_RX_BUFFER_SIZE        (64U)
#define UART2_TX_BUFFER_SIZE        (64U)

#define UART2_RX_INT_ENABLE         (1U)
#define UART2_TX_INT_ENABLE         (1U)
#define UART2_RX_TIMEOUT_INT_ENABLE (1U)
#define UART2_ERROR_INT_ENABLE      (1U)
#define UART2_INT_PRIORITY          (3U)

#endif /* UART2_ENABLED */

/*******************************************************************************
 *                          UART3 Configuration                                *
 *******************************************************************************/
#if (UART3_ENABLED == 1U)

#define UART3_BAUDRATE              (115200UL)
#define UART3_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART3_PARITY                (UART_PARITY_NONE)
#define UART3_STOP_BITS             (UART_STOP_BITS_1)
#define UART3_FIFO_ENABLE           (1U)
#define UART3_RX_BUFFER_SIZE        (64U)
#define UART3_TX_BUFFER_SIZE        (64U)

#define UART3_RX_INT_ENABLE         (1U)
#define UART3_TX_INT_ENABLE         (1U)
#define UART3_RX_TIMEOUT_INT_ENABLE (1U)
#define UART3_ERROR_INT_ENABLE      (1U)
#define UART3_INT_PRIORITY          (3U)

#endif /* UART3_ENABLED */

/*******************************************************************************
 *                          UART4 Configuration                                *
 *******************************************************************************/
#if (UART4_ENABLED == 1U)

#define UART4_BAUDRATE              (115200UL)
#define UART4_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART4_PARITY                (UART_PARITY_NONE)
#define UART4_STOP_BITS             (UART_STOP_BITS_1)
#define UART4_FIFO_ENABLE           (1U)
#define UART4_RX_BUFFER_SIZE        (64U)
#define UART4_TX_BUFFER_SIZE        (64U)

#define UART4_RX_INT_ENABLE         (1U)
#define UART4_TX_INT_ENABLE         (1U)
#define UART4_RX_TIMEOUT_INT_ENABLE (1U)
#define UART4_ERROR_INT_ENABLE      (1U)
#define UART4_INT_PRIORITY          (3U)

#endif /* UART4_ENABLED */

/*******************************************************************************
 *                          UART5 Configuration                                *
 *******************************************************************************/
#if (UART5_ENABLED == 1U)

#define UART5_BAUDRATE              (115200UL)
#define UART5_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART5_PARITY                (UART_PARITY_NONE)
#define UART5_STOP_BITS             (UART_STOP_BITS_1)
#define UART5_FIFO_ENABLE           (1U)
#define UART5_RX_BUFFER_SIZE        (64U)
#define UART5_TX_BUFFER_SIZE        (64U)

#define UART5_RX_INT_ENABLE         (1U)
#define UART5_TX_INT_ENABLE         (1U)
#define UART5_RX_TIMEOUT_INT_ENABLE (1U)
#define UART5_ERROR_INT_ENABLE      (1U)
#define UART5_INT_PRIORITY          (3U)

#endif /* UART5_ENABLED */

/*******************************************************************************
 *                          UART6 Configuration                                *
 *******************************************************************************/
#if (UART6_ENABLED == 1U)

#define UART6_BAUDRATE              (115200UL)
#define UART6_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART6_PARITY                (UART_PARITY_NONE)
#define UART6_STOP_BITS             (UART_STOP_BITS_1)
#define UART6_FIFO_ENABLE           (1U)
#define UART6_RX_BUFFER_SIZE        (64U)
#define UART6_TX_BUFFER_SIZE        (64U)

#define UART6_RX_INT_ENABLE         (1U)
#define UART6_TX_INT_ENABLE         (1U)
#define UART6_RX_TIMEOUT_INT_ENABLE (1U)
#define UART6_ERROR_INT_ENABLE      (1U)
#define UART6_INT_PRIORITY          (3U)

#endif /* UART6_ENABLED */

/*******************************************************************************
 *                          UART7 Configuration                                *
 *******************************************************************************/
#if (UART7_ENABLED == 1U)

#define UART7_BAUDRATE              (115200UL)
#define UART7_DATA_BITS             (UART_WORD_LENGTH_8)
#define UART7_PARITY                (UART_PARITY_NONE)
#define UART7_STOP_BITS             (UART_STOP_BITS_1)
#define UART7_FIFO_ENABLE           (1U)
#define UART7_RX_BUFFER_SIZE        (64U)
#define UART7_TX_BUFFER_SIZE        (64U)

#define UART7_RX_INT_ENABLE         (1U)
#define UART7_TX_INT_ENABLE         (1U)
#define UART7_RX_TIMEOUT_INT_ENABLE (1U)
#define UART7_ERROR_INT_ENABLE      (1U)
#define UART7_INT_PRIORITY          (3U)

#endif /* UART7_ENABLED */

#endif /* UART_CFG_H_ */
