/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart.c
 *
 * Description: Source file for TM4C123GH6PM UART driver
 *              Interrupt-driven with circular buffers and callbacks
 *
 ******************************************************************************/

#include "uart.h"
#include "uart_private.h"

/*******************************************************************************
 *                          Static Buffer Allocations                          *
 *******************************************************************************/

#if (UART0_ENABLED == 1U)
static uint8 UART0_RxBufferData[UART0_RX_BUFFER_SIZE];
static uint8 UART0_TxBufferData[UART0_TX_BUFFER_SIZE];
#endif

#if (UART1_ENABLED == 1U)
static uint8 UART1_RxBufferData[UART1_RX_BUFFER_SIZE];
static uint8 UART1_TxBufferData[UART1_TX_BUFFER_SIZE];
#endif

#if (UART2_ENABLED == 1U)
static uint8 UART2_RxBufferData[UART2_RX_BUFFER_SIZE];
static uint8 UART2_TxBufferData[UART2_TX_BUFFER_SIZE];
#endif

#if (UART3_ENABLED == 1U)
static uint8 UART3_RxBufferData[UART3_RX_BUFFER_SIZE];
static uint8 UART3_TxBufferData[UART3_TX_BUFFER_SIZE];
#endif

#if (UART4_ENABLED == 1U)
static uint8 UART4_RxBufferData[UART4_RX_BUFFER_SIZE];
static uint8 UART4_TxBufferData[UART4_TX_BUFFER_SIZE];
#endif

#if (UART5_ENABLED == 1U)
static uint8 UART5_RxBufferData[UART5_RX_BUFFER_SIZE];
static uint8 UART5_TxBufferData[UART5_TX_BUFFER_SIZE];
#endif

#if (UART6_ENABLED == 1U)
static uint8 UART6_RxBufferData[UART6_RX_BUFFER_SIZE];
static uint8 UART6_TxBufferData[UART6_TX_BUFFER_SIZE];
#endif

#if (UART7_ENABLED == 1U)
static uint8 UART7_RxBufferData[UART7_RX_BUFFER_SIZE];
static uint8 UART7_TxBufferData[UART7_TX_BUFFER_SIZE];
#endif

/*******************************************************************************
 *                          UART Handle Array                                  *
 *******************************************************************************/

static UART_HandleType UART_Handles[UART_ID_MAX] = {0};

/* U23-2: bytes discarded by UART_SendBuffer because back-pressure did not clear
 * inside UART_PRIV_TXFF_SPIN_CAP. Written from main context; volatile because it
 * is observability state a debugger/ISR may read at any point. Read via
 * UART_GetTxDroppedCount(), mirroring Can_GetIfTimeoutCount(). */
static volatile uint32 UART_TxDroppedCount[UART_ID_MAX] = {0};

/*******************************************************************************
 *                          UART Base Address Lookup                           *
 *******************************************************************************/

static const uint32 UART_BaseAddresses[UART_ID_MAX] = {
    UART0_BASE_ADDR,
    UART1_BASE_ADDR,
    UART2_BASE_ADDR,
    UART3_BASE_ADDR,
    UART4_BASE_ADDR,
    UART5_BASE_ADDR,
    UART6_BASE_ADDR,
    UART7_BASE_ADDR
};

static const uint8 UART_IrqNumbers[UART_ID_MAX] = {
    UART0_IRQ_NUM,
    UART1_IRQ_NUM,
    UART2_IRQ_NUM,
    UART3_IRQ_NUM,
    UART4_IRQ_NUM,
    UART5_IRQ_NUM,
    UART6_IRQ_NUM,
    UART7_IRQ_NUM
};

/*******************************************************************************
 *                          Private Function Prototypes                        *
 *******************************************************************************/

static void UART_ConfigureGPIO(UART_IdType uartId);
static void UART_ConfigureBaudRate(uint32 baseAddr, uint32 baudRate);
static void UART_ConfigureLineControl(uint32 baseAddr, const UART_ConfigType *config);
static void UART_ConfigureInterrupts(UART_IdType uartId, const UART_ConfigType *config);
static void UART_EnableNVIC(UART_IdType uartId, uint8 priority);
static void UART_DisableNVIC(UART_IdType uartId);
static void UART_InitBuffers(UART_IdType uartId);
static boolean CircularBuffer_IsEmpty(const UART_CircularBufferType *buf);
static boolean CircularBuffer_Put(UART_CircularBufferType *buf, uint8 data);
static boolean CircularBuffer_Get(UART_CircularBufferType *buf, uint8 *data);
static boolean CircularBuffer_GetToDR(UART_CircularBufferType *buf, uint32 baseAddr);
static boolean CircularBuffer_Peek(const UART_CircularBufferType *buf, uint8 *data);
static void CircularBuffer_Flush(UART_CircularBufferType *buf);
static void UART_HandleRxInterrupt(UART_IdType uartId);
static void UART_HandleTxInterrupt(UART_IdType uartId);
static void UART_HandleErrorInterrupt(UART_IdType uartId);
static void UART_GenericISR(UART_IdType uartId);

/*******************************************************************************
 *                          Critical Section Helpers                           *
 *
 * The TX circular buffer's shared fields (count/head/tail) are touched from
 * BOTH thread context (UART_SendBuffer) and the TX ISR (UART_HandleTxInterrupt).
 * count++ / count-- are non-atomic read-modify-write; if an interrupt lands
 * mid-update the count desyncs from head/tail, which makes the buffer re-read
 * stale bytes (replaying an entire ~buffer-sized window) and/or drop data.
 * Every count/head/tail mutation must therefore run with interrupts masked.
 *******************************************************************************/

static inline uint32 UART_EnterCritical(void)
{
    uint32 primask;
    __asm volatile ("MRS %0, primask" : "=r" (primask));
    __asm volatile ("CPSID i" ::: "memory");
    return primask;
}

static inline void UART_ExitCritical(uint32 primask)
{
    __asm volatile ("MSR primask, %0" :: "r" (primask) : "memory");
}

/*******************************************************************************
 *                          Circular Buffer Implementation                     *
 *******************************************************************************/

static boolean CircularBuffer_IsEmpty(const UART_CircularBufferType *buf)
{
    return (buf->count == 0);
}

static boolean CircularBuffer_Put(UART_CircularBufferType *buf, uint8 data)
{
    uint32 crit = UART_EnterCritical();

    if (buf->count >= buf->size)
    {
        UART_ExitCritical(crit);
        return FALSE;
    }

    buf->buffer[buf->head] = data;
    buf->head = (buf->head + 1) % buf->size;
    buf->count++;

    UART_ExitCritical(crit);
    return TRUE;
}

static boolean CircularBuffer_Get(UART_CircularBufferType *buf, uint8 *data)
{
    uint32 crit = UART_EnterCritical();

    if (buf->count == 0)
    {
        UART_ExitCritical(crit);
        return FALSE;
    }

    *data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % buf->size;
    buf->count--;

    UART_ExitCritical(crit);
    return TRUE;
}

/*
 * U23-1: dequeue ONE byte and hand it to the hardware inside ONE critical
 * section.
 *
 * Splitting these two steps is a byte-REORDERING hole: between the dequeue and
 * the UART_DR store, the TX ISR (or, after the FreeRTOS port, another task) can
 * dequeue and emit byte N+1, putting it on the wire ahead of byte N. Each byte
 * is still dequeued exactly once, so nothing is lost or duplicated - only the
 * order breaks - but a transposed pair corrupts a CSV row just as thoroughly.
 *
 * Every place that moves a byte from the TX buffer to UART_DR must go through
 * here: UART_SendByte's kick-start, both UART_SendBuffer paths, and the TX ISR.
 * The ISR already runs with interrupts masked at its own priority; the nested
 * PRIMASK save/restore below is idempotent on Cortex-M (it restores, it does not
 * unconditionally re-enable), so routing the ISR through the same helper is free
 * and keeps the four sites identical.
 *
 * Keep this critical section minimal - dequeue and store, nothing else. The
 * caller polls UART_FR for FIFO room OUTSIDE the section.
 */
static boolean CircularBuffer_GetToDR(UART_CircularBufferType *buf, uint32 baseAddr)
{
    boolean got;
    uint8 data;
    uint32 crit = UART_EnterCritical();

    got = CircularBuffer_Get(buf, &data);   /* nested critical section: safe */
    if (got)
    {
        UART_DR(baseAddr) = data;
    }

    UART_ExitCritical(crit);
    return got;
}

static boolean CircularBuffer_Peek(const UART_CircularBufferType *buf, uint8 *data)
{
    boolean got = FALSE;
    /* U23-4: the empty-check and the read of buffer[tail] must be one step, or
     * the RX ISR's consumer can take that byte in between and Peek hands back a
     * byte that has already been delivered. */
    uint32 crit = UART_EnterCritical();

    if (!CircularBuffer_IsEmpty(buf))
    {
        *data = buf->buffer[buf->tail];
        got = TRUE;
    }

    UART_ExitCritical(crit);
    return got;
}

static void CircularBuffer_Flush(UART_CircularBufferType *buf)
{
    /* U23-3: reachable at runtime via UART_FlushRxBuffer/UART_FlushTxBuffer, not
     * just from UART_InitBuffers before interrupts are live. An ISR landing
     * between the head=0 and the count=0 store sees count > 0 against zeroed
     * indices and replays stale buffer contents - the exact signature the
     * Put/Get critical sections were added to eliminate. */
    uint32 crit = UART_EnterCritical();

    buf->head = 0;
    buf->tail = 0;
    buf->count = 0;

    UART_ExitCritical(crit);
}

/*******************************************************************************
 *                          GPIO Configuration                                 *
 *******************************************************************************/

static void UART_ConfigureGPIO(UART_IdType uartId)
{
    uint32 gpioBase;
    uint8 rxPin, txPin;
    uint8 pctlShiftRx, pctlShiftTx;
    uint8 gpioPortBit;
    
    switch (uartId)
    {
        case UART_ID_0:
            gpioBase = GPIO_PORTA_BASE;
            rxPin = 0; txPin = 1;
            pctlShiftRx = 0; pctlShiftTx = 4;
            gpioPortBit = 0;
            break;
            
        case UART_ID_1:
            gpioBase = GPIO_PORTB_BASE;
            rxPin = 0; txPin = 1;
            pctlShiftRx = 0; pctlShiftTx = 4;
            gpioPortBit = 1;
            break;
            
        case UART_ID_2:
            gpioBase = GPIO_PORTD_BASE;
            rxPin = 6; txPin = 7;
            pctlShiftRx = 24; pctlShiftTx = 28;
            gpioPortBit = 3;
            break;
            
        case UART_ID_3:
            gpioBase = GPIO_PORTC_BASE;
            rxPin = 6; txPin = 7;
            pctlShiftRx = 24; pctlShiftTx = 28;
            gpioPortBit = 2;
            break;
            
        case UART_ID_4:
            gpioBase = GPIO_PORTC_BASE;
            rxPin = 4; txPin = 5;
            pctlShiftRx = 16; pctlShiftTx = 20;
            gpioPortBit = 2;
            break;
            
        case UART_ID_5:
            gpioBase = GPIO_PORTE_BASE;
            rxPin = 4; txPin = 5;
            pctlShiftRx = 16; pctlShiftTx = 20;
            gpioPortBit = 4;
            break;
            
        case UART_ID_6:
            gpioBase = GPIO_PORTD_BASE;
            rxPin = 4; txPin = 5;
            pctlShiftRx = 16; pctlShiftTx = 20;
            gpioPortBit = 3;
            break;
            
        case UART_ID_7:
            gpioBase = GPIO_PORTE_BASE;
            rxPin = 0; txPin = 1;
            pctlShiftRx = 0; pctlShiftTx = 4;
            gpioPortBit = 4;
            break;
            
        default:
            return;
    }
    
    /* Enable GPIO clock */
    SYSCTL_RCGCGPIO |= (1U << gpioPortBit);
    while (!(SYSCTL_PRGPIO & (1U << gpioPortBit)));
    
    /* Unlock PD7 if using UART2 (special case - NMI pin) */
    if ((uartId == UART_ID_2) && (txPin == 7))
    {
        GPIO_REG(gpioBase, GPIO_LOCK_OFFSET) = GPIO_LOCK_KEY;
        GPIO_REG(gpioBase, GPIO_CR_OFFSET) |= (1U << txPin);
    }
    
    /* Disable analog mode */
    GPIO_REG(gpioBase, GPIO_AMSEL_OFFSET) &= ~((1U << rxPin) | (1U << txPin));
    
    /* Enable alternate function */
    GPIO_REG(gpioBase, GPIO_AFSEL_OFFSET) |= (1U << rxPin) | (1U << txPin);
    
    /* Configure PCTL for UART function (value = 1) */
    GPIO_REG(gpioBase, GPIO_PCTL_OFFSET) &= ~((0xFU << pctlShiftRx) | (0xFU << pctlShiftTx));
    GPIO_REG(gpioBase, GPIO_PCTL_OFFSET) |= (GPIO_PCTL_UART << pctlShiftRx) | (GPIO_PCTL_UART << pctlShiftTx);
    
    /* Enable digital function */
    GPIO_REG(gpioBase, GPIO_DEN_OFFSET) |= (1U << rxPin) | (1U << txPin);
}

/*******************************************************************************
 *                          Baud Rate Configuration                            *
 *******************************************************************************/

static void UART_ConfigureBaudRate(uint32 baseAddr, uint32 baudRate)
{
    uint32 uartClk = UART_SYSTEM_CLOCK;
    uint32 brdInteger;
    uint32 brdFraction;
    float32 brd;
    
    brd = (float32)uartClk / (16.0f * (float32)baudRate);
    brdInteger = (uint32)brd;
    brdFraction = (uint32)(((brd - (float32)brdInteger) * 64.0f) + 0.5f);
    
    UART_IBRD(baseAddr) = brdInteger;
    UART_FBRD(baseAddr) = brdFraction;
}

/*******************************************************************************
 *                          Line Control Configuration                         *
 *******************************************************************************/

static void UART_ConfigureLineControl(uint32 baseAddr, const UART_ConfigType *config)
{
    uint32 lcrh = 0;
    
    lcrh |= ((uint32)config->wordLength << UART_LCRH_WLEN_POS);
    
    if (config->fifoEnable)
    {
        lcrh |= UART_LCRH_FEN;
    }
    
    if (config->parity != UART_PARITY_NONE)
    {
        lcrh |= UART_LCRH_PEN;
        if (config->parity == UART_PARITY_EVEN)
        {
            lcrh |= UART_LCRH_EPS;
        }
    }
    
    if (config->stopBits == UART_STOP_BITS_2)
    {
        lcrh |= UART_LCRH_STP2;
    }
    
    UART_LCRH(baseAddr) = lcrh;
}

/*******************************************************************************
 *                          Interrupt Configuration                            *
 *******************************************************************************/

static void UART_ConfigureInterrupts(UART_IdType uartId, const UART_ConfigType *config)
{
    uint32 baseAddr = UART_Handles[uartId].baseAddr;
    uint32 intMask = 0;
    
    UART_IFLS(baseAddr) = UART_IFLS_RX_1_2 | UART_IFLS_TX_1_2;
    
    if (config->rxIntEnable)
    {
        intMask |= UART_INT_RX;
    }
    
    if (config->txIntEnable)
    {
        intMask |= UART_INT_TX;
    }
    
    if (config->rxTimeoutIntEnable)
    {
        intMask |= UART_INT_RT;
    }
    
    if (config->errorIntEnable)
    {
        intMask |= UART_INT_ALL_ERRORS;
    }
    
    UART_ICR(baseAddr) = 0xFFFFFFFF;
    UART_IM(baseAddr) = intMask;
}

/*******************************************************************************
 *                          NVIC Configuration                                 *
 *******************************************************************************/

static void UART_EnableNVIC(UART_IdType uartId, uint8 priority)
{
    uint8 irqNum = UART_IrqNumbers[uartId];
    volatile uint32 *priReg;
    uint8 priShift;
    
    priReg = (volatile uint32 *)(NVIC_BASE_ADDR + 0x400 + ((irqNum / 4) * 4));
    priShift = ((irqNum % 4) * 8) + 5;
    
    *priReg = (*priReg & ~(0x7U << priShift)) | ((priority & 0x7U) << priShift);
    
    if (irqNum < 32)
    {
        NVIC_EN0 |= (1U << irqNum);
    }
    else
    {
        NVIC_EN1 |= (1U << (irqNum - 32));
    }
}

static void UART_DisableNVIC(UART_IdType uartId)
{
    uint8 irqNum = UART_IrqNumbers[uartId];
    volatile uint32 *disReg;
    
    disReg = (volatile uint32 *)(NVIC_BASE_ADDR + 0x180 + ((irqNum / 32) * 4));
    *disReg |= (1U << (irqNum % 32));
}

/*******************************************************************************
 *                          Buffer Initialization                              *
 *******************************************************************************/

static void UART_InitBuffers(UART_IdType uartId)
{
    UART_HandleType *handle = &UART_Handles[uartId];
    
    switch (uartId)
    {
#if (UART0_ENABLED == 1U)
        case UART_ID_0:
            handle->rxBuffer.buffer = UART0_RxBufferData;
            handle->rxBuffer.size = UART0_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART0_TxBufferData;
            handle->txBuffer.size = UART0_TX_BUFFER_SIZE;
            break;
#endif

#if (UART1_ENABLED == 1U)
        case UART_ID_1:
            handle->rxBuffer.buffer = UART1_RxBufferData;
            handle->rxBuffer.size = UART1_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART1_TxBufferData;
            handle->txBuffer.size = UART1_TX_BUFFER_SIZE;
            break;
#endif

#if (UART2_ENABLED == 1U)
        case UART_ID_2:
            handle->rxBuffer.buffer = UART2_RxBufferData;
            handle->rxBuffer.size = UART2_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART2_TxBufferData;
            handle->txBuffer.size = UART2_TX_BUFFER_SIZE;
            break;
#endif

#if (UART3_ENABLED == 1U)
        case UART_ID_3:
            handle->rxBuffer.buffer = UART3_RxBufferData;
            handle->rxBuffer.size = UART3_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART3_TxBufferData;
            handle->txBuffer.size = UART3_TX_BUFFER_SIZE;
            break;
#endif

#if (UART4_ENABLED == 1U)
        case UART_ID_4:
            handle->rxBuffer.buffer = UART4_RxBufferData;
            handle->rxBuffer.size = UART4_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART4_TxBufferData;
            handle->txBuffer.size = UART4_TX_BUFFER_SIZE;
            break;
#endif

#if (UART5_ENABLED == 1U)
        case UART_ID_5:
            handle->rxBuffer.buffer = UART5_RxBufferData;
            handle->rxBuffer.size = UART5_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART5_TxBufferData;
            handle->txBuffer.size = UART5_TX_BUFFER_SIZE;
            break;
#endif

#if (UART6_ENABLED == 1U)
        case UART_ID_6:
            handle->rxBuffer.buffer = UART6_RxBufferData;
            handle->rxBuffer.size = UART6_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART6_TxBufferData;
            handle->txBuffer.size = UART6_TX_BUFFER_SIZE;
            break;
#endif

#if (UART7_ENABLED == 1U)
        case UART_ID_7:
            handle->rxBuffer.buffer = UART7_RxBufferData;
            handle->rxBuffer.size = UART7_RX_BUFFER_SIZE;
            handle->txBuffer.buffer = UART7_TxBufferData;
            handle->txBuffer.size = UART7_TX_BUFFER_SIZE;
            break;
#endif

        default:
            break;
    }
    
    CircularBuffer_Flush(&handle->rxBuffer);
    CircularBuffer_Flush(&handle->txBuffer);
}

/*******************************************************************************
 *                          Initialization Functions                           *
 *******************************************************************************/

UART_StatusType UART_InitWithConfig(UART_IdType uartId, const UART_ConfigType *config)
{
    UART_HandleType *handle;
    uint32 baseAddr;
    
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    if (config == NULL_PTR)
    {
        return UART_ERROR_NULL_PTR;
    }
    
    handle = &UART_Handles[uartId];
    baseAddr = UART_BaseAddresses[uartId];
    
    handle->baseAddr = baseAddr;
    
    UART_ConfigureGPIO(uartId);
    
    SYSCTL_RCGCUART |= (1U << uartId);
    while (!(SYSCTL_PRUART & (1U << uartId)));
    
    UART_CTL(baseAddr) = 0;
    UART_CC(baseAddr) = 0;
    
    UART_ConfigureBaudRate(baseAddr, config->baudRate);
    UART_ConfigureLineControl(baseAddr, config);
    
    UART_InitBuffers(uartId);
    
    UART_ECR(baseAddr) = 0xFF;
    handle->lastError.framingError = 0;
    handle->lastError.parityError = 0;
    handle->lastError.overrunError = 0;
    handle->lastError.breakError = 0;
    
    UART_ConfigureInterrupts(uartId, config);
    UART_EnableNVIC(uartId, config->intPriority);
    
    UART_CTL(baseAddr) = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
    
    handle->state = UART_STATE_IDLE;
    handle->initialized = TRUE;
    
    return UART_OK;
}

UART_StatusType UART_Init(void)
{
    UART_ConfigType config;
    UART_StatusType status = UART_OK;
    
#if (UART0_ENABLED == 1U)
    config.baudRate = UART0_BAUDRATE;
    config.wordLength = UART0_DATA_BITS;
    config.parity = UART0_PARITY;
    config.stopBits = UART0_STOP_BITS;
    config.fifoEnable = UART0_FIFO_ENABLE;
    config.rxIntEnable = UART0_RX_INT_ENABLE;
    config.txIntEnable = UART0_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART0_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART0_ERROR_INT_ENABLE;
    config.intPriority = UART0_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_0, &config);
    if (status != UART_OK) return status;
#endif

#if (UART1_ENABLED == 1U)
    config.baudRate = UART1_BAUDRATE;
    config.wordLength = UART1_DATA_BITS;
    config.parity = UART1_PARITY;
    config.stopBits = UART1_STOP_BITS;
    config.fifoEnable = UART1_FIFO_ENABLE;
    config.rxIntEnable = UART1_RX_INT_ENABLE;
    config.txIntEnable = UART1_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART1_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART1_ERROR_INT_ENABLE;
    config.intPriority = UART1_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_1, &config);
    if (status != UART_OK) return status;
#endif

#if (UART2_ENABLED == 1U)
    config.baudRate = UART2_BAUDRATE;
    config.wordLength = UART2_DATA_BITS;
    config.parity = UART2_PARITY;
    config.stopBits = UART2_STOP_BITS;
    config.fifoEnable = UART2_FIFO_ENABLE;
    config.rxIntEnable = UART2_RX_INT_ENABLE;
    config.txIntEnable = UART2_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART2_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART2_ERROR_INT_ENABLE;
    config.intPriority = UART2_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_2, &config);
    if (status != UART_OK) return status;
#endif

#if (UART3_ENABLED == 1U)
    config.baudRate = UART3_BAUDRATE;
    config.wordLength = UART3_DATA_BITS;
    config.parity = UART3_PARITY;
    config.stopBits = UART3_STOP_BITS;
    config.fifoEnable = UART3_FIFO_ENABLE;
    config.rxIntEnable = UART3_RX_INT_ENABLE;
    config.txIntEnable = UART3_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART3_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART3_ERROR_INT_ENABLE;
    config.intPriority = UART3_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_3, &config);
    if (status != UART_OK) return status;
#endif

#if (UART4_ENABLED == 1U)
    config.baudRate = UART4_BAUDRATE;
    config.wordLength = UART4_DATA_BITS;
    config.parity = UART4_PARITY;
    config.stopBits = UART4_STOP_BITS;
    config.fifoEnable = UART4_FIFO_ENABLE;
    config.rxIntEnable = UART4_RX_INT_ENABLE;
    config.txIntEnable = UART4_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART4_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART4_ERROR_INT_ENABLE;
    config.intPriority = UART4_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_4, &config);
    if (status != UART_OK) return status;
#endif

#if (UART5_ENABLED == 1U)
    config.baudRate = UART5_BAUDRATE;
    config.wordLength = UART5_DATA_BITS;
    config.parity = UART5_PARITY;
    config.stopBits = UART5_STOP_BITS;
    config.fifoEnable = UART5_FIFO_ENABLE;
    config.rxIntEnable = UART5_RX_INT_ENABLE;
    config.txIntEnable = UART5_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART5_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART5_ERROR_INT_ENABLE;
    config.intPriority = UART5_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_5, &config);
    if (status != UART_OK) return status;
#endif

#if (UART6_ENABLED == 1U)
    config.baudRate = UART6_BAUDRATE;
    config.wordLength = UART6_DATA_BITS;
    config.parity = UART6_PARITY;
    config.stopBits = UART6_STOP_BITS;
    config.fifoEnable = UART6_FIFO_ENABLE;
    config.rxIntEnable = UART6_RX_INT_ENABLE;
    config.txIntEnable = UART6_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART6_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART6_ERROR_INT_ENABLE;
    config.intPriority = UART6_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_6, &config);
    if (status != UART_OK) return status;
#endif

#if (UART7_ENABLED == 1U)
    config.baudRate = UART7_BAUDRATE;
    config.wordLength = UART7_DATA_BITS;
    config.parity = UART7_PARITY;
    config.stopBits = UART7_STOP_BITS;
    config.fifoEnable = UART7_FIFO_ENABLE;
    config.rxIntEnable = UART7_RX_INT_ENABLE;
    config.txIntEnable = UART7_TX_INT_ENABLE;
    config.rxTimeoutIntEnable = UART7_RX_TIMEOUT_INT_ENABLE;
    config.errorIntEnable = UART7_ERROR_INT_ENABLE;
    config.intPriority = UART7_INT_PRIORITY;
    status = UART_InitWithConfig(UART_ID_7, &config);
    if (status != UART_OK) return status;
#endif

    return status;
}

UART_StatusType UART_DeInit(UART_IdType uartId)
{
    UART_HandleType *handle;
    uint32 baseAddr;
    
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    handle = &UART_Handles[uartId];
    
    if (!handle->initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = handle->baseAddr;
    
    UART_DisableNVIC(uartId);
    UART_CTL(baseAddr) = 0;
    SYSCTL_RCGCUART &= ~(1U << uartId);
    
    handle->initialized = FALSE;
    handle->state = UART_STATE_UNINIT;
    
    return UART_OK;
}

/*******************************************************************************
 *                          Callback Registration                              *
 *******************************************************************************/

UART_StatusType UART_RegisterCallbacks(UART_IdType uartId, const UART_CallbackConfigType *callbacks)
{
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    if (callbacks == NULL_PTR)
    {
        return UART_ERROR_NULL_PTR;
    }
    
    UART_Handles[uartId].callbacks = *callbacks;
    
    return UART_OK;
}

UART_StatusType UART_SetRxCallback(UART_IdType uartId, UART_RxCallbackType callback)
{
    if (uartId >= UART_ID_MAX) return UART_ERROR_INVALID_ID;
    if (!UART_Handles[uartId].initialized) return UART_ERROR_NOT_INITIALIZED;
    
    UART_Handles[uartId].callbacks.rxCallback = callback;
    return UART_OK;
}

UART_StatusType UART_SetTxCompleteCallback(UART_IdType uartId, UART_TxCompleteCallbackType callback)
{
    if (uartId >= UART_ID_MAX) return UART_ERROR_INVALID_ID;
    if (!UART_Handles[uartId].initialized) return UART_ERROR_NOT_INITIALIZED;
    
    UART_Handles[uartId].callbacks.txCompleteCallback = callback;
    return UART_OK;
}

UART_StatusType UART_SetErrorCallback(UART_IdType uartId, UART_ErrorCallbackType callback)
{
    if (uartId >= UART_ID_MAX) return UART_ERROR_INVALID_ID;
    if (!UART_Handles[uartId].initialized) return UART_ERROR_NOT_INITIALIZED;
    
    UART_Handles[uartId].callbacks.errorCallback = callback;
    return UART_OK;
}

UART_StatusType UART_SetIdleCallback(UART_IdType uartId, UART_IdleCallbackType callback)
{
    if (uartId >= UART_ID_MAX) return UART_ERROR_INVALID_ID;
    if (!UART_Handles[uartId].initialized) return UART_ERROR_NOT_INITIALIZED;
    
    UART_Handles[uartId].callbacks.idleCallback = callback;
    return UART_OK;
}

/*******************************************************************************
 *                          Transmit Functions                                 *
 *******************************************************************************/

UART_StatusType UART_SendByte(UART_IdType uartId, uint8 data)
{
    UART_HandleType *handle;
    uint32 baseAddr;
    
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    handle = &UART_Handles[uartId];
    
    if (!handle->initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = handle->baseAddr;
    
    if (!CircularBuffer_Put(&handle->txBuffer, data))
    {
        return UART_ERROR_BUFFER_FULL;
    }
    
    /* ⭐ KICK-START: Try to send immediately if FIFO has space */
    if (!(UART_FR(baseAddr) & UART_FR_TXFF))
    {
        (void)CircularBuffer_GetToDR(&handle->txBuffer, baseAddr);
    }
    
    /* Enable TX interrupt for any remaining bytes */
    UART_IM(baseAddr) |= UART_INT_TX;
    
    return UART_OK;
}

uint16 UART_SendBuffer(UART_IdType uartId, const uint8 *buffer, uint16 length)
{
    UART_HandleType *handle;
    uint32 baseAddr;
    uint16 sent = 0;   /* bytes actually QUEUED (excludes U23-2 drops) */
    uint16 i;

    if ((uartId >= UART_ID_MAX) || (buffer == NULL_PTR))
    {
        return 0;
    }
    
    handle = &UART_Handles[uartId];
    
    if (!handle->initialized)
    {
        return 0;
    }
    
    baseAddr = handle->baseAddr;

    /* Queue every byte. If the software buffer is momentarily full (producer
     * outran the 115200-baud drain — e.g. the startup banner burst), make
     * forward progress by pushing a byte straight from the buffer into the
     * hardware FIFO whenever the FIFO has room, then retry.
     *
     * U23-2 POLICY: CAP + DROP + COUNT. This retry loop USED TO BE UNBOUNDED —
     * "make progress rather than drop" — because a silent drop is what corrupted
     * the early CSV rows (30/60/100 RPM segments). That intent was right; the
     * cost model was not. With TX wedged (TXE cleared, the UART clock gated, the
     * line held) the FIFO never drains, Put never succeeds, and this loop NEVER
     * RETURNS — the super-loop is dead. Same defect class as the ADC hang and the
     * CAN IF-busy spins (C6-3), in the driver main.c prints from.
     *
     * So the spin is capped at UART_PRIV_TXFF_SPIN_CAP; if back-pressure has not
     * cleared by then the byte is DROPPED and counted (UART_GetTxDroppedCount).
     * This is the same stop-the-hazard-over-perfect-data call as the CAN bus-off
     * policy, and it is affordable HERE specifically because this is the diag /
     * CSV UART and not a control path: a lost diag byte under pathological load
     * beats wedging the control loop. DO NOT "restore" the unbounded spin —
     * check the drop counter instead; it reads 0 under normal load. Any future
     * control-carrying UART needs a different answer (back-pressure the producer),
     * not a longer spin.
     *
     * SCOPE, so nobody mis-reads what this bought: the cap is PER BYTE. It kills
     * the unbounded wedge, not the proportional cost of pushing more data than
     * the link can carry — a caller that dumps 40 KB at 115200 still occupies
     * main context for ~87 us per byte, because the alternative is truncating its
     * data. See the derivation in uart_private.h. */
    for (i = 0; i < length; i++)
    {
        uint32  spins   = 0;
        boolean queued  = FALSE;

        while (!(queued = CircularBuffer_Put(&handle->txBuffer, buffer[i])))
        {
            if (spins >= UART_PRIV_TXFF_SPIN_CAP)
            {
                /* Back-pressure outlasted the cap: drop this byte, count it, and
                 * hand control back to the super-loop rather than stalling it. */
                if (UART_TxDroppedCount[uartId] < 0xFFFFFFFFUL)
                {
                    UART_TxDroppedCount[uartId]++;
                }
                break;
            }
            spins++;

            /* Buffer full: drain one queued byte to the FIFO to free a slot.
             * Guarantees progress even on a cold start where the TX ISR has
             * not begun running yet. */
            if (!(UART_FR(baseAddr) & UART_FR_TXFF))
            {
                (void)CircularBuffer_GetToDR(&handle->txBuffer, baseAddr);
            }
            /* else FIFO also full: retry until the hardware shifts a byte out,
             * but only up to the cap above. */
        }

        if (queued)
        {
            sent++;
        }
    }

    /* Kick-start: prime the hardware FIFO so transmission begins, then enable
     * the TX interrupt to drain the remaining queued bytes in the background. */
    if (sent > 0)
    {
        while (!(UART_FR(baseAddr) & UART_FR_TXFF))
        {
            if (!CircularBuffer_GetToDR(&handle->txBuffer, baseAddr))
            {
                break;  /* Buffer empty */
            }
        }

        UART_IM(baseAddr) |= UART_INT_TX;
    }

    return sent;
}

uint16 UART_SendString(UART_IdType uartId, const char *str)
{
    uint16 length = 0;
    
    if (str == NULL_PTR)
    {
        return 0;
    }
    
    while (str[length] != '\0')
    {
        length++;
    }
    
    return UART_SendBuffer(uartId, (const uint8 *)str, length);
}

uint16 UART_SendInteger(UART_IdType uartId, sint32 number)
{
    char buffer[12];
    uint8 idx = 0;
    uint8 i;
    boolean negative = FALSE;
    uint32 absNum;
    
    if (number < 0)
    {
        negative = TRUE;
        absNum = (uint32)(-(number + 1)) + 1;
    }
    else
    {
        absNum = (uint32)number;
    }
    
    do
    {
        buffer[idx++] = (char)((absNum % 10) + '0');
        absNum /= 10;
    } while (absNum > 0);
    
    if (negative)
    {
        buffer[idx++] = '-';
    }
    
    for (i = 0; i < idx / 2; i++)
    {
        char temp = buffer[i];
        buffer[i] = buffer[idx - 1 - i];
        buffer[idx - 1 - i] = temp;
    }
    
    return UART_SendBuffer(uartId, (const uint8 *)buffer, idx);
}

uint16 UART_SendFloat(UART_IdType uartId, float32 number, uint8 decimals)
{
    char buffer[24];
    uint8 idx = 0;
    sint32 intPart;
    float32 decPart;
    uint8 i;
    uint16 sent = 0;
    
    if (number < 0)
    {
        buffer[idx++] = '-';
        number = -number;
    }
    
    intPart = (sint32)number;
    decPart = number - (float32)intPart;
    
    if (idx > 0)
    {
        sent += UART_SendBuffer(uartId, (const uint8 *)buffer, idx);
    }
    
    sent += UART_SendInteger(uartId, intPart);
    
    buffer[0] = '.';
    sent += UART_SendBuffer(uartId, (const uint8 *)buffer, 1);
    
    for (i = 0; i < decimals; i++)
    {
        decPart *= 10.0f;
        buffer[i] = (char)((uint8)decPart + '0');
        decPart -= (float32)(uint8)decPart;
    }
    
    sent += UART_SendBuffer(uartId, (const uint8 *)buffer, decimals);
    
    return sent;
}

/*******************************************************************************
 *                          Receive Functions                                  *
 *******************************************************************************/

UART_StatusType UART_ReceiveByte(UART_IdType uartId, uint8 *data)
{
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    if (data == NULL_PTR)
    {
        return UART_ERROR_NULL_PTR;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    if (!CircularBuffer_Get(&UART_Handles[uartId].rxBuffer, data))
    {
        return UART_ERROR_BUFFER_EMPTY;
    }
    
    return UART_OK;
}

uint16 UART_ReceiveBuffer(UART_IdType uartId, uint8 *buffer, uint16 maxLength)
{
    uint16 received = 0;
    
    if ((uartId >= UART_ID_MAX) || (buffer == NULL_PTR))
    {
        return 0;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return 0;
    }
    
    while ((received < maxLength) && CircularBuffer_Get(&UART_Handles[uartId].rxBuffer, &buffer[received]))
    {
        received++;
    }
    
    return received;
}

UART_StatusType UART_PeekByte(UART_IdType uartId, uint8 *data)
{
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    if (data == NULL_PTR)
    {
        return UART_ERROR_NULL_PTR;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    if (!CircularBuffer_Peek(&UART_Handles[uartId].rxBuffer, data))
    {
        return UART_ERROR_BUFFER_EMPTY;
    }
    
    return UART_OK;
}

uint16 UART_ReadUntilDelimiter(UART_IdType uartId, uint8 *buffer, uint16 maxLength,
                               uint8 delimiter, boolean includeDelimiter)
{
    UART_CircularBufferType *rxBuf;
    uint16 count = 0;
    uint16 tempTail;
    boolean found = FALSE;
    uint16 i;
    
    if ((uartId >= UART_ID_MAX) || (buffer == NULL_PTR) || (maxLength == 0))
    {
        return 0;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return 0;
    }
    
    rxBuf = &UART_Handles[uartId].rxBuffer;
    
    /* Search for delimiter without removing data */
    tempTail = rxBuf->tail;
    for (i = 0; i < rxBuf->count && i < maxLength; i++)
    {
        if (rxBuf->buffer[tempTail] == delimiter)
        {
            found = TRUE;
            count = i + 1;
            break;
        }
        tempTail = (tempTail + 1) % rxBuf->size;
    }
    
    if (!found)
    {
        return 0;
    }
    
    /* Extract the data */
    for (i = 0; i < count; i++)
    {
        CircularBuffer_Get(rxBuf, &buffer[i]);
    }
    
    if (!includeDelimiter && count > 0)
    {
        count--;
    }
    
    return count;
}

/*******************************************************************************
 *                          Status Functions                                   *
 *******************************************************************************/

boolean UART_IsDataAvailable(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return FALSE;
    }
    
    return !CircularBuffer_IsEmpty(&UART_Handles[uartId].rxBuffer);
}

uint16 UART_GetRxCount(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return 0;
    }
    
    return UART_Handles[uartId].rxBuffer.count;
}

uint16 UART_GetTxFreeSpace(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return 0;
    }
    
    return UART_Handles[uartId].txBuffer.size - UART_Handles[uartId].txBuffer.count;
}

uint32 UART_GetTxDroppedCount(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX)
    {
        return 0;
    }

    /* Deliberately NOT gated on initialized: a drop count is diagnostic state
     * and stays readable after a de-init. */
    return UART_TxDroppedCount[uartId];
}

boolean UART_IsTxComplete(UART_IdType uartId)
{
    UART_HandleType *handle;
    
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return TRUE;
    }
    
    handle = &UART_Handles[uartId];
    
    return CircularBuffer_IsEmpty(&handle->txBuffer) && 
           !(UART_FR(handle->baseAddr) & UART_FR_BUSY);
}

UART_StateType UART_GetState(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX)
    {
        return UART_STATE_UNINIT;
    }
    
    return UART_Handles[uartId].state;
}

UART_ErrorFlagsType UART_GetLastError(UART_IdType uartId)
{
    UART_ErrorFlagsType emptyError = {0, 0, 0, 0, 0};
    
    if (uartId >= UART_ID_MAX)
    {
        return emptyError;
    }
    
    return UART_Handles[uartId].lastError;
}

void UART_ClearErrors(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return;
    }
    
    UART_Handles[uartId].lastError.framingError = 0;
    UART_Handles[uartId].lastError.parityError = 0;
    UART_Handles[uartId].lastError.overrunError = 0;
    UART_Handles[uartId].lastError.breakError = 0;
    
    UART_ECR(UART_Handles[uartId].baseAddr) = 0xFF;
}

/*******************************************************************************
 *                          Buffer Management                                  *
 *******************************************************************************/

void UART_FlushRxBuffer(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return;
    }
    
    CircularBuffer_Flush(&UART_Handles[uartId].rxBuffer);
}

void UART_FlushTxBuffer(UART_IdType uartId)
{
    if (uartId >= UART_ID_MAX || !UART_Handles[uartId].initialized)
    {
        return;
    }
    
    CircularBuffer_Flush(&UART_Handles[uartId].txBuffer);
}

/*******************************************************************************
 *                          Control Functions                                  *
 *******************************************************************************/

UART_StatusType UART_SetEnable(UART_IdType uartId, boolean enable)
{
    uint32 baseAddr;
    
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = UART_Handles[uartId].baseAddr;
    
    if (enable)
    {
        UART_CTL(baseAddr) |= UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
        UART_Handles[uartId].state = UART_STATE_IDLE;
    }
    else
    {
        UART_CTL(baseAddr) &= ~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
        UART_Handles[uartId].state = UART_STATE_UNINIT;
    }
    
    return UART_OK;
}

UART_StatusType UART_SetBaudRate(UART_IdType uartId, uint32 baudRate)
{
    uint32 baseAddr;
    uint32 ctlBackup;
    
    if (uartId >= UART_ID_MAX)
    {
        return UART_ERROR_INVALID_ID;
    }
    
    if (!UART_Handles[uartId].initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }
    
    baseAddr = UART_Handles[uartId].baseAddr;

    /* Wait for TX to complete.
     * U23-2 (second unbounded runtime spin): bounded by the drain in practice,
     * but unbounded if TX is ever wedged. Latent today — no env calls this at
     * runtime — capped so it cannot bite a future caller. On expiry we proceed
     * anyway: reprogramming the divisor with the old data still shifting out
     * garbles at most the in-flight bytes, which beats never returning. */
    {
        uint32 spins = UART_PRIV_FR_BUSY_SPIN_CAP;
        while (((UART_FR(baseAddr) & UART_FR_BUSY) != 0UL) && (spins > 0UL))
        {
            spins--;
        }
    }


    ctlBackup = UART_CTL(baseAddr);
    UART_CTL(baseAddr) = 0;
    
    UART_ConfigureBaudRate(baseAddr, baudRate);
    
    UART_CTL(baseAddr) = ctlBackup;
    
    return UART_OK;
}

/*******************************************************************************
 *                          Interrupt Handlers                                 *
 *******************************************************************************/

static void UART_HandleRxInterrupt(UART_IdType uartId)
{
    UART_HandleType *handle = &UART_Handles[uartId];
    uint32 baseAddr = handle->baseAddr;
    uint8 data;
    
    while (!(UART_FR(baseAddr) & UART_FR_RXFE))
    {
        data = (uint8)(UART_DR(baseAddr) & 0xFF);
        
        if (!CircularBuffer_Put(&handle->rxBuffer, data))
        {
            handle->lastError.overrunError = 1;
            if (handle->callbacks.errorCallback != NULL_PTR)
            {
                handle->callbacks.errorCallback(uartId, handle->lastError);
            }
        }
        
        if (handle->callbacks.rxCallback != NULL_PTR)
        {
            handle->callbacks.rxCallback(uartId, data);
        }
    }
    
    if ((handle->callbacks.rxBufferCallback != NULL_PTR) &&
        (handle->rxBuffer.count >= handle->callbacks.rxBufferThreshold))
    {
        handle->callbacks.rxBufferCallback(uartId, handle->rxBuffer.count);
    }
}

static void UART_HandleTxInterrupt(UART_IdType uartId)
{
    UART_HandleType *handle = &UART_Handles[uartId];
    uint32 baseAddr = handle->baseAddr;

    while (!(UART_FR(baseAddr) & UART_FR_TXFF))
    {
        /* U23-1: dequeue + DR store as one step, same helper as the main-context
         * sites. Redundant against this ISR's own preemption, but it keeps all
         * four TX hand-off sites identical and holds once tasks can preempt. */
        if (!CircularBuffer_GetToDR(&handle->txBuffer, baseAddr))
        {
            /* Disable TX interrupt - nothing more to send */
            UART_IM(baseAddr) &= ~UART_INT_TX;

            if (handle->callbacks.txCompleteCallback != NULL_PTR)
            {
                handle->callbacks.txCompleteCallback(uartId);
            }
            break;
        }
    }
}

static void UART_HandleErrorInterrupt(UART_IdType uartId)
{
    UART_HandleType *handle = &UART_Handles[uartId];
    uint32 baseAddr = handle->baseAddr;
    uint32 rsr;
    
    rsr = UART_RSR(baseAddr);
    
    if (rsr & UART_RSR_FE)
    {
        handle->lastError.framingError = 1;
    }
    if (rsr & UART_RSR_PE)
    {
        handle->lastError.parityError = 1;
    }
    if (rsr & UART_RSR_OE)
    {
        handle->lastError.overrunError = 1;
    }
    if (rsr & UART_RSR_BE)
    {
        handle->lastError.breakError = 1;
    }
    
    UART_ECR(baseAddr) = 0xFF;
    
    if (handle->callbacks.errorCallback != NULL_PTR)
    {
        handle->callbacks.errorCallback(uartId, handle->lastError);
    }
}

static void UART_GenericISR(UART_IdType uartId)
{
    UART_HandleType *handle = &UART_Handles[uartId];
    uint32 baseAddr = handle->baseAddr;
    uint32 mis;
    
    mis = UART_MIS(baseAddr);
    
    if (mis & (UART_INT_RX | UART_INT_RT))
    {
        UART_HandleRxInterrupt(uartId);
        UART_ICR(baseAddr) = UART_INT_RX | UART_INT_RT;
        
        if ((mis & UART_INT_RT) && (handle->callbacks.idleCallback != NULL_PTR))
        {
            handle->callbacks.idleCallback(uartId);
        }
    }
    
    if (mis & UART_INT_TX)
    {
        UART_HandleTxInterrupt(uartId);
        UART_ICR(baseAddr) = UART_INT_TX;
    }
    
    if (mis & UART_INT_ALL_ERRORS)
    {
        UART_HandleErrorInterrupt(uartId);
        UART_ICR(baseAddr) = UART_INT_ALL_ERRORS;
    }
}

/*******************************************************************************
 *                          UART ISR Entry Points                              *
 *******************************************************************************/

#if (UART0_ENABLED == 1U)
void UART0_Handler(void)
{
    UART_GenericISR(UART_ID_0);
}
#endif

#if (UART1_ENABLED == 1U)
void UART1_Handler(void)
{
    UART_GenericISR(UART_ID_1);
}
#endif

#if (UART2_ENABLED == 1U)
void UART2_Handler(void)
{
    UART_GenericISR(UART_ID_2);
}
#endif

#if (UART3_ENABLED == 1U)
void UART3_Handler(void)
{
    UART_GenericISR(UART_ID_3);
}
#endif

#if (UART4_ENABLED == 1U)
void UART4_Handler(void)
{
    UART_GenericISR(UART_ID_4);
}
#endif

#if (UART5_ENABLED == 1U)
void UART5_Handler(void)
{
    UART_GenericISR(UART_ID_5);
}
#endif

#if (UART6_ENABLED == 1U)
void UART6_Handler(void)
{
    UART_GenericISR(UART_ID_6);
}
#endif

#if (UART7_ENABLED == 1U)
void UART7_Handler(void)
{
    UART_GenericISR(UART_ID_7);
}
#endif
