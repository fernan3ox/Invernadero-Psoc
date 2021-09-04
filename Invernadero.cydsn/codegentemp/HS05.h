/*******************************************************************************
* File Name: HS05.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_HS05_H)
#define CY_UART_HS05_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define HS05_RX_ENABLED                     (1u)
#define HS05_TX_ENABLED                     (1u)
#define HS05_HD_ENABLED                     (0u)
#define HS05_RX_INTERRUPT_ENABLED           (0u)
#define HS05_TX_INTERRUPT_ENABLED           (0u)
#define HS05_INTERNAL_CLOCK_USED            (1u)
#define HS05_RXHW_ADDRESS_ENABLED           (0u)
#define HS05_OVER_SAMPLE_COUNT              (8u)
#define HS05_PARITY_TYPE                    (0u)
#define HS05_PARITY_TYPE_SW                 (0u)
#define HS05_BREAK_DETECT                   (0u)
#define HS05_BREAK_BITS_TX                  (13u)
#define HS05_BREAK_BITS_RX                  (13u)
#define HS05_TXCLKGEN_DP                    (1u)
#define HS05_USE23POLLING                   (1u)
#define HS05_FLOW_CONTROL                   (0u)
#define HS05_CLK_FREQ                       (0u)
#define HS05_TX_BUFFER_SIZE                 (4u)
#define HS05_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define HS05_CONTROL_REG_REMOVED            (0u)
#else
    #define HS05_CONTROL_REG_REMOVED            (1u)
#endif /* End HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct HS05_backupStruct_
{
    uint8 enableState;

    #if(HS05_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End HS05_CONTROL_REG_REMOVED */

} HS05_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void HS05_Start(void) ;
void HS05_Stop(void) ;
uint8 HS05_ReadControlRegister(void) ;
void HS05_WriteControlRegister(uint8 control) ;

void HS05_Init(void) ;
void HS05_Enable(void) ;
void HS05_SaveConfig(void) ;
void HS05_RestoreConfig(void) ;
void HS05_Sleep(void) ;
void HS05_Wakeup(void) ;

/* Only if RX is enabled */
#if( (HS05_RX_ENABLED) || (HS05_HD_ENABLED) )

    #if (HS05_RX_INTERRUPT_ENABLED)
        #define HS05_EnableRxInt()  CyIntEnable (HS05_RX_VECT_NUM)
        #define HS05_DisableRxInt() CyIntDisable(HS05_RX_VECT_NUM)
        CY_ISR_PROTO(HS05_RXISR);
    #endif /* HS05_RX_INTERRUPT_ENABLED */

    void HS05_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void HS05_SetRxAddress1(uint8 address) ;
    void HS05_SetRxAddress2(uint8 address) ;

    void  HS05_SetRxInterruptMode(uint8 intSrc) ;
    uint8 HS05_ReadRxData(void) ;
    uint8 HS05_ReadRxStatus(void) ;
    uint8 HS05_GetChar(void) ;
    uint16 HS05_GetByte(void) ;
    uint8 HS05_GetRxBufferSize(void)
                                                            ;
    void HS05_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define HS05_GetRxInterruptSource   HS05_ReadRxStatus

#endif /* End (HS05_RX_ENABLED) || (HS05_HD_ENABLED) */

/* Only if TX is enabled */
#if(HS05_TX_ENABLED || HS05_HD_ENABLED)

    #if(HS05_TX_INTERRUPT_ENABLED)
        #define HS05_EnableTxInt()  CyIntEnable (HS05_TX_VECT_NUM)
        #define HS05_DisableTxInt() CyIntDisable(HS05_TX_VECT_NUM)
        #define HS05_SetPendingTxInt() CyIntSetPending(HS05_TX_VECT_NUM)
        #define HS05_ClearPendingTxInt() CyIntClearPending(HS05_TX_VECT_NUM)
        CY_ISR_PROTO(HS05_TXISR);
    #endif /* HS05_TX_INTERRUPT_ENABLED */

    void HS05_SetTxInterruptMode(uint8 intSrc) ;
    void HS05_WriteTxData(uint8 txDataByte) ;
    uint8 HS05_ReadTxStatus(void) ;
    void HS05_PutChar(uint8 txDataByte) ;
    void HS05_PutString(const char8 string[]) ;
    void HS05_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void HS05_PutCRLF(uint8 txDataByte) ;
    void HS05_ClearTxBuffer(void) ;
    void HS05_SetTxAddressMode(uint8 addressMode) ;
    void HS05_SendBreak(uint8 retMode) ;
    uint8 HS05_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define HS05_PutStringConst         HS05_PutString
    #define HS05_PutArrayConst          HS05_PutArray
    #define HS05_GetTxInterruptSource   HS05_ReadTxStatus

#endif /* End HS05_TX_ENABLED || HS05_HD_ENABLED */

#if(HS05_HD_ENABLED)
    void HS05_LoadRxConfig(void) ;
    void HS05_LoadTxConfig(void) ;
#endif /* End HS05_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_HS05) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    HS05_CyBtldrCommStart(void) CYSMALL ;
    void    HS05_CyBtldrCommStop(void) CYSMALL ;
    void    HS05_CyBtldrCommReset(void) CYSMALL ;
    cystatus HS05_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus HS05_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_HS05)
        #define CyBtldrCommStart    HS05_CyBtldrCommStart
        #define CyBtldrCommStop     HS05_CyBtldrCommStop
        #define CyBtldrCommReset    HS05_CyBtldrCommReset
        #define CyBtldrCommWrite    HS05_CyBtldrCommWrite
        #define CyBtldrCommRead     HS05_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_HS05) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define HS05_BYTE2BYTE_TIME_OUT (25u)
    #define HS05_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define HS05_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define HS05_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define HS05_SET_SPACE      (0x00u)
#define HS05_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (HS05_TX_ENABLED) || (HS05_HD_ENABLED) )
    #if(HS05_TX_INTERRUPT_ENABLED)
        #define HS05_TX_VECT_NUM            (uint8)HS05_TXInternalInterrupt__INTC_NUMBER
        #define HS05_TX_PRIOR_NUM           (uint8)HS05_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* HS05_TX_INTERRUPT_ENABLED */

    #define HS05_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define HS05_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define HS05_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(HS05_TX_ENABLED)
        #define HS05_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (HS05_HD_ENABLED) */
        #define HS05_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (HS05_TX_ENABLED) */

    #define HS05_TX_STS_COMPLETE            (uint8)(0x01u << HS05_TX_STS_COMPLETE_SHIFT)
    #define HS05_TX_STS_FIFO_EMPTY          (uint8)(0x01u << HS05_TX_STS_FIFO_EMPTY_SHIFT)
    #define HS05_TX_STS_FIFO_FULL           (uint8)(0x01u << HS05_TX_STS_FIFO_FULL_SHIFT)
    #define HS05_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << HS05_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (HS05_TX_ENABLED) || (HS05_HD_ENABLED)*/

#if( (HS05_RX_ENABLED) || (HS05_HD_ENABLED) )
    #if(HS05_RX_INTERRUPT_ENABLED)
        #define HS05_RX_VECT_NUM            (uint8)HS05_RXInternalInterrupt__INTC_NUMBER
        #define HS05_RX_PRIOR_NUM           (uint8)HS05_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* HS05_RX_INTERRUPT_ENABLED */
    #define HS05_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define HS05_RX_STS_BREAK_SHIFT             (0x01u)
    #define HS05_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define HS05_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define HS05_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define HS05_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define HS05_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define HS05_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define HS05_RX_STS_MRKSPC           (uint8)(0x01u << HS05_RX_STS_MRKSPC_SHIFT)
    #define HS05_RX_STS_BREAK            (uint8)(0x01u << HS05_RX_STS_BREAK_SHIFT)
    #define HS05_RX_STS_PAR_ERROR        (uint8)(0x01u << HS05_RX_STS_PAR_ERROR_SHIFT)
    #define HS05_RX_STS_STOP_ERROR       (uint8)(0x01u << HS05_RX_STS_STOP_ERROR_SHIFT)
    #define HS05_RX_STS_OVERRUN          (uint8)(0x01u << HS05_RX_STS_OVERRUN_SHIFT)
    #define HS05_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << HS05_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define HS05_RX_STS_ADDR_MATCH       (uint8)(0x01u << HS05_RX_STS_ADDR_MATCH_SHIFT)
    #define HS05_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << HS05_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define HS05_RX_HW_MASK                     (0x7Fu)
#endif /* End (HS05_RX_ENABLED) || (HS05_HD_ENABLED) */

/* Control Register definitions */
#define HS05_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define HS05_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define HS05_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define HS05_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define HS05_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define HS05_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define HS05_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define HS05_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define HS05_CTRL_HD_SEND               (uint8)(0x01u << HS05_CTRL_HD_SEND_SHIFT)
#define HS05_CTRL_HD_SEND_BREAK         (uint8)(0x01u << HS05_CTRL_HD_SEND_BREAK_SHIFT)
#define HS05_CTRL_MARK                  (uint8)(0x01u << HS05_CTRL_MARK_SHIFT)
#define HS05_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << HS05_CTRL_PARITY_TYPE0_SHIFT)
#define HS05_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << HS05_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define HS05_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define HS05_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define HS05_SEND_BREAK                         (0x00u)
#define HS05_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define HS05_REINIT                             (0x02u)
#define HS05_SEND_WAIT_REINIT                   (0x03u)

#define HS05_OVER_SAMPLE_8                      (8u)
#define HS05_OVER_SAMPLE_16                     (16u)

#define HS05_BIT_CENTER                         (HS05_OVER_SAMPLE_COUNT - 2u)

#define HS05_FIFO_LENGTH                        (4u)
#define HS05_NUMBER_OF_START_BIT                (1u)
#define HS05_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define HS05_TXBITCTR_BREAKBITS8X   ((HS05_BREAK_BITS_TX * HS05_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define HS05_TXBITCTR_BREAKBITS ((HS05_BREAK_BITS_TX * HS05_OVER_SAMPLE_COUNT) - 1u)

#define HS05_HALF_BIT_COUNT   \
                            (((HS05_OVER_SAMPLE_COUNT / 2u) + (HS05_USE23POLLING * 1u)) - 2u)
#if (HS05_OVER_SAMPLE_COUNT == HS05_OVER_SAMPLE_8)
    #define HS05_HD_TXBITCTR_INIT   (((HS05_BREAK_BITS_TX + \
                            HS05_NUMBER_OF_START_BIT) * HS05_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define HS05_RXBITCTR_INIT  ((((HS05_BREAK_BITS_RX + HS05_NUMBER_OF_START_BIT) \
                            * HS05_OVER_SAMPLE_COUNT) + HS05_HALF_BIT_COUNT) - 1u)

#else /* HS05_OVER_SAMPLE_COUNT == HS05_OVER_SAMPLE_16 */
    #define HS05_HD_TXBITCTR_INIT   ((8u * HS05_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define HS05_RXBITCTR_INIT      (((7u * HS05_OVER_SAMPLE_COUNT) - 1u) + \
                                                      HS05_HALF_BIT_COUNT)
#endif /* End HS05_OVER_SAMPLE_COUNT */

#define HS05_HD_RXBITCTR_INIT                   HS05_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 HS05_initVar;
#if (HS05_TX_INTERRUPT_ENABLED && HS05_TX_ENABLED)
    extern volatile uint8 HS05_txBuffer[HS05_TX_BUFFER_SIZE];
    extern volatile uint8 HS05_txBufferRead;
    extern uint8 HS05_txBufferWrite;
#endif /* (HS05_TX_INTERRUPT_ENABLED && HS05_TX_ENABLED) */
#if (HS05_RX_INTERRUPT_ENABLED && (HS05_RX_ENABLED || HS05_HD_ENABLED))
    extern uint8 HS05_errorStatus;
    extern volatile uint8 HS05_rxBuffer[HS05_RX_BUFFER_SIZE];
    extern volatile uint8 HS05_rxBufferRead;
    extern volatile uint8 HS05_rxBufferWrite;
    extern volatile uint8 HS05_rxBufferLoopDetect;
    extern volatile uint8 HS05_rxBufferOverflow;
    #if (HS05_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 HS05_rxAddressMode;
        extern volatile uint8 HS05_rxAddressDetected;
    #endif /* (HS05_RXHW_ADDRESS_ENABLED) */
#endif /* (HS05_RX_INTERRUPT_ENABLED && (HS05_RX_ENABLED || HS05_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define HS05__B_UART__AM_SW_BYTE_BYTE 1
#define HS05__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define HS05__B_UART__AM_HW_BYTE_BY_BYTE 3
#define HS05__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define HS05__B_UART__AM_NONE 0

#define HS05__B_UART__NONE_REVB 0
#define HS05__B_UART__EVEN_REVB 1
#define HS05__B_UART__ODD_REVB 2
#define HS05__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define HS05_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define HS05_NUMBER_OF_STOP_BITS    (1u)

#if (HS05_RXHW_ADDRESS_ENABLED)
    #define HS05_RX_ADDRESS_MODE    (0u)
    #define HS05_RX_HW_ADDRESS1     (0u)
    #define HS05_RX_HW_ADDRESS2     (0u)
#endif /* (HS05_RXHW_ADDRESS_ENABLED) */

#define HS05_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << HS05_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << HS05_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << HS05_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << HS05_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << HS05_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << HS05_RX_STS_BREAK_SHIFT) \
                                        | (0 << HS05_RX_STS_OVERRUN_SHIFT))

#define HS05_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << HS05_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << HS05_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << HS05_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << HS05_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define HS05_CONTROL_REG \
                            (* (reg8 *) HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define HS05_CONTROL_PTR \
                            (  (reg8 *) HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(HS05_TX_ENABLED)
    #define HS05_TXDATA_REG          (* (reg8 *) HS05_BUART_sTX_TxShifter_u0__F0_REG)
    #define HS05_TXDATA_PTR          (  (reg8 *) HS05_BUART_sTX_TxShifter_u0__F0_REG)
    #define HS05_TXDATA_AUX_CTL_REG  (* (reg8 *) HS05_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define HS05_TXDATA_AUX_CTL_PTR  (  (reg8 *) HS05_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define HS05_TXSTATUS_REG        (* (reg8 *) HS05_BUART_sTX_TxSts__STATUS_REG)
    #define HS05_TXSTATUS_PTR        (  (reg8 *) HS05_BUART_sTX_TxSts__STATUS_REG)
    #define HS05_TXSTATUS_MASK_REG   (* (reg8 *) HS05_BUART_sTX_TxSts__MASK_REG)
    #define HS05_TXSTATUS_MASK_PTR   (  (reg8 *) HS05_BUART_sTX_TxSts__MASK_REG)
    #define HS05_TXSTATUS_ACTL_REG   (* (reg8 *) HS05_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define HS05_TXSTATUS_ACTL_PTR   (  (reg8 *) HS05_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(HS05_TXCLKGEN_DP)
        #define HS05_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define HS05_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define HS05_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define HS05_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define HS05_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define HS05_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define HS05_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define HS05_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define HS05_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define HS05_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) HS05_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* HS05_TXCLKGEN_DP */

#endif /* End HS05_TX_ENABLED */

#if(HS05_HD_ENABLED)

    #define HS05_TXDATA_REG             (* (reg8 *) HS05_BUART_sRX_RxShifter_u0__F1_REG )
    #define HS05_TXDATA_PTR             (  (reg8 *) HS05_BUART_sRX_RxShifter_u0__F1_REG )
    #define HS05_TXDATA_AUX_CTL_REG     (* (reg8 *) HS05_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define HS05_TXDATA_AUX_CTL_PTR     (  (reg8 *) HS05_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define HS05_TXSTATUS_REG           (* (reg8 *) HS05_BUART_sRX_RxSts__STATUS_REG )
    #define HS05_TXSTATUS_PTR           (  (reg8 *) HS05_BUART_sRX_RxSts__STATUS_REG )
    #define HS05_TXSTATUS_MASK_REG      (* (reg8 *) HS05_BUART_sRX_RxSts__MASK_REG )
    #define HS05_TXSTATUS_MASK_PTR      (  (reg8 *) HS05_BUART_sRX_RxSts__MASK_REG )
    #define HS05_TXSTATUS_ACTL_REG      (* (reg8 *) HS05_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define HS05_TXSTATUS_ACTL_PTR      (  (reg8 *) HS05_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End HS05_HD_ENABLED */

#if( (HS05_RX_ENABLED) || (HS05_HD_ENABLED) )
    #define HS05_RXDATA_REG             (* (reg8 *) HS05_BUART_sRX_RxShifter_u0__F0_REG )
    #define HS05_RXDATA_PTR             (  (reg8 *) HS05_BUART_sRX_RxShifter_u0__F0_REG )
    #define HS05_RXADDRESS1_REG         (* (reg8 *) HS05_BUART_sRX_RxShifter_u0__D0_REG )
    #define HS05_RXADDRESS1_PTR         (  (reg8 *) HS05_BUART_sRX_RxShifter_u0__D0_REG )
    #define HS05_RXADDRESS2_REG         (* (reg8 *) HS05_BUART_sRX_RxShifter_u0__D1_REG )
    #define HS05_RXADDRESS2_PTR         (  (reg8 *) HS05_BUART_sRX_RxShifter_u0__D1_REG )
    #define HS05_RXDATA_AUX_CTL_REG     (* (reg8 *) HS05_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define HS05_RXBITCTR_PERIOD_REG    (* (reg8 *) HS05_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define HS05_RXBITCTR_PERIOD_PTR    (  (reg8 *) HS05_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define HS05_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) HS05_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define HS05_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) HS05_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define HS05_RXBITCTR_COUNTER_REG   (* (reg8 *) HS05_BUART_sRX_RxBitCounter__COUNT_REG )
    #define HS05_RXBITCTR_COUNTER_PTR   (  (reg8 *) HS05_BUART_sRX_RxBitCounter__COUNT_REG )

    #define HS05_RXSTATUS_REG           (* (reg8 *) HS05_BUART_sRX_RxSts__STATUS_REG )
    #define HS05_RXSTATUS_PTR           (  (reg8 *) HS05_BUART_sRX_RxSts__STATUS_REG )
    #define HS05_RXSTATUS_MASK_REG      (* (reg8 *) HS05_BUART_sRX_RxSts__MASK_REG )
    #define HS05_RXSTATUS_MASK_PTR      (  (reg8 *) HS05_BUART_sRX_RxSts__MASK_REG )
    #define HS05_RXSTATUS_ACTL_REG      (* (reg8 *) HS05_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define HS05_RXSTATUS_ACTL_PTR      (  (reg8 *) HS05_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (HS05_RX_ENABLED) || (HS05_HD_ENABLED) */

#if(HS05_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define HS05_INTCLOCK_CLKEN_REG     (* (reg8 *) HS05_IntClock__PM_ACT_CFG)
    #define HS05_INTCLOCK_CLKEN_PTR     (  (reg8 *) HS05_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define HS05_INTCLOCK_CLKEN_MASK    HS05_IntClock__PM_ACT_MSK
#endif /* End HS05_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(HS05_TX_ENABLED)
    #define HS05_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End HS05_TX_ENABLED */

#if(HS05_HD_ENABLED)
    #define HS05_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End HS05_HD_ENABLED */

#if( (HS05_RX_ENABLED) || (HS05_HD_ENABLED) )
    #define HS05_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (HS05_RX_ENABLED) || (HS05_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define HS05_WAIT_1_MS      HS05_BL_CHK_DELAY_MS   

#define HS05_TXBUFFERSIZE   HS05_TX_BUFFER_SIZE
#define HS05_RXBUFFERSIZE   HS05_RX_BUFFER_SIZE

#if (HS05_RXHW_ADDRESS_ENABLED)
    #define HS05_RXADDRESSMODE  HS05_RX_ADDRESS_MODE
    #define HS05_RXHWADDRESS1   HS05_RX_HW_ADDRESS1
    #define HS05_RXHWADDRESS2   HS05_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define HS05_RXAddressMode  HS05_RXADDRESSMODE
#endif /* (HS05_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define HS05_initvar                    HS05_initVar

#define HS05_RX_Enabled                 HS05_RX_ENABLED
#define HS05_TX_Enabled                 HS05_TX_ENABLED
#define HS05_HD_Enabled                 HS05_HD_ENABLED
#define HS05_RX_IntInterruptEnabled     HS05_RX_INTERRUPT_ENABLED
#define HS05_TX_IntInterruptEnabled     HS05_TX_INTERRUPT_ENABLED
#define HS05_InternalClockUsed          HS05_INTERNAL_CLOCK_USED
#define HS05_RXHW_Address_Enabled       HS05_RXHW_ADDRESS_ENABLED
#define HS05_OverSampleCount            HS05_OVER_SAMPLE_COUNT
#define HS05_ParityType                 HS05_PARITY_TYPE

#if( HS05_TX_ENABLED && (HS05_TXBUFFERSIZE > HS05_FIFO_LENGTH))
    #define HS05_TXBUFFER               HS05_txBuffer
    #define HS05_TXBUFFERREAD           HS05_txBufferRead
    #define HS05_TXBUFFERWRITE          HS05_txBufferWrite
#endif /* End HS05_TX_ENABLED */
#if( ( HS05_RX_ENABLED || HS05_HD_ENABLED ) && \
     (HS05_RXBUFFERSIZE > HS05_FIFO_LENGTH) )
    #define HS05_RXBUFFER               HS05_rxBuffer
    #define HS05_RXBUFFERREAD           HS05_rxBufferRead
    #define HS05_RXBUFFERWRITE          HS05_rxBufferWrite
    #define HS05_RXBUFFERLOOPDETECT     HS05_rxBufferLoopDetect
    #define HS05_RXBUFFER_OVERFLOW      HS05_rxBufferOverflow
#endif /* End HS05_RX_ENABLED */

#ifdef HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define HS05_CONTROL                HS05_CONTROL_REG
#endif /* End HS05_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(HS05_TX_ENABLED)
    #define HS05_TXDATA                 HS05_TXDATA_REG
    #define HS05_TXSTATUS               HS05_TXSTATUS_REG
    #define HS05_TXSTATUS_MASK          HS05_TXSTATUS_MASK_REG
    #define HS05_TXSTATUS_ACTL          HS05_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(HS05_TXCLKGEN_DP)
        #define HS05_TXBITCLKGEN_CTR        HS05_TXBITCLKGEN_CTR_REG
        #define HS05_TXBITCLKTX_COMPLETE    HS05_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define HS05_TXBITCTR_PERIOD        HS05_TXBITCTR_PERIOD_REG
        #define HS05_TXBITCTR_CONTROL       HS05_TXBITCTR_CONTROL_REG
        #define HS05_TXBITCTR_COUNTER       HS05_TXBITCTR_COUNTER_REG
    #endif /* HS05_TXCLKGEN_DP */
#endif /* End HS05_TX_ENABLED */

#if(HS05_HD_ENABLED)
    #define HS05_TXDATA                 HS05_TXDATA_REG
    #define HS05_TXSTATUS               HS05_TXSTATUS_REG
    #define HS05_TXSTATUS_MASK          HS05_TXSTATUS_MASK_REG
    #define HS05_TXSTATUS_ACTL          HS05_TXSTATUS_ACTL_REG
#endif /* End HS05_HD_ENABLED */

#if( (HS05_RX_ENABLED) || (HS05_HD_ENABLED) )
    #define HS05_RXDATA                 HS05_RXDATA_REG
    #define HS05_RXADDRESS1             HS05_RXADDRESS1_REG
    #define HS05_RXADDRESS2             HS05_RXADDRESS2_REG
    #define HS05_RXBITCTR_PERIOD        HS05_RXBITCTR_PERIOD_REG
    #define HS05_RXBITCTR_CONTROL       HS05_RXBITCTR_CONTROL_REG
    #define HS05_RXBITCTR_COUNTER       HS05_RXBITCTR_COUNTER_REG
    #define HS05_RXSTATUS               HS05_RXSTATUS_REG
    #define HS05_RXSTATUS_MASK          HS05_RXSTATUS_MASK_REG
    #define HS05_RXSTATUS_ACTL          HS05_RXSTATUS_ACTL_REG
#endif /* End  (HS05_RX_ENABLED) || (HS05_HD_ENABLED) */

#if(HS05_INTERNAL_CLOCK_USED)
    #define HS05_INTCLOCK_CLKEN         HS05_INTCLOCK_CLKEN_REG
#endif /* End HS05_INTERNAL_CLOCK_USED */

#define HS05_WAIT_FOR_COMLETE_REINIT    HS05_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_HS05_H */


/* [] END OF FILE */
