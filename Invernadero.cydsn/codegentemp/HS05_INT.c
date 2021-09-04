/*******************************************************************************
* File Name: HS05INT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HS05.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (HS05_RX_INTERRUPT_ENABLED && (HS05_RX_ENABLED || HS05_HD_ENABLED))
    /*******************************************************************************
    * Function Name: HS05_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_rxBuffer - RAM buffer pointer for save received data.
    *  HS05_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  HS05_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  HS05_rxBufferOverflow - software overflow flag. Set to one
    *     when HS05_rxBufferWrite index overtakes
    *     HS05_rxBufferRead index.
    *  HS05_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when HS05_rxBufferWrite is equal to
    *    HS05_rxBufferRead
    *  HS05_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  HS05_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(HS05_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef HS05_RXISR_ENTRY_CALLBACK
        HS05_RXISR_EntryCallback();
    #endif /* HS05_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START HS05_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = HS05_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in HS05_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (HS05_RX_STS_BREAK | 
                            HS05_RX_STS_PAR_ERROR |
                            HS05_RX_STS_STOP_ERROR | 
                            HS05_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                HS05_errorStatus |= readStatus & ( HS05_RX_STS_BREAK | 
                                                            HS05_RX_STS_PAR_ERROR | 
                                                            HS05_RX_STS_STOP_ERROR | 
                                                            HS05_RX_STS_OVERRUN);
                /* `#START HS05_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef HS05_RXISR_ERROR_CALLBACK
                HS05_RXISR_ERROR_Callback();
            #endif /* HS05_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & HS05_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = HS05_RXDATA_REG;
            #if (HS05_RXHW_ADDRESS_ENABLED)
                if(HS05_rxAddressMode == (uint8)HS05__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & HS05_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & HS05_RX_STS_ADDR_MATCH) != 0u)
                        {
                            HS05_rxAddressDetected = 1u;
                        }
                        else
                        {
                            HS05_rxAddressDetected = 0u;
                        }
                    }
                    if(HS05_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        HS05_rxBuffer[HS05_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    HS05_rxBuffer[HS05_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                HS05_rxBuffer[HS05_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (HS05_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(HS05_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        HS05_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    HS05_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(HS05_rxBufferWrite >= HS05_RX_BUFFER_SIZE)
                    {
                        HS05_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(HS05_rxBufferWrite == HS05_rxBufferRead)
                    {
                        HS05_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (HS05_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            HS05_RXSTATUS_MASK_REG  &= (uint8)~HS05_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(HS05_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (HS05_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & HS05_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START HS05_RXISR_END` */

        /* `#END` */

    #ifdef HS05_RXISR_EXIT_CALLBACK
        HS05_RXISR_ExitCallback();
    #endif /* HS05_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (HS05_RX_INTERRUPT_ENABLED && (HS05_RX_ENABLED || HS05_HD_ENABLED)) */


#if (HS05_TX_INTERRUPT_ENABLED && HS05_TX_ENABLED)
    /*******************************************************************************
    * Function Name: HS05_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_txBuffer - RAM buffer pointer for transmit data from.
    *  HS05_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  HS05_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(HS05_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef HS05_TXISR_ENTRY_CALLBACK
        HS05_TXISR_EntryCallback();
    #endif /* HS05_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START HS05_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((HS05_txBufferRead != HS05_txBufferWrite) &&
             ((HS05_TXSTATUS_REG & HS05_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(HS05_txBufferRead >= HS05_TX_BUFFER_SIZE)
            {
                HS05_txBufferRead = 0u;
            }

            HS05_TXDATA_REG = HS05_txBuffer[HS05_txBufferRead];

            /* Set next pointer */
            HS05_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START HS05_TXISR_END` */

        /* `#END` */

    #ifdef HS05_TXISR_EXIT_CALLBACK
        HS05_TXISR_ExitCallback();
    #endif /* HS05_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (HS05_TX_INTERRUPT_ENABLED && HS05_TX_ENABLED) */


/* [] END OF FILE */
