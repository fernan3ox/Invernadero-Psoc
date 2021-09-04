/*******************************************************************************
* File Name: HS05.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HS05.h"
#if (HS05_INTERNAL_CLOCK_USED)
    #include "HS05_IntClock.h"
#endif /* End HS05_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 HS05_initVar = 0u;

#if (HS05_TX_INTERRUPT_ENABLED && HS05_TX_ENABLED)
    volatile uint8 HS05_txBuffer[HS05_TX_BUFFER_SIZE];
    volatile uint8 HS05_txBufferRead = 0u;
    uint8 HS05_txBufferWrite = 0u;
#endif /* (HS05_TX_INTERRUPT_ENABLED && HS05_TX_ENABLED) */

#if (HS05_RX_INTERRUPT_ENABLED && (HS05_RX_ENABLED || HS05_HD_ENABLED))
    uint8 HS05_errorStatus = 0u;
    volatile uint8 HS05_rxBuffer[HS05_RX_BUFFER_SIZE];
    volatile uint8 HS05_rxBufferRead  = 0u;
    volatile uint8 HS05_rxBufferWrite = 0u;
    volatile uint8 HS05_rxBufferLoopDetect = 0u;
    volatile uint8 HS05_rxBufferOverflow   = 0u;
    #if (HS05_RXHW_ADDRESS_ENABLED)
        volatile uint8 HS05_rxAddressMode = HS05_RX_ADDRESS_MODE;
        volatile uint8 HS05_rxAddressDetected = 0u;
    #endif /* (HS05_RXHW_ADDRESS_ENABLED) */
#endif /* (HS05_RX_INTERRUPT_ENABLED && (HS05_RX_ENABLED || HS05_HD_ENABLED)) */


/*******************************************************************************
* Function Name: HS05_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  HS05_Start() sets the initVar variable, calls the
*  HS05_Init() function, and then calls the
*  HS05_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The HS05_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time HS05_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the HS05_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void HS05_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(HS05_initVar == 0u)
    {
        HS05_Init();
        HS05_initVar = 1u;
    }

    HS05_Enable();
}


/*******************************************************************************
* Function Name: HS05_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call HS05_Init() because
*  the HS05_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void HS05_Init(void) 
{
    #if(HS05_RX_ENABLED || HS05_HD_ENABLED)

        #if (HS05_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(HS05_RX_VECT_NUM, &HS05_RXISR);
            CyIntSetPriority(HS05_RX_VECT_NUM, HS05_RX_PRIOR_NUM);
            HS05_errorStatus = 0u;
        #endif /* (HS05_RX_INTERRUPT_ENABLED) */

        #if (HS05_RXHW_ADDRESS_ENABLED)
            HS05_SetRxAddressMode(HS05_RX_ADDRESS_MODE);
            HS05_SetRxAddress1(HS05_RX_HW_ADDRESS1);
            HS05_SetRxAddress2(HS05_RX_HW_ADDRESS2);
        #endif /* End HS05_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        HS05_RXBITCTR_PERIOD_REG = HS05_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        HS05_RXSTATUS_MASK_REG  = HS05_INIT_RX_INTERRUPTS_MASK;
    #endif /* End HS05_RX_ENABLED || HS05_HD_ENABLED*/

    #if(HS05_TX_ENABLED)
        #if (HS05_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(HS05_TX_VECT_NUM, &HS05_TXISR);
            CyIntSetPriority(HS05_TX_VECT_NUM, HS05_TX_PRIOR_NUM);
        #endif /* (HS05_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (HS05_TXCLKGEN_DP)
            HS05_TXBITCLKGEN_CTR_REG = HS05_BIT_CENTER;
            HS05_TXBITCLKTX_COMPLETE_REG = ((HS05_NUMBER_OF_DATA_BITS +
                        HS05_NUMBER_OF_START_BIT) * HS05_OVER_SAMPLE_COUNT) - 1u;
        #else
            HS05_TXBITCTR_PERIOD_REG = ((HS05_NUMBER_OF_DATA_BITS +
                        HS05_NUMBER_OF_START_BIT) * HS05_OVER_SAMPLE_8) - 1u;
        #endif /* End HS05_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (HS05_TX_INTERRUPT_ENABLED)
            HS05_TXSTATUS_MASK_REG = HS05_TX_STS_FIFO_EMPTY;
        #else
            HS05_TXSTATUS_MASK_REG = HS05_INIT_TX_INTERRUPTS_MASK;
        #endif /*End HS05_TX_INTERRUPT_ENABLED*/

    #endif /* End HS05_TX_ENABLED */

    #if(HS05_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        HS05_WriteControlRegister( \
            (HS05_ReadControlRegister() & (uint8)~HS05_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(HS05_PARITY_TYPE << HS05_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End HS05_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: HS05_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call HS05_Enable() because the HS05_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  HS05_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void HS05_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (HS05_RX_ENABLED || HS05_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        HS05_RXBITCTR_CONTROL_REG |= HS05_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        HS05_RXSTATUS_ACTL_REG  |= HS05_INT_ENABLE;

        #if (HS05_RX_INTERRUPT_ENABLED)
            HS05_EnableRxInt();

            #if (HS05_RXHW_ADDRESS_ENABLED)
                HS05_rxAddressDetected = 0u;
            #endif /* (HS05_RXHW_ADDRESS_ENABLED) */
        #endif /* (HS05_RX_INTERRUPT_ENABLED) */
    #endif /* (HS05_RX_ENABLED || HS05_HD_ENABLED) */

    #if(HS05_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!HS05_TXCLKGEN_DP)
            HS05_TXBITCTR_CONTROL_REG |= HS05_CNTR_ENABLE;
        #endif /* End HS05_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        HS05_TXSTATUS_ACTL_REG |= HS05_INT_ENABLE;
        #if (HS05_TX_INTERRUPT_ENABLED)
            HS05_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            HS05_EnableTxInt();
        #endif /* (HS05_TX_INTERRUPT_ENABLED) */
     #endif /* (HS05_TX_INTERRUPT_ENABLED) */

    #if (HS05_INTERNAL_CLOCK_USED)
        HS05_IntClock_Start();  /* Enable the clock */
    #endif /* (HS05_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: HS05_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void HS05_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (HS05_RX_ENABLED || HS05_HD_ENABLED)
        HS05_RXBITCTR_CONTROL_REG &= (uint8) ~HS05_CNTR_ENABLE;
    #endif /* (HS05_RX_ENABLED || HS05_HD_ENABLED) */

    #if (HS05_TX_ENABLED)
        #if(!HS05_TXCLKGEN_DP)
            HS05_TXBITCTR_CONTROL_REG &= (uint8) ~HS05_CNTR_ENABLE;
        #endif /* (!HS05_TXCLKGEN_DP) */
    #endif /* (HS05_TX_ENABLED) */

    #if (HS05_INTERNAL_CLOCK_USED)
        HS05_IntClock_Stop();   /* Disable the clock */
    #endif /* (HS05_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (HS05_RX_ENABLED || HS05_HD_ENABLED)
        HS05_RXSTATUS_ACTL_REG  &= (uint8) ~HS05_INT_ENABLE;

        #if (HS05_RX_INTERRUPT_ENABLED)
            HS05_DisableRxInt();
        #endif /* (HS05_RX_INTERRUPT_ENABLED) */
    #endif /* (HS05_RX_ENABLED || HS05_HD_ENABLED) */

    #if (HS05_TX_ENABLED)
        HS05_TXSTATUS_ACTL_REG &= (uint8) ~HS05_INT_ENABLE;

        #if (HS05_TX_INTERRUPT_ENABLED)
            HS05_DisableTxInt();
        #endif /* (HS05_TX_INTERRUPT_ENABLED) */
    #endif /* (HS05_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: HS05_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 HS05_ReadControlRegister(void) 
{
    #if (HS05_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(HS05_CONTROL_REG);
    #endif /* (HS05_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: HS05_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  HS05_WriteControlRegister(uint8 control) 
{
    #if (HS05_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       HS05_CONTROL_REG = control;
    #endif /* (HS05_CONTROL_REG_REMOVED) */
}


#if(HS05_RX_ENABLED || HS05_HD_ENABLED)
    /*******************************************************************************
    * Function Name: HS05_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      HS05_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      HS05_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      HS05_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      HS05_RX_STS_BREAK            Interrupt on break.
    *      HS05_RX_STS_OVERRUN          Interrupt on overrun error.
    *      HS05_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      HS05_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void HS05_SetRxInterruptMode(uint8 intSrc) 
    {
        HS05_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: HS05_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  HS05_rxBuffer - RAM buffer pointer for save received data.
    *  HS05_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  HS05_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  HS05_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 HS05_ReadRxData(void) 
    {
        uint8 rxData;

    #if (HS05_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        HS05_DisableRxInt();

        locRxBufferRead  = HS05_rxBufferRead;
        locRxBufferWrite = HS05_rxBufferWrite;

        if( (HS05_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = HS05_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= HS05_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            HS05_rxBufferRead = locRxBufferRead;

            if(HS05_rxBufferLoopDetect != 0u)
            {
                HS05_rxBufferLoopDetect = 0u;
                #if ((HS05_RX_INTERRUPT_ENABLED) && (HS05_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( HS05_HD_ENABLED )
                        if((HS05_CONTROL_REG & HS05_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            HS05_RXSTATUS_MASK_REG  |= HS05_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        HS05_RXSTATUS_MASK_REG  |= HS05_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end HS05_HD_ENABLED */
                #endif /* ((HS05_RX_INTERRUPT_ENABLED) && (HS05_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = HS05_RXDATA_REG;
        }

        HS05_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = HS05_RXDATA_REG;

    #endif /* (HS05_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: HS05_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  HS05_RX_STS_FIFO_NOTEMPTY.
    *  HS05_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  HS05_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   HS05_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   HS05_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 HS05_ReadRxStatus(void) 
    {
        uint8 status;

        status = HS05_RXSTATUS_REG & HS05_RX_HW_MASK;

    #if (HS05_RX_INTERRUPT_ENABLED)
        if(HS05_rxBufferOverflow != 0u)
        {
            status |= HS05_RX_STS_SOFT_BUFF_OVER;
            HS05_rxBufferOverflow = 0u;
        }
    #endif /* (HS05_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: HS05_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. HS05_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  HS05_rxBuffer - RAM buffer pointer for save received data.
    *  HS05_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  HS05_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  HS05_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 HS05_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (HS05_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        HS05_DisableRxInt();

        locRxBufferRead  = HS05_rxBufferRead;
        locRxBufferWrite = HS05_rxBufferWrite;

        if( (HS05_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = HS05_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= HS05_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            HS05_rxBufferRead = locRxBufferRead;

            if(HS05_rxBufferLoopDetect != 0u)
            {
                HS05_rxBufferLoopDetect = 0u;
                #if( (HS05_RX_INTERRUPT_ENABLED) && (HS05_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( HS05_HD_ENABLED )
                        if((HS05_CONTROL_REG & HS05_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            HS05_RXSTATUS_MASK_REG |= HS05_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        HS05_RXSTATUS_MASK_REG |= HS05_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end HS05_HD_ENABLED */
                #endif /* HS05_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = HS05_RXSTATUS_REG;
            if((rxStatus & HS05_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = HS05_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (HS05_RX_STS_BREAK | HS05_RX_STS_PAR_ERROR |
                                HS05_RX_STS_STOP_ERROR | HS05_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        HS05_EnableRxInt();

    #else

        rxStatus =HS05_RXSTATUS_REG;
        if((rxStatus & HS05_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = HS05_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (HS05_RX_STS_BREAK | HS05_RX_STS_PAR_ERROR |
                            HS05_RX_STS_STOP_ERROR | HS05_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (HS05_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: HS05_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 HS05_GetByte(void) 
    {
        
    #if (HS05_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        HS05_DisableRxInt();
        locErrorStatus = (uint16)HS05_errorStatus;
        HS05_errorStatus = 0u;
        HS05_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | HS05_ReadRxData() );
    #else
        return ( ((uint16)HS05_ReadRxStatus() << 8u) | HS05_ReadRxData() );
    #endif /* HS05_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: HS05_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  HS05_rxBufferWrite - used to calculate left bytes.
    *  HS05_rxBufferRead - used to calculate left bytes.
    *  HS05_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 HS05_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (HS05_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        HS05_DisableRxInt();

        if(HS05_rxBufferRead == HS05_rxBufferWrite)
        {
            if(HS05_rxBufferLoopDetect != 0u)
            {
                size = HS05_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(HS05_rxBufferRead < HS05_rxBufferWrite)
        {
            size = (HS05_rxBufferWrite - HS05_rxBufferRead);
        }
        else
        {
            size = (HS05_RX_BUFFER_SIZE - HS05_rxBufferRead) + HS05_rxBufferWrite;
        }

        HS05_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((HS05_RXSTATUS_REG & HS05_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (HS05_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: HS05_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_rxBufferWrite - cleared to zero.
    *  HS05_rxBufferRead - cleared to zero.
    *  HS05_rxBufferLoopDetect - cleared to zero.
    *  HS05_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void HS05_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        HS05_RXDATA_AUX_CTL_REG |= (uint8)  HS05_RX_FIFO_CLR;
        HS05_RXDATA_AUX_CTL_REG &= (uint8) ~HS05_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (HS05_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        HS05_DisableRxInt();

        HS05_rxBufferRead = 0u;
        HS05_rxBufferWrite = 0u;
        HS05_rxBufferLoopDetect = 0u;
        HS05_rxBufferOverflow = 0u;

        HS05_EnableRxInt();

    #endif /* (HS05_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: HS05_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  HS05__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  HS05__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  HS05__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  HS05__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  HS05__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  HS05_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void HS05_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(HS05_RXHW_ADDRESS_ENABLED)
            #if(HS05_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* HS05_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = HS05_CONTROL_REG & (uint8)~HS05_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << HS05_CTRL_RXADDR_MODE0_SHIFT);
                HS05_CONTROL_REG = tmpCtrl;

                #if(HS05_RX_INTERRUPT_ENABLED && \
                   (HS05_RXBUFFERSIZE > HS05_FIFO_LENGTH) )
                    HS05_rxAddressMode = addressMode;
                    HS05_rxAddressDetected = 0u;
                #endif /* End HS05_RXBUFFERSIZE > HS05_FIFO_LENGTH*/
            #endif /* End HS05_CONTROL_REG_REMOVED */
        #else /* HS05_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End HS05_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: HS05_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void HS05_SetRxAddress1(uint8 address) 
    {
        HS05_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: HS05_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void HS05_SetRxAddress2(uint8 address) 
    {
        HS05_RXADDRESS2_REG = address;
    }

#endif  /* HS05_RX_ENABLED || HS05_HD_ENABLED*/


#if( (HS05_TX_ENABLED) || (HS05_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: HS05_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   HS05_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   HS05_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   HS05_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   HS05_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void HS05_SetTxInterruptMode(uint8 intSrc) 
    {
        HS05_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: HS05_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  HS05_txBuffer - RAM buffer pointer for save data for transmission
    *  HS05_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  HS05_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  HS05_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void HS05_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(HS05_initVar != 0u)
        {
        #if (HS05_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            HS05_DisableTxInt();

            if( (HS05_txBufferRead == HS05_txBufferWrite) &&
                ((HS05_TXSTATUS_REG & HS05_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                HS05_TXDATA_REG = txDataByte;
            }
            else
            {
                if(HS05_txBufferWrite >= HS05_TX_BUFFER_SIZE)
                {
                    HS05_txBufferWrite = 0u;
                }

                HS05_txBuffer[HS05_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                HS05_txBufferWrite++;
            }

            HS05_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            HS05_TXDATA_REG = txDataByte;

        #endif /*(HS05_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: HS05_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 HS05_ReadTxStatus(void) 
    {
        return(HS05_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: HS05_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_txBuffer - RAM buffer pointer for save data for transmission
    *  HS05_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  HS05_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  HS05_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void HS05_PutChar(uint8 txDataByte) 
    {
    #if (HS05_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            HS05_DisableTxInt();
        #endif /* (HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = HS05_txBufferWrite;
            locTxBufferRead  = HS05_txBufferRead;

        #if ((HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            HS05_EnableTxInt();
        #endif /* (HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(HS05_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((HS05_TXSTATUS_REG & HS05_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            HS05_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= HS05_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            HS05_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3))
            HS05_DisableTxInt();
        #endif /* (HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3) */

            HS05_txBufferWrite = locTxBufferWrite;

        #if ((HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3))
            HS05_EnableTxInt();
        #endif /* (HS05_TX_BUFFER_SIZE > HS05_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (HS05_TXSTATUS_REG & HS05_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                HS05_SetPendingTxInt();
            }
        }

    #else

        while((HS05_TXSTATUS_REG & HS05_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        HS05_TXDATA_REG = txDataByte;

    #endif /* HS05_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: HS05_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void HS05_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(HS05_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                HS05_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: HS05_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void HS05_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(HS05_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                HS05_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: HS05_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void HS05_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(HS05_initVar != 0u)
        {
            HS05_PutChar(txDataByte);
            HS05_PutChar(0x0Du);
            HS05_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: HS05_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  HS05_txBufferWrite - used to calculate left space.
    *  HS05_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 HS05_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (HS05_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        HS05_DisableTxInt();

        if(HS05_txBufferRead == HS05_txBufferWrite)
        {
            size = 0u;
        }
        else if(HS05_txBufferRead < HS05_txBufferWrite)
        {
            size = (HS05_txBufferWrite - HS05_txBufferRead);
        }
        else
        {
            size = (HS05_TX_BUFFER_SIZE - HS05_txBufferRead) +
                    HS05_txBufferWrite;
        }

        HS05_EnableTxInt();

    #else

        size = HS05_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & HS05_TX_STS_FIFO_FULL) != 0u)
        {
            size = HS05_FIFO_LENGTH;
        }
        else if((size & HS05_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (HS05_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: HS05_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_txBufferWrite - cleared to zero.
    *  HS05_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void HS05_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        HS05_TXDATA_AUX_CTL_REG |= (uint8)  HS05_TX_FIFO_CLR;
        HS05_TXDATA_AUX_CTL_REG &= (uint8) ~HS05_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (HS05_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        HS05_DisableTxInt();

        HS05_txBufferRead = 0u;
        HS05_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        HS05_EnableTxInt();

    #endif /* (HS05_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: HS05_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   HS05_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   HS05_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   HS05_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   HS05_SEND_WAIT_REINIT - Performs both options: 
    *      HS05_SEND_BREAK and HS05_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  HS05_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with HS05_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The HS05_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void HS05_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(HS05_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(HS05_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == HS05_SEND_BREAK) ||
                (retMode == HS05_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                HS05_WriteControlRegister(HS05_ReadControlRegister() |
                                                      HS05_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                HS05_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = HS05_TXSTATUS_REG;
                }
                while((tmpStat & HS05_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == HS05_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == HS05_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = HS05_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & HS05_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == HS05_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == HS05_REINIT) ||
                (retMode == HS05_SEND_WAIT_REINIT) )
            {
                HS05_WriteControlRegister(HS05_ReadControlRegister() &
                                              (uint8)~HS05_CTRL_HD_SEND_BREAK);
            }

        #else /* HS05_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == HS05_SEND_BREAK) ||
                (retMode == HS05_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (HS05_PARITY_TYPE != HS05__B_UART__NONE_REVB) || \
                                    (HS05_PARITY_TYPE_SW != 0u) )
                    HS05_WriteControlRegister(HS05_ReadControlRegister() |
                                                          HS05_CTRL_HD_SEND_BREAK);
                #endif /* End HS05_PARITY_TYPE != HS05__B_UART__NONE_REVB  */

                #if(HS05_TXCLKGEN_DP)
                    txPeriod = HS05_TXBITCLKTX_COMPLETE_REG;
                    HS05_TXBITCLKTX_COMPLETE_REG = HS05_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = HS05_TXBITCTR_PERIOD_REG;
                    HS05_TXBITCTR_PERIOD_REG = HS05_TXBITCTR_BREAKBITS8X;
                #endif /* End HS05_TXCLKGEN_DP */

                /* Send zeros */
                HS05_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = HS05_TXSTATUS_REG;
                }
                while((tmpStat & HS05_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == HS05_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == HS05_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = HS05_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & HS05_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == HS05_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == HS05_REINIT) ||
                (retMode == HS05_SEND_WAIT_REINIT) )
            {

            #if(HS05_TXCLKGEN_DP)
                HS05_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                HS05_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End HS05_TXCLKGEN_DP */

            #if( (HS05_PARITY_TYPE != HS05__B_UART__NONE_REVB) || \
                 (HS05_PARITY_TYPE_SW != 0u) )
                HS05_WriteControlRegister(HS05_ReadControlRegister() &
                                                      (uint8) ~HS05_CTRL_HD_SEND_BREAK);
            #endif /* End HS05_PARITY_TYPE != NONE */
            }
        #endif    /* End HS05_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: HS05_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       HS05_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       HS05_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears HS05_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void HS05_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( HS05_CONTROL_REG_REMOVED == 0u )
            HS05_WriteControlRegister(HS05_ReadControlRegister() |
                                                  HS05_CTRL_MARK);
        #endif /* End HS05_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( HS05_CONTROL_REG_REMOVED == 0u )
            HS05_WriteControlRegister(HS05_ReadControlRegister() &
                                                  (uint8) ~HS05_CTRL_MARK);
        #endif /* End HS05_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndHS05_TX_ENABLED */

#if(HS05_HD_ENABLED)


    /*******************************************************************************
    * Function Name: HS05_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void HS05_LoadRxConfig(void) 
    {
        HS05_WriteControlRegister(HS05_ReadControlRegister() &
                                                (uint8)~HS05_CTRL_HD_SEND);
        HS05_RXBITCTR_PERIOD_REG = HS05_HD_RXBITCTR_INIT;

    #if (HS05_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        HS05_SetRxInterruptMode(HS05_INIT_RX_INTERRUPTS_MASK);
    #endif /* (HS05_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: HS05_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void HS05_LoadTxConfig(void) 
    {
    #if (HS05_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        HS05_SetRxInterruptMode(0u);
    #endif /* (HS05_RX_INTERRUPT_ENABLED) */

        HS05_WriteControlRegister(HS05_ReadControlRegister() | HS05_CTRL_HD_SEND);
        HS05_RXBITCTR_PERIOD_REG = HS05_HD_TXBITCTR_INIT;
    }

#endif  /* HS05_HD_ENABLED */


/* [] END OF FILE */
