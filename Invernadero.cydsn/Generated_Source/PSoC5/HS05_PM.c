/*******************************************************************************
* File Name: HS05_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
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


/***************************************
* Local data allocation
***************************************/

static HS05_BACKUP_STRUCT  HS05_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: HS05_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the HS05_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  HS05_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void HS05_SaveConfig(void)
{
    #if(HS05_CONTROL_REG_REMOVED == 0u)
        HS05_backup.cr = HS05_CONTROL_REG;
    #endif /* End HS05_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: HS05_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  HS05_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling HS05_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void HS05_RestoreConfig(void)
{
    #if(HS05_CONTROL_REG_REMOVED == 0u)
        HS05_CONTROL_REG = HS05_backup.cr;
    #endif /* End HS05_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: HS05_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The HS05_Sleep() API saves the current component state. Then it
*  calls the HS05_Stop() function and calls 
*  HS05_SaveConfig() to save the hardware configuration.
*  Call the HS05_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  HS05_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void HS05_Sleep(void)
{
    #if(HS05_RX_ENABLED || HS05_HD_ENABLED)
        if((HS05_RXSTATUS_ACTL_REG  & HS05_INT_ENABLE) != 0u)
        {
            HS05_backup.enableState = 1u;
        }
        else
        {
            HS05_backup.enableState = 0u;
        }
    #else
        if((HS05_TXSTATUS_ACTL_REG  & HS05_INT_ENABLE) !=0u)
        {
            HS05_backup.enableState = 1u;
        }
        else
        {
            HS05_backup.enableState = 0u;
        }
    #endif /* End HS05_RX_ENABLED || HS05_HD_ENABLED*/

    HS05_Stop();
    HS05_SaveConfig();
}


/*******************************************************************************
* Function Name: HS05_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  HS05_Sleep() was called. The HS05_Wakeup() function
*  calls the HS05_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  HS05_Sleep() function was called, the HS05_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  HS05_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void HS05_Wakeup(void)
{
    HS05_RestoreConfig();
    #if( (HS05_RX_ENABLED) || (HS05_HD_ENABLED) )
        HS05_ClearRxBuffer();
    #endif /* End (HS05_RX_ENABLED) || (HS05_HD_ENABLED) */
    #if(HS05_TX_ENABLED || HS05_HD_ENABLED)
        HS05_ClearTxBuffer();
    #endif /* End HS05_TX_ENABLED || HS05_HD_ENABLED */

    if(HS05_backup.enableState != 0u)
    {
        HS05_Enable();
    }
}


/* [] END OF FILE */
