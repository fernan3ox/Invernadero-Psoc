/*******************************************************************************
* File Name: ADC1_PM.c
* Version 3.30
*
* Description:
*  This file provides the power manager source code to the API for the
*  Delta-Sigma ADC Component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "ADC1.h"

static ADC1_BACKUP_STRUCT ADC1_backup =
{
    ADC1_DISABLED,
    ADC1_CFG1_DEC_CR
};


/*******************************************************************************
* Function Name: ADC1_SaveConfig
********************************************************************************
*
* Summary:
*  Save the register configuration which are not retention.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC1_backup:  This global structure variable is used to store
*  configuration registers which are non retention whenever user wants to go
*  sleep mode by calling Sleep() API.
*
*******************************************************************************/
void ADC1_SaveConfig(void) 
{
    ADC1_backup.deccr = ADC1_DEC_CR_REG;
}


/*******************************************************************************
* Function Name: ADC1_RestoreConfig
********************************************************************************
*
* Summary:
*  Restore the register configurations which are not retention.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC1_backup:  This global structure variable is used to restore
*  configuration registers which are non retention whenever user wants to switch
*  to active power mode by calling Wakeup() API.
*
*******************************************************************************/
void ADC1_RestoreConfig(void) 
{
    ADC1_DEC_CR_REG = ADC1_backup.deccr;
}


/*******************************************************************************
* Function Name: ADC1_Sleep
********************************************************************************
*
* Summary:
*  Stops the operation of the block and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC1_backup:  The structure field 'enableState' is modified
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void ADC1_Sleep(void) 
{
    /* Save ADC enable state */
    if((ADC1_ACT_PWR_DEC_EN == (ADC1_PWRMGR_DEC_REG & ADC1_ACT_PWR_DEC_EN)) &&
       (ADC1_ACT_PWR_DSM_EN == (ADC1_PWRMGR_DSM_REG & ADC1_ACT_PWR_DSM_EN)))
    {
        /* Component is enabled */
        ADC1_backup.enableState = ADC1_ENABLED;
        if((ADC1_DEC_CR_REG & ADC1_DEC_START_CONV) != 0u)
        {   
            /* Conversion is started */
            ADC1_backup.enableState |= ADC1_STARTED;
        }
		
        /* Stop the configuration */
        ADC1_Stop();
    }
    else
    {
        /* Component is disabled */
        ADC1_backup.enableState = ADC1_DISABLED;
    }

    /* Save the user configuration */
    ADC1_SaveConfig();
}


/*******************************************************************************
* Function Name: ADC1_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and enables the power to the block.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  ADC1_backup:  The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void ADC1_Wakeup(void) 
{
    /* Restore the configuration */
    ADC1_RestoreConfig();

    /* Enables the component operation */
    if(ADC1_backup.enableState != ADC1_DISABLED)
    {
        ADC1_Enable();
        if((ADC1_backup.enableState & ADC1_STARTED) != 0u)
        {
            ADC1_StartConvert();
        }
    } /* Do nothing if component was disable before */
}


/* [] END OF FILE */
