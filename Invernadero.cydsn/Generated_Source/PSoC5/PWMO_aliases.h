/*******************************************************************************
* File Name: PWMO.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_PWMO_ALIASES_H) /* Pins PWMO_ALIASES_H */
#define CY_PINS_PWMO_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define PWMO_0			(PWMO__0__PC)
#define PWMO_0_INTR	((uint16)((uint16)0x0001u << PWMO__0__SHIFT))

#define PWMO_INTR_ALL	 ((uint16)(PWMO_0_INTR))

#endif /* End Pins PWMO_ALIASES_H */


/* [] END OF FILE */
