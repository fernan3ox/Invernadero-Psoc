/*******************************************************************************
* File Name: isrTX.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_isrTX_H)
#define CY_ISR_isrTX_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void isrTX_Start(void);
void isrTX_StartEx(cyisraddress address);
void isrTX_Stop(void);

CY_ISR_PROTO(isrTX_Interrupt);

void isrTX_SetVector(cyisraddress address);
cyisraddress isrTX_GetVector(void);

void isrTX_SetPriority(uint8 priority);
uint8 isrTX_GetPriority(void);

void isrTX_Enable(void);
uint8 isrTX_GetState(void);
void isrTX_Disable(void);

void isrTX_SetPending(void);
void isrTX_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the isrTX ISR. */
#define isrTX_INTC_VECTOR            ((reg32 *) isrTX__INTC_VECT)

/* Address of the isrTX ISR priority. */
#define isrTX_INTC_PRIOR             ((reg8 *) isrTX__INTC_PRIOR_REG)

/* Priority of the isrTX interrupt. */
#define isrTX_INTC_PRIOR_NUMBER      isrTX__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable isrTX interrupt. */
#define isrTX_INTC_SET_EN            ((reg32 *) isrTX__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the isrTX interrupt. */
#define isrTX_INTC_CLR_EN            ((reg32 *) isrTX__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the isrTX interrupt state to pending. */
#define isrTX_INTC_SET_PD            ((reg32 *) isrTX__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the isrTX interrupt. */
#define isrTX_INTC_CLR_PD            ((reg32 *) isrTX__INTC_CLR_PD_REG)


#endif /* CY_ISR_isrTX_H */


/* [] END OF FILE */
