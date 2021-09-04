/*******************************************************************************
* File Name: PWMO.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_PWMO_H) /* Pins PWMO_H */
#define CY_PINS_PWMO_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "PWMO_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 PWMO__PORT == 15 && ((PWMO__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    PWMO_Write(uint8 value);
void    PWMO_SetDriveMode(uint8 mode);
uint8   PWMO_ReadDataReg(void);
uint8   PWMO_Read(void);
void    PWMO_SetInterruptMode(uint16 position, uint16 mode);
uint8   PWMO_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the PWMO_SetDriveMode() function.
     *  @{
     */
        #define PWMO_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define PWMO_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define PWMO_DM_RES_UP          PIN_DM_RES_UP
        #define PWMO_DM_RES_DWN         PIN_DM_RES_DWN
        #define PWMO_DM_OD_LO           PIN_DM_OD_LO
        #define PWMO_DM_OD_HI           PIN_DM_OD_HI
        #define PWMO_DM_STRONG          PIN_DM_STRONG
        #define PWMO_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define PWMO_MASK               PWMO__MASK
#define PWMO_SHIFT              PWMO__SHIFT
#define PWMO_WIDTH              1u

/* Interrupt constants */
#if defined(PWMO__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in PWMO_SetInterruptMode() function.
     *  @{
     */
        #define PWMO_INTR_NONE      (uint16)(0x0000u)
        #define PWMO_INTR_RISING    (uint16)(0x0001u)
        #define PWMO_INTR_FALLING   (uint16)(0x0002u)
        #define PWMO_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define PWMO_INTR_MASK      (0x01u) 
#endif /* (PWMO__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define PWMO_PS                     (* (reg8 *) PWMO__PS)
/* Data Register */
#define PWMO_DR                     (* (reg8 *) PWMO__DR)
/* Port Number */
#define PWMO_PRT_NUM                (* (reg8 *) PWMO__PRT) 
/* Connect to Analog Globals */                                                  
#define PWMO_AG                     (* (reg8 *) PWMO__AG)                       
/* Analog MUX bux enable */
#define PWMO_AMUX                   (* (reg8 *) PWMO__AMUX) 
/* Bidirectional Enable */                                                        
#define PWMO_BIE                    (* (reg8 *) PWMO__BIE)
/* Bit-mask for Aliased Register Access */
#define PWMO_BIT_MASK               (* (reg8 *) PWMO__BIT_MASK)
/* Bypass Enable */
#define PWMO_BYP                    (* (reg8 *) PWMO__BYP)
/* Port wide control signals */                                                   
#define PWMO_CTL                    (* (reg8 *) PWMO__CTL)
/* Drive Modes */
#define PWMO_DM0                    (* (reg8 *) PWMO__DM0) 
#define PWMO_DM1                    (* (reg8 *) PWMO__DM1)
#define PWMO_DM2                    (* (reg8 *) PWMO__DM2) 
/* Input Buffer Disable Override */
#define PWMO_INP_DIS                (* (reg8 *) PWMO__INP_DIS)
/* LCD Common or Segment Drive */
#define PWMO_LCD_COM_SEG            (* (reg8 *) PWMO__LCD_COM_SEG)
/* Enable Segment LCD */
#define PWMO_LCD_EN                 (* (reg8 *) PWMO__LCD_EN)
/* Slew Rate Control */
#define PWMO_SLW                    (* (reg8 *) PWMO__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define PWMO_PRTDSI__CAPS_SEL       (* (reg8 *) PWMO__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define PWMO_PRTDSI__DBL_SYNC_IN    (* (reg8 *) PWMO__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define PWMO_PRTDSI__OE_SEL0        (* (reg8 *) PWMO__PRTDSI__OE_SEL0) 
#define PWMO_PRTDSI__OE_SEL1        (* (reg8 *) PWMO__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define PWMO_PRTDSI__OUT_SEL0       (* (reg8 *) PWMO__PRTDSI__OUT_SEL0) 
#define PWMO_PRTDSI__OUT_SEL1       (* (reg8 *) PWMO__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define PWMO_PRTDSI__SYNC_OUT       (* (reg8 *) PWMO__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(PWMO__SIO_CFG)
    #define PWMO_SIO_HYST_EN        (* (reg8 *) PWMO__SIO_HYST_EN)
    #define PWMO_SIO_REG_HIFREQ     (* (reg8 *) PWMO__SIO_REG_HIFREQ)
    #define PWMO_SIO_CFG            (* (reg8 *) PWMO__SIO_CFG)
    #define PWMO_SIO_DIFF           (* (reg8 *) PWMO__SIO_DIFF)
#endif /* (PWMO__SIO_CFG) */

/* Interrupt Registers */
#if defined(PWMO__INTSTAT)
    #define PWMO_INTSTAT            (* (reg8 *) PWMO__INTSTAT)
    #define PWMO_SNAP               (* (reg8 *) PWMO__SNAP)
    
	#define PWMO_0_INTTYPE_REG 		(* (reg8 *) PWMO__0__INTTYPE)
#endif /* (PWMO__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_PWMO_H */


/* [] END OF FILE */
