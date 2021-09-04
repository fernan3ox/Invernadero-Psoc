/*******************************************************************************
* File Name: Cooler.h  
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

#if !defined(CY_PINS_Cooler_H) /* Pins Cooler_H */
#define CY_PINS_Cooler_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Cooler_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Cooler__PORT == 15 && ((Cooler__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    Cooler_Write(uint8 value);
void    Cooler_SetDriveMode(uint8 mode);
uint8   Cooler_ReadDataReg(void);
uint8   Cooler_Read(void);
void    Cooler_SetInterruptMode(uint16 position, uint16 mode);
uint8   Cooler_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the Cooler_SetDriveMode() function.
     *  @{
     */
        #define Cooler_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define Cooler_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define Cooler_DM_RES_UP          PIN_DM_RES_UP
        #define Cooler_DM_RES_DWN         PIN_DM_RES_DWN
        #define Cooler_DM_OD_LO           PIN_DM_OD_LO
        #define Cooler_DM_OD_HI           PIN_DM_OD_HI
        #define Cooler_DM_STRONG          PIN_DM_STRONG
        #define Cooler_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define Cooler_MASK               Cooler__MASK
#define Cooler_SHIFT              Cooler__SHIFT
#define Cooler_WIDTH              1u

/* Interrupt constants */
#if defined(Cooler__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Cooler_SetInterruptMode() function.
     *  @{
     */
        #define Cooler_INTR_NONE      (uint16)(0x0000u)
        #define Cooler_INTR_RISING    (uint16)(0x0001u)
        #define Cooler_INTR_FALLING   (uint16)(0x0002u)
        #define Cooler_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define Cooler_INTR_MASK      (0x01u) 
#endif /* (Cooler__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Cooler_PS                     (* (reg8 *) Cooler__PS)
/* Data Register */
#define Cooler_DR                     (* (reg8 *) Cooler__DR)
/* Port Number */
#define Cooler_PRT_NUM                (* (reg8 *) Cooler__PRT) 
/* Connect to Analog Globals */                                                  
#define Cooler_AG                     (* (reg8 *) Cooler__AG)                       
/* Analog MUX bux enable */
#define Cooler_AMUX                   (* (reg8 *) Cooler__AMUX) 
/* Bidirectional Enable */                                                        
#define Cooler_BIE                    (* (reg8 *) Cooler__BIE)
/* Bit-mask for Aliased Register Access */
#define Cooler_BIT_MASK               (* (reg8 *) Cooler__BIT_MASK)
/* Bypass Enable */
#define Cooler_BYP                    (* (reg8 *) Cooler__BYP)
/* Port wide control signals */                                                   
#define Cooler_CTL                    (* (reg8 *) Cooler__CTL)
/* Drive Modes */
#define Cooler_DM0                    (* (reg8 *) Cooler__DM0) 
#define Cooler_DM1                    (* (reg8 *) Cooler__DM1)
#define Cooler_DM2                    (* (reg8 *) Cooler__DM2) 
/* Input Buffer Disable Override */
#define Cooler_INP_DIS                (* (reg8 *) Cooler__INP_DIS)
/* LCD Common or Segment Drive */
#define Cooler_LCD_COM_SEG            (* (reg8 *) Cooler__LCD_COM_SEG)
/* Enable Segment LCD */
#define Cooler_LCD_EN                 (* (reg8 *) Cooler__LCD_EN)
/* Slew Rate Control */
#define Cooler_SLW                    (* (reg8 *) Cooler__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Cooler_PRTDSI__CAPS_SEL       (* (reg8 *) Cooler__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Cooler_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Cooler__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Cooler_PRTDSI__OE_SEL0        (* (reg8 *) Cooler__PRTDSI__OE_SEL0) 
#define Cooler_PRTDSI__OE_SEL1        (* (reg8 *) Cooler__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Cooler_PRTDSI__OUT_SEL0       (* (reg8 *) Cooler__PRTDSI__OUT_SEL0) 
#define Cooler_PRTDSI__OUT_SEL1       (* (reg8 *) Cooler__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Cooler_PRTDSI__SYNC_OUT       (* (reg8 *) Cooler__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(Cooler__SIO_CFG)
    #define Cooler_SIO_HYST_EN        (* (reg8 *) Cooler__SIO_HYST_EN)
    #define Cooler_SIO_REG_HIFREQ     (* (reg8 *) Cooler__SIO_REG_HIFREQ)
    #define Cooler_SIO_CFG            (* (reg8 *) Cooler__SIO_CFG)
    #define Cooler_SIO_DIFF           (* (reg8 *) Cooler__SIO_DIFF)
#endif /* (Cooler__SIO_CFG) */

/* Interrupt Registers */
#if defined(Cooler__INTSTAT)
    #define Cooler_INTSTAT            (* (reg8 *) Cooler__INTSTAT)
    #define Cooler_SNAP               (* (reg8 *) Cooler__SNAP)
    
	#define Cooler_0_INTTYPE_REG 		(* (reg8 *) Cooler__0__INTTYPE)
#endif /* (Cooler__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Cooler_H */


/* [] END OF FILE */
