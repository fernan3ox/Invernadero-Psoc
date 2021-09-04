/*******************************************************************************
* File Name: MotorB.h  
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

#if !defined(CY_PINS_MotorB_H) /* Pins MotorB_H */
#define CY_PINS_MotorB_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "MotorB_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 MotorB__PORT == 15 && ((MotorB__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    MotorB_Write(uint8 value);
void    MotorB_SetDriveMode(uint8 mode);
uint8   MotorB_ReadDataReg(void);
uint8   MotorB_Read(void);
void    MotorB_SetInterruptMode(uint16 position, uint16 mode);
uint8   MotorB_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the MotorB_SetDriveMode() function.
     *  @{
     */
        #define MotorB_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define MotorB_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define MotorB_DM_RES_UP          PIN_DM_RES_UP
        #define MotorB_DM_RES_DWN         PIN_DM_RES_DWN
        #define MotorB_DM_OD_LO           PIN_DM_OD_LO
        #define MotorB_DM_OD_HI           PIN_DM_OD_HI
        #define MotorB_DM_STRONG          PIN_DM_STRONG
        #define MotorB_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define MotorB_MASK               MotorB__MASK
#define MotorB_SHIFT              MotorB__SHIFT
#define MotorB_WIDTH              1u

/* Interrupt constants */
#if defined(MotorB__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in MotorB_SetInterruptMode() function.
     *  @{
     */
        #define MotorB_INTR_NONE      (uint16)(0x0000u)
        #define MotorB_INTR_RISING    (uint16)(0x0001u)
        #define MotorB_INTR_FALLING   (uint16)(0x0002u)
        #define MotorB_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define MotorB_INTR_MASK      (0x01u) 
#endif /* (MotorB__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MotorB_PS                     (* (reg8 *) MotorB__PS)
/* Data Register */
#define MotorB_DR                     (* (reg8 *) MotorB__DR)
/* Port Number */
#define MotorB_PRT_NUM                (* (reg8 *) MotorB__PRT) 
/* Connect to Analog Globals */                                                  
#define MotorB_AG                     (* (reg8 *) MotorB__AG)                       
/* Analog MUX bux enable */
#define MotorB_AMUX                   (* (reg8 *) MotorB__AMUX) 
/* Bidirectional Enable */                                                        
#define MotorB_BIE                    (* (reg8 *) MotorB__BIE)
/* Bit-mask for Aliased Register Access */
#define MotorB_BIT_MASK               (* (reg8 *) MotorB__BIT_MASK)
/* Bypass Enable */
#define MotorB_BYP                    (* (reg8 *) MotorB__BYP)
/* Port wide control signals */                                                   
#define MotorB_CTL                    (* (reg8 *) MotorB__CTL)
/* Drive Modes */
#define MotorB_DM0                    (* (reg8 *) MotorB__DM0) 
#define MotorB_DM1                    (* (reg8 *) MotorB__DM1)
#define MotorB_DM2                    (* (reg8 *) MotorB__DM2) 
/* Input Buffer Disable Override */
#define MotorB_INP_DIS                (* (reg8 *) MotorB__INP_DIS)
/* LCD Common or Segment Drive */
#define MotorB_LCD_COM_SEG            (* (reg8 *) MotorB__LCD_COM_SEG)
/* Enable Segment LCD */
#define MotorB_LCD_EN                 (* (reg8 *) MotorB__LCD_EN)
/* Slew Rate Control */
#define MotorB_SLW                    (* (reg8 *) MotorB__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define MotorB_PRTDSI__CAPS_SEL       (* (reg8 *) MotorB__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define MotorB_PRTDSI__DBL_SYNC_IN    (* (reg8 *) MotorB__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define MotorB_PRTDSI__OE_SEL0        (* (reg8 *) MotorB__PRTDSI__OE_SEL0) 
#define MotorB_PRTDSI__OE_SEL1        (* (reg8 *) MotorB__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define MotorB_PRTDSI__OUT_SEL0       (* (reg8 *) MotorB__PRTDSI__OUT_SEL0) 
#define MotorB_PRTDSI__OUT_SEL1       (* (reg8 *) MotorB__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define MotorB_PRTDSI__SYNC_OUT       (* (reg8 *) MotorB__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(MotorB__SIO_CFG)
    #define MotorB_SIO_HYST_EN        (* (reg8 *) MotorB__SIO_HYST_EN)
    #define MotorB_SIO_REG_HIFREQ     (* (reg8 *) MotorB__SIO_REG_HIFREQ)
    #define MotorB_SIO_CFG            (* (reg8 *) MotorB__SIO_CFG)
    #define MotorB_SIO_DIFF           (* (reg8 *) MotorB__SIO_DIFF)
#endif /* (MotorB__SIO_CFG) */

/* Interrupt Registers */
#if defined(MotorB__INTSTAT)
    #define MotorB_INTSTAT            (* (reg8 *) MotorB__INTSTAT)
    #define MotorB_SNAP               (* (reg8 *) MotorB__SNAP)
    
	#define MotorB_0_INTTYPE_REG 		(* (reg8 *) MotorB__0__INTTYPE)
#endif /* (MotorB__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_MotorB_H */


/* [] END OF FILE */
