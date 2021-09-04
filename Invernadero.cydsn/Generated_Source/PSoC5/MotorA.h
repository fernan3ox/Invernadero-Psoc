/*******************************************************************************
* File Name: MotorA.h  
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

#if !defined(CY_PINS_MotorA_H) /* Pins MotorA_H */
#define CY_PINS_MotorA_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "MotorA_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 MotorA__PORT == 15 && ((MotorA__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    MotorA_Write(uint8 value);
void    MotorA_SetDriveMode(uint8 mode);
uint8   MotorA_ReadDataReg(void);
uint8   MotorA_Read(void);
void    MotorA_SetInterruptMode(uint16 position, uint16 mode);
uint8   MotorA_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the MotorA_SetDriveMode() function.
     *  @{
     */
        #define MotorA_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define MotorA_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define MotorA_DM_RES_UP          PIN_DM_RES_UP
        #define MotorA_DM_RES_DWN         PIN_DM_RES_DWN
        #define MotorA_DM_OD_LO           PIN_DM_OD_LO
        #define MotorA_DM_OD_HI           PIN_DM_OD_HI
        #define MotorA_DM_STRONG          PIN_DM_STRONG
        #define MotorA_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define MotorA_MASK               MotorA__MASK
#define MotorA_SHIFT              MotorA__SHIFT
#define MotorA_WIDTH              1u

/* Interrupt constants */
#if defined(MotorA__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in MotorA_SetInterruptMode() function.
     *  @{
     */
        #define MotorA_INTR_NONE      (uint16)(0x0000u)
        #define MotorA_INTR_RISING    (uint16)(0x0001u)
        #define MotorA_INTR_FALLING   (uint16)(0x0002u)
        #define MotorA_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define MotorA_INTR_MASK      (0x01u) 
#endif /* (MotorA__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MotorA_PS                     (* (reg8 *) MotorA__PS)
/* Data Register */
#define MotorA_DR                     (* (reg8 *) MotorA__DR)
/* Port Number */
#define MotorA_PRT_NUM                (* (reg8 *) MotorA__PRT) 
/* Connect to Analog Globals */                                                  
#define MotorA_AG                     (* (reg8 *) MotorA__AG)                       
/* Analog MUX bux enable */
#define MotorA_AMUX                   (* (reg8 *) MotorA__AMUX) 
/* Bidirectional Enable */                                                        
#define MotorA_BIE                    (* (reg8 *) MotorA__BIE)
/* Bit-mask for Aliased Register Access */
#define MotorA_BIT_MASK               (* (reg8 *) MotorA__BIT_MASK)
/* Bypass Enable */
#define MotorA_BYP                    (* (reg8 *) MotorA__BYP)
/* Port wide control signals */                                                   
#define MotorA_CTL                    (* (reg8 *) MotorA__CTL)
/* Drive Modes */
#define MotorA_DM0                    (* (reg8 *) MotorA__DM0) 
#define MotorA_DM1                    (* (reg8 *) MotorA__DM1)
#define MotorA_DM2                    (* (reg8 *) MotorA__DM2) 
/* Input Buffer Disable Override */
#define MotorA_INP_DIS                (* (reg8 *) MotorA__INP_DIS)
/* LCD Common or Segment Drive */
#define MotorA_LCD_COM_SEG            (* (reg8 *) MotorA__LCD_COM_SEG)
/* Enable Segment LCD */
#define MotorA_LCD_EN                 (* (reg8 *) MotorA__LCD_EN)
/* Slew Rate Control */
#define MotorA_SLW                    (* (reg8 *) MotorA__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define MotorA_PRTDSI__CAPS_SEL       (* (reg8 *) MotorA__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define MotorA_PRTDSI__DBL_SYNC_IN    (* (reg8 *) MotorA__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define MotorA_PRTDSI__OE_SEL0        (* (reg8 *) MotorA__PRTDSI__OE_SEL0) 
#define MotorA_PRTDSI__OE_SEL1        (* (reg8 *) MotorA__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define MotorA_PRTDSI__OUT_SEL0       (* (reg8 *) MotorA__PRTDSI__OUT_SEL0) 
#define MotorA_PRTDSI__OUT_SEL1       (* (reg8 *) MotorA__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define MotorA_PRTDSI__SYNC_OUT       (* (reg8 *) MotorA__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(MotorA__SIO_CFG)
    #define MotorA_SIO_HYST_EN        (* (reg8 *) MotorA__SIO_HYST_EN)
    #define MotorA_SIO_REG_HIFREQ     (* (reg8 *) MotorA__SIO_REG_HIFREQ)
    #define MotorA_SIO_CFG            (* (reg8 *) MotorA__SIO_CFG)
    #define MotorA_SIO_DIFF           (* (reg8 *) MotorA__SIO_DIFF)
#endif /* (MotorA__SIO_CFG) */

/* Interrupt Registers */
#if defined(MotorA__INTSTAT)
    #define MotorA_INTSTAT            (* (reg8 *) MotorA__INTSTAT)
    #define MotorA_SNAP               (* (reg8 *) MotorA__SNAP)
    
	#define MotorA_0_INTTYPE_REG 		(* (reg8 *) MotorA__0__INTTYPE)
#endif /* (MotorA__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_MotorA_H */


/* [] END OF FILE */
