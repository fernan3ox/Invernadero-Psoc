/*******************************************************************************
* File Name: ADC1_AMux.h
* Version 1.80
*
*  Description:
*    This file contains the constants and function prototypes for the Analog
*    Multiplexer User Module AMux.
*
*   Note:
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_AMUX_ADC1_AMux_H)
#define CY_AMUX_ADC1_AMux_H

#include "cyfitter.h"
#include "cyfitter_cfg.h"

#if ((CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3) || \
         (CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC4) || \
         (CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC5))    
    #include "cytypes.h"
#else
    #include "syslib/cy_syslib.h"
#endif /* ((CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3) */


/***************************************
*        Function Prototypes
***************************************/

void ADC1_AMux_Start(void) ;
#define ADC1_AMux_Init() ADC1_AMux_Start()
void ADC1_AMux_FastSelect(uint8 channel) ;
/* The Stop, Select, Connect, Disconnect and DisconnectAll functions are declared elsewhere */
/* void ADC1_AMux_Stop(void); */
/* void ADC1_AMux_Select(uint8 channel); */
/* void ADC1_AMux_Connect(uint8 channel); */
/* void ADC1_AMux_Disconnect(uint8 channel); */
/* void ADC1_AMux_DisconnectAll(void) */


/***************************************
*         Parameter Constants
***************************************/

#define ADC1_AMux_CHANNELS  2u
#define ADC1_AMux_MUXTYPE   1
#define ADC1_AMux_ATMOSTONE 0

/***************************************
*             API Constants
***************************************/

#define ADC1_AMux_NULL_CHANNEL 0xFFu
#define ADC1_AMux_MUX_SINGLE   1
#define ADC1_AMux_MUX_DIFF     2


/***************************************
*        Conditional Functions
***************************************/

#if ADC1_AMux_MUXTYPE == ADC1_AMux_MUX_SINGLE
# if !ADC1_AMux_ATMOSTONE
#  define ADC1_AMux_Connect(channel) ADC1_AMux_Set(channel)
# endif
# define ADC1_AMux_Disconnect(channel) ADC1_AMux_Unset(channel)
#else
# if !ADC1_AMux_ATMOSTONE
void ADC1_AMux_Connect(uint8 channel) ;
# endif
void ADC1_AMux_Disconnect(uint8 channel) ;
#endif

#if ADC1_AMux_ATMOSTONE
# define ADC1_AMux_Stop() ADC1_AMux_DisconnectAll()
# define ADC1_AMux_Select(channel) ADC1_AMux_FastSelect(channel)
void ADC1_AMux_DisconnectAll(void) ;
#else
# define ADC1_AMux_Stop() ADC1_AMux_Start()
void ADC1_AMux_Select(uint8 channel) ;
# define ADC1_AMux_DisconnectAll() ADC1_AMux_Start()
#endif

#endif /* CY_AMUX_ADC1_AMux_H */


/* [] END OF FILE */
