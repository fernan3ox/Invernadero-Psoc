/*******************************************************************************
* File Name: PWMO.c  
* Version 2.20
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "PWMO.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 PWMO__PORT == 15 && ((PWMO__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: PWMO_Write
****************************************************************************//**
*
* \brief Writes the value to the physical port (data output register), masking
*  and shifting the bits appropriately. 
*
* The data output register controls the signal applied to the physical pin in 
* conjunction with the drive mode parameter. This function avoids changing 
* other bits in the port by using the appropriate method (read-modify-write or
* bit banding).
*
* <b>Note</b> This function should not be used on a hardware digital output pin 
* as it is driven by the hardware signal attached to it.
*
* \param value
*  Value to write to the component instance.
*
* \return 
*  None 
*
* \sideeffect
*  If you use read-modify-write operations that are not atomic; the Interrupt 
*  Service Routines (ISR) can cause corruption of this function. An ISR that 
*  interrupts this function and performs writes to the Pins component data 
*  register can cause corrupted port data. To avoid this issue, you should 
*  either use the Per-Pin APIs (primary method) or disable interrupts around 
*  this function.
*
* \funcusage
*  \snippet PWMO_SUT.c usage_PWMO_Write
*******************************************************************************/
void PWMO_Write(uint8 value)
{
    uint8 staticBits = (PWMO_DR & (uint8)(~PWMO_MASK));
    PWMO_DR = staticBits | ((uint8)(value << PWMO_SHIFT) & PWMO_MASK);
}


/*******************************************************************************
* Function Name: PWMO_SetDriveMode
****************************************************************************//**
*
* \brief Sets the drive mode for each of the Pins component's pins.
* 
* <b>Note</b> This affects all pins in the Pins component instance. Use the
* Per-Pin APIs if you wish to control individual pin's drive modes.
*
* \param mode
*  Mode for the selected signals. Valid options are documented in 
*  \ref driveMode.
*
* \return
*  None
*
* \sideeffect
*  If you use read-modify-write operations that are not atomic, the ISR can
*  cause corruption of this function. An ISR that interrupts this function 
*  and performs writes to the Pins component Drive Mode registers can cause 
*  corrupted port data. To avoid this issue, you should either use the Per-Pin
*  APIs (primary method) or disable interrupts around this function.
*
* \funcusage
*  \snippet PWMO_SUT.c usage_PWMO_SetDriveMode
*******************************************************************************/
void PWMO_SetDriveMode(uint8 mode)
{
	CyPins_SetPinDriveMode(PWMO_0, mode);
}


/*******************************************************************************
* Function Name: PWMO_Read
****************************************************************************//**
*
* \brief Reads the associated physical port (pin status register) and masks 
*  the required bits according to the width and bit position of the component
*  instance. 
*
* The pin's status register returns the current logic level present on the 
* physical pin.
*
* \return 
*  The current value for the pins in the component as a right justified number.
*
* \funcusage
*  \snippet PWMO_SUT.c usage_PWMO_Read  
*******************************************************************************/
uint8 PWMO_Read(void)
{
    return (PWMO_PS & PWMO_MASK) >> PWMO_SHIFT;
}


/*******************************************************************************
* Function Name: PWMO_ReadDataReg
****************************************************************************//**
*
* \brief Reads the associated physical port's data output register and masks 
*  the correct bits according to the width and bit position of the component 
*  instance. 
*
* The data output register controls the signal applied to the physical pin in 
* conjunction with the drive mode parameter. This is not the same as the 
* preferred PWMO_Read() API because the 
* PWMO_ReadDataReg() reads the data register instead of the status 
* register. For output pins this is a useful function to determine the value 
* just written to the pin.
*
* \return 
*  The current value of the data register masked and shifted into a right 
*  justified number for the component instance.
*
* \funcusage
*  \snippet PWMO_SUT.c usage_PWMO_ReadDataReg 
*******************************************************************************/
uint8 PWMO_ReadDataReg(void)
{
    return (PWMO_DR & PWMO_MASK) >> PWMO_SHIFT;
}


/* If interrupt is connected for this Pins component */ 
#if defined(PWMO_INTSTAT) 

    /*******************************************************************************
    * Function Name: PWMO_SetInterruptMode
    ****************************************************************************//**
    *
    * \brief Configures the interrupt mode for each of the Pins component's
    *  pins. Alternatively you may set the interrupt mode for all the pins
    *  specified in the Pins component.
    *
    *  <b>Note</b> The interrupt is port-wide and therefore any enabled pin
    *  interrupt may trigger it.
    *
    * \param position
    *  The pin position as listed in the Pins component. You may OR these to be 
    *  able to configure the interrupt mode of multiple pins within a Pins 
    *  component. Or you may use PWMO_INTR_ALL to configure the
    *  interrupt mode of all the pins in the Pins component.       
    *  - PWMO_0_INTR       (First pin in the list)
    *  - PWMO_1_INTR       (Second pin in the list)
    *  - ...
    *  - PWMO_INTR_ALL     (All pins in Pins component)
    *
    * \param mode
    *  Interrupt mode for the selected pins. Valid options are documented in
    *  \ref intrMode.
    *
    * \return 
    *  None
    *  
    * \sideeffect
    *  It is recommended that the interrupt be disabled before calling this 
    *  function to avoid unintended interrupt requests. Note that the interrupt
    *  type is port wide, and therefore will trigger for any enabled pin on the 
    *  port.
    *
    * \funcusage
    *  \snippet PWMO_SUT.c usage_PWMO_SetInterruptMode
    *******************************************************************************/
    void PWMO_SetInterruptMode(uint16 position, uint16 mode)
    {
		if((position & PWMO_0_INTR) != 0u) 
		{ 
			 PWMO_0_INTTYPE_REG = (uint8)mode; 
		}
    }
    
    
    /*******************************************************************************
    * Function Name: PWMO_ClearInterrupt
    ****************************************************************************//**
    *
    * \brief Clears any active interrupts attached with the component and returns 
    *  the value of the interrupt status register allowing determination of which
    *  pins generated an interrupt event.
    *
    * \return 
    *  The right-shifted current value of the interrupt status register. Each pin 
    *  has one bit set if it generated an interrupt event. For example, bit 0 is 
    *  for pin 0 and bit 1 is for pin 1 of the Pins component.
    *  
    * \sideeffect
    *  Clears all bits of the physical port's interrupt status register, not just
    *  those associated with the Pins component.
    *
    * \funcusage
    *  \snippet PWMO_SUT.c usage_PWMO_ClearInterrupt
    *******************************************************************************/
    uint8 PWMO_ClearInterrupt(void)
    {
        return (PWMO_INTSTAT & PWMO_MASK) >> PWMO_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
