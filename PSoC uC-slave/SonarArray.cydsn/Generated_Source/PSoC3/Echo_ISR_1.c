/*******************************************************************************
* File Name: Echo_ISR_1.c  
* Version 1.50
*
*  Description:
*   API for controlling the state of an interrupt.
*
*
*  Note:
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/


#include <CYDEVICE.H>
#include <CYDEVICE_TRM.H>
#include <CYLIB.H>
#include <Echo_ISR_1.H>


/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START Echo_ISR_1_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: Echo_ISR_1_Start
********************************************************************************
* Summary:
*  Set up the interrupt and enable it.
*
* Parameters:  
*   void.
*
*
* Return:
*   void.
*
*******************************************************************************/
void Echo_ISR_1_Start(void)
{
    /* For all we know the interrupt is active. */
    Echo_ISR_1_Disable();

    /* Set the ISR to point to the Echo_ISR_1 Interrupt. */
    Echo_ISR_1_SetVector(Echo_ISR_1_Interrupt);

    /* Set the priority. */
    Echo_ISR_1_SetPriority(Echo_ISR_1_INTC_PRIOR_NUMBER);

    /* Enable it. */
    Echo_ISR_1_Enable();
}

/*******************************************************************************
* Function Name: Echo_ISR_1_StartEx
********************************************************************************
* Summary:
*  Set up the interrupt and enable it.
*
* Parameters:  
*   address: Address of the ISR to set in the interrupt vector table.
*
*
* Return:
*   void.
*
*******************************************************************************/
void Echo_ISR_1_StartEx(cyisraddress address)
{
    /* For all we know the interrupt is active. */
    Echo_ISR_1_Disable();

    /* Set the ISR to point to the Echo_ISR_1 Interrupt. */
    Echo_ISR_1_SetVector(address);

    /* Set the priority. */
    Echo_ISR_1_SetPriority(Echo_ISR_1_INTC_PRIOR_NUMBER);

    /* Enable it. */
    Echo_ISR_1_Enable();
}

/*******************************************************************************
* Function Name: Echo_ISR_1_Stop
********************************************************************************
* Summary:
*   Disables and removes the interrupt.
*
* Parameters:  
*
*
* Return:
*   void.
*
*******************************************************************************/
void Echo_ISR_1_Stop(void) 
{
    /* Disable this interrupt. */
    Echo_ISR_1_Disable();
}

/*******************************************************************************
* Function Name: Echo_ISR_1_Interrupt
********************************************************************************
* Summary:
*   The default Interrupt Service Routine for Echo_ISR_1.
*
*   Add custom code between the coments to keep the next version of this file
*   from over writting your code.
*
*
*
* Parameters:  
*
*
* Return:
*   void.
*
*******************************************************************************/
CY_ISR(Echo_ISR_1_Interrupt)
{
    /*  Place your Interrupt code here. */
    /* `#START Echo_ISR_1_Interrupt` */

    /* `#END` */

    /* PSoC3 ES1, ES2 RTC ISR PATCH  */ 
    #if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
        #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (Echo_ISR_1__ES2_PATCH ))      
            Echo_ISR_1_ISR_PATCH();
        #endif
    #endif
}

/*******************************************************************************
* Function Name: Echo_ISR_1_SetVector
********************************************************************************
* Summary:
*   Change the ISR vector for the Interrupt. Note calling Echo_ISR_1_Start
*   will override any effect this method would have had. To set the vector before
*   the component has been started use Echo_ISR_1_StartEx instead.
*
*
* Parameters:
*   address: Address of the ISR to set in the interrupt vector table.
*
*
* Return:
*   void.
*
*
*******************************************************************************/
void Echo_ISR_1_SetVector(cyisraddress address) 
{
    CY_SET_REG16(Echo_ISR_1_INTC_VECTOR, (uint16) address);
}

/*******************************************************************************
* Function Name: Echo_ISR_1_GetVector
********************************************************************************
* Summary:
*   Gets the "address" of the current ISR vector for the Interrupt.
*
*
* Parameters:
*   void.
*
*
* Return:
*   Address of the ISR in the interrupt vector table.
*
*
*******************************************************************************/
cyisraddress Echo_ISR_1_GetVector(void) 
{
    return (cyisraddress) CY_GET_REG16(Echo_ISR_1_INTC_VECTOR);
}

/*******************************************************************************
* Function Name: Echo_ISR_1_SetPriority
********************************************************************************
* Summary:
*   Sets the Priority of the Interrupt. Note calling Echo_ISR_1_Start
*   or Echo_ISR_1_StartEx will override any effect this method would have had. 
*	This method should only be called after Echo_ISR_1_Start or 
*	Echo_ISR_1_StartEx has been called. To set the initial
*	priority for the component use the cydwr file in the tool.
*
*
* Parameters:
*   priority: Priority of the interrupt. 0 - 7, 0 being the highest.
*
*
* Return:
*   void.
*
*
*******************************************************************************/
void Echo_ISR_1_SetPriority(uint8 priority) 
{
    *Echo_ISR_1_INTC_PRIOR = priority << 5;
}

/*******************************************************************************
* Function Name: Echo_ISR_1_GetPriority
********************************************************************************
* Summary:
*   Gets the Priority of the Interrupt.
*
*
* Parameters:
*   void.
*
*
* Return:
*   Priority of the interrupt. 0 - 7, 0 being the highest.
*
*
*******************************************************************************/
uint8 Echo_ISR_1_GetPriority(void) 
{
    uint8 priority;


    priority = *Echo_ISR_1_INTC_PRIOR >> 5;

    return priority;
}

/*******************************************************************************
* Function Name: Echo_ISR_1_Enable
********************************************************************************
* Summary:
*   Enables the interrupt.
*
*
* Parameters:
*   void.
*
*
* Return:
*   void.
*
*
*******************************************************************************/
void Echo_ISR_1_Enable(void) 
{
    /* Enable the general interrupt. */
    *Echo_ISR_1_INTC_SET_EN = Echo_ISR_1__INTC_MASK;
}

/*******************************************************************************
* Function Name: Echo_ISR_1_GetState
********************************************************************************
* Summary:
*   Gets the state (enabled, disabled) of the Interrupt.
*
*
* Parameters:
*   void.
*
*
* Return:
*   1 if enabled, 0 if disabled.
*
*
*******************************************************************************/
uint8 Echo_ISR_1_GetState(void) 
{
    /* Get the state of the general interrupt. */
    return (*Echo_ISR_1_INTC_SET_EN & Echo_ISR_1__INTC_MASK) ? 1:0;
}

/*******************************************************************************
* Function Name: Echo_ISR_1_Disable
********************************************************************************
* Summary:
*   Disables the Interrupt.
*
*
* Parameters:
*   void.
*
*
* Return:
*   void.
*
*
*******************************************************************************/
void Echo_ISR_1_Disable(void) 
{
    /* Disable the general interrupt. */
    *Echo_ISR_1_INTC_CLR_EN = Echo_ISR_1__INTC_MASK;
}

/*******************************************************************************
* Function Name: Echo_ISR_1_SetPending
********************************************************************************
* Summary:
*   Causes the Interrupt to enter the pending state, a software method of
*   generating the interrupt.
*
*
* Parameters:
*   void.
*
*
* Return:
*   void.
*
*
*******************************************************************************/
void Echo_ISR_1_SetPending(void) 
{
    *Echo_ISR_1_INTC_SET_PD = Echo_ISR_1__INTC_MASK;
}

/*******************************************************************************
* Function Name: Echo_ISR_1_ClearPending
********************************************************************************
* Summary:
*   Clears a pending interrupt.
*
* Parameters:
*   void.
*
*
* Return:
*   void.
*
*
*******************************************************************************/
void Echo_ISR_1_ClearPending(void) 
{
    *Echo_ISR_1_INTC_CLR_PD = Echo_ISR_1__INTC_MASK;
}
