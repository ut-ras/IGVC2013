/*******************************************************************************
* File Name: Left_Encoder_isr.c  
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
#include <Left_Encoder_isr.H>


/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START Left_Encoder_isr_intc` */

/* `#END` */

#ifndef CYINT_IRQ_BASE
#define CYINT_IRQ_BASE	16
#endif
#ifndef CYINT_VECT_TABLE
#define CYINT_VECT_TABLE    ((cyisraddress **) CYREG_NVIC_VECT_OFFSET)
#endif

/* Declared in startup, used to set unused interrupts to. */
CY_ISR_PROTO(IntDefaultHandler);

/*******************************************************************************
* Function Name: Left_Encoder_isr_Start
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
void Left_Encoder_isr_Start(void)
{
    /* For all we know the interrupt is active. */
    Left_Encoder_isr_Disable();

    /* Set the ISR to point to the Left_Encoder_isr Interrupt. */
    Left_Encoder_isr_SetVector(Left_Encoder_isr_Interrupt);

    /* Set the priority. */
    Left_Encoder_isr_SetPriority(Left_Encoder_isr_INTC_PRIOR_NUMBER);

    /* Enable it. */
    Left_Encoder_isr_Enable();
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_StartEx
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
void Left_Encoder_isr_StartEx(cyisraddress address)
{
    /* For all we know the interrupt is active. */
    Left_Encoder_isr_Disable();

    /* Set the ISR to point to the Left_Encoder_isr Interrupt. */
    Left_Encoder_isr_SetVector(address);

    /* Set the priority. */
    Left_Encoder_isr_SetPriority(Left_Encoder_isr_INTC_PRIOR_NUMBER);

    /* Enable it. */
    Left_Encoder_isr_Enable();
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_Stop
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
void Left_Encoder_isr_Stop(void) 
{
    /* Disable this interrupt. */
    Left_Encoder_isr_Disable();

    /* Set the ISR to point to the passive one. */
    Left_Encoder_isr_SetVector(IntDefaultHandler);
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_Interrupt
********************************************************************************
* Summary:
*   The default Interrupt Service Routine for Left_Encoder_isr.
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
CY_ISR(Left_Encoder_isr_Interrupt)
{
    /*  Place your Interrupt code here. */
    /* `#START Left_Encoder_isr_Interrupt` */

    /* `#END` */
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_SetVector
********************************************************************************
* Summary:
*   Change the ISR vector for the Interrupt. Note calling Left_Encoder_isr_Start
*   will override any effect this method would have had. To set the vector before
*   the component has been started use Left_Encoder_isr_StartEx instead.
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
void Left_Encoder_isr_SetVector(cyisraddress address) 
{
    cyisraddress * ramVectorTable;

    ramVectorTable = (cyisraddress *) *CYINT_VECT_TABLE;

    ramVectorTable[CYINT_IRQ_BASE + Left_Encoder_isr__INTC_NUMBER] = address;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_GetVector
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
cyisraddress Left_Encoder_isr_GetVector(void) 
{
    cyisraddress * ramVectorTable;

    ramVectorTable = (cyisraddress *) *CYINT_VECT_TABLE;

    return ramVectorTable[CYINT_IRQ_BASE + Left_Encoder_isr__INTC_NUMBER];
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_SetPriority
********************************************************************************
* Summary:
*   Sets the Priority of the Interrupt. Note calling Left_Encoder_isr_Start
*   or Left_Encoder_isr_StartEx will override any effect this method would have had. 
*	This method should only be called after Left_Encoder_isr_Start or 
*	Left_Encoder_isr_StartEx has been called. To set the initial
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
void Left_Encoder_isr_SetPriority(uint8 priority) 
{
    *Left_Encoder_isr_INTC_PRIOR = priority << 5;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_GetPriority
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
uint8 Left_Encoder_isr_GetPriority(void) 
{
    uint8 priority;


    priority = *Left_Encoder_isr_INTC_PRIOR >> 5;

    return priority;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_Enable
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
void Left_Encoder_isr_Enable(void) 
{
    /* Enable the general interrupt. */
    *Left_Encoder_isr_INTC_SET_EN = Left_Encoder_isr__INTC_MASK;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_GetState
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
uint8 Left_Encoder_isr_GetState(void) 
{
    /* Get the state of the general interrupt. */
    return (*Left_Encoder_isr_INTC_SET_EN & Left_Encoder_isr__INTC_MASK) ? 1:0;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_Disable
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
void Left_Encoder_isr_Disable(void) 
{
    /* Disable the general interrupt. */
    *Left_Encoder_isr_INTC_CLR_EN = Left_Encoder_isr__INTC_MASK;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_SetPending
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
void Left_Encoder_isr_SetPending(void) 
{
    *Left_Encoder_isr_INTC_SET_PD = Left_Encoder_isr__INTC_MASK;
}

/*******************************************************************************
* Function Name: Left_Encoder_isr_ClearPending
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
void Left_Encoder_isr_ClearPending(void) 
{
    *Left_Encoder_isr_INTC_CLR_PD = Left_Encoder_isr__INTC_MASK;
}
