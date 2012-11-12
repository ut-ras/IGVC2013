/*******************************************************************************
* File Name: IMU_Interrupt.c  
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
#include <IMU_Interrupt.H>


/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START IMU_Interrupt_intc` */

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
* Function Name: IMU_Interrupt_Start
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
void IMU_Interrupt_Start(void)
{
    /* For all we know the interrupt is active. */
    IMU_Interrupt_Disable();

    /* Set the ISR to point to the IMU_Interrupt Interrupt. */
    IMU_Interrupt_SetVector(IMU_Interrupt_Interrupt);

    /* Set the priority. */
    IMU_Interrupt_SetPriority(IMU_Interrupt_INTC_PRIOR_NUMBER);

    /* Enable it. */
    IMU_Interrupt_Enable();
}

/*******************************************************************************
* Function Name: IMU_Interrupt_StartEx
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
void IMU_Interrupt_StartEx(cyisraddress address)
{
    /* For all we know the interrupt is active. */
    IMU_Interrupt_Disable();

    /* Set the ISR to point to the IMU_Interrupt Interrupt. */
    IMU_Interrupt_SetVector(address);

    /* Set the priority. */
    IMU_Interrupt_SetPriority(IMU_Interrupt_INTC_PRIOR_NUMBER);

    /* Enable it. */
    IMU_Interrupt_Enable();
}

/*******************************************************************************
* Function Name: IMU_Interrupt_Stop
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
void IMU_Interrupt_Stop(void) 
{
    /* Disable this interrupt. */
    IMU_Interrupt_Disable();

    /* Set the ISR to point to the passive one. */
    IMU_Interrupt_SetVector(IntDefaultHandler);
}

/*******************************************************************************
* Function Name: IMU_Interrupt_Interrupt
********************************************************************************
* Summary:
*   The default Interrupt Service Routine for IMU_Interrupt.
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
CY_ISR(IMU_Interrupt_Interrupt)
{
    /*  Place your Interrupt code here. */
    /* `#START IMU_Interrupt_Interrupt` */

    /* `#END` */
}

/*******************************************************************************
* Function Name: IMU_Interrupt_SetVector
********************************************************************************
* Summary:
*   Change the ISR vector for the Interrupt. Note calling IMU_Interrupt_Start
*   will override any effect this method would have had. To set the vector before
*   the component has been started use IMU_Interrupt_StartEx instead.
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
void IMU_Interrupt_SetVector(cyisraddress address) 
{
    cyisraddress * ramVectorTable;

    ramVectorTable = (cyisraddress *) *CYINT_VECT_TABLE;

    ramVectorTable[CYINT_IRQ_BASE + IMU_Interrupt__INTC_NUMBER] = address;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_GetVector
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
cyisraddress IMU_Interrupt_GetVector(void) 
{
    cyisraddress * ramVectorTable;

    ramVectorTable = (cyisraddress *) *CYINT_VECT_TABLE;

    return ramVectorTable[CYINT_IRQ_BASE + IMU_Interrupt__INTC_NUMBER];
}

/*******************************************************************************
* Function Name: IMU_Interrupt_SetPriority
********************************************************************************
* Summary:
*   Sets the Priority of the Interrupt. Note calling IMU_Interrupt_Start
*   or IMU_Interrupt_StartEx will override any effect this method would have had. 
*	This method should only be called after IMU_Interrupt_Start or 
*	IMU_Interrupt_StartEx has been called. To set the initial
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
void IMU_Interrupt_SetPriority(uint8 priority) 
{
    *IMU_Interrupt_INTC_PRIOR = priority << 5;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_GetPriority
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
uint8 IMU_Interrupt_GetPriority(void) 
{
    uint8 priority;


    priority = *IMU_Interrupt_INTC_PRIOR >> 5;

    return priority;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_Enable
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
void IMU_Interrupt_Enable(void) 
{
    /* Enable the general interrupt. */
    *IMU_Interrupt_INTC_SET_EN = IMU_Interrupt__INTC_MASK;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_GetState
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
uint8 IMU_Interrupt_GetState(void) 
{
    /* Get the state of the general interrupt. */
    return (*IMU_Interrupt_INTC_SET_EN & IMU_Interrupt__INTC_MASK) ? 1:0;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_Disable
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
void IMU_Interrupt_Disable(void) 
{
    /* Disable the general interrupt. */
    *IMU_Interrupt_INTC_CLR_EN = IMU_Interrupt__INTC_MASK;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_SetPending
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
void IMU_Interrupt_SetPending(void) 
{
    *IMU_Interrupt_INTC_SET_PD = IMU_Interrupt__INTC_MASK;
}

/*******************************************************************************
* Function Name: IMU_Interrupt_ClearPending
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
void IMU_Interrupt_ClearPending(void) 
{
    *IMU_Interrupt_INTC_CLR_PD = IMU_Interrupt__INTC_MASK;
}
