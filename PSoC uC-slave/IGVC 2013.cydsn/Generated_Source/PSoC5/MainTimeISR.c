/*******************************************************************************
* File Name: MainTimeISR.c  
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
#include <MainTimeISR.H>


/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START MainTimeISR_intc` */

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
* Function Name: MainTimeISR_Start
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
void MainTimeISR_Start(void)
{
    /* For all we know the interrupt is active. */
    MainTimeISR_Disable();

    /* Set the ISR to point to the MainTimeISR Interrupt. */
    MainTimeISR_SetVector(MainTimeISR_Interrupt);

    /* Set the priority. */
    MainTimeISR_SetPriority(MainTimeISR_INTC_PRIOR_NUMBER);

    /* Enable it. */
    MainTimeISR_Enable();
}

/*******************************************************************************
* Function Name: MainTimeISR_StartEx
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
void MainTimeISR_StartEx(cyisraddress address)
{
    /* For all we know the interrupt is active. */
    MainTimeISR_Disable();

    /* Set the ISR to point to the MainTimeISR Interrupt. */
    MainTimeISR_SetVector(address);

    /* Set the priority. */
    MainTimeISR_SetPriority(MainTimeISR_INTC_PRIOR_NUMBER);

    /* Enable it. */
    MainTimeISR_Enable();
}

/*******************************************************************************
* Function Name: MainTimeISR_Stop
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
void MainTimeISR_Stop(void) 
{
    /* Disable this interrupt. */
    MainTimeISR_Disable();

    /* Set the ISR to point to the passive one. */
    MainTimeISR_SetVector(IntDefaultHandler);
}

/*******************************************************************************
* Function Name: MainTimeISR_Interrupt
********************************************************************************
* Summary:
*   The default Interrupt Service Routine for MainTimeISR.
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
CY_ISR(MainTimeISR_Interrupt)
{
    /*  Place your Interrupt code here. */
    /* `#START MainTimeISR_Interrupt` */

    /* `#END` */
}

/*******************************************************************************
* Function Name: MainTimeISR_SetVector
********************************************************************************
* Summary:
*   Change the ISR vector for the Interrupt. Note calling MainTimeISR_Start
*   will override any effect this method would have had. To set the vector before
*   the component has been started use MainTimeISR_StartEx instead.
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
void MainTimeISR_SetVector(cyisraddress address) 
{
    cyisraddress * ramVectorTable;

    ramVectorTable = (cyisraddress *) *CYINT_VECT_TABLE;

    ramVectorTable[CYINT_IRQ_BASE + MainTimeISR__INTC_NUMBER] = address;
}

/*******************************************************************************
* Function Name: MainTimeISR_GetVector
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
cyisraddress MainTimeISR_GetVector(void) 
{
    cyisraddress * ramVectorTable;

    ramVectorTable = (cyisraddress *) *CYINT_VECT_TABLE;

    return ramVectorTable[CYINT_IRQ_BASE + MainTimeISR__INTC_NUMBER];
}

/*******************************************************************************
* Function Name: MainTimeISR_SetPriority
********************************************************************************
* Summary:
*   Sets the Priority of the Interrupt. Note calling MainTimeISR_Start
*   or MainTimeISR_StartEx will override any effect this method would have had. 
*	This method should only be called after MainTimeISR_Start or 
*	MainTimeISR_StartEx has been called. To set the initial
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
void MainTimeISR_SetPriority(uint8 priority) 
{
    *MainTimeISR_INTC_PRIOR = priority << 5;
}

/*******************************************************************************
* Function Name: MainTimeISR_GetPriority
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
uint8 MainTimeISR_GetPriority(void) 
{
    uint8 priority;


    priority = *MainTimeISR_INTC_PRIOR >> 5;

    return priority;
}

/*******************************************************************************
* Function Name: MainTimeISR_Enable
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
void MainTimeISR_Enable(void) 
{
    /* Enable the general interrupt. */
    *MainTimeISR_INTC_SET_EN = MainTimeISR__INTC_MASK;
}

/*******************************************************************************
* Function Name: MainTimeISR_GetState
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
uint8 MainTimeISR_GetState(void) 
{
    /* Get the state of the general interrupt. */
    return (*MainTimeISR_INTC_SET_EN & MainTimeISR__INTC_MASK) ? 1:0;
}

/*******************************************************************************
* Function Name: MainTimeISR_Disable
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
void MainTimeISR_Disable(void) 
{
    /* Disable the general interrupt. */
    *MainTimeISR_INTC_CLR_EN = MainTimeISR__INTC_MASK;
}

/*******************************************************************************
* Function Name: MainTimeISR_SetPending
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
void MainTimeISR_SetPending(void) 
{
    *MainTimeISR_INTC_SET_PD = MainTimeISR__INTC_MASK;
}

/*******************************************************************************
* Function Name: MainTimeISR_ClearPending
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
void MainTimeISR_ClearPending(void) 
{
    *MainTimeISR_INTC_CLR_PD = MainTimeISR__INTC_MASK;
}
