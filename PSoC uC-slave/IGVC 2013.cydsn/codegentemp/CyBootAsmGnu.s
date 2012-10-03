/******************************************************************************
* File Name: CyBootAsmGnu.s
* Version 2.40
*
*  Description:
*   Assembly routines for GNU as.
*
********************************************************************************
* Copyright 2010-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

.syntax unified
.text
.thumb


/*******************************************************************************
* Function Name: CyDelayCycles
********************************************************************************
* Summary:
*   Delays for the specified number of cycles.
*
* Parameters:
*   cycles: number of cycles to delay.
*
* Return:
*   void.
*
*******************************************************************************/
/* void CyDelayCycles(uint32 cycles) */
.align 3                    /* Align to 8 byte boundary (2^n) */
.global CyDelayCycles
.func CyDelayCycles, CyDelayCycles
.type CyDelayCycles, %function
.thumb_func
CyDelayCycles:              /* cycles bytes */
	ADDS r0, r0, #2         /*	1	2	Round to nearest multiple of 4 */
	LSRS r0, r0, #2         /*	1	2	Divide by 4 and set flags */
	BEQ CyDelayCycles_done  /*	2	2	Skip if 0 */
	NOP                     /*	1	2	Loop alignment padding */
CyDelayCycles_loop:
	SUBS r0, r0, #1         /*	1	2 */
	MOV r0, r0              /*	1	2	Pad loop to power of two cycles */
	BNE CyDelayCycles_loop  /*	2	2 */
CyDelayCycles_done:
	BX lr                   /*	3	2 */
.endfunc


/*******************************************************************************
* Function Name: CyEnterCriticalSection
********************************************************************************
* Summary:
*   Enters a critical section and disables interrupts.
*   Implementation of CyEnterCriticalSection manipulates the IRQ enable bit with 
*   interrupts still enabled. The test and set of the interrupt bits is not atomic; 
*   this is true for both PSoC3 and PSoC5. 
*   Therefore, to avoid corrupting processor state, 
*   it must be the policy that all interrupt routines restore the 
*   interrupt enable bits as they were found on entry.
*
* Parameters:
*   void.
*
* Return:
*   Returns 1 if interrupts were previously enabled or 0 if interrupts were
*     previously disabled.
*
*******************************************************************************/
/* uint8 CyEnterCriticalSection(void) */
.global CyEnterCriticalSection
.func CyEnterCriticalSection, CyEnterCriticalSection
.type CyEnterCriticalSection, %function
.thumb_func
CyEnterCriticalSection:
	MRS r0, PRIMASK         /* Save and return interrupt state */
	CPSID I                 /* Disable interrupts */
	BX lr
.endfunc


/*******************************************************************************
* Function Name: CyExitCriticalSection
********************************************************************************
* Summary:
*   Ends a critical section, re-enabling interrupts if they were enabled before
*   entering the critical section.
*   Implementation of CyEnterCriticalSection manipulates the IRQ enable bit with 
*   interrupts still enabled. The test and set of the interrupt bits is not atomic; 
*   this is true for both PSoC3 and PSoC5. 
*   Therefore, to avoid corrupting processor state, 
*   it must be the policy that all interrupt routines restore the 
*   interrupt enable bits as they were found on entry.
*
*
* Parameters:
*   savedIntrStatus: Nonzero to enable interrupts. Should be the value that was
*     returned from CyEnterCriticalSection.
*
* Return:
*   void.
*
*******************************************************************************/
/* void CyExitCriticalSection(uint8 savedIntrStatus) */
.global CyExitCriticalSection
.func CyExitCriticalSection, CyExitCriticalSection
.type CyExitCriticalSection, %function
.thumb_func
CyExitCriticalSection:
	MSR PRIMASK, r0         /* Restore interrupt state */
	BX lr
.endfunc

.end


/* [] END OF FILE */
