/*******************************************************************************
* File Name: I2C.c
* Version 3.1
*
* Description:
*  This file provides the source code of APIs for the I2C component.
*  Actual protocol and operation code resides in the interrupt service routine 
*  file.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C.h"


/**********************************
*      System variables
**********************************/

uint8 I2C_initVar = 0u;
extern volatile uint8 I2C_state;    /* Current state of I2C state machine */

/* Master variables */
#if (0u != (I2C_MODE & I2C_MODE_MASTER))
   extern volatile uint8 I2C_mstrStatus;          /* Master Status byte */
   extern volatile uint8 I2C_mstrControl;         /* Master Control byte */
   
   /* Transmit buffer variables */
   extern volatile uint8 * I2C_mstrRdBufPtr;      /* Pointer to Master Read buffer */
   extern volatile uint8   I2C_mstrRdBufSize;     /* Master Read buffer size */
   extern volatile uint8   I2C_mstrRdBufIndex;    /* Master Read buffer Index */
    
   /* Receive buffer variables */
   extern volatile uint8 * I2C_mstrWrBufPtr;      /* Pointer to Master Write buffer */
   extern volatile uint8   I2C_mstrWrBufSize;     /* Master Write buffer size */
   extern volatile uint8   I2C_mstrWrBufIndex;    /* Master Write buffer Index */
   
#endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */

/* Slave variables */
#if (0u != (I2C_MODE & I2C_MODE_SLAVE))
   extern volatile uint8 I2C_slStatus;            /* Slave Status  */
   
   #if (I2C_ADDR_DECODE == I2C_SW_DECODE)
      extern volatile uint8 I2C_slAddress;        /* Software address variable */
   #endif   /* End (I2C_ADDR_DECODE == I2C_SW_DECODE) */
   
   /* Transmit buffer variables */
   extern volatile uint8 * I2C_slRdBufPtr;        /* Pointer to Transmit buffer */
   extern volatile uint8   I2C_slRdBufSize;       /* Slave Transmit buffer size */
   extern volatile uint8   I2C_slRdBufIndex;      /* Slave Transmit buffer Index */

   /* Receive buffer variables */
   extern volatile uint8 * I2C_slWrBufPtr;        /* Pointer to Receive buffer */
   extern volatile uint8   I2C_slWrBufSize;       /* Slave Receive buffer size */
   extern volatile uint8   I2C_slWrBufIndex;      /* Slave Receive buffer Index */

#endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */

extern I2C_BACKUP_STRUCT I2C_backup;
    
/*******************************************************************************
* Function Name: I2C_Init
********************************************************************************
*
* Summary:
*  Initializes I2C registers with initial values provided from customizer.
*
* Parameters:  
*  None
*
* Return: 
*  None
*
* Global variables:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void I2C_Init(void) 
{
    #if (I2C_IMPLEMENTATION == I2C_FF)
        /* Set CFG register */
        I2C_CFG_REG = I2C_DEFAULT_CFG;
        
        /* Set XCFG register */
        I2C_XCFG_REG = I2C_DEFAULT_XCFG;
        
        /* Set devide factor */
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
            I2C_CLKDIV_REG = I2C_DEFAULT_DIVIDE_FACTOR;
        #else
            I2C_CLKDIV1_REG = LO8(I2C_DEFAULT_DIVIDE_FACTOR);
            I2C_CLKDIV2_REG = HI8(I2C_DEFAULT_DIVIDE_FACTOR);
        #endif /* End (CY_PSOC3_ES2 || CY_PSOC3_ES2) */
        
    #else
        uint8 enableInterrupts;
        
        /* Set CFG register */
        I2C_CFG_REG = I2C_DEFAULT_CFG;
        
        /* Set interrupt source: enable Byte Complete interrupt */
        I2C_INT_MASK_REG = I2C_BYTE_COMPLETE_IE_MASK;
        
        /* Enable interrupts from block */
        enableInterrupts = CyEnterCriticalSection();
        I2C_INT_ENABLE_REG |= I2C_INT_ENABLE_MASK;
        CyExitCriticalSection(enableInterrupts);
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */
    
    /* Disable Interrupt and set vector and priority */
    CyIntDisable(I2C_ISR_NUMBER);
    CyIntSetVector(I2C_ISR_NUMBER, I2C_ISR);
    CyIntSetPriority(I2C_ISR_NUMBER, I2C_ISR_PRIORITY);
    
    /* Put state machine in idle state */
    I2C_state = I2C_SM_IDLE;
    
    #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
        /* Reset status and buffer index */
        I2C_SlaveClearReadBuf();
        I2C_SlaveClearWriteBuf();
        I2C_SlaveClearReadStatus();
        I2C_SlaveClearWriteStatus();
        
        /* Set default address */
        I2C_SlaveSetAddress(I2C_DEFAULT_ADDR);
    
    #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
    
    #if (0u != (I2C_MODE & I2C_MODE_MASTER))
        /* Reset status and buffer index */
        I2C_MasterClearReadBuf();
        I2C_MasterClearWriteBuf();
        I2C_MasterClearStatus();
        
    #endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */
}


/*******************************************************************************
* Function Name: I2C_Enable
********************************************************************************
*
* Summary:
*  Enables I2C operations.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
*******************************************************************************/
void I2C_Enable(void) 
{
    #if ((I2C_IMPLEMENTATION != I2C_UDB) || \
        (0u != (I2C_MODE & I2C_MODE_SLAVE)))
        uint8 enableInterrupts;
    #endif  /* End ((I2C_IMPLEMENTATION != I2C_UDB) || \
                    (0u != (I2C_MODE & I2C_MODE_SLAVE)) ) */
    
    #if (I2C_IMPLEMENTATION == I2C_FF)
        enableInterrupts = CyEnterCriticalSection();
        /* Enable power to I2C Module */
        I2C_ACT_PWRMGR_REG  |= I2C_ACT_PWR_EN;
        I2C_STBY_PWRMGR_REG |= I2C_STBY_PWR_EN;
        CyExitCriticalSection(enableInterrupts);
        
    #else
        /* Enable the I2C */
        I2C_CFG_REG |= (I2C_ENABLE_MASTER | I2C_ENABLE_SLAVE);
        
        /* Enable bit counter */
        #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
            enableInterrupts = CyEnterCriticalSection();
            /* Enable Counter7 */
            I2C_COUNTER_AUX_CTL_REG |= I2C_COUNTER_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);
        #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
        
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */
}


/*******************************************************************************
* Function Name: I2C_Start
********************************************************************************
*
* Summary:
*  Starts the I2C hardware. Enables Active mode power template bits or clock 
*  gating as appropriate. It is required to be executed before I2C bus operation.
*  The I2C interrupt remains disabled after this function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This component automatically enables it's interrupt.  If I2C is enabled
*  without the interrupt enabled, it could lock up the I2C bus.
*
* Global variables:
*  I2C_initVar - used to check initial configuration, modified 
*  on first function call.
*
* Reentrant:
*  No
*
*******************************************************************************/
void I2C_Start(void) 
{
    /* Initialize I2C registers, reset I2C buffer index and clears status */
    if (0u == I2C_initVar)
    {
        I2C_Init();
        I2C_initVar = 1u;
    }
    
    /* Enable component */
    I2C_Enable();
    
    /* Enable interrupt */
    I2C_EnableInt();
}


/*******************************************************************************
* Function Name: I2C_Stop
********************************************************************************
*
* Summary:
*  Disables I2C hardware and disables I2C interrupt. Disables Active mode power 
*  template bits or clock gating as appropriate.
*
* Parameters:
*  None 
*
* Return:
*  None
*
*******************************************************************************/
void I2C_Stop(void) 
{
    #if ((I2C_IMPLEMENTATION != I2C_UDB) || \
         (0u != (I2C_MODE & I2C_MODE_SLAVE)))
        uint8 enableInterrupts;
    #endif  /* End ( (I2C_IMPLEMENTATION != I2C_FF) || \
                   (0u != (I2C_MODE & I2C_MODE_SLAVE)) ) */
    
    /* Disable Interrupt */
    I2C_DisableInt();
    
    #if (I2C_IMPLEMENTATION == I2C_FF)
        
        #if (CY_PSOC3_ES3)
            /* Store resgisters which are held in reset when Master or Slave disabled */
            #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
                I2C_backup.addr = I2C_ADDR_REG;
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
            
            I2C_backup.clk_div1  = I2C_CLKDIV1_REG;
            I2C_backup.clk_div2  = I2C_CLKDIV2_REG;
            
            /* Reset the state machine of FF block */
            I2C_CFG_REG &= ~(I2C_ENABLE_MASTER | I2C_ENABLE_SLAVE);
            
            #if (I2C_MODE != I2C_MODE_SLAVE)
                CyDelayUs(2);   /* Delay required for Master reset */
            #endif /* End (I2C_MODE != I2C_MODE_SLAVE) */
            
            /* Restore registers */
            I2C_CFG_REG |= (I2C_ENABLE_MASTER | I2C_ENABLE_SLAVE);
            
            /* Restore registers */
            #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
                I2C_ADDR_REG = I2C_backup.addr;
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
            
            I2C_CLKDIV1_REG = I2C_backup.clk_div1;
            I2C_CLKDIV2_REG = I2C_backup.clk_div2;
            
        #endif  /* End (CY_PSOC3_ES3) */
        
        enableInterrupts = CyEnterCriticalSection();
        /* Disable power to I2C block */
        I2C_ACT_PWRMGR_REG  &= ~I2C_ACT_PWR_EN;
        I2C_STBY_PWRMGR_REG &= ~I2C_STBY_PWR_EN;
        CyExitCriticalSection(enableInterrupts);
    
    #else
        /* Clears enable bits in control register */
        I2C_CFG_REG &= ~(I2C_ENABLE_MASTER | I2C_ENABLE_SLAVE);
        
        /* Disable bit counter */
        #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
            enableInterrupts = CyEnterCriticalSection();
            /* Disable Counter7 */
            I2C_COUNTER_AUX_CTL_REG &= ~I2C_COUNTER_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);
        #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
        
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */
    
    /* Clear the interrupt history */
    CyIntClearPending(I2C_ISR_NUMBER);
    
    /* Put state machine in IDLE state */
    I2C_state = I2C_SM_IDLE;
    
    /* Statuses and buffers are not cleared for Slave and Master */
}


/*******************************************************************************
* Function Name: I2C_EnableInt
********************************************************************************
*
* Summary:
*  This function is implemented as macro in I2C.h file.
*  Enables I2C interrupt. Interrupts are required for most operations.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/


/*******************************************************************************
* Function Name: I2C_DisableInt
********************************************************************************
*
* Summary:
*  This function is implemented as macro in I2C.h file.
*  Disables I2C interrupts. Normally this function is not required since the 
*  Stop function disables the interrupt. If the I2C interrupt is disabled while 
*  the I2C master is still running, it may cause the I2C bus to lock up.
*
* Parameters:
*  None
*
* Return: 
*  None
*
* Side Effects:
*  If the I2C interrupt is disabled and the master is addressing the current 
*  slave, the bus will be locked until the interrupt is re-enabled.
*
*******************************************************************************/


#if (0u != (I2C_MODE & I2C_MODE_MASTER))
    /*******************************************************************************
    * Function Name: I2C_MasterStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the master's communication status.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Current status of I2C master.
    *
    * Global variables:
    *  I2C_mstrStatus - used to store current status of I2C Master.
    *
    *******************************************************************************/
    uint8 I2C_MasterStatus(void) 
    {
        uint8 status;
        
        status = I2C_mstrStatus;
        
        /* When in Master state only transaction is in progress */
        if (0u != (I2C_state & I2C_SM_MASTER))
        {
            /* Add transaction in progress activity to master status */
            status |= I2C_MSTAT_XFER_INP;
        }
        else
        {
            /* Current master status is valid */
        }
        
        return (status);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterClearStatus
    ********************************************************************************
    *
    * Summary:
    *  Clears all status flags and returns the master status.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Current status of I2C master.
    *
    * Global variables:
    *  I2C_mstrStatus - used to store current status of I2C Master.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterClearStatus(void) 
    {
        /* Current master status */
        uint8 status;
        
        /* Read and clear master status */
        status = I2C_mstrStatus;
        I2C_mstrStatus = I2C_MSTAT_CLEAR; 
        
        return (status);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterWriteBuf
    ********************************************************************************
    *
    * Summary:
    *  Automatically writes an entire buffer of data to a slave device. Once the 
    *  data transfer is initiated by this function, further data transfer is handled
    *  by the included ISR in byte by byte mode.
    *
    * Parameters:
    *  slaveAddr: 7-bit slave address.
    *  xferData:  Pointer to buffer of data to be sent.
    *  cnt:       Size of buffer to send.
    *  mode:      Transfer mode defines: start or restart condition generation at 
    *             begin of the transfer and complete the transfer or halt before 
    *             generating a stop.
    *
    * Return:
    *  Status error - zero means no errors.
    *
    * Side Effects:
    *  The included ISR will start transfer after start or restart condition will 
    *  be generated.
    *
    * Global variables:
    *  I2C_mstrStatus  - used to store current status of I2C Master.
    *  I2C_state       - used to store current state of software FSM.
    *  I2C_mstrControl - used to control master end of transaction with
    *  or without the Stop generation.
    *  I2C_mstrWrBufPtr - used to store pointer to master write buffer.
    *  I2C_mstrWrBufIndex - used to current index within master write 
    *  buffer.
    *  I2C_mstrWrBufSize - used to store master write buffer size.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterWriteBuf(uint8 slaveAddress, uint8 * xferData, uint8 cnt, uint8 mode)
           
    {
        uint8 errStatus = I2C_MSTR_NOT_READY;
        
        /* Check for proper buffer */
        if (NULL != xferData)
        {
            /* Check if I2C in proper state to generate Start/ReStart condition */
            if ((I2C_state == I2C_SM_IDLE) || 
               (I2C_state == I2C_SM_MSTR_HALT))
            {
                /* If IDLE, check if bus is free */
                if (I2C_state == I2C_SM_IDLE)
                {
                    /* If Bus is free proceed, no exist */
                    if (I2C_CHECK_BUS_FREE(I2C_MCSR_REG))
                    {
                        errStatus = I2C_MSTR_NO_ERROR;
                    }
                    else
                    {
                        errStatus = I2C_MSTR_BUS_BUSY;
                    }
                }
                else   /* Bus halted waiting for restart */
                {
                    CyIntClearPending(I2C_ISR_NUMBER);
                    I2C_mstrStatus &= ~I2C_MSTAT_XFER_HALT;
                    errStatus = I2C_MSTR_NO_ERROR;
                }
                
                /* If no errors, generate start */
                if (errStatus == I2C_MSTR_NO_ERROR)
                {
                    /* Determine whether or not to generate a Stop condition at the end of write */
                    if (0u != (mode & I2C_MODE_NO_STOP))
                    {
                        I2C_mstrControl |= I2C_MSTR_NO_STOP;  /* Without a Stop */
                    }
                    else
                    {
                        I2C_mstrControl &= ~I2C_MSTR_NO_STOP; /* Generate a Stop */
                    }
                    
                    I2C_state = I2C_SM_MSTR_WR_ADDR;  /* Start from address write state */
                    slaveAddress <<= I2C_SLAVE_ADDR_SHIFT;         /* Set Address */
                    I2C_DATA_REG = slaveAddress;                   /* Write address to data reg */
                    
                    I2C_mstrWrBufIndex = 0u;       /* Start buffer at zero */
                    I2C_mstrWrBufSize  = cnt;      /* Set buffer size */
                    I2C_mstrWrBufPtr   = (volatile uint8 *) xferData; /* Set buffer pointer */
                    
                    /* Generate a Start or ReStart depending on flag passed */
                    if (0u != (mode & I2C_MODE_REPEAT_START))
                    {
                        I2C_GENERATE_RESTART;  /* Generate a ReStart */
                    }
                    else
                    {
                        I2C_GENERATE_START;    /* Generate a Start */
                    }
                    
                    /* Enable interrupts to process transfer */
                    I2C_EnableInt();
                    
                    /* Clear write complete flag */
                    I2C_mstrStatus &= ~I2C_MSTAT_WR_CMPLT;
                }
            }
        }
        
        return (errStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterReadBuf
    ********************************************************************************
    *
    * Summary:
    *  Automatically writes an entire buffer of data to a slave device. Once the 
    *  data transfer is initiated by this function, further data transfer is handled
    *  by the included ISR in byte by byte mode.  
    *
    * Parameters:
    *  slaveAddr: 7-bit slave address.
    *  xferData:  Pointer to buffer where to put data from slave.
    *  cnt:       Size of buffer to read.
    *  mode:      Transfer mode defines: start or restart condition generation at 
    *             begin of the transfer and complete the transfer or halt before 
    *             generating a stop.
    *
    * Return:
    *  Status error - zero means no errors.
    *
    * Side Effects:
    *  The included ISR will start transfer after start or restart condition will 
    *  be generated.
    *
    * Global variables:
    *  I2C_mstrStatus  - used to store current status of I2C Master.
    *  I2C_state       - used to store current state of software FSM.
    *  I2C_mstrControl - used to control master end of transaction with
    *  or without the Stop generation.
    *  I2C_mstrRdBufPtr - used to store pointer to master write buffer.
    *  I2C_mstrRdBufIndex - used to current index within master write 
    *  buffer.
    *  I2C_mstrRdBufSize - used to store master write buffer size.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterReadBuf(uint8 slaveAddress, uint8 * xferData, uint8 cnt, uint8 mode)
          
    {
        uint8 errStatus = I2C_MSTR_NOT_READY;
        
        /* Check for proper buffer */
        if (NULL != xferData)
        {
            /* Check if I2C in proper state to generate Start/ReStart condition */
            if ((I2C_state == I2C_SM_IDLE) ||
               (I2C_state == I2C_SM_MSTR_HALT))
            {
                /* If IDLE, check if bus is free */
                if (I2C_state == I2C_SM_IDLE)
                {
                    /* If Bus is free proceed, no exist */
                    if (I2C_CHECK_BUS_FREE(I2C_MCSR_REG))
                    {
                        errStatus = I2C_MSTR_NO_ERROR;
                    }
                    else
                    {
                        errStatus = I2C_MSTR_BUS_BUSY;
                    }
                }
                else   /* Bus halted waiting for restart */
                {
                    CyIntClearPending(I2C_ISR_NUMBER);
                    I2C_mstrStatus &= ~I2C_MSTAT_XFER_HALT;
                    errStatus = I2C_MSTR_NO_ERROR;
                }
                
                /* If no error, generate Start/ReStart condition */
                if (errStatus == I2C_MSTR_NO_ERROR)
                {
                    /* Determine whether or not to generate a Stop condition at the end of read */
                    if (0u != (mode & I2C_MODE_NO_STOP))
                    {
                        I2C_mstrControl |= I2C_MSTR_NO_STOP;   /* Without Stop */
                    }
                    else
                    {
                        I2C_mstrControl &= ~I2C_MSTR_NO_STOP; /* Generate a Stop */
                    }
                    
                    I2C_state = I2C_SM_MSTR_RD_ADDR;  /* Start from address read state */ 
                    slaveAddress <<= I2C_SLAVE_ADDR_SHIFT;         /* Set Address */
                    slaveAddress |= I2C_READ_FLAG;                 /* Set the Read flag */
                    I2C_DATA_REG = slaveAddress;                   /* Write address to data reg */
                    
                    I2C_mstrRdBufIndex  = 0u;      /* Start buffer at zero */
                    I2C_mstrRdBufSize   = cnt;     /* Set buffer size */
                    I2C_mstrRdBufPtr    = (volatile uint8 *) xferData; /* Set buffer pointer */
                    
                    /* Generate a Start or ReStart depending on flag passed */
                    if (0u != (mode & I2C_MODE_REPEAT_START))
                    {
                        I2C_GENERATE_RESTART;  /* Generate a ReStart */
                    }
                    else
                    {
                        I2C_GENERATE_START;    /* Generate a Start */
                    }
                    
                    /* Enable interrupts to process transfer */
                    I2C_EnableInt();
                    
                    /* Clear read complete flag */
                    I2C_mstrStatus &= ~I2C_MSTAT_RD_CMPLT;
                }
            }
        }
        
        return (errStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterSendStart
    ********************************************************************************
    *
    * Summary:
    *  Generates Start condition and sends slave address with read/write bit.
    *
    * Parameters:  
    *  slaveAddress:  7-bit slave address.
    *  R_nW:          Zero, send write command, non-zero send read command.
    *
    * Return: 
    *  Status error - zero means no errors.
    *
    * Side Effects:
    *  This function is entered without a 'byte complete' bit set in the I2C_CSR 
    *  register. It does not exit until it will be set.
    *
    * Global variables:
    *  I2C_state - used to store current state of software FSM.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterSendStart(uint8 slaveAddress, uint8 R_nW) 
          
    {
        uint8 errStatus = I2C_MSTR_NOT_READY;
        
        /* If IDLE, check if bus is free */
        if (I2C_state == I2C_SM_IDLE)
        {
            /* If bus is free, generate Start condition */
            if (I2C_CHECK_BUS_FREE(I2C_MCSR_REG))
            {
                /* Disable ISR for Manual functions */
                I2C_DisableInt();
                
                slaveAddress <<= I2C_SLAVE_ADDR_SHIFT; /* Set Address */
                if (0u != R_nW)                                      /* Set the Read/Write flag */
                {
                    slaveAddress |= I2C_READ_FLAG;
                    I2C_state = I2C_SM_MSTR_RD_ADDR;
                }
                else
                {
                    I2C_state = I2C_SM_MSTR_WR_ADDR;
                }
                I2C_DATA_REG = slaveAddress;   /* Write address to data reg */
                
                /* Generates a START */
                I2C_GENERATE_START;
                
                /* Wait for the address to be transfered */
                while (I2C_WAIT_BYTE_COMPLETE(I2C_CSR_REG));
                
                #if (I2C_MODE == I2C_MODE_MULTI_MASTER_SLAVE)
                    if (I2C_CHECK_START_GEN(I2C_MCSR_REG))
                    {
                        /* Clear Start Gen bit */
                        I2C_CLEAR_START_GEN;
                        
                        /* Arbitration has been lost, reset state machine to IDLE */
                        I2C_state = I2C_SM_IDLE;
                        errStatus = I2C_MSTR_ERR_ABORT_START_GEN;
                    }
                    else
                #endif  /* End (I2C_MODE == I2C_MULTI_MASTER_ENABLE) */
                    
                #if (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE))
                    /* Check for loss of arbitration */
                    if (I2C_CHECK_LOST_ARB(I2C_CSR_REG))
                    {
                        /* Arbitration has been lost, reset state machine to IDLE */
                        I2C_state = I2C_SM_IDLE;
                        errStatus = I2C_MSTR_ERR_ARB_LOST; /* Master lost arbitrage */
                    }
                    else
                #endif  /* (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE)) */
                    
                    if (I2C_CHECK_ADDR_NAK(I2C_CSR_REG))
                    {
                        /* Address has been NACKed, reset state machine to IDLE */
                        I2C_state = I2C_SM_IDLE;
                        errStatus = I2C_MSTR_ERR_LB_NAK;    /* No device ACKed the Master */
                    }
                    else
                    {
                        errStatus = I2C_MSTR_NO_ERROR;     /* Send Start witout errors */
                    }
            }
            else
            {
                errStatus = I2C_MSTR_BUS_BUSY;     /* Bus is busy */
            }
        }
        
        return (errStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterSendRestart
    ********************************************************************************
    *
    * Summary:
    *  Generates ReStart condition and sends slave address with read/write bit.
    *
    * Parameters:  
    *  slaveAddress:  7-bit slave address.
    *  R_nW:          Zero, send write command, non-zero send read command.
    *
    * Return: 
    *  Status error - zero means no errors.
    *
    * Side Effects:
    *  This function is entered without a 'byte complete' bit set in the I2C_CSR 
    *  register. It does not exit until it will be set.
    *
    * Global variables:
    *  I2C_state - used to store current state of software FSM.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterSendRestart(uint8 slaveAddress, uint8 R_nW) 
          
    {
        uint8 errStatus = I2C_MSTR_NOT_READY;
        
        /* Check if START condition was generated */
        if (I2C_CHECK_MASTER_MODE(I2C_MCSR_REG))
        {
            slaveAddress <<= I2C_SLAVE_ADDR_SHIFT; /* Set Address */
            if (0u != R_nW)                                      /* Set the Read/Write flag */
            {
                slaveAddress |= I2C_READ_FLAG;
                I2C_state = I2C_SM_MSTR_RD_ADDR;
            }
            else
            {
                I2C_state = I2C_SM_MSTR_WR_ADDR;
            }
            I2C_DATA_REG = slaveAddress;    /* Write address to data reg */
            
            /* Generates RESTART */
            I2C_GENERATE_RESTART;
            #if (I2C_IMPLEMENTATION == I2C_UDB)
                while (I2C_CHECK_BYTE_COMPLETE(I2C_CSR_REG));
            #endif /* End (I2C_IMPLEMENTATION == I2C_UDB) */
            
            /* Wait for the address to be transfered */
            while (I2C_WAIT_BYTE_COMPLETE(I2C_CSR_REG));
            
            #if (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE))
                /* Check for loss of arbitration */
                if (I2C_CHECK_LOST_ARB(I2C_CSR_REG))
                {
                    /* Arbitration has been lost, reset state machine to IDLE */
                    I2C_state = I2C_SM_IDLE;
                    errStatus = I2C_MSTR_ERR_ARB_LOST; /* Master lost arbitrage */
                }
                else
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE)) */    
            
                /* Check ACK address if Master mode */
                if (I2C_CHECK_ADDR_NAK(I2C_CSR_REG))
                {
                    /* Address has been NACKed, reset state machine to IDLE */
                    I2C_state = I2C_SM_IDLE;
                    errStatus = I2C_MSTR_ERR_LB_NAK;    /* No device ACKed the Master */
                }
                else
                {
                    errStatus = I2C_MSTR_NO_ERROR;     /* Send START witout errors */
                }
        }
        
        return (errStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterSendStop
    ********************************************************************************
    *
    * Summary:
    *  Generates I2C Stop condition on bus. Function do nothing if Start or Restart 
    *  condition was failed before call this function.
    *
    * Parameters:  
    *  None
    *
    * Return: 
    *  Status error - zero means no errors.
    *
    * Side Effects:
    *  The Stop generation is required to complete transaction.
    *  This function does not wait while Stop condition will be generated.
    *
    * Global variables:
    *  I2C_state - used to store current state of software FSM.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterSendStop(void) 
    {
        uint8 errStatus = I2C_MSTR_NOT_READY;
        
        /* Check if START condition was generated */
        if (I2C_CHECK_MASTER_MODE(I2C_MCSR_REG))
        {
            I2C_GENERATE_STOP;                     /* Generate STOP */
            I2C_state = I2C_SM_IDLE; /* Reset state to IDLE */
            errStatus = I2C_MSTR_NO_ERROR;         /* Start send STOP witout errors */
                
            /* Wait for STOP generation or BYTE COMPLETE (lost arbitrage) */
            #if (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE))
                
                #if (I2C_IMPLEMENTATION == I2C_UDB)
                    while (I2C_CHECK_BYTE_COMPLETE(I2C_CSR_REG));
                #endif /* End (I2C_IMPLEMENTATION == I2C_UDB) */
                
                while (0u == (I2C_CSR_REG & (I2C_CSR_BYTE_COMPLETE |
                                                         I2C_CSR_STOP_STATUS)));
                
                /* Check LOST ARBITRAGE */
                if (I2C_CHECK_LOST_ARB(I2C_CSR_REG))
                {
                    errStatus = I2C_MSTR_ERR_ARB_LOST; /* NACK was generated instead Stop */
                }
                /* STOP condition generated */
                else
                {
                    errStatus = I2C_MSTR_NO_ERROR;     /* Stop was generated */
                }
            #else
                /* Wait till Stop will be generated */
                while (0u == (I2C_CSR_REG & I2C_CSR_STOP_STATUS));
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE)) */
        }
        
        return (errStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterWriteByte
    ********************************************************************************
    *
    * Summary:
    *  Sends one byte to a slave. A valid Start or ReStart condition must be 
    *  generated before this call this function. Function do nothing if Start or 
    *  Restart condition was failed before call this function.
    *
    * Parameters:
    *  data:  The data byte to send to the slave.
    *
    * Return:
    *  Status error - zero means no errors.
    *
    * Side Effects:
    *  This function is entered without a 'byte complete' bit set in the I2C_CSR 
    *  register. It does not exit until it will be set.
    *
    * Global variables:
    *  I2C_state - used to store current state of software FSM.
    *
    *******************************************************************************/
    uint8 I2C_MasterWriteByte(uint8 theByte) 
    {
        uint8 errStatus = I2C_MSTR_NOT_READY;
        
        /* Check if START condition was generated */
        if (I2C_CHECK_MASTER_MODE(I2C_MCSR_REG))
        {
            I2C_DATA_REG = theByte;                        /* Write DATA register */
            I2C_TRANSMIT_DATA;                             /* Set transmit mode */
            I2C_state = I2C_SM_MSTR_WR_DATA;  /* Set state WR_DATA */
            #if (I2C_IMPLEMENTATION == I2C_UDB)
                while (I2C_CHECK_BYTE_COMPLETE(I2C_CSR_REG));
            #endif
            
            /* Make sure the last byte has been transfered first */
            while (I2C_WAIT_BYTE_COMPLETE(I2C_CSR_REG));
            
            #if (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE))
                /* Check for LOST ARBITRATION */
                if (I2C_CHECK_LOST_ARB(I2C_CSR_REG))
                {
                    /* Arbitration has been lost, reset state machine to IDLE */
                    I2C_state = I2C_SM_IDLE;          /* Reset state to IDLE */
                    errStatus = I2C_MSTR_ERR_ARB_LOST;             /* The Master LOST ARBITRAGE */
                }
                /* Check LRB bit */
                else 
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE)) */
            
                if (I2C_CHECK_DATA_ACK(I2C_CSR_REG))
                {
                    I2C_state = I2C_SM_MSTR_HALT;     /* Set state to HALT */
                    errStatus = I2C_MSTR_NO_ERROR;                 /* The LRB was ACKed */
                }
                else
                {
                    I2C_state = I2C_SM_MSTR_HALT;     /* Set state to HALT */
                    errStatus = I2C_MSTR_ERR_LB_NAK;               /* The LRB was NACKed */
                }
        }
        
        return (errStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterReadByte
    ********************************************************************************
    *
    * Summary:
    *  Reads one byte from a slave and ACK or NACK the transfer. A valid Start or 
    *  ReStart condition must be generated before this call this function. Function
    *  do nothing if Start or Restart condition was failed before call this 
    *  function.
    *
    * Parameters:
    *  acknNack:  Zero, response with NACK, if non-zero response with ACK.
    *
    * Return:
    *  Byte read from slave.
    *
    * Side Effects:
    *  This function is entered without a 'byte complete' bit set in the I2C_CSR 
    *  register. It does not exit until it will be set.
    *
    * Global variables:
    *  I2C_state - used to store current state of software FSM.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_MasterReadByte(uint8 acknNak) 
    {
        uint8 theByte = 0u;
        
        /* Check if START condition was generated */
        if (I2C_CHECK_MASTER_MODE(I2C_MCSR_REG))
        {
            /* When address phase need release the bus and receive the byte, then decide ACK or NACK */
            if (I2C_state == I2C_SM_MSTR_RD_ADDR)
            {
                I2C_READY_TO_READ;
                I2C_state = I2C_SM_MSTR_RD_DATA;
                #if (I2C_IMPLEMENTATION == I2C_UDB)
                    while (I2C_CHECK_BYTE_COMPLETE(I2C_CSR_REG));
                #endif /* End (I2C_IMPLEMENTATION == I2C_UDB) */
            }
            
            while (I2C_WAIT_BYTE_COMPLETE(I2C_CSR_REG));
            
            theByte = I2C_DATA_REG;
            
            /* Now if the ACK flag was set, ACK the data which will release the bus and start the next byte in
               otherwise do NOTHING to the CSR reg.
               This will allow the calling routine to generate a repeat start or a stop depending on it's preference. */
            if (acknNak != 0u)   /* Do ACK */
            {
                I2C_ACK_AND_RECEIVE;
                #if (I2C_IMPLEMENTATION == I2C_UDB)
                    while (I2C_CHECK_BYTE_COMPLETE(I2C_CSR_REG));
                #endif /* End (I2C_IMPLEMENTATION == I2C_UDB) */
            }
            else                /* Do NACK */
            {
                /* Do nothing to be able work with ReStart */
                I2C_state = I2C_SM_MSTR_HALT;
            }
        }
        
        return (theByte);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterGetReadBufSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the amount of bytes that has been transferred with an 
    *  I2C_MasterReadBuf command.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Byte count of transfer. If the transfer is not yet complete, it will return 
    *  the byte count transferred so far.
    *
    * Global variables:
    *  I2C_mstrRdBufIndex - used to current index within master read 
    *  buffer.
    *
    *******************************************************************************/
    uint16 I2C_MasterGetReadBufSize(void) 
    {
        return (I2C_mstrRdBufIndex);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterGetWriteBufSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the amount of bytes that has been transferred with an 
    *  I2C_MasterWriteBuf command.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Byte count of transfer. If the transfer is not yet complete, it will return 
    *  the byte count transferred so far.
    *
    * Global variables:
    *  I2C_mstrWrBufIndex - used to current index within master write 
    *  buffer.
    *
    *******************************************************************************/
    uint16 I2C_MasterGetWriteBufSize(void) 
    {
        return (I2C_mstrWrBufIndex);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterClearReadBuf
    ********************************************************************************
    *
    * Summary:
    *  Resets the read buffer pointer back to the first byte in the buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_mstrRdBufIndex - used to current index within master read 
    *   buffer.
    *  I2C_mstrStatus - used to store current status of I2C Master.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_MasterClearReadBuf(void) 
    {
        I2C_mstrRdBufIndex = 0u;
        I2C_mstrStatus &= ~I2C_MSTAT_RD_CMPLT;
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_MasterClearWriteBuf
    ********************************************************************************
    *
    * Summary:
    *  Resets the write buffer pointer back to the first byte in the buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_mstrRdBufIndex - used to current index within master read 
    *   buffer.
    *  I2C_mstrStatus - used to store current status of I2C Master.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_MasterClearWriteBuf(void) 
    {
        I2C_mstrWrBufIndex = 0u;
        I2C_mstrStatus &= ~I2C_MSTAT_WR_CMPLT;
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_Workaround
    ********************************************************************************
    *
    * Summary:
    *  Do nothing. This fake fuction use as workaround for CDT 78083.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_Workaround(void) 
    {
    
    }
    
#endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */


#if (0u != (I2C_MODE & I2C_MODE_SLAVE))

    /*******************************************************************************
    * Function Name: I2C_SlaveStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns I2C slave's communication status.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Current status of I2C slave.
    *
    * Global variables:
    *  I2C_slStatus  - used to store current status of I2C slave.
    *
    *******************************************************************************/
    uint8 I2C_SlaveStatus(void) 
    {
        return (I2C_slStatus);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveClearReadStatus
    ********************************************************************************
    *
    * Summary:
    *  Clears the read status flags and returns they values. No other status flags 
    *  are affected.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Current read status of I2C slave.
    *
    * Global variables:
    *  I2C_slStatus  - used to store current status of I2C slave.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_SlaveClearReadStatus(void) 
    {
        uint8 status;
        
        /* Mask of transfer complete flag and Error status */
        status = I2C_slStatus & I2C_SSTAT_RD_MASK;
        I2C_slStatus &= ~I2C_SSTAT_RD_CLEAR;
        
        return (status);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveClearWriteStatus
    ********************************************************************************
    *
    * Summary:
    *  Clears the write status flags and returns they values. No other status flags
    *  are affected.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Current write status of I2C slave.
    *
    * Global variables:
    *  I2C_slStatus  - used to store current status of I2C slave.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    uint8 I2C_SlaveClearWriteStatus(void) 
    {
        uint8 status;
        
        /* Mask of transfer complete flag and Error status */
        status = I2C_slStatus & I2C_SSTAT_WR_MASK;
        I2C_slStatus &= ~I2C_SSTAT_WR_CLEAR;
        
        return (status);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveSetAddress
    ********************************************************************************
    *
    * Summary:
    *  Sets the I2C slave address.
    *
    * Parameters:
    *  address: I2C slave address for the primary device. This value may be any 
    *  address between 0 and 127.
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_Address  - used to store I2C slave address for the primary 
    *  device when software address detect feature is used.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_SlaveSetAddress(uint8 address) 
    {
        #if (I2C_ADDR_DECODE == I2C_HDWR_DECODE)
            I2C_ADDR_REG = address & I2C_SLAVE_ADDR_MASK; /* Set I2C Address register */
        #else
            I2C_slAddress = address & I2C_SLAVE_ADDR_MASK;  /* Set Address variable */
        #endif  /* End (I2C_ADDR_DECODE == I2C_HDWR_DECODE) */ 
    }

    
    /*******************************************************************************
    * Function Name: I2C_SlaveInitReadBuf
    ********************************************************************************
    *
    * Summary:
    *  Sets the buffer pointer and size of the read buffer. This function also 
    *  resets the transfer count returned with the I2C_SlaveGetReadBufSize function.
    *
    * Parameters:
    *  readBuf:  Pointer to the data buffer to be read by the master.
    *  bufSize:  Size of the read buffer exposed to the I2C master.
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_slRdBufPtr   - used to store pointer to slave read buffer.
    *  I2C_slRdBufSize  - used to store salve read buffer size.
    *  I2C_slRdBufIndex - used to store current index within slave
    *  read buffer.
    *
    * Side Effects:
    *  If this function is called during a bus transaction, data from the previous 
    *  buffer location and the beginning of current buffer may be transmitted.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_SlaveInitReadBuf(uint8 * readBuf, uint8 bufSize) 
         
    {
        /* Check for proper buffer */
        if (NULL != readBuf)
        {
            I2C_slRdBufPtr   = (volatile uint8 *) readBuf;    /* Set buffer pointer */
            I2C_slRdBufSize  = bufSize;    /* Set buffer size */
            I2C_slRdBufIndex = 0u;         /* Clears buffer index */
        }
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveInitWriteBuf
    ********************************************************************************
    *
    * Summary:
    *  Sets the buffer pointer and size of the read buffer. This function also 
    *  resets the transfer count returned with the I2C_SlaveGetReadBufSize function.
    *
    * Parameters:
    *  writeBuf:  Pointer to the data buffer to be read by the master.
    *  bufSize:  Size of the buffer exposed to the I2C master.
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_slWrBufPtr   - used to store pointer to slave write buffer.
    *  I2C_slWrBufSize  - used to store salve write buffer size.
    *  I2C_slWrBufIndex - used to store current index within slave
    *  write buffer.
    *
    * Side Effects:
    *  If this function is called during a bus transaction, data from the previous 
    *  buffer location and the beginning of current buffer may be transmitted.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_SlaveInitWriteBuf(uint8 * writeBuf, uint8 bufSize) 
         
    {
        /* Check for proper buffer */
        if (NULL != writeBuf)
        {
            I2C_slWrBufPtr   = (volatile uint8 *) writeBuf;  /* Set buffer pointer */
            I2C_slWrBufSize  = bufSize;   /* Set buffer size */
            I2C_slWrBufIndex = 0u;        /* Clears buffer index */
        }
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveGetReadBufSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes read by the I2C master since an 
    *  I2C_SlaveInitReadBuf or I2C_SlaveClearReadBuf function was executed. 
    *  The maximum return value will be the size of the read buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Bytes read by master.
    *
    * Global variables:
    *  I2C_slRdBufIndex - used to store current index within slave
    *  read buffer.
    *
    *******************************************************************************/
    uint8 I2C_SlaveGetReadBufSize(void) 
    {
        return (I2C_slRdBufIndex);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveGetWriteBufSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes written by the I2C master since an 
    *  I2C_SlaveInitWriteBuf or I2C_SlaveClearWriteBuf function was executed.
    *  The maximum return value will be the size of the write buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  Bytes written by master.
    *
    * Global variables:
    *  I2C_slWrBufIndex - used to store current index within slave
    *  write buffer.
    *
    *******************************************************************************/
    uint8 I2C_SlaveGetWriteBufSize(void) 
    {
        return (I2C_slWrBufIndex);
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveClearReadBuf
    ********************************************************************************
    *
    * Summary:
    *  Resets the read pointer to the first byte in the read buffer. The next byte 
    *  read by the master will be the first byte in the read buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_slRdBufIndex - used to store current index within slave
    *  read buffer.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_SlaveClearReadBuf(void)  
    {
        I2C_slRdBufIndex = 0u;
    }
    
    
    /*******************************************************************************
    * Function Name: I2C_SlaveClearRxBuf
    ********************************************************************************
    *
    * Summary:
    *  Resets the write pointer to the first byte in the write buffer. The next byte
    *  written by the master will be the first byte in the write buffer.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    * Global variables:
    *  I2C_slWrBufIndex - used to store current index within slave
    *  write buffer.
    *
    * Reentrant:
    *  No
    *
    *******************************************************************************/
    void I2C_SlaveClearWriteBuf(void)  
    {
        I2C_slWrBufIndex = 0u;
    }

#endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */


/* [] END OF FILE */
