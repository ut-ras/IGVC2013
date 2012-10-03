/*******************************************************************************
* File Name: I2C_INT.c
* Version 3.1
*
* Description:
*  This file provides the source code of Interrupt Service Routine (ISR)
*  for I2C component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C.h"

/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START I2C_ISR_intc` */

/* `#END` */

/**********************************
*      System variables
**********************************/

volatile uint8 I2C_state = I2C_SM_IDLE; /* Current state of I2C state machine */

/* Master variables */
#if (0u != (I2C_MODE & I2C_MODE_MASTER))
    volatile uint8 I2C_mstrStatus;         /* Master Status byte */
    volatile uint8 I2C_mstrControl;        /* Master Control byte */
    
    /* Transmit buffer variables */
    volatile uint8 * I2C_mstrRdBufPtr;     /* Pointer to Master Tx/Rx buffer */
    volatile uint8   I2C_mstrRdBufSize;    /* Master buffer size */
    volatile uint8   I2C_mstrRdBufIndex;   /* Master buffer Index */
    
    /* Receive buffer variables */
    volatile uint8 * I2C_mstrWrBufPtr;     /* Pointer to Master Tx/Rx buffer */
    volatile uint8   I2C_mstrWrBufSize;    /* Master buffer size  */
    volatile uint8   I2C_mstrWrBufIndex;   /* Master buffer Index */

#endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */

/* Slave variables */
#if (0u != (I2C_MODE & I2C_MODE_SLAVE))
    volatile uint8 I2C_slStatus;             /* Slave Status byte */

    #if (I2C_ADDR_DECODE == I2C_SW_DECODE)
        volatile uint8 I2C_slAddress;        /* Software address variable */
    #endif  /* End (I2C_ADDR_DECODE == I2C_SW_DECODE) */
    
    /* Transmit buffer variables */
    volatile uint8 * I2C_slRdBufPtr;     /* Pointer to Transmit buffer */
    volatile uint8   I2C_slRdBufSize;    /* Slave Transmit buffer size */
    volatile uint8   I2C_slRdBufIndex;   /* Slave Transmit buffer Index */
    
    /* Receive buffer variables */
    volatile uint8 * I2C_slWrBufPtr;     /* Pointer to Receive buffer */
    volatile uint8   I2C_slWrBufSize;    /* Slave Receive buffer size */
    volatile uint8   I2C_slWrBufIndex;   /* Slave Receive buffer Index */

#endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */


/*******************************************************************************
* Function Name: I2C_ISR
********************************************************************************
*
* Summary:
*  Handle Interrupt Service Routine.  
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Reentrant:
*  No
*
*******************************************************************************/
CY_ISR(I2C_ISR)
{
    #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
       static uint8  tmp8;    /* Making these static so not wasting time allocating */
    #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
    
    static uint8  tmpCsr;  /* on the stack each time and no one else can see them */
    
    /* Entry from interrupt */
    /* In hardware address compare mode, we can assume we only get interrupted when */
    /* a valid address is recognized. In software address compare mode, we have to  */
    /* check every address after a start condition.                                 */
    
    tmpCsr = I2C_CSR_REG;          /* Make temp copy so that we can check */
                                                /* for stop condition after we are done */
    
    /* Check if Start Condition was generated */
    #if (I2C_MODE == I2C_MODE_MULTI_MASTER_SLAVE)
        if (I2C_CHECK_START_GEN(I2C_MCSR_REG))
        {
            /* Clear Start Gen bit */
            I2C_CLEAR_START_GEN;
            
            /* Check State for READ one: SM_MSTR_RD_ADDR or SM_MSTR_RD_DATA */
            if (0u != (I2C_state & I2C_SM_MSTR_RD))
            {
                /* Set READ complete, but was aborted */
                I2C_mstrStatus |= (I2C_MSTAT_RD_CMPLT | I2C_MSTAT_ERR_XFER);
            }
            else /* All other: should be only write states */
            {
                /* Set WRITE complete, but was aborted */
                I2C_mstrStatus |= (I2C_MSTAT_WR_CMPLT | I2C_MSTAT_ERR_XFER);
            }
            
            /* Reset State Machine to IDLE to enable the Slave */
            I2C_state = I2C_SM_IDLE;
        }
    #endif  /* End (I2C_MODE == I2C_MULTI_MASTER_ENABLE) */
    
    #if (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE))
        /* Check for lost of arbitration  */
        if (I2C_CHECK_LOST_ARB(tmpCsr))
        {
            /* MultiMaster-Slave */
            #if (I2C_MODE == I2C_MODE_MULTI_MASTER_SLAVE)
                /* Check on which state of transaction lost ARBITRAGE:
                  Address    - if Address and pass control to Slave
                  No Address - release the bus */
                if (0u == (tmpCsr & I2C_CSR_ADDRESS))
                {
            #endif  /* End (I2C_MODE == I2C_MULTI_MASTER_ENABLE) */
            
                    /* Lost ARBITRAGE:
                       Data - reset state machine to IDLE, Disable Slave enable event */
                    
                    /* Disable interrupt on STOP in case that it was READ */
                    I2C_DISABLE_INT_ON_STOP;
                    
                    /* Clear CSR to release the bus, if MultiMaster */
                    I2C_READY_TO_READ;
            
            #if (I2C_MODE == I2C_MODE_MULTI_MASTER_SLAVE)
                    /* Clean up the Slave enable events: Byte Complete and Stop */
                    tmpCsr &= ~ (I2C_CSR_BYTE_COMPLETE | I2C_CSR_STOP_STATUS);
                }
            #endif  /* End (I2C_MODE == I2C_MULTI_MASTER_ENABLE) */
            
            /* Check State for READ one: SM_MSTR_RD_ADDR or SM_MSTR_RD_DATA */
            if (0u != (I2C_state & I2C_SM_MSTR_RD))
            {
                /* Set READ complete */
                I2C_mstrStatus |= I2C_MSTAT_RD_CMPLT;
            }
            else /* All other: should be only write states */
            {
                /* Set WRITE complete */
                I2C_mstrStatus |= I2C_MSTAT_WR_CMPLT;
            }
            
            /* Set status error transfer and arbitration lost */
            I2C_mstrStatus |= (I2C_MSTAT_ERR_ARB_LOST | 
                                            I2C_MSTAT_ERR_XFER);
            
            /* Reset State Machine to IDLE */
            I2C_state = I2C_SM_IDLE;
        }
    #endif  /* End (0u != (I2C_MODE & I2C_MODE_MULTI_MASTER_ENABLE)) */
     
    /* Check for Master operation mode */
    if (0u != (I2C_state & I2C_SM_MASTER))
    {
        #if (0u != (I2C_MODE & I2C_MODE_MASTER))
            
            /* Enter Master state machine */
            if (I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                /* Clear external or previous stop event */
                tmpCsr &= ~I2C_CSR_STOP_STATUS;  /* Clear STOP bit */
                
                switch (I2C_state)
                {
                    case I2C_SM_MSTR_WR_ADDR:    /* After address is sent, WRITE some data */
                    case I2C_SM_MSTR_RD_ADDR:    /* After address is sent, READ some data */
                    
                        /* Check for Slave address ACK */
                        if (I2C_CHECK_ADDR_ACK(tmpCsr))  /* Check ACK */
                        {
                            /* Setup for transmit or receive of data */
                            if (I2C_state == I2C_SM_MSTR_WR_ADDR)   /* TRANSMIT data */
                            {
                                if (I2C_mstrWrBufSize > 0u)    /* Check if at least one byte is transfered */
                                {
                                    /* Load first data byte */
                                    I2C_DATA_REG = I2C_mstrWrBufPtr[0u];
                                    I2C_TRANSMIT_DATA;            /* Transmit data */
                                    I2C_mstrWrBufIndex = 1u;      /* Set index to 2nd location */
                                    
                                    /* Set transmit state until done */
                                    I2C_state = I2C_SM_MSTR_WR_DATA; 
                                }
                                else   /* No data to tranfer */
                                {
                                    /* Handles 0 bytes transfer - Not HALT is allowed */
                                    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
                                        I2C_GENERATE_STOP;     /* Generate STOP */
                                        
                                        /* Set WRITE complete */
                                        I2C_mstrStatus |= I2C_MSTAT_WR_CMPLT;
                                        
                                        /* Reset State Machine to IDLE */
                                        I2C_state  = I2C_SM_IDLE;
                                        
                                    #else  /* The PSoC3 ES3 only handles this well */
                                        if (I2C_CHECK_NO_STOP(I2C_mstrControl))
                                        {
                                            /* Reset State Machine to HALT, expect RESTART */
                                            I2C_state  = I2C_SM_MSTR_HALT;
                                            
                                            /* Set WRITE complete and Master HALTED */
                                            I2C_mstrStatus |= (I2C_MSTAT_WR_CMPLT |
                                                                            I2C_MSTAT_XFER_HALT);
                                            
                                            I2C_DisableInt();
                                        }
                                        else  /* Do normal STOP */
                                        {
                                            I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                                            I2C_GENERATE_STOP;         /* Generate STOP */
                                        }
                                        
                                    #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
                                }
                            }
                            else  /* Master Receive data */
                            {
                                I2C_READY_TO_READ;     /* Ready to READ data */
                                
                                /* Set state machine to READ data */
                                I2C_state  = I2C_SM_MSTR_RD_DATA;
                            }
                        }
                        /* Check for Slave address NAK */
                        else if (I2C_CHECK_ADDR_NAK(tmpCsr))  /* Check NACK */
                        {
                            if (I2C_CHECK_NO_STOP(I2C_mstrControl))
                            {
                                /* Check State for READ one: SM_MSTR_RD_ADDR or SM_MSTR_RD_DATA */
                                if (0u != (I2C_state & I2C_SM_MSTR_RD))
                                {
                                    /* Set READ complete */
                                    I2C_mstrStatus |= I2C_MSTAT_RD_CMPLT;
                                }
                                else /* All other: should be only write states */
                                {
                                    /* Set WRITE complete */
                                    I2C_mstrStatus |= I2C_MSTAT_WR_CMPLT;
                                }
                                
                                /* Set Address NAK Error and Master HALTED */
                                I2C_mstrStatus |= (I2C_MSTAT_XFER_HALT |
                                                                I2C_MSTAT_ERR_ADDR_NAK |
                                                                I2C_MSTAT_ERR_XFER);
                                                                
                                /* Reset State Machine to HALT, expect RESTART */
                                I2C_state  = I2C_SM_MSTR_HALT;
                                I2C_DisableInt();
                            }
                            else  /* Do normal Stop */
                            {
                                I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                                I2C_GENERATE_STOP;         /* Generate STOP */
                                
                                /* Set Address NAK and ERR transfer */
                                I2C_mstrStatus |= (I2C_MSTAT_ERR_ADDR_NAK | 
                                                                I2C_MSTAT_ERR_XFER);
                            }
                        }
                        /* Should never gets here: Address status is NOT set */
                        else
                        {
                            CYASSERT(0);
                        }
                        break;
                        
                    case I2C_SM_MSTR_WR_DATA:    /* Write data to slave */
                        
                        if (I2C_CHECK_DATA_ACK(tmpCsr))       /* Check ACK */
                        {
                            /* Check if end buffer */
                            if (I2C_mstrWrBufIndex  < I2C_mstrWrBufSize)
                            {
                                 /* Load first data byte  */
                                I2C_DATA_REG = I2C_mstrWrBufPtr[I2C_mstrWrBufIndex];
                                I2C_TRANSMIT_DATA;     /* Transmit */
                                
                                I2C_mstrWrBufIndex++;  /* Advance to data location */
                            }
                            else   /* Last byte was transmitted, send STOP */
                            {
                                if (I2C_CHECK_NO_STOP(I2C_mstrControl))
                                {
                                    /* Reset State Machine to HALT, expect RESTART */
                                    I2C_state  = I2C_SM_MSTR_HALT;
                                    
                                    /* Set WRITE complete and Master HALTED */
                                    I2C_mstrStatus |= (I2C_MSTAT_WR_CMPLT |
                                                                    I2C_MSTAT_XFER_HALT);
                                    
                                    I2C_DisableInt();
                                }
                                else  /* Do normal STOP */
                                {
                                    I2C_Workaround();          /* Workaround for CDT 78083 */
                                    I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                                    I2C_GENERATE_STOP;         /* Generate STOP */
                                }
                            }
                        }
                        else /* If last byte NAKed, stop transmit and send STOP */
                        {
                            /* Check STOP generation */
                            if (I2C_CHECK_NO_STOP(I2C_mstrControl))
                            {
                                /* Reset State Machine to HALT, expect RESTART */
                                I2C_state  = I2C_SM_MSTR_HALT;
                                
                                /* Set WRITE complete, SHORT transfer and Master HALTED */
                                I2C_mstrStatus |= (I2C_MSTAT_WR_CMPLT |
                                                                I2C_MSTAT_XFER_HALT |
                                                                I2C_MSTAT_ERR_SHORT_XFER |
                                                                I2C_MSTAT_ERR_XFER);
                                
                                I2C_DisableInt();
                            }
                            else  /* Do normal STOP */
                            {
                                I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                                I2C_GENERATE_STOP;         /* Generate STOP */
                                                                
                                /* Set SHORT and ERR transfer */
                                I2C_mstrStatus |= (I2C_MSTAT_ERR_SHORT_XFER | 
                                                                I2C_MSTAT_ERR_XFER);
                            }
                        }
                        break;
                        
                    case I2C_SM_MSTR_RD_DATA:    /* Data received */
                        
                        I2C_mstrRdBufPtr[I2C_mstrRdBufIndex] = I2C_DATA_REG;
                        I2C_mstrRdBufIndex++;      /* Inc pointer */
                        /* Check if end of buffer */
                        if (I2C_mstrRdBufIndex < I2C_mstrRdBufSize)
                        {
                            I2C_ACK_AND_RECEIVE;       /* ACK and receive */
                        }
                        else   /* End of data, generate a STOP */
                        {
                            if (I2C_CHECK_NO_STOP(I2C_mstrControl)) /* Check STOP generation */
                            {
                                /* Reset State Machine to HALT, expect RESTART */
                                I2C_state = I2C_SM_MSTR_HALT;
                                
                                /* Set READ complete and Master HALTED */
                                I2C_mstrStatus |= (I2C_MSTAT_RD_CMPLT |
                                                                I2C_MSTAT_XFER_HALT );
                            }
                            else   /* Do normal STOP */
                            {
                                I2C_ENABLE_INT_ON_STOP;        /* Enable interrupt on STOP, to catch it */
                                I2C_NAK_AND_RECEIVE;           /* NACK and TRY to generate STOP */
                            }
                        }
                        break;
                        
                    default: /* This is an invalid state and should not occur */
                        
                        CYASSERT(0);
                        break;
                }
            }
            
            /* Check if STOP was detected */
            if (I2C_CHECK_STOP_STS(tmpCsr))
            {
                /* Check State for READ one: SM_MSTR_RD_ADDR or SM_MSTR_RD_DATA */
                if (0u != (I2C_state & I2C_SM_MSTR_RD))
                {
                    /* Set READ complete */
                    I2C_mstrStatus |= I2C_MSTAT_RD_CMPLT;
                }
                else /* All other: should be only write states */
                {
                    /* Set WRITE complete */
                    I2C_mstrStatus |= I2C_MSTAT_WR_CMPLT;
                }
                
                /* Catch STOP, disable the interrupt on STOP */
                I2C_DISABLE_INT_ON_STOP;
                I2C_state = I2C_SM_IDLE;  /* Set state to IDLE */
            }
        #endif  /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */
    }
    else    /* Slave */
    {
        #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
            /* Check to see if a Start/Address is detected */
            if (0u != (tmpCsr & I2C_CSR_ADDRESS))
            {
                /* Clears STOP status bit. This status bit sets by ANY of STOP condition detection on the bus */
                tmpCsr &= ~I2C_CSR_STOP_STATUS;  /* Clear STOP bit */
                
                /* This is a Start or ReStart. Reset the state machine and check for a Read/Write condition */
                
                /* Check for software address detection */
                #if (I2C_ADDR_DECODE == I2C_SW_DECODE)
                    tmp8 = ((I2C_DATA_REG >> I2C_SLAVE_ADDR_SHIFT) & 
                             I2C_SLAVE_ADDR_MASK);
                    if (tmp8 == I2C_slAddress)   /* Check for address match */
                    {
                        /* Check for read or write command */
                        if (0u != (I2C_DATA_REG & I2C_READ_FLAG))
                        {
                            /*        Place code to prepare read buffer here           */
                            /* `#START I2C_SW_PREPARE_READ_BUF_interrupt` */

                            /* `#END` */
                            
                            /* Prepare next opeation to read, get data and place in data register */
                            if (I2C_slRdBufIndex < I2C_slRdBufSize)
                            {
                                /* Load first data byte */
                                I2C_DATA_REG = I2C_slRdBufPtr[I2C_slRdBufIndex];
                                I2C_ACK_AND_TRANSMIT;  /* ACK and transmit */
                                I2C_slRdBufIndex++;    /* Advance to data location */
                                
                                /* Set READ activity */
                                I2C_slStatus |= I2C_SSTAT_RD_BUSY;
                            }
                            else    /* Data overflow */
                            {
                                I2C_DATA_REG = 0xFFu;    /* Out of range, send 0xFF */
                                I2C_ACK_AND_TRANSMIT;    /* ACK and transmit */
                                
                                /* Set READ activity with OVERFLOW */
                                I2C_slStatus  |= (I2C_SSTAT_RD_BUSY | 
                                                               I2C_SSTAT_RD_ERR_OVFL);
                            }
                            
                            I2C_state = I2C_SM_SL_RD_DATA; /* Prepare for Read transaction */
                        }
                        else  /* Start of a Write transaction, ready to write of the first byte */
                        {
                            /* Prepare to write the first byte */
                            I2C_ACK_AND_RECEIVE;
                            I2C_state = I2C_SM_SL_WR_DATA; /* Prepare for Write transaction */
                            
                            /* Set WRITE activity */
                            I2C_slStatus |= I2C_SSTAT_WR_BUSY;
                            I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on Stop */
                        }
                    }
                    else   /* No address match */
                    {
                        /*     Place code to compare for additional address here    */
                        /* `#START I2C_SW_ADDR_COMPARE_interruptStart` */

                        /* `#END` */
                        
                            I2C_NAK_AND_RECEIVE;   /* NAK address */
                        
                        /* Place code to end of condition for NACK generation here */
                        /* `#START I2C_SW_ADDR_COMPARE_interruptEnd`  */

                        /* `#END` */
                    }
                    
                #else  /* Hardware address detection */
                    /* Check for read or write command */
                    if (0u != (I2C_DATA_REG & I2C_READ_FLAG))
                    {
                        /*          Place code to prepare read buffer here         */
                        /* `#START I2C_HW_PREPARE_READ_BUF_interrupt` */

                        /* `#END` */
                         
                        /* Prepare next opeation to read, get data and place in data register */
                        if (I2C_slRdBufIndex < I2C_slRdBufSize)
                        {
                            /* Load first data byte */
                            I2C_DATA_REG = I2C_slRdBufPtr[I2C_slRdBufIndex];
                            I2C_ACK_AND_TRANSMIT;  /* ACK and transmit */
                            I2C_slRdBufIndex++;    /* Advance to data location */
                            
                            /* Set READ activity */
                            I2C_slStatus  |= I2C_SSTAT_RD_BUSY;
                        }
                        else    /* Data overflow */
                        {
                            I2C_DATA_REG = 0xFFu;    /* Out of range, send 0xFF  */
                            I2C_ACK_AND_TRANSMIT;    /* ACK and transmit */
                            
                            /* Set READ activity with OVERFLOW */
                            I2C_slStatus  |= (I2C_SSTAT_RD_BUSY |
                                                           I2C_SSTAT_RD_ERR_OVFL);
                        }
                        
                        I2C_state = I2C_SM_SL_RD_DATA;    /* Prepare for Read transaction */
                    }
                    else  /* Start of a Write transaction, ready to write of the first byte */
                    {
                        /* Prepare to write the first byte */
                        I2C_ACK_AND_RECEIVE;       /* ACK and ready to receive addr */
                        I2C_state = I2C_SM_SL_WR_DATA;    /* Prepare for write transaction */
                        
                        /* Set WRITE activity */
                        I2C_slStatus |= I2C_SSTAT_WR_BUSY;
                        I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on Stop */
                    }
                    
                #endif  /* End (I2C_ADDR_DECODE == I2C_SW_DECODE) */
            }
            /* Check for data transfer */
            else if (I2C_CHECK_BYTE_COMPLETE(tmpCsr))  
            {
                /* Data write from Master to Slave */
                if (I2C_state == I2C_SM_SL_WR_DATA)
                {
                    if (I2C_slWrBufIndex < I2C_slWrBufSize)       /* Check for valid range */
                    {
                        tmp8 = I2C_DATA_REG;                        /* Get data, to ACK quickly */
                        I2C_ACK_AND_RECEIVE;                        /* ACK and ready to receive */
                        I2C_slWrBufPtr[I2C_slWrBufIndex] = tmp8; /* Write data to array */
                        I2C_slWrBufIndex++;                        /* Inc pointer */
                    }
                    else
                    {
                        I2C_NAK_AND_RECEIVE;       /* NAK cause beyond write area */
                        
                        /* Set OVERFLOW, write completes on Stop */
                        I2C_slStatus |= I2C_SSTAT_WR_ERR_OVFL;
                    }
                }
                /* Data Read from Slave to Master */
                else if (I2C_state == I2C_SM_SL_RD_DATA)
                {
                    if (I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        if (I2C_slRdBufIndex < I2C_slRdBufSize) 
                        {
                             /* Get data from array */
                            I2C_DATA_REG = I2C_slRdBufPtr[I2C_slRdBufIndex];
                            I2C_TRANSMIT_DATA;         /* Send Data */
                            I2C_slRdBufIndex++;        /* Inc pointer */
                        }
                        else   /* Over flow */
                        {
                            I2C_DATA_REG = 0xFFu;  /* Send 0xFF at the end of the buffer */
                            I2C_TRANSMIT_DATA;     /* Send Data */
                            
                            /* Set OVERFLOW */
                            I2C_slStatus |= I2C_SSTAT_RD_ERR_OVFL;
                        }
                    }
                    else  /* Last byte NAKed, done */
                    {
                        I2C_DATA_REG = 0xFFu;  /* End of read transaction */
                        I2C_NAK_AND_TRANSMIT;  /* Clear transmit bit at the end of read transaction */
                         
                        I2C_slStatus &= ~I2C_SSTAT_RD_BUSY;   /* Clear RD_BUSY Flag */
                        I2C_slStatus |= I2C_SSTAT_RD_CMPLT;    /* Set RD_CMPLT Flag */
                        
                        I2C_state = I2C_SM_IDLE;  /* Return to IDLE state */
                    }
                }
                /* This is an invalid state and should not occur */
                else
                {
                    CYASSERT(0);
                }   /* End Transfer mode */
            }
            /* EMPTY else: there is no Slave enable event. */
            else
            {
                /* The Multi-Master-Slave exist here when arbitrage happen on other than
                   address stage of transaction == No Slave enable event. */
            }
            
            /* Check if STOP was detected */
            if (I2C_CHECK_STOP_STS(tmpCsr))
            {
                /* The Write transaction only IE on STOP, so Read never gets here */
                /* The WR_BUSY flag will be cleared at the end of "Write-ReStart-Read-Stop" transaction */
                
                I2C_slStatus &= ~I2C_SSTAT_WR_BUSY;   /* Clear WR_BUSY Flag */
                I2C_slStatus |= I2C_SSTAT_WR_CMPLT;    /* Set WR_CMPT Flag */
                
                I2C_DISABLE_INT_ON_STOP;               /* Disable interrupt on STOP */
                I2C_state = I2C_SM_IDLE;  /* Return to IDLE */
            }
        
        #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
    }
    
    #if (CY_PSOC3_ES2 && (I2C_I2C_IRQ__ES2_PATCH))
        I2C_ISR_PATCH();
    #endif  /* End (CY_PSOC3_ES2 && (I2C_I2C_IRQ__ES2_PATCH)) */
}


/* [] END OF FILE */
