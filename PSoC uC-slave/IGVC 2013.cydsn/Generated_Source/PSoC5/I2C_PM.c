/*******************************************************************************
* File Name: I2C_PM.c
* Version 3.1
*
* Description:
*  This file provides Low power mode APIs for I2C component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C.h"

I2C_BACKUP_STRUCT I2C_backup =
{   
    0u, /* enableState */
    
    #if (I2C_IMPLEMENTATION == I2C_FF)
        I2C_DEFAULT_XCFG,  /* xcfg */
        I2C_DEFAULT_CFG,   /* cfg */
        
        #if (I2C_MODE & I2C_MODE_SLAVE)
            I2C_DEFAULT_ADDR, /* addr */
        #endif  /* End (I2C_MODE & I2C_MODE_SLAVE) */
        
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
           I2C_DEFAULT_DIVIDE_FACTOR,
        #else
            LO8(I2C_DEFAULT_DIVIDE_FACTOR), /*  clk_div1 */
            HI8(I2C_DEFAULT_DIVIDE_FACTOR), /*  clk_div2 */
        #endif  /* End  (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
        
    #else /* (I2C_IMPLEMENTATION == I2C_UDB) */
        I2C_DEFAULT_CFG,    /* control */
        
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
            I2C_INT_ENABLE_MASK, /* int_mask */
            
            #if (I2C_MODE & I2C_MODE_SLAVE)
                I2C_DEFAULT_ADDR, /* addr */
            #endif  /* End (I2C_MODE & I2C_MODE_SLAVE) */
        #else
            /* Retention registers for ES3:
                - Status Int mask: int_mask;
                - D0 register: addr;
                - Auxiliary Control: aux_ctl;
                - Period Register: always 7;
                - D0 and D1: clock generator 7, 15;
            */
        #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1)*/
    #endif  /* End ((I2C_IMPLEMENTATION == I2C_FF) */
};


/*******************************************************************************
* Function Name: I2C_SaveConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: disables I2C Master(if was enabled before go
*  to sleep), enables I2C backup regulator. Waits while on-going transaction be 
*  will completed and I2C will be ready go to sleep. All incoming transaction 
*  will be NACKed till power down will be asserted. The address match event 
*  wakes up the chip. 
*  Wakeup on address match disabled: saves I2C configuration and non-retention 
*  register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  I2C_backup - used to save component configuration and none-retention
*  registers before enter sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void I2C_SaveConfig(void) 
{
    #if (I2C_IMPLEMENTATION == I2C_FF)
        #if (I2C_ENABLE_WAKEUP)
            uint8 enableInterrupts;
        #endif  /* End (I2C_ENABLE_WAKEUP) */
        
        /* Store regiters in either Sleep mode */
        I2C_backup.cfg  = I2C_CFG_REG;
        I2C_backup.xcfg = I2C_XCFG_REG;
        
            #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
                I2C_backup.addr = I2C_ADDR_REG;
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
            
            #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
                I2C_backup.clk_div  = I2C_CLKDIV_REG;
            
            #else
                I2C_backup.clk_div1  = I2C_CLKDIV1_REG;
                I2C_backup.clk_div2  = I2C_CLKDIV2_REG;
            #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
        
        #if (I2C_ENABLE_WAKEUP)
            /* Need to disable Master */
            #if (0u != (I2C_MODE & I2C_MODE_MASTER))
                if (0u != (I2C_CFG_REG & I2C_ENABLE_MASTER))
                {
                    I2C_CFG_REG &= ~I2C_ENABLE_MASTER;
                    
                    /* Store state of I2C Master */
                    I2C_backup.enableState = I2C_ENABLE_MASTER;
                }
            #endif  /* ((0u != (I2C_MODE & I2C_MODE_MASTER)) */
            
            /* Enable the I2C regulator backup */
            enableInterrupts = CyEnterCriticalSection();
            I2C_PWRSYS_CR1_REG |= I2C_PWRSYS_CR1_I2C_REG_BACKUP;
            CyExitCriticalSection(enableInterrupts);
            
            /* 1) Set force NACK to ignore I2C transactions 
               2) Wait while I2C will be ready go to Sleep 
               3) These bits are cleared on wake up */
            I2C_XCFG_REG |= I2C_XCFG_FORCE_NACK;
            while (0u == (I2C_XCFG_REG  & I2C_XCFG_RDY_TO_SLEEP));
            
        #endif  /* End (I2C_ENABLE_WAKEUP) */
        
    #else
        /* Store only address match bit */
        I2C_backup.control = (I2C_CFG_REG & I2C_CTRL_ANY_ADDRESS_MASK);
        
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
            /* Store interrupt mask bits */
            I2C_backup.int_mask = I2C_INT_MASK_REG;
            
            #if (I2C_MODE & I2C_MODE_SLAVE)
                /* Store slave address */
                I2C_backup.addr = I2C_ADDR_REG;
            #endif  /* End (I2C_MODE & I2C_MODE_SLAVE) */
            
        #else
            /* Retention registers for ES3:
                - Status Int mask: int_mask;
                - D0 register: addr;
                - Auxiliary Control: aux_ctl;
                - Period Register: always 7;
                - D0 and D1: clock generator 7, 15;
            */
        #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
        
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */
    
    /* Disable interrupts */
    I2C_DisableInt();
}


/*******************************************************************************
* Function Name: I2C_Sleep
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: All incoming transaction will be NACKed till 
*  power down will be asserted. The address match event wakes up the chip.
*  Wakeup on address match disabled: Disables active mode power template bits or 
*  clock gating as appropriate. Saves I2C configuration and non-retention 
*  register values. 
*  Disables I2C interrupt.
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
void I2C_Sleep(void) 
{
    #if (I2C_ENABLE_WAKEUP)
        /* The I2C block should be always enabled if used as wakeup source */
        I2C_backup.enableState = 0u;
        
    #else
        /* Store I2C enable state */
        if (I2C_IS_I2C_ENABLE(I2C_I2C_ENABLE_REG))
        {
            I2C_backup.enableState = 1u;
            I2C_Stop();
        }
        else
        {
            I2C_backup.enableState = 0u;
        }
    #endif  /* End (I2C_ENABLE_WAKEUP) */
    
    I2C_SaveConfig();
}


/*******************************************************************************
* Function Name: I2C_RestoreConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep), disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and 
*  non-retention register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  I2C_backup - used to save component configuration and 
*  none-retention registers before exit sleep mode.
*
*******************************************************************************/
void I2C_RestoreConfig(void) 
{
    #if (I2C_IMPLEMENTATION == I2C_FF)
        #if (I2C_ENABLE_WAKEUP)
            uint8 enableInterrupts;
            
            /* Disable the I2C regulator backup */
            enableInterrupts = CyEnterCriticalSection();
            if (0u != (I2C_PWRSYS_CR1_I2C_REG_BACKUP & I2C_PWRSYS_CR1_REG))
            {
                I2C_PWRSYS_CR1_REG &= ~I2C_PWRSYS_CR1_I2C_REG_BACKUP;
                CyExitCriticalSection(enableInterrupts);
                
                /* Need to re-enable Master */
                #if (0u != (I2C_MODE & I2C_MODE_MASTER))
                    if (I2C_backup.enableState != 0u)
                    {
                        I2C_CFG_REG |= I2C_ENABLE_MASTER;
                        
                        /* Clear state of I2C Master */
                        I2C_backup.enableState = 0u;
                    }
                #endif  /* End (0u != (I2C_CFG_REG & I2C_ENABLE_MASTER)) */
            }
            else
            {
                /* Disable power to I2C block before register restore */
                I2C_ACT_PWRMGR_REG  &= ~I2C_ACT_PWR_EN;
                I2C_STBY_PWRMGR_REG &= ~I2C_STBY_PWR_EN;
                 
                /* The I2C_PWRSYS_CR1_I2C_REG_BACKUP already cleaned by PM APIs */
                CyExitCriticalSection(enableInterrupts);
                
                /* Enable component after restore complete */
                I2C_backup.enableState = 1u;
                
        #endif  /* End (I2C_ENABLE_WAKEUP) */
                
                /* Restore component registers */
                I2C_XCFG_REG = I2C_backup.xcfg;
                I2C_CFG_REG  = I2C_backup.cfg;
                
                #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
                    I2C_ADDR_REG = I2C_backup.addr;
                #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
                
                #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
                    I2C_CLKDIV_REG =I2C_backup.clk_div;
                
                #else
                    I2C_CLKDIV1_REG = I2C_backup.clk_div1;
                    I2C_CLKDIV2_REG = I2C_backup.clk_div2;
                #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
           
        #if (I2C_ENABLE_WAKEUP)
            }
        #endif  /* End (I2C_ENABLE_WAKEUP) */
        
    #else
        
        #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)
            uint8 enableInterrupts;
            
            /* Enable interrupts from block */
            enableInterrupts = CyEnterCriticalSection();
            I2C_INT_ENABLE_REG |= I2C_INT_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);
            
            /* Restore interrupt mask bits */
            I2C_INT_MASK_REG |= I2C_backup.int_mask;
            
            #if (0u != (I2C_MODE & I2C_MODE_MASTER))
                /* Restore Master Clock generator */
                I2C_MCLK_PRD_REG = I2C_MCLK_PERIOD_VALUE;
                I2C_MCLK_CMP_REG = I2C_MCLK_COMPARE_VALUE;
            #endif /* End (0u != (I2C_MODE & I2C_MODE_MASTER)) */
            
            #if (0u != (I2C_MODE & I2C_MODE_SLAVE))
                /* Store slave address */
                I2C_ADDR_REG = I2C_backup.addr;
                
                /* Restore slave bit counter period */
                I2C_PERIOD_REG = I2C_PERIOD_VALUE;
            #endif  /* End (0u != (I2C_MODE & I2C_MODE_SLAVE)) */
            
        #else
            /* Retention registers for ES3:
                - Status Int mask: int_mask;
                - D0 register: addr;
                - Auxiliary Control: aux_ctl;
                - Period Register: always 7;
                - D0 and D1: clock generator 7, 15;
            */
        #endif  /* End (CY_PSOC3_ES2 || CY_PSOC5_ES1) */
        
         /* Restore CFG register */
        I2C_CFG_REG = I2C_backup.control;
        
    #endif  /* End (I2C_IMPLEMENTATION == I2C_FF) */
    
    /* Enable interrupts */
    I2C_EnableInt();
}


/*******************************************************************************
* Function Name: I2C_Wakeup
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep) and disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and non-retention 
*  register values. Restores Active mode power template bits or clock gating as 
*  appropriate.
*  The I2C interrupt remains disabled after function call.
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
void I2C_Wakeup(void) 
{
    /* Restore I2C register settings */
    I2C_RestoreConfig();
    
    /* Restore I2C Enable state */
    if (0u != I2C_backup.enableState)
    {
        I2C_Enable();
    }
}


/* [] END OF FILE */
