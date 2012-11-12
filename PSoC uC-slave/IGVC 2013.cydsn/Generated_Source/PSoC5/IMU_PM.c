/*******************************************************************************
* File Name: IMU.c
* Version 2.10
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "IMU.h"


/***************************************
* Local data allocation
***************************************/

static IMU_BACKUP_STRUCT  IMU_backup =
{
    /* enableState - disabled */
    0u,
};        



/*******************************************************************************
* Function Name: IMU_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  IMU_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void IMU_SaveConfig(void)
{
    /* PSoC3 ES2 or early, PSoC5 ES1*/
    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)

        #if(IMU_CONTROL_REG_REMOVED == 0u)
            IMU_backup.cr = IMU_CONTROL_REG;
        #endif /* End IMU_CONTROL_REG_REMOVED */

        #if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
            IMU_backup.rx_period = IMU_RXBITCTR_PERIOD_REG;
            IMU_backup.rx_mask = IMU_RXSTATUS_MASK_REG;
            #if (IMU_RXHW_ADDRESS_ENABLED)
                IMU_backup.rx_addr1 = IMU_RXADDRESS1_REG;
                IMU_backup.rx_addr2 = IMU_RXADDRESS2_REG;
            #endif /* End IMU_RXHW_ADDRESS_ENABLED */
        #endif /* End IMU_RX_ENABLED | IMU_HD_ENABLED*/

        #if(IMU_TX_ENABLED)
            #if(IMU_TXCLKGEN_DP)
                IMU_backup.tx_clk_ctr = IMU_TXBITCLKGEN_CTR_REG;
                IMU_backup.tx_clk_compl = IMU_TXBITCLKTX_COMPLETE_REG;
            #else
                IMU_backup.tx_period = IMU_TXBITCTR_PERIOD_REG;
            #endif /*End IMU_TXCLKGEN_DP */
            IMU_backup.tx_mask = IMU_TXSTATUS_MASK_REG;
        #endif /*End IMU_TX_ENABLED */

    /* PSoC3 ES3 or later, PSoC5 ES2 or later*/
    #elif (CY_PSOC3_ES3 || CY_PSOC5_ES2)

        #if(IMU_CONTROL_REG_REMOVED == 0u)
            IMU_backup.cr = IMU_CONTROL_REG;
        #endif /* End IMU_CONTROL_REG_REMOVED */

    #endif  /* End PSOC3_ES3 || PSOC5_ES2 */
}


/*******************************************************************************
* Function Name: IMU_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  IMU_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void IMU_RestoreConfig(void)
{
    /* PSoC3 ES2 or early, PSoC5 ES1*/
    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)

        #if(IMU_CONTROL_REG_REMOVED == 0u)
            IMU_CONTROL_REG = IMU_backup.cr;
        #endif /* End IMU_CONTROL_REG_REMOVED */

        #if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
            IMU_RXBITCTR_PERIOD_REG = IMU_backup.rx_period;
            IMU_RXSTATUS_MASK_REG = IMU_backup.rx_mask;
            #if (IMU_RXHW_ADDRESS_ENABLED)
                IMU_RXADDRESS1_REG = IMU_backup.rx_addr1;
                IMU_RXADDRESS2_REG = IMU_backup.rx_addr2;
            #endif /* End IMU_RXHW_ADDRESS_ENABLED */
        #endif  /* End (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */

        #if(IMU_TX_ENABLED)
            #if(IMU_TXCLKGEN_DP)
                IMU_TXBITCLKGEN_CTR_REG = IMU_backup.tx_clk_ctr;
                IMU_TXBITCLKTX_COMPLETE_REG = IMU_backup.tx_clk_compl;
            #else
                IMU_TXBITCTR_PERIOD_REG = IMU_backup.tx_period;
            #endif /*End IMU_TXCLKGEN_DP */
            IMU_TXSTATUS_MASK_REG = IMU_backup.tx_mask;
        #endif /*End IMU_TX_ENABLED */

    /* PSoC3 ES3 or later, PSoC5 ES2 or later*/
    #elif (CY_PSOC3_ES3 || CY_PSOC5_ES2)

        #if(IMU_CONTROL_REG_REMOVED == 0u)
            IMU_CONTROL_REG = IMU_backup.cr;
        #endif /* End IMU_CONTROL_REG_REMOVED */

    #endif  /* End PSOC3_ES3 || PSOC5_ES2 */
}


/*******************************************************************************
* Function Name: IMU_Sleep
********************************************************************************
*
* Summary:
*  Stops and saves the user configuration. Should be called 
*  just prior to entering sleep.
*  
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  IMU_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void IMU_Sleep(void)
{

    #if(IMU_RX_ENABLED || IMU_HD_ENABLED)
        if((IMU_RXSTATUS_ACTL_REG  & IMU_INT_ENABLE) != 0u) 
        {
            IMU_backup.enableState = 1u;
        }
        else
        {
            IMU_backup.enableState = 0u;
        }
    #else
        if((IMU_TXSTATUS_ACTL_REG  & IMU_INT_ENABLE) !=0u)
        {
            IMU_backup.enableState = 1u;
        }
        else
        {
            IMU_backup.enableState = 0u;
        }
    #endif /* End IMU_RX_ENABLED || IMU_HD_ENABLED*/

    IMU_Stop();
    IMU_SaveConfig();
}


/*******************************************************************************
* Function Name: IMU_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called
*  just after awaking from sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  IMU_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void IMU_Wakeup(void)
{
    IMU_RestoreConfig();
    #if( (IMU_RX_ENABLED) || (IMU_HD_ENABLED) )
        IMU_ClearRxBuffer();
    #endif /* End (IMU_RX_ENABLED) || (IMU_HD_ENABLED) */
    #if(IMU_TX_ENABLED || IMU_HD_ENABLED)
        IMU_ClearTxBuffer();
    #endif /* End IMU_TX_ENABLED || IMU_HD_ENABLED */

    if(IMU_backup.enableState != 0u)
    {
        IMU_Enable();
    } 
}


/* [] END OF FILE */
