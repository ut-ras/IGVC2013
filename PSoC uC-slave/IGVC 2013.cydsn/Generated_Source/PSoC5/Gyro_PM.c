/*******************************************************************************
* File Name: Gyro.c
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

#include "Gyro.h"


/***************************************
* Local data allocation
***************************************/

static Gyro_BACKUP_STRUCT  Gyro_backup =
{
    /* enableState - disabled */
    0u,
};        



/*******************************************************************************
* Function Name: Gyro_SaveConfig
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
*  Gyro_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Gyro_SaveConfig(void)
{
    /* PSoC3 ES2 or early, PSoC5 ES1*/
    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)

        #if(Gyro_CONTROL_REG_REMOVED == 0u)
            Gyro_backup.cr = Gyro_CONTROL_REG;
        #endif /* End Gyro_CONTROL_REG_REMOVED */

        #if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
            Gyro_backup.rx_period = Gyro_RXBITCTR_PERIOD_REG;
            Gyro_backup.rx_mask = Gyro_RXSTATUS_MASK_REG;
            #if (Gyro_RXHW_ADDRESS_ENABLED)
                Gyro_backup.rx_addr1 = Gyro_RXADDRESS1_REG;
                Gyro_backup.rx_addr2 = Gyro_RXADDRESS2_REG;
            #endif /* End Gyro_RXHW_ADDRESS_ENABLED */
        #endif /* End Gyro_RX_ENABLED | Gyro_HD_ENABLED*/

        #if(Gyro_TX_ENABLED)
            #if(Gyro_TXCLKGEN_DP)
                Gyro_backup.tx_clk_ctr = Gyro_TXBITCLKGEN_CTR_REG;
                Gyro_backup.tx_clk_compl = Gyro_TXBITCLKTX_COMPLETE_REG;
            #else
                Gyro_backup.tx_period = Gyro_TXBITCTR_PERIOD_REG;
            #endif /*End Gyro_TXCLKGEN_DP */
            Gyro_backup.tx_mask = Gyro_TXSTATUS_MASK_REG;
        #endif /*End Gyro_TX_ENABLED */

    /* PSoC3 ES3 or later, PSoC5 ES2 or later*/
    #elif (CY_PSOC3_ES3 || CY_PSOC5_ES2)

        #if(Gyro_CONTROL_REG_REMOVED == 0u)
            Gyro_backup.cr = Gyro_CONTROL_REG;
        #endif /* End Gyro_CONTROL_REG_REMOVED */

    #endif  /* End PSOC3_ES3 || PSOC5_ES2 */
}


/*******************************************************************************
* Function Name: Gyro_RestoreConfig
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
*  Gyro_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Gyro_RestoreConfig(void)
{
    /* PSoC3 ES2 or early, PSoC5 ES1*/
    #if (CY_PSOC3_ES2 || CY_PSOC5_ES1)

        #if(Gyro_CONTROL_REG_REMOVED == 0u)
            Gyro_CONTROL_REG = Gyro_backup.cr;
        #endif /* End Gyro_CONTROL_REG_REMOVED */

        #if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
            Gyro_RXBITCTR_PERIOD_REG = Gyro_backup.rx_period;
            Gyro_RXSTATUS_MASK_REG = Gyro_backup.rx_mask;
            #if (Gyro_RXHW_ADDRESS_ENABLED)
                Gyro_RXADDRESS1_REG = Gyro_backup.rx_addr1;
                Gyro_RXADDRESS2_REG = Gyro_backup.rx_addr2;
            #endif /* End Gyro_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */

        #if(Gyro_TX_ENABLED)
            #if(Gyro_TXCLKGEN_DP)
                Gyro_TXBITCLKGEN_CTR_REG = Gyro_backup.tx_clk_ctr;
                Gyro_TXBITCLKTX_COMPLETE_REG = Gyro_backup.tx_clk_compl;
            #else
                Gyro_TXBITCTR_PERIOD_REG = Gyro_backup.tx_period;
            #endif /*End Gyro_TXCLKGEN_DP */
            Gyro_TXSTATUS_MASK_REG = Gyro_backup.tx_mask;
        #endif /*End Gyro_TX_ENABLED */

    /* PSoC3 ES3 or later, PSoC5 ES2 or later*/
    #elif (CY_PSOC3_ES3 || CY_PSOC5_ES2)

        #if(Gyro_CONTROL_REG_REMOVED == 0u)
            Gyro_CONTROL_REG = Gyro_backup.cr;
        #endif /* End Gyro_CONTROL_REG_REMOVED */

    #endif  /* End PSOC3_ES3 || PSOC5_ES2 */
}


/*******************************************************************************
* Function Name: Gyro_Sleep
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
*  Gyro_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Gyro_Sleep(void)
{

    #if(Gyro_RX_ENABLED || Gyro_HD_ENABLED)
        if((Gyro_RXSTATUS_ACTL_REG  & Gyro_INT_ENABLE) != 0u) 
        {
            Gyro_backup.enableState = 1u;
        }
        else
        {
            Gyro_backup.enableState = 0u;
        }
    #else
        if((Gyro_TXSTATUS_ACTL_REG  & Gyro_INT_ENABLE) !=0u)
        {
            Gyro_backup.enableState = 1u;
        }
        else
        {
            Gyro_backup.enableState = 0u;
        }
    #endif /* End Gyro_RX_ENABLED || Gyro_HD_ENABLED*/

    Gyro_Stop();
    Gyro_SaveConfig();
}


/*******************************************************************************
* Function Name: Gyro_Wakeup
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
*  Gyro_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Gyro_Wakeup(void)
{
    Gyro_RestoreConfig();
    #if( (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) )
        Gyro_ClearRxBuffer();
    #endif /* End (Gyro_RX_ENABLED) || (Gyro_HD_ENABLED) */
    #if(Gyro_TX_ENABLED || Gyro_HD_ENABLED)
        Gyro_ClearTxBuffer();
    #endif /* End Gyro_TX_ENABLED || Gyro_HD_ENABLED */

    if(Gyro_backup.enableState != 0u)
    {
        Gyro_Enable();
    } 
}


/* [] END OF FILE */
