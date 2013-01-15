/*******************************************************************************
* File Name: ADC_1.c
* Version 1.71
*
* Description:
*  This file provides the source code to the API for the Successive
*  approximation ADC Component.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "CyLib.h"
#include "ADC_1.h"

#if(ADC_1_DEFAULT_INTERNAL_CLK)
    #include "ADC_1_theACLK.h"
#endif /* End ADC_1_DEFAULT_INTERNAL_CLK */


/***************************************
* Forward function references
***************************************/
void ADC_1_SetRef(int8 refMode);
void ADC_1_CalcGain(uint8 resolution);


/***************************************
* Global data allocation
***************************************/
uint8 ADC_1_initVar = 0u;
volatile int16 ADC_1_offset;
volatile int16 ADC_1_countsPerVolt;   /* Gain compensation */
volatile int16 ADC_1_shift;
#if(CY_PSOC5_ES1)
    uint8 ADC_1_resolution;
#endif /* End CY_PSOC5_ES1 */


/*******************************************************************************
* Function Name: ADC_1_Start
********************************************************************************
*
* Summary:
*  Performs all required initialization for this component and enables the
*  power.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The ADC_1_initVar variable is used to indicate when/if initial 
*  configuration of this component has happened. The variable is initialized to 
*  zero and set to 1 the first time ADC_Start() is called. This allows for 
*  component Re-Start without re-initialization in all subsequent calls to the 
*  ADC_1_Start() routine.
*  If re-initialization of the component is required the variable should be set 
*  to zero before call of ADC_1_Start() routine, or the user may call 
*  ADC_1_Init() and ADC_1_Enable() as done in the 
*  ADC_1_Start() routine.
*
*******************************************************************************/
void ADC_1_Start(void)
{

    /* If not Initialized then initialize all required hardware and software */
    if(ADC_1_initVar == 0u)
    {
        ADC_1_Init();
        ADC_1_initVar = 1u;
    }
    ADC_1_Enable();
}


/*******************************************************************************
* Function Name: ADC_1_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the 
*  customizer of the component placed onto schematic. Usually called in 
*  ADC_1_Start().
*
* Parameters:  
*  None.
*
* Return: 
*  None.
*
* Global variables:
*  The ADC_1_offset variable is initialized to 0.
*
*******************************************************************************/
void ADC_1_Init(void)
{

    /* This is only valid if there is an internal clock */
    #if(ADC_1_DEFAULT_INTERNAL_CLK)
        ADC_1_theACLK_SetMode(CYCLK_DUTY);
    #endif /* End ADC_1_DEFAULT_INTERNAL_CLK */

    /* Start and set interrupt vector */
    CyIntSetPriority(ADC_1_INTC_NUMBER, ADC_1_INTC_PRIOR_NUMBER);
    CyIntSetVector(ADC_1_INTC_NUMBER, ADC_1_ISR );

    /* Enable IRQ mode*/
    ADC_1_SAR_CSR1_REG |= ADC_1_SAR_IRQ_MASK_EN | ADC_1_SAR_IRQ_MODE_EDGE;
    
    /*Set SAR ADC resolution ADC */
    ADC_1_SetResolution(ADC_1_DEFAULT_RESOLUTION);
    ADC_1_offset = 0;
}


/*******************************************************************************
* Function Name: ADC_1_Enable
********************************************************************************
*  
* Summary: 
*  Enables the reference, clock and power for SAR ADC.
*
* Parameters:  
*  None.
*
* Return: 
*  None.
*
*******************************************************************************/
void ADC_1_Enable(void)
{
    uint8 tmpReg;
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

     /* Enable the SAR ADC block in Active Power Mode */
    ADC_1_PWRMGR_SAR_REG |= ADC_1_ACT_PWR_SAR_EN;

     /* Enable the SAR ADC in Standby Power Mode*/
    ADC_1_STBY_PWRMGR_SAR_REG |= ADC_1_STBY_PWR_SAR_EN;

    /* This is only valid if there is an internal clock */
    #if(ADC_1_DEFAULT_INTERNAL_CLK)
        ADC_1_PWRMGR_CLK_REG |= ADC_1_ACT_PWR_CLK_EN;
        ADC_1_STBY_PWRMGR_CLK_REG |= ADC_1_STBY_PWR_CLK_EN;
    #endif /* End ADC_1_DEFAULT_INTERNAL_CLK */

    /*Sets input range of the ADC*/
    #if(ADC_1_DEFAULT_RANGE == ADC_1__VSS_TO_VREF)
        ADC_1_SetRef(ADC_1_DEFAULT_RANGE);
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VSSA_TO_VDDA)
        ADC_1_SAR_CSR3_REG = ADC_1_SAR_EN_RESVDA_EN;     /* Set bit for VDD/2 mode */
        ADC_1_SetRef(ADC_1__VSSA_TO_VDDA);
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VSSA_TO_VDAC)
        ADC_1_SetRef(ADC_1_DEFAULT_RANGE);
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VREF_DIFF)
        ADC_1_SAR_CSR3_REG = ADC_1_SAR_EN_CP_EN;         /* Enable charge pump*/
        ADC_1_SetRef(ADC_1_DEFAULT_RANGE);
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VDDA_DIFF)
        ADC_1_SAR_CSR3_REG = ADC_1_SAR_EN_CP_EN;         /* Enable charge pump*/
        ADC_1_SetRef(ADC_1__VSSA_TO_VDDA);
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VDDA_2_DIFF)
        ADC_1_SAR_CSR3_REG = ADC_1_SAR_EN_RESVDA_EN |    /* Set bit for VDD/2 mode*/
                                        ADC_1_SAR_EN_CP_EN;         /* Enable charge pump*/
        ADC_1_SetRef(ADC_1__VSSA_TO_VDDA);
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VDAC_DIFF)
        ADC_1_SetRef(ADC_1_DEFAULT_RANGE);
    #endif /* end ADC_1_DEFAULT_RANGE */

    /* Set default power */
    tmpReg = (ADC_1_DEFAULT_POWER << ADC_1_SAR_POWER_SHIFT);
    /* Clear DAC value at beginning of sampling when internal reference used */
    #if(ADC_1_DEFAULT_REFERENCE != ADC_1__EXT_REF)
        tmpReg |= ADC_1_SAR_HIZ_CLEAR;
    #endif /* End ADC_1_DEFAULT_REFERENCE != ADC_1__EXT_REF */    
    /*Set Convertion mode */
    #if(ADC_1_DEFAULT_CONV_MODE == ADC_1__TRIGGERED)      /* If triggered mode */
        tmpReg |= ADC_1_SAR_MX_SOF_UDB |          /* source: UDB */
                  ADC_1_SAR_SOF_MODE_EDGE;        /* Set edge-sensitive sof source */
    #endif /* ADC_1_DEFAULT_CONV_MODE */
    ADC_1_SAR_CSR0_REG = tmpReg;

    /* Enable clock for SAR ADC*/
    ADC_1_SAR_CLK_REG |= ADC_1_SAR_MX_CLK_EN;
   
    #if(CY_PSOC5_ES1)
        /* Software Reset */
        ADC_1_SAR_CSR0_REG |= ADC_1_SAR_RESET_SOFT_ACTIVE;
        CyDelayUs(2); /* 2us delay is required for the lowest 1Mhz clock connected to SAR */
        ADC_1_SAR_CSR0_REG &= ~ADC_1_SAR_RESET_SOFT_ACTIVE;
    #endif /* End CY_PSOC5_ES1 */

    /* Clear a pending interrupt */
    CyIntClearPending(ADC_1_INTC_NUMBER);
    
    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: ADC_1_SetRef
********************************************************************************
*
* Summary:
*   Sets reference for ADC
*
* Parameters:
*  refMode: Reference configuration.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_1_SetRef(int8 refMode)
{
    uint8 tmpReg;
    if(refMode == ADC_1__VSSA_TO_VDDA)
    {
        #if(ADC_1_DEFAULT_REFERENCE == ADC_1__INT_REF_NOT_BYPASSED)
            tmpReg = ADC_1_SAR_REF_S3_LSB_EN | ADC_1_SAR_REF_S4_LSB_EN;
            ADC_1_SAR_CSR3_REG |= ADC_1_SAR_EN_BUF_VREF_EN; /* Enable Int Ref Amp*/
        #elif(ADC_1_DEFAULT_REFERENCE == ADC_1__INT_REF_BYPASS)
            tmpReg = ADC_1_SAR_REF_S2_LSB_EN | ADC_1_SAR_REF_S3_LSB_EN | 
                     ADC_1_SAR_REF_S4_LSB_EN;
            ADC_1_SAR_CSR3_REG |= ADC_1_SAR_EN_BUF_VREF_EN; /* Enable Int Ref Amp*/
        #elif(ADC_1_DEFAULT_REFERENCE == ADC_1__EXT_REF)
            tmpReg = ADC_1_SAR_REF_S2_LSB_EN;
            ADC_1_SAR_CSR3_REG &= ~ADC_1_SAR_EN_BUF_VREF_EN; /* Disable Int Ref Amp*/
        #endif /*  End ADC_1_DEFAULT_REFERENCE */
    }
    else
    {
        #if(ADC_1_DEFAULT_REFERENCE == ADC_1__INT_REF_NOT_BYPASSED)
            tmpReg = ADC_1_SAR_REF_S3_LSB_EN | ADC_1_SAR_REF_S4_LSB_EN;
            ADC_1_SAR_CSR3_REG |= ADC_1_SAR_EN_BUF_VREF_EN; /* Enable Int Ref Amp*/
        #elif(ADC_1_DEFAULT_REFERENCE == ADC_1__INT_REF_BYPASS)
            tmpReg = ADC_1_SAR_REF_S2_LSB_EN | ADC_1_SAR_REF_S3_LSB_EN | 
                     ADC_1_SAR_REF_S4_LSB_EN;
            ADC_1_SAR_CSR3_REG |= ADC_1_SAR_EN_BUF_VREF_EN; /* Enable Int Ref Amp*/
        #elif(ADC_1_DEFAULT_REFERENCE == ADC_1__EXT_REF)
            tmpReg = ADC_1_SAR_REF_S2_LSB_EN;
            ADC_1_SAR_CSR3_REG &= ~ADC_1_SAR_EN_BUF_VREF_EN; /* Disable Int Ref Amp*/
        #endif /*  End ADC_1_DEFAULT_REFERENCE */
    }
    ADC_1_SAR_CSR6_REG = tmpReg;
    ADC_1_SAR_CSR7_REG = ADC_1_SAR_REF_S_MSB_DIS;
}


/*******************************************************************************
* Function Name: ADC_1_Stop
********************************************************************************
*
* Summary:
*  Stops convertion and reduce the power to the minimum.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_1_Stop(void)
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Stop all conversions */
    ADC_1_SAR_CSR0_REG &= ~ADC_1_SAR_SOF_START_CONV;
    
    #if(CY_PSOC5_ES1)
        /* Leave the SAR block powered and reduce the power to the minimum */
        ADC_1_SAR_CSR0_REG |= ADC_1_ICONT_LV_3;
        /* Disable reference buffer and reduce the reference power to the minimum */
        ADC_1_SAR_CSR3_REG = ADC_1_SAR_PWR_CTRL_VREF_DIV_BY4;    
    #else    
        /* Disable the SAR ADC block in Active Power Mode */
        ADC_1_PWRMGR_SAR_REG &= ~ADC_1_ACT_PWR_SAR_EN;
        /* Disable the SAR ADC in Standby Power Mode */
        ADC_1_STBY_PWRMGR_SAR_REG &= ~ADC_1_STBY_PWR_SAR_EN;
        /* Disable power for reference buffer and charge pump*/
        ADC_1_SAR_CSR3_REG = ADC_1_SAR_EN_BUF_VREF_DIS;    
    #endif /* End CY_PSOC5_ES1 */
    

    /* This is only valid if there is an internal clock */
    #if(ADC_1_DEFAULT_INTERNAL_CLK)
        ADC_1_PWRMGR_CLK_REG &= ~ADC_1_ACT_PWR_CLK_EN;
        ADC_1_STBY_PWRMGR_CLK_REG &= ~ADC_1_STBY_PWR_CLK_EN;
    #endif /* End ADC_1_DEFAULT_INTERNAL_CLK */

    CyExitCriticalSection(enableInterrupts);
    
}


/*******************************************************************************
* Function Name: ADC_1_IRQ_Enable
********************************************************************************
* Summary:
*  Enables the interrupt.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_1_IRQ_Enable(void)
{
    /* Enable the general interrupt. */
    CyIntEnable(ADC_1_INTC_NUMBER);
}


/*******************************************************************************
* Function Name: ADC_1_IRQ_Disable
********************************************************************************
*
* Summary:
*  Disables the Interrupt.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_1_IRQ_Disable(void)
{
    /* Disable the general interrupt. */
    CyIntDisable(ADC_1_INTC_NUMBER);
}


/*******************************************************************************
* Function Name: ADC_1_SetPower
********************************************************************************
*
* Summary:
*  Sets the Power mode.
*
* Parameters:
*  power:  Power setting for ADC
*  0 ->    Normal
*  1 ->    Half power
*  2 ->    1/3rd power
*  3 ->    Quarter power.
*
* Return:
*  None.
*
*******************************************************************************/
void ADC_1_SetPower(uint8 power)
{
    uint8 tmpReg;

    /* mask off invalid power settings */
    power &= ADC_1_SAR_API_POWER_MASK;

    /* Set Power parameter  */
    tmpReg = ADC_1_SAR_CSR0_REG & ~ADC_1_SAR_POWER_MASK;
    tmpReg |= (power << ADC_1_SAR_POWER_SHIFT);
    ADC_1_SAR_CSR0_REG = tmpReg;
}


/*******************************************************************************
* Function Name: ADC_1_SetResolution
********************************************************************************
*
* Summary:
*  Sets the Relution of the SAR.
*  This function does not affect the actual conversion with PSoC5 ES1 silicon.
*
* Parameters:
*  resolution:
*  12 ->    RES12
*  10 ->    RES10
*  8  ->    RES8
*
* Return:
*  None.
*
* Side Effects:
*  The ADC resolution cannot be changed during a conversion cycle. The recommended
*  best practice is to stop conversions with ADC_StopConvert(), change the 
*  resolution, then restart the conversions with ADC_StartConvert().
*  If you decide not to stop conversions before calling this API, you should use 
*  ADC_IsEndConversion() to wait until conversion is complete before changing the 
*  resolution.
*  If you call ADC_SetResolution() during a conversion, the resolution will not 
*  be changed until the current conversion is complete. Data will not be available 
*  in the new resolution for another 6 + “New Resolution(in bits)” clock cycles. 
*  You may need add a delay of this number of clock cycles after ADC_SetResolution()
*  is called before data is valid again.
*  Affects ADC_CountsTo_Volts(), ADC_CountsTo_mVolts(), and ADC_CountsTo_uVolts() 
*  by calculating the correct conversion between ADC counts and the applied input 
*  voltage. Calculation depends on resolution, input range, and voltage reference.
*
*******************************************************************************/
void ADC_1_SetResolution(uint8 resolution)
{
    uint8 tmpReg;

    /* remember resolution for the GetResult APIs */
    #if(CY_PSOC5_ES1)
        ADC_1_resolution = resolution;
    #endif /* End CY_PSOC5_ES1 */
    
    /* Set SAR ADC resolution */
    switch (resolution)
    {
        case ADC_1__BITS_12:
            tmpReg = ADC_1_SAR_RESOLUTION_12BIT;
            break;
        case ADC_1__BITS_10:
            /* Use 12bits for PSoC5 production silicon and shift the 
            *  results for lower resolution in GetResult16() API 
            */
            #if(CY_PSOC5_ES1)
                tmpReg = ADC_1_SAR_RESOLUTION_12BIT;
            #else    
                tmpReg = ADC_1_SAR_RESOLUTION_10BIT;
            #endif /* End CY_PSOC5_ES1 */
            break;
        case ADC_1__BITS_8:
            #if(CY_PSOC5_ES1)
                tmpReg = ADC_1_SAR_RESOLUTION_12BIT;
            #else    
                tmpReg = ADC_1_SAR_RESOLUTION_8BIT;
            #endif /* End CY_PSOC5_ES1 */
            break;
        default:
            tmpReg = ADC_1_SAR_RESOLUTION_12BIT;
            break;
    }
    
     tmpReg |= ADC_1_SAR_SAMPLE_WIDTH;   /* 18 conversion cycles @ 12bits + 1 gap*/
    ADC_1_SAR_CSR2_REG = tmpReg;
    
     /* Calculate gain for convert counts to volts */
    ADC_1_CalcGain(resolution);
}


/*******************************************************************************
* Function Name: ADC_1_StartConvert
********************************************************************************
*
* Summary:
*  Starts ADC conversion using the given mode.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Forces the ADC to initiate a conversion. In Free Running mode, the ADC will 
*  run continuously. In a triggered mode the function also acts as a software
*  version of the SOC. Here every conversion has to be triggered by the routine. 
*  This writes into the SOC bit in SAR_CTRL reg.
*
* Side Effects:
*  In a triggered mode the function switches source for SOF from the external 
*  pin to the internal SOF generation. Application should not call StartConvert
*  if external source used for SOF.
*******************************************************************************/
void ADC_1_StartConvert(void)
{
    #if(ADC_1_DEFAULT_CONV_MODE == ADC_1__TRIGGERED)   /* If triggered mode */
        ADC_1_SAR_CSR0_REG &= ~ADC_1_SAR_MX_SOF_UDB;   /* source: SOF bit */
    #endif /* End ADC_1_DEFAULT_CONV_MODE */
    
    /* Start the conversion */
    ADC_1_SAR_CSR0_REG |= ADC_1_SAR_SOF_START_CONV;
}


/*******************************************************************************
* Function Name: ADC_1_StopConvert
********************************************************************************
*
* Summary:
*  Stops ADC conversion using the given mode.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Stops ADC conversion in Free Running mode. 
*  This writes into the SOC bit in SAR_CTRL reg.
*
* Side Effects:
*  In a triggered mode the function set a software version of the SOC to low level
*  and switch SOF source to hardware SOF input.
*  
*******************************************************************************/
void ADC_1_StopConvert(void)
{
    /* Stop all conversions */
    ADC_1_SAR_CSR0_REG &= ~ADC_1_SAR_SOF_START_CONV;

    #if(ADC_1_DEFAULT_CONV_MODE == ADC_1__TRIGGERED)   /* If triggered mode */
        /* Return source to UDB for hardware SOF signal */
        ADC_1_SAR_CSR0_REG |= ADC_1_SAR_MX_SOF_UDB;    /* source: UDB */
    #endif /* End ADC_1_DEFAULT_CONV_MODE */
    
}


/*******************************************************************************
* Function Name: ADC_1_IsEndConversion
********************************************************************************
*
* Summary:
*  Queries the ADC to see if conversion is complete
*
* Parameters:
*  retMode:  Wait mode,
*   0 if return with answer imediately.
*   1 if wait until conversion is complete, then return.
*
* Return:
*  (uint8)  0 =>  Conversion not complete.
*           1 =>  Conversion complete.
*
*******************************************************************************/
uint8 ADC_1_IsEndConversion(uint8 retMode)
{
    uint8 status;

    do
    {
        status = ADC_1_SAR_CSR1_REG & ADC_1_SAR_EOF_1;
    } while ((status != ADC_1_SAR_EOF_1) && (retMode == ADC_1_WAIT_FOR_RESULT));
    /* If convertion complete, wait until EOF bit released */
    if(status == ADC_1_SAR_EOF_1)
    {
        /* wait one ADC clock to let the EOC status bit release */
        CyDelayUs(1);
        /* Do the unconditional read operation of the CSR1 register to make sure the EOC bit has been cleared */
        CY_GET_REG8(ADC_1_SAR_CSR1_PTR);
    }    

    return(status);
}


/*******************************************************************************
* Function Name: ADC_1_GetResult8
********************************************************************************
*
* Summary:
*  Returns an 8-bit result or the LSB of the last conversion. 
*  ADC_1_IsEndConversion() should be called to verify that the data 
*   sample is ready 
*
* Parameters:
*  None.
*
* Return:
*  ADC result.
*
* Global Variables:
*  ADC_1_shift - used to convert the ADC counts to the 2's 
*  compliment form.
*  ADC_1_resolution – used to shift the results for lower 
*   resolution.
*
*******************************************************************************/
int8 ADC_1_GetResult8( void )
{
    
    #if(CY_PSOC5_ES1)

        int16 res;

        res = CY_GET_REG16(ADC_1_SAR_WRK0_PTR) - ADC_1_shift;
        
        /* Use 12bits for PSoC5 production silicon and shift the results for lower resolution in this API */
        if(ADC_1_resolution == ADC_1__BITS_10)
        {
            res >>= 2u;
        }
        else if(ADC_1_resolution == ADC_1__BITS_8)
        {
            res >>= 4u;
        }
        else    /* Do not shift for 12 bits */
        {
        }
        return( (int8)res );
        
    #else
        return( ADC_1_SAR_WRK0_REG - (int8)ADC_1_shift);
    #endif /* End CY_PSOC5_ES1 */

}


/*******************************************************************************
* Function Name: ADC_1_GetResult16
********************************************************************************
*
* Summary:
*  Gets the data available in the SAR DATA registers.
*  ADC_1_IsEndConversion() should be called to verify that the data 
*   sample is ready 
*
* Parameters:
*  None.
*
* Return:
*  ADC result. WORD value which has the converted 12bits. In the differential 
*  input mode the SAR ADC outputs digitally converted data in binary offset 
*  scheme, this function converts the data into 2's compliment form. 
*
* Global Variables:
*  ADC_1_shift - used to convert the ADC counts to the 2's 
*  compliment form.
*  ADC_1_resolution – used to shift the results for lower 
*   resolution.
*
*******************************************************************************/
int16 ADC_1_GetResult16( void )
{
    int16 res;
    
    res = CY_GET_REG16(ADC_1_SAR_WRK0_PTR) - ADC_1_shift;

    #if(CY_PSOC5_ES1)
        /* Use 12bits for PSoC5 production silicon and shift the results for lower resolution in this API */
        if(ADC_1_resolution == ADC_1__BITS_10)
        {
            res >>= 2u;
        }
        else if(ADC_1_resolution == ADC_1__BITS_8)
        {
            res >>= 4u;
        }
        else    /* Do not shift for 12 bits */
        {
        }
    #endif /* End CY_PSOC5_ES1 */

    return( res );
}


/*******************************************************************************
* Function Name: ADC_1_SetOffset
********************************************************************************
*
* Summary:
*  This function sets the offset for voltage readings.
*
* Parameters:
*  int16: Offset in counts
*
* Return:
*  None.
*
* Global Variables:
*  The ADC_1_offset variable modified. This variable is used for 
*  offset calibration purpose. 
*  Affects the ADC_1_CountsTo_Volts, 
*  ADC_1_CountsTo_mVolts, ADC_1_CountsTo_uVolts functions 
*  by subtracting the given offset. 
*
*******************************************************************************/
void ADC_1_SetOffset(int16 offset)
{
    ADC_1_offset = offset;
}


/*******************************************************************************
* Function Name: ADC_1_CalcGain
********************************************************************************
*
* Summary:
*  This function calculates the ADC gain in counts per volt.
*
* Parameters:
*  uint8: resolution
*
* Return:
*  None.
*
* Global Variables:
*  ADC_1_shift variable initialized. This variable is used to 
*  convert the ADC counts to the 2's compliment form. 
*  ADC_1_countsPerVolt variable initialized. This variable is used 
*  for gain calibration purpose. 
*
*******************************************************************************/
void ADC_1_CalcGain( uint8 resolution )
{
    uint32 counts = ADC_1_SAR_WRK_MAX;       /*default 12 bits*/
    uint16 diff_zero = ADC_1_SAR_DIFF_SHIFT;
    
    if(resolution == ADC_1__BITS_10)
    {
        counts >>= 2u;
        /* Use 12bits for PSoC5 production silicon */
        #if(CY_PSOC5_ES2)
            diff_zero >>= 2u;
        #else  /* To avoid the warning */  
            diff_zero = diff_zero;
        #endif /* End CY_PSOC5_ES2 */
    }
    if(resolution == ADC_1__BITS_8)
    {
        counts >>= 4u;
        /* Use 12bits for PSoC5 production silicon */
        #if(CY_PSOC5_ES2)
            diff_zero >>= 4u;
        #else  /* To avoid the warning */  
            diff_zero = diff_zero;
        #endif /* End CY_PSOC5_ES2 */
    }
    counts *= 1000u; /* To avoid float point arithmetic*/
    ADC_1_countsPerVolt = counts / ADC_1_DEFAULT_REF_VOLTAGE_MV / 2;

    #if(ADC_1_DEFAULT_RANGE == ADC_1__VSS_TO_VREF)
        ADC_1_shift = 0;
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VSSA_TO_VDDA)
        ADC_1_shift = 0;
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VSSA_TO_VDAC)
        ADC_1_shift = 0;
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VREF_DIFF)
        ADC_1_shift = diff_zero;
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VDDA_DIFF)
        ADC_1_shift = diff_zero;
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VDDA_2_DIFF)
        ADC_1_shift = diff_zero;
    #elif(ADC_1_DEFAULT_RANGE == ADC_1__VNEG_VDAC_DIFF)
        ADC_1_shift = diff_zero;
    #endif /* End ADC_1_DEFAULT_RANGE */    
}


/*******************************************************************************
* Function Name: ADC_1_SetGain
********************************************************************************
*
* Summary:
*  This function sets the ADC gain in counts per volt.
*
* Parameters:
*  int16  adcGain  counts per volt
*
* Return:
*  None.
*
* Global Variables:
*  ADC_1_countsPerVolt variable modified. This variable is used 
*  for gain calibration purpose. 
*
*******************************************************************************/
void ADC_1_SetGain(int16 adcGain)
{
    ADC_1_countsPerVolt = adcGain;
}


/*******************************************************************************
* Function Name: ADC_1_CountsTo_mVolts
********************************************************************************
*
* Summary:
*  This function converts ADC counts to mVolts
*
* Parameters:
*  int16  adcCounts   Reading from ADC.
*
* Return:
*  int16  Result in mVolts
*
* Global Variables:
*  ADC_1_offset variable used.
*  ADC_1_countsPerVolt variable used.
*
*******************************************************************************/
int16 ADC_1_CountsTo_mVolts(int16 adcCounts)
{

    int16 mVolts;

    /* Subtract ADC offset */
    adcCounts -= ADC_1_offset;

    mVolts = ( (int32)adcCounts * 1000 ) / ADC_1_countsPerVolt ;

    return( mVolts );
}


/*******************************************************************************
* Function Name: ADC_1_CountsTo_uVolts
********************************************************************************
*
* Summary:
*  This function converts ADC counts to micro Volts
*
* Parameters:
*  int16  adcCounts   Reading from ADC.
*
* Return:
*  int32  Result in micro Volts
*
* Global Variables:
*  ADC_1_offset variable used.
*  ADC_1_countsPerVolt variable used.
*
*******************************************************************************/
int32 ADC_1_CountsTo_uVolts(int16 adcCounts)
{

    int32 uVolts;

    /* Subtract ADC offset */
    adcCounts -= ADC_1_offset;
    /* To convert adcCounts to microVolts it is required to be multiplied
    *  on 1 million. It is multiplied on 500000 and later on 2 to 
    *  to avoid 32bit arithmetic overflows. 
    */
    uVolts = (( (int32)adcCounts * 500000 ) / ADC_1_countsPerVolt) * 2;

    return( uVolts );
}


/*******************************************************************************
* Function Name: ADC_1_CountsTo_Volts
********************************************************************************
*
* Summary:
*  This function converts ADC counts to Volts
*
* Parameters:
*  int16  adcCounts   Reading from ADC.
*
* Return:
*  float  Result in mVolts
*
* Global Variables:
*  ADC_1_offset variable used.
*  ADC_1_countsPerVolt variable used.
*
*******************************************************************************/
float ADC_1_CountsTo_Volts(int16 adcCounts)
{
    float volts;

    /* Subtract ADC offset */
    adcCounts -= ADC_1_offset;

    volts = (float)adcCounts / (float)ADC_1_countsPerVolt;   

    return( volts );
}


/* [] END OF FILE */
