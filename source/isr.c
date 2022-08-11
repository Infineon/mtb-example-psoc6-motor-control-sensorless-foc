/*******************************************************************************
* File Name: isr.c
* Version  : V1.0
*
* Description:
*  This file provides the functions for system interrupts
* 
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


#include "cy_pdl.h"
#include "cycfg.h"
#include "define.h"
#include "hardware_config.h"
#include "isr.h"
#include "adc_sample.h"

uint32_t led_count_time = 0;


/*******************************************************************************
* Function Name:   PwmInterrupt
********************************************************************************
*
* Summary:
*  PWM main ISR, implement base FOC flow           
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void PwmInterrupt(void)
{
    uint32_t clear_isr;
    clear_isr = Cy_TCPWM_GetInterruptStatus(PWM_TRI_HW, PWM_TRI_NUM); /* read interrupt status register */
    MotorControlProcess();
    led_count_time++;
    Cy_TCPWM_ClearInterrupt(PWM_TRI_HW, PWM_TRI_NUM, clear_isr);/* clear handled interrupt */
}


/*******************************************************************************
* Function Name:   HardwareOvercurrentInterrupt
********************************************************************************
*
* Summary:
*  Interrupt for bus current over protection         
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void HardwareOvercurrentInterrupt(void)
{
    motor_control_run_par.u32_error_type = MOTOR_OVER_CURRENT;
    MotorControlStop();
    Cy_LPComp_ClearInterrupt(lpcomp_0_comp_0_HW, lpcomp_0_comp_0_IRQ);  /* clear handled interrupt */
}


/*******************************************************************************
* Function Name:   AdcSampleInterrupt
********************************************************************************
*
* Summary:
*  ADC ISR, sampling ADC value         
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void AdcSampleInterrupt(void)
{
    uint32 intr_status;
    
    /* Read interrupt status register */
    intr_status = Cy_SAR_GetInterruptStatus(SAR0_MOTOR_HW);
    
    /************************************************************************
    *  Custom Code
    *  - add user ISR code between the following #START and #END tags
    *************************************************************************/
    if ((intr_status & (uint32_t) CY_SAR_INTR_EOS_MASK) == (uint32_t) CY_SAR_INTR_EOS_MASK)
    {
        Cy_SAR_ClearInterrupt(SAR0_MOTOR_HW, CY_SAR_INTR_EOS_MASK);
        AdcReadSample();
        motor_control_run_par.i32q8_vbus = (adc_rslt_of_adc[SYS_ADC_CH_VDC] * adc_stc_sample.i32_vbusk) >> MATH_SHIFT_SIX;
        motor_control_run_par.i32q8_vr = (adc_rslt_of_adc[MOTOR_SPEED_VR]) >> MATH_SHIFT_FIVE;
        if(adc_motor_status != SENSOR_RUNNING)
        {    
             AdcMotorSensorOffsetDetect();
        }
        adc_stc_sample.u8_complete_flag = TRUE;
    }
}


/* [] END OF FILE */
