/*******************************************************************************
* File Name: adc_sample.c
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the motor phase current 
*  sense functions.
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
#include "adc_sample.h"
#include "hardware_config.h"
#include "define.h"
#include "chip_init.h"
#include "motor_ctrl.h"
#include "svpwm.h"
#include "coordinate_transform.h"


/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
stc_sensor_t            adc_stc_sample;
volatile uint8_t        adc_motor_status = 0;
uint16_t                adc_rslt_of_adc[ADC_CH_AMOUNT];

static  int32_t                  adc_motor_iuvw_offset_max;
static  int32_t                  adc_motor_iuvw_offset_min;
static  uint32_t                 adc_motor_offset_check_delay = 0;
static  stc_muti_shunt_offest_t  adc_stc_motor_offset;


/*******************************************************************************
* Function Name:   AdcStart
********************************************************************************
*
* Summary:
*  Init SAR ADC Module             
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void AdcStart(void)
{
    /* start SARADC  */
    Cy_SAR_Init(SAR0_MOTOR_HW, &SAR0_MOTOR_config);
    Cy_SAR_Enable(SAR0_MOTOR_HW);
    Cy_SAR_SetConvertMode(SAR0_MOTOR_HW, CY_SAR_TRIGGER_MODE_FW_AND_HWEDGE);
}


/*******************************************************************************
* Function Name:   AdcReadSample
********************************************************************************
*
* Summary:
*  Read ADC Sample Values and Put into adc_rslt_of_adc              
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void AdcReadSample(void)
{
    uint8 adc_channel_count  = 0;
    for(adc_channel_count = 0; adc_channel_count  < ADC_CH_AMOUNT; adc_channel_count++)
    {
        adc_rslt_of_adc[adc_channel_count] = Cy_SAR_GetResult16(SAR0_MOTOR_HW, adc_channel_count);
    }
}


/*******************************************************************************
* Function Name:   AdcInitSensorPar
********************************************************************************
*
* Summary:
*  ADC parameters initial
*
* Parameters: 
*  u16_sample_freq
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void AdcInitSensorPar(uint16_t u16_sample_freq)
{
    adc_stc_motor_offset.offset_sample_num = 0;
    adc_stc_motor_offset.i32_xu = 0;
    adc_stc_motor_offset.i32_xv = 0;
    adc_stc_motor_offset.i32_xw = 0;
    adc_motor_iuvw_offset_max = i32_motor_iuvw_offset_normal
                            + i32_motor_iuvw_offset_range;
    adc_motor_iuvw_offset_min = i32_motor_iuvw_offset_normal
                            - i32_motor_iuvw_offset_range;
    adc_stc_sample.i32q14_motor_current_factor
            = Q14(ADC_VOLT_REF / (MOTOR_IUVW_SAMPLE_RESISTOR
            * MOTOR_IUVW_AMPLIFIER_FACTOR * ADC_VALUE_MAX));
    adc_motor_offset_check_delay = (signed long) (u16_sample_freq * MOTOR_OFFSET_CHECK_DELAY);
    adc_stc_sample.u8_complete_flag = 0;
    adc_stc_sample.i32_vbusk    = Q14(ADC_VOLT_REF * SYS_VDC_FACTOR / ADC_VALUE_MAX);
}


/*******************************************************************************
* Function Name:   AdcMotorSensorOffsetDetect
********************************************************************************
*
* Summary:
*  Motor sensor offset detect
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void AdcMotorSensorOffsetDetect(void)
{
    if(adc_motor_offset_check_delay < 1)
    {     
        adc_stc_motor_offset.i32_xu += adc_rslt_of_adc[COMP_ADC_CH_IU];
        adc_stc_motor_offset.i32_xw += adc_rslt_of_adc[COMP_ADC_CH_IW];
        adc_stc_motor_offset.offset_sample_num++;
        if(adc_stc_motor_offset.offset_sample_num >= i32_motor_iuvw_offset_check_times)
        {
            adc_stc_motor_offset.i32_xu >>= RIGHT_SHIFT_NUM;
            adc_stc_motor_offset.i32_xw >>= RIGHT_SHIFT_NUM;
            adc_stc_motor_offset.offset_sample_num = 0;
            if(adc_stc_motor_offset.i32_xu < adc_motor_iuvw_offset_min || adc_stc_motor_offset.i32_xu > adc_motor_iuvw_offset_max)
            {
                motor_control_run_par.u32_error_type |= AD_MIDDLE_ERROR;
            }
            else if(adc_stc_motor_offset.i32_xw < adc_motor_iuvw_offset_min || adc_stc_motor_offset.i32_xw > adc_motor_iuvw_offset_max)
            {
                motor_control_run_par.u32_error_type |= AD_MIDDLE_ERROR;
            }
            else
            {
                adc_motor_status = SENSOR_RUNNING;
            }
        }
    }
    else
    {
        adc_motor_offset_check_delay--;
    }
}


/*******************************************************************************
* Function Name:   AdcMotorCurrentSense
********************************************************************************
*
* Summary:
*  Motor phase current calcultate         
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void AdcMotorCurrentSense(void)
{
    if(CLOCK_WISE == u8_motor_running_direction)
    {
        motor_stc_iuvw_sensed.i32q8_xu
            = ((adc_stc_motor_offset.i32_xu - adc_rslt_of_adc[COMP_ADC_CH_IU])
            * adc_stc_sample.i32q14_motor_current_factor) >> RIGHT_SHIFT_NUM;
        motor_stc_iuvw_sensed.i32q8_xv
            = -(motor_stc_iuvw_sensed.i32q8_xu
                + motor_stc_iuvw_sensed.i32q8_xw);
        motor_stc_iuvw_sensed.i32q8_xw
             = ((adc_stc_motor_offset.i32_xw - adc_rslt_of_adc[COMP_ADC_CH_IW])
            * adc_stc_sample.i32q14_motor_current_factor) >> RIGHT_SHIFT_NUM;
    }
    if(COUNT_LOCK_WISE == u8_motor_running_direction)
    {
        motor_stc_iuvw_sensed.i32q8_xv
            = ((adc_stc_motor_offset.i32_xu - adc_rslt_of_adc[COMP_ADC_CH_IU])
            * adc_stc_sample.i32q14_motor_current_factor) >> RIGHT_SHIFT_NUM;
        motor_stc_iuvw_sensed.i32q8_xu
            = -(motor_stc_iuvw_sensed.i32q8_xv
                    + motor_stc_iuvw_sensed.i32q8_xw);
        motor_stc_iuvw_sensed.i32q8_xw
            = ((adc_stc_motor_offset.i32_xw - adc_rslt_of_adc[COMP_ADC_CH_IW])
            * adc_stc_sample.i32q14_motor_current_factor) >> RIGHT_SHIFT_NUM;
    }        
}

