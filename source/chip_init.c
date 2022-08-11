/*******************************************************************************
* File Name: chip_init.c
* Version  : V1.0
*
* Description:
*  This file provides the functions for PWM initials
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


#include <stdlib.h>
#include "cy_pdl.h"
#include "cycfg.h"
#include "define.h"
#include "motor_ctrl.h"
#include "svpwm.h"
#include "customer_interface.h"
#include "hardware_config.h"
#include "chip_init.h"

uint16_t pwm_peak_value;


/*******************************************************************************
* Function Name:   EnableSwitch
********************************************************************************
*
* Summary:
*  Software trigger enable switch        
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void EnableSwitch(void)
{
    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_A_HW, PWM_A_NUM);
    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_B_HW, PWM_B_NUM);
    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_C_HW, PWM_C_NUM);
}


/*******************************************************************************
* Function Name:   SetNormalGpioMode
********************************************************************************
*
* Summary:
*  Set three phase pin to normal GPIO mode     
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void SetNormalGpioMode(void)
{
    /**Change the PWM Pins work mode**/
    Cy_GPIO_SetHSIOM(PWM_AH_PORT, PWM_AH_NUM, (en_hsiom_sel_t)HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWM_AL_PORT, PWM_AL_NUM, (en_hsiom_sel_t)HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWM_BH_PORT, PWM_BH_NUM, (en_hsiom_sel_t)HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWM_BL_PORT, PWM_BL_NUM, (en_hsiom_sel_t)HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWM_CH_PORT, PWM_CH_NUM, (en_hsiom_sel_t)HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWM_CL_PORT, PWM_CL_NUM, (en_hsiom_sel_t)HSIOM_SEL_GPIO);
}


/*******************************************************************************
* Function Name:   SetPwmGpioMode
********************************************************************************
*
* Summary:
*  Set three phase pin to PWM mode   
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void SetPwmGpioMode(void)
{    
    if((PWM_AH_HSIOM == HSIOM_SEL_ACT_0) && (PWM_AL_HSIOM == HSIOM_SEL_ACT_0) && (PWM_BH_HSIOM == HSIOM_SEL_ACT_0) && (PWM_BL_HSIOM == HSIOM_SEL_ACT_0) && (PWM_CH_HSIOM == HSIOM_SEL_ACT_0)&&(PWM_CL_HSIOM == HSIOM_SEL_ACT_0))
    {
        Cy_GPIO_SetHSIOM(PWM_AH_PORT, PWM_AH_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_0);
        Cy_GPIO_SetHSIOM(PWM_AL_PORT, PWM_AL_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_0);
        Cy_GPIO_SetHSIOM(PWM_BH_PORT, PWM_BH_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_0);
        Cy_GPIO_SetHSIOM(PWM_BL_PORT, PWM_BL_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_0);
        Cy_GPIO_SetHSIOM(PWM_CH_PORT, PWM_CH_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_0);
        Cy_GPIO_SetHSIOM(PWM_CL_PORT, PWM_CL_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_0);
    }
    else if((PWM_AH_HSIOM == HSIOM_SEL_ACT_1) && (PWM_AL_HSIOM == HSIOM_SEL_ACT_1) && (PWM_BH_HSIOM == HSIOM_SEL_ACT_1) && (PWM_BL_HSIOM == HSIOM_SEL_ACT_1) && (PWM_CH_HSIOM == HSIOM_SEL_ACT_1) && (PWM_CL_HSIOM == HSIOM_SEL_ACT_1))
    {
        Cy_GPIO_SetHSIOM(PWM_AH_PORT, PWM_AH_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_1);
        Cy_GPIO_SetHSIOM(PWM_AL_PORT, PWM_AL_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_1);
        Cy_GPIO_SetHSIOM(PWM_BH_PORT, PWM_BH_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_1);
        Cy_GPIO_SetHSIOM(PWM_BL_PORT, PWM_BL_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_1);
        Cy_GPIO_SetHSIOM(PWM_CH_PORT, PWM_CH_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_1);
        Cy_GPIO_SetHSIOM(PWM_CL_PORT, PWM_CL_NUM, (en_hsiom_sel_t)HSIOM_SEL_ACT_1);
    }
    else
    {

    }
}


/*******************************************************************************
* Function Name:   SetToHighLevel
********************************************************************************
*
* Summary:
*  Set three phase pin output high level   
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void SetToHighLevel(void)
{    
    Cy_GPIO_Write(PWM_AH_PORT, PWM_AH_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_AL_PORT, PWM_AL_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_BH_PORT, PWM_BH_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_BL_PORT, PWM_BL_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_CH_PORT, PWM_CH_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_CL_PORT, PWM_CL_NUM, PWM_OUT_HIGH_LEVEL);
}


/*******************************************************************************
* Function Name:   SetToLowLevel
********************************************************************************
*
* Summary:
*  Set three phase pin output low level   
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void SetToLowLevel(void)
{
    Cy_GPIO_Write(PWM_AH_PORT, PWM_AH_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_AL_PORT, PWM_AL_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_BH_PORT, PWM_BH_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_BL_PORT, PWM_BL_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_CH_PORT, PWM_CH_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_CL_PORT, PWM_CL_NUM, PWM_OUT_LOW_LEVEL);
}


/*******************************************************************************
* Function Name:   PwmEnable
********************************************************************************
*
* Summary:
*  Enable PWM output
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void PwmEnable(void)
{
    Cy_TCPWM_PWM_SetCompare0Val(PWM_A_HW, PWM_A_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_B_HW, PWM_B_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_C_HW, PWM_C_NUM, motor_stc_svm_gen.i16_cycle);

    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_A_HW, PWM_A_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_B_HW, PWM_B_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_C_HW, PWM_C_NUM, motor_stc_svm_gen.i16_cycle);

    SetPwmGpioMode();
}


/*******************************************************************************
* Function Name:   PwmDisable
********************************************************************************
*
* Summary:
*  Disable PWM output
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void PwmDisable(void)
{
    SetToLowLevel();
    SetNormalGpioMode();
}


/*******************************************************************************
* Function Name:   PwmStop
********************************************************************************
*
* Summary:
*  Disable PWM output
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void PwmStop(void)
{
    SetToLowLevel();
    SetNormalGpioMode();

    Cy_TCPWM_PWM_SetCompare0Val(PWM_A_HW, PWM_A_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_B_HW, PWM_B_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_C_HW, PWM_C_NUM, motor_stc_svm_gen.i16_cycle);

    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_A_HW, PWM_A_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_B_HW, PWM_B_NUM, motor_stc_svm_gen.i16_cycle);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_C_HW, PWM_C_NUM, motor_stc_svm_gen.i16_cycle);
}


/*******************************************************************************
* Function Name:   PwmBrake
********************************************************************************
*
* Summary:
*  Set PWM to brake mode
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void PwmBrake(void)
{
    Cy_GPIO_Write(PWM_AH_PORT, PWM_AH_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_AL_PORT, PWM_AL_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_BH_PORT, PWM_BH_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_BL_PORT, PWM_BL_NUM, PWM_OUT_HIGH_LEVEL);
    Cy_GPIO_Write(PWM_CH_PORT, PWM_CH_NUM, PWM_OUT_LOW_LEVEL);
    Cy_GPIO_Write(PWM_CL_PORT, PWM_CL_NUM, PWM_OUT_HIGH_LEVEL);
    SetNormalGpioMode();   
}


/*******************************************************************************
* Function Name:   PwmStart
********************************************************************************
*
* Summary:
*  Start PWM output
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void PwmStart(void)
{
    /* start 3 phases PWM  */
    /*GPIO set to low*/
    SetToLowLevel();

    SetNormalGpioMode();

    Cy_TCPWM_PWM_Init(PWM_TRI_HW, PWM_TRI_NUM, &PWM_TRI_config);

    Cy_TCPWM_PWM_Init(PWM_A_HW, PWM_A_NUM, &PWM_A_config);
    
    Cy_TCPWM_PWM_Init(PWM_B_HW, PWM_B_NUM, &PWM_B_config);

    Cy_TCPWM_PWM_Init(PWM_C_HW, PWM_C_NUM, &PWM_C_config);

    /* clear counter value after start PWM    */
    Cy_TCPWM_PWM_SetCounter(PWM_TRI_HW, PWM_TRI_NUM, CONSTANT_ZERO);
    Cy_TCPWM_PWM_SetCounter(PWM_A_HW, PWM_A_NUM, CONSTANT_ZERO);
    Cy_TCPWM_PWM_SetCounter(PWM_B_HW, PWM_B_NUM, CONSTANT_ZERO);
    Cy_TCPWM_PWM_SetCounter(PWM_C_HW, PWM_C_NUM, CONSTANT_ZERO);


    MotorCtrlConfigPwm();
    
    /* disable GPIO output as default, enable later based on firmware */
    CyDelay(DELAY_1MS);
    SetPwmGpioMode();

    Cy_TCPWM_PWM_Enable(PWM_TRI_HW, PWM_TRI_NUM);
    Cy_TCPWM_PWM_Enable(PWM_A_HW, PWM_A_NUM);
    Cy_TCPWM_PWM_Enable(PWM_B_HW, PWM_B_NUM);
    Cy_TCPWM_PWM_Enable(PWM_C_HW, PWM_C_NUM);
    
    Cy_TCPWM_InputTriggerSetup(PWM_B_HW, PWM_B_NUM, CY_TCPWM_INPUT_TR_START, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_1);
    Cy_TCPWM_InputTriggerSetup(PWM_C_HW, PWM_C_NUM, CY_TCPWM_INPUT_TR_START, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_1);
    Cy_TCPWM_InputTriggerSetup(PWM_A_HW, PWM_A_NUM, CY_TCPWM_INPUT_TR_START, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_1);
    Cy_TCPWM_InputTriggerSetup(PWM_TRI_HW, PWM_TRI_NUM, CY_TCPWM_INPUT_TR_START, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_1);

    Cy_TCPWM_InputTriggerSetup(PWM_B_HW, PWM_B_NUM, CY_TCPWM_INPUT_TR_STOP_OR_KILL, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_2);
    Cy_TCPWM_InputTriggerSetup(PWM_C_HW, PWM_C_NUM, CY_TCPWM_INPUT_TR_STOP_OR_KILL, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_2);
    Cy_TCPWM_InputTriggerSetup(PWM_A_HW, PWM_A_NUM, CY_TCPWM_INPUT_TR_STOP_OR_KILL, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_2);
    Cy_TCPWM_InputTriggerSetup(PWM_TRI_HW, PWM_TRI_NUM, CY_TCPWM_INPUT_TR_STOP_OR_KILL, CY_TCPWM_INPUT_LEVEL, CY_TCPWM_INPUT_TRIG_2);
}

/*******************************************************************************
* Function Name:   MotorCtrlConfigPwm
********************************************************************************
*
* Summary:
*  TCPWM modules parameters init      
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void MotorCtrlConfigPwm(void)
{
    int32_t temp_period;
    if (u16_motor_carrier_freq < u16_motor_carrier_freq_min)       /*min carry frequecy limit*/
    {
        u16_motor_carrier_freq = u16_motor_carrier_freq_min;
    }
    else if (u16_motor_carrier_freq > u16_motor_carrier_freq_max) /*max carry frequecy limit*/
    {
        u16_motor_carrier_freq = u16_motor_carrier_freq_max;
    }
    else
    {

    }
    temp_period = (PWM_FREQUENCY / u16_motor_carrier_freq) / CONSTANT_TWO;
    Cy_TCPWM_PWM_SetPeriod0(PWM_A_HW, PWM_A_NUM, temp_period);
    Cy_TCPWM_PWM_SetPeriod0(PWM_B_HW, PWM_B_NUM, temp_period);
    Cy_TCPWM_PWM_SetPeriod0(PWM_C_HW, PWM_C_NUM, temp_period);
    Cy_TCPWM_PWM_SetPeriod0(PWM_TRI_HW, PWM_TRI_NUM, (temp_period << MATH_SHIFT_ONE) - CONSTANT_ONE);
    pwm_peak_value = temp_period;
    if((Cy_TCPWM_PWM_GetPeriod0(PWM_A_HW, PWM_A_NUM) != Cy_TCPWM_PWM_GetPeriod0(PWM_B_HW, PWM_B_NUM)) || (Cy_TCPWM_PWM_GetPeriod0(PWM_A_HW, PWM_A_NUM) != Cy_TCPWM_PWM_GetPeriod0(PWM_C_HW, PWM_C_NUM)))
    {
        motor_control_run_par.u32_error_type |= UNDEFINED_INT;
        MotorControlStop();
    }

    if (f32_motor_dead_time_micro_sec < DEAD_TIME_MIN)   /*dead time min limit unit: us*/
    {
        f32_motor_dead_time_micro_sec = DEAD_TIME_MIN;
    }

    uint32_t u32temp_deadtime = 0;
    u32temp_deadtime = Q8(f32_motor_dead_time_micro_sec * PWM_FREQUENCY_75M);
    u32temp_deadtime >>= MATH_SHIFT_EIGHT;
    Cy_TCPWM_PWM_PWMDeadTime(PWM_A_HW, PWM_A_NUM, u32temp_deadtime);
    Cy_TCPWM_PWM_PWMDeadTimeN(PWM_A_HW, PWM_A_NUM, u32temp_deadtime);
    Cy_TCPWM_PWM_PWMDeadTime(PWM_B_HW, PWM_B_NUM, u32temp_deadtime);
    Cy_TCPWM_PWM_PWMDeadTimeN(PWM_B_HW, PWM_B_NUM, u32temp_deadtime);
    Cy_TCPWM_PWM_PWMDeadTime(PWM_C_HW, PWM_C_NUM, u32temp_deadtime);
    Cy_TCPWM_PWM_PWMDeadTimeN(PWM_C_HW, PWM_C_NUM, u32temp_deadtime);

    Cy_TCPWM_PWM_SetCompare0Val(PWM_A_HW, PWM_A_NUM, temp_period);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_B_HW, PWM_B_NUM, temp_period);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_C_HW, PWM_C_NUM, temp_period);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_TRI_HW, PWM_TRI_NUM, ((temp_period << MATH_SHIFT_ONE) - ADC_FORWARTIME));

    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_A_HW, PWM_A_NUM, temp_period);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_B_HW, PWM_B_NUM, temp_period);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_C_HW, PWM_C_NUM, temp_period);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_TRI_HW, PWM_TRI_NUM, ((temp_period << MATH_SHIFT_ONE) - ADC_FORWARTIME));
    
    motor_stc_svm_gen.i8_shunt_num = MOTOR_SHUNT_NUM;
    motor_stc_svm_gen.i16_cycle = Cy_TCPWM_PWM_GetPeriod0(PWM_A_HW, PWM_A_NUM);
    motor_stc_svm_gen.i16_duty_max = Q0(motor_stc_svm_gen.i16_cycle - (PWM_FREQUENCY_75M * DUTY_MAX_FACTOR));
    motor_stc_svm_gen.i16_dead_time = Q0(f32_motor_dead_time_micro_sec * PWM_FREQUENCY_75M);
}


/* [] END OF FILE */
