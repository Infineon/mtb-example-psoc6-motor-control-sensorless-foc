/*******************************************************************************
* File Name: motor_ctrl.c
* Version  : V1.0
*
* Description:
*  This file provides motor control function of this project
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
#include "math.h"
#include "svpwm.h"
#include "smo_calculate.h"
#include "limitation.h"
#include "motor_ctrl.h"
#include "protect.h"
#include "motor_startup.h"
#include "chip_init.h"
#include "hardware_config.h"


/* current sample */
stc_uvw_t  motor_stc_iuvw_sensed;

/* coordinate transformation */ 
stc_ab_t   motor_stc_iab_sensed;
stc_dq_t   motor_stc_idq_sensed;
stc_dq_t   motor_contrl_vdq_ref;
stc_dq_t   motor_control_idq_ref;
stc_ab_t   motor_stc_vab_ref;
stc_ab_t   motor_stc_vab_real;
stc_uvw_t  motor_stc_vuvw_ref;

/* LPF */
stc_one_order_lpf_t motor_stc_spd_lpf = { Q12(MOTOR_SPEED_LPF_FACTOR), Q20(0.0) };

/* PID */
stc_pid_t  motor_contrl_iq_pid_reg;
stc_pid_t  motor_control_id_pid_reg;
stc_pid_t  motor_control_spd_pid_reg;

/* motor ctrl */
stc_motor_run_t    motor_control_run_par;
stc_soft_timers_t  motor_ctrl_soft_timer;

int32_t  i32q12_motor_control_is_max_square;
int32_t  i32q8_motor_control_iqmax;

int32_t  i32q8_motor_spd_pi_high_wm_hz;
int32_t  i32q16_motor_high_spd_pi_ki;
int32_t  i32q10_motor_high_spd_pi_kp;
int32_t  i32q16_motor_low_spd_pi_ki;
int32_t  i32q10_motor_low_spd_pi_kp;

int32_t  i32_motor_contrl_vq_max;

/* spd loop */
stc_spdLoop_t  motor_spd_loop_ctrl;


/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
static void MotorControlSpdReg(void);
static void MotorControlThetaGenerate(void);
static void MotorControlWriteOccp(stc_svm_gen_t *pstc_value);


/*******************************************************************************
* Function Name:   MotorControlInitPar
********************************************************************************
*
* Summary:
*  Init for some parameters before motor run           
*
* Parameters: 
*  u16_sample_freq
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void MotorControlInitPar(uint16_t u16_sample_freq)
{
 
    motor_control_run_par.i32q8_estmi_wm_hzf        = 0;
    motor_control_run_par.i32q22_delta_theta_kts    = Q22(QUANTIZATION_FACTOR * i32_motor_pole_pairs / u16_sample_freq);
    motor_control_run_par.i32q8_target_speed_wm_hz  = Q8(u16_motor_close_loop_target_spdhz);
    motor_control_run_par.i32q22_target_speed_wm_hz = Q22(u16_motor_close_loop_target_spdhz);
    motor_control_run_par.i32q22_target_wm_inc_ts   = Q22(f32_motor_spd_acceleration_hz * SPEED_ACC_DEC_FACTOR / u16_sample_freq);
    motor_control_run_par.i32q22_target_wm_dec_ts   = Q22(f32_motor_spd_deceleration_hz * SPEED_ACC_DEC_FACTOR / u16_sample_freq);
    motor_control_run_par.i32_target_speed_rpm_max  = Q0(u16_motor_spdmax);
    motor_control_run_par.i32_target_speed_rpm_min  = Q0(u16_motor_spdmin);
    motor_control_run_par.i32q22_elec_angle         = Q22(0.0);
    motor_control_run_par.u32_error_type            = 0;
    motor_control_run_par.u8_speed_pi_enable        = 0;
    motor_control_run_par.u8_startup_complete_flag  = 0;
    motor_control_run_par.u8_running_stage          = MOTOR_ORIENTATION;
    motor_control_run_par.u8_running_level          = u8_motor_run_level;
    motor_control_run_par.u8_close_loop_flag        = 0;
    motor_control_run_par.u8_change_speed_enable    = 0;
    motor_ctrl_soft_timer.u32_timer0     = 0;
    motor_ctrl_soft_timer.u32_timer1     = 0;
    motor_ctrl_soft_timer.u32_timer2     = 0;
    motor_ctrl_soft_timer.u32_timer3     = 0;
    motor_control_idq_ref.i32q8_xd = Q8(0.0f);
    motor_control_idq_ref.i32q8_xq = Q8(0.0f);
    motor_stc_vab_ref.i32q8_xa     = Q8(0.0f);
    motor_stc_vab_ref.i32q8_xb     = Q8(0.0f);
    motor_contrl_vdq_ref.i32q8_xd  = Q8(0.0f);
    motor_contrl_vdq_ref.i32q8_xq  = Q8(0.0f);

    motor_stc_svm_gen.i8_shunt_num = MOTOR_SHUNT_NUM;
    motor_stc_svm_gen.i16_t1       = 0;
    motor_stc_svm_gen.i16_t2       = 0;

    i32q8_motor_control_iqmax           = i16q8_motor_close_loop_iqref_max;
    i32q12_motor_control_is_max_square  = (i16q8_motor_close_loop_is_max * i16q8_motor_close_loop_is_max)  >>  MATH_SHIFT_FOUR;

    i32q8_motor_spd_pi_high_wm_hz = Q8(u16_motor_change_pi_spdhz);
    i32q16_motor_high_spd_pi_ki   = Q16(f32_motor_ski);
    i32q10_motor_high_spd_pi_kp   = Q10(f32_motor_skp);
    i32q16_motor_low_spd_pi_ki    = Q16(f32_motor_low_speed_ki);
    i32q10_motor_low_spd_pi_kp    = Q10(f32_motor_low_speed_kp);

    /* init pi regulator */
    motor_control_id_pid_reg.i32q15_kp = Q15(f32_motor_dkp);
    motor_control_id_pid_reg.i32q15_ki = Q15(f32_motor_dki);
    motor_control_id_pid_reg.i32q15_kd = Q15(0);
    motor_control_id_pid_reg.i32_out_max = Q8(PI_OUT_INIT_MAX_VALUE);
    motor_control_id_pid_reg.i32_out_min = Q8(PI_OUT_INIT_MIN_VALUE);
    motor_control_id_pid_reg.i32qn_i_out = 0;
    motor_control_id_pid_reg.i32_error0  = Q8(0);
    motor_control_id_pid_reg.i32_error1  = Q8(0);
    motor_control_id_pid_reg.i32_i_out   = Q8(0);
    motor_control_id_pid_reg.i32_out     = Q8(0);
    motor_control_id_pid_reg.i32_out_pre = 0;
    motor_control_id_pid_reg.i32_p_out   = 0;
    motor_control_id_pid_reg.i32_error0_max = Q8(CURRENT_PI_ERROR_INIT_MAX_VALUE);
    motor_control_id_pid_reg.i32_error0_min = Q8(CURRENT_PI_ERROR_INIT_MIN_VALUE);
    
    motor_contrl_iq_pid_reg.i32q15_kp = Q15(f32_motor_qkp);
    motor_contrl_iq_pid_reg.i32q15_ki = Q15(f32_motor_qki);
    motor_contrl_iq_pid_reg.i32q15_kd = Q15(0);
    motor_contrl_iq_pid_reg.i32_out_max = Q8(PI_OUT_INIT_MAX_VALUE);
    motor_contrl_iq_pid_reg.i32_out_min = Q8(0);
    motor_contrl_iq_pid_reg.i32qn_i_out = 0;
    motor_contrl_iq_pid_reg.i32_error0  = Q8(0);
    motor_contrl_iq_pid_reg.i32_error1  = Q8(0);
    motor_contrl_iq_pid_reg.i32_i_out   = Q8(0);
    motor_contrl_iq_pid_reg.i32_out     = Q8(0);
    motor_contrl_iq_pid_reg.i32_out_pre = 0;
    motor_contrl_iq_pid_reg.i32_p_out   = 0;
    motor_contrl_iq_pid_reg.i32_error0_max = Q8(CURRENT_PI_ERROR_INIT_MAX_VALUE);
    motor_contrl_iq_pid_reg.i32_error0_min = Q8(CURRENT_PI_ERROR_INIT_MIN_VALUE);
   
    motor_control_spd_pid_reg.i32q15_kp = Q15(f32_motor_low_speed_kp);
    motor_control_spd_pid_reg.i32q15_ki = Q15(f32_motor_low_speed_ki);
    motor_control_spd_pid_reg.i32q15_kd = Q15(0);
    motor_control_spd_pid_reg.i32_out_max = i16q8_motor_close_loop_iqref_max;
    motor_control_spd_pid_reg.i32_out_min = 0;
    motor_control_spd_pid_reg.i32qn_i_out = 0;
    motor_control_spd_pid_reg.i32_error0  = Q8(0);
    motor_control_spd_pid_reg.i32_error1  = Q8(0);
    motor_control_spd_pid_reg.i32_i_out   = Q8(0);
    motor_control_spd_pid_reg.i32_out     = Q8(0);
    motor_control_spd_pid_reg.i32_out_pre = 0;
    motor_control_spd_pid_reg.i32_p_out   = 0;
    motor_control_spd_pid_reg.i32_d_out   = 0;
    motor_control_spd_pid_reg.i32_error0_max = Q8(SPEED_PI_ERROR_INIT_MAX_VALUE);
    motor_control_spd_pid_reg.i32_error0_min = Q8(SPEED_PI_ERROR_INIT_MIN_VALUE);
    motor_control_spd_pid_reg.i_cnt   = 0;
    motor_control_spd_pid_reg.i_timer = SPEED_PI_TIMER;
    
    motor_spd_loop_ctrl.i32_speed_loop_cnt   = 0;
    motor_spd_loop_ctrl.i32_speed_loop_cycle = SPEED_LOOP_CYCLE;
    MotorProtectInit(u16_motor_carrier_freq);
}


/*******************************************************************************
* Function Name:   MotorControlStart
********************************************************************************
*
* Summary:
*  Init for start parameters            
*
* Parameters: 
*  u16_sample_freq
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void MotorControlStart(uint16_t u16_sample_freq)
{
    MotorProtectInit(u16_sample_freq);
    MotorControlInitPar(u16_sample_freq);
    MotorStartInit(&motor_stc_startup);
    CvLimitationInit(&cv_limit_pars);
    SmoInit(&motor_stc_smo);
    
    /* SVPWM initial */
    motor_stc_svm_cal.q26_Invsvpwm_duty = Q26(QUANTIZATION_FACTOR / motor_stc_svm_gen.i16_cycle);
    motor_stc_svm_cal.svpwm_duty        = motor_stc_svm_gen.i16_cycle;
    motor_stc_svm_cal.svpwm_duty_max    = motor_stc_svm_gen.i16_duty_max;
    motor_stc_svm_cal.u16_t1            = 0;
    motor_stc_svm_cal.u16_t2            = 0;
    
    if(SENSOR_RUNNING == adc_motor_status)
    {
        PwmEnable();
        motor_control_run_par.u8status = MOTOR_RUNNING;
    }
}


/*******************************************************************************
* Function Name:   MotorControlStop
********************************************************************************
*
* Summary:
*  set for stop parameters                
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void MotorControlStop(void)
{
    motor_control_run_par.u8status = MOTOR_STOP;
    motor_control_run_par.i32_target_speed_rpm = 0;
    motor_control_run_par.i32q8_estmi_wm_hz   = 0;
    motor_control_run_par.i32q8_estmi_wm_hzf  = 0;
    motor_stc_iuvw_sensed.i32q8_xu = 0;
    motor_stc_iuvw_sensed.i32q8_xv = 0;
    motor_stc_iuvw_sensed.i32q8_xw = 0;
    
    PwmStop();
    
    /*prevent running restart*/
    CyDelay(DELAY_500MS);
}


/*******************************************************************************
* Function Name:   MotorControlSpdReg
********************************************************************************
*
* Summary:
*  motor speed regulator     
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
static void MotorControlSpdReg(void)
{
    motor_spd_loop_ctrl.i32_speed_loop_cnt ++;
    if(motor_spd_loop_ctrl.i32_speed_loop_cnt > motor_spd_loop_ctrl.i32_speed_loop_cycle)
    {
        motor_spd_loop_ctrl.i32_speed_loop_cnt = 0;
    }
    int32_t i32q8_target_speed_hz_temp;
    FilterOneOrderLpf(motor_control_run_par.i32q8_estmi_wm_hz, &motor_control_run_par.i32q8_estmi_wm_hzf, &motor_stc_spd_lpf);
    motor_control_run_par.i32_motor_speed_lpf = (motor_control_run_par.i32q8_estmi_wm_hzf*FRENQUENCY_TO_SPEED_CONSTANT) >> MATH_SHIFT_EIGHT;

    /********************************Motor Target Speed Regulate***************************/
    if (TRUE == motor_control_run_par.u8_change_speed_enable)
    {   
        if(0 != motor_control_run_par.i32_target_speed_rpm)
        {            
            if (motor_control_run_par.i32_target_speed_rpm > motor_control_run_par.i32_target_speed_rpm_max)
            {
                    motor_control_run_par.i32_target_speed_rpm = motor_control_run_par.i32_target_speed_rpm_max;
            }
            else if(motor_control_run_par.i32_target_speed_rpm < motor_control_run_par.i32_target_speed_rpm_min)
            {
                    motor_control_run_par.i32_target_speed_rpm = motor_control_run_par.i32_target_speed_rpm_min;
            }         
            i32q8_target_speed_hz_temp = (motor_control_run_par.i32_target_speed_rpm << MATH_SHIFT_EIGHT) / FRENQUENCY_TO_SPEED_CONSTANT;
            if ((i32q8_target_speed_hz_temp - motor_control_run_par.i32q8_target_speed_wm_hz) > (motor_control_run_par.i32q22_target_wm_inc_ts  >>  MATH_SHIFT_FOURTEEN))
            {
                motor_control_run_par.i32q22_target_speed_wm_hz += motor_control_run_par.i32q22_target_wm_inc_ts;
                motor_control_run_par.i32q8_target_speed_wm_hz = motor_control_run_par.i32q22_target_speed_wm_hz  >>  MATH_SHIFT_FOURTEEN;
            }
            else if ((i32q8_target_speed_hz_temp - motor_control_run_par.i32q8_target_speed_wm_hz) < -(motor_control_run_par.i32q22_target_wm_dec_ts  >>  MATH_SHIFT_FOURTEEN))
            {
                motor_control_run_par.i32q22_target_speed_wm_hz -= motor_control_run_par.i32q22_target_wm_dec_ts;
                motor_control_run_par.i32q8_target_speed_wm_hz = motor_control_run_par.i32q22_target_speed_wm_hz  >>  MATH_SHIFT_FOURTEEN;
            }
            else
            {
                motor_control_run_par.i32q8_target_speed_wm_hz = i32q8_target_speed_hz_temp;
            }
        }
    }
    motor_control_spd_pid_reg.i32q15_kp = i32q10_motor_high_spd_pi_kp << MATH_SHIFT_FIVE;
    motor_control_spd_pid_reg.i32q15_ki = i32q16_motor_high_spd_pi_ki >> MATH_SHIFT_ONE;

    /*******************************Motor PI Paramater Regulate***************************/
    if (SPEEDPI_RUNNING == motor_control_run_par.u8_speed_pi_enable && motor_spd_loop_ctrl.i32_speed_loop_cnt == motor_spd_loop_ctrl.i32_speed_loop_cycle)
    {
        motor_spd_loop_ctrl.i32_speed_loop_cnt = 0; 
        PidPosition(&motor_control_spd_pid_reg, motor_control_run_par.i32q8_target_speed_wm_hz-motor_control_run_par.i32q8_estmi_wm_hzf);
        motor_control_idq_ref.i32q8_xq = motor_control_spd_pid_reg.i32_out;
    } 
}


/*******************************************************************************
* Function Name:   MotorControlThetaGenerate
********************************************************************************
*
* Summary:
*  motor angle generate  
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
static void MotorControlThetaGenerate(void)
{
    uint32_t ui32_temp_angle;

    if (motor_control_run_par.u8_close_loop_flag == 2)
    {
        motor_control_run_par.i32q8_estmi_wm_hz = motor_stc_smo.i32q8_estmi_wm_hz;
        motor_control_run_par.i32q22_elec_angle = (int32_t) (motor_stc_smo.i32q22_theta);
        motor_control_run_par.i32q22_delta_theta_ts = motor_stc_smo.i32q22_delta_theta;
        motor_stc_smo.u8_close_loop_flag = 1;
    }
    else if (motor_control_run_par.u8_close_loop_flag == 0)
    {
        motor_control_run_par.i32q8_estmi_wm_hz = motor_stc_smo.i32q8_estmi_wm_hz;
        motor_control_run_par.i32q22_elec_angle += motor_control_run_par.i32q22_delta_theta_ts;
        motor_stc_smo.u8_close_loop_flag = 0;
    }
    motor_control_run_par.i32q22_elec_angle &= 0x3FFFFF;
    
    ui32_temp_angle = motor_control_run_par.i32q22_elec_angle >> MATH_SHIFT_TWELVE;

    motor_contrl_vdq_ref.i32q12_sin = MathSin(ui32_temp_angle);  
    motor_contrl_vdq_ref.i32q12_cos = MathCos(ui32_temp_angle);

    motor_stc_idq_sensed.i32q12_sin = motor_contrl_vdq_ref.i32q12_sin;
    motor_stc_idq_sensed.i32q12_cos = motor_contrl_vdq_ref.i32q12_cos;
    motor_control_idq_ref.i32q12_sin = motor_contrl_vdq_ref.i32q12_sin;
    motor_control_idq_ref.i32q12_cos = motor_contrl_vdq_ref.i32q12_cos;
}


/*******************************************************************************
* Function Name:   MotorControlWriteOccp
********************************************************************************
*
* Summary:
*  PWM output
*
* Parameters: 
*  pstc_value
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void MotorControlWriteOccp(stc_svm_gen_t *pstc_value)
{
    if(CLOCK_WISE == u8_motor_running_direction)
    {
        Cy_TCPWM_PWM_SetCompare1(PWM_A_HW, PWM_A_NUM, pstc_value->i16_aon);
        Cy_TCPWM_PWM_SetCompare1(PWM_B_HW, PWM_B_NUM, pstc_value->i16_bon);
        Cy_TCPWM_PWM_SetCompare1(PWM_C_HW, PWM_C_NUM, pstc_value->i16_con);
    }
    else
    {
        Cy_TCPWM_PWM_SetCompare1(PWM_A_HW, PWM_A_NUM, pstc_value->i16_bon);
        Cy_TCPWM_PWM_SetCompare1(PWM_B_HW, PWM_B_NUM, pstc_value->i16_aon);
        Cy_TCPWM_PWM_SetCompare1(PWM_C_HW, PWM_C_NUM, pstc_value->i16_con);
    }
}


/*******************************************************************************
* Function Name:   MotorControlProcess
********************************************************************************
*
* Summary:
*  Motor control main flow       
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void MotorControlProcess(void)
{
    motor_ctrl_soft_timer.u32_timer3++;
    motor_control_run_par.u32_error_type |= MotorProtectVoltage(motor_control_run_par.i32q8_vbus, &protect_out_voltage_par);

    while (adc_stc_sample.u8_complete_flag != TRUE);
    adc_stc_sample.u8_complete_flag = 0;
    AdcMotorCurrentSense();
    Clarke(&motor_stc_iuvw_sensed, &motor_stc_iab_sensed);
    if (MOTOR_RUNNING == motor_control_run_par.u8status)
    {
        motor_control_run_par.u32_error_type |= MotorProtectOverCurrent(&protect_over_current_par, &motor_stc_iuvw_sensed);
        motor_control_run_par.u32_error_type |= MotorProtectLosePhase(&protect_lose_phase_par, &motor_stc_iuvw_sensed, &motor_control_idq_ref);
        Park(&motor_stc_iab_sensed, &motor_stc_idq_sensed);
        SmoEstimate(&motor_stc_smo, &motor_stc_vab_real, &motor_stc_iab_sensed);
        motor_ctrl_soft_timer.u32_timer3 &= MOTOR_SOFT_TIMER;
        if (SOFT_TIMER_0 == motor_ctrl_soft_timer.u32_timer3)
        {
            MotorControlSpdReg();
        }
        else if (SOFT_TIMER_1 == motor_ctrl_soft_timer.u32_timer3)
        {
            CvLimitationControl();
        }
        else if (SOFT_TIMER_2 == motor_ctrl_soft_timer.u32_timer3)
        {
            /* CY_NOP; */
        }
        if(ERROR_TYPE_NONE != motor_control_run_par.u32_error_type)
        {
            MotorControlStop();
        }
  
        PidPosition(&motor_control_id_pid_reg, motor_control_idq_ref.i32q8_xd - motor_stc_idq_sensed.i32q8_xd);
        motor_contrl_vdq_ref.i32q8_xd = motor_control_id_pid_reg.i32_out;
        PidPosition(&motor_contrl_iq_pid_reg, motor_control_idq_ref.i32q8_xq - motor_stc_idq_sensed.i32q8_xq);
        motor_contrl_vdq_ref.i32q8_xq = motor_contrl_iq_pid_reg.i32_out;
        if(motor_contrl_vdq_ref.i32q8_xq > i32_motor_contrl_vq_max)
        {
            motor_contrl_vdq_ref.i32q8_xq = i32_motor_contrl_vq_max;
        }
        MotorControlThetaGenerate();
        if(FALSE == motor_control_run_par.u8_startup_complete_flag)
        {
            motor_control_run_par.u8_startup_complete_flag = MotorStartUp(&motor_stc_startup);
            motor_control_idq_ref.i32q8_xq = (motor_stc_startup.i32q12_iq) >> MATH_SHIFT_FOUR;
        }

        InvPark(&motor_contrl_vdq_ref, &motor_stc_vab_ref);
        InvClarkeTransform(&motor_stc_vab_ref, &motor_stc_vuvw_ref);
        
        motor_control_run_par.i32q8_vdc_invt = Q16(motor_stc_svm_cal.svpwm_duty) / motor_control_run_par.i32q8_vbus;
        SvmCalc(&motor_stc_svm_cal, &motor_stc_vab_ref, &motor_stc_vuvw_ref, &motor_stc_vab_real, &motor_control_run_par);
        motor_stc_svm_gen.i16_aon = motor_stc_svm_cal.u16_uon;
        motor_stc_svm_gen.i16_bon = motor_stc_svm_cal.u16_von;
        motor_stc_svm_gen.i16_con = motor_stc_svm_cal.u16_won;
        motor_stc_svm_gen.i16_t1  = motor_stc_svm_cal.u16_t1;
        motor_stc_svm_gen.i16_t2  = motor_stc_svm_cal.u16_t2;
        motor_stc_svm_gen.i8_sector_pre = motor_stc_svm_gen.i8_sector;
        motor_stc_svm_gen.i8_sector = motor_stc_svm_cal.sector;
        MotorControlWriteOccp(&motor_stc_svm_gen);
    }  
    EnableSwitch();
}


/* [] END OF FILE */
