/*******************************************************************************
* File Name: motor_ctrl.h
* Version  : V1.0
*
* Description:
*  This file provides the parameters and definitions for FOC algoritm.
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


#ifndef __MOTOR_CTRL_H__   /*__MOTOR_CTRL_H__ */
#define __MOTOR_CTRL_H__

#include "coordinate_transform.h"
#include "customer_interface.h"
#include "adc_sample.h"
#include "pid.h"
#include "filter.h"


/*************************************** 
*   Conditional Compilation Parameters    
***************************************/ 

/* motor running */

typedef struct
{
    volatile int32_t  i32_target_speed_rpm; /* motor target speed */
    int32_t  i32_motor_speed_lpf;
    int32_t  i32_target_speed_rpm_max;      /* motor max target speed */
    int32_t  i32_target_speed_rpm_min;      /* motor min target speed */
    int32_t  i32q8_estmi_wm_hz;             /* motor speed Hz */
    int32_t  i32q8_estmi_wm_hzf;            /* motor speed Hz LPF */
    uint8_t  u8status;                      /* motor running status */
    uint32_t u32_error_type;                /* motor running error type */
    int32_t  i32q8_vbus;                    /* sampled bus voltage */
    int32_t  i32q8_vr;                      /* sampled VR value */
    int32_t  i32q8_vdc_invt;

    int32_t  i32q22_delta_theta_ts;         /* detheta */
    int32_t  i32q22_delta_theta_kts;        /* detheta calculate factor */
    int32_t  i32q8_target_speed_wm_hz;      /* motor target speed Hz format */
    int32_t  i32q22_target_speed_wm_hz;     /* motor target speed Hz format */
    int32_t  i32q22_target_wm_inc_ts;       /* motor target speed acceleration */
    int32_t  i32q22_target_wm_dec_ts;       /* motor target speed decleration */
    int32_t  i32q22_elec_angle;             /* motor target electric angle */
    uint8_t  u8_speed_pi_enable;            /* flag for speed PI enable or disable */
    uint8_t  u8_startup_complete_flag;      /* flag for startup complete */
    uint8_t  u8_running_stage;              /* motor running stage */
    uint8_t  u8_running_level;
    uint8_t  u8_close_loop_flag;            /* flag for enter closed loop or not */
    uint8_t  u8_change_speed_enable;        /* flag for change speed or not */
} stc_motor_run_t;

typedef struct
{
    volatile uint32_t u32_timer0;
    volatile uint32_t u32_timer1;
    volatile uint32_t u32_timer2;
    volatile uint32_t u32_timer3;
} stc_soft_timers_t;

/* structure for spd regulate */
typedef struct
{
     int32_t i32_speed_loop_cnt;
     int32_t i32_speed_loop_cycle;
} stc_spdLoop_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
/* sample */
extern stc_uvw_t       motor_stc_iuvw_sensed;  /* motor current sampling results */

/* coordinate transformation */
extern stc_ab_t        motor_stc_iab_sensed;  /* alpha-beta axis current */
extern stc_dq_t        motor_stc_idq_sensed;  /* d-q axis current */
extern stc_dq_t        motor_control_idq_ref; /* d-q axis reference current */
extern stc_dq_t        motor_contrl_vdq_ref;  /* d-q axis reference voltage */
extern stc_ab_t        motor_stc_vab_ref;     /* alpha-beta axis reference voltage */
extern stc_ab_t        motor_stc_vab_real;    /* alpha-beta axis real voltage */

/* PID */
extern stc_pid_t       motor_contrl_iq_pid_reg;   /* structure for Iq Pi */
extern stc_pid_t       motor_control_id_pid_reg;  /* structure for Id Pi */
extern stc_pid_t       motor_control_spd_pid_reg; /* structure for spd Pi */

/* motor ctrl */
extern stc_motor_run_t  motor_control_run_par;
extern stc_soft_timers_t       motor_ctrl_soft_timer;
extern stc_one_order_lpf_t     motor_stc_spd_lpf;

extern int32_t i32q12_motor_control_is_max_square;
extern int32_t i32q8_motor_control_iqmax;

extern int32_t i32q8_motor_spd_pi_high_wm_hz;
extern int32_t i32q16_motor_high_spd_pi_ki;
extern int32_t i32q10_motor_high_spd_pi_kp;
extern int32_t i32q16_motor_low_spd_pi_ki;
extern int32_t i32q10_motor_low_spd_pi_kp;

extern int32_t i32_motor_contrl_vq_max;

/***************************************
*        Function Prototypes
***************************************/

extern void MotorControlStop(void);
extern void MotorControlProcess(void);
extern void MotorControlStart(uint16_t u16_sample_freq);
extern void MotorControlInitPar(uint16_t u16_sample_freq);


/***************************************
*           API Constants        
***************************************/

#define MOTOR_STOP             (0u)
#define MOTOR_RUNNING          (1u)

#define SPEEDPI_STOP           (0u)
#define SPEEDPI_RUNNING        (1u)

#define SENSOR_STOP            (0u)
#define SENSOR_RUNNING         (1u)

#define QUANTIZATION_FACTOR    (1.0f)
#define SPEED_ACC_DEC_FACTOR   (3.0f)
#define MOTOR_SPEED_LPF_FACTOR (0.01f)
/*motor error type defination*/
#define NORMAL_RUNNING       (0x00)
#define OVER_VOLTAGE         (0x01)
#define UNDER_VOLTAGE        (0x02)
#define SW_OVER_CURRENT      (0x04)
#define MOTOR_OVER_CURRENT   (0x08)
#define MOTOR_LOSE_PHASE     (0x10)
#define AD_MIDDLE_ERROR      (0x40)
#define UNDEFINED_INT        (0x200)

#define FRENQUENCY_TO_SPEED_CONSTANT (60u)

#define PI_OUT_INIT_MAX_VALUE   (10)
#define PI_OUT_INIT_MIN_VALUE   (-10)

#define CURRENT_PI_ERROR_INIT_MAX_VALUE  (5)
#define CURRENT_PI_ERROR_INIT_MIN_VALUE  (-5)
#define SPEED_PI_ERROR_INIT_MAX_VALUE    (10)
#define SPEED_PI_ERROR_INIT_MIN_VALUE    (-10)
#define SPEED_PI_TIMER       (1u)
#define SPEED_LOOP_CYCLE     (2u)
#define MOTOR_SOFT_TIMER     (0x03)
#define SOFT_TIMER_0         (0u)
#define SOFT_TIMER_1         (1u)
#define SOFT_TIMER_2         (2u)
#define ERROR_TYPE_NONE      (0u)


#endif /* __MOTOR_CTRL_H__ */


/* [] END OF FILE */
