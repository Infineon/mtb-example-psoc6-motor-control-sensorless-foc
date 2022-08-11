/*******************************************************************************
* File Name: svpwm.h
* Version  : V1.0
*
* Description:
*  This file provides the parameters and definitions about svpwm algorithm.
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


#ifndef __SVPWM_H__     /* SVPWM_H */
#define __SVPWM_H__

#include <stdint.h>
#include "coordinate_transform.h"
#include "motor_ctrl.h"


/*************************************** 
*   Conditional Compilation Parameters    
***************************************/ 

typedef struct
{
     int32_t q8_xyz_k1;
     int32_t q8_xyz_k2;
     int32_t q12_time_extra_par;
     int32_t svpwm_duty_max;
     int32_t svpwm_duty;
     int32_t q26_Invsvpwm_duty;
     int32_t t1_t2_min;
     int16_t x;
     int16_t y;
     int16_t z;
     int16_t u16_t1;
     int16_t u16_t2;
     uint16_t aon;
     uint16_t bon;
     uint16_t con;
     uint16_t occp_update_point_next;
     uint16_t accp0_num_next;
     uint16_t accp00_next;
     uint16_t accp01_next;
     uint16_t accp02_next;
     uint8_t sector;
     uint8_t sector_pre;
     uint8_t time_extra_f;

    uint16_t u16_uon;
    uint16_t u16_von;
    uint16_t u16_won;
}stc_svm_cal_t;

typedef struct stc_svm_gen
{
    int16_t    i16_duty_max;
    int16_t    i16_cycle;
    int16_t    i16_dead_time;
    int16_t    i16_sample_wind;
    int16_t    i16_sample_offset;

    int16_t    i16_t1;
    int16_t    i16_t2;

    int16_t    i16_aon;
    int16_t    i16_bon;
    int16_t    i16_con;

    int16_t    i16_aon_up;
    int16_t    i16_bon_up;
    int16_t    i16_con_up;
    int16_t    i16_aon_down;
    int16_t    i16_bon_down;
    int16_t    i16_con_down;

    volatile int16_t i16_adc_trig_t2;
    volatile int16_t i16_adc_trig_t1;

    volatile int16_t i16_adc_trig_index;
    volatile int16_t i16_adc_trig_t1_buf;

    int8_t i8_shunt_num;
    int8_t i8_sector;
    int8_t i8_sector_pre;
}stc_svm_gen_t;

/* SVPWM */
extern stc_svm_gen_t           motor_stc_svm_gen; /* generated svpwm value */
extern stc_svm_cal_t           motor_stc_svm_cal;


/***************************************
*        Function Prototypes
***************************************/

extern void SvmCalc(stc_svm_cal_t *motor_svpwm_par ,stc_ab_t *motor_2s_voltage, stc_uvw_t *_3sv, stc_ab_t *_2sv_real, stc_motor_run_t*motor_run_par);


#endif /* __SVPWM_H__ */


/* [] END OF FILE */
