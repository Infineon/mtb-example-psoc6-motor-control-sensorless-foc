/*******************************************************************************
* File Name: customer_interface.c
* Version  : V1.0
*
* Description:
*  This file provides the parameters for customer debugging
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


#include "define.h"
#include "customer_interface.h"


/** UI_01  define the used motor parameter in this projec t*/ 
int32_t   i32_motor_pole_pairs      = 4;    /* the pole pairs of rotor */
float     f32_motor_ld              = 0.6;  /* the d axis reductance,unit:mh */
float     f32_motor_lq              = 0.6;  /* the q axis reductance,unit:mh */
float     f32_motor_res             = 0.8;  /* the resistance between two phases */

/** UI_02  define the hardware's a/d sample information, carrier frequency and deadtime */
int32_t   i32_motor_iuvw_offset_normal      = 2048;  /* the middle value of 12-bits ADC */
int32_t   i32_motor_iuvw_offset_range       = 150;   /* ADC offset range of Iuvw sampling  */
int32_t   i32_motor_iuvw_offset_check_times = 64;    /* Iuvw ADC sample offset check times  */
float     f32_motor_dead_time_micro_sec     = 1.0f;  /* Dead timer us  */
uint16_t  u16_motor_carrier_freq            = 10000; /* motor carry frequency (hz)  */
uint16_t  u16_motor_carrier_freq_min        = 5000;
uint16_t  u16_motor_carrier_freq_max        = 20000;


/** UI_03  define pid control parameter in this projec t*/ 

float f32_motor_dki                   = 0.02f;      /* d axis current PI regulator integral constant */
float f32_motor_dkp                   = 0.1999f;    /* d axis current PI regulator proportion constant */
float f32_motor_qki                   = 0.02f;      /* q axis current PI regulator integral constant */
float f32_motor_qkp                   = 0.1999f;    /* q axis current PI regulator proportion constant */

float f32_motor_low_speed_ki          = 0.005;     /* speed PI regulator integral constant */
float f32_motor_low_speed_kp          = 0.25f;     /* speed PI regulator proportion constant */
float f32_motor_ski                   = 0.0008f;   /* speed PI regulator integral constant */
float f32_motor_skp                   = 0.25;      /* speed PI regulator proportion constant */

uint16_t  u16_motor_change_pi_spdhz   = 20;        /* PID parameters change at this speed  */

/** UI_04  Cofigure startup parameters */
uint8_t   u8_motor_run_level           = 4;       /* 1->orientation */
                                                  /* 2->open loop running */
                                                  /* 3->closed loop running */
                                                  /* 4->change speed enable */
int16_t   i16q8_motor_orient_end_iqref       = Q8(2.0); /* orientation current */
int16_t   i16q8_motor_orient_init_iqref      = Q8(0);   /* orient start current */
float     f32q8_motor_orient_iqref_inc_aps   = 4.0f;    /* reference vary step in orient stage */
float     f32q8_motor_orient_time            = 0.5f;    /* orientation time,unit:s */
 
uint16_t  u16_motor_open_loop_spd_init_hz    = 5;       /* open loop start speed */
uint16_t  u16_motor_open_loop_spd_end_hz     = 10;      /* open loop end speed */
uint16_t  u16_motor_open_loop_spd_inc_hz     = 10;      /* acceleration speed of open loop */

int16_t   i16q8_motor_open_loop_init_iqref   = Q8(2.0);   /* q axis current reference of open loop */
int16_t   i16q8_motor_open_loop_end_iqref    = Q8(2.0);   /* q axis current reference in open loop stage */
float     f32_motor_open_loop_iqref_inc_aps  = 1.0f;      /* q axis current reference vary step in open loop */

/** UI_05  close-loop running parameters setting*/
uint16_t  u16_motor_close_loop_target_spdhz  = 10;         /* target speed when switching to close loop */
uint8_t   u8_motor_running_direction         = CLOCK_WISE; /* motor run sirection 0: CLOCK_WISE, 1:COUNT_LOCK_WISE */

int16_t   i16q8_motor_close_loop_is_max     = Q8(4.0);     /* max torque current when motor running */ 
int16_t   i16q8_motor_close_loop_iqref_max  = Q8(4.0);     /* the maximum value of q axis current reference in closed loop */
int16_t   u16_motor_spdmax                  = 4000;        /* motor run maximum speed rpm */ 
int16_t   u16_motor_spdmin                  = 500;         /* motor run minimum speed rpm */ 
float     f32_motor_spd_acceleration_hz     = 100.0f;      /* acceleration speed */
float     f32_motor_spd_deceleration_hz     = 100.0f;      /* deceleration speed */

/** UI_06  protection parameters setting*/
int16_t   i16q8_motor_current_max       = Q8(4);    /* motor phase current peak A */
uint16_t  u16_motor_vbus_max            = 30;       /* the maximum value of DC voltage */ 
uint16_t  u16_motor_vbus_min            = 20;       /* the minimum value of DC voltage */

/** UI_7  enable speed set parameters setting*/
uint8_t   u8_motor_speed_set_enable     = TRUE;


/* [] END OF FILE */
