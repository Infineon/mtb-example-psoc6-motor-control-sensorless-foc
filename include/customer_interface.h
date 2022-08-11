/*******************************************************************************
* File Name: customer_interface.h
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the customer debugging
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


#ifndef __CUSTOMER_INTERFACE_H__
#define __CUSTOMER_INTERFACE_H__

#include "define.h"

/***************************************  
*   Conditional Compilation Parameters     
***************************************/  

extern int32_t   i32_motor_pole_pairs; /* the pole pairs of rotor */
extern float     f32_motor_ld;         /* the d axis reductance */
extern float     f32_motor_lq;         /* the q axis reductance */
extern float     f32_motor_res;        /* the resistance between two phases */

/* sampling */
extern int32_t   i32_motor_iuvw_offset_normal;
extern int32_t   i32_motor_iuvw_offset_range;       /* ADC offset range of Iuvw sampling */
extern int32_t   i32_motor_iuvw_offset_check_times; /* Iuvw ADC sample offset check */

/* Deadtime, carrier frequency */

extern float     f32_motor_dead_time_micro_sec;  /* dead time, unit:us */
extern uint16_t  u16_motor_carrier_freq;         /* motor carry frequency (hz) */
extern uint16_t  u16_motor_carrier_freq_min;
extern uint16_t  u16_motor_carrier_freq_max;


/* PID */

extern float     f32_motor_dki;         /* d axis current PI regulator integral constant */
extern float     f32_motor_dkp;         /* d axis current PI regulator proportion constant */
extern float     f32_motor_qki;         /* q axis current PI regulator integral constant */
extern float     f32_motor_qkp;         /* q axis current PI regulator proportion constant */

extern float     f32_motor_low_speed_ki;    /* speed PI regulator integral constant */
extern float     f32_motor_low_speed_kp;    /* speed PI regulator proportion constant */
extern float     f32_motor_ski;             /* speed PI regulator integral constant */
extern float     f32_motor_skp;             /* speed PI regulator proportion constant */

extern uint16_t  u16_motor_change_pi_spdhz;  /* PID parameters change at this speed */

/* running control of open loop */

extern uint8_t   u8_motor_run_level; /* 1->orientation */
                                     /* 2->open loop running */
                                     /* 3->closed loop running */
                                     /* 4->change speed enable */

extern int16_t   i16q8_motor_orient_end_iqref;    /* orientation current  */
extern int16_t   i16q8_motor_orient_init_iqref;

extern float     f32q8_motor_orient_iqref_inc_aps;
extern float     f32q8_motor_orient_time;

extern uint16_t  u16_motor_open_loop_spd_init_hz;
extern uint16_t  u16_motor_open_loop_spd_inc_hz;
extern uint16_t  u16_motor_open_loop_spd_end_hz;

extern int16_t   i16q8_motor_open_loop_init_iqref;
extern int16_t   i16q8_motor_open_loop_end_iqref;  /* q axis current reference in open loop stage */
extern float     f32_motor_open_loop_iqref_inc_aps;

/* running control of closeloop */

extern uint16_t  u16_motor_close_loop_target_spdhz;
extern uint8_t   u8_motor_running_direction;
extern int16_t   u16_motor_spdmax;            /* motor run maximum speed rpm */
extern int16_t   u16_motor_spdmin;            /* motor run minimum speed rpm */
extern float     f32_motor_spd_acceleration_hz; /* acceleration speed */
extern float     f32_motor_spd_deceleration_hz; /* deceleration speed */

extern int16_t   i16q8_motor_close_loop_is_max;
extern int16_t   i16q8_motor_close_loop_iqref_max; /* the maximum value of q axis current reference in closed loop */

/* protection */

extern int16_t   i16q8_motor_current_max;    /* motor phase current peak A */
extern uint16_t  u16_motor_vbus_max;         /* the maximum value of DC voltage is 26V */
extern uint16_t  u16_motor_vbus_min;         /* the minimum value of DC voltage is 20V */

extern uint8_t   u8_motor_speed_set_enable;

/***************************************  
*    Function Prototypes                    
***************************************/   

#endif /* __CUSTOMER_INTERFACE_H__ */


/* [] END OF FILE */
