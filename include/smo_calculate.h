/*******************************************************************************
* File Name: smo_calculate.h
* Version  : V1.0
*
* Description:
*  This file provides the definitions about smo_calculate algorithm.
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

#ifndef __SMO_CAL_H__      /* SMO_CAL_H */
#define __SMO_CAL_H__

#include <stdint.h>
#include "define.h"
#include "filter.h"
#include "math.h"
#include "coordinate_transform.h"
    
/*************************************** 
*   Conditional Compilation Parameters    
***************************************/ 

typedef struct stc_smo_estimator
{
    int32_t i32q8_res;            /* the phase resistance */           
    int32_t i32q8_lddt;           /* q axis inductance digital factor */
    int32_t i32q12_ld_lq;         /* dq Axis Mutual Inductance */
    
    int32_t i32q8_ialpha_pre;     /* stationary alpha-axis stator current */
    int32_t i32q8_ibeta_pre;      /* stationary beta-axis stator current */
    int32_t i32q8_valpha_pre;     /* stationary alpha-axis stator voltage */
    int32_t i32q8_vbeta_pre;      /* stationary beta-axis stator voltage */

    int32_t i32q8_valpha_bemf;    /*eistimated alpha Back EMF */
    int32_t i32q8_vbeta_bemf;     /* eistimated beta Back EMF */
    int32_t i32q8_valpha_bemfLpf; /* filtered alpha Back EMF for angle calculate */
    int32_t i32q8_vbeta_bemfLpf;  /* filtered beta Back EMF for angle calculate */
    stc_one_order_lpf_t valpha_bemf_lpfk; /* LPF factor */
    stc_one_order_lpf_t vbeta_bemf_lpfk; /* LPF factor */
    
    int32_t i32q22_estim_wm_hz;   /* estimated rotor speed Q22 format */
    int32_t i32q8_estmi_wm_hz;    /* estimated rotor speed Q8 format */
    int32_t i32q8_estmi_wm_hzf;   /* filtered estimated rotor speed Q8 format */
    stc_one_order_lpf_t stc_clock_wisem_lpf; /* LPF factor */
    
    int32_t i32q12_cos;      /* sin value */
    int32_t i32q12_sin;      /* cos value */
    int32_t i32q12_cosPre;   /* last sin value */
    int32_t i32q12_sinPre;   /* last cos value */
    
    int32_t i32q22_theta;     /* estimated rotor angle */
    int32_t i32q22_theta_old;  /* estimated rotor angle old */
    int32_t i32q22_delta_theta; /* delta theta of rotor angle for speed calculate */
    uint16_t u16_1ms_count;    /* counter used to calculate motor speed */

    int32_t i32q12_max_lpfk;   /* BackEMF voltage's max filter parameter */
    int32_t i32q12_min_lpfk;   /* BackEMF voltage's min filter parameter */
    int32_t i32q15_lpf_kts;    /* BackEMF filter's calculation factor */
    uint16_t u16_1ms_timer;
    int32_t i32_speed_cal_kts;
    uint8_t u8_close_loop_flag;
}stc_smo_estimator_t;

extern stc_smo_estimator_t motor_stc_smo;


/***************************************
*        Function Prototypes
***************************************/

extern void SmoEstimate(stc_smo_estimator_t *pstc_estim_par, stc_ab_t *pstc_2s_vol, stc_ab_t *pstc_2s_current);
extern void SmoInit(stc_smo_estimator_t *smo_eistimator_t);


#endif   /* __SMO_CAL_H__ */


/* [] END OF FILE */
