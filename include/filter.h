/*******************************************************************************
* File Name: filter.h
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the filter functions
*  
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

#ifndef __FILTER_H__
#define __FILTER_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <math.h>
#include <stdint.h>
#include "define.h"
/***************************************
*      Data Struct Definition
***************************************/
/* one order LPF structure */
typedef struct stc_one_order_lpf
{
    int32_t i32_q12_lpf_coe;   /** one-order low pass filter coefficient */
    int32_t i32_q20_low_bits;/** accumulation for the low pass filter */
}stc_one_order_lpf_t;
/* dual one order LPF structure */
typedef struct
{
    int32_t q12_up_lpf_coe;/** filter coefficient when input < previous output */
    int32_t q12_do_lpf_coe;/** filter coefficient when input > previous output */
    int32_t q20_low_bits;/** accumulation for the filter */
} stc_dual_one_oder_lpf_t;

/***************************************
*        Function Prototypes
***************************************/
extern void FilterOneOrderLpf(int32_t i32input, int32_t *pi32output, stc_one_order_lpf_t *pstc_lpf_par);

extern void FilterDualOneOrderLpf(int32_t i32input, int32_t *pi32output, stc_dual_one_oder_lpf_t *pstc_lpf_par);

#endif /* __FILTER_H__ */

