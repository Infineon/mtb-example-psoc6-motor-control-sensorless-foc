/*******************************************************************************
* File Name: coordinate_transform.h
* Version  : V1.0
*
* Description:
*  This file provides the function and variables' definition for coordinate tranforms 
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


#ifndef __EQU_TRANSFORM_H__
#define __EQU_TRANSFORM_H__

#include <stdint.h>

/***************************************
*      Data Struct Definition
***************************************/
/* u-v-w axis structure */
typedef struct
{
    int32_t i32q8_xu;    /** phase-a variable  */
    int32_t i32q8_xv;    /** phase-b variable  */
    int32_t i32q8_xw;    /** phase-c variable  */
}stc_uvw_t;
/* alpha-beta axis structure */
typedef struct
{
    int32_t i32q8_xa;    /** alpha variable of fixed 2- phase  */
    int32_t i32q8_xb;    /** beta variable of fixed 2- phase  */
}stc_ab_t;
/* d-q axis structure */
typedef struct
{
    int32_t i32q8_xd;    /** d-axis variable   */
    int32_t i32q8_xq;    /** q-axis variable   */
    int32_t i32q12_cos;  /** cosine value with angle */
    int32_t i32q12_sin;  /** sine value with angle */
}stc_dq_t;
/***************************************
*        Function Prototypes
***************************************/
extern void Clarke(stc_uvw_t *pstc_uvw, stc_ab_t *pstc_ab);

extern void InvClarke(stc_ab_t *pstc_ab, stc_uvw_t *pstc_uvw);

extern void Park(stc_ab_t *pstc_ab, stc_dq_t *pstc_dq);

extern void InvPark(stc_dq_t *pstc_dq, stc_ab_t *pstc_ab);

extern void InvClarkeTransform(stc_ab_t * input, stc_uvw_t * output);

#endif /* __EQU_TRANSFORM_H__ */

