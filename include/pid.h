/*******************************************************************************
* File Name: pid.h
* Version  : V1.0
*
* Description:
*  This file provides the definitions about pid algorithm.
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


#ifndef __PID_REG_H__     /* PID_REG_H */
#define __PID_REG_H__

#include <stdint.h> 

#ifdef __cplusplus
extern "C"
#endif


/*************************************** 
*   Conditional Compilation Parameters    
***************************************/ 

/* PID structure */

typedef struct
{
    int32_t i32q15_kp; /*p coefficient for pid calculate*/
    int32_t i32q15_ki; /*i coefficient for pid calculate*/
    int32_t i32q15_kd; /*d coefficient for pid calculate*/
    int32_t i_cnt;        /*counter for I Out calculate*/
    int32_t i_timer;   /*cycle for I Out calculate*/
    int32_t i32_p_out;  /*Output: Item P */
    int32_t i32_i_out;  /*Output: Item I */
    int32_t i32_d_out;  /*Output: Item D */
    int32_t i32_out;   /*Output: pid regulator*/
    int32_t i32_out_pre;/*Last Output: pid regulator*/
    int32_t i32qn_i_out; /*Output: Item I QN format*/
    int32_t i32_out_max; /*output upper limitation*/
    int32_t i32_out_min; /*output lower limitation*/  
    int32_t i32_error0;     /** input-error */
    int32_t i32_error1;     /** last input-error */
    int32_t i32_error0_max;  /** input-error max limitation */
    int32_t i32_error0_min;  /** input-error min limitation */

}stc_pid_t;


/***************************************
*        Function Prototypes
***************************************/

extern void PidPosition(stc_pid_t *pstc_value, int32_t error);
extern void PidIncrement(stc_pid_t *pstc_value, int32_t error);

#ifdef __cplusplus
}
#endif


#endif /* __PID_REG_H__ */


/* [] END OF FILE */
