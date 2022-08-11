/*******************************************************************************
* File Name: protect.c
* Version  : V1.0
*
* Description:
*  This file provides the protect functions for motor control
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
#include "define.h"
#include "protect.h"
#include "coordinate_transform.h"
#include "customer_interface.h"
#include "protect.h"


stc_losephase_t            protect_lose_phase_par;
stc_out_voltage_t          protect_out_voltage_par;
stc_over_current_t         protect_over_current_par;

uint8_t  fault_occur_num = 1;
uint8_t  lose_phase_detect_num = 100;

/*******************************************************************************
* Function Name:   MotorProtectLosePhase
********************************************************************************
*
* Summary:
*  Lose phase protect             
*
* Parameters: 
*  *pstcPar:   structure for lose phase check   
*
*  *pstc_motor_iuvw_current: motor phase current
*
*  *pstc_motor_idq_current_ref: motor d-q axis reference current 
*                            
* Return:  
*  u32_error_type
*                            
**********************************************************************************/
uint32_t  MotorProtectLosePhase(stc_losephase_t *pstcPar, stc_uvw_t *pstc_motor_iuvw_current, stc_dq_t *pstc_motor_idq_current_ref)
{
    int32_t i32_temp_xu;
    int32_t i32_temp_xv;
    int32_t i32_temp_xw;
    uint32_t u32_error_type = 0;
    stc_ab_t iab_ref;

    InvPark(pstc_motor_idq_current_ref,  &iab_ref);
    InvClarke(&iab_ref, &pstcPar->iuvw_ref);

    pstcPar->iuvw_ref.i32q8_xu = GET_ABS(pstcPar->iuvw_ref.i32q8_xu) >> MATH_SHIFT_TWO;
    pstcPar->iuvw_ref.i32q8_xv = GET_ABS(pstcPar->iuvw_ref.i32q8_xv) >> MATH_SHIFT_TWO;
    pstcPar->iuvw_ref.i32q8_xw = GET_ABS(pstcPar->iuvw_ref.i32q8_xw) >> MATH_SHIFT_TWO;

    i32_temp_xu = GET_ABS(pstc_motor_iuvw_current->i32q8_xu);
    i32_temp_xv = GET_ABS(pstc_motor_iuvw_current->i32q8_xv);
    i32_temp_xw = GET_ABS(pstc_motor_iuvw_current->i32q8_xw);


    if(i32_temp_xu >= pstcPar->iuvw_ref.i32q8_xu)
    {
        pstcPar->iu_cnt++;
    }

    if(i32_temp_xv >= pstcPar->iuvw_ref.i32q8_xv)
    {
        pstcPar->iv_cnt++;
    }
    if(i32_temp_xw >= pstcPar->iuvw_ref.i32q8_xw)
    {
        pstcPar->iw_cnt++;
    }
    if(++pstcPar->detect_time_cnt > pstcPar->detect_time_num)
    {
        if(0 == pstcPar->iu_cnt || 0 == pstcPar->iv_cnt || 0 == pstcPar->iw_cnt)
        {
            u32_error_type = MOTOR_LOSE_PHASE;
        }
        pstcPar->detect_time_cnt = 0;
        pstcPar->iu_cnt           = 0;
        pstcPar->iv_cnt           = 0;
        pstcPar->iw_cnt           = 0;
    }
    return u32_error_type;
}


/*******************************************************************************
* Function Name:   MotorProtectOverCurrent
********************************************************************************
*
* Summary:
*  Over Current protect          
*
* Parameters: 
*  *pstcPar:   structure for over current check
*
*  *pstc_motor_iuvw_current: motor phase current
*                            
* Return:  
*  u32_error_type
*                            
**********************************************************************************/
uint32_t  MotorProtectOverCurrent(stc_over_current_t *pstcPar,  stc_uvw_t *pstc_motor_iuvw_current)
{
    int32_t i32_temp_xu;
    int32_t i32_temp_xv;
    int32_t i32_temp_xw;
    
    uint32_t u32_error_type = 0;
    i32_temp_xu = GET_ABS(pstc_motor_iuvw_current->i32q8_xu);
    i32_temp_xv = GET_ABS(pstc_motor_iuvw_current->i32q8_xv);
    i32_temp_xw = GET_ABS(pstc_motor_iuvw_current->i32q8_xw);
    
    if(i32_temp_xu > pstcPar->motor_current_max || i32_temp_xv > pstcPar->motor_current_max || i32_temp_xw > pstcPar->motor_current_max)
    {
        if(++pstcPar->over_current_cnt > pstcPar->over_current_num)
        {
            u32_error_type |= SW_OVER_CURRENT;
            pstcPar->over_current_cnt = 0;
        }
    }
    else
    {
        pstcPar->over_current_cnt = 0;
    }
    return u32_error_type;
}


/*******************************************************************************
* Function Name:   MotorProtectVoltage
********************************************************************************
*
* Summary:
*  Over voltage protect          
*
* Parameters: 
*  i32q8_vbus: DC-link voltage
*
*  *pstcPar:   structure for over voltage check
*                            
* Return:  
*  u32_error_type
*                            
**********************************************************************************/
uint32_t  MotorProtectVoltage(int32_t i32q8_vbus,  stc_out_voltage_t *pstcPar)
{
    uint32_t u32_error_type = 0;
    if(i32q8_vbus > pstcPar->bus_voltage_max)
    {
         if(++pstcPar->over_voltage_cnt > pstcPar->over_voltage_num)
         {
            u32_error_type |= OVER_VOLTAGE;
            pstcPar->over_voltage_cnt = 0;
         }
    }
    else if(i32q8_vbus < pstcPar->bus_voltage_min)
    {
         if(++pstcPar->under_voltage_cnt > pstcPar->under_voltage_num)
         {
            u32_error_type |= UNDER_VOLTAGE;
            pstcPar->under_voltage_cnt = 0;
         }
    }
    else
    {
        pstcPar->over_voltage_cnt  = 0;
        pstcPar->under_voltage_cnt = 0;
    }
    return u32_error_type;
}


/*******************************************************************************
* Function Name:   Protector_Init
********************************************************************************
*
* Summary:
*  Motor protect parameters init             
*
* Parameters: 
*  sample_freq      
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void MotorProtectInit(int32_t sample_freq)
{
    protect_out_voltage_par.bus_voltage_max        = Q8(u16_motor_vbus_max);
    protect_out_voltage_par.bus_voltage_min        = Q8(u16_motor_vbus_min);
    protect_out_voltage_par.over_voltage_num       = fault_occur_num;
    protect_out_voltage_par.under_voltage_num      = fault_occur_num;
    protect_out_voltage_par.over_voltage_cnt       = 0;
    protect_out_voltage_par.under_voltage_cnt      = 0;

    protect_over_current_par.motor_current_max     = i16q8_motor_current_max;
    protect_over_current_par.over_current_num      = fault_occur_num;
    protect_over_current_par.over_current_cnt      = 0;

    protect_lose_phase_par.detect_time_num         = lose_phase_detect_num;
    protect_lose_phase_par.detect_time_cnt         = 0;
    protect_lose_phase_par.iu_cnt                  = 0;
    protect_lose_phase_par.iv_cnt                  = 0;
    protect_lose_phase_par.iw_cnt                  = 0;
}


/* [] END OF FILE */
