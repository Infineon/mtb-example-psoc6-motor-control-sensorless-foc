/*******************************************************************************
* File Name: motor_startup
* Version  : V1.0
*
* Description:
*  This file provides the parameters and definitions for motor start up.
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


#ifndef _MotorStartUp_H_
#define _MotorStartUp_H_

#include <stdint.h>
#include "define.h"


/*************************************** 
*   Conditional Compilation Parameters    
***************************************/ 
typedef struct
{
    INT8U    start_finish_flag;  /* flag for motor start complete */
    int32_t  i32q12_iq;      /* q axis current */
    uint32_t u32_timer;      /* count time for motor start up */
    uint32_t u32_orient_time;  /* orient time */
    int32_t  u32_orient_iqref;  /* q axis current reference at orient stage */
    int32_t  i32q20_orient_iq;     /* q axis current at orient stage */
    int32_t  i32q20_orient_inc_iq_ts; /* q axis current acceleration at orient stage */
    int32_t  i32q20_force_inc_iq_ts; /* q axis current acceleration at open loop stage */
    int32_t  i32q12_force_iq_ref;   /* q axis current reference at open loop stage */
    int32_t  i32q20_force_iq;      /* q axis current at open loop stage */
    int32_t  i32q22_force_wm_hz;    /* speed at open loop */
    int32_t  i32q22_force_wm_hz_ref; /* open loop end speed */
    int32_t  i32q22_force_open_inc_ts; /* acceleration at open loop */
} stc_motor_start_t;

extern stc_motor_start_t    motor_stc_startup;


/***************************************
*        Function Prototypes
***************************************/

void MotorStartInit(stc_motor_start_t* pstc_start);
INT8U MotorStartUp(stc_motor_start_t* pstc_start);

/***************************************
*           API Constants        
***************************************/

#define MOTOR_PREHEAT            0
#define MOTOR_ORIENTATION        1
#define MOTOR_FORCE_STARTUP      2
#define MOTOR_ENTER_CLOSELOOP    3
#define MOTOR_CHANGE_SPEED       4


#endif /* _MotorStartUp_H_ */


/* [] END OF FILE */
