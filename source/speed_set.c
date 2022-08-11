/*******************************************************************************
* File Name: SpeedSet.c
* Version  : V1.0
*
* Description:
*  This file provides motor speed set function of this project
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
#include "cy_pdl.h"
#include "cycfg.h"
#include "define.h"
#include "speed_set.h"
#include "motor_ctrl.h"
#include "isr.h"


uint8_t start_cnt = 0;
uint8_t protect_cnt = 0;


/*******************************************************************************
* Function Name:   SpeedSet
********************************************************************************
*
* Summary:
*  Use AD sampled value to set motor speed            
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/
void SpeedSet(void)
{
    if(motor_control_run_par.i32q8_vr >= SPEED_VR_START)
    {
        if(motor_control_run_par.i32_target_speed_rpm > 0)
        {
            motor_control_run_par.i32_target_speed_rpm = ((motor_control_run_par.i32q8_vr*(u16_motor_spdmax - u16_motor_spdmin)) >> MATH_SHIFT_SEVEN) + u16_motor_spdmin;
        }
        else
        {
            start_cnt++;
            if(start_cnt > 250)
            {
               motor_control_run_par.i32_target_speed_rpm = ((motor_control_run_par.i32q8_vr*(u16_motor_spdmax - u16_motor_spdmin)) >> MATH_SHIFT_SEVEN) + u16_motor_spdmin;
               start_cnt = 0;
            }
        }
    }

    if(motor_control_run_par.i32q8_vr <= SPEED_VR_MIN)
    {
        motor_control_run_par.i32_target_speed_rpm = 0;
        start_cnt = 0;
    }

    if((motor_control_run_par.i32q8_vr > SPEED_VR_MIN) && (motor_control_run_par.i32q8_vr < SPEED_VR_START))
    {
        start_cnt = 0;
    }
}


/*******************************************************************************
* Function Name:   peripheral_function
********************************************************************************
*
* Summary:
*  Running direction and led set            
*
* Parameters: 
*  None
*                            
* Return:  
*  None
*                            
**********************************************************************************/

void peripheral_function(void)
{
    static uint8_t  motor_run_dir = 0;
    static uint32_t led_motor_speed = 0;
    static uint32_t led_count_speed = 0;
    static uint8_t  sw_status = 0;
    sw_status = Cy_GPIO_Read(IO_SW_PORT, IO_SW_NUM);
    
    /***run direction change***/
    if((FALSE == sw_status) && (COUNT_LOCK_WISE == motor_run_dir))
    {
        motor_run_dir = 0;
        CyDelay(DELAY_10MS);
        if(FALSE == sw_status)
        {
            MotorControlStop();
            if(CLOCK_WISE == u8_motor_running_direction)
            {
                u8_motor_running_direction = COUNT_LOCK_WISE;
            }
            else
            {
                u8_motor_running_direction = CLOCK_WISE;
            }
            CyDelay(DELAY_1000MS);
        }
    }
    if(TRUE == sw_status)
    {
        motor_run_dir = COUNT_LOCK_WISE;
    }
    
     /***********LED Status********/
    if(motor_control_run_par.i32_target_speed_rpm > 0)
    {
        led_motor_speed = (uint8_t)(motor_control_run_par.i32_target_speed_rpm >> MATH_SHIFT_TEN);
        if(led_motor_speed <= 0)
        {
            led_motor_speed = LED_BLINK_MIN;
        }
        led_count_speed = LED_ON_OFF_CYCLE / led_motor_speed;
        if(led_count_time < led_count_speed)
        {
            Cy_GPIO_Write(IO_LED_PORT, IO_LED_NUM, LED_OFF);
        }
        else if((led_count_time >= led_count_speed) && (led_count_time < (LED_BLINK_TIME * led_count_speed)))
        {
            Cy_GPIO_Write(IO_LED_PORT, IO_LED_NUM, LED_ON);
        }
        else if(led_count_time >= (LED_BLINK_TIME * led_count_speed))
        {
            led_count_time = 0;
        }
    }
    else
    {
        Cy_GPIO_Write(IO_LED_PORT, IO_LED_NUM, LED_ON);
    }

    if((motor_control_run_par.u32_error_type != 0) && (protect_cnt < PROTECT_RESTART_NUM) && (motor_control_run_par.i32_target_speed_rpm == 0))
    {
        if((motor_control_run_par.i32q8_vr != 0) && (motor_control_run_par.i32q8_vr <= SPEED_VR_MIN))/***Clear errorType*****/
        {
            protect_cnt++;
            motor_control_run_par.u32_error_type = 0;
        }
    }
}


/* [] END OF FILE */
