/*******************************************************************************
* File Name: chip_init.h
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the MCU component 
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



#ifndef __CHIP_INIT_H__         /* CHIP_INIT_H */
#define __CHIP_INIT_H__

#include "define.h"

/***************************************  
*   Conditional Compilation Parameters     
***************************************/    


/***************************************  
*    Function Prototypes                    
***************************************/ 

extern void EnableSwitch(void);
extern void SetNormalGpioMode(void);
extern void SetPwmGpioMode(void);
extern void SetToHighLevel(void);
extern void SetToLowLevel(void);
extern void PwmStart(void);
extern void PwmStop(void);
extern void PwmBrake(void);
extern void MotorCtrlConfigPwm(void);
extern void PwmEnable(void);
extern void PwmDisable(void);

/* Constants SetMode(), mode parameter */  
#define     PWM_FREQUENCY_75M    (75)     
#define     PWM_FREQUENCY        (75000000)
#define     ADC_FORWARTIME       (24u)
#define     DUTY_MAX_FACTOR      (2.6f)
#define     DEAD_TIME_MIN        (1.0f)
#define     PWM_OUT_LOW_LEVEL     (0u)
#define     PWM_OUT_HIGH_LEVEL    (1u)
#define     CONSTANT_ZERO         (0u)
#define     DELAY_1MS             (1u)
#define     DELAY_500MS           (500u)
#define     CONSTANT_ONE          (1u)
#define     CONSTANT_TWO          (2u)
#endif /* __CHIP_INIT_H__ */


/* [] END OF FILE */
