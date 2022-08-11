/*******************************************************************************
* File Name: hardware_config.h
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the hardware 
*  configuration
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


#ifndef __HW_CFG_H__
#define __HW_CFG_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "define.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/******************************************************************************/
/******************************************************************************/
   
#define SYS_VDC_FACTOR         (20.1f)   /* DC voltage sample resistor factor */
#define MOTOR_SHUNT_NUM        (2)       /* The number of shunt used to sense current */
#define MOTOR_IUVW_SAMPLE_RESISTOR    (0.03f) /* Iuvw sample resistor (ohm) */
#define MOTOR_IUVW_AMPLIFIER_FACTOR   (4.16f) /* Iuvw calculation factor */


#define ADC_VOLT_REF           (3.3f)     /* Reference voltage for ADC */
#define ADC_VALUE_MAX          (4096.0f)  /* 12-bits ADC max value */

#define COMP_ADC_CH_IU         (0)      /* ADC channel - Iu */
#define COMP_ADC_CH_IV                  /* ADC channel - Iv */
#define COMP_ADC_CH_IW         (1)      /* ADC channel - Iw */
#define SYS_ADC_CH_VDC         (2)      /* ADC channel - Vdc */
#define MOTOR_SPEED_VR         (3)


/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
                          
/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

#endif /* __HW_CFG_H__ */

