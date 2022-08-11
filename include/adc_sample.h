/*******************************************************************************
* File Name: adc_sample.h
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the motor phase current 
*  sense functions.
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

#ifndef __ADC_SAMPLE_H__        /* ADC_SAMPLE_H */
#define __ADC_SAMPLE_H__

#include "customer_interface.h"
#include "define.h"


/***************************************  
*   Conditional Compilation Parameters     
***************************************/  
  
typedef struct stc_muti_shunt_offest
{
    int32_t i32_xu; /* u phase sample offset */
    int32_t i32_xv; /* v phase sample offset */
    int32_t i32_xw; /* W phase sample offset */
    uint32_t offset_sample_num; /* offset check times  */
} stc_muti_shunt_offest_t;

typedef struct
{
    int32_t i32q14_motor_current_factor; /* current sample calculate factor */
    volatile uint8_t u8_complete_flag; /* ADC sampling complete flag */
    int32_t i32_vbusk; /* DC voltage sample calculate factor  */
} stc_sensor_t;

extern stc_sensor_t     adc_stc_sample;
extern  uint16_t        adc_rslt_of_adc[];
extern  volatile uint8_t adc_motor_status;


/***************************************  
*    Function Prototypes                    
***************************************/    

extern void AdcStart(void);
extern void AdcReadSample(void);
extern void AdcMotorCurrentSense(void);
extern void AdcInitSensorPar(uint16_t u16_sample_freq);
extern void AdcMotorSensorOffsetDetect(void);

/***************************************
*           API Constants        
***************************************/
#define MOTOR_OFFSET_CHECK_DELAY  (0.5f)     /* 0.5s adc offset check delay */
#define RIGHT_SHIFT_NUM           (6)        /* adc offset right shift number */
#define ADC_CH_AMOUNT             (4u)

#endif /* ADC_SAMPLE_H */


/* [] END OF FILE */
