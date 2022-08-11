/*******************************************************************************
* File Name: limitation.h
* Version  : V1.0
*
* Description:
*  This file provides the definitions about voltage and current limit during 
*  motor running process.
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


#ifndef _CV_LIMIT_           /* CV_LIMIT_H */
#define _CV_LIMIT_

#include <stdint.h>
#include "filter.h"


/*************************************** 
*   Conditional Compilation Parameters    
***************************************/ 

/* limitation structure */
typedef struct
{
    int32_t i32_vbusf; /** filtered DC bus voltage */
    int32_t i32_vdf;   /** filtered Vd voltage */
    int32_t i32_vqf;   /** filtered Vq voltage */
    int32_t i32_isdf;  /** filtered Id current */
    stc_dual_one_oder_lpf_t stc_vbus_lpf;/** LPF structure for DC bus voltage */
    stc_one_order_lpf_t stc_vd_lpf;      /** LPF structure Vd voltage */
    stc_one_order_lpf_t stc_vq_lpf;      /** LPF structure Vq voltage */
}stc_limit_t;

extern stc_limit_t  cv_limit_pars;


/***************************************
*        Function Prototypes
***************************************/

extern void CvLimitationInit(stc_limit_t *pstc_value);
extern void CvLimitationControl(void);

#endif    /* _CVLIMIT_H_ */


/* [] END OF FILE */
