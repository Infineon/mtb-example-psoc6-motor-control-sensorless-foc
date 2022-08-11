/*******************************************************************************
* File Name: math.h
* Version  : V1.0
*
* Description:
*  This file provides constants and parameter values for the mathematics  
*  calculation
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


#ifndef __MATH_H__          /* MATH_H */
#define __MATH_H__

#include <stdint.h>

/***************************************  
*   Conditional Compilation Parameters     
***************************************/ 


/***************************************  
*    Function Prototypes                    
***************************************/  

extern int32_t MathRightShift(int32_t i32data, uint32_t i32shift_bits);
extern int32_t MathSin(int32_t i32angle);
extern int32_t MathCos(int32_t i32angle);
extern int32_t MathSqrt(uint32_t i32root);
extern int32_t MathArcTan(int32_t i32_xpu, int32_t i32_ypu);


/***************************************  
*    Global pre-processor symbols/macros               
***************************************/  

#define qn_qm(qn, n, m) (((n) > (m)) ? ((int32_t)(qn) >> ((n) - (m))) : ((int32_t)(qn) << ((m) - (n))))


#endif /* __MATH_H__ */


/* [] END OF FILE */
