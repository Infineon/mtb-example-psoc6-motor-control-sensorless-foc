/*******************************************************************************
* File Name: define.h
* Version  : V1.0
*
* Description:
*  This file provides the macro definitions that required by the project
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


#ifndef _DEFINE_H      /* DEFINE_H */
#define _DEFINE_H

#include <stdint.h>

/* header files for Motor Control binary library */

#ifdef  __cplusplus
extern "C" {
#endif

#ifdef  __cplusplus
}
#endif

/*************************************** 
*    Initial Parameter Constants
***************************************/
#ifndef CLOCK_WISE
#define CLOCK_WISE      (0u)       /* motor running direction clock wise */
#endif

#ifndef COUNT_LOCK_WISE
#define COUNT_LOCK_WISE (1u)      /* motor running direction count clock wise */
#endif

#ifndef TRUE
#define TRUE    (1u)               /* constant definition */
#endif

#ifndef FALSE
#define FALSE   (0u)               /* constant definition */
#endif

#ifndef LED_ON
#define LED_ON  (0u)               /* constant definition */
#endif

#ifndef LED_OFF
#define LED_OFF (1u)               /* constant definition */
#endif

#ifndef PI
#define PI  (3.1415926f)          /* PI constant definition */
#endif

#ifndef MATH_SHIFT_ONE
#define MATH_SHIFT_ONE     (1u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_TWO
#define MATH_SHIFT_TWO     (2u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_THREE
#define MATH_SHIFT_THREE   (3u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_FOUR
#define MATH_SHIFT_FOUR    (4u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_FIVE
#define MATH_SHIFT_FIVE    (5u)        /* define for math shift */
#endif

#ifndef MATH_SHIFT_SIX
#define MATH_SHIFT_SIX     (6u)        /* define for math shift */
#endif

#ifndef MATH_SHIFT_SEVEN
#define MATH_SHIFT_SEVEN    (7u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_EIGHT
#define MATH_SHIFT_EIGHT    (8u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_TEN
#define MATH_SHIFT_TEN      (10u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_TWELVE
#define MATH_SHIFT_TWELVE   (12u)         /* define for math shift */
#endif

#ifndef MATH_SHIFT_FOURTEEN
#define MATH_SHIFT_FOURTEEN   (14u)         /* define for math shift */
#endif

/*************************************** 
*    Define data types
***************************************/

#ifndef _STD_TYPE__
#define _STD_TYPE__

typedef signed   char       BOOL;   /* Boolean */
typedef unsigned char       INT8U;  /* Unsigned  8 bit quantity */
typedef signed   char       INT8S;  /* Signed    8 bit quantity */
typedef unsigned short      INT16U; /* Unsigned 16 bit quantity */
typedef signed   short      INT16S; /* Signed   16 bit quantity */
typedef unsigned int        INT32U; /* Unsigned 32 bit quantity */
typedef signed   int        INT32S; /* Signed   32 bit quantity */
typedef unsigned long       LONG32U;/* Unsigned 32 bit quantity */
typedef signed   long       LONG32S;/* Signed   32 bit quantity */
typedef unsigned long long  INT64U; /* Unsigned 64 bit quantity */
typedef signed   long long  INT64S; /* signed   64 bit quantity */
typedef float               FP32;   /* single precision floating point */
typedef double              FP64;   /* double precision floating point */

#endif

/* Qn format storage type */
typedef INT8S  Qn_VAL8;
typedef INT16S Qn_VAL16;
typedef INT32S Qn_VAL32;
typedef INT64S Qn_VAL64;


/***************************************************
*    Define Q(n) format functions
****************************************************/
#define  Q0(value)  (Qn_VAL32)((value) * 0x00000001)
#define  Q1(value)  (Qn_VAL32)((value) * 0x00000002)
#define  Q2(value)  (Qn_VAL32)((value) * 0x00000004)
#define  Q3(value)  (Qn_VAL32)((value) * 0x00000008)
#define  Q4(value)  (Qn_VAL32)((value) * 0x00000010)
#define  Q5(value)  (Qn_VAL32)((value) * 0x00000020)
#define  Q6(value)  (Qn_VAL32)((value) * 0x00000040)
#define  Q7(value)  (Qn_VAL32)((value) * 0x00000080)
#define  Q8(value)  (Qn_VAL32)((value) * 0x00000100)
#define  Q9(value)  (Qn_VAL32)((value) * 0x00000200)
#define Q10(value)  (Qn_VAL32)((value) * 0x00000400)
#define Q11(value)  (Qn_VAL32)((value) * 0x00000800)
#define Q12(value)  (Qn_VAL32)((value) * 0x00001000)
#define Q13(value)  (Qn_VAL32)((value) * 0x00002000)
#define Q14(value)  (Qn_VAL32)((value) * 0x00004000)
#define Q15(value)  (Qn_VAL32)((value) * 0x00008000)
#define Q16(value)  (Qn_VAL32)((value) * 0x00010000)
#define Q17(value)  (Qn_VAL32)((value) * 0x00020000)
#define Q18(value)  (Qn_VAL32)((value) * 0x00040000)
#define Q19(value)  (Qn_VAL32)((value) * 0x00080000)
#define Q20(value)  (Qn_VAL32)((value) * 0x00100000)
#define Q21(value)  (Qn_VAL32)((value) * 0x00200000)
#define Q22(value)  (Qn_VAL32)((value) * 0x00400000)
#define Q23(value)  (Qn_VAL32)((value) * 0x00800000)
#define Q24(value)  (Qn_VAL32)((value) * 0x01000000)
#define Q25(value)  (Qn_VAL32)((value) * 0x02000000)
#define Q26(value)  (Qn_VAL32)((value) * 0x04000000)
#define Q27(value)  (Qn_VAL32)((value) * 0x08000000)
#define Q28(value)  (Qn_VAL32)((value) * 0x10000000)
#define Q29(value)  (Qn_VAL32)((value) * 0x20000000)
#define Q30(value)  (Qn_VAL32)((value) * 0x40000000)

#define GET_ABS(x)          (((x) >= 0) ? (x) : (-x))    /* get absolute value */
#define DEGREE(VALUE) (0x3FFFFF/360*VALUE)               /* for angle calculate */

#endif /* DEFINE_H */


/* [] END OF FILE */
