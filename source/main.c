/*******************************************************************************
* File Name: main.c
* Version  : V1.0
*
* Description:
*  This file provides main function of this project
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


#include "stdlib.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "define.h"
#include "isr.h"
#include "motor_ctrl.h"
#include "chip_init.h"
#include "speed_set.h"


const cy_stc_sysint_t ADC_IRQ_cfg = {
    #if (CY_CPU_CORTEX_M0P)
         .intrSrc         = NvicMux8_IRQn,             /* CM0+ interrupt is NVIC #8 */
        .cm0pSrc         = pass_interrupt_sar_0_IRQn,    /* Source of NVIC #8 is the SAR interrupt */
         .intrPriority    =  7UL                        /* Interrupt priority is 7 */
    #else
         .intrSrc         =  pass_interrupt_sar_0_IRQn,    /* Interrupt source is the SAR interrupt */
        .intrPriority    = 3UL                        /* Interrupt priority is 7 *///5
    #endif
};

const cy_stc_sysint_t PwmInterrupt_cfg = {

         .intrSrc         =  tcpwm_0_interrupts_263_IRQn,    /* Interrupt source is the SAR interrupt *///tcpwm_1_interrupts_0_IRQn
        .intrPriority    = 7UL                        /* Interrupt priority is 7 */
};

const cy_stc_sysint_t IBus_OC_ISR_cfg = {

         .intrSrc         =  lpcomp_interrupt_IRQn,    /* Interrupt source is the SAR interrupt */
        .intrPriority    = 2UL                        /* Interrupt priority is 7 */
};

uint32 readClock;

/*******************************************************************************
* Function Name:   main
********************************************************************************
*
* Summary:
*  main function
*
* Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
int main()
{
    /*********************************************************************
    * motor initialization                                               *
    **********************************************************************/
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    PwmStart();

    /* Motor Paramters Init */
    MotorControlInitPar(u16_motor_carrier_freq);
    AdcInitSensorPar(u16_motor_carrier_freq);
    Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);
    /* Turn on the hardware block. */
    Cy_SysAnalog_Enable();
    /* SAR ADC Init */
    AdcStart();
    /* Opamp OPA0 Init */
    Cy_CTB_OpampInit(OPA0_HW, CY_CTB_OPAMP_0, &OPA0_config);

    Cy_CTB_Enable(OPA0_HW);

    cy_en_ctb_switch_register_sel_t switchSelect = CY_CTB_SWITCH_OA0_SW;

    /* Select two switches for Pin 0 and Pin 1 of the CTB port. */
    uint32_t switchMask = CY_CTB_SW_OA0_POS_PIN0_MASK | CY_CTB_SW_OA0_NEG_PIN1_MASK;

    /* Set the state of the switches to closed */
    cy_en_ctb_switch_state_t state = CY_CTB_SWITCH_CLOSE;

    Cy_CTB_SetAnalogSwitch(OPA0_HW, switchSelect, switchMask, state);

    /* OA0 has an additional isolation switch (CIS) on its vplus line */
    Cy_CTB_SetAnalogSwitch(OPA0_HW, CY_CTB_SWITCH_CTD_SW, CY_CTB_SW_CTD_CHOLD_OA0_POS_ISOLATE_MASK, state);
    CyDelay(500u);

    /* ADC ISR Init    */
    /* Configure the interrupt with vector at SAR_Interrupt(). */
    (void)Cy_SysInt_Init(&ADC_IRQ_cfg, AdcSampleInterrupt);
    /* Enable the interrupt. */
    NVIC_EnableIRQ(ADC_IRQ_cfg.intrSrc);
    /* Configure the PWM Main Loop IAR */
    Cy_SysInt_Init(&PwmInterrupt_cfg,PwmInterrupt);
    /* Enable the PWM Main Loop interrupt */
    NVIC_EnableIRQ(PwmInterrupt_cfg.intrSrc);
    /* Configure the IBus_OverCurrent_ISR */
    Cy_SysInt_Init(&IBus_OC_ISR_cfg, HardwareOvercurrentInterrupt);
    /* Enable the IBus_OverCurrent_ISR interrupt */
    NVIC_EnableIRQ(IBus_OC_ISR_cfg.intrSrc);

    Cy_LPComp_Init(lpcomp_0_comp_0_HW,lpcomp_0_comp_0_CHANNEL,&lpcomp_0_comp_0_config);
    Cy_LPComp_Enable(lpcomp_0_comp_0_HW,lpcomp_0_comp_0_CHANNEL);
    /*====================================================================*
     * protection initialization                                          *
     *====================================================================*/
    readClock = Cy_SysClk_ClkHfGetFrequency(0);

    CyDelay(500u);
    __enable_irq();

    PwmBrake();
    MotorControlInitPar(u16_motor_carrier_freq);
    Cy_TrigMux_SwTrigger(TRIG_OUT_MUX_2_TCPWM0_TR_IN0, CY_TRIGGER_TWO_CYCLES);
    CyDelay(1000u);
    Cy_GPIO_Write(IO_LED_PORT,IO_LED_NUM,0);

    while(1)
    { 
        /* motor */
        if ((motor_control_run_par.u8status == 0) &&(motor_control_run_par.u32_error_type == 0))
        {
            if(motor_control_run_par.i32_target_speed_rpm != 0)
            {
                MotorControlStart(u16_motor_carrier_freq);

            }
        }
        if(motor_control_run_par.u8status == 1)
        {
            if(motor_control_run_par.u32_error_type != 0||motor_control_run_par.i32_target_speed_rpm == 0)
            {
                MotorControlStop();
            }
        }
        /*******set speed*************/
        if(u8_motor_speed_set_enable == 1)
        {
            SpeedSet();
        }
        /***********run deriction and LED**********/
        peripheral_function();
    }
}


/* [] END OF FILE */
