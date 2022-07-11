//#############################################################################
//
// FILE:   adc_ex2_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
//! \addtogroup driver_example_list
//! <h1>ADC ePWM Triggering</h1>
//!
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A0 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults - A sequence of analog-to-digital conversion samples from
//!   pin A0. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
//
// $Release Date: $
// $Copyright:
// Copyright (C) 2013-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################
//

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define EX_ADC_RESOLUTION       12
// 12 for 12-bit conversion resolution, which supports (ADC_MODE_SINGLE_ENDED)
// Sample on single pin (VREFLO is the low reference)
// Or 16 for 16-bit conversion resolution, which supports (ADC_MODE_DIFFERENTIAL)
// Sample on pair of pins (difference between pins is converted, subject to
// common mode voltage requirements; see the device data manual)

//
// Globals
//
#include"globalDefinition.h"
#include"coreFunctions.h"

//
// Function Prototypes
//
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
__interrupt void adcA1ISR(void);


/*
 * CMF FUnctions
 */
void CMFconfigBoard(void);
void CMFconfigFiringSignals(void);
void CMFconfigIOSignals(void);


//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init(); //

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO(); //

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule(); //

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable(); //

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_ADCA1, &adcA1ISR); //

    //
    // Set up the ADC and the ePWM and initialize the SOC
    //
    initADC(); //
    initEPWM(); //
    initADCSOC(); //


    // CMF Definitions
    CMFconfigBoard();
    CMFconfigFiringSignals();
    CMFconfigIOSignals();


    //
    // Enable ADC interrupt
    //
    Interrupt_enable(INT_ADCA1);//

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop indefinitely
    //
    while(1)
    {
        // CMF

        // Enpty loop
        //GPIO_writePin(14, 1);
        //
        // Start ePWM1, enabling SOCA and putting the counter in up-count mode
        //
        EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
        EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

        //EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);
        //EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, 100);
    }
}

//
// Function to configure and power up ADCA.
//
void initADC(void)
{
    
    //
    // Set ADCDLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADC and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

//
// Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{
    //
      // Disable SOCA
      //
      EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

      //
      // Configure the SOC to occur on the first up-count event
      //
      EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
      EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

      //
      // Set the compare A value to 1000 and the period to 1999
      // Assuming ePWM clock is 100MHz, this would give 50kHz sampling
      // 50MHz ePWM clock would give 25kHz sampling, etc.
      // The sample rate can also be modulated by changing the ePWM period
      // directly (ensure that the compare A value is less than the period).
      //
      EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 1000);
      EPWM_setTimeBasePeriod(EPWM1_BASE, 1999);

      //
      // Set the local ePWM module clock divider to /1
      //
      EPWM_setClockPrescaler(EPWM1_BASE,
                             EPWM_CLOCK_DIVIDER_1,
                             EPWM_HSCLOCK_DIVIDER_1);

      //
      // Freeze the counter
      //
      EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
}

//
// Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{


    //
     // Configure SOC0 of ADCA to convert pin A0. The EPWM1SOCA signal will be
     // the trigger.
     //
     // For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
     // SYSCLK rate) will be used.  For 16-bit resolution, a sampling window of
     // 64 (320 ns at a 200MHz SYSCLK rate) will be used.
     //
 #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                     ADC_CH_ADCIN0, 15);
 #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                     ADC_CH_ADCIN0, 64);
 #endif

     //
     // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
     // sure its flag is cleared.
     //
     ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
     ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
     ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

//
// ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{

    static uint16_t epwm4_comp_a;
    static uint16_t epwm4_comp_b;

    static uint16_t epwm5_comp_a;
    static uint16_t epwm5_comp_b;

    static uint16_t epwm6_comp_a;
    static uint16_t epwm6_comp_b;

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);



    //GPIO_writePin(14, 1);

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 400);

    //epwm4_comp = (uint16_t)((vin+1)*250);
    //epwm4_comp = 50;
    //EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, epwm4_comp);

    //epwm4_comp = (uint16_t)((sin(2)+1)*250);

    //void SinGenerator(float *wt, float *vsin, float *wfreq, float freq, float phi, float Am, float Ts);

    //SinGenerator(wt_in, &v_in, w_in, 60, 0, 5, Tsamp);
    SinGenerator_h(wt_in,
                        &v_in,
                        w_in,
                        f1,
                        phi1,
                        V1p,
                        0,
                        V3p,
                        0,
                        V5p,
                        0,
                        V7p,
                        Tsamp);


    SOGI_EPLL_mod(&sda_valfa,
                  &sda_valfa_old,
                  &sda_vbeta,
                  &sda_vbeta_old,
                  &w,
                  &w_old,
                  &vbeta,
                  &vq,
                  &vq_old,
                  &pll_alfa,
                  &pll_alfa_old,
                  &pll_beta,
                  &pll_beta_old,
                  &teta,
                  &teta_old,
                  &acum_w,
                  &acum_w_old,
                  &wp,
                  &wi,
                  &erro_amp,
                  &erro_amp_old,
                  &amp,
                  &amp_old,
                  &v_in,
                  &erro1_sogi,
                  &erro1_sogi_old,
                  &erro2_sogi,
                  &erro2_sogi_old,
                  Tsamp,
                  ki_1,
                  kp_1,
                  wo_1,
                  Ka_1,
                  K_1);



    epwm4_comp_a = (uint16_t)((1+v_in*0.1)*250);
    epwm4_comp_b = (uint16_t)((1-v_in*0.1)*250);

    //epwm4_comp_a = (uint16_t)((1+w*0.002)*250);
    //epwm4_comp_b = (uint16_t)((1-w*0.002)*250);

    epwm5_comp_a = (uint16_t)((1+pll_alfa*0.1)*250);
    epwm5_comp_b = (uint16_t)((1-pll_alfa*0.1)*250);

    //epwm5_comp_a = (uint16_t)((1+teta*0.1)*250);
    //epwm5_comp_b = (uint16_t)((1-teta*0.1)*250);

    //epwm6_comp_a = (uint16_t)((1+pll_beta*0.1)*250);
    //epwm6_comp_b = (uint16_t)((1-pll_beta*0.1)*250);

    //epwm5_comp_a = (uint16_t)((1+amp*0.1)*250);
    //epwm5_comp_b = (uint16_t)((1-amp*0.1)*250);

    //epwm6_comp_a = (uint16_t)((1+w*0.001)*250);
    //epwm6_comp_b = (uint16_t)((1-w*0.001)*250);


    epwm6_comp_a = (uint16_t)((1+teta*0.1)*250);
    epwm6_comp_b = (uint16_t)((1-teta*0.1)*250);


    //epwm6_comp_a = (uint16_t)((1+sda_valfa*0.1)*250);
    //epwm6_comp_b = (uint16_t)((1-sda_valfa*0.1)*250);

    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, epwm4_comp_a);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_B, epwm4_comp_b);

    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, epwm5_comp_a);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, epwm5_comp_b);

    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, epwm6_comp_a);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, epwm6_comp_b);


    //GPIO_writePin(14, 0);

    if (flag<2000){
        flag++;
        GPIO_writePin(14, 0);
    }
    else{
        GPIO_writePin(14, 1);
        //phi1 = -1.57;
        //f1 = 50;
        flag++;


    }

    if (flag>4000){
        V3p = 1.667;
        V5p = 1;
        V7p = 0.714;
    }


    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}











// ***********************************************************************
//! Function to configure the board
// ***********************************************************************
void CMFconfigBoard(void){



    //
    // Set up PLL control and clock dividers
    //

    #define DEVICE_SETCLOCK_CFG_CMF         (SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(20) |  \
                                     SYSCTL_FMULT_NONE | SYSCTL_SYSDIV(0) |   \
                                     SYSCTL_PLL_ENABLE)
    SysCtl_setClock(DEVICE_SETCLOCK_CFG_CMF);


    //
    // Disable pull up on GPIO 0 configure them as PWM1A
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM1_BASE, 2500);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 100);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Disable SOCA
    //
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the first up-count event
    //
    //EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

}


// ***********************************************************************
//! Function to configure the PWM signals used to drive converters
//! To keep it apart from the ADC, I would consider here the ePWM 4, 5 and 6
// ***********************************************************************
void CMFconfigFiringSignals(void){



    /*  Setting of the ePWM 4*/

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM4_BASE, 500);
    EPWM_setPhaseShift(EPWM4_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM4_BASE);


    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);


    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM4_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);


    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set actions
    //
    /*
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                                  */
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    //
    // Configure the dead band
    //
    /*
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);

    EPWM_setDeadBandDelayPolarity(EPWM4_BASE,EPWM_DB_RED,EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE,EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    EPWM_setRisingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);

    EPWM_setRisingEdgeDelayCount(EPWM4_BASE, 20);
    EPWM_setFallingEdgeDelayCount(EPWM4_BASE, 20);
    */


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, 100);




    /*  Setting of the ePWM 5*/

        //
        // Set-up TBCLK
        //
        EPWM_setTimeBasePeriod(EPWM5_BASE, 500);
        EPWM_setPhaseShift(EPWM5_BASE, 0U);
        EPWM_setTimeBaseCounter(EPWM5_BASE, 0U);
        EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);
        EPWM_disablePhaseShiftLoad(EPWM5_BASE);


        //
        // Set up shadowing
        //
        EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE,
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);


        //
        // Set ePWM clock pre-scaler
        //
        EPWM_setClockPrescaler(EPWM5_BASE,
                               EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);


        //
        // Set actions
        //
        EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
        EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        //
        // Set actions
        //
        /*
        EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
        EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                                      */
        EPWM_setActionQualifierAction(EPWM5_BASE,
                                          EPWM_AQ_OUTPUT_B,
                                          EPWM_AQ_OUTPUT_HIGH,
                                          EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
        EPWM_setActionQualifierAction(EPWM5_BASE,
                                          EPWM_AQ_OUTPUT_B,
                                          EPWM_AQ_OUTPUT_LOW,
                                          EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        //
        // Configure the dead band
        //
        /*
        EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);

        EPWM_setDeadBandDelayPolarity(EPWM4_BASE,EPWM_DB_RED,EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(EPWM4_BASE,EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

        EPWM_setRisingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
        EPWM_setFallingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);

        EPWM_setRisingEdgeDelayCount(EPWM4_BASE, 20);
        EPWM_setFallingEdgeDelayCount(EPWM4_BASE, 20);
        */


        //
        // Set-up compare
        //
        EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, 100);


        /*  Setting of the ePWM 6*/

            //
            // Set-up TBCLK
            //
            EPWM_setTimeBasePeriod(EPWM6_BASE, 500);
            EPWM_setPhaseShift(EPWM6_BASE, 0U);
            EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);
            EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);
            EPWM_disablePhaseShiftLoad(EPWM6_BASE);


            //
            // Set up shadowing
            //
            EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE,
                                                 EPWM_COUNTER_COMPARE_A,
                                                 EPWM_COMP_LOAD_ON_CNTR_ZERO);


            //
            // Set ePWM clock pre-scaler
            //
            EPWM_setClockPrescaler(EPWM6_BASE,
                                   EPWM_CLOCK_DIVIDER_1,
                                   EPWM_HSCLOCK_DIVIDER_1);


            //
            // Set actions
            //
            EPWM_setActionQualifierAction(EPWM6_BASE,
                                          EPWM_AQ_OUTPUT_A,
                                          EPWM_AQ_OUTPUT_HIGH,
                                          EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
            EPWM_setActionQualifierAction(EPWM6_BASE,
                                          EPWM_AQ_OUTPUT_A,
                                          EPWM_AQ_OUTPUT_LOW,
                                          EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

            //
            // Set actions
            //
            /*
            EPWM_setActionQualifierAction(EPWM4_BASE,
                                          EPWM_AQ_OUTPUT_B,
                                          EPWM_AQ_OUTPUT_LOW,
                                          EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
            EPWM_setActionQualifierAction(EPWM4_BASE,
                                          EPWM_AQ_OUTPUT_B,
                                          EPWM_AQ_OUTPUT_HIGH,
                                          EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                                          */
            EPWM_setActionQualifierAction(EPWM6_BASE,
                                              EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_HIGH,
                                              EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
            EPWM_setActionQualifierAction(EPWM6_BASE,
                                              EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_LOW,
                                              EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

            //
            // Configure the dead band
            //
            /*
            EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
            EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);

            EPWM_setDeadBandDelayPolarity(EPWM4_BASE,EPWM_DB_RED,EPWM_DB_POLARITY_ACTIVE_HIGH);
            EPWM_setDeadBandDelayPolarity(EPWM4_BASE,EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

            EPWM_setRisingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
            EPWM_setFallingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);

            EPWM_setRisingEdgeDelayCount(EPWM4_BASE, 20);
            EPWM_setFallingEdgeDelayCount(EPWM4_BASE, 20);
            */


            //
            // Set-up compare
            //
            EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, 100);


}



// ***********************************************************************
//! Function to configure the IO signals used for analysis purposes
// ***********************************************************************
void CMFconfigIOSignals(void){

    // Configuring GPIO14 as a output
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFF);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_OUT);



    //
    // Disable pull up on GPIO 6 and 7 configure them as PWM4A and PWM4B
    //
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_7_EPWM4B);


    //
        // Disable pull up on GPIO 8 and 9 configure them as PWM5A and PWM5B
        //
        GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_8_EPWM5A);
        GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(GPIO_9_EPWM5B);


        //
                // Disable pull up on GPIO 10 and 11 configure them as PWM5A and PWM5B
                //
                GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
                GPIO_setPinConfig(GPIO_10_EPWM6A);
                GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
                GPIO_setPinConfig(GPIO_11_EPWM6B);



}
