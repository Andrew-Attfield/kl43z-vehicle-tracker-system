/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKL43Z4_Project_Lab 7.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"

volatile uint32_t hundredths_counter = 0;
volatile uint8_t timer_running = 0;

// Interrupt handler for PORTD
void PORTC_PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1u << 3)) {
        timer_running = 1;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        PTD->PTOR |= (1u << 5);
        PORTD->ISFR = (1u << 3);
    }
}

// Interrupt handler for PORTA
void PORTA_IRQHandler(void) {
    if (PORTA->ISFR & (1u << 4)) {
        timer_running = 0;
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        hundredths_counter = 0;
        PTD->PTOR |= (1u << 5);
        PORTA->ISFR = (1u << 4);
    }
}

// SysTick interrupt handler
void SysTick_Handler(void) {
    if (timer_running) {
        PTD->PTOR |= (1u << 5);
        hundredths_counter++;
    }
}

int main(void) {
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    __disable_irq();

    // Disable IRQs during configuration
    NVIC_DisableIRQ(PORTA_IRQn);
    NVIC_DisableIRQ(PORTC_PORTD_IRQn);

    // Activate clocks for PORTA, PORTD, PORTE, PORTB
    SIM->SCGC5 |= (1u << 9) | (1u << 12) | (1u << 13) | (1u << 10);

    // Configure pin mux for GPIO mode
    PORTD->PCR[3] = 0x100;
    PORTD->PCR[5] = 0x100;
    PORTA->PCR[4] = 0x100;

    // Configure PTB0 as analog input
    PORTB->PCR[0] = 0x000;

    // Configure GPIO direction
    PTD->PDDR &= ~(1u << 3);
    PTD->PDDR |= (1u << 5);
    PTA->PDDR &= ~(1u << 4);

    // Configure interrupt on falling edge for PTD3
    PORTD->PCR[3] &= ~0xF0000;
    PORTD->PCR[3] |= 0xA0000;

    // Configure interrupt on falling edge for PTA4
    PORTA->PCR[4] &= ~0xF0000;
    PORTA->PCR[4] |= 0xA0000;

    // Set priority for port interrupts
    NVIC_SetPriority(PORTC_PORTD_IRQn, 192);
    NVIC_SetPriority(PORTA_IRQn, 192);

    // Clear any pending port interrupts
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_ClearPendingIRQ(PORTA_IRQn);

    // Enable port interrupts in NVIC
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);

    // Configure ADC
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  // Enable ADC0 clock

    // Configure ADC for 12-bit mode with internal 3.3V reference
    ADC0->CFG1 = ADC_CFG1_MODE(1);
    ADC0->CFG2 = ADC_CFG2_ADHSC_MASK;
    ADC0->SC2 = ADC_SC2_REFSEL(1);


    SysTick->LOAD = 0x493E00;
    SysTick->VAL = 0;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);

    // Enable SysTick Timer in main() after all configuration
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    __enable_irq();


    while(1) {
        ADC0->SC1[0] = ADC_SC1_ADCH(8);


        while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));


        uint16_t adc_value = ADC0->R[0] & 0xFFF;
        PRINTF("ADC Value: %d\r\n", adc_value);
    }

    return 0;
}
