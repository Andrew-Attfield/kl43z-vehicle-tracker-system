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
 * @file    MKL43Z4_Project_Lab 6.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"

// Interrupt handler for PORTD
void PORTC_PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1u << 3)) {
        PTD->PTOR |= (1u << 5);
        PORTD->ISFR = (1u << 3);
    }
}

// Interrupt handler for PORTA
void PORTA_IRQHandler(void) {
    if (PORTA->ISFR & (1u << 4)) {
        PTE->PTOR |= (1u << 31);
        PORTA->ISFR = (1u << 4);
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

    // Activate clocks for PORTA, PORTD, PORTE
    SIM->SCGC5 |= (1u << 9) | (1u << 12) | (1u << 13);

    // Configure pin mux for GPIO mode
    PORTD->PCR[3] = 0x100;
    PORTD->PCR[5] = 0x100;
    PORTA->PCR[4] = 0x100;
    PORTE->PCR[31] = 0x100;

    // Configure GPIO direction
    PTD->PDDR &= ~(1u << 3);
    PTD->PDDR |= (1u << 5);
    PTA->PDDR &= ~(1u << 4);
    PTE->PDDR |= (1u << 31);

    // Configure interrupt on falling edge for PTD3
    PORTD->PCR[3] &= ~0xF0000;  // Clear interrupt config bits
    PORTD->PCR[3] |= 0xA0000;

    // Configure interrupt on falling edge for PTA4
    PORTA->PCR[4] &= ~0xF0000;
    PORTA->PCR[4] |= 0xA0000;

    // Set priority for interrupts
    NVIC_SetPriority(PORTC_PORTD_IRQn, 192);
    NVIC_SetPriority(PORTA_IRQn, 192);

    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_ClearPendingIRQ(PORTA_IRQn);

    // Enable interrupts in NVIC
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);

    __enable_irq();


    while(1) {
        __asm volatile ("nop");
    }

    return 0;
}
