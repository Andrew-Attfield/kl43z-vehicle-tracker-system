#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"

#define SYSTEM_CORE_CLOCK 48000000u //48 MHz-------
#define ADC_MIN 372 //ADC lower bound----------
#define ADC_MAX 4095 //ADC upper bound---------
#define SPEED_MIN 20 //speed lower bound----------
#define SPEED_MAX 100 //speed upper bound-----------

#define GREEN_LED_PIN (1u <<5) //green led-------------
#define RED_LED_PIN (1u << 31) //red led-----------

volatile uint32_t hundredths_counter = 0;
volatile uint8_t timer_running = 0;
volatile uint16_t current_speed_limit = SPEED_MIN; //map from ADC MIN------
volatile uint32_t vehicle_id = 0; //vehicle ID------------

// Interrupt handler for PORTD
void PORTC_PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1u << 3)) {

        timer_running = 1;

        SysTick->VAL = 0; //reset systick current value;

        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

        hundreths_counter = 0; //reset hundreths counter

        PTE->PSOR = (1u << 31); //turn off red LED----

        PTD->PTOR |= (1u << 5);
        PORTD->ISFR = (1u << 3);
    }
}

// Interrupt handler for PORTA
void PORTA_IRQHandler(void) {
    if (PORTA->ISFR & (1u << 4)) {


        timer_running = 0; //stop the timer
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

        float elapsed_time = hundreths_counter * 0.1f + ((float)(SysTick -> VAL) / SysTick->LOAD) * 0.1f; //elasped time equation

        float vehicle_speed = 10.0f/elapsed_time; //m/s-------------

        vehicle_speed = vehicle_speed * 3.6f; //km/h conversion------------

        uint16_t speed = (uint16_t)(vehicle_speed + 0.5f); //rounding-------------

        if (speed > current_speed_limit) {
        PTE->PCOR = (1u << 31); //red led on if speeding------------

        }

        else {
        PTE->PSOR = (1u << 31); //red led off if not speeding--------------

        }

        hundredths_counter = 0; //reset counter


        PTD->PTOR |= (1u << 5); //toggle green led once

        PORTA->ISFR = (1u << 4); //clear interrupt

    }
}

// SysTick interrupt handler
void SysTick_Handler(void) {
    if (timer_running) {
        PTD->PTOR |= (1u << 5);
        hundredths_counter++;
    }
}

uint16_t speedLimit(uint16_t adc) {
if (adc < 372) {
adc = 372; //cap adc lower------
}
if (adc > 4095) {
adc = 4095; //cap adc higher------
}

float speedLimit = ((SPEED_MAX - SPEED_MIN)/(ADC_MAX - ADC_MIN)) * adc + 12.0f; //speed limit equation------

if (speedLimit < SPEED_MIN) { //set min----------
speedLimit = SPEED_MIN;
}

if (speedLimit > SPEED_MAX) { //set max-------------
speedLimit = SPEED_MAX;

}

return (uint16_t)(speedLimit + 0.5f); //return speedLimit cast as int16, add 0.5 to round properly----

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
    PORTE->PCR[31] = 0x100; // configure red LED ---------

    // Configure PTB0 as analog input
    PORTB->PCR[0] = 0x000;

    // Configure GPIO direction
    PTD->PDDR &= ~(1u << 3);
    PTD->PDDR |= (1u << 5);
    PTA->PDDR &= ~(1u << 4);
    PTE->PDDR |= (1u << 31); //configure GPIO red LED-------

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

    ADC0->SC1[0] = ADC_SC1_ADCH(8); //potentiometer PTB0 -> ADC8-------------


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
        current_speed_limit = speedLimit(adc_value); //maps ADC to speed limit and store to global-------

        PRINTF("ADC: %d, SpeedLimit: %d\r\n", adc_value, current_speed_limit); //prints speed limit and adc-------

    }

    return 0;
}
