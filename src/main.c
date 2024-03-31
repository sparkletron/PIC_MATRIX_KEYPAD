/*
 * File:   main.c
 * Author: John Convertino
 *
 * Created on January 29, 2016, 5:15 PM
 * 
    Copyright (C) 2016 John Convertino

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 

 */

/*******************************************************************************
 * @file    main.c
 * @author  Jay Convertino(electrobs@gmail.com)
 * @date    2014.11.06
 * @brief   Matrix Keypad for PIC uC
 *
 * CONTROLLER: PIC16F648A
 *
 * Matrix Keypad 0 to 15 press, update only when pressed for 4 x 4 matrix.
 * Button press will signal an interrupt, ack will clear interrupt.
 * Uses internal oscillator, and pin change interrupt on port b 7:4.
 *
 * RA0 to 3 = Keypad value
 * RA4      = Ack input
 * RA5      = unused
 * RA6      = unused
 * RA7      = Interrupt Output
 *
 * @license mit
 *
 * Copyright 2024 Johnathan Convertino
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#include <xc.h>
#include <stdint.h>
#include <pic16f648a.h>

//other defines, debounce time is in milliseconds
#define DEBOUNCE_TIME   50
#define ACK_BIT         4
#define IRQ_BIT         7
#define FLG_OFF         0
#define FLG_ON          1
#define ENA_IRQ         1
//port state decoding normally open switch
#define ROW0_COL0       0b11101110
#define ROW0_COL1       0b11101101
#define ROW0_COL2       0b11101011
#define ROW0_COL3       0b11100111
#define ROW1_COL0       0b11011110
#define ROW1_COL1       0b11011101
#define ROW1_COL2       0b11011011
#define ROW1_COL3       0b11010111
#define ROW2_COL0       0b10111110
#define ROW2_COL1       0b10111101
#define ROW2_COL2       0b10111011
#define ROW2_COL3       0b10110111
#define ROW3_COL0       0b01111110
#define ROW3_COL1       0b01111101
#define ROW3_COL2       0b01111011
#define ROW3_COL3       0b01110111
//port state value
#define V_ROW0_COL0     0  
#define V_ROW0_COL1     1 
#define V_ROW0_COL2     2 
#define V_ROW0_COL3     3 
#define V_ROW1_COL0     4  
#define V_ROW1_COL1     5  
#define V_ROW1_COL2     6  
#define V_ROW1_COL3     7  
#define V_ROW2_COL0     8  
#define V_ROW2_COL1     9  
#define V_ROW2_COL2     10  
#define V_ROW2_COL3     11  
#define V_ROW3_COL0     12  
#define V_ROW3_COL1     13  
#define V_ROW3_COL2     14  
#define V_ROW3_COL3     15  
//pin redefines
#define PIN_CHANGE_ENA  INTCONbits.RBIE
#define PIN_CHANGE_FLG  INTCONbits.RBIF
//io direction
#define TOP_ROW_IN   0xF0
#define BOT_COL_IN   0x0F
#define TOP_ROW_MSK  0xF0
#define BOT_COL_MSK  0x0F
//macros
#define __irq_bit_off   ~(1 << IRQ_BIT)
#define __irq_bit_on     (1 << IRQ_BIT)
//setup to XTAL freq, for _delay macros
#define _XTAL_FREQ      4000000

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

//needed since chip is read-modify write only, and read in the same line doesn't work
uint8_t g_portaBuffer = 0;

void main(void) 
{
    //Sets int osc to 4 MHz
    PCONbits.OSCF = 1;
    
    //set each comparator for digital i/o on port a bits 0 to 3
    CMCONbits.CM0 = 1;
    CMCONbits.CM1 = 1;
    CMCONbits.CM2 = 1;
    
    //setup pullups in options
    //OPTION_REGbits.nRBPU = 0;
    
    //Port A set all, but bits 4, 5, and 6, to output
    TRISA = 0x70;
    
    //Port B set to inputs on top bits, outputs on bottom.
    TRISB = TOP_ROW_IN;
    
    //PORT A set to 0
    PORTA = 0;
    
    //PORT B set to 0
    PORTB = 0;
    
    //enable Port B 7:4 interrupt
    PIN_CHANGE_ENA = ENA_IRQ;
    
    //enable General Interrupt
    ei();
    
    for(;;)
    {
        if(PORTA & (1 << ACK_BIT))
        {
            g_portaBuffer &= __irq_bit_off;
            PORTA = g_portaBuffer;
        }
    }
}

//handles all interrupts, but is setup for portb pin interrupt only
void interrupt PORTB_Handler(void)
{
    uint8_t prev_portaBuffer = 0;
    //check for correct interrupt, is it enabled, and is the flag correct?
    if(PIN_CHANGE_ENA && PIN_CHANGE_FLG)
    {
        __delay_ms(DEBOUNCE_TIME);
        //only check for button if a column bit is high(actually low, ~ = cleaner logic) and IRQ bit is low
        if((~PORTB & 0xF0) && (~g_portaBuffer & (1 << IRQ_BIT)))
        {
            //buffer for portb
            uint8_t tempPortb = 0;
            
            //check top nibble (row)
            tempPortb = TOP_ROW_MSK & PORTB;
            
            //switch row to output, column to input
            TRISB = BOT_COL_IN;
            
            //set outputs to 0
            PORTB = 0;
            
            //get bottom nibble (col)
            tempPortb |= BOT_COL_MSK & PORTB;
            
            //reset to top nibble (row) as in
            TRISB = TOP_ROW_IN;
            
            //set outputs to 0
            PORTB = 0;
            
            prev_portaBuffer = g_portaBuffer;
            
            //decode byte of information to a number 0 to 15
            switch(tempPortb)
            {
                case ROW0_COL0:
                    g_portaBuffer = V_ROW0_COL0;
                    break;
                case ROW0_COL1:
                    g_portaBuffer = V_ROW0_COL1;
                    break;
                case ROW0_COL2:
                    g_portaBuffer = V_ROW0_COL2;
                    break;
                case ROW0_COL3:
                    g_portaBuffer = V_ROW0_COL3;
                    break;
                case ROW1_COL0:
                    g_portaBuffer = V_ROW1_COL0;
                    break;
                case ROW1_COL1:
                    g_portaBuffer = V_ROW1_COL1;
                    break;
                case ROW1_COL2:
                    g_portaBuffer = V_ROW1_COL2;
                    break;
                case ROW1_COL3:
                    g_portaBuffer = V_ROW1_COL3;
                    break;
                case ROW2_COL0:
                    g_portaBuffer = V_ROW2_COL0;
                    break;
                case ROW2_COL1:
                    g_portaBuffer = V_ROW2_COL1;
                    break;
                case ROW2_COL2:
                    g_portaBuffer = V_ROW2_COL2;
                    break;
                case ROW2_COL3:
                    g_portaBuffer = V_ROW2_COL3;
                    break;
                case ROW3_COL0:
                    g_portaBuffer = V_ROW3_COL0;
                    break;
                case ROW3_COL1:
                    g_portaBuffer = V_ROW3_COL1;
                    break;
                case ROW3_COL2:
                    g_portaBuffer = V_ROW3_COL2;
                    break;
                case ROW3_COL3:
                    g_portaBuffer = V_ROW3_COL3;
                    break;
                default:
                    //if an invalid press is made, just output last press again.
                    g_portaBuffer = prev_portaBuffer;
                    break;
            }
            //turn on irq output bit
            g_portaBuffer |= __irq_bit_on;
            
            //pass buffer to output port a
            PORTA = g_portaBuffer;
        }
        //clear irq flag
        PIN_CHANGE_FLG = FLG_OFF;
    }
}
