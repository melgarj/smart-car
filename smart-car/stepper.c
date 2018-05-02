// Stepper.c
// Runs on LM4F120/TM4C123
// Provide functions that step the motor once clockwise, step
// once counterclockwise, and initialize the stepper motor
// interface.
// Daniel Valvano
// September 12, 2013
// Modified by Dr. Min He April 28, 2017

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// PD3 connected to driver for stepper motor coil A
// PD2 connected to driver for stepper motor coil A'
// PD1 connected to driver for stepper motor coil B
// PD0 connected to driver for stepper motor coil B'

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "systick.h"
struct State{
  uint8_t Out;     // Output
  uint8_t Next[2]; // CW/CCW
};
typedef const struct State StateType;

#define clockwise 0        // Next index
#define counterclockwise 1 // Next index
StateType fsm[4]={
  {12,{1,3}},
  { 6,{2,0}},
  { 3,{3,1}},
  { 1,{0,2}}
};

StateType fsm2[4]={
  {192,{1,3}},
  { 96,{2,0}},
  { 48,{3,1}},
  { 16,{0,2}}
};
unsigned char s; // current state
unsigned char r;

#define STEPPERL  (*((volatile uint32_t *)0x4000703C))
#define STEPPERR  (*((volatile uint32_t *)0x400063C0))
	
// Move 1.8 degrees clockwise, delay is the time to wait after each step
void Stepper_CW(uint32_t delay){
  s = fsm[s].Next[clockwise]; // clock wise circular
  STEPPERL = fsm[s].Out; // step motor
  //SysTick_Wait(delay);
}
// Move 1.8 degrees counterclockwise, delay is wait after each step
void Stepper_CCW(uint32_t delay){
  s = fsm[s].Next[counterclockwise]; // counter clock wise circular
  STEPPERL = fsm[s].Out; // step motor
  //SysTick_Wait(delay); // blind-cycle wait
}

void StepperR_CW(){
	r = fsm2[r].Next[clockwise];
	STEPPERR = fsm2[r].Out;
}

void StepperR_CCW(){
	r = fsm2[r].Next[counterclockwise];
	STEPPERR = fsm2[r].Out;
}

// Initialize Stepper interface
void Stepper_Init(unsigned long period){
  //SYSCTL_RCGCGPIO_R |= 0x0C; // 1) activate port D & C
	SYSCTL_RCGC2_R |= 0x0000000C;
  SysTick_Init(period);
  s = 0; 
	r = 0;
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= 0x0F;   // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTD_DEN_R |= 0x0F;   // 7) enable digital I/O on PD3-0 
	
	GPIO_PORTC_AMSEL_R &= ~0xF0;      // 3) disable analog functionality on PC7-4
  GPIO_PORTC_PCTL_R &= ~0xFFFF0000; // 4) GPIO configure PC7-4 as GPIO
  GPIO_PORTC_DIR_R |= 0xF0;   // 5) make PC7-4 out
  GPIO_PORTC_AFSEL_R &= ~0xF0;// 6) disable alt funct on PC7-4
  GPIO_PORTC_DR8R_R |= 0xF0;  // enable 8 mA drive
  GPIO_PORTC_DEN_R |= 0xF0;   // 7) enable digital I/O on PC7-4
}
