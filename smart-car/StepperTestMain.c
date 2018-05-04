// StepperTestMain.c
// Runs on LM4F120/TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Min HE

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
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
#define LIGHT										(*((volatile unsigned long *)0x40025038))
#include <stdint.h>
#include "stepper.h"
#include "tm4c123gh6pm.h"
#define T1ms 16000    // assumes using 16 MHz PIOSC (default setting for clock source)

void PortF_Init(void);
void PortA_Init(void);
void EnableInterrupts(void);

//unsigned int presence = 0; // init at closed state
//unsigned int doorOpen = 0; // activates when door is fully opened
//unsigned int activate = 1;
//unsigned int ledCounter = 0;

unsigned int rRotations = 0;
unsigned int lRotations = 0;
unsigned int rRotation = 0;
unsigned int lRotation = 0;
unsigned int forward = 0;
unsigned int backward = 0;

int main(void){
	unsigned int i=0;
	PortF_Init();
	PortA_Init();
  Stepper_Init(160000); // 10 ms for stepper, *50 = 500 ms for LED flash
	EnableInterrupts();
	//LIGHT = 0x08;
	
	
	
  while(1){
		
		if(rRotation == 1 && lRotation == 1){StepperR_CW(); Stepper_CW(0);lRotations += 1; rRotations += 1;}
		else if (rRotation == 0 && lRotation == 1){
			Stepper_CW(0); lRotations += 1;
			if (lRotations == 4500){rRotations = 0; lRotations = 0;}
		}
		
  }
}


void PortF_Init(void){volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  // Unlock at beginning, broke code 
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0 
  //GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4 and PF0 in (built-in button)
	GPIO_PORTF_DIR_R = 0x0E;          // 5)PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4  
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void PortA_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000001;
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTA_DIR_R &= ~0x80; // Input, PA7
	GPIO_PORTA_AFSEL_R &= ~0xFF;
	GPIO_PORTA_DEN_R |= 0x80;
	GPIO_PORTA_PCTL_R &= ~0xF0000000;
	GPIO_PORTA_AMSEL_R = 0;
	GPIO_PORTA_IS_R &= ~0x80;
	GPIO_PORTA_IBE_R |= 0x80; // Both edges
	GPIO_PORTA_ICR_R = 0x80;  
	GPIO_PORTA_IM_R |= 0x80;
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF00) | 0x00000080; //priority 4
	NVIC_EN0_R |= 0x00000001;
	
}	

void GPIOPortF_Handler(void){
	GPIO_PORTF_ICR_R = 0x11;
	
	if(GPIO_PORTF_RIS_R & 0x01){forward = 1;}
	if(GPIO_PORTF_RIS_R & 0x10){backward = 1;}

}

void SysTick_Handler(void){
	
	if(forward == 1 && rRotations < 4000 && lRotations < 4000){rRotation = 1; lRotation = 1;}
	else if (forward == 1 && rRotations >= 4000 && lRotations >= 4000){rRotation = 0; rRotations = 0;}
	else{rRotation = 0; lRotation = 0;}

}

void GPIOPortA_Handler(void){
	GPIO_PORTA_ICR_R = 0x80; // acknowledge
 
}

