
/*
// PWM BLUE LED  PF.2
#include "PWM.h"
#include "../tm4c123gh6pm.h"

//void PWM_Init(uint16_t period, uint16_t duty); 
//void PWM_PF2_Duty(uint16_t duty); 

void PWM_Init(uint16_t period, uint16_t duty){
	volatile unsigned long delay; 
	SYSCTL_RCGCPWM_R  |= 0x02;			
	SYSCTL_RCGCGPIO_R |= 0x20;		  
	delay = SYSCTL_RCGCGPIO_R;   			
	GPIO_PORTF_PCTL_R &= ~0x00000F00;  
	GPIO_PORTF_PCTL_R|= 0x00000500;
	GPIO_PORTF_DEN_R |= 0x04;	
	GPIO_PORTF_AFSEL_R |= 0x04;
	GPIO_PORTF_AMSEL_R &= ~0x04;					
	SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
	SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
	SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;
	PWM1_3_CTL_R = 0; 							
	PWM1_3_GENA_R = 0xC8;						
	PWM1_3_LOAD_R = period - 1;			 
	PWM1_3_CMPA_R = duty - 1;
	PWM1_3_CTL_R |= 0x00000001;			
	PWM1_ENABLE_R |= 0x40;					
}


void PWM_PF2_Duty(uint16_t duty){
	PWM1_3_CMPA_R = duty - 1; 
}
*/
