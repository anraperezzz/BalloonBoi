// Main.c
// TM4C123
// Angel Perez
// September 12, 2020


#include "stdint.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "UART.h"
#include "SysTick.h"

// 2. Declarations Section
//   Global Variables
uint8_t i = 0;
int command = 0;
int height = 0;
int left = 0;
int right = 0;
// char rxChar[4];//for PC application,uncomment this
unsigned int go;
unsigned int per;
unsigned int duty;


void PWM_Init(uint16_t period, uint16_t duty); 
void PWM_PF23_Duty(uint16_t duty); 
void PWM_PF1_Duty(uint16_t duty);
void PWM_PA6_Duty(uint16_t duty);

void PORTB_Init(void);
void EnableInterrupts(void);
unsigned long to_number(char string[4]); 

void UART2_Handler(){//this interrupt routine is for receiving data from bluetooth
		command = UART2_DR_R;
		height = (command & 0x00C0) >> 6;
		left =   (command & 0x0038) >> 3;
		right =  command & 0x0007;
    /* rxChar[i] = UART2_DR_R;
		UART3_OutChar(rxChar[i]);
		i++;
		if(rxChar[i-1]==13){	// If carriage return
			rxChar[i-1]='\0';
			i=0;
			go = 1;
		}
		*/
		/*
		if(rxChar[i-1]==42){	// If *
			rxChar[i-1]='\0';
			i=0;
			go = 2;
		}
		*/
		go = 1;
		UART2_ICR_R=UART_ICR_RXIC;//clear interrupt
}

void PORTB_Init(void){ 
  SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
  GPIO_PORTB_CR_R |= 0xFF;           // allow changes to PB7-0      
  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R = 0xFF  ;          // 5) PB7-PB0 output 
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTB_DEN_R |= 0xFF;          // 7) enable digital pins P7-PB0   
}

void SPI_Setup(void){
    //Sets up SPI0  PA2 PA3 PA4 PA5
    //              Clk Fss Rx  Tx
    SYSCTL_RCGCSSI_R    |=  0x01;   //Enable SSI Pg 346
    SYSCTL_RCGCGPIO_R   |=  0x01;   //Enable Clock Pg 340
    while((SYSCTL_PRGPIO_R&0x01) == 0){};   // ready?
    GPIO_PORTA_AFSEL_R  |=  0x3C;   //Enable AF    Pg 671
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0000FF)+0x00222200;//PortCntrl Pg 688
    GPIO_PORTA_DEN_R    |=  0x3C;   //Enable Dig   Pg 649
    GPIO_PORTA_AMSEL_R = 0;         // disable analog functionality on PA
    SSI0_CR1_R          &=  ~0x00000002;    //Disable SSE
    SSI0_CR1_R          =   0x00000000;     //MASTER
    SSI0_CC_R           |=  0x00;   //CLK SOURCE
    SSI0_CPSR_R         |=  0x50;   //CLK Divider 1MHz
    SSI0_CR0_R          |=  0x0000; //No serial Clock rate
    SSI0_CR0_R          |=  0x0000; //CLK, Polarity, phase
    SSI0_CR0_R          |=  0x0000; //Freescale mode
    SSI0_CR0_R          |=  0x000F; //Data Size
    SSI0_CR1_R          |=  0x00000002; //Enable SSI
}


uint16_t Com_SPI(uint16_t data){    uint16_t receive;
    while((SSI0_SR_R&0x2)==0){};    //While tx full
    SSI0_DR_R           =   data;
    while((SSI0_SR_R&0x4)==0){};// while rx empty
    receive = SSI0_DR_R;    // acknowledge response
    return receive;
}

void PWM_Init(uint16_t period, uint16_t duty){volatile unsigned long delay;
	// This function enables PF 1, 2, 3 and PA6. NOTE: this function assumes that the clock is 80MHz
	// Generator 1, Generator 2, Generator 3
	// PA6(L)			  PF1(R)			 PF2,3(UP)
	
	SYSCTL_RCGCPWM_R  |= 0x02;			// Enable and provide a clock to PWM module 1 in Run mode
	
	SYSCTL_RCGCGPIO_R |= 0x21;		  // Enable and provide a clock to GPIO Port F and A in Run mode

	delay = SYSCTL_RCGCGPIO_R;   		// Wait
	
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0;  	// Turn off pins  PF1, PF2, PF3
	GPIO_PORTF_PCTL_R|= 0x00005550;	// Turn on M1 PWM5, PWM6, PWM7
	GPIO_PORTF_DEN_R |= 0x0E;				// Pin digital enable 1,2,3
	GPIO_PORTF_AFSEL_R |= 0x0E;			// Alternate function select 1,2,3
	GPIO_PORTF_AMSEL_R &= ~0x0E;		// Disable Analog mode 1,2,3
	
	GPIO_PORTA_PCTL_R &= ~0x0F00000;  	// Turn off pins  PA6
	GPIO_PORTA_PCTL_R|= 0x05000000;	// Turn on M1 PWM2
	GPIO_PORTA_DEN_R |= 0x40;				// Pin digital enable 6
	GPIO_PORTA_AFSEL_R |= 0x40;			// Alternate function select 6
	GPIO_PORTA_AMSEL_R &= ~0x40;		// Disable Analog mode 6
	
	PWM1_3_GENA_R |= 0x0042;					// When load start High PF2,3
	PWM1_3_GENB_R |= 0x0402;					// When load start High PF2,3
	
	PWM1_2_GENB_R |= 0x0402;					// When load start High PF1
	
	PWM1_1_GENA_R |= 0x0042;					// When load start High PA6
	
	
	PWM1_3_LOAD_R = period - 1;			// Load value (20KHz) PF2,3
	
	PWM1_2_LOAD_R = period - 1;			// Load value (20KHz) PF1
	
	PWM1_1_LOAD_R = period - 1;			// Load value (20KHz) PA6
	
	
	PWM1_3_CMPA_R = duty - 1;				// Duty PF2
	PWM1_3_CMPB_R = duty - 1;				// Duty PF3
	
	PWM1_2_CMPB_R = duty - 1;				// Duty PF1
	
	PWM1_1_CMPA_R = duty - 1;				// Duty PA6
	
	
	PWM1_3_CTL_R |= 0x00000001;			// Start Timer PF2,3
	
	PWM1_2_CTL_R |= 0x00000001;			// Start Timer PF1
	
	PWM1_1_CTL_R |= 0x00000001;			// Start Timer PA6 count up
	
	PWM1_ENABLE_R |= 0xE4;					// Enable Output from module 1
}

void PWM_PF23_Duty(uint16_t duty){
	PWM1_3_CMPA_R = duty - 1;				// Duty PF2
	PWM1_3_CMPB_R = duty - 1;				// Duty PF3 
}

void PWM_PF1_Duty(uint16_t duty){
	PWM1_2_CMPB_R = duty - 1;				// Duty PF1
}

void PWM_PA6_Duty(uint16_t duty){
	PWM1_1_CMPA_R = duty - 1;				// Duty PA6
}
int main(void){ int delay;
	int receive;
	//char string[5]; 
	go = 0;
	//unsigned long pwm_value, last_pwm_value, frequency;
	PLL_Init();
	UART2_Init();
	UART3_Init();
	//SPI_Setup();
	UART3_OutString("Input Command:\n\r");
	per = 0x0FA0; // fa0 20KHz
	duty = 0x0000;	// 7d0 50%
	PWM_Init( per, duty);
  while(1){
		
		if (go == 1){
			//NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
			//duty = (to_number(rxChar) * ((per-1))) / (255);
			//UART3_OutString(rxChar);
			/*
			UART3_OutUDec(height);
			UART3_OutString("\n\r");
			UART3_OutUDec(left);
			UART3_OutString("\n\r");
			UART3_OutUDec(right);
			UART3_OutString("\n\r");
			*/
			height = (height * 0x0F9F) / 3;
			left = (left * 0x0F9F) / 7;
			right = (right * 0x0F9F) / 7;
			/*
			UART3_OutUDec(command);
			UART3_OutString("\n\r");
			UART3_OutUDec(height);
			UART3_OutString("\n\r");
			UART3_OutUDec(left);
			UART3_OutString("\n\r");
			UART3_OutUDec(right);
			*/
			PWM_PF23_Duty(height);
			PWM_PA6_Duty(left);
			PWM_PF1_Duty(right);
			//UART3_OutUDec(duty);
			//UART3_OutString("--->");
			//UART3_OutUDec();
			
			UART3_OutString("\n\rInput Command:\n\r");
			go = 0;
		}
		/*
		if (go == 2){
			receive = Com_SPI(0xC100); // 1100 0101 0000 0000
			receive &= 0x00FF;//0x00FF;
			UART3_OutString("SPI--->");
			UART3_OutUDec(receive);
			UART3_OutString("\n\rInput Command:\n\r");
			go = 0;
		}
		*/
		//receive = Com_SPI(0xC100); // 1100 0011 0000 0000
		//receive &= 0x00FF;
		//UART3_OutUDec(receive);UART3_OutString("\r\n");
		/*
		//Find out if first letter is f or b 
		UART3_OutString("Input Value:"); 
		UART2_InString(string, 5);  OutCRLF3(); 
		
		if (string[0] == 'f'){
			frequency = to_number(string);
			UART3_OutUDec(frequency); OutCRLF2();
		}
		
		else{
			pwm_value = to_number(string);
			if(pwm_value > 255){
				UART3_OutString("b out of range. Try Again from 0 - 255:\r\n");
				pwm_value = last_pwm_value;
			}
			else if(pwm_value==0){
				pwm_value = 0;
				last_pwm_value = pwm_value;
			}
			else{
				pwm_value = (int)((((float)pwm_value/255)*38000)+ 1000 - 3);
				last_pwm_value = pwm_value;
			}
			PWM_PF2_Duty(pwm_value);  
		}
		*/
	}
}

unsigned long to_number(char string[4]){
	int i;  
	int result = 0;
	int dig_pos = 1; 
	
	for (i=2;i>-1;i--){
		if (string[i] != CR && string[i] != LF && string[i]!= 0){
			result += dig_pos * (string[i]-'0');   
			dig_pos *= 10; 
		}
	}
	return result; 
}

