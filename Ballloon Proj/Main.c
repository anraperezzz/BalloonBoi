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
int state = 0;

// char rxChar[4];//for PC application,uncomment this
char final_message[5]; 
char data_packet[5];
unsigned int go;
unsigned int per;
unsigned int duty;

uint16_t com_SPI0(uint16_t address, uint16_t data, uint16_t code);
void UART2_Handler();
void PWM_Init(uint16_t period, uint16_t duty); 
void SPI0_Setup(void);
void MPU_9250_Init();
void PWM_PF23_Duty(uint16_t duty); 
void PWM_PF1_Duty(uint16_t duty);
void PWM_PA6_Duty(uint16_t duty);

void PORTB_Init(void);
void EnableInterrupts(void);
unsigned long to_number(char string[4]); 





int main(void){ int delay;
	uint16_t spi;
	go = 0;
	PLL_Init();
	UART2_Init();
	UART3_Init();
	SPI0_Setup();
	MPU_9250_Init();
	SysTick_Init(8000000);
	per = 0x0FA0; // fa0 20KHz
	duty = 0x0000;	// 7d0 50%
	PWM_Init( per, duty);
  while(1){



		if (go == 1){
			UART3_OutString(final_message);
			go = 0;
			//height = (height * 0x0F9F) / 3;
			//left = (left * 0x0F9F) / 7;
			//right = (right * 0x0F9F) / 7;			
		}
		/*
		if (go == 2){
			go = 0;
		}
		*/
		//receive = read_SPI0(0xC100); // 1100 0011 0000 0000
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

void UART2_Handler(){
		NVIC_ST_CTRL_R &=  ~NVIC_ST_CTRL_ENABLE;//Disable systick
		NVIC_ST_CURRENT_R = 0; //Writing clears count register
		data_packet[state] = UART2_DR_R; //read from uart2 to register 
		switch (state){
			case 0:
			{
				if (data_packet[state] == 0x24){ 
					state++;
					NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; //Enable systick
				}
				else{state = 0;} // Start Over
				break;
			}
			case 4: 
			{
				char checksum = (char)(data_packet[0] + data_packet[1] + data_packet[2] + data_packet[3]);
				if (checksum == data_packet[4]){					
					final_message[0] = data_packet[0];
					final_message[1] = data_packet[1];
					final_message[2] = data_packet[2];
					final_message[3] = data_packet[3];
					final_message[4] = data_packet[4];
					go = 1;
				}
				state = 0;
				data_packet[0] = 0;
				data_packet[1] = 0;
				data_packet[2] = 0;
				data_packet[3] = 0;
				data_packet[4] = 0;
					
				break;
			}
			default:
			{
				if ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) != NVIC_ST_CTRL_COUNT){
					state++;
					NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; //Enable systick
				}
				else {state = 0;}
			}
	}	
		UART2_ICR_R = UART_ICR_RXIC;//clear interrupt
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

void SPI0_Setup(void){
    //Sets up SPI0  PA2 PA3 PA4 PA5
    //              Clk Fss Rx  Tx
    SYSCTL_RCGCSSI_R    |=  0x01;   // Activate SSI0
    SYSCTL_RCGCGPIO_R   |=  0x01;   // Activate port A
    while((SYSCTL_PRGPIO_R&0x01) == 0){};   // Wait for clock
			
    GPIO_PORTA_AFSEL_R  |=  0x3C;   //Enable ALT PA2, 3, 4, 5
		GPIO_PORTA_DEN_R    |=  0x3C;   //Enable Dig  PA2, 3, 4, 5
																		// configure PA2, 3, 4, 5 as SSI
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0000FF)+0x00222200;
    GPIO_PORTA_AMSEL_R &= ~0x3C;    // disable AF on PA2, 3, 4, 5
			
    SSI0_CR1_R          &=  ~SSI_CR1_SSE;    //Disable SSE
    SSI0_CR1_R          &=  ~SSI_CR1_MS;     //MASTER
												// configure for system clock/PLL baud clock source
    SSI0_CC_R            = (SSI0_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_SYSPLL;
    SSI0_CPSR_R         |=  0x50;   //CLK Divider 1MHz
		SSI0_CR0_R &= ~(SSI_CR0_SCR_M |       // SCR = 0 (8 Mbps data rate)
										SSI_CR0_SPH |         // SPH = 0
										SSI_CR0_SPO);         // SPO = 0
																					// FRF = Freescale format
		SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;
																					// DSS = 16-bit data
		SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_16;
    SSI0_CR1_R          |=  SSI_CR1_SSE; 	//Enable SSI
}


uint16_t com_SPI0(uint16_t address, uint16_t data, uint16_t code){uint16_t receive;
    while((SSI0_SR_R&SSI_SR_TNF)==0){};      // Wait for Tx
    SSI0_DR_R = ((address | code ) << 8) | data;// Data Out
    while((SSI0_SR_R&SSI_SR_RNE)==0){};			// While rx is not full
    receive = SSI0_DR_R;    					// Data in
		receive &= 0x00FF;
    return receive;
}

void MPU_9250_Init(){uint16_t rx;
	com_SPI0(0x6A, 0x10, 0x00);	//Enable SPI only
	com_SPI0(0x1D, 0x06, 0x00);	//Set accelerometers low pass filter at 5Hz
	com_SPI0(0x1A, 0x06, 0x00);	//Set gyroscope low pass filter at 5Hz
	
	com_SPI0(0x1B, 0x10, 0x00);	// Configure gyroscope range 1000dps
	com_SPI0(0x1C, 0x08, 0x00); // Configure accelerometers range 4G
	
	//rx = com_SPI0(0x6A, 0x00, 0x80);	// Example of reading
	//UART3_OutString("Test:\n\r");
}

void PWM_Init(uint16_t period, uint16_t duty){volatile unsigned long delay;
	// This function enables PF 1, 2, 3 and PA6. NOTE: this function assumes that the clock is 80MHz
	// Generator 1, Generator 2, Generator 3
	// PA6(L)			  PF1(R)			 PF2,3(UP)
	SYSCTL_RCGCPWM_R  |= 0x02;			// Enable and provide a clock to PWM module 1 in Run mode
	SYSCTL_RCGCGPIO_R |= 0x21;		  // Enable and provide a clock to GPIO Port F and A in Run mode

	while((SYSCTL_PRGPIO_R&0x21) == 0){};   // Wait for clock
	
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0;  	// Turn off pins  PF1, PF2, PF3
	GPIO_PORTF_PCTL_R|= 0x00005550;	// Turn on M1 PWM5, PWM6, PWM7
	GPIO_PORTF_DEN_R |= 0x0E;				// Pin digital enable 1,2,3
	GPIO_PORTF_AFSEL_R |= 0x0E;			// Alternate function select 1,2,3
	GPIO_PORTF_AMSEL_R &= ~0x0E;		// Disable Analog mode 1,2,3
	
	GPIO_PORTA_PCTL_R&=~0x0F000000;  	// Turn off pins  PA6
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

