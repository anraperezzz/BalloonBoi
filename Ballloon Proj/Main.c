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
#include "mpu9250.h"
#include "UART.h"
#include "SysTick.h"

// 2. Declarations Section
//   Global Variables
int state = 0;

// char rxChar[4];//for PC application,uncomment this
char final_message[5]; 
char data_packet[5];
bool receivedPacket;
unsigned int per;
unsigned int duty;

void Wait100msX(int rep);
uint16_t com_SPI0(uint16_t address, uint16_t data, uint16_t code);
void write_SPI0(uint16_t address, uint16_t data);
uint16_t read_SPI0(uint16_t address);
void read_Mult_SPI0(uint16_t address, uint8_t *ReadBuf, int bytes);
void UART2_Handler();
void PWM_Init(uint16_t period, uint16_t duty); 
void SPI0_Setup(void);
void MPU_9250_Init(void);
void PWM_PF23_Duty(unsigned int duty); 
void PWM_PF1_Duty(unsigned int duty);
void PWM_PA6_Duty(unsigned int duty);
void UART3TransmitPacket(char code, char spi1, char spi2);
void processPacket(void);
void PORTB_Init(void);
void EnableInterrupts(void);
unsigned long to_number(char string[4]); 





int main(void){ int delay;
	char spi;
	receivedPacket = false;
	PLL_Init();
	SysTick_Init(8000000);
	SPI0_Setup();
	MPU_9250_Init();
	UART2_Init();
	UART3_Init();
	per = 0x0FA0; // fa0 20KHz
	duty = 0x0000;	// 7d0 50%
	PWM_Init( per, duty);
  while(1){
		if (receivedPacket){processPacket();}
		
			//height = (height * 0x0F9F) / 3;
			//left = (left * 0x0F9F) / 7;
			//right = (right * 0x0F9F) / 7;		
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

void UART3TransmitPacket(char code, char spi1, char spi2){
	while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = 0x24;
	while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = code;
	while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = spi1;
	while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = spi2;
	while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = (char)(spi1 + spi2 + 0x24 + code);
}



void processPacket(void){
	if ((final_message[1] & 0x80) == 0x00){
		char spi1;
		char spi2;
		switch ((final_message[1] & 0x7F)){
			case 0x00: {
				//AccelX
				spi1 = com_SPI0(0X3B, 0x00, 0x80);
				spi2 = com_SPI0(0X3C, 0x00, 0x80);
				
				UART3TransmitPacket(0x80, spi2, spi1);
				//No Break!
			}
			case 0x01: {
				//AccelY
				spi1 = com_SPI0(0X3D, 0x00, 0x80);
				spi2 = com_SPI0(0X3E, 0x00, 0x80);
				UART3TransmitPacket(0x81, spi2, spi1);
				//No Break!
			}
			case 0x02: {
				//AccelZ
				spi1 = com_SPI0(0X3F, 0x00, 0x80);
				spi2 = com_SPI0(0X40, 0x00, 0x80);
				UART3TransmitPacket(0x82, spi2, spi1);
				break;
			}
			case 0x03: {
				//Temp
				spi1 = com_SPI0(0x41, 0x00, 0x80);
				spi2 = com_SPI0(0x42, 0x00, 0x80);
				UART3TransmitPacket(0x83, spi2, spi1);
				break;
			}
			case 0x04: {
				//GyroX
				spi1 = com_SPI0(0x43, 0x00, 0x80);
				spi2 = com_SPI0(0x44, 0x00, 0x80);
				UART3TransmitPacket(0x84, spi2, spi1);
				//No Break!
			}
			case 0x05: {
				//GyroY
				spi1 = com_SPI0(0x45, 0x00, 0x80);
				spi2 = com_SPI0(0x46, 0x00, 0x80);
				UART3TransmitPacket(0x85, spi2, spi1);
				//No Break!
			}
			case 0x06: {
				//GyroZ
				spi1 = com_SPI0(0x47, 0x00, 0x80);
				spi2 = com_SPI0(0x48, 0x00, 0x80);
				UART3TransmitPacket(0x86, spi2, spi1);
				break;
			}
			case 0x07: {
				//MagX
				write_SPI0(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
				write_SPI0(MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // I2C slave 0 register address from where to begin data transfer
				write_SPI0(MPUREG_I2C_SLV0_CTRL, 0x87);                      // Read 6 bytes from the magnetometer

				spi1 = com_SPI0(MPUREG_EXT_SENS_DATA_01, 0x00, 0x80);
				spi2 = com_SPI0(MPUREG_EXT_SENS_DATA_00, 0x00, 0x80);
				UART3TransmitPacket(0x87, spi1, spi2);
			}
			case 0x08: {
				//MagY
				spi1 = com_SPI0(MPUREG_EXT_SENS_DATA_03, 0x00, 0x80);
				spi2 = com_SPI0(MPUREG_EXT_SENS_DATA_02, 0x00, 0x80);
				UART3TransmitPacket(0x88, spi1, spi2);
			}
			case 0x09: {
				//MagZ
				spi1 = com_SPI0(MPUREG_EXT_SENS_DATA_05, 0x00, 0x80);
				spi2 = com_SPI0(MPUREG_EXT_SENS_DATA_04, 0x00, 0x80);
				read_SPI0(MPUREG_EXT_SENS_DATA_06);
				UART3TransmitPacket(0x89, spi1, spi2);
				break;
			}
			
			case 0x0A: {
				//Pressure to be added
				break;
			}
			case 0x0B: {
				//Motor Left/right
				PWM_PA6_Duty(final_message[2] << 8);
				PWM_PF1_Duty(final_message[3] << 8);
				//UART3TransmitPacket(0x69, final_message[2], final_message[3]);
				break;
			}
			case 0x0C: {
				//Motor Bottom
				//PWM_PF23_Duty
				//PWM_PF23_Duty((final_message[2] << 8) & final_message[3]);
				PWM_PF23_Duty((final_message[2] << 8) );
				break;
			}
		}
	}
	receivedPacket = false;
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
					receivedPacket = true;
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

void Wait100msX(int rep){
	int in;
	for (in = 0; in < rep; in++){
		NVIC_ST_CTRL_R &=  ~NVIC_ST_CTRL_ENABLE;	//Disable systick
		NVIC_ST_CURRENT_R = 0; 										//Writing clears count register
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; 		//Enable systick
		while((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) != NVIC_ST_CTRL_COUNT){};	//Wait for 100ms to pass
	}
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
		while((SSI0_SR_R&SSI_SR_TFE)==0){};      // Wait for Tx
    //while((SSI0_SR_R&SSI_SR_RNE)==0){};			// While rx is not full
    receive = SSI0_DR_R;    					// Data in
		//receive &= 0x00FF;
    return receive;
}

uint16_t read_SPI0(uint16_t address){uint16_t receive;
		// 0x80 is the reading code
    while((SSI0_SR_R&SSI_SR_TNF)==0){};      // Wait for Tx
    SSI0_DR_R = ((address | 0x80 ) << 8);		// Data Out register and reading bit
		while((SSI0_SR_R&SSI_SR_TFE)==0){};      // Wait for Tx
    receive = SSI0_DR_R;    					// Data in
    return receive;
}

void read_Mult_SPI0(uint16_t address, uint8_t *ReadBuf, int bytes){int addi;int bufi;
		// 0x80 is the reading code
		bufi = 0;
		for (addi = address; (addi < (address + bytes)); addi++){
			while((SSI0_SR_R&SSI_SR_TNF)==0){};      // Wait for Tx
			SSI0_DR_R = ((address | 0x80 ) << 8);		// Data Out register and reading bit
			while((SSI0_SR_R&SSI_SR_TFE)==0){};      // Wait for Tx
			ReadBuf[bufi] = (uint8_t) SSI0_DR_R;    					// Data in
			bufi++;
		}
    
}

void write_SPI0(uint16_t address, uint16_t data){
    while((SSI0_SR_R&SSI_SR_TNF)==0){};      // Wait for Tx
    SSI0_DR_R = (address << 8) | data;			 // Data Out
		while((SSI0_SR_R&SSI_SR_TFE)==0){};      // Wait for Tx
}

void MPU_9250_Init(void){uint16_t rx;
	write_SPI0(MPUREG_PWR_MGMT_1, BIT_H_RESET);	// // Reset Device
	Wait100msX(1);
	write_SPI0(MPUREG_PWR_MGMT_1, 0x01);	// Clock Source
	write_SPI0(MPUREG_PWR_MGMT_2, 0x00);	// Enable Acc & Gyro
	Wait100msX(2);
	
	write_SPI0(MPUREG_USER_CTRL, 0x30);	// I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
	write_SPI0(MPUREG_I2C_MST_CTRL, 0x8D);	// I2C configuration multi-master  IIC 400KHz
	
	write_SPI0(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);	// +-250dps
	write_SPI0(MPUREG_ACCEL_CONFIG, BITS_FS_2G);	// +-2G
	write_SPI0(MPUREG_CONFIG, BITS_DLPF_CFG_188HZ);	// Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz

	write_SPI0(MPUREG_ACCEL_CONFIG_2, BITS_DLPF_CFG_188HZ);	// Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
	
	
	
	write_SPI0(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);	// Set the I2C slave addres of AK8963 and set for write.
	write_SPI0(MPUREG_I2C_SLV0_REG, AK8963_CNTL2);	// I2C slave 0 register address from where to begin data transfer
	write_SPI0(MPUREG_I2C_SLV0_DO, 0x01);	/// Reset AK8963
	write_SPI0(MPUREG_I2C_SLV0_CTRL, 0x81);	// Enable I2C and set 1 byte
	
	write_SPI0(MPUREG_I2C_SLV0_REG, AK8963_CNTL1);	// I2C slave 0 register address from where to begin data transfer
	write_SPI0(MPUREG_I2C_SLV0_DO, 0x16);	// Register value to 100Hz continuous measurement in 16bit
	write_SPI0(MPUREG_I2C_SLV0_CTRL, 0x81);	//Enable I2C and set 1 byte
	  
	  
	
 
	/*
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers

	// reset device
	write_SPI0(MPUREG_PWR_MGMT_1, 0x80);	// Write a one to bit 7 reset bit; toggle reset device

	Wait100msX(1);
	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
  // else use the internal oscillator, bits 2:0 = 001
	write_SPI0(MPUREG_PWR_MGMT_1, 0x01);	//Get stable time source
	write_SPI0(MPUREG_PWR_MGMT_2, 0x00); 	//Enable all stuff
	Wait100msX(2);
	
	// Configure device for bias calculation
	write_SPI0(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
  write_SPI0(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
  write_SPI0(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  write_SPI0(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
  write_SPI0(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  write_SPI0(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  Wait100msX(1);
	
	// Configure MPU6050 gyro and accelerometer for bias calculation
  //write_SPI0(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  //write_SPI0(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  //write_SPI0(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  //write_SPI0(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
  
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
  write_SPI0(MPUREG_USER_CTRL, 0x40);   // Enable FIFO  
  write_SPI0(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  Wait100msX(1); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	write_SPI0(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	read_Mult_SPI0(MPUREG_FIFO_COUNTH, data, 2);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
	for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        read_Mult_SPI0(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
		accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
		
		if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
		
		// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
		
		// Push gyro biases to hardware registers
    //write_SPI0(MPUREG_XG_OFFS_USRH, data[0]);
    //write_SPI0(MPUREG_XG_OFFS_USRL, data[1]);
    //write_SPI0(MPUREG_YG_OFFS_USRH, data[2]);
    //write_SPI0(MPUREG_YG_OFFS_USRL, data[3]);
    //write_SPI0(MPUREG_ZG_OFFS_USRH, data[4]);
    //write_SPI0(MPUREG_ZG_OFFS_USRL, data[5]);
		
		// Output scaled gyro biases for display in the main program
    //dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    //dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    //dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
		
		// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

		
    read_Mult_SPI0(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    read_Mult_SPI0(MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    read_Mult_SPI0(MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		
    
    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
		
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
  
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
		// Apparently this is not working for the acceleration biases in the MPU-9250
		// Are we handling the temperature correction bit properly?
		// Push accelerometer biases to hardware registers
    //write_SPI0(MPUREG_XA_OFFSET_H, data[0]);
    //write_SPI0(MPUREG_XA_OFFSET_L, data[1]);
    //write_SPI0(MPUREG_YA_OFFSET_H, data[2]);
    //write_SPI0(MPUREG_YA_OFFSET_L, data[3]);
    //write_SPI0(MPUREG_ZA_OFFSET_H, data[4]);
    //write_SPI0(MPUREG_ZA_OFFSET_L, data[5]);
		
		write_SPI0(MPUREG_USER_CTRL, 0x30);	// I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
		write_SPI0(MPUREG_I2C_MST_CTRL, 0x0D);	// I2C configuration multi-master  IIC 400KHz
 
		write_SPI0(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);	// Set the I2C slave addres of AK8963 and set for write.
		
		write_SPI0(MPUREG_I2C_SLV0_REG, AK8963_CNTL2);	// I2C slave 0 register address from where to begin data transfer
		write_SPI0(MPUREG_I2C_SLV0_DO, 0x01);	// Reset AK8963
		write_SPI0(MPUREG_I2C_SLV0_CTRL, 0x81);	// Enable I2C and set 1 byte

		write_SPI0(MPUREG_I2C_SLV0_REG, AK8963_CNTL1);	// I2C slave 0 register address from where to begin data transfer
		write_SPI0(MPUREG_I2C_SLV0_DO, 0x16);	// Register value to 100Hz continuous measurement in 16bit
		write_SPI0(MPUREG_I2C_SLV0_CTRL, 0x81);	//Enable I2C and set 1 byte

		//UART3_OutString("SUCCESS!!!");

		// Output scaled accelerometer biases for display in the main program
    //dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    //dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    //dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
		//com_SPI0(0x1B, 0x18, 0x00);	// Configure gyroscope range 2000dps
		//com_SPI0(0x1C, 0x18, 0x00); // Configure accelerometers range 16G
	


		//com_SPI0(0x1D, 0x06, 0x00);	//Set accelerometers low pass filter at 5Hz
		//com_SPI0(0x1A, 0x06, 0x00);	//Set gyroscope low pass filter at 5Hz
	
	
	

	
		//rx = com_SPI0(0x6A, 0x00, 0x80);	// Example of reading
		//UART3_OutString("Test:\n\r");
		*/
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

void PWM_PF23_Duty(unsigned int duty){
	PWM1_3_CMPA_R = duty - 1;				// Duty PF2
	PWM1_3_CMPB_R = duty - 1;				// Duty PF3 
}

void PWM_PF1_Duty(unsigned int duty){
	PWM1_2_CMPB_R = (duty - 1);				// Duty PF1
}

void PWM_PA6_Duty(unsigned int duty){
	PWM1_1_CMPA_R = (duty - 1);				// Duty PA6
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

