// UART.c
// Runs on LM3S811, LM3S1968, LM3S8962, LM4F120, TM4C123
// Simple device driver for the UART.
// Daniel Valvano
// September 11, 2013
// Modified by EE345L students Charlie Gough && Matt Hawk
// Modified by EE345M students Agustinus Darmawan && Mingjie Qiu

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Program 4.12, Section 4.9.4, Figures 4.26 and 4.40

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

#include "UART.h"
#include "tm4c123gh6pm.h"
#include "stdint.h"


//------------UART_Init------------
// Initialize the UART for 9600 baud rate (assuming 50 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART3_Init(void){uint16_t delay;
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3;      //Clock for UART3
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; // activate port C
	delay = SYSCTL_RCGC2_R;           // allow time for clock to star
  GPIO_PORTC_DEN_R |= 0xC0; //Digital enable
  GPIO_PORTC_AFSEL_R |= 0xC0; //Alternate function select
  GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC7_U3TX; //PC7 is transmit i.e RXD in bluetooth module

	// Configure UART3 to 9600 baud, 8N1 format
  UART3_CTL_R = 0;											// Disable UART3
  UART3_CC_R = UART_CC_CS_SYSCLK;       // use system clock (80 MHz)
	UART3_IBRD_R = 520;    								// IBRD = int(80,000,000 / (16 * 9600)) = int(520.83333)
  UART3_FBRD_R = 53;                    // FBRD = int(0.83333 * 64 + 0.5) = 53
	UART3_LCRH_R = UART_LCRH_WLEN_8; 			// configure for 8N1 w/o FIFO
  UART3_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN; // enable TX and module
  //UART3_IM_R = UART_IM_TXIM;            // turn-on TX interrupt
  //NVIC_EN1_R = 1<<27;										//enable interrupt
}

void UART2_Init(void){uint16_t delay;
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2; 	// 	Turn-on UART2, leave other uarts in same status
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // activate port D
	delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTD_DEN_R |= 0x40;				// 	Digital Enable 
  GPIO_PORTD_AFSEL_R |= 0x40;								//	Alternate function
  GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_U2RX;	//	PD6 is recieve. i.e TXD in bluetooth module
	UART2_CTL_R = 0;													//	Disable UART2
  UART2_CC_R = UART_CC_CS_SYSCLK;           // 	Use system clock (80 MHz)
  UART2_IBRD_R = 520;    										// IBRD = int(80,000,000 / (16 * 9600)) = int(520.83333)
  UART2_FBRD_R = 53;                    		// FBRD = int(0.83333 * 64 + 0.5) = 53
  UART2_LCRH_R = UART_LCRH_WLEN_8; 					// configure for 8N1 w/o FIFO
  UART2_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN; // enable RX, and module
  UART2_IM_R = UART_IM_RXIM;                // turn-on RX interrupt
  NVIC_EN1_R = 1<<1;												//enable interrupt
}


//------------UART_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART_InChar(void){
  while((UART0_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART0_DR_R&0xFF));
}

//------------UART3_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART3_OutString(char *pt){
  while(*pt){
    UART3_OutChar(*pt);
    pt++;
  }
}


//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART_OutChar(unsigned char data){
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}


//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt){
  while(*pt){
    UART_OutChar(*pt);
    pt++;
  }
}

//------------UART_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
unsigned long UART_InUDec(void){
unsigned long number=0, length=0;
char character;
  character = UART_InChar();
  while(character != CR){ // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.
// If the character is not 0-9, it is ignored and not echoed
    if((character>='0') && (character<='9')) {
      number = 10*number+(character-'0');   // this line overflows if above 4294967295
      length++;
      UART_OutChar(character);
    }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
    else if((character==BS) && length){
      number /= 10;
      length--;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  return number;
}

//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART_OutUDec(unsigned long n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART_OutUDec(n/10);
    n = n%10;
  }
  UART_OutChar(n+'0'); /* n is between 0 and 9 */
}

//---------------------UART_InUHex----------------------------------------
// Accepts ASCII input in unsigned hexadecimal (base 16) format
// Input: none
// Output: 32-bit unsigned number
// No '$' or '0x' need be entered, just the 1 to 8 hex digits
// It will convert lower case a-f to uppercase A-F
//     and converts to a 16 bit unsigned number
//     value range is 0 to FFFFFFFF
// If you enter a number above FFFFFFFF, it will return an incorrect value
// Backspace will remove last digit typed
unsigned long UART_InUHex(void){
unsigned long number=0, digit, length=0;
char character;
  character = UART_InChar();
  while(character != CR){
    digit = 0x10; // assume bad
    if((character>='0') && (character<='9')){
      digit = character-'0';
    }
    else if((character>='A') && (character<='F')){
      digit = (character-'A')+0xA;
    }
    else if((character>='a') && (character<='f')){
      digit = (character-'a')+0xA;
    }
// If the character is not 0-9 or A-F, it is ignored and not echoed
    if(digit <= 0xF){
      number = number*0x10+digit;
      length++;
      UART_OutChar(character);
    }
// Backspace outputted and return value changed if a backspace is inputted
    else if((character==BS) && length){
      number /= 0x10;
      length--;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  return number;
}

//--------------------------UART_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void UART_OutUHex(unsigned long number){
// This function uses recursion to convert the number of
//   unspecified length as an ASCII string
  if(number >= 0x10){
    UART_OutUHex(number/0x10);
    UART_OutUHex(number%0x10);
  }
  else{
    if(number < 0xA){
      UART_OutChar(number+'0');
     }
    else{
      UART_OutChar((number-0x0A)+'A');
    }
  }
}

//------------UART_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is inputted.
// If a backspace is inputted, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses busy-waiting synchronization on RDRF
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --
void UART_InString(char *bufPt, unsigned short max) {
int length=0;
char character;
  character = UART_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  *bufPt = 0;
}

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}


//------------UART_InCharNonBlocking------------
// Get oldest serial port input and return immediately
// if there is no data.
// Input: none
// Output: ASCII code for key typed or 0 if no character
unsigned char UART_InCharNonBlocking(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  if((UART0_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART0_DR_R&0xFF));
  } else{
    return 0;
  }
}

/////////////////////////////////////////////////////////////////////////////
//------------UART3_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART3_InChar(void){
  while((UART3_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART3_DR_R&0xFF));
}

//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART3_OutUDec(unsigned long n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART3_OutUDec(n/10);
    n = n%10;
  }
  UART3_OutChar(n+'0'); /* n is between 0 and 9 */
}


//------------UART3_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART3_OutChar(unsigned char data){
  while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = data;
}

unsigned char UART3_InCharNonBlocking(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  if((UART3_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART3_DR_R&0xFF));
  } else{
    return 0;
  }
}

void UART3_InString(char *bufPt, unsigned short max) {
int length=0;
char character;
  character = UART3_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART3_OutChar(BS);
      }
    }
    else if(length < max){
			
      *bufPt = character;
      bufPt++;
      length++;
      UART3_OutChar(character);
    }
    character = UART3_InChar();
  }
  *bufPt = 0;
}

//---------------------OutCRLF3---------------------
// Output a CR,LF to UART3 to go to a new line
// Input: none
// Output: none
void OutCRLF3(void){
  UART3_OutChar(CR);
  UART3_OutChar(LF);
}

////////////////////////////////////////////////////////////////////////////////
//------------UART2_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART2_InChar(void){
  while((UART2_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART2_DR_R&0xFF));
}

void UART2_InString(char *bufPt, unsigned short max) {
int length=0;
char character;
  character = UART2_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART2_OutChar(BS);
      }
    }
    else if(length < max){
			
      *bufPt = character;
      bufPt++;
      length++;
      UART2_OutChar(character);
    }
    character = UART2_InChar();
  }
  *bufPt = 0;
}

//------------UART2_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART2_OutChar(unsigned char data){
  while((UART2_FR_R&UART_FR_TXFF) != 0);
  UART2_DR_R = data;
}

unsigned char UART2_InCharNonBlocking(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  if((UART2_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART2_DR_R&0xFF));
  } else{
    return 0;
  }
}

void UART2_OutString(char *pt){
  while(*pt){
    UART2_OutChar(*pt);
    pt++;
  }
}

//-----------------------UART2_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
	void UART2_OutUDec(unsigned long n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART2_OutUDec(n/10);
    n = n%10;
  }
  UART2_OutChar(n+'0'); /* n is between 0 and 9 */
}

//---------------------OutCRLF2---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF2(void){
  UART2_OutChar(CR);
  UART2_OutChar(LF);
}

