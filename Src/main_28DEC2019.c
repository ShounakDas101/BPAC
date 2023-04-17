/*

Initialize processor
Initialize all the necessary peripherals
Test all the peripherals GPIO UART SPI I2C
Read Board selection PINS
Check reset state conditions of the the relays

 Read Board selection pins (2-bits) : BSPINS

 If BSPINS=0 Communication Control Card (CCC0)
 If BSPINS=1 System 1 Card              (CPU1)
 If BSPINS=2 System 2 Card              (CPU2)
 if BSPINS=3 System 3 Card              (CPU3)

 Case: (BSPINS=0)
 Write to I2C LCD Testing Communication Control Card
 Read Local address(LOCADD) and Remote address(REMADD)

 Test SPI by requesting Board Identification ID (C1PU, CP2U, CPU3)
 Write to I2C LCD: Communication Control Card OK
 Send SPI command to CPU1
 Wait for SPI acknowledgement text Ready
 Write to I2C LCD: CPU One ready
 Send SPI command to System
 Wait for SPI acknowledgement text Ready
 Write to I2C LCD: CPU Two ready
 Send SPI command to System
 Wait for SPI acknowledgement text Ready
 Write to I2C LCD: CPU Three ready
 Write to I2C LCD: System OK  Now Press "Start BI Button"
 Wait for "Start BI Button" pressed
 Ask CPU1: relay status
 Receive CPU1 relay status

 Ask CPU2: relay status
 Receive CPU2 relay status
 Ask CPU3: relay status
 Receive CPU3 relay status
 If all CPU relay status satisfactory
 Display "Block Instrument OK"
 Send "Block Instrumenr OK + Local Address" to Remote station
 Wait till receive "Block Instrument OK + Remote Address"
 Display "Block Instrument ready to operate"
 Enable WDT
 Enter Infinite loop


 Case: (BSPIN=1, 2, 3)
 Test SPI by requesting Board Identification ID (C1PU, CP2U, CPU3)
  // Write to I2C LCD: Communication Control Card OK
  // Send SPI command to CPU1
  // Wait for SPI acknowledgement text 1Ready
 Write to I2C LCD: CPU One ready
  // Send SPI command to Sys2tem
  // Wait for SPI acknowledgement text Rea2dy
 Write to I2C LCD: CPU Two ready
 Send SPI command to 3System
  // Wait for SPI acknowledgement text Ready3
 Write to I2C LCD: CPU Three ready
 Write to I2C LCD: System OK  Now Press "Start BI Button"
 Wait for "Start BI Button" pressed
 Ask CPU1: relay status
  // Receive CPU1 relay status
 Ask CPU2: relay status
  // Receive CPU2 relay status
 Ask CPU3: relay status
  // Receive CPU3 relay status
 If all CPU relay status satisfactory
 Display "Block Instrument OK"
 Send "Block Instrumenr OK + Local Address" to Remote station
 Wait till receive "Block Instrument OK + Remote Address"
  // Display "Block Instrument ready to operate"
  // Enable WDT
 Enter Infinite loop
*/


/* GPIO are used for INPUT FEEDBACK and OUTPUT */
/* PORT A is used for UART2, SPI1 and Debugger */
/* --------------  UART2  -------------------- */
/* 1. Parameter Settings
				Asynchronous mode, 
				Hardware flow control disabled
				Baud rate: 2400 bits/s
				Word length : 8Bits (including Parity)
				Parity : None
				Stop bits : 1
				Data Direction : Transmit and Receive
				Over Sampling: 16 samples
   2. GPIO Settings 
				PA2=UART2_TX  PA3=UART2_RX
				GPIO output : N/A
				GPIO mode: Alternate Function Mode
				GPIO Pull-up/dn : Pull UP
				GPIO Speed: Very High
   3. NVIC Settings
				Global Interrupt disabled
	 4. DMA Settings 
	      None

*/

/* ---------------- I2C3 --------------------  */
/* 1. Parameter Settings
				Master mode, 
				Speed Standard
				Clock No Stretch mode - Disabled
				Address length : 7 Bits
				
   2. GPIO Settings 
				PH7=I2C3_SCL  PH8=I2C3_SDA
				GPIO output : N/A
				GPIO mode: Alternate Function Mode
				GPIO Pull-up/dn : Pull UP
				GPIO Speed: Very High
   3. NVIC Settings
				Global Interrupt disabled
	 4. DMA Settings 
	      None
*/

/* --------------- SPI1 ---------------------- */
/*
1. Parameter Settings
				Frame Format - Motorola, 
				Data - 8 Bits
				First Bit - MSB
				Prescaler for baudrate - 256
				Baud rate - 15.625 KBits/s
				CPOL - Low
				CPHA - 1st Edge
				CRC - Disabled
				NSS type - Software
   2. GPIO Settings 
				PA5-PA7 =(CLK_MISO_MOSI)
				GPIO output : N/A
				GPIO mode: Alternate Function Mode
				GPIO Pull-up/dn : No Pull UP
				GPIO Speed: Very High
   3. NVIC Settings
				Global Interrupt disabled
	 4. DMA Settings 
	      None
*/


#include "main.h"

/* Message Lists */
#define CPU1_ready	"CPU1 Board-READY"
#define CPU1_idle		"CPU1 Board-IDLE "
#define CPU2_ready	"CPU2 Board-READY"
#define CPU2_idle		"CPU2 Board-IDLE "
#define CPU3_ready	"CPU3 Board-READY"
#define CPU3_idle		"CPU3 Board-IDLE "

#define SPI_CR1_SPE		(1 << 6)
#define SPI_SR_RXNE   (1 << 0)
#define SPI_SR_TXE    (1 << 1)
#define SPI_SR_BSY    (1 << 7)


char LCD_MSG[16] ="XXXXXXXXXXXXXXXX";

uint8_t INP_FRONT_REG;
uint8_t INP_BACK_REG;
uint8_t FDB_FRONT_REG;
uint8_t FDB_BACK_REG;
uint8_t OUP_REG;

uint8_t boardName;
const uint8_t CCC0=0;
const uint8_t	CPU1=1;
const uint8_t	CPU2=2;
const uint8_t	CPU3=3;
const uint8_t XXX0=4;

uint8_t msgCCC0_CPU1[16];
uint8_t msgCCC0_CPU2[16];
uint8_t msgCCC0_CPU3[16];
uint8_t msgCPU1_CCC0[16];
uint8_t msgCPU2_CCC0[16];
uint8_t msgCPU3_CCC0[16];

uint8_t msgCCC0[16];




//uint8_t rs_CPU1;
//uint8_t rs_CPU2;
//uint8_t rs_CPU3;



void	SystemInit(void);
uint8_t  IdentifyBoard(void);
void  CCC0IO_Init(void);
void  CPUxIO_Init(void);
void  UART2_Init(void);
void  SPI_MasterInit(void); //CPOL=0, CPHA=0
void  SPI_SlaveInit(void); //CPOL=0, CPHA=0
uint8_t SPI_SendRecv(uint8_t);
void  SPI_SendRecvBuf(uint8_t*, uint8_t);
//uint8_t spi_read(uint8_t);
void  I2C_Init(void);
void  I2C_Start(void);
void  I2C_Stop(void);
void 	I2C_Write( uint8_t c);
void  LCD_Send_cmd(uint8_t cmd);
void  LCD_Send_data(uint8_t data);
void  LCD_Init(void);
void  LCD_Send_string(char* str);
void  LCD_Send_string_Line1(char *str);
void  LCD_Send_string_Line2(char *str);
void  LCD_Clear(void);
void  LCD_BacklightON(void);
void  LCD_BacklightOFF(void);
void	mSec(unsigned int );
void  uSec(unsigned int);


int	main (void) {

	
  // Initialize clock and system clock speed, flash
SystemInit();
	// Identify the boards
boardName = IdentifyBoard();
	// Initialize i2c communication
I2C_Init();	
	
	switch(boardName){
		case 0:{
		  CCC0IO_Init();
			SPI_CS1_SET;//mSec(5000);	
	    SPI_CS2_SET;//mSec(5000);
	    SPI_CS3_SET;//mSec(5000);
			SPI_MasterInit();
			UART2_Init();	
			LCD_Init();
			LCD_Send_string_Line1("AKG Electrinics.");	
			LCD_Send_string_Line2("Indian Railways.");
      mSec(5000);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			
			
			while(1){
			
//				// wait till CCC0 receives 
//				while(1){
//				SPI_CS1_RESET;uSec(50);
//			  SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
//			  msgCPU1_CCC0[0]=SPI_SendRecv('1');
//			  SPI_CS1_SET;uSec(50);
//			  SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
//			  LCD_MSG[13]=msgCPU1_CCC0[0];
//			  
//					break;
//				}
			*msgCCC0_CPU1=(uint8_t)"?BPN1";	
			SPI_CS1_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			msgCPU1_CCC0[0]=SPI_SendRecv(msgCCC0_CPU1[0]);
			msgCPU1_CCC0[1]=SPI_SendRecv(msgCCC0_CPU1[1]);
			msgCPU1_CCC0[2]=SPI_SendRecv(msgCCC0_CPU1[2]);
			msgCPU1_CCC0[3]=SPI_SendRecv(msgCCC0_CPU1[3]);
			msgCPU1_CCC0[4]=SPI_SendRecv(msgCCC0_CPU1[4]);
			SPI_CS1_SET;uSec(50);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
			LCD_MSG[13]=msgCPU1_CCC0[0];
							
			SPI_CS2_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			//msgCPU2_CCC0[0]='f';	
			msgCPU2_CCC0[0]=SPI_SendRecv('?');
			msgCPU2_CCC0[1]=SPI_SendRecv('B');
			msgCPU2_CCC0[2]=SPI_SendRecv('P');
			msgCPU2_CCC0[3]=SPI_SendRecv('N');
			msgCPU2_CCC0[4]=SPI_SendRecv('2');
			SPI_CS2_SET;uSec(50);	
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
			LCD_MSG[14]=msgCPU2_CCC0[0];	

			SPI_CS3_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			msgCPU3_CCC0[0]=SPI_SendRecv('?');//mSec(20);
			msgCPU3_CCC0[1]=SPI_SendRecv('B');
			msgCPU3_CCC0[2]=SPI_SendRecv('P');
			msgCPU3_CCC0[3]=SPI_SendRecv('N');
			msgCPU3_CCC0[4]=SPI_SendRecv('3');
			SPI_CS3_SET;uSec(50);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LCD_MSG[0]=msgCPU1_CCC0[0];
			LCD_MSG[1]=msgCPU1_CCC0[1];
			LCD_MSG[2]=msgCPU1_CCC0[2];
			LCD_MSG[3]=msgCPU1_CCC0[3];
			LCD_MSG[4]=msgCPU1_CCC0[4];
			LCD_MSG[5]=msgCPU2_CCC0[0];
			LCD_MSG[6]=msgCPU2_CCC0[1];
			LCD_MSG[7]=msgCPU2_CCC0[2];
			LCD_MSG[8]=msgCPU2_CCC0[3];
			LCD_MSG[9]=msgCPU2_CCC0[4];
			LCD_MSG[10]=msgCPU3_CCC0[0];
			LCD_MSG[11]=msgCPU3_CCC0[1];
			LCD_MSG[12]=msgCPU3_CCC0[2];
			LCD_MSG[13]=msgCPU3_CCC0[3];
			LCD_MSG[14]=msgCPU3_CCC0[4];
			
				
			LCD_Send_string_Line2(LCD_MSG);
			SPI_CS1_SET;uSec(50);	
      SPI_CS2_SET;uSec(50);
      SPI_CS3_SET;uSec(50);
			
//			mSec(2000);
//			if(LCD_MSG[1]=='a')LCD_Send_string_Line2("01asdfgh23456789");
//			mSec(2000);
//			if(LCD_MSG[1]=='b')LCD_Send_string_Line2("0123asdfgh456789");
//			mSec(2000);
//			if(LCD_MSG[1]=='c')LCD_Send_string_Line2("0123456789asdfgh");
//			
//			else
//													 LCD_Send_string_Line2("0xxx456789asdfgh");
			while(1);
			
			}
      
			
		}
		case 1:{
			CPUxIO_Init();GPIOD_ODR = 0xF00F;mSec(5000);
			SPI_SlaveInit(); //CPOL=0, CPHA=0
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LFR_SET;
			TCFXR_SET;
			TCFCR_SET;
			TGTYR_SET;
			while(1){
				// Wait till cpu receives command or data
				
				// Read all the INPUT
				
				// Send status of input relays contacts, feedback relay contacts, output relay contacts
			  while(SPI_CS1_STATUS);
				
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
				msgCCC0_CPU1[0]=SPI_SendRecv('1');
				if(msgCCC0_CPU1[0]=='?'){
					msgCCC0_CPU1[1]=SPI_SendRecv('1');
					msgCCC0_CPU1[1]=SPI_SendRecv('R');
					msgCCC0_CPU1[1]=SPI_SendRecv('s');
					msgCCC0_CPU1[1]=SPI_SendRecv('N');
					msgCCC0_CPU1[1]=SPI_SendRecv('a');
					
					if((msgCCC0_CPU1[1]=='B') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU1[2]=SPI_SendRecv('S');
					if((msgCCC0_CPU1[1]=='B') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU1[2]=SPI_SendRecv('R');
					
					if((msgCCC0_CPU1[2]=='P') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU1[3]=SPI_SendRecv('S');
					if((msgCCC0_CPU1[2]=='P') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU1[3]=SPI_SendRecv('R');

					if((msgCCC0_CPU1[3]=='N') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU1[4]=SPI_SendRecv('S');
					if((msgCCC0_CPU1[3]=='N') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU1[4]=SPI_SendRecv('R');
			    }
				while(!SPI_CS1_STATUS);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				if(msgCCC0_CPU1[0]=='1'){
				LFR_SET;
				TCFXR_RESET;
				TCFCR_RESET;
				TGTYR_SET;
				}
				}// end of main while loop
			 while(1);
		}
		
		case 2:{
			CPUxIO_Init();GPIOD_ODR = 0xF00F;mSec(5000);
			SPI_SlaveInit(); //CPOL=0, CPHA=0
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LFR_SET;
			TCFXR_SET;
			TCFCR_SET;
			TGTYR_SET;
      	
			while(1){
				
				while(SPI_CS2_STATUS);
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
				msgCCC0_CPU2[0]=SPI_SendRecv('2');
				if(msgCCC0_CPU2[0]=='?'){
					msgCCC0_CPU2[1]=SPI_SendRecv('2');
					
					if((msgCCC0_CPU2[1]=='B') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU2[2]=SPI_SendRecv('S');
					if((msgCCC0_CPU2[1]=='B') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU2[2]=SPI_SendRecv('R');
					
					if((msgCCC0_CPU2[2]=='P') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU2[3]=SPI_SendRecv('S');
					if((msgCCC0_CPU2[2]=='P') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU2[3]=SPI_SendRecv('R');

					if((msgCCC0_CPU2[3]=='N') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU2[4]=SPI_SendRecv('S');
					if((msgCCC0_CPU2[3]=='N') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU2[4]=SPI_SendRecv('R');
			    }
				while(!SPI_CS2_STATUS);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				if(msgCCC0_CPU2[0]=='2'){
				LFR_SET;
				TCFXR_RESET;
				TCFCR_RESET;
				TGTYR_SET;
				}
				
			}// end of main while loop
			 while(1);
		}
		
		case 3:{
			CPUxIO_Init();GPIOD_ODR = 0xF00F;mSec(5000);
			SPI_SlaveInit(); //CPOL=0, CPHA=0
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LFR_SET;
			TCFXR_SET;
			TCFCR_SET;
			TGTYR_SET;
			
			while(SPI_CS3_STATUS);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			msgCCC0_CPU3[0]=SPI_SendRecv('3');
				if(msgCCC0_CPU3[0]=='?'){
					msgCCC0_CPU3[1]=SPI_SendRecv('3');
					
					if((msgCCC0_CPU3[1]=='B') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU3[2]=SPI_SendRecv('S');
					if((msgCCC0_CPU3[1]=='B') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU3[2]=SPI_SendRecv('R');
					
					if((msgCCC0_CPU3[2]=='P') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU3[3]=SPI_SendRecv('S');
					if((msgCCC0_CPU3[2]=='P') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU3[3]=SPI_SendRecv('R');

					if((msgCCC0_CPU3[3]=='N') && (BPNR_FRONT_STATUS !=0) && (BPNR_BACK_STATUS==0))
					msgCCC0_CPU3[4]=SPI_SendRecv('S');
					if((msgCCC0_CPU3[3]=='N') && (BPNR_FRONT_STATUS ==0) && (BPNR_BACK_STATUS!=1))
					msgCCC0_CPU3[4]=SPI_SendRecv('R');
			    }
			while(!SPI_CS3_STATUS);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			if(msgCCC0_CPU3[0]=='3'){
				LFR_SET;
				TCFXR_RESET;
				TCFCR_RESET;
				TGTYR_SET;
				}
				 
			}// end of main while loop
		 while(1);
		
		default :
			while(1);
	
	}// end of switch-case 

// Endless loop	
while (1);

}// End of main



/*  Command()  */
void Command(uint8_t cpu){
	if(cpu==1){
	    SPI_CS1_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			msgCPU1_CCC0[0]=SPI_SendRecv(msgCCC0_CPU1[0]);
			msgCPU1_CCC0[1]=SPI_SendRecv(msgCCC0_CPU1[1]);
			msgCPU1_CCC0[2]=SPI_SendRecv(msgCCC0_CPU1[2]);
			msgCPU1_CCC0[3]=SPI_SendRecv(msgCCC0_CPU1[3]);
			msgCPU1_CCC0[4]=SPI_SendRecv(msgCCC0_CPU1[4]);
			SPI_CS1_SET;uSec(50);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
	}
	else if(cpu==2){						
			SPI_CS2_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			//msgCPU2_CCC0[0]='f';	
			msgCPU2_CCC0[0]=SPI_SendRecv(msgCCC0_CPU2[0]);
			msgCPU2_CCC0[1]=SPI_SendRecv(msgCCC0_CPU2[1]);
			msgCPU2_CCC0[2]=SPI_SendRecv(msgCCC0_CPU2[2]);
			msgCPU2_CCC0[3]=SPI_SendRecv(msgCCC0_CPU2[3]);
			msgCPU2_CCC0[4]=SPI_SendRecv(msgCCC0_CPU2[4]);
			SPI_CS2_SET;uSec(50);	
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
	}	
  else{
			SPI_CS3_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			msgCPU3_CCC0[0]=SPI_SendRecv(msgCCC0_CPU3[0]);
			msgCPU3_CCC0[1]=SPI_SendRecv(msgCCC0_CPU3[1]);
			msgCPU3_CCC0[2]=SPI_SendRecv(msgCCC0_CPU3[2]);
			msgCPU3_CCC0[3]=SPI_SendRecv(msgCCC0_CPU3[3]);
			msgCPU3_CCC0[4]=SPI_SendRecv(msgCCC0_CPU3[4]);
			SPI_CS3_SET;uSec(50);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
	}
}


/*  Query()   */
void Query(uint8_t cpu){
if(cpu==1){
	    SPI_CS1_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			msgCPU1_CCC0[0]=SPI_SendRecv(msgCCC0_CPU1[0]);
			msgCPU1_CCC0[1]=SPI_SendRecv(msgCCC0_CPU1[1]);
			msgCPU1_CCC0[2]=SPI_SendRecv(msgCCC0_CPU1[2]);
			msgCPU1_CCC0[3]=SPI_SendRecv(msgCCC0_CPU1[3]);
			msgCPU1_CCC0[4]=SPI_SendRecv(msgCCC0_CPU1[4]);
			SPI_CS1_SET;uSec(50);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
	}
	else if(cpu==2){						
			SPI_CS2_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			//msgCPU2_CCC0[0]='f';	
			msgCPU2_CCC0[0]=SPI_SendRecv(msgCCC0_CPU2[0]);
			msgCPU2_CCC0[1]=SPI_SendRecv(msgCCC0_CPU2[1]);
			msgCPU2_CCC0[2]=SPI_SendRecv(msgCCC0_CPU2[2]);
			msgCPU2_CCC0[3]=SPI_SendRecv(msgCCC0_CPU2[3]);
			msgCPU2_CCC0[4]=SPI_SendRecv(msgCCC0_CPU2[4]);
			SPI_CS2_SET;uSec(50);	
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
	}	
  else{
			SPI_CS3_RESET;uSec(50);
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			msgCPU3_CCC0[0]=SPI_SendRecv(msgCCC0_CPU3[0]);
			msgCPU3_CCC0[1]=SPI_SendRecv(msgCCC0_CPU3[1]);
			msgCPU3_CCC0[2]=SPI_SendRecv(msgCCC0_CPU3[2]);
			msgCPU3_CCC0[3]=SPI_SendRecv(msgCCC0_CPU3[3]);
			msgCPU3_CCC0[4]=SPI_SendRecv(msgCCC0_CPU3[4]);
			SPI_CS3_SET;uSec(50);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
	}

}


/*   Feedback()   */
void Feedback(uint8_t cpu){
	      if(cpu==1){
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  msgCCC0_CPU1[0]=SPI_SendRecv(msgCPU1_CCC0[0]);
				msgCCC0_CPU1[1]=SPI_SendRecv(msgCPU1_CCC0[1]);
				msgCCC0_CPU1[2]=SPI_SendRecv(msgCPU1_CCC0[2]);
				msgCCC0_CPU1[3]=SPI_SendRecv(msgCPU1_CCC0[3]);
				msgCCC0_CPU1[4]=SPI_SendRecv(msgCPU1_CCC0[4]);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
	      }
				else if(cpu==2){
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  msgCCC0_CPU2[0]=SPI_SendRecv(msgCPU2_CCC0[0]);
				msgCCC0_CPU2[1]=SPI_SendRecv(msgCPU2_CCC0[1]);
				msgCCC0_CPU2[2]=SPI_SendRecv(msgCPU2_CCC0[2]);
				msgCCC0_CPU2[3]=SPI_SendRecv(msgCPU2_CCC0[3]);
				msgCCC0_CPU2[4]=SPI_SendRecv(msgCPU2_CCC0[4]);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				}
				else{
	      SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  msgCCC0_CPU3[0]=SPI_SendRecv(msgCPU3_CCC0[0]);
				msgCCC0_CPU3[1]=SPI_SendRecv(msgCPU3_CCC0[1]);
				msgCCC0_CPU3[2]=SPI_SendRecv(msgCPU3_CCC0[2]);
				msgCCC0_CPU3[3]=SPI_SendRecv(msgCPU3_CCC0[3]);
				msgCCC0_CPU3[4]=SPI_SendRecv(msgCPU3_CCC0[4]);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				}
}

/*   Respond()    */
void Respond(uint8_t cpu){
	
				if(cpu==1){
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  msgCCC0_CPU1[0]=SPI_SendRecv(msgCPU1_CCC0[0]);
				msgCCC0_CPU1[1]=SPI_SendRecv(msgCPU1_CCC0[1]);
				msgCCC0_CPU1[2]=SPI_SendRecv(msgCPU1_CCC0[2]);
				msgCCC0_CPU1[3]=SPI_SendRecv(msgCPU1_CCC0[3]);
				msgCCC0_CPU1[4]=SPI_SendRecv(msgCPU1_CCC0[4]);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
	      }
				else if(cpu==2){
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  msgCCC0_CPU2[0]=SPI_SendRecv(msgCPU2_CCC0[0]);
				msgCCC0_CPU2[1]=SPI_SendRecv(msgCPU2_CCC0[1]);
				msgCCC0_CPU2[2]=SPI_SendRecv(msgCPU2_CCC0[2]);
				msgCCC0_CPU2[3]=SPI_SendRecv(msgCPU2_CCC0[3]);
				msgCCC0_CPU2[4]=SPI_SendRecv(msgCPU2_CCC0[4]);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				}
				else{
	      SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  msgCCC0_CPU3[0]=SPI_SendRecv(msgCPU3_CCC0[0]);
				msgCCC0_CPU3[1]=SPI_SendRecv(msgCPU3_CCC0[1]);
				msgCCC0_CPU3[2]=SPI_SendRecv(msgCPU3_CCC0[2]);
				msgCCC0_CPU3[3]=SPI_SendRecv(msgCPU3_CCC0[3]);
				msgCCC0_CPU3[4]=SPI_SendRecv(msgCPU3_CCC0[4]);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				}
}


/*       
------------Configuring the AF in the STM32F407-----------------------------
1. Attach the alternate function.
2. Enable the clock to the alternate function.
3. Enable clock to corresponding GPIO
4. Configure the input-output port and pins (of the corresponding GPI0x) to match the AF .
5. If desired enable the nested vector interrupt control to generate interrupts.
6. Program the AF/peripheral for the required configuration (eg baud rate for a USART) */

void  UART2_Init( ){
	//Enable clock GPIOA for UART2 PA2, PA3 
	RCC_AHB1ENR |= (1<<0);
	uSec(50);
	// enable USART2 clock, bit 4 on APB1ENR
	RCC_APB1ENR |= (1 << 17);
	uSec(50);
	RCC_APB2ENR |= (1 << 14);
	uSec(50);
	// Reset/Clear Bits 4,5 for PA2 of Tx
	GPIOA_MODER &= ~(3<<4);
	uSec(50);
	// Reset/Clear Bits 6,7 for PA3 of Rx
	GPIOA_MODER &= ~(3<<6);
  uSec(50);
	GPIOA_SPEEDR=0x0C0000F0;
	uSec(50);
	GPIOA_PUPDR=0x64000050;
	uSec(50);
	
	// GPIOA PA2 -Alternate Function Mode
	GPIOA_MODER |= 2<<4;
	uSec(50);
	// GPIOA PA3 -Alternate Function Mode
	GPIOA_MODER |= 2<<6;
	uSec(50);
	// set pin modes as high speed
	//GPIOA_OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)
	//uSec();
	// choose AF7 for USART2 in Alternate Function Registers
	// PA2 - UART2_TX
	GPIOA_AFR0 |= (0x7 << 8); // for pin 2
	uSec(50);
	// PA3 - UART2_RX
	GPIOA_AFR0 |= (0x7 << 12); // for pin 3
	uSec(50);
	
	
	// USART2 word length M=0, bit number 12
	UART2_CR1 &= ~(1 << 12); // 0 - 1,8,n
	uSec(50);
	// USART2 parity control, 0 - no parity PS=0 PCE=0
	UART2_CR1 &= ~(3 << 9); 
	uSec(50);	
	// USART2 stop bit, STOP = 00b one stop bit
	UART2_CR2 &= ~(3 << 12); 
	uSec(50);
	// USART2 flow control, no flow control RTSE=0 CTSE=0
	UART2_CR3 &= ~(3 << 8);
	uSec(50);
	// USART2 TX enable, TE bit 3
	UART2_CR1 |= (1 << 3);
	uSec(50);
	// usart1 rx enable, RE bit 2
	UART2_CR1 |= (1 << 2);
	uSec(50);
	//   UART2  we should use PCLK1 in APB1
	//   baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
	//   for fCK = 42 Mhz, baud = 115200, OVER8 = 0
	//   USARTDIV = 42Mhz / 115200 / 16
	//   = 22.7864 22.8125
	//   we can also look at the table in RM0090
	//   for 42 Mhz PCLK, OVER8 = 0 and 115.2 KBps baud
	//   we need to program 22.8125
	// Fraction : 16*0.8125 = 13 (multiply fraction with 16)
	// Mantissa : 22
	// 12-bit mantissa and 4-bit fraction
	
	// NKDas calculation BRR= 4/(16 * 0.0024)
	UART2_BRR |= (104 << 4);
	uSec(50);
	UART2_BRR |= 0x11;
	uSec(50);
	
	// enable usart - UE=1, bit 13
	UART2_CR1 |= (1 << 13);
	uSec(50);

}// End of UART1Init()


void	SystemInit( ) {
// Set HSION bit	
RCC_CR  |= 0x00000001;
uSec(50);	
// Reset CFGR
RCC_CFGR = 0x00000000;
uSec(50);
// Reset HSEON CSSON PLLON
//RCC_CR  &= 0xFEF6FFFF;
// Reset PLLCFGR
//RCC_PLLCFGR = 0x24003010;
// Reset HSEBYP Bit
//RCC_CR  &= 0xFFFBFFFF;
// Disable all interrupts
RCC_CIR  = 0x00000000;
uSec(50);
/******************************************************************************/
/*                        HSI used as System clock source                     */
/* Configure the System clock source, PLL Multiplier and Divider factors, 
     AHB/APBx prescalers and Flash settings ----------------------------------*/
/******************************************************************************/

  /* At this stage the HSI is already enabled and used as System clock source */

  /* Configure Flash prefetch, Instruction cache, Data cache and uSec state */
  FLASH_ACR |= 0x00000600; //FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_0WS;
  uSec(50);
  /* HCLK = SYSCLK / 1*/
  //RCC_CFGR |= 0x00000000; //RCC_CFGR_HPRE_DIV1 (not divided)
      
  /* PCLK2 = HCLK / 1*/
  //RCC_CFGR |= 0x00000000;//RCC_CFGR_PPRE2_DIV1 (not divided )    
  /* PCLK1 = HCLK / 1*/
  //RCC_CFGR |= 0x00000000 ;//RCC_CFGR_PPRE1_DIV1 (not divided )
	
	// Devived by 4
	RCC_CFGR |= 0x00000090 ;
	
	// check system clock is ready 
	do{
	}while(!(RCC_CR & 0x00000002));	

	RCC_CR |= 0x00000081;	
	uSec(50);
}

uint8_t IdentifyBoard(void){
	RCC_AHB1ENR |= 1 << 0; 	uSec(50);
	// xxxxxx00 00xxxxxx xxxxxxxx xxxxxxxx
	GPIOA_MODER  &= 0xfc3fffff; 	uSec(50); //PA12,PA11 as input boardID
	
	// Check for X0000000 00X00000 of GPIOH_IDR
	     if((GPIOA_IDR & 0x1800) == 0x0000) return 0;//boardName=CCC0;
	else if((GPIOA_IDR & 0x1800) == 0x1000) return 1;//boardName=CPU1;
	else if((GPIOA_IDR & 0x1800) == 0x0800) return 2;//boardName=CPU2;
	else if((GPIOA_IDR & 0x1800) == 0x1800) return 3;//boardName=CPU3;
	else 																		return 4;//boardName=XXX0;
   
}// End of IdentifyBoard()

void  CCC0IO_Init(void){
	/*        Care should be taken while setting the bits because it 
					  is used by JTAG programmer also, 
						Keep  PA15 PA14 PA13 PA12 PA11 PA1 PA0  UNCHANGED
                  JTAG---------- BID1	BID0 JTAG---	
				
				 GPIOA    PA7		PA6		PA5		PA4		PA3		PA2
									MOSI	MISO	SCLK	CS1		RX		TX
									OUT		IN		OUT		OUT		AFR		AFR	
									xxxxxxxx xxxxxxxx 01000101 1010xxxx
				This is CCC0  BIT BANGING
	*/
					RCC_AHB1ENR |= 1 << 0; uSec(50);
					// BIT BANGING GPIO
					GPIOA_MODER  &= 0xffff000f; uSec(50); // clear bits 4 t 15	
					GPIOA_MODER  |= 0x000045a0; uSec(50); // // set bits, tx rx initialized here as AFR
					GPIOA_SPEEDR |= 0x00005550; uSec(50);//HIGH SPEED: xxxxxxxx xxxxxxxx 01010101 0101xxxx
	
	
	
					
				/*	GPIOC		PC6		PC5
										INP		INP
										xxxxxxxx xxxxxxxx xx0000xx xxxxxxxx
				*/
					RCC_AHB1ENR |= 1 << 2;uSec(50);
					GPIOC_MODER  &= 0xffffc3ff;uSec(50);
				
				/*	GPIOD		PD15	PD14	PD13	PD12 	rest are unused
				  					OUT  	OUT		OUT		OUT		
										01010101 xxxxxxxx xxxxxxxx xxxxxxxx	
				*/
					RCC_AHB1ENR |= 1 << 3;uSec(50);
					GPIOD_MODER  &= 0x00000000;uSec(50);
					GPIOD_MODER  |= 0x55FFFFFF;uSec(50);
					
				/* GPIOE		PE7		PE6		PE5		PE4		PE3		PE2		PE1		PE0
					  				LAD0	LAD1	LAD2	LAD3	LAD4	LDA5	LAD6	LAD7  (local addr)
										xxxxxxxx xxxxxxxx 00000000 00000000	
				*/
					RCC_AHB1ENR |= 1 << 4;uSec(50);
					GPIOE_MODER  &= 0xffff0000;uSec(50);
					
					
					
				/* GPIOF		PF15	PF14	PF13	PF12	PF11	PF10	PF9		PF8		PF7 	PF6
					  				RAD0	RAD1	RAD2	RAD3	RAD4	RAD5	RAD6	RAD7	CS3		CS2
										IN		IN		IN		IN		IN		IN		IN		IN		OUT		OUT		
										00000000 00000000 0101xxxx xxxxxxxx
				*/
					RCC_AHB1ENR |= 1 << 5;uSec(50);
					GPIOF_MODER  &= 0x00000fff;uSec(50);//clear bits PF6 - PF15
					GPIOF_MODER  |= 0x00005000;uSec(50);
					
				// I2C_SDA and I2C_SCL  : should not be initialized at this moment  
	      // PH8=I2C_SDA(01), PH7=I2C_SCL(01)
	      //11111111 11111101 01111111 11111111
				RCC_AHB1ENR |= 1<<7;uSec(50);
				GPIOH_MODER  &= ~(3<<14); uSec(50);// clear bits
				GPIOH_MODER  &= ~(3<<16); uSec(50);
				GPIOH_MODER  |=  (1<<14); uSec(50);// set bits (0b01) for I2C SDA, SCL
				GPIOH_MODER  |=  (1<<16); uSec(50);
				GPIOH_OTYPER |= 1 <<7;			//PH7 open drain
				GPIOH_OTYPER |= 1 <<8;			//PH8 open drain
				GPIOH_PUPDR &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
				GPIOH_PUPDR &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
				I2C_SDA_HIGH;
				I2C_SCL_HIGH;
				//HIGH SPEED
				// 00000000 00000001 01000000 00000000
				GPIOH_SPEEDR |= 0x00014000; uSec(50);	
				
				/*	GPIOI		PI7		PI6
				//					OUT		OUT
				//  11111111 11111111 01011111 11111111		
				*/
				RCC_AHB1ENR |= 1 << 8;uSec(50);
				GPIOI_MODER  = 0xffff5fff;uSec(50);


}
void  CPUxIO_Init(void){
	/* Care should be taken while setting the bits because it 
					  is used by JTAG programmer also, 
						Keep  PA15 PA14 PA13 PA12 PA11 PA1 PA0  UNCHANGED
                  JTAG---------- BID1	BID0 JTAG---	
				
				 GPIOA    PA7		PA6		PA5		PA4		
									MOSI	MISO	SCLK	CS1				
								  IN		OUT		IN    IN		
									xxxxxxxx xxxxxxxx 00010000 xxxxxxxx
	*/
					RCC_AHB1ENR |= 1 << 0; uSec(50);
					GPIOA_MODER  &= 0xffff00ff; uSec(50); // clear bits 4 t 15	
					GPIOA_MODER  |= 0x00001000; uSec(50); // // set bits
					GPIOA_SPEEDR |= 0x00005500; uSec(50);//HIGH SPEED: xxxxxxxx xxxxxxxx 01010101 xxxxxxxx
	

	
/*	PB3 PB4 bits should not be disturbed
		GPIOB		PB14	PB13	PB11	PB10	PB8
						NC		NC		BLR		BLR		TGTZR
						IN		IN		IN		IN 		IN
  11000011 00001100 11111111 11111111	
	*/
	RCC_AHB1ENR |= 1 << 1;uSec(50);
	GPIOB_MODER  |= 0x0000ffff;uSec(50);
	GPIOB_MODER  |= 0xc30c0000;uSec(50);
	
/*	GPIOC		PC6		PC5		PC3		PC2		PC0
  					NC		NC		BPNR	BPNR	HSGNCR
						IN		IN		IN		IN		IN
						xxxxxxxxx xxxxxxxx xx0000xx 0000xx00
*/
  RCC_AHB1ENR |= 1 << 2;uSec(50);
	GPIOC_MODER  = 0xffffc30c;uSec(50);
	

/*GPIOD			PD15	PD14	PD13	PD12			PD3		PD2		PD1		PD0	
  					OUT		BLR		TGTZR	ASGNCPR		TGTYR	TCFCR	TCFXR	LFR
  					OUT		OUT		OUT		OUT				OUT		OUT		OUT		OUT
						01010101 xxxxxxxx xxxxxxxx 01010101
*/
	RCC_AHB1ENR |= 1 << 3;uSec(50);
	GPIOD_MODER  = 0x55ffff55;uSec(50);
	
					
/*	GPIOE		PE15	PE14	PE12	PE11	PE9		PE8
  					LFR_F	LFR_B	TCFXR TCFXR	TCFCR	TCFCR   all input
  					PE7		PE6		PE4		PE3		PE1			PE0
  					PWR		PWR		TGTXR	TGTXR	ASGNCR	ASGNCR  all input
            00001100 00110000 00001100 00110000	
*/
	RCC_AHB1ENR |= 1 << 4;uSec(50);
	GPIOE_MODER  = 0x0c300c30;uSec(50);
	

/*	GPIOF		PF14	PF13	PF11	PF10	PF8			PF7		PF6			
  					TCFR	TCFR	BTSR	BTSR	HSGNCR	CS2		CS3   all input
            xx0000xx 0000xx00 0000xxxx xxxxxxxx    					
*/
	RCC_AHB1ENR |= 1 << 5;uSec(50);
	GPIOF_MODER  = 0xc30c0fff;uSec(50);
	
/*	GPIOG		PG15	PG13		PG12		PG10	PG9		
  					TGTZR	ASGNCPR	ASGNCPR	TGTYR	TGTYR    all input
            00xx0000 xx0000xx xxxxxxxx xxxxxxxx
*/
	RCC_AHB1ENR |= 1 << 6;uSec(50);
	GPIOG_MODER  = 0x30c3ffff;uSec(50);
	
	/* GPIOH should not be enabled here */
	GPIOH_OTYPER |= 1 <<7;			//PH7 open drain
  GPIOH_OTYPER |= 1 <<8;			//PH8 open drain
  GPIOH_PUPDR &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
  GPIOH_PUPDR &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
	
	
}
void  SPI_MasterInit(){
// Ensure that CS1, CS2, CS3 are configured
		//Already configured 	
// SPI1 data pins setup

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC_AHB1ENR |= (1 << 0);
    // enable SPI1 clock, bit 12 on APB2ENR
    RCC_APB2ENR |= (1 << 12);

    // set pin modes as alternate mode
    GPIOA_MODER &= 0xFFFF03FF; // Reset bits 10-15 to clear old values
    GPIOA_MODER |= 0x0000A800; // Set pin 5/6/7 to alternate func. mode (0b10)

    // set pin modes as very high speed
    GPIOA_SPEEDR |= 0x0000FC00; // Set pin 5/6/7 to very high speed mode (0b11)
		
    // choose AF5 for SPI1 in Alternate Function registers
    GPIOA_AFR0 |= (0x5 << 20); // for pin 5
    GPIOA_AFR0 |= (0x5 << 24); // for pin 6
    GPIOA_AFR0 |= (0x5 << 28); // for pin 7

    // Disable SPI1 and set the rest (no OR'ing)

    // baud rate - BR[2:0] is bit 5:3
    // fPCLK/2 is selected by default
    //SPI1_CR1   &= ~(7 << 3); // set baud rate to fPCLK/2
		  SPI1_CR1   |=  (7 << 3); // set baud rate to fPCLK/256

    // 8/16-bit mode - DFF is bit 11
    //SPI1_CR1 |= (1 << 11); // 1 - 16-bit mode
      SPI1_CR1 &=~(1 << 11); // 1 - 8-bit mode
    
		// motion sensor expects clk to be high and
    // transmission happens on the falling edge
    // clock polarity - CPOL bit 1
    SPI1_CR1 |= (0 << 1); // clk goes 1 when idle
    // clock phase - CPHA bit 0
    SPI1_CR1 |= (0 << 0); // first clock transaction

    // frameformat - LSBFIRST bit 7, msb/lsb transmit first
    // 0 - MSB transmitted first
    //SPI1->CR1 |= (0 << 7); // 1 - LSB transmitted first

    // frame format - FRF bit 4 on CR2
    // 0 - Motorola mode, 1 - TI mode
    //   TI Mode autosets a lot of things
    //   so do not enable it, unless that is what you want
    //SPI1->CR2 |= (1 << 4); // 1 - SPI TI mode

    // software slave management - SSM bit 9
    SPI1_CR1 |= (1 << 9); // 1- ssm enabled
    // internal slave select - SSI bit 8
    SPI1_CR1 |= (1 << 8); // set ssi to 1

    // master config - MSTR bit 2
    SPI1_CR1 |= (1 << 2); // 1 - master mode

    // enable SPI - SPE bit 6
    SPI1_CR1 |= (1 << 6);

}


void  SPI_SlaveInit(){
// Ensure that CS1, CS2, CS3 are configured
		//Already configured 	
// SPI1 data pins setup

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC_AHB1ENR |= (1 << 0);
    // enable SPI1 clock, bit 12 on APB2ENR
    RCC_APB2ENR |= (1 << 12);

    // set pin modes as alternate mode
    GPIOA_MODER &= 0xFFFF03FF; // Reset bits 10-15 to clear old values
    GPIOA_MODER |= 0x0000A800; // Set pin 5/6/7 to alternate func. mode (0b10)

    // set pin modes as very high speed
    GPIOA_SPEEDR |= 0x0000FC00; // Set pin 5/6/7 to very high speed mode (0b11)
		
    // choose AF5 for SPI1 in Alternate Function registers
    GPIOA_AFR0 |= (0x5 << 20); // for pin 5
    GPIOA_AFR0 |= (0x5 << 24); // for pin 6
    GPIOA_AFR0 |= (0x5 << 28); // for pin 7

    // Disable SPI1 and set the rest (no OR'ing)

    // baud rate - BR[2:0] is bit 5:3
    // fPCLK/2 is selected by default
    //SPI1_CR1   &= ~(7 << 3); // set baud rate to fPCLK/2
		SPI1_CR1   |=  (7 << 3); // set baud rate to fPCLK/256

    // 8/16-bit mode - DFF is bit 11
    // SPI1_CR1 |= (1 << 11); // 1 - 16-bit mode
		SPI1_CR1 &=~(1 << 11); // 1 - 8-bit mode

    // here clk to be high and
    // transmission happens on the falling edge
    // clock polarity - CPOL bit 1
    SPI1_CR1 |= (0 << 1); // clk goes 1 when idle
    // clock phase - CPHA bit 0
    SPI1_CR1 |= (0 << 0); // first clock transaction

    // frameformat - LSBFIRST bit 7, msb/lsb transmit first
    // 0 - MSB transmitted first
    //SPI1->CR1 |= (0 << 7); // 1 - LSB transmitted first

    // frame format - FRF bit 4 on CR2
    // 0 - Motorola mode, 1 - TI mode
    //   TI Mode autosets a lot of things
    //   so do not enable it, unless that is what you want
    //SPI1->CR2 |= (1 << 4); // 1 - SPI TI mode

    // software slave management - SSM bit 9
    SPI1_CR1 |= (1 << 9); // 1- ssm enabled
    // internal slave select - SSI bit 8
    SPI1_CR1 &= ~(1 << 8); // clear ssi to 0

    // slave config - MSTR bit 2
    SPI1_CR1 &= ~(1 << 2); // 0 - slave mode

    // enable SPI - SPE bit 6
    SPI1_CR1 |= (1 << 6);
}

/*
 * Send Receive SPI function
 */
uint8_t SPI_SendRecv(uint8_t data)
{
    // wait until transmit is done (TXE flag)
    while (!(SPI1_SR & (1 << 1)));
    // Send data
    SPI1_DR = data;
    // wait until rx buf is not empty (RXNE flag)
    while (!(SPI1_SR & (1 << 0)));
  
    return SPI1_DR; // read
}



//static __inline uint8_t  SPI_Send_CCC0(uint8_t Byte){
//	if(SPI_CS1_STATUS)
//	SPI1_DR = Byte;
//	while (!(SPI1_RXNE_STATUS));	/* Wait for send to finish */
//	SPI_CS1_SET;
//	return (SPI1_DR);
//}

//static __inline uint8_t  SPI_Send2CPU1(uint8_t Byte){
//	SPI_CS1_RESET;
//	SPI1_DR = Byte;
//	while (!(SPI1_RXNE_STATUS));	/* Wait for send to finish */
//	SPI_CS1_SET;
//	return (SPI1_DR);
//}

//static __inline uint8_t  SPI_Send2CPU2(uint8_t Byte){
//	SPI_CS2_RESET;
//	SPI1_DR = Byte;
//	while (!(SPI1_RXNE_STATUS));	/* Wait for send to finish */
//	//while(! (SPI1_SR & 0x00000001));
//	SPI_CS2_SET;
//	return (SPI1_DR);
//}

// Transmit block of data from specified data buffer and receive data in same buffer
// input:
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: receive only in full duplex mode
void SPI_SendRecvBuf(uint8_t *pBuf, uint8_t length) {
	while (length--) {
		while (!(SPI1_SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI1_DR = *pBuf; // Send byte (TXE cleared)
		while (!(SPI1_SR & SPI_SR_RXNE)); // Wait while RX buffer is empty
		*pBuf++ = SPI1_DR; // Read received byte
	}
	while (SPI1_SR & SPI_SR_BSY); // Wait until the transmission of the last byte is complete
}


/*

A basic Master to slave read or write sequence for I2C follows the following order:

1. Send the START bit (S).
2. Send the slave address (ADDR).
3. Send the Read(R)=1 / Write(W)=0 bit.
4. Wait for/Send an acknowledge bit (A).
5. Send/Receive the data byte (8 bits) (DATA).
6. Expect/Send acknowledge bit (A).
7. Send the STOP bit (P).

*/
void  I2C_Init(){
			
				// I2C_SDA and I2C_SCL  : should not be initialized at this moment  
	      // PH8=I2C_SDA(01), PH7=I2C_SCL(01)
	      //11111111 11111101 01111111 11111111
				RCC_AHB1ENR |= 1<<7;uSec(50);
				GPIOH_MODER  &= ~(3<<14); uSec(50);// clear bits
				GPIOH_MODER  &= ~(3<<16); uSec(50);
				GPIOH_MODER  |=  (1<<14); uSec(50);// set bits (0b01) for I2C SDA, SCL
				GPIOH_MODER  |=  (1<<16); uSec(50);
				GPIOH_OTYPER |= 1 <<7;			//PH7 open drain
				GPIOH_OTYPER |= 1 <<8;			//PH8 open drain
				GPIOH_PUPDR &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
				GPIOH_PUPDR &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
				I2C_SDA_HIGH;
				I2C_SCL_HIGH;
				//HIGH SPEED
				// 00000000 00000001 01000000 00000000
				GPIOH_SPEEDR |= 0x00014000; uSec(50);	
}


void  I2C_Start(){	
/*  I2C Start condition, data line goes low when clock is high */	
		I2C_SDA_HIGH;
		I2C_SCL_HIGH;
    mSec(1);
    I2C_SDA_LOW;
	  mSec(1);
	  I2C_SCL_LOW;
	  mSec(1);
		}

void 	I2C_Stop (){
    /* I2C Stop condition, clock goes high when data is low */
    I2C_SCL_LOW;
	  I2C_SDA_LOW;
	  mSec(1);
    I2C_SCL_HIGH;
	  mSec(1);
    I2C_SDA_HIGH;
    mSec(1);
}

void 	I2C_Write( uint8_t data){
	uint8_t outBits;
	//unsigned char inBit;
	
 	/* 8 bits */
	for(outBits = 0; outBits < 8; outBits++) 
	{
	    if((data & 0x80)==0)
		    I2C_SDA_LOW;
			else
		    I2C_SDA_HIGH;
	      
			data  <<= 1;

		/* Generate clock for 8 data bits */
		I2C_SCL_HIGH;
		mSec(1);	
		I2C_SCL_LOW;
		}
	
	mSec(1);
	mSec(1);
	/* Generate clock for ACK bit */
		I2C_SCL_HIGH;
		mSec(1);	
		I2C_SCL_LOW;
}

/* LCD Implementation using I2C3
   LCD  D7 D6 D5 D4 D3 D2 D1 D0
ccccc        d3 d2 d1 d0 nc en rw rs
*/
void  LCD_Send_cmd(uint8_t cmd){
	char  data_u, data_l;
	uint8_t data_t0,data_t1,data_t2,data_t3;
	data_u = cmd & 0xf0;       // select only upper nibble
	data_l = (cmd << 4) & 0xf0;// select only lower nibble
	data_t0 = data_u | 0x04;   // en=1 rs=0
	data_t1 = data_u ;         // en=0 rs=0
	data_t2 = data_l | 0x04;   // en=1 rs=0
	data_t3 = data_l;				   // en=0 rs=0	
	I2C_Start();
	I2C_Write(LCD_Slave_Addr<<1);// 8th bit = 0 for writing
	I2C_Write(data_t0);
	I2C_Write(data_t1);
	I2C_Write(data_t2);
	I2C_Write(data_t3);
	I2C_Stop();
}
void  LCD_Send_data(uint8_t data){
  char  data_u, data_l;
	uint8_t data_t0,data_t1,data_t2,data_t3;
	data_u = data & 0xf0; // select only upper nibble
	data_l = (data << 4) & 0xf0;// select only lower nibble
	data_t0 = data_u | 0x05; // en=1 rs=1
	data_t1 = data_u | 0x01; // en=0 rs=1
	data_t2 = data_l | 0x05; // en=1 rs=1
	data_t3 = data_l | 0x01; // en=0 rs=1
	I2C_Start();
	I2C_Write(LCD_Slave_Addr<<1);// 8th bit = 0 for writing
	I2C_Write(data_t0);
	I2C_Write(data_t1);
	I2C_Write(data_t2);
	I2C_Write(data_t3);
	I2C_Stop();
}

void  LCD_Init(void){
	/*The serial LCD can make backlight On or Off by sending special command character 0xFF(decimal
255) followed by character 0x03(decimal 3) for Backlight ON or followed by character 0x04(decimal
4) for backlight OFF. Default value is ON. This setting is stored in internal EEPROM and loaded
during power up. */
/*	Working code below */
//	mSec(200);
//	LCD_Send_cmd(0xff);
//	mSec(15);//15msec
//	LCD_Send_cmd(0x33);
//	mSec(5);//4.1msec
//	LCD_Send_cmd(0x32);
//	uSec(100);//100usec
//	LCD_Send_cmd(0x28);
//	uSec(4);
//	LCD_Send_cmd(0x0C);
//	uSec(4);
//	LCD_Send_cmd(0x06);
//	uSec(4);
//	
	
	mSec(200);
	LCD_Send_cmd(0xff);
	mSec(15);//15msec
	LCD_Send_cmd(0x38);// may be 0x33 be OK
	mSec(5);//4.1msec
	LCD_Send_cmd(0x32);
	uSec(100);//100usec
	LCD_Send_cmd(0x28);
	uSec(4);
  LCD_Send_cmd(0x0C);
	uSec(4);
	LCD_Send_cmd(0x06);
	uSec(4);
	LCD_Send_cmd(0x80);
	uSec(4);
	}


void  LCD_BacklightON(void){
  I2C_Start();
  I2C_Write(LCD_Slave_Addr<<1);
  I2C_Write(LCD_BACKLIGHT);
  I2C_Stop();
}
void  LCD_BacklightOFF(void){
  I2C_Start();
  I2C_Write(LCD_Slave_Addr<<1);
  I2C_Write(LCD_NOBACKLIGHT);
  I2C_Stop();
}
	
void  LCD_Send_string_Line1(char *str){
	// I2C_SDA and I2C_SCL  : should not be initialized at this moment  
	      // PH8=I2C_SDA(01), PH7=I2C_SCL(01)
	      //11111111 11111101 01111111 11111111
				RCC_AHB1ENR |= 1<<7;uSec(50);// Enable Clock
				
	GPIOH_MODER &= ~(3 << 14);uSec(50);	//clear bits 14 & 15 (PH7)
  GPIOH_MODER |= 1 << 14;uSec(50);			//MODER7[1:0] = 10 bin (PH7)
  GPIOH_MODER &= ~(3 << 16);uSec(50);	//clear bits 16 & 17 (PH8)
  GPIOH_MODER |= 1 << 16;uSec(50);			//MODER8[1:0] = 10 bin (PH7)
	LCD_Send_cmd(0x80);
	uSec(4);
	//LCD_Clear();
	while(*str) LCD_Send_data(*str++);
	LCD_BacklightON();
	GPIOH_MODER  &= ~(3<<14); uSec(50);// clear bits
	GPIOH_MODER  &= ~(3<<16); uSec(50);
	RCC_AHB1ENR &= ~(1<<7);uSec(50); // Disable Clock
	}	
void  LCD_Send_string_Line2(char *str){
	RCC_AHB1ENR |= 1<<7;uSec(50); // Enable Clock
	GPIOH_MODER &= ~(3 << 14);uSec(50);	//clear bits 14 & 15 (PH7)
  GPIOH_MODER |= 1 << 14;uSec(50);			//MODER7[1:0] = 10 bin (PH7)
  GPIOH_MODER &= ~(3 << 16);uSec(50);	//clear bits 16 & 17 (PH8)
  GPIOH_MODER |= 1 << 16;uSec(50);			//MODER8[1:0] = 10 bin (PH7)
	LCD_Send_cmd(0xc0);
	uSec(4);
	//LCD_Clear();
	while(*str) LCD_Send_data(*str++);
	LCD_BacklightON();
	GPIOH_MODER  &= ~(3<<14); uSec(50);// clear bits
	GPIOH_MODER  &= ~(3<<16); uSec(50);
	RCC_AHB1ENR &= ~(1<<7);uSec(50); // Disable clock
}	
void  LCD_Send_string(char *str){
	while(*str) LCD_Send_data(*str++);
}
void  LCD_Clear(){
	LCD_Send_cmd(0x01);
	mSec(2);
}
void  uSec(unsigned int time){
	unsigned int ii;
	for(ii=0; ii<time; ii++);
}

void	mSec(unsigned int t ) {
unsigned int jj;
	int msec = t*1000;
	for (jj=0; jj < msec; jj++);
}
