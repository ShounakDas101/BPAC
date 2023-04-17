/*

Initialize processor
Initialize all the necessary peripherals
Test all the peripherals GPIO UART SPI I2C
Read Board selection PINS
Check reset state conditions of the the relays

 Read Board selection pins (2-bits) : BSPINS

 If BSPINS=0x00 Communication Control Card (CCC0)
 If BSPINS=0x01 System 1 Card              (CPU1)
 If BSPINS=0x10 System 2 Card              (CPU2)
 if BSPINS=0x11 System 3 Card              (CPU3)

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
#define INP1_fail   "INPUT CARD FAIL "
#define CMD_fail		"COMMAND    FAIL "
#define RPD_fail		"RESPOND    FAIL "

#define SPI_CR1_SPE		(1 << 6)
#define SPI_SR_RXNE   (1 << 0)
#define SPI_SR_TXE    (1 << 1)
#define SPI_SR_BSY    (1 << 7)

/* USART_DR[8:0]: DR[8:0]: Data value */
 #define USART_DR_MASK                   0x1FF
 
 /** RXNE: Read data register not empty */
 #define USART_SR_RXNE                   (1 << 5)
 
 /** TXE: Transmit data buffer empty */
 #define USART_SR_TXE                    (1 << 7)
 

#define RECEIVING  0xA0
#define SENDING    0x90
//#define BI_OK			 0x80	

const uint8_t CCC0=0;
const uint8_t	CPU1=1;
const uint8_t	CPU2=2;
const uint8_t	CPU3=3;
 
 //OUTPUT AND FEEDBACK RELAYS
const uint8_t LFR    =0x01;
const uint8_t	TCFXR  =0X02;
const uint8_t	TCFCR  =0x04;
const uint8_t	TGTYR  =0x08;
const uint8_t ASGNCPR=0x10;
const uint8_t TGTZR  =0x20;
const uint8_t	BLR    =0x40;
const uint8_t	XXXR   =0x80; // this is only OUTPUT relay
const uint8_t BIPR2  =0x80;

//INPUT RELAYS
const uint8_t POWER  =0x01;
const uint8_t	TGTXR  =0X02;
const uint8_t	CANCO  =0x04;
const uint8_t	TCFR   =0x08;
const uint8_t ASGNCR =0x10;
const uint8_t HSGNCR =0x20;
const uint8_t	BPNR   =0x40;
const uint8_t	BIPR1  =0x80;

const uint8_t SET		 ='S';
const uint8_t RESET  ='R';
const uint8_t RELAY  =0xAA;
const uint8_t READ   =0x55;
const uint8_t HEALTH =0x39;
const uint8_t SYSTEM =0x93;
const uint8_t SEND   =0xA5;
const uint8_t RECV   =0x5A;
const uint8_t BIOK   =0x80;
const uint8_t BELL   =0xBB;
const uint8_t TGTC   ='T';
const uint8_t CLEAR  ='C';

char LCD_MSG[16] ="XXXXXXXXXXXXXXXX";
uint16_t comm_data;
uint32_t relayStatus_all, relayStatus1, relayStatus2, relayStatus3, relayStatus;
//uint8_t INP_F, INP_B, FDB_F, FDB_B;
uint8_t INP_RELAY;
//uint8_t FDB_RELAY;
uint8_t OTP_RELAY;
uint8_t boardName;
uint32_t temp;
uint8_t status;
uint8_t mode;






////////////////////////////////////
// Initialization Code
////////////////////////////////////
void	SystemInit(void);
uint8_t  IdentifyBoard(void);
void  CCC0IO_Init(void);
void  CPUxIO_Init(void);
/* ------------------------------ */

////////////////////////////////////
// UART Code
////////////////////////////////////
void  UART2_Init(void);
uint16_t usart_recv_blocking(void);
void usart_send_blocking(uint16_t data);

/* ------------------------------ */

////////////////////////////////////
// SPI Code
////////////////////////////////////
void  SPI_MasterInit(void); //CPOL=0, CPHA=0
void  SPI_SlaveInit(void); //CPOL=0, CPHA=0
uint8_t SPI_SendRecv(uint8_t);
void SPI_SendRecvBuf(uint8_t *pTXbuf, uint8_t *pRXbuf, uint8_t length);
//uint8_t spi_read(uint8_t);
/* ------------------------------ */

////////////////////////////////////
// I2C LCD Code
////////////////////////////////////
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
/* --------------------------------- */
void	mSec(unsigned int );
void  uSec(unsigned int);

uint32_t  Command(uint8_t cpu, uint8_t relay, uint8_t setreset);
uint8_t  Respond(void);
void  sendingStation(void);
void  recvingStation(void);
uint8_t ErrorHandle(char* error, uint8_t data);
uint8_t  twoOOthree(uint8_t, uint8_t, uint8_t);
void  testCode(void);
uint32_t statusGPIO(void);
void  displayGPIO(uint8_t INP_F, uint8_t INP_B, uint8_t FDB_F, uint8_t FDB_B);
int	main (void) {
  // Initialize clock and system clock speed, flash
SystemInit();
	// Identify the boards
boardName = IdentifyBoard();
	// Initialize i2c communication
I2C_Init();	
	
	switch(boardName){
		case CCC0:{
		  CCC0IO_Init();
			SPI_CS1_SET;//mSec(5000);	
	    SPI_CS2_SET;//mSec(5000);
	    SPI_CS3_SET;//mSec(5000);
			SPI_MasterInit();
			UART2_Init();	
			LCD_Init();
			LCD_Send_string_Line1("AKG Electrinics.");	
			LCD_Send_string_Line2("Kolkata, India..");
      mSec(2000);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LCD_Send_string_Line1("Indian Railways.");
			LCD_Send_string_Line2("UFSBI-Operating.");
			mSec(2000);
									
			LCD_Send_string_Line2("PRESS BI-ON.....");
					do{		
					relayStatus1 = Command(CPU1,RELAY,READ);
					relayStatus2 = Command(CPU2,RELAY,READ);
					relayStatus3 = Command(CPU3,RELAY,READ);
					OTP_RELAY = twoOOthree((uint8_t)relayStatus1, (uint8_t)relayStatus2, (uint8_t)relayStatus3);
					INP_RELAY =	twoOOthree(relayStatus1>>8, relayStatus2>>8, relayStatus3>>8);			
					status = (INP_RELAY & BIPR1) & (OTP_RELAY & BIPR2);						
					//displayGPIO(OTP_RELAY,status,INP_RELAY,status);
					mSec(50);
					}while(status==0x00);
		  LCD_Send_string_Line2("BLOCK INSTRU ON");
			
			// Check link failure if link OK set LFR
    	
			usart_send_blocking(0x46);mSec(200);
			comm_data=usart_recv_blocking();
			LCD_MSG[12]=comm_data;
			LCD_Send_string_Line2(LCD_MSG);
			mSec(2000);	
			relayStatus1 = Command(CPU1,LFR,SET);
			relayStatus2 = Command(CPU2,LFR,SET);
			relayStatus3 = Command(CPU3,LFR,SET);
			LCD_Send_string_Line2("Communication OK");
			
			relayStatus1 = Command(CPU1,TGTZR,SET);
			relayStatus2 = Command(CPU2,TGTZR,SET);
			relayStatus3 = Command(CPU3,TGTZR,SET);
			LCD_Send_string_Line2("TGTZR SET HIGH  ");
					
			relayStatus1 = Command(CPU1,ASGNCPR,SET);
			relayStatus2 = Command(CPU2,ASGNCPR,SET);
			relayStatus3 = Command(CPU3,ASGNCPR,SET);
			LCD_Send_string_Line2("ASGNCPR SET HIGH  ");
							
//			relayStatus1 = Command(CPU1,TCFXR,SET);
//			relayStatus2 = Command(CPU2,TCFXR,SET);
//			relayStatus3 = Command(CPU3,TCFXR,SET);
//			while(1);
			
			// continuous while loop for UFSBI Operation
			while(1){
			
					do{		
					relayStatus1 = Command(CPU1,RELAY,READ);
					relayStatus2 = Command(CPU2,RELAY,READ);
					relayStatus3 = Command(CPU3,RELAY,READ);
					OTP_RELAY = twoOOthree((uint8_t)relayStatus1, (uint8_t)relayStatus2, (uint8_t)relayStatus3);
					INP_RELAY =	twoOOthree(relayStatus1>>8, relayStatus2>>8, relayStatus3>>8);			
					temp=(INP_RELAY & BIPR1) & (OTP_RELAY & BIPR2); // Checking Power OK
					if(temp==BIOK){
							LCD_Send_string_Line2("BI POWER OK..   ");
							status = INP_RELAY;
										if(status==SENDING)  {mode=SEND;}
							else 	if(status==RECEIVING){mode=RECV;}
							else  {mode=0x00;LCD_Send_string_Line1("UFSBI waiting...");}
											}
					else{ 
							LCD_Send_string_Line2("BI FAIL power???");
						  }
					//displayGPIO(OTP_RELAY,status,INP_RELAY,status);
					mSec(50);
					if(mode != 0x00) break;	
			    }while(1);
					
				if(mode==SEND){
					LCD_Send_string_Line1("UFSBI TGT MODE..");
					sendingStation();
				}
				else if(mode==RECV){
					LCD_Send_string_Line1("UFSBI TCF MODE..");
					recvingStation();
				}
				else{
				LCD_Send_string_Line1("Waiting for SM....");
				}
				
		}// end of continuous while
      
			
		}
		case CPU1:{
			CPUxIO_Init();//GPIOD_ODR = 0xF00F;mSec(5000);
			SPI_SlaveInit(); //CPOL=0, CPHA=0
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LFR_RESET;
			TCFXR_RESET;
			TCFCR_RESET;
			TGTYR_RESET;
			ASGNCPR_RESET;
			TGTZR_RESET;
			BLR_RESET;
			XXXR_RESET;
					
			while(1){
				
				while(SPI_CS1_STATUS);
				Respond();
				while(!SPI_CS1_STATUS);
				//LCD_Send_string_Line2("TEST #1 CPU");	
				}// end of main while loop
			
				while(1);
		}// end of case 1
		
		case CPU2:{
			CPUxIO_Init();//GPIOD_ODR = 0xF00F;mSec(5000);
			SPI_SlaveInit(); //CPOL=0, CPHA=0
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LFR_RESET;
			TCFXR_RESET;
			TCFCR_RESET;
			TGTYR_RESET;
			ASGNCPR_RESET;
			TGTZR_RESET;
			BLR_RESET;
			XXXR_RESET;
      									
			while(1){
				
				while(SPI_CS2_STATUS);
				Respond();
				while(!SPI_CS2_STATUS);
				//LCD_Send_string_Line2("TEST #2 CPU");				
				}// end of main while loop
			 while(1);
		}//end of case 2
		
		case CPU3:{
			CPUxIO_Init();//GPIOD_ODR = 0xF00F;mSec(5000);
			SPI_SlaveInit(); //CPOL=0, CPHA=0
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
			LFR_RESET;
			TCFXR_RESET;
			TCFCR_RESET;
			TGTYR_RESET;
			ASGNCPR_RESET;
			TGTZR_RESET;
			BLR_RESET;
			XXXR_RESET;
			
					
			while(1){
			
			while(SPI_CS3_STATUS);
			Respond();
			while(!SPI_CS3_STATUS);
			//LCD_Send_string_Line2("TEST #3 CPU");			
			}// end of main while loop
		 while(1);
		}// End of case 3
		
		default :
			while(1);
	
	}// end of switch-case 

// Endless loop	
while (1);

}// End of main



/*         Command()                          
   input parameters: 
					uint8_t CPUx    : command to the CPU number
					uint8_t relay   : relay to be selected
					uint8_t setreset: SET/RESET 
	 return value:
					uint32_t status : sends all the relay front status duplicate 4x8 bits
*/


uint32_t Command(uint8_t cpu, uint8_t relay, uint8_t setreset){
	uint32_t relayStatus;
	uint8_t  inpData1,  inpData2,  inpData3;
	uint8_t  fdbData1,  fdbData2,  fdbData3;
	uint8_t  inpRelay;
	uint8_t  fdbRelay;
	
	if(cpu==CPU1){
	    SPI_CS1_RESET;uSec(500);// wait , CPU1 reads all the relay status 
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
		  inpData1=SPI_SendRecv(setreset);
			inpData2=SPI_SendRecv(setreset);
			inpData3=SPI_SendRecv(setreset);
			fdbData1=SPI_SendRecv(relay);
			fdbData2=SPI_SendRecv(relay);
		  fdbData3=SPI_SendRecv(relay);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
      SPI_CS1_SET;uSec(500);
		  //inpData1=0x54; inpData2=0x56; inpData3=0x54;
		  //fdbData1=0x34;fdbData2=0x29;fdbData3=0x34;
		  
	}
	else if(cpu==CPU2){						
			SPI_CS2_RESET;uSec(500);// wait , CPU2 reads all the relay status
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			inpData1=SPI_SendRecv(setreset);
			inpData2=SPI_SendRecv(setreset);
			inpData3=SPI_SendRecv(setreset);
			fdbData1=SPI_SendRecv(relay);
			fdbData2=SPI_SendRecv(relay);
		  fdbData3=SPI_SendRecv(relay);
   		SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
		  SPI_CS2_SET;uSec(500);
		  //inpData1=0x54; inpData2=0x56; inpData3=0x54;
		  //fdbData1=0x34;fdbData2=0x29;fdbData3=0x34;
		  //inpRelay=inpData1;//twoOOthree(inpData1,inpData2,inpData3);
		  //fdbRelay=fdbData1;//twoOOthree(fdbData1,fdbData2,fdbData3);
		  			
	}	
  else if(cpu==CPU3){
			SPI_CS3_RESET;uSec(500);// wait , CPU3 reads all the relay status
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
			inpData1=SPI_SendRecv(setreset);
			inpData2=SPI_SendRecv(setreset);
			inpData3=SPI_SendRecv(setreset);
			fdbData1=SPI_SendRecv(relay);
			fdbData2=SPI_SendRecv(relay);
		  fdbData3=SPI_SendRecv(relay);
  		SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
		  SPI_CS3_SET;uSec(500);
		  //inpData1=0x54;inpData2=0x56; inpData3=0x54;
		  //fdbData1=0x34;fdbData2=0x29;fdbData3=0x34;
  	  //inpRelay=inpData1;//twoOOthree(inpData1,inpData2,inpData3);
  	  //fdbRelay=fdbData1;//twoOOthree(fdbData1,fdbData2,fdbData3);
		  
			
	}
	else{
	ErrorHandle(CMD_fail,'X');
	}
	
	    inpRelay= twoOOthree(inpData1,inpData2,inpData3);
		  fdbRelay= twoOOthree(fdbData1,fdbData2,fdbData3);
		  //INP_RELAY=inpRelay;
		  //OTP_RELAY=fdbRelay;
		  //testCode();		  			
	
	  //inpRelay=0x57; fdbRelay=0x65;
	  relayStatus=(inpRelay<<24) + (fdbRelay<<16) + (inpRelay<<8) + fdbRelay ;
    
return relayStatus;			
	
}

/*   Respond()    */
  uint8_t Respond(void){
	uint32_t relay;
	uint8_t inF =0x00;
	uint8_t inB =0x00;
	uint8_t fbF =0x00;
	uint8_t fbB =0x00;
		
	uint8_t out1,out2,out3,out;   // relay output Byte
	uint8_t sr1,sr2,sr3,setreset; // Set Reset Byte
  
	// read all the relay status	
	//retValue = inputFront | (fedbkFront<<8) | (inputBack<<16) | (fedbkBack<<24);
	relay = statusGPIO();
	inF =	relay;
	fbF = relay>>8;
  inB = relay>>16;
  fbB = relay>>24;
		
	//if(inF != (~inB))
	//ErrorHandle(INP1_fail,'F');
	//if(fbF != (~fbB))
	//ErrorHandle(INP1_fail,'B');
	//inputFront=0xf1;
	//fedbkFront=0x5e;
	
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  sr1 =SPI_SendRecv(inF);
				sr2 =SPI_SendRecv(inF);
				sr3 =SPI_SendRecv(inF);
				out1=SPI_SendRecv(fbF);
				out2=SPI_SendRecv(fbF);
				out3=SPI_SendRecv(fbF);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				out= twoOOthree(out1,out2,out3);
				setreset=twoOOthree(sr1,sr2,sr3);
			  
				// Set or Reset output relays as per outputCCC0_CPUx value
				if(setreset==SET)
				{ 
					if		 (out==LFR)  		  LFR_SET;
					else if(out==TCFXR) 		TCFXR_SET; 
				  else if(out==TCFCR)     TCFCR_SET;
				  else if(out==TGTYR)     TGTYR_SET;
				  else if(out==ASGNCPR)   ASGNCPR_SET;
				  else if(out==TGTZR)     TGTZR_SET;
				  else if(out==BLR)       BLR_SET;
				  else if(out==XXXR)      XXXR_SET;
					else ErrorHandle(RPD_fail,'?');
					
				}  
				else if(setreset==RESET){
			
					   if(out==LFR) 			  LFR_RESET;	
				else if(out==TCFXR) 			TCFXR_RESET;
				else if(out==TCFCR) 			TCFCR_RESET;
				else if(out==TGTYR) 			TGTYR_RESET;
				else if(out==ASGNCPR) 		ASGNCPR_RESET;
				else if(out==TGTZR) 			TGTZR_RESET;
				else if(out==BLR) 				BLR_RESET;
				else if(out==XXXR) 			  XXXR_RESET;
				else ErrorHandle(RPD_fail,'!');
				}
				
return setreset;
      				
}


void  sendingStation(void){
/*
	Normal condition : Sending Station :
	                    ASGNCR HIGH, 
											[D]AZTR HIGH
	line closed: yellow, SNK :YELLOW, LINE FREE GREEN LSS RED
  TGTZR SET HIGH
	
	Operation Start:
	1. BELL 
	2. TGT + BELL
	3. IF TGTXR pickup ask LINECLEAR
	4. set TGTYR HIGH
	5. delay
	6. reset TGTZR LOW
	5. delay 2sec drop TGTYR low
	6. 
	
	
	
	*/
	
// Check link failure if link OK set LFR	
			relayStatus1 = Command(CPU1,TGTZR,SET);
			relayStatus2 = Command(CPU2,TGTZR,SET);
			relayStatus3 = Command(CPU3,TGTZR,SET);
			LCD_Send_string_Line2("NORMAL POSITION ");	
	
	    LCD_Send_string_Line2("pressBELL button");
					
					do{		
			relayStatus1 = Command(CPU1,RELAY,READ);
			relayStatus2 = Command(CPU2,RELAY,READ);
			relayStatus3 = Command(CPU3,RELAY,READ);
			OTP_RELAY = twoOOthree((uint8_t)relayStatus1, (uint8_t)relayStatus2, (uint8_t)relayStatus3);
      INP_RELAY =	twoOOthree(relayStatus1>>8, relayStatus2>>8, relayStatus3>>8);			
      status = INP_RELAY &  BPNR  ;						
			//displayGPIO(OTP_RELAY,status,INP_RELAY,status);
				mSec(50);	
					}while(status==0x00);
					
			//Send Bell Code
					
			LCD_Send_string_Line1("bell code sent..");
      
			LCD_Send_string_Line2("PRESS TGT+BELL..");						
			  do{		
			relayStatus1 = Command(CPU1,RELAY,READ);
			relayStatus2 = Command(CPU2,RELAY,READ);
			relayStatus3 = Command(CPU3,RELAY,READ);
			OTP_RELAY = twoOOthree((uint8_t)relayStatus1, (uint8_t)relayStatus2, (uint8_t)relayStatus3);
      INP_RELAY =	twoOOthree(relayStatus1>>8, relayStatus2>>8, relayStatus3>>8);			
      status = INP_RELAY &  TGTXR  ;						
			//displayGPIO(OTP_RELAY,status,INP_RELAY,status);
				mSec(50);	
					}while(status==0x00);
			LCD_Send_string_Line2("TGT BELL pressed");
			
			relayStatus1 = Command(CPU1,TGTYR,SET);
			relayStatus2 = Command(CPU2,TGTYR,SET);
			relayStatus3 = Command(CPU3,TGTYR,SET);
			mSec(2000);
			relayStatus1 = Command(CPU1,TGTZR,RESET);
			relayStatus2 = Command(CPU2,TGTZR,RESET);
			relayStatus3 = Command(CPU3,TGTZR,RESET);	
			mSec(1000);		
			relayStatus1 = Command(CPU1,TGTYR,RESET);
			relayStatus2 = Command(CPU2,TGTYR,RESET);
			relayStatus3 = Command(CPU3,TGTYR,RESET);
			mSec(3000);
				
					
					
			mSec(45000);
					
      relayStatus1 = Command(CPU1,TGTZR,SET);
			relayStatus2 = Command(CPU2,TGTZR,SET);
			relayStatus3 = Command(CPU3,TGTZR,SET);
      mSec(5000);

			
      		
			SPI_CS1_SET;uSec(50);	
      SPI_CS2_SET;uSec(50);
      SPI_CS3_SET;uSec(50);			
			
}
void  recvingStation(void){
	
	    relayStatus1 = Command(1,ASGNCPR,SET);
			relayStatus2 = Command(2,ASGNCPR,SET);
			relayStatus3 = Command(3,ASGNCPR,SET);
			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);
	    mSec(10000);
	    // wait for BELL code from sending station
	    // temp=usart_recv_blocking();
	temp=BELL;
	if(temp==BELL){
		  relayStatus1 = Command(1,BLR,SET);
			relayStatus2 = Command(2,BLR,SET);
			relayStatus3 = Command(3,BLR,SET);
			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);
	}
	mSec(2000);	
	
	// wait for TGT code from sending station
	    // temp=usart_recv_blocking();
	temp=TGTC;
	if(temp==TGTC){
		  relayStatus1 = Command(1,TCFXR,SET);
			relayStatus2 = Command(2,TCFXR,SET);
			relayStatus3 = Command(3,TCFXR,SET);
			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);
		 }
	mSec(5000);
	// Send LINE CLEAR GRANTED CODE to sending station
	    // usart_send_blocking(CLEAR);	
			
		  relayStatus1 = Command(1,TCFXR,RESET);
			relayStatus2 = Command(2,TCFXR,RESET);
			relayStatus3 = Command(3,TCFXR,RESET);
		 mSec(3000);
	
if(temp==BELL){
		  relayStatus1 = Command(1,BLR,RESET);
			relayStatus2 = Command(2,BLR,RESET);
			relayStatus3 = Command(3,BLR,RESET);
			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);
	}

			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);
      relayStatus1 = Command(1,ASGNCPR,RESET);
			relayStatus2 = Command(2,ASGNCPR,RESET);
			relayStatus3 = Command(3,ASGNCPR,RESET);
			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);
			
mSec(3000);
      relayStatus1 = Command(1,TGTZR,SET);
			relayStatus2 = Command(2,TGTZR,SET);
			relayStatus3 = Command(3,TGTZR,SET);
			relayStatus=twoOOthree(relayStatus1,relayStatus2,relayStatus3);	
while(1);		 
			
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
	
	// 
	/*
	For the STM33F407 using USART2 with a 
	peripheral clock frequency of 36MHz 
	to obtain a baud rate of 115.2k need:

BRR = fclk / (16 x Baud) = 36/(16 x 0.1152) =19.53 decimal
DIV_Mantissa = 19 dec = 0x13
DIV_Fraction = 0.53 dec = 16 x 0.53 = 8.48 ~ 8
USART_BRR = 0x138
NKDas calculation BRR= 4/(16 * 0.0024)
*/
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
	GPIOB_MODER  &= 0x0000ffff;uSec(50);
	GPIOB_MODER  |= 0xc30c0000;uSec(50);
	
/*	GPIOC		PC6		PC5		PC3		PC1		PC0
  					NC		NC		BPNR	BPNR	HSGNCR
						IN		IN		IN		IN		IN
						xxxxxxxxx xxxxxxxx xx0000xx 00xx0000
*/
  RCC_AHB1ENR |= 1 << 2;uSec(50);
	GPIOC_MODER  = 0xffffc330;uSec(50);
	

/*GPIOD			PD15	PD14	PD13	PD12			PD3		PD2		PD1		PD0	
  					OUT		BLR		TGTZR	ASGNCPR		TGTYR	TCFCR	TCFXR	LFR
  					OUT		OUT		OUT		OUT				OUT		OUT		OUT		OUT
						01010101 xxxxxxxx xxxxxxxx 01010101
*/
	RCC_AHB1ENR |= 1 << 3;uSec(50);
	GPIOD_MODER  = 0x55ffff55;uSec(50);
	
					
/*	GPIOE		PE15	PE14	PE12	PE11	PE9		  PE8
  					LFR 	TCFXR TCFCR	TGTYR	ASGNCPR TGTZR  all input
  					PE7		PE6		PE4		PE3		PE1			PE0
  					PWR		TGTXR	CANCO	TCFR	ASGNCR	HSGNCR  all input
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
    while (!(SPI1_SR & SPI_SR_TXE));
    // Send data
    SPI1_DR = data;
    // wait until rx buf is not empty (RXNE flag)
    while (!(SPI1_SR & SPI_SR_RXNE));
    while (SPI1_SR & SPI_SR_BSY); // Wait until the transmission of the last byte is complete
    return SPI1_DR; // read
}


// Transmit block of data from specified data buffer and receive data in same buffer
// input:
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: receive only in full duplex mode

void SPI_SendRecvBuf(uint8_t *pTXbuf, uint8_t *pRXbuf, uint8_t length) {
	while (length--) {
		while (!(SPI1_SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI1_DR = *pTXbuf++; // Send byte (TXE cleared)
		while (!(SPI1_SR & SPI_SR_RXNE)); // Wait while RX buffer is empty
		*pRXbuf++ = SPI1_DR; // Read received byte
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

/*    2OO3 logic      
parameters:  A B C
return    :  2/3 output
*/
uint8_t twoOOthree(uint8_t A ,uint8_t B,uint8_t C){
if((A !=B ) && (B != C) && (C!=A))
	return 0xff;
else
	return ((A&B)|(B&C)|(C&A));
}

uint32_t twoOOthree32(uint32_t A ,uint32_t B,uint32_t C){
if((A !=B ) && (B != C) && (C!=A))
	return 0xffffffff;
else
	return ((A&B)|(B&C)|(C&A));
}




uint32_t statusGPIO(void){
	uint8_t inputFront=0x00;
	uint8_t inputBack =0x00;
	uint8_t fedbkFront=0x00;
	uint8_t fedbkBack =0x00;
  uint32_t retValue;
		
  if(POWER_FRONT_STATUS) inputFront |= POWER;
		if(TGTXR_FRONT_STATUS) inputFront |= TGTXR;
			if(CANCO_FRONT_STATUS) inputFront |= CANCO;
				if(TCFR_FRONT_STATUS)  inputFront |= TCFR;
					if(ASGNCR_FRONT_STATUS) inputFront|= ASGNCR;
						if(HSGNCR_FRONT_STATUS) inputFront|= HSGNCR;
							if(BPNR_FRONT_STATUS)  inputFront |= BPNR;
								if(BIPR1_FRONT_STATUS) inputFront |= BIPR1;
  inputBack =0x00;
  if(POWER_BACK_STATUS) inputBack |= POWER;
		if(TGTXR_BACK_STATUS) inputBack |= TGTXR;
			if(CANCO_BACK_STATUS) inputBack |= CANCO;
				if(TCFR_BACK_STATUS)  inputBack |= TCFR;
					if(ASGNCR_BACK_STATUS) inputBack|= ASGNCR;
						if(HSGNCR_BACK_STATUS) inputBack|= HSGNCR;
							if(BPNR_BACK_STATUS)  inputBack |= BPNR;
								if(BIPR1_BACK_STATUS) inputBack |= BIPR1;

	if(LFR_FRONT_STATUS) fedbkFront |= LFR;
				if(TCFXR_FRONT_STATUS) fedbkFront |=TCFXR;
						if(TCFCR_FRONT_STATUS) fedbkFront |=TCFCR;
								if(TGTYR_FRONT_STATUS) fedbkFront |=TGTYR;
										if(ASGNCPR_FRONT_STATUS) fedbkFront |=ASGNCPR;
												if(TGTZR_FRONT_STATUS) fedbkFront |=TGTZR;
														if(BLR_FRONT_STATUS) fedbkFront |=BLR;
																if(BIPR2_FRONT_STATUS) fedbkFront |=BIPR2;
		

	if(LFR_BACK_STATUS) fedbkBack |= LFR;
			if(TCFXR_BACK_STATUS) fedbkBack |=TCFXR;
					if(TCFCR_BACK_STATUS) fedbkBack |=TCFCR;
							if(TGTYR_BACK_STATUS) fedbkBack |=TGTYR;
									if(ASGNCPR_BACK_STATUS) fedbkBack |=ASGNCPR;
											if(TGTZR_BACK_STATUS) fedbkBack |=TGTZR;
													if(BLR_BACK_STATUS) fedbkBack |=BLR;
															if(BIPR2_BACK_STATUS) fedbkBack |=BIPR2;
															
  retValue = inputFront | (fedbkFront<<8) | (inputBack<<16) | (fedbkBack<<24);
	return retValue;
}

/*---------------------------------------------------------------------------*/
 /** @brief USART Send Data Word with Blocking
 
 Blocks until the transmit data buffer becomes empty then writes the next data
 word for transmission.
 
 @param[in] usart unsigned 32 bit. USART block register address base @ref
 usart_reg_base
 @param[in] data unsigned 16 bit.
 */
 
 void usart_send_blocking(uint16_t data)
 {
   /* Wait until the data has been transferred into the shift register. */
         while ((UART2_SR & USART_SR_TXE) == 0);
   /* Send data. */
         UART2_DR = (data & USART_DR_MASK);
 }
 
 /*---------------------------------------------------------------------------*/
 /** @brief USART Read a Received Data Word with Blocking.
 
 Wait until a data word has been received then return the word.
 
 @param[in] usart unsigned 32 bit. USART block register address base @ref
 usart_reg_base
 @returns unsigned 16 bit data word.
 */
  
 uint16_t usart_recv_blocking(void)
 {
    /* Wait until the data is ready to be received. */
         while ((UART2_SR & USART_SR_RXNE) == 0);
 
   /* Receive data. */
         return UART2_DR & USART_DR_MASK;
 }
 
 
 
 uint32_t Command_to_CPUx(uint8_t cpu, uint8_t relay, uint8_t setreset){
	uint32_t relayStatus;
//	uint8_t  inpData1,  inpData2,  inpData3;
//	uint8_t  fdbData1,  fdbData2,  fdbData3;
	uint8_t  inpRelay;
	uint8_t  fdbRelay;
	uint8_t  DataSend[6];
  uint8_t  DataRecv[6];	 
	
	if(cpu==CPU1){
	    SPI_CS1_RESET;uSec(500);// wait , CPU1 reads all the relay status 
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI	
		  SPI_SendRecvBuf(DataSend,DataRecv,6);
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
      SPI_CS1_SET;uSec(500);
		  //inpData1=0x54; inpData2=0x56; inpData3=0x54;
		  //fdbData1=0x34;fdbData2=0x29;fdbData3=0x34;
		  inpRelay=DataRecv[0];// twoOOthree(inpData1,inpData2,inpData3);
		  fdbRelay=DataRecv[3];//twoOOthree(fdbData1,fdbData2,fdbData3);
		  //INP_RELAY=inpRelay;
		  //OTP_RELAY=fdbRelay;
		  //testCode();		  			
	}
	else if(cpu==CPU2){						
			SPI_CS2_RESET;uSec(500);// wait , CPU2 reads all the relay status
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			SPI_SendRecvBuf(DataSend,DataRecv,6);		
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI	
		  SPI_CS2_SET;uSec(500);
		  //inpData1=0x54; inpData2=0x56; inpData3=0x54;
		  //fdbData1=0x34;fdbData2=0x29;fdbData3=0x34;
		  inpRelay=DataRecv[0];//twoOOthree(inpData1,inpData2,inpData3);
		  fdbRelay=DataRecv[3];//twoOOthree(fdbData1,fdbData2,fdbData3);
		  			
	}	
  else if(cpu==CPU3){
			SPI_CS3_RESET;uSec(500);// wait , CPU3 reads all the relay status
			SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			SPI_SendRecvBuf(DataSend,DataRecv,6);		
			SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
		  SPI_CS3_SET;uSec(500);
		  //inpData1=0x54;inpData2=0x56; inpData3=0x54;
		  //fdbData1=0x34;fdbData2=0x29;fdbData3=0x34;
  	  inpRelay=DataRecv[0];//twoOOthree(inpData1,inpData2,inpData3);
  	  fdbRelay=DataRecv[3];//twoOOthree(fdbData1,fdbData2,fdbData3);
		  
			
	}
	else{
	ErrorHandle(CMD_fail,'X');
	}
	
	
	  //inpRelay=0x57; fdbRelay=0x65;
	  relayStatus=(inpRelay<<24) + (fdbRelay<<16) + (inpRelay<<8) + fdbRelay ;
    
return relayStatus;			
	
}

/*   Respond()    */
  uint8_t Respond_from_CPUx(void){
	uint32_t relay;
	uint8_t inF =0x00;
	uint8_t inB =0x00;
	uint8_t fbF =0x00;
	uint8_t fbB =0x00;
	uint8_t send[6];	
	uint8_t recv[6];	
	uint8_t out;      // relay output Byte
	uint8_t	setreset; // Set Reset Byte
  
	// read all the relay status	
	//retValue = inputFront | (fedbkFront<<8) | (inputBack<<16) | (fedbkBack<<24);
	relay = statusGPIO();
	inF =	relay;
	fbF = relay>>8;
  inB = relay>>16;
  fbB = relay>>24;
	
  		
	//if(inF != (~inB))
	//ErrorHandle(INP1_fail,'F');
	//if(fbF != (~fbB))
	//ErrorHandle(INP1_fail,'B');
	//inputFront=0xf1;
	//fedbkFront=0x5e;
	
	send[0]=send[1]=send[2]=inF;
	send[3]=send[4]=send[5]=fbF;
	
	
				SPI1_CR1 |= SPI_CR1_SPE;// enable SPI
			  SPI_SendRecvBuf(send,recv,6);
				SPI1_CR1 &= ~SPI_CR1_SPE;// disable SPI
				out= twoOOthree(recv[3],recv[4],recv[5]);
				setreset=twoOOthree(recv[0],recv[1],recv[2]);

				// Set or Reset output relays as per outputCCC0_CPUx value
				if(setreset==SET)
				{ 
					if		 (out==LFR)  		  LFR_SET;
					else if(out==TCFXR) 		TCFXR_SET; 
				  else if(out==TCFCR)     TCFCR_SET;
				  else if(out==TGTYR)     TGTYR_SET;
				  else if(out==ASGNCPR)   ASGNCPR_SET;
				  else if(out==TGTZR)     TGTZR_SET;
				  else if(out==BLR)       BLR_SET;
				  else if(out==XXXR)      XXXR_SET;
					else ErrorHandle(RPD_fail,'?');
					
				}  
				else if(setreset==RESET){
				if     (out==LFR) 			  LFR_RESET;	
				else if(out==TCFXR) 			TCFXR_RESET;
				else if(out==TCFCR) 			TCFCR_RESET;
				else if(out==TGTYR) 			TGTYR_RESET;
				else if(out==ASGNCPR) 		ASGNCPR_RESET;
				else if(out==TGTZR) 			TGTZR_RESET;
				else if(out==BLR) 				BLR_RESET;
				else if(out==XXXR) 			  XXXR_RESET;
				else ErrorHandle(RPD_fail,'!');
				}
				
return setreset;
}

uint8_t ErrorHandle(char* error, uint8_t data){
	
error[16]=data;
LCD_Send_string_Line2(error);
return 0;
}

void  testCode(void){
	uint8_t i;
	char LCD_Char[16]="ZZZZZZZZZZZZZZZZ";
	for(i=0;i<8;i++)	LCD_Char[7-i]   = 0x30 + ((OTP_RELAY>>i) & 0x1);
	for(i=0;i<8;i++)	LCD_Char[15-i]  = 0x30 + ((INP_RELAY>>i) & 0x1);
	LCD_Send_string_Line1("LSB OTP><INP MSB");
	LCD_Send_string_Line2(LCD_Char);
}


void  displayGPIO(uint8_t INP_F, uint8_t INP_B, uint8_t FDB_F, uint8_t FDB_B){
	uint8_t i;
	char LCD_Char[16]="ZZZZZZZZZZZZZZZZ";
	for(i=0;i<8;i++)	LCD_Char[7-i]   = 0x30 + ((INP_F>>i) & 0x1);
	for(i=0;i<8;i++)	LCD_Char[15-i]  = 0x30 + ((FDB_F>>i) & 0x1);
	LCD_Send_string_Line1(LCD_Char);
	for(i=0;i<8;i++)	LCD_Char[7-i]   = 0x30 + ((INP_B>>i) & 0x1);
	for(i=0;i<8;i++)	LCD_Char[15-i]  = 0x30 + ((FDB_B>>i) & 0x1);
	LCD_Send_string_Line2(LCD_Char);
	
}
