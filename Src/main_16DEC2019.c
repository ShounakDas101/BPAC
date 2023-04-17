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

/*                                             */

/*                                             */


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
char LCD_MSG[16] ="XXXXXXXXXXXXXXXX";

uint8_t boardName;
uint8_t msgCCC0_CPU1;
uint8_t msgCCC0_CPU2;
uint8_t msgCCC0_CPU3;
uint8_t msgCPU1_CCC0;
uint8_t msgCPU2_CCC0;
uint8_t msgCPU3_CCC0;



void	SystemInit(void);
void  IdentifyBoard(void);
void  CCC0IO_Init(void);
void  CPUxIO_Init(void);
void  UART2_Init(void);
//void  SPI_Init(void); //CPOL=1, CPHA=1
//void  WaitForRisingEdge(void);
//uint8_t SPI_MasterTransfer(uint8_t byte);
//uint8_t SPI_SlaveTransfer(uint8_t data);
//uint8_t SPI_Send_CPU1(uint8_t);
//void  I2C_Init(void);
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
msgCCC0_CPU1 = 0x00;
msgCCC0_CPU2 = 0x00;
msgCCC0_CPU3 = 0x00;
msgCPU1_CCC0 = 0x00;
msgCPU2_CCC0 = 0x00;
msgCPU3_CCC0 = 0x00;
	
	
SystemInit();
IdentifyBoard();
		
  if(boardName==CCC0){
	CCC0IO_Init();	
	UART2_Init();	
	LCD_Init();
	LCD_Send_string_Line1("AKG Electrinics.");	
	LCD_Send_string_Line2("Indian Railways.");
	SPI_CS1_RESET;uSec(20);		
	SPI_CS2_RESET;uSec(20);
	SPI_CS3_RESET;uSec(20);	
				
	while(1){
		msgCCC0_CPU1=0x46; // message to send
		//LCD_Send_string_Line1("Chip Select high");
		SPI_CS1_SET;uSec(20);mSec(2000);
		SPI_CS2_SET;uSec(20);mSec(2000);
		SPI_CS3_SET;uSec(20);mSec(2000);
		mSec(5000);
		//LCD_Send_string_Line1("Chip Select low ");
		SPI_CS1_RESET;uSec(20);mSec(2000);
		SPI_CS2_RESET;uSec(20);mSec(2000);
		SPI_CS3_RESET;uSec(20);mSec(2000);
		mSec(5000);
		
		
		}
	}
	else if(boardName==CPU1)
	{
		CPUxIO_Init();
		mSec(5000);
		LCD_Send_string_Line2(CPU1_ready);
		RCC_AHB1ENR |= 1 << 3;uSec(50);
	  GPIOD_MODER  = 0X55000055;//0xFFFEBFFF;
	  //uSec(50);
		while(1){
			if((SPI_CS1_STATUS)!=0)
			{
				// Message received and send
				//msgCCC0_CPU1=SPI_SlaveTransfer(msgCPU1_CCC0);
				// action taken
				GPIOD_ODR = 0x500a;break;
//				LFR_RESET;uSec(50);
//				TCFXR_RESET;uSec(50);
//				TCFCR_SET;uSec(50);
//				TGTYR_RESET;uSec(50);
//				ASGNCPR_SET;uSec(50);
//				TGTZR_RESET;uSec(50);
//				BLR_SET;uSec(50);
				// reported to LCD
				LCD_Send_string_Line2("CPU1_HIGH.......");
				
			}
			else//((SPI_CS1_STATUS)==0)
			{
			// read all relay status
				GPIOD_ODR = 0xa005;break;
//			  LFR_SET;uSec(50);
//				TCFXR_SET;uSec(50);
//				TCFCR_RESET;uSec(50);
//				TGTYR_SET;uSec(50);
//				ASGNCPR_RESET;uSec(50);
//				TGTZR_SET;uSec(50);
//				BLR_RESET;uSec(50);
			  LCD_Send_string_Line2("CPU1_LOW .......");
				
			}
		}
		

	}
	
	else if(boardName==CPU2){
		CPUxIO_Init();
		mSec(5000);
		LCD_Send_string_Line2(CPU2_ready);
		RCC_AHB1ENR |= 1 << 3;uSec(50);
	  GPIOD_MODER  = 0X55000055;//0xFFFEBFFF;
	  //uSec(50);
    while(1){
			if((SPI_CS2_STATUS)!=0)
			{
				// Message received and send
				//msgCCC0_CPU1=SPI_SlaveTransfer(msgCPU1_CCC0);
				// action taken
				GPIOD_ODR = 0x500a;break;
//				LFR_RESET;uSec(50);
//				TCFXR_RESET;uSec(50);
//				TCFCR_SET;uSec(50);
//				TGTYR_RESET;uSec(50);
//				ASGNCPR_SET;uSec(50);
//				TGTZR_RESET;uSec(50);
//				BLR_SET;uSec(50);
				// reported to LCD
				LCD_Send_string_Line2("CPU1_HIGH.......");
				
			}
			else //((SPI_CS2_STATUS)==0)
			{
				// read all relay status
//				LFR_SET;uSec(50);
//				TCFXR_SET;uSec(50);
//				TCFCR_RESET;uSec(50);
//				TGTYR_SET;uSec(50);
//				ASGNCPR_RESET;uSec(50);
//				TGTZR_SET;uSec(50);
//				BLR_RESET;uSec(50);
				GPIOD_ODR = 0xa005;break;
			  LCD_Send_string_Line2("CPU1_LOW .......");
				
			}
		}

	}
	else if(boardName==CPU3){
		CPUxIO_Init();
		mSec(5000);
		LCD_Send_string_Line2(CPU3_ready);
		RCC_AHB1ENR |= 1 << 3;uSec(50);
	  GPIOD_MODER  = 0X55000055;//0xFFFEBFFF;
	  //uSec(50);
		
		while(1){
			if((SPI_CS3_STATUS)!=0)
			{
				// Message received and send
				//msgCCC0_CPU1=SPI_SlaveTransfer(msgCPU1_CCC0);
				// action taken
				GPIOD_ODR = 0x500a;break;
//				LFR_RESET;uSec(50);
//				TCFXR_RESET;uSec(50);
//				TCFCR_SET;uSec(50);
//				TGTYR_RESET;uSec(50);
//				ASGNCPR_SET;uSec(50);
//				TGTZR_RESET;uSec(50);
//				BLR_SET;uSec(50);
				// reported to LCD
				LCD_Send_string_Line2("CPU1_HIGH.......");
				
			}
			else //((SPI_CS3_STATUS)==0)
			{
				// read all relay status
				GPIOD_ODR = 0x500a;break;
//				LFR_SET;uSec(50);
//				TCFXR_SET;uSec(50);
//				TCFCR_RESET;uSec(50);
//				TGTYR_SET;uSec(50);
//				ASGNCPR_RESET;uSec(50);
//				TGTZR_SET;uSec(50);
//				BLR_RESET;uSec(50);
			  LCD_Send_string_Line2("CPU1_LOW .......");
				}
		}
		
	}
	else{
	while(1);
	}
	

// Endless loop	
while (1);

}// End of main

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

void  IdentifyBoard(){
	RCC_AHB1ENR |= 1 << 0; 	uSec(50);
	// xxxxxx00 00xxxxxx xxxxxxxx xxxxxxxx
	GPIOA_MODER  &= 0xfc3fffff; 	uSec(50); //PA12,PA11 as input boardID
	
	// Check for X0000000 00X00000 of GPIOH_IDR
	     if((GPIOA_IDR & 0x1800) == 0x0000) boardName=CCC0;
	else if((GPIOA_IDR & 0x1800) == 0x1000) boardName=CPU1;
	else if((GPIOA_IDR & 0x1800) == 0x0800) boardName=CPU2;
	else if((GPIOA_IDR & 0x1800) == 0x1800) boardName=CPU3;
	else 																		boardName=XXX0;

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
				This is CCC0
	*/
					RCC_AHB1ENR |= 1 << 0; uSec(50);
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


/*void  SPI_MasterInit(){
// Ensure that CS1, CS2, CS3 are configured
		//Already configured 	
//code to enable Alternate Function - SPI1
RCC_AHB1ENR |= 1 << 0;	//enable clock to GPIOA
RCC_APB2ENR |= 1 << 12;	//clock to SPI1
GPIOA_AFR0 |= (5<< 20);	//enable SPI CLK to PA5
GPIOA_AFR0 |= (5<< 24);	//enable MISO to PA6
GPIOA_AFR0 |= (5<< 28);	//enable MOSI to PA7
GPIOA_MODER &= ~(3 << 10);	//clear bits 10 & 11
GPIOA_MODER |= 2 << 10;	//MODER5[1:0] = 10 bin  AF mode
GPIOA_MODER &= ~(3 << 12);	//clear bits 12 & 13
GPIOA_MODER |= 2 << 12;	//MODER6[1:0] = 10 bin  AF mode
GPIOA_MODER &= ~(3 << 14);	//clear bits 14 & 15
GPIOA_MODER |= 2 << 14;	//MODER7[1:0] = 10 bin  AF mode
SPI1_CR1	 = 0x0003;	//CPOL=1, CPHA=1
SPI1_CR1	|= 1 << 2;	//Master Mode
SPI1_CR1	|= 1<<6;	//SPI enabled
//SPI1_CR1	&= ~(7<<3);	//Use maximum frequency
SPI1_CR1	|= (7<<3);	//Use minimum frequency	
SPI1_CR1	|= 3<<8;	//Soltware disables slave function
SPI1_CR2   = 0x0000;	//Motorola Format -
}


void  SPI_SlaveInit(){
// Ensure that CS1, CS2, CS3 are configured
		//Already configured 	
//code to enable Alternate Function - SPI1
RCC_AHB1ENR |= 1 << 0;	//enable clock to GPIOA
RCC_APB2ENR |= 1 << 12;	//clock to SPI1
GPIOA_AFR0 |= (5<< 20);	//enable SPI CLK to PA5
GPIOA_AFR0 |= (5<< 24);	//enable MISO to PA6
GPIOA_AFR0 |= (5<< 28);	//enable MOSI to PA7
GPIOA_MODER &= ~(3 << 10);	//clear bits 10 & 11
GPIOA_MODER |= 2 << 10;	//MODER5[1:0] = 10 bin  AF mode
GPIOA_MODER &= ~(3 << 12);	//clear bits 12 & 13
GPIOA_MODER |= 2 << 12;	//MODER6[1:0] = 10 bin  AF mode
GPIOA_MODER &= ~(3 << 14);	//clear bits 14 & 15
GPIOA_MODER |= 2 << 14;	//MODER7[1:0] = 10 bin  AF mode
SPI1_CR1	 = 0x0003;	//CPOL=1, CPHA=1
SPI1_CR1	&= ~(1 << 2);	//Slave Mode
SPI1_CR1	|= 1<<6;	//SPI enabled
//SPI1_CR1	&= ~(7<<3);	//Use maximum frequency
SPI1_CR1	|= (7<<3);	//Use minimum frequency	
SPI1_CR1	|= 3<<8;	//Soltware disables slave function
SPI1_CR2   = 0x0000;	//Motorola Format -
}
*/

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



uint8_t  SPI_Send_CPU1(uint8_t Byte){
	SPI_CS1_RESET;
	SPI_CS3_SET;
	SPI_CS2_SET;
	//SPI_CLK_HIGH;
	//SPI_MISO_HIGH;
	//SPI_MOSI_HIGH;
	SPI1_DR = Byte;
	//while(! (SPI1_SR & 0x00000001));
	SPI_CS1_SET;
	SPI_CS3_RESET;
	SPI_CS2_RESET;
	//SPI_CLK_LOW;
	//SPI_MISO_LOW;
	//SPI_MOSI_LOW;
	return 0;//SPI1_DR;
}

//void  I2C_Init(){
////configuring the AF is not required if we do bit banging 
//RCC_AHB1ENR |= 1<<7;				//Clock for GPIOH
//GPIOH_MODER &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
//GPIOH_MODER |= 1 << 14;			//MODER7[1:0] = 10 bin (PH7)
////GPIOH_MODER &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
////GPIOH_MODER |= 1 << 16;			//MODER8[1:0] = 10 bin (PH7)
//GPIOH_OTYPER |= 1 <<7;			//PH7 open drain
//GPIOH_OTYPER |= 1 <<8;			//PH8 open drain
//GPIOH_PUPDR &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
//GPIOH_PUPDR &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
//I2C_SDA_HIGH;
//I2C_SCL_HIGH;	
//}


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
				
	GPIOH_MODER &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
  GPIOH_MODER |= 1 << 14;			//MODER7[1:0] = 10 bin (PH7)
  GPIOH_MODER &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
  GPIOH_MODER |= 1 << 16;			//MODER8[1:0] = 10 bin (PH7)
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
	GPIOH_MODER &= ~(3 << 14);	//clear bits 14 & 15 (PH7)
  GPIOH_MODER |= 1 << 14;			//MODER7[1:0] = 10 bin (PH7)
  GPIOH_MODER &= ~(3 << 16);	//clear bits 16 & 17 (PH8)
  GPIOH_MODER |= 1 << 16;			//MODER8[1:0] = 10 bin (PH7)
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

void WaitForRisingEdge(void) 
{
	uint8_t spi_clk=0;
	// Wait for clock pin to go low
	while(spi_clk) {
		spi_clk= SPI_CLK_STATUS;
		continue;
	}

	// Wait for clock pin to go high
	while(! spi_clk) {
		spi_clk= SPI_CLK_STATUS;
		continue;
	}	
}



/* a byte transfer from master in (0,0) mode */
uint8_t SPI_MasterTransfer(uint8_t byte)
{
uint8_t counter;
uint8_t miso;
	
for(counter = 8; counter; counter--)
{
if (byte & 0x80)
{SPI_MOSI_HIGH;uSec(5);}
else
{SPI_MOSI_LOW;uSec(5);}

byte <<= 1;
SPI_CLK_HIGH; /* a slave latches input data bit */
uSec(5);
miso=SPI_MISO_STATUS;uSec(5);
if (miso)
byte |= 0x01;
SPI_CLK_LOW;//SCK = 0; /* a slave shifts out next output data bit */
uSec(5);
}

return(byte);
}


/* a byte transfer from slave in (0,0) mode */
uint8_t SPI_SlaveTransfer(uint8_t data) {
	uint8_t mosi;
	uint8_t counter = 0;
	//uint8_t data=0;

	/*	 At every rising edge of the clock
	(read the MOSI pin and put your data onto the MISO pin)
	*/
	while(counter < 8) {
	
	WaitForRisingEdge();
		
		// Send bit to master (MSB first)
		if(data & 0x80) {
			SPI_MISO_HIGH;
		}
		else {
			SPI_MISO_LOW;
		}

		// Shuffle data along
		data <<= 1;
		// Read bit from master (MSB first)
		mosi=SPI_MOSI_STATUS;
		if(mosi) {
			data |= 0x01;
		}
		// Next bit
		counter++;
	}	
	return data;
}
