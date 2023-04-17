
typedef unsigned char 	uint8_t;
typedef unsigned short 	uint16_t;
typedef unsigned int		uint32_t;

#define REG_32(ADDRESS) (*((volatile unsigned int *)(ADDRESS)))
// Clock Control Register
#define RCC_BASE 					0x40023800


#define GPIOA_BASE        0x40020000  // UART2, SPI1
#define GPIOB_BASE        0x40020400  // Feedback
#define GPIOC_BASE        0x40020800  // Input
#define GPIOD_BASE        0x40020C00  // Output
#define GPIOE_BASE        0x40021000	// Feedback and Input
#define GPIOF_BASE        0x40021400  // Input
#define GPIOG_BASE        0x40021800  // Feedback
#define GPIOH_BASE        0x40021C00  // I2C3 BoardID_bits
#define GPIOI_BASE        0x40022000

#define RCC_CR						REG_32(RCC_BASE+0x00) 
#define RCC_PLLCFGR				REG_32(RCC_BASE+0x04)
#define RCC_CFGR					REG_32(RCC_BASE+0x08)
#define RCC_CIR						REG_32(RCC_BASE+0x0C)
#define	RCC_AHB1ENR				REG_32(RCC_BASE+0x30)
#define RCC_APB1ENR				REG_32(RCC_BASE+0x40)
#define RCC_APB2ENR				REG_32(RCC_BASE+0x44)

#define GPIOA_MODER				REG_32(GPIOA_BASE+0x00)
#define GPIOA_OTYPER			REG_32(GPIOA_BASE+0x04)
#define GPIOA_SPEEDR			REG_32(GPIOA_BASE+0x08)
#define GPIOA_PUPDR				REG_32(GPIOA_BASE+0x0C)
#define GPIOA_IDR					REG_32(GPIOA_BASE+0x10)
#define GPIOA_ODR					REG_32(GPIOA_BASE+0x14)
#define GPIOA_AFR0				REG_32(GPIOA_BASE+0x20)
#define GPIOA_AFR1				REG_32(GPIOA_BASE+0x24)

#define GPIOB_MODER				REG_32(GPIOB_BASE+0x00)
#define GPIOB_OTYPER			REG_32(GPIOB_BASE+0x04)
#define GPIOB_IDR					REG_32(GPIOB_BASE+0x10)
#define GPIOB_ODR					REG_32(GPIOB_BASE+0x14)

#define GPIOC_MODER				REG_32(GPIOC_BASE+0x00)
#define GPIOC_OTYPER			REG_32(GPIOC_BASE+0x04)
#define GPIOC_IDR					REG_32(GPIOC_BASE+0x10)
#define GPIOC_ODR					REG_32(GPIOC_BASE+0x14)

#define GPIOD_MODER				REG_32(GPIOD_BASE+0x00)
#define GPIOD_OTYPER			REG_32(GPIOD_BASE+0x04)
#define GPIOD_IDR					REG_32(GPIOD_BASE+0x10)
#define GPIOD_ODR					REG_32(GPIOD_BASE+0x14)
#define GPIOD_BSRR				REG_32(GPIOD_BASE+0x18)

#define GPIOE_MODER				REG_32(GPIOE_BASE+0x00)
#define GPIOE_OTYPER			REG_32(GPIOE_BASE+0x04)
#define GPIOE_IDR					REG_32(GPIOE_BASE+0x10)
#define GPIOE_ODR					REG_32(GPIOE_BASE+0x14)


#define GPIOF_MODER				REG_32(GPIOF_BASE+0x00)
#define GPIOF_OTYPER			REG_32(GPIOF_BASE+0x04)
#define GPIOF_SPEEDR			REG_32(GPIOF_BASE+0x08)
#define GPIOF_IDR					REG_32(GPIOF_BASE+0x10)
#define GPIOF_ODR					REG_32(GPIOF_BASE+0x14)

#define GPIOG_MODER				REG_32(GPIOG_BASE+0x00)
#define GPIOG_OTYPER			REG_32(GPIOG_BASE+0x04)
#define GPIOG_IDR					REG_32(GPIOG_BASE+0x10)
#define GPIOG_ODR					REG_32(GPIOG_BASE+0x14)

#define GPIOH_MODER				REG_32(GPIOH_BASE+0x00)
#define GPIOH_OTYPER			REG_32(GPIOH_BASE+0x04)
#define GPIOH_SPEEDR			REG_32(GPIOH_BASE+0x08)
#define GPIOH_PUPDR				REG_32(GPIOH_BASE+0x0C)
#define GPIOH_IDR					REG_32(GPIOH_BASE+0x10)
#define GPIOH_ODR					REG_32(GPIOH_BASE+0x14)
#define GPIOH_AFR0				REG_32(GPIOH_BASE+0x20)
#define GPIOH_AFR1				REG_32(GPIOH_BASE+0x24)


#define GPIOI_MODER				REG_32(GPIOI_BASE+0x00)
#define GPIOI_OTYPER			REG_32(GPIOI_BASE+0x04)
#define GPIOI_IDR					REG_32(GPIOI_BASE+0x10)
#define GPIOI_ODR					REG_32(GPIOI_BASE+0x14)


#define FLASH_BASE				0x40023C00
#define FLASH_ACR					REG_32(FLASH_BASE+0x00)
#define FLASH_CR					REG_32(FLASH_BASE+0x10)
#define FLASH_OPTCR				REG_32(FLASH_BASE+0x14)

#define UART1_BASE				0x40011000
#define UART1_SR					REG_32(UART1_BASE+0x00)
#define UART1_DR					REG_32(UART1_BASE+0x04)
#define UART1_BRR					REG_32(UART1_BASE+0x08)
#define UART1_CR1					REG_32(UART1_BASE+0x0C)
#define UART1_CR2					REG_32(UART1_BASE+0x10)
#define UART1_CR3					REG_32(UART1_BASE+0x14)

#define UART2_BASE				0x40004400
#define UART2_SR					REG_32(UART2_BASE+0x00)
#define UART2_DR					REG_32(UART2_BASE+0x04)
#define UART2_BRR					REG_32(UART2_BASE+0x08)
#define UART2_CR1					REG_32(UART2_BASE+0x0C)
#define UART2_CR2					REG_32(UART2_BASE+0x10)
#define UART2_CR3					REG_32(UART2_BASE+0x14)

#define UART4_BASE				0x40004C00
#define UART4_SR					REG_32(UART4_BASE+0x00)
#define UART4_DR					REG_32(UART4_BASE+0x04)
#define UART4_BRR					REG_32(UART4_BASE+0x08)
#define UART4_CR1					REG_32(UART4_BASE+0x0C)
#define UART4_CR2					REG_32(UART4_BASE+0x10)
#define UART4_CR3					REG_32(UART4_BASE+0x14)

#define I2C3_BASE					0x40005C00
#define I2C3_CR1					REG_32(I2C3_BASE+0x00)
#define	I2C3_CR2					REG_32(I2C3_BASE+0x04)
#define I2C3_OAR1					REG_32(I2C3_BASE+0x08)
#define	I2C3_CCR					REG_32(I2C3_BASE+0x1C)
#define I2C3_DR						REG_32(I2C3_BASE+0x10)
#define I2C3_SR1					REG_32(I2C3_BASE+0x14)
#define I2C3_SR2					REG_32(I2C3_BASE+0x18)
#define I2C3_TRISE				REG_32(I2C3_BASE+0x20)

#define SPI1_BASE					0x40013000
#define SPI1_CR1					REG_32(SPI1_BASE+0x00)
#define SPI1_CR2					REG_32(SPI1_BASE+0x04)
#define SPI1_SR						REG_32(SPI1_BASE+0x08)
#define SPI1_DR						REG_32(SPI1_BASE+0x0C)
#define SPI1_RXNE_STATUS	SPI1_SR & 0x0001
/*		OUTPUT RELAYS       */
#define LFR_RESET 				GPIOD_ODR |=  (1<<0)
#define LFR_SET 					GPIOD_ODR &= ~(1<<0)
#define TCFXR_RESET 			GPIOD_ODR |=  (1<<1)
#define TCFXR_SET 				GPIOD_ODR &= ~(1<<1)
#define TCFCR_RESET 			GPIOD_ODR |=  (1<<2)
#define TCFCR_SET 				GPIOD_ODR &= ~(1<<2)
#define TGTYR_RESET 			GPIOD_ODR |=  (1<<3)
#define TGTYR_SET 				GPIOD_ODR &= ~(1<<3)
#define ASGNCPR_RESET 		GPIOD_ODR |=  (1<<12)
#define ASGNCPR_SET 			GPIOD_ODR &= ~(1<<12)
#define TGTZR_RESET 			GPIOD_ODR |=  (1<<13)
#define TGTZR_SET 				GPIOD_ODR &= ~(1<<13)
#define BLR_RESET 				GPIOD_ODR |=  (1<<14)
#define BLR_SET 					GPIOD_ODR &= ~(1<<14)
#define XXXR_RESET 				GPIOD_ODR |=  (1<<15)
#define XXXR_SET 					GPIOD_ODR &= ~(1<<15)

/*		INPUT RELAYS  */
#define POWER								GPIOE_IDR & 0x0080 //PE7

#define TGTXR_FRONT_STATUS 	GPIOE_IDR & 0x0010 //PE4
#define TGTXR_BACK_STATUS 	GPIOE_IDR & 0x0008 //PE3
#define ASGNCR_FRONT_STATUS GPIOE_IDR & 0x0002 //PE1
#define ASGNCR_BACK_STATUS 	GPIOE_IDR & 0x0001 //PE0
#define TCFR_FRONT_STATUS 	GPIOF_IDR & 0x4000 //PF14
#define TCFR_BACK_STATUS 		GPIOF_IDR & 0x2000 //PF13
#define BTSR_FRONT_STATUS 	GPIOF_IDR & 0x0800 //PF11
#define BTSR_BACK_STATUS 		GPIOF_IDR & 0x0400 //PF10
#define HSGNCR_FRONT_STATUS GPIOF_IDR & 0x0100 //PF8
#define HSGNCR_BACK_STATUS 	GPIOC_IDR & 0x0001 //PC0
#define BPNR_FRONT_STATUS 	GPIOC_IDR & 0x0004 //PC2
#define BPNR_BACK_STATUS 		GPIOC_IDR & 0x0008 //PC3
#define BIPR1_FRONT_STATUS 	GPIOC_IDR & 0x0008 //PC5
#define BIPR1_BACK_STATUS 		GPIOC_IDR & 0x0008 //PC6
/*		FEEDBACK RELAYS       */
#define LFR_FRONT_STATUS 		GPIOE_IDR & 0x8000 //PE15
#define LFR_BACK_STATUS 		GPIOE_IDR & 0x4000 //PE14
#define TCFXR_FRONT_STATUS 	GPIOE_IDR & 0x1000 //PE12
#define TCFXR_BACK_STATUS 	GPIOE_IDR & 0x0800 //PE11
#define TCFCR_FRONT_STATUS 	GPIOE_IDR & 0x0200 //PE9
#define TCFCR_BACK_STATUS 	GPIOE_IDR & 0x0100 //PE8
#define TGTYR_FRONT_STATUS 	GPIOG_IDR & 0x0200 //PG9
#define TGTYR_BACK_STATUS 	GPIOG_IDR & 0x0400 //PG10
#define ASGNCPR_FRONT_STATUS GPIOG_IDR & 0x1000 //PG12
#define ASGNCPR_BACK_STATUS GPIOG_IDR & 0x2000 //PG13 
#define TGTZR_FRONT_STATUS 	GPIOG_IDR & 0x8000 //PG15
#define TGTZR_BACK_STATUS 	GPIOB_IDR & 0x0100 //PB8 
#define BLR_FRONT_STATUS 		GPIOB_IDR & 0x0400 //PB10
#define BLR_BACK_STATUS 		GPIOB_IDR & 0x0800 //PB11
#define BIPR2_FRONT_STATUS 	GPIOC_IDR & 0x0008 //PB13
#define BIPR2_BACK_STATUS 	GPIOC_IDR & 0x0008 //PB14

/*      I2C  PH7 PH8       */
#define I2C_SDA_HIGH 				GPIOH_ODR |=  (1<<8) //PH8
#define I2C_SCL_HIGH 				GPIOH_ODR |=  (1<<7) //PH7
#define I2C_SDA_LOW 				GPIOH_ODR &= ~(1<<8) //PH8
#define I2C_SCL_LOW		 			GPIOH_ODR &= ~(1<<7) //PH7

/*      SPI   */
#define SPI_CS2_SET					GPIOF_ODR |=  (1<<7) //PF7  CPU2
#define SPI_CS3_SET					GPIOF_ODR |=  (1<<6) //PF6  CPU3
#define SPI_CS1_SET					GPIOA_ODR |=  (1<<4) //PA4  CPU1
#define SPI_CLK_HIGH				GPIOA_ODR |=  (1<<5) //PA5
#define SPI_MOSI_HIGH				GPIOA_ODR |=  (1<<7) //PA7
#define SPI_MISO_HIGH				GPIOA_ODR |=  (1<<6) //PA6

#define SPI_CS2_RESET				GPIOF_ODR &= ~(1<<7) //PF7  CPU2
#define SPI_CS3_RESET				GPIOF_ODR &= ~(1<<6) //PF6  CPU3
#define SPI_CS1_RESET				GPIOA_ODR &= ~(1<<4) //PA4  CPU1
#define SPI_CLK_LOW	 				GPIOA_ODR &= ~(1<<5) //PA5
#define SPI_MOSI_LOW				GPIOA_ODR &= ~(1<<7) //PA7
#define SPI_MISO_LOW				GPIOA_ODR &= ~(1<<6) //PA6

#define SPI_CS2_STATUS			GPIOF_IDR & 0x0080  //PF7  CPU2
#define SPI_CS3_STATUS			GPIOF_IDR & 0x0040  //PF6  CPU3
#define SPI_CS1_STATUS			GPIOA_IDR & 0x0010  //PA4  CPU1
#define SPI_CLK_STATUS			GPIOA_IDR & 0x0020  //PA5
#define SPI_MOSI_STATUS			GPIOA_IDR & 0x0080  //PA7
#define SPI_MISO_STATUS			GPIOA_IDR & 0x0040  //PA6




/*     LCD    */
#define LCD_Slave_Addr 	0x27 //PCF8574T  CHIP
//#define LCD_Slave_Addr		0x3f //PCF8574AT CHIP

#define LCD_BACKLIGHT          0x08
#define LCD_NOBACKLIGHT        0x00
#define LCD_FIRST_ROW          0x80
#define LCD_SECOND_ROW         0xC0
#define LCD_THIRD_ROW          0x94
#define LCD_FOURTH_ROW         0xD4
#define LCD_CLEAR              0x01
#define LCD_RETURN_HOME        0x02
#define LCD_ENTRY_MODE_SET     0x04
#define LCD_CURSOR_OFF         0x0C
#define LCD_UNDERLINE_ON       0x0E
#define LCD_BLINK_CURSOR_ON    0x0F
#define LCD_MOVE_CURSOR_LEFT   0x10
#define LCD_MOVE_CURSOR_RIGHT  0x14
#define LCD_TURN_ON            0x0C
#define LCD_TURN_OFF           0x08
#define LCD_SHIFT_LEFT         0x18
#define LCD_SHIFT_RIGHT        0x1E

#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */
