/*
 * stm32f407xx.h
 *
 *  Created on: Mar 14, 2024
 *      Author: ImadF
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#include<stddef.h>


#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)


/* BASE ADDRESSES OF FLASH AND SRAM MEORIES*/

#define FLASH_BASEADDR							0x08000000U	// base address of flash memory
#define SRAM1_BASEADDR							0x20000000U  //112kb SRAM1 base address
#define SRAM2_BASEADDR							0x2001C000U	 //SRAM2 base address
#define ROM_BASEADDR							0x1FFF0000U	 // ROM/System Memory base address
#define SRAM									SRAM1_BASEADDR //base address of the SRAM being used

/* AHBx and APBx Peripheral Base Addresses */

#define PERIPH_BASEADDR							0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*Base Addresses of Peripherals hanging on AHB1 bus*  (required ones) */

#define GPIOA_BASEADDR							(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR							(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR							(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR							(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR							(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR							(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR							(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR							(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR							(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR							(AHB1PERIPH_BASEADDR + 0x3800)


/*Base Addresses of Peripherals hanging on APB1 bus*  (required ones) */
#define I2C1_BASEADDR 							(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 							(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 							(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR 							(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 							(APB1PERIPH_BASEADDR + 0x3C00)


#define USART2_BASEADDR							(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR							(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR							(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR							(APB1PERIPH_BASEADDR + 0x5000)


/*Base Addresses of Peripherals hanging on APB2 bus*  (required ones) */
#define EXTI_BASEADDR							(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR							(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR							(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR							(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR							(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR							(APB2PERIPH_BASEADDR + 0x1400)

#define __weak									__attribute__((weak))
typedef struct
{
	__vo uint32_t MODER;				/*GPIO Port Mode Reg  												 Offset: 0x00*/
	__vo uint32_t OTYPER;				/*GPIO Port Output Type Reg  Offset: 0x04						 	 Offset: 0x04*/
	__vo uint32_t OSPEEDR;				/*GPIO Port Mode Output Speed Reg   Offset: 0x00				 	 Offset: 0x08*/
	__vo uint32_t PUPDR;				/*GPIO Port Pullup Pulldown Reg   Offset: 0x00						 Offset: 0x0C*/
	__vo uint32_t IDR;					/*GPIO Port Input data Reg   Offset: 0x00						     Offset: 0x10*/
	__vo uint32_t ODR;					/*GPIO Port Output Data Reg   Offset: 0x00							 Offset: 0x14*/
	__vo uint32_t BSRR;					/*GPIO Port Bit Set/Reset Reg   Offset: 0x00						 Offset: 0x18*/
	__vo uint32_t LCKR;					/*GPIO Port Mode Reg   Offset: 0x00									 Offset: 0x1C*/
	__vo uint32_t AFR[2];				/*GPIO Port Alternate Fun Reg
										AFO[0]: AFLR & AFO[1]: AFHR Offset:0x20 & 0X24*/
}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t RCC_CR;				/*RCC clock control register   		   								OFFSET:0x00*/
	__vo uint32_t RCC_PLLCFGR;			/*RCC PLL configuration register	  							    OFFSET:0x04*/
	__vo uint32_t RCC_CFGR;				/*RCC clock configuration register     								OFFSET:0x08*/
	__vo uint32_t RCC_CIR;				/*RCC clock interrupt register   	   								OFFSET:0x0C*/
	__vo uint32_t RCC_AHB1RSTR;			/*RCC AHB1 peripheral reset register   								OFFSET:0x10*/
	__vo uint32_t RCC_AHB2RSTR;			/*RCC AHB2 peripheral reset   		   								OFFSET:0x14*/
	__vo uint32_t RCC_AHB3RSTR;			/*RCC AHB3 peripheral reset   		   								OFFSET:0x18*/
	__vo uint32_t RESERVED0;			/*RESERVED 							   								OFFSET:0x1C*/
	__vo uint32_t RCC_APB1RSTR;			/*RCC APB1 peripheral reset  		 							    OFFSET:0x20*/
	__vo uint32_t RCC_APB2RSTR;			/*RCC APB2 peripheral reset register  							    OFFSET:0x24*/
	__vo uint32_t RESERVED1;			/*RCC clock control register   		   							`	OFFSET:0x28*/
	__vo uint32_t RESERVED2;			/*RCC clock control register   		  							    OFFSET:0x2C*/
	__vo uint32_t RCC_AHB1ENR;			/*RCC AHB1 peripheral clock enable register  					    OFFSET:0x30*/
	__vo uint32_t RCC_AHB2ENR;			/*RCC AHB2 peripheral clock enable register   						OFFSET:0x34*/
	__vo uint32_t RCC_AHB3ENR;			/*RCC AHB3 peripheral clock enable register   					    OFFSET:0x38*/
	__vo uint32_t RESERVED3;			/*RESERVED							  							    OFFSET:0x3C*/
	__vo uint32_t RCC_APB1ENR;			/*RCC APB1 peripheral clock enable register   						OFFSET:0x40*/
	__vo uint32_t RCC_APB2ENR;			/*RCC APB2 peripheral clock enable register   						OFFSET:0x44*/
	__vo uint32_t RESREVED4;			/*RESERVED					   		   								OFFSET:0x48*/
	__vo uint32_t RESREVED5;			/*RESERVED					   		   								OFFSET:0x4C*/
	__vo uint32_t RCC_AHB1LPENR;		/*RCC AHB1 peripheral clock enable in low power mode register 		OFFSET:0x50*/
	__vo uint32_t RCC_AHB2LPENR;		/*RCC AHB2 peripheral clock enable in low power mode register 		OFFSET:0x54*/
	__vo uint32_t RCC_AHB3LPENR;		/*RCC AHB3 peripheral clock enable in low power mode register 		OFFSET:0x58*/
    __vo uint32_t RESERVED6;			/*RESERVED   														OFFSET:0x5C*/
	__vo uint32_t RCC_APB1LPENR;		/*RCC APB1 peripheral clock enable in low power mode register 		OFFSET:0x60*/
	__vo uint32_t RCC_APB2LPENR;		/*RCC APB2 peripheral clock enable in low power mode register 		OFFSET:0x64*/
	__vo uint32_t RESERVED7;			/*RESERVED   														OFFSET:0x68*/
	__vo uint32_t RESREVED8;			/*RESERVED 														    OFFSET:0x6C*/
	__vo uint32_t RCC_BDCR;				/*RCC Backup domain control register 								OFFSET:0x70*/
	__vo uint32_t RCC_CSR;				/*RCC clock control & status register 							    OFFSET:0x74*/
	__vo uint32_t RESERVED9;			/*RESERVED   														OFFSET:0x78*/
	__vo uint32_t RESREVED10;			/*RESERVED   														OFFSET:0x7C*/
	__vo uint32_t RCC_SSCGR;			/*RCC spread spectrum clock generation register  					OFFSET:0x80*/
	__vo uint32_t RCC_PLLI2SCFGR;		/*RCC PLLI2S configuration register 							    OFFSET:0x84*/
	__vo uint32_t RCC_PLLSAICFGR;		/*RCC PLL configuration register  								    OFFSET:0x88*/
	__vo uint32_t RCC_DCKCFGR;			/*RCC Dedicated Clock Configuration Register					    OFFSET:0x8C*/
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!<                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!<  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!<, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!<  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!<,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< ,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!<  , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!<           							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!<          								  Address offset: 0x20      */

} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;


typedef struct
{
	__vo uint32_t USART_SR;			//Data Register
	__vo uint32_t USART_DR;			//Data Register
	__vo uint32_t USART_BRR;		//Baud rate register
	__vo uint32_t USART_CR1;		//Control register 1
	__vo uint32_t USART_CR2;		//Control register 2
	__vo uint32_t USART_CR3;		//Control register 3
	__vo uint32_t USART_GTPR;		//Guard time and prescaler register

}USART_RegDef_t;

#define GPIOA									 ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB									 ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC									 ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD									 ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE									 ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF									 ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG									 ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH									 ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI									 ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC										 ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 									 ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG									 ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define SPI1									 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2									 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3									 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4									 ((SPI_RegDef_t*)SPI4_BASEADDR)
#define I2C1									 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2									 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3									 ((I2C_RegDef_t*)I2C3_BASEADDR)
#define USART1									 ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2									 ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3									 ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4									 ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5									 ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6									 ((USART_RegDef_t*)USART6_BASEADDR)



/*Clock Enable Macros for GPIOx Peripherals*/
#define GPIOA_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN() 						 ( RCC->RCC_AHB1ENR |= (1<<8) )

/*Clock Enable Macros for I2Cx Peripherals*/
#define I2C1_PCLK_EN() 						 ( RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() 						 ( RCC->RCC_APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() 						 ( RCC->RCC_APB1ENR |= (1<<23))

/*Clock Enable Macros for SPIx Peripherals*/
#define SPI1_PCLK_EN() 						 ( RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() 						 ( RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() 						 ( RCC->RCC_APB1ENR |= (1<<15))
#define SPI4_PCLK_EN() 						 ( RCC->RCC_APB2ENR |= (1<<13))


/*Clock enable Macros for UARTx/USARTx peripherals*/
#define USART1_PCLK_EN()						  ( RCC->RCC_APB2ENR |= (1<<4))
#define USART2_PCLK_EN()						  ( RCC->RCC_APB1ENR |= (1<<17))
#define USART3_PCLK_EN()						  ( RCC->RCC_APB1ENR |= (1<<18))
#define USART6_PCLK_EN()						  ( RCC->RCC_APB2ENR |= (1<<5))
#define UART4_PCLK_EN()							  ( RCC->RCC_APB1ENR |= (1<<19))
#define UART5_PCLK_EN()							  ( RCC->RCC_APB1ENR |= (1<<20))

/*Clock enable Macros for SYSCFG peripherals*/

#define SYSCFG_PCLK_EN()						  ( RCC->RCC_APB2ENR |= (1<<14))

/*Clock Disable Macros for GPIOx Peripherals*/
#define GPIOA_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI() 						 ( RCC->RCC_AHB1ENR &= ~(1<<8))

/*Clock Disable Macros for I2Cx Peripherals*/
#define I2C1_PCLK_DI () 						 ( RCC->RCC_APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI () 						 ( RCC->RCC_APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI () 						 ( RCC->RCC_APB1ENR &= ~(1<<23))

/*Clock Disable Macros for SPIx Peripherals*/
#define SPI1_PCLK_DI() 							 ( RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() 						 	 ( RCC->RCC_APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() 						 	 ( RCC->RCC_APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI() 						 	 ( RCC->RCC_APB1ENR &= ~(1<<15))


/*Clock disable Macros for UARTx/USARTx peripherals*/
#define USART1_PCLK_DI()						  ( RCC->RCC_APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()						  ( RCC->RCC_APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()						  ( RCC->RCC_APB1ENR &= ~(1<<18))
#define USART6_PCLK_DI()						  ( RCC->RCC_APB2ENR &= ~(1<<5))
#define UART4_PCLK_DI()							  ( RCC->RCC_APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()							  ( RCC->RCC_APB1ENR &= ~(1<<20))

/*Clock disable Macros for SYSCFG peripherals*/

#define SYSCFG_PCLK_DI()						  ( RCC->RCC_APB2ENR &= ~(1<<14))

/*Reset macros for GPIOs*/

#define GPIOA_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<0)); ( RCC->RCC_AHB1RSTR &= ~(1<<0)); }  while(0)
#define GPIOB_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<1));  ( RCC->RCC_AHB1RSTR &= ~(1<<1)); } while(0)
#define GPIOC_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<2));  ( RCC->RCC_AHB1RSTR &= ~(1<<2)); } while(0)
#define GPIOD_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<3));  ( RCC->RCC_AHB1RSTR &= ~(1<<3)); } while(0)
#define GPIOE_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<4));  ( RCC->RCC_AHB1RSTR &= ~(1<<4)); } while(0)
#define GPIOF_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<5));  ( RCC->RCC_AHB1RSTR &= ~(1<<5)); } while(0)
#define GPIOG_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<6)); (RCC->RCC_AHB1RSTR &= ~(1<<6)); }  while(0)
#define GPIOH_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<7));  ( RCC->RCC_AHB1RSTR &= ~(1<<7)); } while(0)
#define GPIOI_REGS_RESET() 						 do{ ( RCC->RCC_AHB1RSTR |= (1<<8));  ( RCC->RCC_AHB1RSTR &= ~(1<<8)) ;} while(0)

/*Reset macros for SPIs*/

#define SPI1_REGS_RESET()						do{ ( RCC->RCC_APB2RSTR |= (1<<12)); ( RCC->RCC_APB2RSTR &= ~(1<<12)); } while(0)
#define SPI2_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<14)); ( RCC->RCC_APB1RSTR &= ~(1<<14)); } while(0)
#define SPI3_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<15)); ( RCC->RCC_APB1RSTR &= ~(1<<15)); } while(0)
#define SPI4_REGS_RESET()						do{ ( RCC->RCC_APB2RSTR |= (1<<13)); ( RCC->RCC_APB2RSTR &= ~(1<<13)); } while(0)

/*Reset macros for I2Cs*/

#define I2C1_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<21)); ( RCC->RCC_APB1RSTR &= ~(1<<21)); } while(0)
#define I2C2_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<22)); ( RCC->RCC_APB1RSTR &= ~(1<<22)); } while(0)
#define I2C3_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<23)); ( RCC->RCC_APB1RSTR &= ~(1<<23)); } while(0)

/*Reset macros for USART/UART*/
#define USART1_REGS_RESET()						do{ ( RCC->RCC_APB2RSTR |= (1<<4)); ( RCC->RCC_APB2RSTR &= ~(1<<4)); } while(0)
#define USART2_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<17)); ( RCC->RCC_APB1RSTR &= ~(1<<17)); } while(0)
#define USART3_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<18)); ( RCC->RCC_APB1RSTR &= ~(1<<18)); } while(0)
#define UART4_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<19)); ( RCC->RCC_APB1RSTR &= ~(1<<19)); } while(0)
#define UART5_REGS_RESET()						do{ ( RCC->RCC_APB1RSTR |= (1<<20)); ( RCC->RCC_APB1RSTR &= ~(1<<20)); } while(0)
#define USART6_REGS_RESET()						do{ ( RCC->RCC_APB2RSTR |= (1<<5)); ( RCC->RCC_APB2RSTR &= ~(1<<5)); } while(0)


//gpio base address to syscgf CR code
#define GPIO_BASEADDR_TO_CODE(x)				( (x==GPIOA) ? 0 :\
												  (x==GPIOB) ? 1 :\
												  (x==GPIOC) ? 2 :\
												  (x==GPIOD) ? 3 :\
												  (x==GPIOE) ? 4 :\
												  (x==GPIOF) ? 5 :\
												  (x==GPIOG) ? 6 :\
												  (x==GPIOH) ? 7 :\
												  (x==GPIOI) ? 8 :0 )

/*EXTI IRQ NUMBER DEFINITIONS*/
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*I2C IRQ NUMBER DEFINITIONS*/
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

/*SPI IRQ NUMBER DEFINITIONS*/

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51


/*IRQ PRIORITY*/
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI15		15

/*some misc. macros*/

#define ENABLE					1
#define DISABLE 				0
#define SET						ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET		    RESET
#define FLAG_RESET       	    RESET
#define FLAG_SET 				SET

//************************BIT POSITION DEFS FOR SPI REGISTERS******************************************************

//  macros for sPI_CR1 bit fields

# define SPI_CR1_CPHA			0
# define SPI_CR1_CPOL 			1
# define SPI_CR1_MSTR 			2
# define SPI_CR1_BR 			3
# define SPI_CR1_SPE 			6
# define SPI_CR1_SSI 			8
# define SPI_CR1_SSM 			9
# define SPI_CR1_RXONLY			10
# define SPI_CR1_DFF 			11

//  macros for sPI_CR2 bit fields
# define SPI_CR2_RXDMAEN 		0
# define SPI_CR2_TXDMAEN 		1
# define SPI_CR2_SSOE 			2
# define SPI_CR2_FRF 			4
# define SPI_CR2_ERRIE 			5
# define SPI_CR2_RXNEIE 		6
# define SPI_CR2_TXEIE			7

//  macros for sPI_SR bit fields
# define SPI_SR_RXNE 			0
# define SPI_SR_TXE	 			1
# define SPI_SR_CHSIDE 			2
# define SPI_SR_UDR				3
# define SPI_SR_CRCERR			4
# define SPI_SR_MODF 			5
# define SPI_SR_OVR				6
# define SPI_SR_BSY				7
# define SPI_SR_FRE				8

//************************BIT POSITION DEFS FOR I2C REGISTERS******************************************************

//BIT POAITIONS FOR CR1
#define I2C_CR1_PE				0
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15

//BIT POAITIONS FOR CR2
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVFEN			9
#define I2C_CR2_ITBUFEN			10

//BIT POAITIONS FOR SR1
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			5
#define I2C_SR1_TXE				6
#define I2C_SR1_BERR			7
#define I2C_SR1_ARLO			8
#define I2C_SR1_AF				9
#define I2C_SR1_OVR				10
#define I2C_SR1_TIMEOUT			11

//BIT POSITIONS FOR SR2
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7

//BIT POSITIONS FOR SR2
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

//************************BIT POSITION DEFS FOR USART REGISTERS******************************************************
//BIT POSITIONS FOR CR1
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

//BIT POSITIONS FOR CR2
#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

//BIT POSITIONS FOR CR3
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

//BIT POSITIONS FOR SR
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_USART_driver.h"
#include "stm2f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
