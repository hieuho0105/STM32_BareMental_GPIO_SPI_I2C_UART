/*
 * stm32f103xx.h
 *
 *  Created on: Sep 28, 2024
 *      Author: hocon
 */

#ifndef DRIVERS_INC_STM32F103XX_H_
#define DRIVERS_INC_STM32F103XX_H_

#include <stdint.h>


/*********************************Start: Processor Specific details*********************************/
/*
 * ARM cortex M3 Processor NVIC ISERx register Address; x = 0 -> 7
 */
#define NVIC_ISER0			((volatile uint32_t* )0xE000E100)
#define NVIC_ISER1			(NVIC_ISER0 + 1)
#define NVIC_ISER2			(NVIC_ISER1 + 1)
#define NVIC_ISER3			(NVIC_ISER2 + 1)
#define NVIC_ISER4			(NVIC_ISER3 + 1)
#define NVIC_ISER5			(NVIC_ISER4 + 1)
#define NVIC_ISER6			(NVIC_ISER5 + 1)
#define NVIC_ISER7			(NVIC_ISER6 + 1)

/*
 * ARM cortex M3 Processor NVIC ICERx register Address; x = 0 -> 7
 */
#define NVIC_ICER0			((volatile uint32_t* )0XE000E180)
#define NVIC_ICER1			(NVIC_ICER0 + 1)
#define NVIC_ICER2			(NVIC_ICER1 + 1)
#define NVIC_ICER3			(NVIC_ICER2 + 1)
#define NVIC_ICER4			(NVIC_ICER3 + 1)
#define NVIC_ICER5			(NVIC_ICER4 + 1)
#define NVIC_ICER6			(NVIC_ICER5 + 1)
#define NVIC_ICER7			(NVIC_ICER6 + 1)

/*
 * ARM cortex M3 Processor NVIC IPRx register Address; x = 0 -> 59
 */
#define NVIC_IPR_BASE_ADDR	((volatile uint32_t* )0xE000E400)

#define NVIC_IPR0			((volatile uint32_t* )0xE000E400)
#define NVIC_IPR1			(NVIC_IPR0 + 1)
#define NVIC_IPR2			(NVIC_IPR1 + 1)
#define NVIC_IPR3			(NVIC_IPR2 + 1)
#define NVIC_IPR4			(NVIC_IPR3 + 1)
#define NVIC_IPR5			(NVIC_IPR4 + 1)
#define NVIC_IPR6			(NVIC_IPR5 + 1)
#define NVIC_IPR7			(NVIC_IPR6 + 1)
#define NVIC_IPR8			(NVIC_IPR7 + 1)
#define NVIC_IPR9			(NVIC_IPR8 + 1)
#define NVIC_IPR10			(NVIC_IPR9 + 1)
#define NVIC_IPR11			(NVIC_IPR10 + 1)
#define NVIC_IPR12			(NVIC_IPR11 + 1)
#define NVIC_IPR13			(NVIC_IPR12 + 1)
#define NVIC_IPR14			(NVIC_IPR13 + 1)
#define NVIC_IPR15			(NVIC_IPR14 + 1)
#define NVIC_IPR16			(NVIC_IPR15 + 1)
#define NVIC_IPR17			(NVIC_IPR16 + 1)
#define NVIC_IPR18			(NVIC_IPR17 + 1)
#define NVIC_IPR19			(NVIC_IPR18 + 1)
#define NVIC_IPR20			(NVIC_IPR19 + 1)
#define NVIC_IPR21			(NVIC_IPR20 + 1)
#define NVIC_IPR22			(NVIC_IPR21 + 1)
#define NVIC_IPR23			(NVIC_IPR22 + 1)
#define NVIC_IPR24			(NVIC_IPR23 + 1)
#define NVIC_IPR25			(NVIC_IPR24 + 1)
#define NVIC_IPR26			(NVIC_IPR25 + 1)
#define NVIC_IPR27			(NVIC_IPR26 + 1)
#define NVIC_IPR28			(NVIC_IPR27 + 1)
#define NVIC_IPR29			(NVIC_IPR28 + 1)
#define NVIC_IPR30			(NVIC_IPR29 + 1)
#define NVIC_IPR31			(NVIC_IPR30 + 1)
#define NVIC_IPR32			(NVIC_IPR31 + 1)
#define NVIC_IPR33			(NVIC_IPR32 + 1)
#define NVIC_IPR34			(NVIC_IPR33 + 1)
#define NVIC_IPR35			(NVIC_IPR34 + 1)
#define NVIC_IPR36			(NVIC_IPR35 + 1)
#define NVIC_IPR37			(NVIC_IPR36 + 1)
#define NVIC_IPR38			(NVIC_IPR37 + 1)
#define NVIC_IPR39			(NVIC_IPR38 + 1)
#define NVIC_IPR40			(NVIC_IPR39 + 1)
#define NVIC_IPR41			(NVIC_IPR40 + 1)
#define NVIC_IPR42			(NVIC_IPR41 + 1)
#define NVIC_IPR43			(NVIC_IPR42 + 1)
#define NVIC_IPR44			(NVIC_IPR43 + 1)
#define NVIC_IPR45			(NVIC_IPR44 + 1)
#define NVIC_IPR46			(NVIC_IPR45 + 1)
#define NVIC_IPR47			(NVIC_IPR46 + 1)
#define NVIC_IPR48			(NVIC_IPR47 + 1)
#define NVIC_IPR49			(NVIC_IPR48 + 1)
#define NVIC_IPR50			(NVIC_IPR49 + 1)
#define NVIC_IPR51			(NVIC_IPR50 + 1)
#define NVIC_IPR52			(NVIC_IPR51 + 1)
#define NVIC_IPR53			(NVIC_IPR52 + 1)
#define NVIC_IPR54			(NVIC_IPR53 + 1)
#define NVIC_IPR55			(NVIC_IPR54 + 1)
#define NVIC_IPR56			(NVIC_IPR55 + 1)
#define NVIC_IPR57			(NVIC_IPR56 + 1)
#define NVIC_IPR58			(NVIC_IPR57 + 1)
#define NVIC_IPR59			(NVIC_IPR58 + 1)
/*********************************END: Processor Specific details*********************************/



#define FLASH_BASE_ADDR		0x08000000U
#define SRAM_BASE_ADDR		0x20000000U
#define ROM_BASE_ADDR		0x1FFFF000U

/*
 * AHBx and APBx Bus Peripheral base address
 */
#define PERIPH_BASE_ADDR		0x40000000U
#define APB1PERIPH_BASE_ADDR	PERIPH_BASE_ADDR
#define APB2PERIPH_BASE_ADDR	0x40010000U
#define AHBPERIPH_BASE_ADDR		0x40018000U

/*
 * Base address of peripherals which are hanging on APB1 bus
 */
#define SPI2_I2S_BASE_ADDR	(APB1PERIPH_BASE_ADDR +  0x3800)
#define SPI3_I2S_BASE_ADDR	(APB1PERIPH_BASE_ADDR +  0x3C00)

#define USART2_BASE_ADDR	(APB1PERIPH_BASE_ADDR +  0x4400)
#define USART3_BASE_ADDR	(APB1PERIPH_BASE_ADDR +  0x4800)
#define UART4_BASE_ADDR		(APB1PERIPH_BASE_ADDR +  0x4C00)
#define UART5_BASE_ADDR		(APB1PERIPH_BASE_ADDR +  0x5000)

#define I2C1_BASE_ADDR		(APB1PERIPH_BASE_ADDR +  0x5400)
#define I2C2_BASE_ADDR		(APB1PERIPH_BASE_ADDR +  0x5800)


/*
 * Base address of peripherals which are hanging on APB2 bus
 */
#define AFIO_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x0000)
#define EXTI_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x0400)

#define GPIOA_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x0800)
#define GPIOB_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x0C00)
#define GPIOC_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x1000)
#define GPIOD_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x1400)
#define GPIOE_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x1800)
#define GPIOF_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x1C00)
#define GPIOG_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x2000)

#define SPI1_BASE_ADDR		(APB2PERIPH_BASE_ADDR +  0x2000)

#define USART1_BASE_ADDR	(APB2PERIPH_BASE_ADDR +  0x3800)

/*
 * Base address of peripherals which are hanging on AHB bus
 */
#define RCC_BASE_ADDR		(AHBPERIPH_BASE_ADDR + 0x9000)


/*
 * Cac thanh ghi cua GPIO peripheral
 */
typedef struct {
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} GPIO_RegDef_t;
/*
 * Dinh nghia cac GPIOx va ep kieu 
 */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASE_ADDR)


/*
 * Cac thanh ghi cua EXTI peripheral
 */
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;
/*
 * Dinh nghia EXTI peripheral va ep kieu
 */
#define EXTI	((EXTI_RegDef_t*)EXTI_BASE_ADDR)


/*
 * Cac thanh ghi cua AFIO peripheral
 */
typedef struct {
	volatile uint32_t EVCR;
	volatile uint32_t MAPR;
	volatile uint32_t EXTICR[4];  /*!< decides which GPIO port should take over this EXTI line */
	volatile uint32_t MAPR2;
}AFIO_RegDef_t;
/*
 * Dinh nghia AFIO peripheral va ep kieu
 */
#define AFIO	((AFIO_RegDef_t*)AFIO_BASE_ADDR)

/*
 * Cac thanh ghi cua RCC peripheral
 */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBSTR;
	volatile uint32_t CFGR2;
} RCC_RegDef_t;
#define RCC		((RCC_RegDef_t*) RCC_BASE_ADDR)


/*
 * CLock enable macros for GPIOx peripheral
 */

#define GPIOA_PCLK_EN()	RCC->APB2ENR |= (1<<2)
#define GPIOB_PCLK_EN()	RCC->APB2ENR |= (1<<3)
#define GPIOC_PCLK_EN()	RCC->APB2ENR |= (1<<4)
#define GPIOD_PCLK_EN()	RCC->APB2ENR |= (1<<5)
#define GPIOE_PCLK_EN()	RCC->APB2ENR |= (1<<6)
#define GPIOF_PCLK_EN()	RCC->APB2ENR |= (1<<7)
#define GPIOG_PCLK_EN()	RCC->APB2ENR |= (1<<8)

/*
 * CLock enable macros for SPI peripheral
 */


/*
 * CLock enable macros for UART peripheral
 */


/*
 * CLock enable macros for I2C peripheral
 */

/*
 * CLock disable macros for GPIOx peripheral
 */

#define GPIOA_PCLK_DIS()	RCC->APB2ENR &= ~(1<<2)
#define GPIOB_PCLK_DIS()	RCC->APB2ENR &= ~(1<<3)
#define GPIOC_PCLK_DIS()	RCC->APB2ENR &= ~(1<<4)
#define GPIOD_PCLK_DIS()	RCC->APB2ENR &= ~(1<<5)
#define GPIOE_PCLK_DIS()	RCC->APB2ENR &= ~(1<<6)
#define GPIOF_PCLK_DIS()	RCC->APB2ENR &= ~(1<<7)
#define GPIOG_PCLK_DIS()	RCC->APB2ENR &= ~(1<<8)


/*
 * Macros to reset GPIOx peripheral
 */
#define GPIOA_REG_RESET() 	do { RCC->APB2RSTR |= (1<<2); RCC->APB2RSTR &= ~(1<<2);} while (0)
#define GPIOB_REG_RESET() 	do { RCC->APB2RSTR |= (1<<3); RCC->APB2RSTR &= ~(1<<2);} while (0)
#define GPIOC_REG_RESET() 	do { RCC->APB2RSTR |= (1<<4); RCC->APB2RSTR &= ~(1<<2);} while (0)
#define GPIOD_REG_RESET() 	do { RCC->APB2RSTR |= (1<<5); RCC->APB2RSTR &= ~(1<<2);} while (0)
#define GPIOE_REG_RESET() 	do { RCC->APB2RSTR |= (1<<6); RCC->APB2RSTR &= ~(1<<2);} while (0)

/*
 * Some generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#endif /* DRIVERS_INC_STM32F103XX_H_ */
