/*
 * stm32f103xx.h
 *
 *  Created on: Sep 28, 2024
 *      Author: hocon
 */

#ifndef DRIVERS_INC_STM32F103XX_H_
#define DRIVERS_INC_STM32F103XX_H_

#include <stdint.h>

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
#define RCC_BASE_ADDR		(AHBPERIPH_BASE_ADDR + 0x1000)

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

#define GPIOA_PCLK_EN()	RCC->APB2ENR |= (1<<2);
#define GPIOB_PCLK_EN()	RCC->APB2ENR |= (1<<3);
#define GPIOC_PCLK_EN()	RCC->APB2ENR |= (1<<4);
#define GPIOD_PCLK_EN()	RCC->APB2ENR |= (1<<5);
#define GPIOE_PCLK_EN()	RCC->APB2ENR |= (1<<6);
#define GPIOF_PCLK_EN()	RCC->APB2ENR |= (1<<7);
#define GPIOG_PCLK_EN()	RCC->APB2ENR |= (1<<8);

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

#define GPIOA_PCLK_DIS()	RCC->APB2ENR &= ~(1<<2);
#define GPIOB_PCLK_DIS()	RCC->APB2ENR &= ~(1<<3);
#define GPIOC_PCLK_DIS()	RCC->APB2ENR &= ~(1<<4);
#define GPIOD_PCLK_DIS()	RCC->APB2ENR &= ~(1<<5);
#define GPIOE_PCLK_DIS()	RCC->APB2ENR &= ~(1<<6);
#define GPIOF_PCLK_DIS()	RCC->APB2ENR &= ~(1<<7);
#define GPIOG_PCLK_DIS()	RCC->APB2ENR &= ~(1<<8);
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
