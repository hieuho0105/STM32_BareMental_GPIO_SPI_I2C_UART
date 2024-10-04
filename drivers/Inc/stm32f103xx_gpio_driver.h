/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Sep 28, 2024
 *      Author: Ho Cong Hieu
 */

#ifndef DRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_
#define DRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

/*
 * Struct chua cac gia tri se gan cho cac thanh ghi cau hinh cua GPIO
 */
typedef struct {
	uint8_t GPIO_PinNumber;				/*!< possible values from @GPIO_PIN_NUMBER */
	uint8_t GPIO_PinMode;				/*!< possible values from @GPIO_PIN_MODE */
	uint8_t GPIO_PinInputType; 			/*!< possible values from @GPIO_PIN_CNF_IN */
	uint8_t GPIO_PinOutputType;			/*!< possible values from @GPIO_PIN_CNF_OUT */
} GPIO_PinConfig_t;

/*
 * struct chua dia chi cua thanh ghi can cau hinh va cac gia tri se cau hinh cho no
 */
typedef struct {
	GPIO_RegDef_t* pGPIOx; 				/*!< This holds the base address of the GPIO to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig; 	/*!< This holds GPIO pin configuration settings >*/
} GPIO_Handle_t;


/****************************************************************************
 * 																		    *
 * 																		    *
 * 																		    *
 ****************************************************************************/

/*
 * @GPIO_PIN_NUMBER
 */
#define GPIO_PIN_0 		0
#define GPIO_PIN_1 		1
#define GPIO_PIN_2 		2
#define GPIO_PIN_3 		3
#define GPIO_PIN_4 		4
#define GPIO_PIN_5 		5
#define GPIO_PIN_6 		6
#define GPIO_PIN_7 		7
#define GPIO_PIN_8 		8
#define GPIO_PIN_9 		9
#define GPIO_PIN_10 	10
#define GPIO_PIN_11 	11
#define GPIO_PIN_12 	12
#define GPIO_PIN_13 	13
#define GPIO_PIN_14 	14
#define GPIO_PIN_15 	15

/*
 * GPIO pin possible modes
 * @GPIO_PIN_MODE
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT_10		1
#define GPIO_MODE_OUTPUT_2		2
#define GPIO_MODE_OUTPUT_50		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6
/*
 * GPIO pin possible configuration in input mode
 * @GPIO_PIN_CNF_IN
 */
#define GPIO_CNF_ANALOG_IN			0
#define GPIO_CNF_FLOATING_IN		1
#define GPIO_CNF_PUPD_IN			2
#define GPIO_CNF_RESERVED_IN		3

/*
 * GPIO pin possible configuration in output mode
 * @GPIO_PIN_CNF_OUT
 * GP = General purpose
 * AF = Alternate function
 * PP = push-pull
 * OD = Open-drain
 */
#define GPIO_CNF_GP_PP_OUT			0
#define GPIO_CNF_GP_OD_OUT			1
#define GPIO_CNF_AF_PP_OUT			2
#define GPIO_CNF_AF_OD_OUT			3


/****************************************************************************
 * 																		    *
 * 							APIS											*
 * 																		    *
 ****************************************************************************/

/*
 * Peripheral clock set up
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enOrDis);
/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enOrDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);
#endif /* DRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_ */
