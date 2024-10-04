#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx.h"
/***************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief 		- This function enables or disable clock for the given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 *
 * @param[in]	- ENABLE or DISABLE macro
 *
 * @return 		- none
 *
 * @note		- none
 */



void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
	}
	else if (enOrDis == DISABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DIS();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DIS();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DIS();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DIS();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DIS();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DIS();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DIS();
		}
	}
}
/***************************************
 * @fn			- GPIO_Init
 *
 * @brief 		- This function enables or disable clock for the given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 *
 *
 * @return 		- none
 *
 * @note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_OUTPUT_50) { // Mode IO
		
		uint32_t temp;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode; 

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT) {
			temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinInputType << 2);
		}
		else {
			temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputType << 2);
		}

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_7) {
			pGPIOHandle->pGPIOx->CRL &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4);
			pGPIOHandle->pGPIOx->CRL |= (temp << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4);
		}
		else {
			pGPIOHandle->pGPIOx->CRH &= ~(0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - GPIO_PIN_8)*4);
			pGPIOHandle->pGPIOx->CRH |= (temp << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - GPIO_PIN_8)*4);
		}
	} 
	else { // Mode external interrupt
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 1. clear the correspond RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1.configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 1. clear the correspond FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 1. Set the correspond RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in AFIO_EXTICR
		AFIO->EXTICR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4] &= ~(0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)*4);
		AFIO->EXTICR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4] |= (pGPIOHandle->pGPIOx - GPIOA) << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)*4);
		// 3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else {
		// PORT F and G can not be reset
	}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber) {
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
	
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx) {
	
	uint16_t value;
	value = (uint16_t)((pGPIOx->IDR) & 0x0000FFFF);
	
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value) {
	if (value == GPIO_PIN_SET) {
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else {
		// write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value) {
	// clear port
	pGPIOx->ODR &= 0;
	// write port 
	pGPIOx->ODR |= value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber) {
	// xor với 0 = giữ nguyên bit.
	// xor với 1 = đảo bit
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << pinNumber);
}
 /*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		if (IRQNumber < 32) {
			*NVIC_ISER0 |= 1 << IRQNumber;
		}
		else if (IRQNumber < 64) {
			*NVIC_ISER1 |= 1 << (IRQNumber % 32);
		}
		else if (IRQNumber < 96) {
			*NVIC_ISER2 |= 1 << (IRQNumber % 32);
		}
	}
	else {
		if (IRQNumber < 32) {
			*NVIC_ICER0 |= 1 << IRQNumber;
		}
		else if (IRQNumber < 64) {
			*NVIC_ICER1 |= 1 << (IRQNumber % 32);
		}
		else if (IRQNumber < 96) {
			*NVIC_ICER2 |= 1 << (IRQNumber % 32);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint32_t iprx = IRQPriority / 4;
	uint32_t iprx_section = IRQPriority % 4;
	*(NVIC_IPR_BASE_ADDR + iprx) &= ~(0xFF << iprx_section*8);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << (iprx_section*8 + 4));
}
void GPIO_IRQHandling(uint8_t pinNumber) {
	if (EXTI->PR & (1 << pinNumber)) {
		EXTI->PR |= (1 << pinNumber); // clear pending
	}
}
