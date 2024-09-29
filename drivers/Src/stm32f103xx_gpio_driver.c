#include "stm32f103xx_gpio_driver.h"

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
	// configure the mode and type of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_OUTPUT_50) {
		
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
			pGPIOHandle->pGPIOx->CRH &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4);
			pGPIOHandle->pGPIOx->CRH |= (temp << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4);
		}
	} 
	else {
		// this part will do later.
	}
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx) {

}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber) {

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx) {

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t value) {

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value) {

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber) {

}
 /*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enOrDis) {

}

void GPIO_IRQHandling(uint8_t pinNumber) {

}
