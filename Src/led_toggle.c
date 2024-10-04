#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

void delay(int timeus) {
    for (volatile int i = 0; i < timeus*1000; i++);
} 

int main() {

    GPIO_Handle_t gpioLed;
    
    gpioLed.pGPIOx = GPIOC;
    
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_10;
    gpioLed.GPIO_PinConfig.GPIO_PinOutputType = GPIO_CNF_GP_PP_OUT;
    
    GPIO_PeriClockControl(GPIOC, ENABLE);
    
    GPIO_Init(&gpioLed);

    while (1) {
        GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        delay(1000);
        GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        delay(1000);
    }

    return 0;
}