/*
 * ledButton.c
 *
 *  Created on: Jul 4, 2024
 *      Author: ADMIN
 */


#include"stm32f072rbt6.h"
#define LOW 0
#define HIGH 1
#define BTN_PRESSED LOW
void delay(uint32_t d);
int main(void)
{
	GPIO_Handle_t GpioLedPC0,GpioButtonPC5;
	GpioLedPC0.pGPIOx = GPIOC;
	GpioButtonPC5.pGPIOx=GPIOC;

	GpioLedPC0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PU_PD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLedPC0);


	GpioButtonPC5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButtonPC5.GPIO_PinConfig.GPIO_PinMode=	GPIO_MODE_INPUT;
	GpioButtonPC5.GPIO_PinConfig.GPIO_PinSpeed=	GPIO_SPEED_FAST;

	GpioButtonPC5.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_Init(&GpioButtonPC5);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GpioButtonPC5.pGPIOx, GpioButtonPC5.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			delay(200);
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_0);

		}
	}

	return 0;
}
void delay(uint32_t d)
{

}
