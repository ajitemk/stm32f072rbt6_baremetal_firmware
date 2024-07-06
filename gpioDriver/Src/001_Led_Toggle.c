/*
 * 001_Led_Toggle.c
 *
 *  Created on: Jul 2, 2024
 *      Author: ADMIN
 */


#include"stm32f072rbt6.h"
void delay(uint32_t d);
int main(void)
{


	GPIO_Handle_t GpioLedPC0,GpioLedPA1;
	GpioLedPC0.pGPIOx = GPIOC;
	GpioLedPA1.pGPIOx = GPIOA;

	GpioLedPC0.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PU_PD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLedPC0);


	GpioLedPA1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GpioLedPA1.GPIO_PinConfig.GPIO_PinMode=	GPIO_MODE_OUTPUT;
	GpioLedPA1.GPIO_PinConfig.GPIO_PinSpeed=	GPIO_SPEED_LOW;
	GpioLedPA1.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GpioLedPA1.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PU_PD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLedPA1);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_0);
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
		delay(30000);
	}
	return 0;
}
void delay(uint32_t d)
{
	for(int i=d;i>0;i--);
}