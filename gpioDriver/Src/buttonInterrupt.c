/*
 * buttonInterrupt.c
 *
 *  Created on: Jul 6, 2024
 *      Author: ADMIN
 */

#include"stm32f072rbt6.h"
#include<string.h>
#define HIGH 1
#define LOW 0
#define BTNPRESSED LOW
uint32_t moc_cut=0;
int main(void)
{
	GPIO_Handle_t GpioLedPC0,GpioButtonPC5;
	memset(&GpioLedPC0,0,sizeof(GpioLedPC0));
		memset(&GpioButtonPC5,0,sizeof(GpioButtonPC5));
	GpioLedPC0.pGPIOx=GPIOC;
		GpioButtonPC5.pGPIOx=GPIOA;


	GpioLedPC0.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLedPC0.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PU_PD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLedPC0);

	GpioButtonPC5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GpioButtonPC5.GPIO_PinConfig.GPIO_PinMode=	GPIO_MODE_IT_FT;
	GpioButtonPC5.GPIO_PinConfig.GPIO_PinSpeed=	GPIO_SPEED_FAST;
	GpioButtonPC5.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_Init(&GpioButtonPC5);

	//IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0_1, NVIC_IRQ_PRI_12);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0_1, ENABLE);
	while(1)
	{
		if(moc_cut == 1)
		{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_0);
		moc_cut=0;
		}
	}


}//end of main

void EXTI0_1_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_1);//clear the pending event from exti line
	moc_cut++;
}
