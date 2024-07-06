/*
 * stm32f072rbt6_GPIO_Driver.c
 *
 *  Created on: Jun 28, 2024
 *      Author: ADMIN
 */
#include"stm32f072rbt6.h"

/*	Peripheral clock setup
 */
/*************************************************************************
 * 	@fn				GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
 * 	@brief			bus which are connected to gpio control the clock of that bus
 * 	@Param[in]		GPIO_RegDef_t *pGPIOx, base address of GPIO
 * 	@Param[in]		uint8_t EnorDi,	Enable or disable
 *
 * 	@return
 * 	@Note
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN;
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN;
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN;
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN;
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN;
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN;
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI;
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI;
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI;
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI;
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI;
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI;
		}
	}
}

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER|=temp;
	}
	else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure the FTSR
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR Bit
			EXTI->EXTI_RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure the RTSR
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the EXTI_FTSR Bit
			EXTI->EXTI_FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure the FTSR and RTSR
			//configure the RTSR
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the EXTI_FTSR Bit
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO Port  selection in SYSCFG_EXTICR
		uint8_t temp1 =	pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;  // 1
		uint8_t temp2 =	pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;	//1
		uint32_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		if(portcode == 0)
		{
			SYSCFG->SYSCFG_EXTICR[temp1] &= ~(0x0f<<(4*temp2));

		}
		else
		{
			SYSCFG->SYSCFG_EXTICR[temp1]|=portcode<<(4*temp2);

		}
		SYSCFG_PCLK_EN;

		//3. enable the exti interuupt
		EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;
	//speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;
	//output type
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER|=temp;
	//pupd register
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR|=temp;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN)
	{
		//configure the alt function

	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET;
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET;
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET;
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET;
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET;
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET;
	}
}

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber)
{
	uint8_t value=0;
	value=(uint8_t)((pGPIOx->IDR >> pinNumber) & 0b0000000000000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}


}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR |= value;
}
//void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint32_t IPRx = (IRQNumber / 4);
	uint32_t ipr_section = (IRQNumber % 4);
	uint8_t shiftAmount= (8*ipr_section)+(8-NO_PR_BITS_IMPLEMENTED);
	//uint8_t shiftAmount= (8*ipr_section);
	*(NVIC_IPR + IPRx) |= (IRQPriority << (shiftAmount));
}
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//enable the interrupt for irq
		//configure	ISER register
		*NVIC_ISER |= (1<<IRQNumber); //Cortex-M0 Devices Generic User Guide
	}
	else
	{
		//disable the interrupt for irq;
		//configure the ICER register

	}
}
void GPIO_IRQHandling(uint8_t pinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->EXTI_PR & (1<<pinNumber))
	{
		//clear
		EXTI->EXTI_PR |= (1<<pinNumber);
	}
}
