/*
 * stm32f072rbt6_GPIO_Driver.h
 *
 *  Created on: Jun 28, 2024
 *      Author: ADMIN
 */

#ifndef DRIVER_INC_STM32F072RBT6_GPIO_DRIVER_H_
#define DRIVER_INC_STM32F072RBT6_GPIO_DRIVER_H_

typedef struct{
	uint8_t GPIO_PinNumber;//possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode; //Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; //Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;//@GPIO_PIN_PUSHPULL_AND_PULLDOWN
	uint8_t GPIO_PinOPType;//possible values from @GPIO_PIN_OUTPUT_TYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;// this holds the base address of the gpio port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;//this hold gpio pin configuration setting
}GPIO_Handle_t;


/*@GPIO_PIN_NUMBER
 * GPIO PIN NUMBER
 */
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

/*@GPIO_PIN_MODES
 * GPIO Pin Possible Modes
 */
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALT_FN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

/*@GPIO_PIN_OUTPUT_TYPE
 * GPIO Pin Output types
 */
#define GPIO_OP_TYPE_PP	0
#define GPIO_OP_TYPE_OD	1

/*@GPIO_PIN_SPEED
 * GPIO Pin Output Speed
 */

#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST 	2
#define GPIO_SPEED_HIGH 	3

/*@GPIO_PIN_PUSHPULL_AND_PULLDOWN
 *GPIO port pull-up/pull-down register
 */
#define GPIO_NO_PU_PD	0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
#define GPIO_PIN_RESERVED	3
/*************************************************************************
 * 					API Supported GPIO Driver
 *
 *
 *************************************************************************/
/*	Peripheral clock setup
 */
/*************************************************************************
 * 	@fn
 * 	@brief
 * 	@Param[in]
 * 	@Param[in]
 *
 * 	@return
 * 	@Note
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
//void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* DRIVER_INC_STM32F072RBT6_GPIO_DRIVER_H_ */
