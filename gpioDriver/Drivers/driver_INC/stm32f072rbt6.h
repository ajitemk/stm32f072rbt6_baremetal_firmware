/*
 * stm32f072rbt6.h
 *
 *  Created on: Jun 26, 2024
 *      Author: ADMIN
 */

#ifndef DRIVER_INC_STM32F072RBT6_H_
#define DRIVER_INC_STM32F072RBT6_H_
#include<stdint.h>
#define __VO volatile
/*************************START: Processor Specific Details
 *
 *ARM Cortex m0 processor NVIC ISER Register Address
 */
#define NVIC_ISER ((__VO uint32_t *)0xE000E100)
#define NVIC_ICER ((__VO uint32_t *)0xE000E180)
#define NVIC_ISPR ((__VO uint32_t *)0xE000E200)
#define NVIC_ICPR ((__VO uint32_t *)0xE000E280)
#define NVIC_IPR ((__VO uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

//system memory = ROM ADDRESS
#define FLASH_BASEADDR 0x08000000UL
#define SRAM1_BASEADDR 0x20000000UL
#define SRAM SRAM1_BASEADDR

// ahb and apb bus peripheal base address
#define PERIPHERAL_BASE 		(0x40000000U)
#define APB1PERIPH_BASE			(PERIPHERAL_BASE+0)
#define APB2PERIPH_BASE			(0x40010000U)
#define AHB1PERIPH_BASE			(0x40020000U)
#define AHB2PERIPH_BASE			(0x48000000U)

// gpio bus address

#define GPIOA_BASEADDR			(AHB2PERIPH_BASE)
#define GPIOB_BASEADDR			(AHB2PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR			(AHB2PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASE+0x1000)
#define GPIOF_BASEADDR			(AHB2PERIPH_BASE+0x1400)

//base address of peripheral which are hanging on apb1 bus
#define I2C1_BASEADDR			(PERIPHERAL_BASE+0x5400)
#define I2C2_BASEADDR			(PERIPHERAL_BASE+0x5800)
#define TIM2_BASEADDR			(PERIPHERAL_BASE+0x0000)
#define TIM3_BASEADDR			(PERIPHERAL_BASE+0x0400)
#define TIM6_BASEADDR			(PERIPHERAL_BASE+0x1000)

#define TIM7_BASEADDR			(PERIPHERAL_BASE+0x1400)

//base address of peripheral which are hanging on apb2 bus
#define TIM1_BASEADDR			(APB2PERIPH_BASE+0x2C00)
#define USART6_BASEADDR			(APB2PERIPH_BASE+0x1400)
#define USART7_BASEADDR			(APB2PERIPH_BASE+0x1800)
#define USART8_BASEADDR			(APB2PERIPH_BASE+0x1C00)
#define ADC_BASEADDR			(APB2PERIPH_BASE+0x2400)
#define SP1_BASEADDR			(APB2PERIPH_BASE+0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE+0x3800)
#define USART2_BASEADDR			(APB1PERIPH_BASE+0x4400)
#define RCC_BASEADDR			(AHB1PERIPH_BASE+0x1000)


#define EXTI_BASEADDR			(APB2PERIPH_BASE+0x0400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE)
/* peripheral refister defination structure for gpio*/

typedef struct{
	__VO uint32_t MODER;		/*GPIO port mode register				   0x00	*/
	__VO uint32_t OTYPER;	/*GPIO port output type register		   0x04	*/
	__VO uint32_t OSPEEDR;	/*GPIO port output speed register		   0x08	*/
	__VO uint32_t PUPDR;		/*GPIO port pull-up/pull-down register	   0x0C	*/
	__VO uint32_t IDR;		/*GPIO port input data register			   0x10	*/
	__VO uint32_t ODR;		/*GPIO port output data register		   0x14	*/
	__VO uint32_t BRR;		/*GPIO port bit reset register			   0x18	*/
	__VO uint32_t BSRR;		/*GPIO port bit set/reset register		   0x1C	*/
	__VO uint32_t LCKR;		/*GPIO port configuration lock register	   0x20	*/
	__VO uint32_t AFR[2];	/*[0]GPIO alternate function low register  0x24	*/
	/*[1]GPIO alternate function high register 0x28	*/
}GPIO_RegDef_t;

/*
 * peripheral refister defination structure for RCC
 */
typedef struct{
	__VO uint32_t RCC_CR;
	__VO uint32_t RCC_CFGR;
	__VO uint32_t RCC_CIR;
	__VO uint32_t RCC_APB2RSTR;
	__VO uint32_t RCC_APB1RSTR;
	__VO uint32_t RCC_AHBENR;
	__VO uint32_t RCC_APB2ENR;
	__VO uint32_t RCC_APB1ENR;
	__VO uint32_t RCC_BDCR;
	__VO uint32_t RCC_CSR;
	__VO uint32_t RCC_AHBRSTR;
	__VO uint32_t RCC_CFGR2;
	__VO uint32_t RCC_CFGR3;
	__VO uint32_t RCC_CR2;
}RCC_RegDef_t;

/*
 * peripheral register defination structure for exti
 */
typedef struct{
	__VO uint32_t EXTI_IMR;
	__VO uint32_t EXTI_EMR;
	__VO uint32_t EXTI_RTSR;
	__VO uint32_t EXTI_FTSR;
	__VO uint32_t EXTI_SWIER;
	__VO uint32_t EXTI_PR;
}EXTI_RegDef_t;
/*
 * peripheral refister defination structure for SYSCFG
 */
typedef struct{
	__VO uint32_t SYSCFG_CFGR1;
	__VO uint32_t rev;
	__VO uint32_t SYSCFG_EXTICR[4];
	__VO uint32_t SYSCFG_CFGR2;
}SYSCFG_RegDef_t;

/*
 * peripheral refister defination structure for i2c
 */
typedef struct{
	__VO uint32_t I2C_CR1;
	__VO uint32_t I2C_CR2;
	__VO uint32_t I2C_OAR1;
	__VO uint32_t I2C_OAR2;
	__VO uint32_t I2C_TIMINGR;
	__VO uint32_t I2C_TIMEOUTR;
	__VO uint32_t I2C_ISR;
	__VO uint32_t I2C_ICR;
	__VO uint32_t I2C_PECR;
	__VO uint32_t I2C_TXDR;
}I2C_RegDef_t;


/*
 * peripheral register definition structure for usart
 */
typedef struct{
	__VO uint32_t USART_CR1;	//00
	__VO uint32_t USART_CR2;	//04
	__VO uint32_t USART_CR3;	//08
	__VO uint32_t USART_BRR;	//0c
	__VO uint32_t USART_GTPR;	//10
	__VO uint32_t USART_RTOR;	//14
	__VO uint32_t USART_RQR;	//18

	__VO uint32_t USART_ISR;	//1c
	__VO uint32_t USART_ICR;	//20
	__VO uint32_t USART_RDR;	//24
	__VO uint32_t USART_TDR;	//28	_
}USART_RegDef_t;
typedef struct{

}I2C2_RegDef_t;
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * i2c peripheral definition macro
 */
#define I2C1	(I2C_RegDef_t*)(I2C1_BASEADDR)
#define I2C2	(I2C_RegDef_t*)(I2C2_BASEADDR)

/*
 * usart1 peripheral definition macro
 */
#define USART2	(USART_RegDef_t*)(USART2_BASEADDR)
//#define I2C2	(I2C_RegDef_t*)(I2C2_BASEADDR)



/* Clock Enable for GPIO Peripherals
 * AHB peripheral clock enable register (RCC_AHBENR)*/
#define GPIOA_PCLK_EN	(RCC->RCC_AHBENR |= (1<<17))
#define GPIOB_PCLK_EN	(RCC->RCC_AHBENR |= (1<<18))
#define GPIOC_PCLK_EN	(RCC->RCC_AHBENR |= (1<<19))
#define GPIOD_PCLK_EN	(RCC->RCC_AHBENR |= (1<<20))
#define GPIOE_PCLK_EN	(RCC->RCC_AHBENR |= (1<<21))
#define GPIOF_PCLK_EN	(RCC->RCC_AHBENR |= (1<<22))
/* Clock disable for GPIO Peripherals
 * AHB peripheral clock enable register (RCC_AHBENR)*/
/*APB peripheral clock enable register 2 (RCC_APB2ENR)
 * SPI1 ClOCK ENABLE*/
#define GPIOA_PCLK_DI	(RCC->RCC_AHBENR &= (~(1<<17)))
#define GPIOB_PCLK_DI	(RCC->RCC_AHBENR &= (~(1<<18)))
#define GPIOC_PCLK_DI	(RCC->RCC_AHBENR &= (~(1<<19)))
#define GPIOD_PCLK_DI	(RCC->RCC_AHBENR &= (~(1<<20)))
#define GPIOE_PCLK_DI	(RCC->RCC_AHBENR &= (~(1<<21)))
#define GPIOF_PCLK_DI	(RCC->RCC_AHBENR &= (~(1<<22)))

#define SPI1_PCLK_EN ((RCC->RCC_APB2ENR) |= (1<<12))
/*APB1 peripheral clock enable register 1 (RCC_APB1ENR)*/
#define SPI2_PCLK_EN ((RCC->RCC_APB1ENR) |= (1<<14))

#define SYSCFG_PCLK_EN ((RCC->RCC_APB2ENR)|= (1<<0))

/*APB1 peripheral clock disable register 1 (RCC_APB1ENR)*/
#define SPI2_PCLK_DI ((RCC->RCC_APB1ENR) &= (~(1<<14)))

/* Clock Enable for I2Cx Peripherals
 * APB peripheral clock enable register 1 (RCC_APB1ENR)*/
#define I2C1_PCLK_EN	(RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN	(RCC->RCC_APB1ENR |= (1<<22))
/*
 * clcok enable for usart2 register
 */
//APB peripheral clock enable register 1 (RCC_APB1ENR)
#define USART2_PCLK_EN 	(RCC->RCC_APB1ENR |= (1<<17))

/*
 * clcok enable for usart2 register
 */
//APB peripheral clock enable register 1 (RCC_APB1ENR)
#define USART2_PCLK_DI 	(RCC->RCC_APB1ENR &= ~(1<<17))


/*
 * bit position defination for i2c register
 */
/*
 * bit position defination for I2C control register 1
 */
//I2C_CR1_PIN;
#define  I2C_CR1_PE 		0
#define	 I2C_CR1_TXIE		1
#define I2C_CR1_RXIE		2
#define I2C_CR1_ADDRIE		3
#define I2C_CR1_NACKIE		4
#define I2C_CR1_STOPIE		5
#define I2C_CR1_TCIE		6
#define I2C_CR1_ERRIE		7
#define I2C_CR1_DNF1		8
#define I2C_CR1_DNF2		9
#define I2C_CR1_DNF3		10
#define I2C_CR1_DNF4		11
#define I2C_CR1_ANFOFF		12
#define I2C_CR1_RESERVED	13
#define I2C_CR1_TXDMAEN		14
#define I2C_CR1_RXDMAEN		15
#define I2C_CR1_SBC			16
#define I2C_CR1_NOSTRETCH	17
#define I2C_CR1_WUPEN		18
#define I2C_CR1_GCEN		19
#define I2C_CR1_SMBHEN		20
#define I2C_CR1_SMBDEN		21
#define I2C_CR1_ALERTEN		22
#define I2C_CR1_PECEN		23
#define I2C_CR1_RESERVED_1	24
#define I2C_CR1_RESERVED_2	25
#define I2C_CR1_RESERVED_3	26
#define I2C_CR1_RESERVED_4	27
#define I2C_CR1_RESERVED_5	28
#define I2C_CR1_RESERVED_6	29
#define I2C_CR1_RESERVED_7	30
#define I2C_CR1_RESERVED_8	31

//I2C_CR2_PIN;
#define  I2C_CR2_SADD_0 		0
#define	 I2C_CR2_SADD_1			1
#define I2C_CR2_SADD_2			2
#define I2C_CR2_SADD_3			3
#define I2C_CR2_SADD_4			4
#define I2C_CR2_SADD_5			5
#define I2C_CR2_SADD_6			6
#define I2C_CR2_SADD_7			7
#define I2C_CR2_SADD_8			8
#define I2C_CR2_SADD_9			9
#define I2C_CR2_RD_WRN			10
#define I2C_CR2_ADD10			11
#define I2C_CR2_HEAD10R			12
#define I2C_CR2_START			13
#define I2C_CR2_STOP			14
#define I2C_CR2_NACK			15
#define I2C_CR2_NBYTES_0		16
#define I2C_CR2_NBYTES_1		17
#define I2C_CR2_NBYTES_2		18
#define I2C_CR2_NBYTES_3		19
#define I2C_CR2_NBYTES_4		20
#define I2C_CR2_NBYTES_5		21
#define I2C_CR2_NBYTES_6		22
#define I2C_CR2_NBYTES_7		23
#define I2C_CR2_RELOAD			24
#define I2C_CR2_AUTOEND			25
#define I2C_CR2_PECBYTE			26
#define I2C_CR2_RESERVED_0		27
#define I2C_CR2_RESERVED_1		28
#define I2C_CR2_RESERVED_2		29
#define I2C_CR2_RESERVED_3		30
#define I2C_CR2_RESERVED_4		31
//typedef enum
//{
//	 PE=0,
//	 TXIE,
//	 RXIE,
//	 ADDRIE,
//	 NACKIE,
//	 STOPIE,
//	 TCIE,
//	 ERRIE,
//	 DNF1,
//	 DNF2,
//	 DNF3,
//	 DNF4,
//	 ANFOFF,
//	 RESERVED,
//	 TXDMAEN,
//	 RXDMAEN,
//	 SBC,
//	 NOSTRETCH,
//	 WUPEN,
//	 GCEN,
//	 SMBHEN,
//	 SMBDEN,
//	 ALERTEN,
//	 PECEN,
//	 RESERVED_1,
//	 RESERVED_2,
//	 RESERVED_3,
//	 RESERVED_4,
//	 RESERVED_5,
//	 RESERVED_6,
//	 RESERVED_7,
//	 RESERVED_8
//}I2C_CR1_PIN;

/*
 *	Macro to Reset GPIO Peripheral
 */
#define GPIOA_REG_RESET  do{RCC->RCC_AHBRSTR |= (1<<17); RCC->RCC_AHBRSTR &= ~(1<<17);}while(0)
#define GPIOB_REG_RESET  do{RCC->RCC_AHBRSTR |= (1<<18); RCC->RCC_AHBRSTR &= ~(1<<18);}while(0)
#define GPIOC_REG_RESET  do{RCC->RCC_AHBRSTR |= (1<<19); RCC->RCC_AHBRSTR &= ~(1<<19);}while(0)
#define GPIOD_REG_RESET  do{RCC->RCC_AHBRSTR |= (1<<20); RCC->RCC_AHBRSTR &= ~(1<<20);}while(0)
#define GPIOE_REG_RESET  do{RCC->RCC_AHBRSTR |= (1<<21); RCC->RCC_AHBRSTR &= ~(1<<21);}while(0)
#define GPIOF_REG_RESET  do{RCC->RCC_AHBRSTR |= (1<<22); RCC->RCC_AHBRSTR &= ~(1<<22);}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0 :\
									(x == GPIOB)?1 :\
									(x == GPIOC)?2 :\
									(x == GPIOD)?3 :\
									(x == GPIOE)?4 :\
									(x == GPIOF)?5 :0)


#define IRQ_NO_EXTI0_1	5 // see position in reference manual "Vector table (continued)"
#define IRQ_NO_EXTI2_3	6
#define IRQ_NO_EXTI4_15	7

/*
 * macro for all priority IRQ Priority
 */
#define NVIC_IRQ_PRI_0 	0
#define NVIC_IRQ_PRI_1	1
#define NVIC_IRQ_PRI_2	2
#define NVIC_IRQ_PRI_3	3
#define NVIC_IRQ_PRI_4	4
#define NVIC_IRQ_PRI_5	5
#define NVIC_IRQ_PRI_6	6
#define NVIC_IRQ_PRI_7	7
#define NVIC_IRQ_PRI_8	8
#define NVIC_IRQ_PRI_9	9
#define NVIC_IRQ_PRI_10	10
#define NVIC_IRQ_PRI_11	11
#define NVIC_IRQ_PRI_12	12
#define NVIC_IRQ_PRI_13	13
#define NVIC_IRQ_PRI_14	14
#define NVIC_IRQ_PRI_15 15
/*some generic MACRO */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include"stm32f072rbt6_GPIO_Driver.h"
#include"i2c_Driver.h"
#include"usart_driver.h"

#endif /* DRIVER_INC_STM32F072RBT6_H_ */
