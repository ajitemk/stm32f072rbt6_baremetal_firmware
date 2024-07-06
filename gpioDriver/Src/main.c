/*
 * main.c
 *
 *  Created on: Jul 4, 2024
 *      Author: ADMIN
 */


int main(void)
{

	return 0;
}

void EXTI4_15_IRQHandler(void)
{
	GPIO_IRQHandling(PC5);
}
