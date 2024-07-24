/*
 * usart_driver.c
 *
 *  Created on: Jul 18, 2024
 *      Author: ADMIN
 */
#include"stm32f072rbt6.h"
void USART_PeriClockControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi)
{

}
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 TODO

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << TODO);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << TODO );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << TODO) | ( 1 << TODO) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << TODO ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << TODO);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << TODO);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = TODO;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.TODO << TODO;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->TODO = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << TODO);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= TODO

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		TODO
	}


	pUSARTHandle->pUSARTx->TODO = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//W
}//end of usart_init
void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}


void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t EnorDi)
{
	USART2_CR1_BITS=UE;
	if(ENABLE==EnorDi)
	{
		pUSARTx->USART_CR1 |= (1<<USART2_CR1_BITS);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(1<<USART2_CR1_BITS);
	}
}
/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
	   //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until TXE flag is set in the SR
			while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

	         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
			if(pUSARTHandle->USART_Config.TODO == USART_WORDLEN_9BITS)
			{
				//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
				pdata = (uint16_t*) pTxBuffer;
				pUSARTx->USART_TDR = (*pdata & (uint16_t)0x01FF);

				//check for USART_ParityControl
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used in this transfer. so, 9bits of user data will be sent
					//Implement the code to increment pTxBuffer twice
					pTxBuffer++;
					pTxBuffer++;
				}
				else
				{
					//Parity bit is used in this transfer . so , 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pTxBuffer++;
				}
			}
			else
			{
				//This is 8bit data transfer
				pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);

				//Implement the code to increment the buffer address
				pTxBuffer++;
			}
		}

		//Implement the code to wait till TC flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
{
	 //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until RXNE flag is set in the SR
			while()

			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == TODO)
				{
					//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits. so, mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)TODO);

					//Now increment the pRxBuffer two times
					TODO
				}
				else
				{
					//Parity is used, so, 8bits will be of user data and 1 bit is parity
					 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					 //Increment the pRxBuffer
					TODO
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					 *pRxBuffer = TODO;
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pRxBuffer = (uint8_t) TODO

				}

				//increment the pRxBuffer
				pRxBuffer++;
			}
		}

}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TODO;

		if(txstate != USART_BUSY_IN_TX)
		{
			pUSARTHandle->TODO = Len;
			pUSARTHandle->pTxBuffer = TODO;
			pUSARTHandle->TxBusyState = TODO;

			//Implement the code to enable interrupt for TXE
			TODO


			//Implement the code to enable interrupt for TC
			TODO


		}

		return txstate;
}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->TODO;

		if(rxstate != TODO)
		{
			pUSARTHandle->RxLen = Len;
			pUSARTHandle->pRxBuffer = TODO;
			pUSARTHandle->RxBusyState = TODO;

			//Implement the code to enable interrupt for RXNE
			TODO

		}

		return rxstate;
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName)
{

}
void USART_ClearFlag(USRT_RegDef_t *pUSARTx,uint16_t StatusFlagName)
{

}
void USART_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

}
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

}
