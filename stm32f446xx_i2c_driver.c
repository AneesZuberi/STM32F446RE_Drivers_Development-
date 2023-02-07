/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 13 Dec 2022
 *      Author: Anees
 */
#include "stm32f446xx_i2c_driver.h"

/*****************************************************************************
 *                   		Helper Functions
 * ***************************************************************************/
static void I2C_GenerateStart(I2C_RegDef_t *pI2cx)
{
	pI2cx->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ExecuteAddr(I2C_RegDef_t *pI2cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);  //r/w bit set as 0
	pI2cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddrWrite(I2C_RegDef_t *pI2cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;  //r/w bit set as 0
	pI2cx->DR = SlaveAddr;
}
static void I2C_ClearADDR(I2C_RegDef_t *pI2cx)
{
	uint32_t dummyread = pI2cx->SR1;
	dummyread = pI2cx->SR2;
	(void)dummyread;
}
static void I2C_ClearADDRIT(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy;
	//check devise mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is master
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//disable ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear ADDR
				dummy = pI2CHandle->pI2Cx->SR1;
				dummy = pI2CHandle->pI2Cx->SR2;
				(void)dummy;
			}
		}else
		{
			dummy = pI2CHandle->pI2Cx->SR1;
			dummy = pI2CHandle->pI2Cx->SR2;
			(void)dummy;
		}
	}else
	{
		//device is slave
		dummy = pI2CHandle->pI2Cx->SR1;
		dummy = pI2CHandle->pI2Cx->SR2;
		(void)dummy;
	}

}
static void I2C_GenerateStop(I2C_RegDef_t *pI2cx)
{
	pI2cx->CR1 |= (1 << I2C_CR1_STOP);
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}
void I2C_CloseRcvData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_EN)
			I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}
void I2C_SlaveEnorDiCallBackEvent(I2C_RegDef_t *pI2C, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2C->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2C->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 |= (1 << I2C_CR2_ITERREN);
	}else
	{
		pI2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}
/******************************************************************************/

/*******************************************************************************
 * @fn			-	I2C_PeripheralControl
 *
 * brief		-	to enable or disable I2C peripehral
 *
 * @param[in]	-	Base address of I2C peripheral
 * @param[in]	-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}

/*******************************************************************************
 * @fn			-	I2C_PeriCLKControl
 *
 * brief		-	Enables or Disables peripheral clock for the given I2C
 *
 * @param[in]	-	Base address of I2C peripheral
 * @param[in]	-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_PeriCLKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PR_CLK_EN();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PR_CLK_EN();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PR_CLK_EN();
			}
		}
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PR_CLK_DI();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PR_CLK_DI();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PR_CLK_DI();
			}
		}
}

/*******************************************************************************
 * @fn			-	I2C_DeInit
 *
 * brief		-	De-Initialize I2C peripheral
 *
 * @param[in]	-	Base address of I2C peripheral
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RST();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RST();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RST();
	}
}

/*******************************************************************************
 * @fn			-	I2C_Init
 *
 * brief		-	Initialize I2C peripheral
 *
 * @param[in]	-	I2C handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint8_t tempreg = 0;

	//ACK control bit
	tempreg |= pI2CHandle->I2CConfig.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field for CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own  address
	tempreg = pI2CHandle->I2CConfig.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLSPEED <= I2C_SCLSPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSPEED));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSPEED));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSPEED));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration -- [Fpclk1 * tr(max)] + 1
	if(pI2CHandle->I2CConfig.I2C_SCLSPEED == I2C_SCLSPEED_SM)
	{
		// for SM
		tempreg = (RCC_GetPCLK1Value() / 1000000U) +1;
	}else
	{
		// for FM
		tempreg = ((RCC_GetPCLK1Value() * 300)/ 10000000000U) +1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/*******************************************************************************
 * @fn			-	I2C_MasterSendData
 *
 * brief		-	Master send data to slave
 *
 * @param[in]	-	I2C handle
 * @param[in]	-	Address of Transmission Buffer
 * @param[in]	-	Length of data to transmit
 * @param[in]	-	Slave address
 * @param[in]	-	Repeated start macro value
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate start condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);

	//2. Confirm start is completed by checking SB flag of SR1
	while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send address of slave with r/w bit set to 0
	I2C_ExecuteAddr(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm address phase is complete by checking ADDR flag in SR1
	while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear ADDR flag by reading SR1 and reading SR2
	I2C_ClearADDR(pI2CHandle->pI2Cx);

	//6. Send data until Len becomes 0
	while(Len > 0)
	{
		while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len = 0 wait for TXE = 1 and BTF = 1
	while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate stop condition
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStop(pI2CHandle->pI2Cx);

}

/*******************************************************************************
 * @fn			-	I2C_MasterReceiveData
 *
 * brief		-	Master receive data from slave
 *
 * @param[in]	-	I2C handle
 * @param[in]	-	Address of Reception Buffer
 * @param[in]	-	Length of data to receive
 * @param[in]	-	Slave address
 * @param[in]	-	Repeated start macro value
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate start condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);

	//2. Confirm start is completed by checking SB flag of SR1
	while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send address of slave with r/w bit set to 0
	I2C_ExecuteAddrWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm address phase is complete by checking ADDR flag in SR1
	while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Procedure to read only 1 byte
	if(Len == 1)
	{
		// Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		// clear ADDR flag
		I2C_ClearADDR(pI2CHandle->pI2Cx);

		// wait for RXNE flag
		while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStop(pI2CHandle->pI2Cx);

		//read data in to Buffer
		*pRxBuffer =pI2CHandle->pI2Cx->DR;
	}

	// procedure for more 1 byte of data reception
	if(Len > 1)
	{
		//clear ADDR flag
		I2C_ClearADDR(pI2CHandle->pI2Cx);

		//read the data until Len becomes 0
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait for RXNE
			while( ! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2)
			{
				// Disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//generate STOP
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStop(pI2CHandle->pI2Cx);
			}

			//read data
			*pRxBuffer =pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	//re-enable acking
	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_EN)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}

/*******************************************************************************
 * @fn			-	I2C_MasterSendDataIT
 *
 * brief		-	Master send data to slave in interrupt mode
 *
 * @param[in]	-	I2C handle
 * @param[in]	-	Address of Transmission Buffer
 * @param[in]	-	Length of data to transmit
 * @param[in]	-	Slave address
 * @param[in]	-	Repeated start macro value
 *
 * @return		-	I2C peripheral state
 *
 * @note		-	none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate start condition
		I2C_GenerateStart(pI2CHandle->pI2Cx);

		//Enable ITBUFEN control bit  for interrupt enabling
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVTEN  for event interrupts
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN  for error interrupts
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*******************************************************************************
 * @fn			-	I2C_MasterReceiveDataIT
 *
 * brief		-	Master receive data from slave in interrupt mode
 *
 * @param[in]	-	I2C handle
 * @param[in]	-	Address of Reception Buffer
 * @param[in]	-	Length of data to receive
 * @param[in]	-	Slave address
 * @param[in]	-	Repeated start macro value
 *
 * @return		-	I2C peripheral state
 *
 * @note		-	none
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->RxSize = Len;

		//Generate start condition
		I2C_GenerateStart(pI2CHandle->pI2Cx);

		//Enable ITBUFEN control bit  for interrupt enabling
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVTEN  for event interrupts
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN  for error interrupts
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*******************************************************************************
 * @fn			-	I2C_IRQITConfig
 *
 * brief		-	IRQ configuration for I2C peripheral
 *
 * @param[in]	-	IRQ number
 * @param[in]	-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*******************************************************************************
 * @fn			-	I2C_IRQPriorityConfig
 *
 * brief		-	IRQ priority configuration for I2C peripheral
 *
 * @param[in]	-	IRQ number
 * @param[in]	-	IRQ priority value
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;
	uint8_t shift_amount = (iprx_sec * 8) + (8 - NO_BITS_IMP);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*******************************************************************************
 * @fn			-	I2C_SlaveSendData
 *
 * brief		-	Slave send data to master
 *
 * @param[in]	-	Base address of I2C peripheral
 * @param[in]	-	Data to transmit
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

/*******************************************************************************
 * @fn			-	I2C_SlaveReceiveData
 *
 * brief		-	Slave receive data from master
 *
 * @param[in]	-	Base address of I2C peripheral
 *
 * @return		-	Data received
 *
 * @note		-	none
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}

/*******************************************************************************
 * @fn			-	I2C_EV_IRQHandling
 *
 * brief		-	IRQ Event handling for I2C peripheral
 *
 * @param[in]	-	I2C Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode
	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_SR1_SB);
	//1. Handle for interrupt generated for SB event
	// Note: SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//Interrupt is generated due to SB event
		//In this block execute addresss phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddrWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else
		{
			I2C_ExecuteAddr(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. ADDR event
	//Note : Master mode : Address is sent
	//       Slave mode : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRIT(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. BTF event
	if(temp1 && temp3)
	{
		//BTF flag is set
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	{
		// make sure TXE is also set
		if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
		{
			// Both BTF, TXE = 1
			if(pI2CHandle->TxLen == 0)
			{
				//Genrate stop
				if(pI2CHandle->Sr == I2C_DISABLE_SR)
					I2C_GenerateStop(pI2CHandle->pI2Cx);
				//reset all transmission
				I2C_CloseSendData(pI2CHandle);
				//notify application about TX completion
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
		}
	}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	{
		;
	}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. STOP event
	// NOTE: only in slave mode
	if(temp1 && temp3)
	{
		//STOPF flag is set
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. TXE event
	if(temp1 && temp2 && temp3)
	{
		//check for device mode, if master
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//load data in DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
					// decrement len
					pI2CHandle->TxLen--;
					//increase tx buffer
					pI2CHandle->pTxBuffer++;
				}
			}else
			{
				//in slave mode
				//make sure slave is in TX mode
				if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
				{
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
				}
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. RXNE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
		//RXNE flag is set
		//data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
				if(pI2CHandle->RxLen == 0)
				{
					I2C_GenerateStop(pI2CHandle->pI2Cx);
					//close reception and notify application
					I2C_CloseRcvData(pI2CHandle);
					//notify application about TX completion
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}else
		{
			//slave mode
			//make sure slave is in TX mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*******************************************************************************
 * @fn			-	I2C_ER_IRQHandling
 *
 * brief		-	IRQ Error handling for I2C peripheral
 *
 * @param[in]	-	I2C Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	//get status of ITERREN bit in CR2
	temp2 = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN));

	/**********************Check for Bus Error ****************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR));
	if(temp1 && temp2)
	{
		//This is bus error

		//Clear bus error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		//notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}
	/**********************Check for Arbitration Lost Error ****************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO));
	if(temp1 && temp2)
	{
		//This is ARLO error

		//Clear ARLO error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		//notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}
	/**********************Check for ACK failure Error ****************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF));
	if(temp1 && temp2)
	{
		//This is ack error

		//Clear ack error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		//notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}
	/**********************Check for Overrun Error ****************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR));
	if(temp1 && temp2)
	{
		//This is ovr error

		//Clear ovr error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		//notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}
	/**********************Check for Timeout Error ****************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT));
	if(temp1 && temp2)
	{
		//This is timeout error

		//Clear timeout error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		//notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

/*******************************************************************************
 * @fn			-	I2C_ManageAcking
 *
 * brief		-	Acknowledge management
 *
 * @param[in]	-	Base address of I2C peripheral
 * @param[in]	-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
