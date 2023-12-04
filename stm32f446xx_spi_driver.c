/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 25 Nov 2022
 *      Author: Anees
 */
#include "stm32f446xx_spi_driver.h"

static void spi_txe_it_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_it_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_it_handle(SPI_Handle_t *pSPIHandle);

/*******************************************************************************
 * @fn			-	SPI_PeriCLKControl
 *
 * brief		-	Enables or Disables peripheral clock for the given SPI
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_PeriCLKControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PR_CLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PR_CLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PR_CLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PR_CLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PR_CLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PR_CLK_DI();
			}
		}
}

/*******************************************************************************
 * @fn			-	SPI_Init
 *
 * brief		-	Initialize SPI peripheral
 *
 * @param[in]		-	SPI handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg=0;
	//confiure the spi cr1 register
	//1. configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure bus confige
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 <<SPI_CR1_BIDIMODE);
	}else 	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY mode should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. serial clock speed
	tempreg |=pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. data format
	tempreg |=pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. CPOL
	tempreg |=pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//5. CPHA
	tempreg |=pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//6. SSM
	tempreg |=pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*******************************************************************************
 * @fn			-	SPI_DeInit
 *
 * brief		-	De-Initialize SPI peripheral
 *
 * @param[in]		-	Base address of SPI peripheral
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RST();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RST();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RST();
	}
}

/*******************************************************************************
 * @fn			-	Get_FlagStatus
 *
 * brief		-	To get the status of given flag
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	Flag name
 *
 * @return		-	Flag Status (SET or RESET)
 *
 * @note		-	none
 */
uint8_t Get_FlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*******************************************************************************
 * @fn			-	SPI_Send
 *
 * brief		-	Send data through SPI in polling mode
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	Address of Transmission Buffer
 * @param[in]		-	Length of data to transmit
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(Get_FlagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_RESET); //checking the status of TX buffer flag
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 bit dff
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}else
		{
			//8 bit dff
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*******************************************************************************
 * @fn			-	SPI_Receive
 *
 * brief		-	Receive data through SPI in polling mode
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	Address of Reception Buffer
 * @param[in]		-	Length of data to receive
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len )
{
	while(Len > 0)
	{
		while(Get_FlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET); //checking the status of RX buffer flag
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 bit dff
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}else
		{
			//8 bit dff
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*******************************************************************************
 * @fn			-	SPI_PeripheralControl
 *
 * brief		-	to enable or disable SPI peripehral
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*******************************************************************************
 * @fn			-	SPI_SSIConfig
 *
 * brief		-	to enable or disable SPI software slave management
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}

/*******************************************************************************
 * @fn			-	SPI_SSOEConfig
 *
 * brief		-	to enable or disable SPI hardware slave management
 *
 * @param[in]		-	Base address of SPI peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}

/*******************************************************************************
 * @fn			-	SPI_IRQITConfig
 *
 * brief		-	IRQ configuration for SPI peripheral
 *
 * @param[in]		-	IRQ number
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn			-	SPI_IRQPriorityConfig
 *
 * brief		-	IRQ priority configuration for SPI peripheral
 *
 * @param[in]		-	IRQ number
 * @param[in]		-	IRQ priority value
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;
	uint8_t shift_amount = (iprx_sec * 8) + (8 - NO_BITS_IMP);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*******************************************************************************
 * @fn			-	SPI_SendIT
 *
 * brief		-	Send data through SPI in interrupt mode
 *
 * @param[in]		-	SPI Handle
 * @param[in]		-	Address of Transmission Buffer
 * @param[in]		-	Length of data to transmit
 *
 * @return		-	SPI peripheral state
 *
 * @note		-	none
 */
uint8_t SPI_SendIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1.Save TxBuffer and Length in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.Mark SPI state as busy(so other code can take over same spi)
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3.Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;
}

/*******************************************************************************
 * @fn			-	SPI_ReceiveIT
 *
 * brief		-	Receive data through SPI in interrupt mode
 *
 * @param[in]		-	SPI Handle
 * @param[in]		-	Address of Reception Buffer
 * @param[in]		-	Length of data to receive
 *
 * @return		-	SPI peripheral state
 *
 * @note		-	none
 */
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len )
{
	uint8_t state = pSPIHandle->RxState;
		if(state != SPI_BUSY_IN_RX)
		{
			//1.Save TxBuffer and Length in global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;
			//2.Mark SPI state as busy(so other code can take over same spi)
			pSPIHandle->RxState = SPI_BUSY_IN_RX;
			//3.Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
		}
		return state;
}

/*******************************************************************************
 * @fn			-	SPI_IRQHandling
 *
 * brief		-	IRQ handling for SPI peripheral
 *
 * @param[in]		-	SPI Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	//1. check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 & temp2)
	{
		//handle TXE
		spi_txe_it_handle(pSPIHandle);
	}

	//2. check for RXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 & temp2)
	{
		//handle RXE
		spi_rxe_it_handle(pSPIHandle);
	}

	//3. check for error
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 & temp2)
	{
		//handle OVR
		spi_ovr_err_it_handle(pSPIHandle);
	}
}

/*******************************************************************************
 * @fn			-	spi_txe_it_handle
 *
 * brief		-	Interrupt handling for transmission event
 *
 * @param[in]		-	SPI Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
static void spi_txe_it_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16 bit dff
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit dff
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen)
	{
		//Tx is over -- disable tx interrupt
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}
}

/*******************************************************************************
 * @fn			-	spi_rxe_it_handle
 *
 * brief		-	Interrupt handling for reception event
 *
 * @param[in]		-	SPI Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
static void spi_rxe_it_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16 bit dff
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}else
	{
		//8 bit dff
		*pSPIHandle->pTxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen)
	{
		//Rx is over -- disable rx interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}
}

/*******************************************************************************
 * @fn			-	spi_ovr_err_it_handle
 *
 * brief		-	Interrupt handling for over run error event
 *
 * @param[in]		-	SPI Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
static void spi_ovr_err_it_handle(SPI_Handle_t *pSPIHandle)
{
	//clear ovr flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*******************************************************************************
 * @fn			-	SPI_ClearOVRFlag
 *
 * brief		-	Clear over run flag
 *
 * @param[in]		-	Base address of SPI peripheral
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*******************************************************************************
 * @fn			-	SPI_CloseTransmission
 *
 * brief		-	Reset all transmission configuration
 *
 * @param[in]		-	SPI Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*******************************************************************************
 * @fn			-	SPI_CloseReception
 *
 * brief		-	Reset all reception configuration
 *
 * @param[in]		-	SPI Handle
 *
 * @return		-	none
 *
 * @note		-	none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/*******************************************************************************
 * @fn			-	SPI_ApplicationEventCallback
 *
 * brief		-	Event call back function for interrupt mode
 *
 * @param[in]		-	SPI Handle
 * @param[in]		-	SPI event
 *
 * @return		-	none
 *
 * @note		-	This is the weak implementation, must not be edited, should be
 * 					copied in application for required functionality
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEV)
{
	//This is the weak implementation.
}
