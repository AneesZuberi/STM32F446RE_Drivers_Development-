/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 25 Nov 2022
 *      Author: Anees
 */

#include "stm32f446xx.h"

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_



/*
 * SPI configuration structure
 */
typedef struct
{
	uint8_t SPI_DeviceMode;		/*!<Possible Values from @SPI_DEVICE_MODES			 		*/
	uint8_t SPI_BusConfig;		/*!<Possible Values from @SPI_BUS_CONFIGURATION		 		*/
	uint8_t SPI_SclkSpeed;		/*!<Possible Values from @SPI_CLOCK_SPEED			 		*/
	uint8_t SPI_DFF;			/*!<Possible Values from @SPI_DATA_FRAME_FORMAT		 		*/
	uint8_t SPI_CPOL;			/*!<Possible Values from @SPI_CLOCK_POLARITY		 		*/
	uint8_t SPI_CPHA;			/*!<Possible Values from @SPI_CLOCK_PHASE			 		*/
	uint8_t SPI_SSM;			/*!<Possible Values from @SPI_SOFTWARE_SLAVE_MANAGEMENT		*/
}SPI_PinConfig_t;

/*
 * SPI Handle structure
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;		/*!<This holds the base address o f the SPI peripheral     	 */
	SPI_PinConfig_t SPIConfig;	/*!<This holds SPI configuration settings			     	 */
	uint8_t *pTxBuffer;			/*!<This holds the address of Transmission Buffer	     	 */
	uint8_t *pRxBuffer;			/*!<This holds the address of Reception Buffer	     	 	 */
	uint32_t TxLen;				/*!<This holds the Length of Transmission Data		     	 */
	uint32_t RxLen;				/*!<This holds the Length of Reception Data			     	 */
	uint8_t TxState;			/*!<This holds the Transmission state @SPI_aplication_state  */
	uint8_t RxState;			/*!<This holds the Reception state @SPI_aplication_state	 */
}SPI_Handle_t;

/*
 * @SPI_aplication_state
 */
#define SPI_READY       0			/*!<SPI ready state  				*/
#define SPI_BUSY_IN_RX  1			/*!<SPI busy in reception  			*/
#define SPI_BUSY_IN_TX  2			/*!<SPI busy in transmission  		*/

/*
 * @SPI_Events
 */
#define SPI_EVENT_TX_CMPLT  	1	/*!<SPI Transmission Complete Event 				*/
#define SPI_EVENT_RX_CMPLT  	2	/*!<SPI Reception Complete Event	 				*/
#define SPI_EVENT_OVR_ERR   	3	/*!<SPI Over Run Error Event 						*/
#define SPI_EVENT_CRC_ERR   	4	/*!<SPI CRC Error Event 							*/
/*
 * @SPI_DEVICE_MODES
 */
#define SPI_DEVICE_MODE_SLAVE    0	/*!<Slave configuration 							*/
#define SPI_DEVICE_MODE_MASTER   1	/*!<Master configuration		 					*/

/*
 * @SPI_BUS_CONFIGURATION
 */
#define SPI_BUS_CONFIG_FD                  1	/*!<Full Duplex Bus Configuration					*/
#define SPI_BUS_CONFIG_HD                  2	/*!<Half Duplex Bus Configuration					*/
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY      3	/*!<Simplex Bus Configuration Transmission only		*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY      4	/*!<Simplex Bus Configuration Reception only		*/

/*
 * @SPI_CLOCK_SPEED
 */
#define SPI_SCLK_SPEED_DV2              0		/*!<Peripehral Clock DIV 2 							*/
#define SPI_SCLK_SPEED_DV4              1		/*!<Peripehral Clock DIV 4 							*/
#define SPI_SCLK_SPEED_DV8              2		/*!<Peripehral Clock DIV 8 							*/
#define SPI_SCLK_SPEED_DV16             3		/*!<Peripehral Clock DIV 16							*/
#define SPI_SCLK_SPEED_DV32             4		/*!<Peripehral Clock DIV 32							*/
#define SPI_SCLK_SPEED_DV64             5		/*!<Peripehral Clock DIV 64							*/
#define SPI_SCLK_SPEED_DV128            6		/*!<Peripehral Clock DIV 128						*/
#define SPI_SCLK_SPEED_DV256            7		/*!<Peripehral Clock DIV 256						*/

/*
 * @SPI_DATA_FRAME_FORMAT
 */
#define SPI_DFF_8BITS              0			/*!<8-bit data frame format							*/
#define SPI_DFF_16BITS             1			/*!<16-bit data frame format						*/

/*
 * @SPI_CLOCK_POLARITY
 */
#define SPI_CPOL_LOW              0				/*!<CK to 0 when idle								*/
#define SPI_CPOL_HIGH             1				/*!<CK to 1 when idle								*/

/*
 * @SPI_CLOCK_PHASE
 */
#define SPI_CPHA_LOW              0				/*!<First clock transition is first data capture edge	*/
#define SPI_CPHA_HIGH             1				/*!<Second clock transition is first data capture edge	*/

/*
 * @SPI_SOFTWARE_SLAVE_MANAGEMENT
 */
#define SPI_SSM_SW_DI              0			/*!<Software slave management disabled					*/
#define SPI_SSM_SW_EN              1			/*!<Software slave management enabled					*/

/*
 * Flag MACROS
 */
#define SPI_TXE_FLAG        (1<<SPI_SR_TXE)		/*!<Transmit buffer empty							*/
#define SPI_BSY_FLAG        (1<<SPI_SR_BSY)		/*!<Busy flag										*/
#define SPI_RXNE_FLAG       (1<<SPI_SR_RXNE)	/*!<Receive buffer not empty						*/

/*****************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 ****************************************************************************/
/*
 * Init & De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Peripheral Clock Setup
 */
void SPI_PeriCLKControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Data Transmission and Reception in Polling Mode
 */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len );

/*
 * Data Transmission and Reception in Interrupt Mode
 */
uint8_t SPI_SendIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len );

/*
 * SPI Peripheral Setup
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Software slave management setup
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Helper Functions
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t Get_FlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * SPI Interrupt Callback
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEV);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
