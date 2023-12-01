/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: 21 Dec 2022
 *      Author: Anees
 */
#include "stm32f446xx.h"

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

/*
 * USART configuration structure
 */
typedef struct
{
	uint8_t USART_Mode;			/*!<Possible Values from @USART_DEVICE_MODES			 		*/
	uint32_t USART_Baud;			/*!<Possible Values from @USART_BAUD_RATE				 	*/
	uint8_t USART_NoOfStopBits;		/*!<Possible Values from @USART_No_OF_STOP_BITS			 		*/
	uint8_t USART_WordLength;		/*!<Possible Values from @USART_WORD_LENGTH				 	*/
	uint8_t USART_ParityControl;		/*!<Possible Values from @USART_PARTIY_CONTROL			 		*/
	uint8_t USART_HWFlowControl;		/*!<Possible Values from @USART_HW_FLOW_CONTROL			 		*/
}USART_Config_t;

/*
 * USART Handle structure
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;			/*!<This holds the base address o f the USART peripheral     	*/
	USART_Config_t USART_Config;			/*!<This holds USART configuration settings			*/
	uint8_t *pTxBuffer;				/*!<This holds the address of Transmission Buffer	     	*/
	uint8_t *pRxBuffer;				/*!<This holds the address of Reception Buffer	     	 	*/
	uint32_t TxLen;					/*!<This holds the Length of Transmission Data		     	*/
	uint32_t RxLen;					/*!<This holds the Length of Reception Data			*/
	uint8_t TxState;				/*!<This holds the Transmission state @I2C_aplication_state  	*/
	uint8_t RxState;				/*!<This holds the Reception state @I2C_aplication_state	*/
}USART_Handle_t;


/*
 * @I2C_aplication_state
 */
#define USART_READY       0						/*!<I2C ready state  				*/
#define USART_BUSY_IN_RX  1						/*!<I2C busy in reception  			*/
#define USART_BUSY_IN_TX  2						/*!<I2C busy in transmission  			*/
/*
 * @USART_DEVICE_MODES
 */
#define USART_MODE_ONLY_TX     0				/*!<I2C transmit only  				*/
#define USART_MODE_ONLY_RX     1				/*!<I2C receive only  				*/
#define USART_MODE_TX_RX       2				/*!<I2C transmit and receive			*/

/*
 * @USART_BAUD_RATE
 */
#define USART_STD_BAUD_1200         1200		/*!<Standard baud rate 1200			*/
#define USART_STD_BAUD_2400         2400		/*!<Standard baud rate 2400			*/
#define USART_STD_BAUD_9600         9600		/*!<Standard baud rate 9600			*/
#define USART_STD_BAUD_19200        19200		/*!<Standard baud rate 19200			*/
#define USART_STD_BAUD_38400        38400		/*!<Standard baud rate 38400			*/
#define USART_STD_BAUD_57600        57600		/*!<Standard baud rate 57600			*/
#define USART_STD_BAUD_115200       115200		/*!<Standard baud rate 115200			*/
#define USART_STD_BAUD_230400       230400		/*!<Standard baud rate 230400			*/
#define USART_STD_BAUD_460800       460800		/*!<Standard baud rate 460800			*/
#define USART_STD_BAUD_921600       921600		/*!<Standard baud rate 921600			*/
#define USART_STD_BAUD_2M           2000000		/*!<Standard baud rate 2000000			*/
#define USART_STD_BAUD_3M           3000000		/*!<Standard baud rate 3000000			*/

/*
 * @USART_PARTIY_CONTROL
 */
#define USART_PARITY_EN_ODD     2			/*!<Parity ODD  				*/
#define USART_PARITY_EN_EVEN    1			/*!<Parity EVEN  				*/
#define USART_PARITY_DISABLE    0			/*!<Parity Control Disable  			*/

/*
 * @USART_WORD_LENGTH
 */
#define USART_WORDLEN_8BITS    0			/*!<1 Start bit, 8 Data bits, n Stop bit 	*/
#define USART_WORDLEN_9BITS    1			/*!<1 Start bit, 9 Data bits, n Stop bit	*/

/*
 * @USART_No_OF_STOP_BITS
 */
#define USART_STOPBITS_1    0				/*!<1 Stop bit  				*/
#define USART_STOPBITS_0_5  1				/*!<0.5 Stop bit  				*/
#define USART_STOPBITS_2    2				/*!<2 Stop bit  				*/
#define USART_STOPBITS_1_5  3				/*!<1.5 Stop bit  				*/
/*
 * @USART_HW_FLOW_CONTROL
 */
#define USART_HW_FLOW_CTRL_NONE       0		/*!<Hardware flow control disable			*/
#define USART_HW_FLOW_CTRL_CTS        1		/*!<CTS hardware flow control enable			*/
#define USART_HW_FLOW_CTRL_RTS        2		/*!<RTS hardware flow control enable  			*/
#define USART_HW_FLOW_CTRL_CTS_RTS    3		/*!<CTS and RTS hardware flow control enable		*/

/*
 * @USART_FLAGs
 */
#define USART_FLAG_RXNE        (1 << USART_SR_RXNE)	/*!<Receive buffer not empty			*/
#define USART_FLAG_TXE         (1 << USART_SR_TXE)	/*!<Transmit buffer empty			*/
#define USART_FLAG_TC          (1 << USART_SR_TC)	/*!<Transmission complete			*/

/*
 * @USART_Events
 */
#define USART_EVENT_TX_CMPLT    0			/*!<I2C Transmission Complete Event 		*/
#define USART_EVENT_RX_CMPLT    1			/*!<I2C Reception Complete Event	 	*/
#define USART_EVENT_CTS         2			/*!<I2C CTS Event	 			*/
#define USART_EVENT_IDLE        3			/*!<I2C IDLE line detected	 		*/
#define USART_EVENT_ORE         4			/*!<I2C Overrun error	 			*/
#define USART_ERREVENT_FE       5			/*!<I2C Framing error	 			*/
#define USART_ERREVENT_NF       6			/*!<I2C Noise detected flag	 		*/

/*****************************************************************************
 * 			APIs supported by this driver
 * 	For more information about the APIs check the function definitions
 ****************************************************************************/
/*
 * Init & De-Init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Peripheral Clock Setup
 */
void USART_PeriCLKControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * I2C Peripheral Setup
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Helper Functions
 */
uint8_t USART_Get_FlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName);
void USART_Clear_FlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName);

/*
 * Data Transmission and Reception in Polling Mode
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data Transmission and Reception in Interrupt Mode
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Baud rate configuration
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * I2C Interrupt Callback
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEV);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
