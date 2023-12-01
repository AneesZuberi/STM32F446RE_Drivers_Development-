/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 13 Dec 2022
 *      Author: Anees
 */

#include "stm32f446xx.h"

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

/*
 * I2C configuration structure
 */
typedef struct
{
	uint32_t I2C_SCLSPEED;			/*!<Possible Values from @I2C_CLOCK_SPEED			 	*/
	uint8_t I2C_DeviceAddress;		/*!<Set value in the application					*/
	uint8_t I2C_ACKControl;			/*!<Possible Values from @I2C_ASKNOWLEDGE_ENABLE	 		*/
	uint8_t I2C_FMDutyCycle;		/*!<Possible Values from @I2C_FM_MODE_DUTY_CYCLE	 		*/
}I2C_PinConfig_t;

/*
 * I2C Handle structure
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;				/*!<This holds the base address o f the I2C peripheral     	 			*/
	I2C_PinConfig_t I2CConfig;			/*!<This holds I2C configuration settings			     	 		*/
	uint8_t *pTxBuffer;				/*!<This holds the address of Transmission Buffer	     	 			*/
	uint8_t *pRxBuffer;				/*!<This holds the address of Reception Buffer	     	 	 			*/
	uint32_t TxLen;					/*!<This holds the Length of Transmission Data		     	 			*/
	uint32_t RxLen;					/*!<This holds the Length of Reception Data			     	 		*/
	uint32_t RxSize;
	uint8_t TxRxState;				/*!<This holds the Transmission/Reception state @I2C_aplication_state   		*/
	uint8_t DevAddr;				/*!<This holds the address for device						 	*/
	uint8_t Sr;					/*!<This holds the Reception state @I2C_REPEATED_START		 			*/
}I2C_Handle_t;

/*
 * @I2C_CLOCK_SPEED
 */
#define I2C_SCLSPEED_SM       100000		/*!<Standard mode	 		*/
#define I2C_SCLSPEED_FM       400000		/*!<Fast mode			 	*/

/*
 * @I2C_ASKNOWLEDGE_ENABLE
 */
#define I2C_ACK_DI  	0				/*!<No acknowledge returned			 			*/
#define I2C_ACK_EN  	1				/*!<Acknowledge returned after a byte is received			*/

/*
 * @I2C_FM_MODE_DUTY_CYCLE
 */
#define I2C_FM_DUTY_2      0			/*!<FM mode t[low]/t[high] = 2			 					*/
#define I2C_FM_DUTY_16_9   1			/*!<FM mode t[low]/t[high] = 16/9			 				*/

/*
 * @I2C_REPEATED_START
 */
#define I2C_DISABLE_SR      0			/*!<Repeated start disable macro					 		*/
#define I2C_ENABLE_SR       1			/*!<Repeated start enable macro						 		*/

/*
 * @I2C_aplication_state
 */
#define I2C_READY          0			/*!<I2C ready state			 						*/
#define I2C_BUSY_IN_TX     1			/*!<I2C busy in transmission			 					*/
#define I2C_BUSY_IN_RX     2			/*!<I2C busy in reception			 					*/

/*
 * Flag MACROS
 */
#define I2C_FLAG_SB        (1<<I2C_SR1_SB)		/*!<Start bit (Master mode)	 				*/
#define I2C_FLAG_ADDR      (1<<I2C_SR1_ADDR)		/*!<Address sent (master mode)/matched (slave mode)		*/
#define I2C_FLAG_BTF       (1<<I2C_SR1_BTF)		/*!<Byte transfer finished	 				*/
#define I2C_FLAG_TXE       (1<<I2C_SR1_TXE)		/*!<Data register empty (transmitters) 				*/
#define I2C_FLAG_RXNE      (1<<I2C_SR1_RXNE)		/*!<Data register not empty (receivers)				*/

/*
 * @I2C_Events
 */
#define I2C_EV_RX_CMPLT    0	/*!<Reception Complete Event	 				*/
#define I2C_EV_TX_CMPLT    1	/*!<Transmission Complete Event 				*/
#define I2C_EV_STOP        2	/*!<Stop detection (slave mode) Event				*/
#define I2C_ERROR_BERR     3	/*!<Bus Error Event		 				*/
#define I2C_ERROR_ARLO     4	/*!<Arbitration lost (master mode) Event			*/
#define I2C_ERROR_AF       5	/*!<Acknowledge Failure Error Event				*/
#define I2C_ERROR_OVR      6	/*!<OverRun/UnderRun Error Event				*/
#define I2C_ERROR_TIMEOUT  7	/*!<Timeout or T[low] error Event				*/
#define I2C_EV_DATA_REQ    8	/*!<Data request Event						*/
#define I2C_EV_DATA_RCV    9	/*!<Data reception Event					*/

/*****************************************************************************
 * 			APIs supported by this driver
 * 	For more information about the APIs check the function definitions
 ****************************************************************************/
/*
 * Init & De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Peripheral Clock Setup
 */
void I2C_PeriCLKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C Peripheral Setup
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C Peripheral Setup
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Data Transmission and Reception in Polling Mode (Master)
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data Transmission and Reception in Polling Mode (Slave)
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * Data Transmission and Reception in Interrupt Mode (Master)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Helper Functions
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseRcvData(I2C_Handle_t *pI2CHandle);
uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flagName);

/*
 * IRQ Configuration and ISR Handling (Event and Error)
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Slave enable and disable callback function
 */
void I2C_SlaveEnorDiCallBackEvent(I2C_RegDef_t *pI2C, uint8_t EnorDi);

/*
 * SPI Interrupt Callback
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEV);



#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
