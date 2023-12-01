/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 22 Nov 2022
 *      Author: Anees
 */



#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * GPIO configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;				/*!<Possible Values from @GPIO_PIN_NUMBERS    	 				 */
	uint8_t GPIO_PinMode;				/*!<Possible Values from @GPIO_PIN_MODES     					 */
	uint8_t GPIO_PinSpeed;				/*!<Possible Values from @GPIO_PIN_SPEED     					 */
	uint8_t GPIO_PinPuPdControl;			/*!<Possible Values from @GPIO_PIN_PUSH_PULL_CONFIGURATION     	 		 */
	uint8_t GPIO_PinOPType;				/*!<Possible Values from @GPIO_PIN_OUTPUT_TYPE     	 			 */
	uint8_t GPIO_PinAltFunMode;			/*!<Possible Values from @GPIO_PIN_ALTERNATE_FUNCTIONALITY     	 		 */
}GPIO_PinConfig_t;

/*
 * GPIO Handle structure
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*!<This holds the base address o f the GPIO port to which the pin belongs     	 */
	GPIO_PinConfig_t GPIO_PinConfig;		/*!<This holds GPIO Pin configuration settings					 */
}GPIO_Handle_t;

/*
 * GPIO PIN MODES
 */
#define GPIO_MODE_IN                 0		/*!<Input    	 				     			 */
#define GPIO_MODE_OUT                1		/*!<General purpose output mode  					 */
#define GPIO_MODE_ALTFN              2		/*!<Alternate function mode    	 				 	 */
#define GPIO_MODE_ANALOG             3		/*!<Analog mode					 			 */
#define GPIO_MODE_IT_FT              4		/*!<Falling Trigger Interrupt mode    	 		 		 */
#define GPIO_MODE_IT_RT              5		/*!<Rising Trigger Interrupt mode     	 		 		 */
#define GPIO_MODE_IT_RFT             6		/*!<Rising and Falling Trigger Interrupt mode	 			 */

/*
 * GPIO_PIN_OUTPUT_TYPE
 */
#define GPIO_OT_PP                 0		/*!<Output push-pull 			     			 */
#define GPIO_OT_OD                 1		/*!<Output open-drain			     			 */

/*
 * GPIO_PIN_SPEED
 */
#define GPIO_SPEED_LOW             0		/*!<Low speed    				     			 */
#define GPIO_SPEED_MEDIUM          1		/*!<Medium speed 				     			 */
#define GPIO_SPEED_FAST            2		/*!<Fast speed 	 				     			 */
#define GPIO_SPEED_HIGH	  	   3		/*!<High speed 	 				     			 */

/*
 * GPIO_PIN_PUSH_PULL_CONFIGURATION
 */
#define GPIO_NO_PUPD              0			/*!<No pull-up, pull-down		     			 */
#define GPIO_PIN_PU               1			/*!<Pull-up    	 				     		 */
#define GPIO_PIN_PD               2			/*!<Pull-down  	 				     		 */

/*
 * GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0                   0	/*!<Pin Number 0 				     			 */
#define GPIO_PIN_NO_1                   1	/*!<Pin Number 1 				     			 */
#define GPIO_PIN_NO_2                   2	/*!<Pin Number 2 				     			 */
#define GPIO_PIN_NO_3                   3	/*!<Pin Number 3				     			 */
#define GPIO_PIN_NO_4                   4	/*!<Pin Number 4 				     			 */
#define GPIO_PIN_NO_5                   5	/*!<Pin Number 5 				     			 */
#define GPIO_PIN_NO_6                   6	/*!<Pin Number 6 				     			 */
#define GPIO_PIN_NO_7                   7	/*!<Pin Number 7 				     			 */
#define GPIO_PIN_NO_8                   8	/*!<Pin Number 8 				     			 */
#define GPIO_PIN_NO_9                   9	/*!<Pin Number 9 				     			 */
#define GPIO_PIN_NO_10                  10	/*!<Pin Number 10 				     			 */
#define GPIO_PIN_NO_11                  11	/*!<Pin Number 11 				     			 */
#define GPIO_PIN_NO_12                  12	/*!<Pin Number 12 				     			 */
#define GPIO_PIN_NO_13                  13	/*!<Pin Number 13 				     			 */
#define GPIO_PIN_NO_14                  14	/*!<Pin Number 14 				     			 */
#define GPIO_PIN_NO_15                  15	/*!<Pin Number 15				     			 */

/*****************************************************************************
 * 			APIs supported by this driver
 * 	For more information about the APIs check the function definitions
 ****************************************************************************/
/*
 * Peripheral Clock Setup
 */
void GPIO_PeriCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init & De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromIPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromIPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
//*****************************************************************************

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
