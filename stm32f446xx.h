/*
 * stm32f446xx.h
 *
 *  Created on: Nov 21, 2022
 *      Author: Anees
 */
#include "stdint.h"
#include "string.h"
#include "stddef.h"

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#define __vo   volatile
#define __weak __attribute__((weak))
/***************************Processor Specific Details*******************
 *
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0           ((__vo uint32_t *)0xE000E100)	/*!<Base Address of Interrupt Set-Enable Register 0      */
#define NVIC_ISER1           ((__vo uint32_t *)0xE000E104)	/*!<Base Address of Interrupt Set-Enable Register 1      */
#define NVIC_ISER2           ((__vo uint32_t *)0xE000E108)	/*!<Base Address of Interrupt Set-Enable Register 2      */
#define NVIC_ISER3           ((__vo uint32_t *)0xE000E10C)	/*!<Base Address of Interrupt Set-Enable Register 3      */
/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0           ((__vo uint32_t *)0xE000E180)	/*!<Base Address of Interrupt Clear-Enable Register 0    */
#define NVIC_ICER1           ((__vo uint32_t *)0xE000E184)	/*!<Base Address of Interrupt Clear-Enable Register 1    */
#define NVIC_ICER2           ((__vo uint32_t *)0xE000E188)	/*!<Base Address of Interrupt Clear-Enable Register 2    */
#define NVIC_ICER3           ((__vo uint32_t *)0xE000E18C)	/*!<Base Address of Interrupt Clear-Enable Register 3    */
/*
 * ARM Cortex Mx Processor NVIC priority register Addresses
 */
#define NVIC_PR_BASEADDR    ((__vo uint32_t *)0xE000E400)	/*!<Base Address of NVIC Peripheral      */

#define NO_BITS_IMP          4		/*!<Number of Bit to implement in Interrupt Priority register      */

/*****************************************************************************************************************/


/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR        0x08000000U		/*!<Base Address of Flash Memory      */
#define SRAM1_BASEADDR        0x20000000U  		/*!<Base Address of SRAM1 Memory      */
#define SRAM2_BASEADDR        0x2001C000U  		/*!<Base Address of SRAM2 Memory      */
#define ROM_BASEADDR          0x1FFF0000U		/*!<Base Address of ROM Memory        */
#define SRAM                  SRAM1_BASEADDR		/*!<Base Address of SRAM Memory       */

/*
 * base addresses of Various Bus domains
 */
#define PERIPH_BASEADDR      0x40000000U		/*!<Base Address of Peripheral Bus Domain      */
#define AHB1_BASEADDR        0x40020000U		/*!<Base Address of AHB1 Bus Domain            */
#define AHB2_BASEADDR        0x50000000U		/*!<Base Address of AHB2 Bus Domain    		   */
#define AHB3_BASEADDR        0xA0001000U		/*!<Base Address of AHB3 Bus Domain    		   */
#define APB1_BASEADDR        PERIPH_BASEADDR	/*!<Base Address of APB1 Bus Domain   		   */
#define APB2_BASEADDR        0x40010000U		/*!<Base Address of aPB2 Bus Domain   		   */

/*
 * base addresses of Peripherals on AHB1 Bus
 */
#define GPIOA_BASEADDR      (AHB1_BASEADDR + 0x0000)	/*!<Base Address of GPIOA Peripheral      */
#define GPIOB_BASEADDR      (AHB1_BASEADDR + 0x0400)	/*!<Base Address of GPIOB Peripheral      */
#define GPIOC_BASEADDR      (AHB1_BASEADDR + 0x0800)	/*!<Base Address of GPIOC Peripheral      */
#define GPIOD_BASEADDR      (AHB1_BASEADDR + 0x0C00)	/*!<Base Address of GPIOD Peripheral      */
#define GPIOE_BASEADDR      (AHB1_BASEADDR + 0x1000)	/*!<Base Address of GPIOE Peripheral      */
#define GPIOF_BASEADDR      (AHB1_BASEADDR + 0x1400)	/*!<Base Address of GPIOF Peripheral      */
#define GPIOG_BASEADDR      (AHB1_BASEADDR + 0x1800)	/*!<Base Address of GPIOG Peripheral      */
#define GPIOH_BASEADDR      (AHB1_BASEADDR + 0x1C00)	/*!<Base Address of GPIOH Peripheral      */

#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800)	/*!<Base Address of RCC Peripheral        */

/*
 * base addresses of Peripherals on APB1 Bus
 */
#define I2C1_BASEADDR        (APB1_BASEADDR + 0x5400)	/*!<Base Address of I2C1 Peripheral      */
#define I2C2_BASEADDR        (APB1_BASEADDR + 0x5800)	/*!<Base Address of I2C2 Peripheral      */
#define I2C3_BASEADDR        (APB1_BASEADDR + 0x5C00)	/*!<Base Address of I2C3 Peripheral      */

#define SPI2_BASEADDR        (APB1_BASEADDR + 0x3800)	/*!<Base Address of SPI2 Peripheral      */
#define SPI3_BASEADDR        (APB1_BASEADDR + 0x3C00)	/*!<Base Address of SPI3 Peripheral      */

#define USART2_BASEADDR      (APB1_BASEADDR + 0x4400)	/*!<Base Address of USART2 Peripheral    */
#define USART3_BASEADDR      (APB1_BASEADDR + 0x4800)	/*!<Base Address of USART3 Peripheral    */

#define UART4_BASEADDR       (APB1_BASEADDR + 0x4C00)	/*!<Base Address of UART4 Peripheral     */
#define UART5_BASEADDR       (APB1_BASEADDR + 0x5000)	/*!<Base Address of UART5 Peripheral     */

/*
 * base addresses of Peripherals on APB2 Bus
 */
#define SPI1_BASEADDR        (APB2_BASEADDR + 0x3000)	/*!<Base Address of SPI1 Peripheral      */

#define USART1_BASEADDR      (APB2_BASEADDR + 0x1000)	/*!<Base Address of USART1 Peripheral    */
#define USART6_BASEADDR      (APB2_BASEADDR + 0x1400)	/*!<Base Address of USART6 Peripheral    */

#define EXTI_BASEADDR        (APB2_BASEADDR + 0x3C00)	/*!<Base Address of External Interrupt Peripheral       */
#define SYSCFG_BASEADDR      (APB2_BASEADDR + 0x3800)	/*!<Base Address of System Configuration Peripheral     */

/*
 * ************************GPIO Peripheral Definition Structures******************
 */
typedef struct
{
	__vo uint32_t MODER;      /*!<GPIO Moder register 			*/
	__vo uint32_t OTYPER;     /*!<GPIO output type register			*/
	__vo uint32_t OSPEEDER;   /*!<GPIO output speed register 		*/
	__vo uint32_t PUPDR;	  /*!<GPIO pull-up pull-down register 		*/
	__vo uint32_t IDR;	  /*!<GPIO input data register 			*/
	__vo uint32_t ODR;	  /*!<GPIO output data register 		*/
	__vo uint32_t BSRR;	  /*!<GPIO bit set/ reset register 		*/
	__vo uint32_t LCKR;	  /*!<GPIO configuration lock register 		*/
	__vo uint32_t AFRL[2];    /*!<AFRL[0] alternate function low register -- AFRL[1] alternate function high register*/
}GPIO_RegDef_t;

/*
 * ************************SPI Peripheral Definition Structures******************
 */
typedef struct
{
	__vo uint32_t CR1;      	 /*!<SPI control register 1 				*/
	__vo uint32_t CR2;     	     	 /*!<SPI control register 2 				*/
	__vo uint32_t SR;    		 /*!<SPI status register    				*/
	__vo uint32_t DR;	    	 /*!<SPI data register      				*/
	__vo uint32_t CRCPR;		 /*!<SPI CRC polynomial register			*/
	__vo uint32_t RXCRCR;		 /*!<SPI RX CRC register   	    			*/
	__vo uint32_t TXCRCR; 		 /*!<SPI TX CRC register    				*/
	__vo uint32_t I2SCFGR; 		 /*!<SPI_I2S configuration register 			*/
	__vo uint32_t I2SPR; 		 /*!<SPI_I2S prescaler register				*/
}SPI_RegDef_t;

/*
 * ************************I2C Peripheral Definition Structures******************
 */
typedef struct
{
	__vo uint32_t CR1;		/*!<I2C control register 1 				*/
	__vo uint32_t CR2;		/*!<I2C control register 2 				*/
	__vo uint32_t OAR1;		/*!<I2C own address register 1				*/
	__vo uint32_t OAR2;		/*!<I2C own address register 2				*/
	__vo uint32_t DR;		/*!<I2C data register	 				*/
	__vo uint32_t SR1;		/*!<I2C status register 1 				*/
	__vo uint32_t SR2;		/*!<I2C status register 2 				*/
	__vo uint32_t CCR;		/*!<I2C clock control register				*/
	__vo uint32_t TRISE;		/*!<I2C TRISE register	 				*/
	__vo uint32_t FLTR;		/*!<2C FLTR register	 				*/
}I2C_RegDef_t;

/*
 * ************************RCC Peripheral Definition Structures******************
 */
typedef struct
{
	__vo uint32_t CR;              /*!<RCC clock control register 							*/
	__vo uint32_t PLLCFGR;         /*!<RCC PLL configuration register						*/
	__vo uint32_t CFGR;            /*!<RCC clock configuration register						*/
	__vo uint32_t CIR;	       /*!<RCC clock interrupt register							*/
	__vo uint32_t AHB1RSTR;	       /*!<RCC AHB1 peripheral reset register						*/
	__vo uint32_t AHB2RSTR;	       /*!<RCC AHB2 peripheral reset register						*/
	__vo uint32_t AHB3RSTR;        /*!<RCC AHB3 peripheral reset register						*/
	uint32_t RESERVED1;            /*!<Register Reserved by MCU 							*/
	__vo uint32_t APB1RSTR;        /*!<RCC APB1 peripheral reset register						*/
	__vo uint32_t APB2RSTR;        /*!<RCC APB2 peripheral reset register						*/
	uint32_t RESERVED2;            /*!<Register Reserved by MCU							*/
	uint32_t RESERVED3;            /*!<Register Reserved by MCU							*/
	__vo uint32_t AHB1ENR;         /*!<RCC AHB1 peripheral clock enable register					*/
	__vo uint32_t AHB2ENR;         /*!<RCC AHB2 peripheral clock enable register					*/
	__vo uint32_t AHB3ENR;	       /*!<RCC AHB3 peripheral clock enable register					*/
	uint32_t RESERVED4;	       /*!<Register Reserved by MCU							*/
	__vo uint32_t APB1ENR;	       /*!<RCC APB1 peripheral clock enable register					*/
	__vo uint32_t APB2ENR;         /*!<RCC AP21 peripheral clock enable register					*/
	uint32_t RESERVED5;	       /*!<Register Reserved by MCU							*/
	uint32_t RESERVED6;	       /*!<Register Reserved by MCU							*/
	__vo uint32_t AHB1LPENR;       /*!<RCC AHB1 peripheral clock enable in low power mode register			*/
	__vo uint32_t AHB2LPENR;       /*!<RCC AHB2 peripheral clock enable in low power mode register			*/
	__vo uint32_t AHB3LPENR;       /*!<RCC AHB3 peripheral clock enable in low power mode register			*/
	uint32_t RESERVED7;	       /*!<Register Reserved by MCU							*/
	__vo uint32_t APB1LPENR;       /*!<RCC APB1 peripheral clock enable in low power mode register			*/
	__vo uint32_t APB2LPENR;       /*!<RCC APB2 peripheral clock enable in low power mode register			*/
	uint32_t RESERVED8;	       /*!<Register Reserved by MCU							*/
	uint32_t RESERVED9;	       /*!<Register Reserved by MCU							*/
	__vo uint32_t BDCR;  	       /*!<RCC Backup domain control register						*/
	__vo uint32_t CSR;	       /*!<RCC clock control & status register						*/
	uint32_t RESERVED10;	       /*!<Register Reserved by MCU							*/
	uint32_t RESERVED11;  	       /*!<Register Reserved by MCU							*/
	__vo uint32_t SSCGR;	       /*!<RCC spread spectrum clock generation register				*/
	__vo uint32_t PLLI2SCFGR;      /*!<RCC PLLI2S configuration register						*/
	__vo uint32_t PLLSAICFGR;      /*!<RCC PLL configuration register						*/
	__vo uint32_t DCKCFGR;	       /*!<RCC dedicated clock configuration register					*/
	__vo uint32_t CKGATENR;	       /*!<RCC clocks gated enable register						*/
	__vo uint32_t DCKCFGR2;	       /*!<RCC dedicated clocks configuration register 2				*/
}RCC_RegDef_t;

/*
 * ************************EXTI Definition Structures******************
 */
typedef struct
{
	__vo uint32_t IMR;              /*!<Interrupt mask register 			*/
	__vo uint32_t EMR;              /*!<Event mask register		 		*/
	__vo uint32_t RTSR;             /*!<Rising trigger selection register		*/
	__vo uint32_t FTSR;             /*!<Falling trigger selection register		*/
	__vo uint32_t SWIER;            /*!<Software interrupt event register		*/
	__vo uint32_t PR;               /*!<Pending register				*/
}EXTI_RegDef_t;

/*
 * ************************SYSCFG Definition Structures******************
 */
typedef struct
{
	__vo uint32_t MEMRMP;               /*!<SYSCFG memory remap register 				*/
	__vo uint32_t PMC;             	    /*!<SYSCFG peripheral mode configuration register		*/
	__vo uint32_t EXTICR[4];            /*!<SYSCFG external interrupt configuration register 1-4	*/
	uint32_t RESERVED[2];               /*!<Register Reserved by MCU				*/
	__vo uint32_t CMPCR;                /*!<Compensation cell control register 			*/
	uint32_t RESERVED1[2];              /*!<Register Reserved by MCU				*/
	__vo uint32_t CFGR;                 /*!<SYSCFG configuration register			 	*/
}SYSCFG_RegDef_t;

/*
 * ************************USART Definition Structures******************
 */
typedef struct
{
	__vo uint32_t SR;              /*!<Status register						*/
	__vo uint32_t DR;              /*!<Data register						*/
	__vo uint32_t BRR;             /*!<Baud rate register						*/
	__vo uint32_t CR1;             /*!<Control register 1						*/
	__vo uint32_t CR2;             /*!<Control register 2						*/
	__vo uint32_t CR3;             /*!<Control register 3						*/
	__vo uint32_t GTPR;            /*!<Guard time and prescaler register				*/
}USART_RegDef_t;

/*
 * Peripheral Definitions for GPIOx
 */
#define GPIOA       ((GPIO_RegDef_t *)GPIOA_BASEADDR)	/*!<GPIOA Peripheral Definitions		*/
#define GPIOB       ((GPIO_RegDef_t *)GPIOB_BASEADDR)	/*!<GPIOB Peripheral Definitions		*/
#define GPIOC       ((GPIO_RegDef_t *)GPIOC_BASEADDR)	/*!<GPIOC Peripheral Definitions		*/
#define GPIOD       ((GPIO_RegDef_t *)GPIOD_BASEADDR)	/*!<GPIOD Peripheral Definitions		*/
#define GPIOE       ((GPIO_RegDef_t *)GPIOE_BASEADDR)	/*!<GPIOE Peripheral Definitions		*/
#define GPIOF       ((GPIO_RegDef_t *)GPIOF_BASEADDR)	/*!<GPIOF Peripheral Definitions		*/
#define GPIOG       ((GPIO_RegDef_t *)GPIOG_BASEADDR)	/*!<GPIOG Peripheral Definitions		*/
#define GPIOH       ((GPIO_RegDef_t *)GPIOH_BASEADDR)	/*!<GPIOH Peripheral Definitions		*/

/*
 * Peripheral Definitions for RCC
 */
#define RCC         ((RCC_RegDef_t *)RCC_BASEADDR)		/*!<RCC Peripheral Definitions			*/

/*
 * Peripheral Definitions for SPIx
 */
#define SPI1        ((SPI_RegDef_t *)SPI1_BASEADDR)		/*!<SPI1 Peripheral Definitions			*/
#define SPI2        ((SPI_RegDef_t *)SPI2_BASEADDR)		/*!<SPI2 Peripheral Definitions			*/
#define SPI3        ((SPI_RegDef_t *)SPI3_BASEADDR)		/*!<SPI3 Peripheral Definitions			*/

/*
 * Peripheral Definitions for I2Cx
 */
#define I2C1        ((I2C_RegDef_t *)I2C1_BASEADDR)		/*!<I2C1 Peripheral Definitions			*/
#define I2C2        ((I2C_RegDef_t *)I2C2_BASEADDR)		/*!<I2C2 Peripheral Definitions			*/
#define I2C3        ((I2C_RegDef_t *)I2C3_BASEADDR)		/*!<I2C3 Peripheral Definitions			*/

/*
 * Peripheral Definitions for USARTx
 */
#define USART1      ((USART_RegDef_t *)USART1_BASEADDR) /*!<USART1 Peripheral Definitions		*/
#define USART2      ((USART_RegDef_t *)USART2_BASEADDR) /*!<USART2 Peripheral Definitions		*/
#define USART3      ((USART_RegDef_t *)USART3_BASEADDR) /*!<USART3 Peripheral Definitions		*/
#define UART4       ((USART_RegDef_t *)UART4_BASEADDR)  /*!<UART4 Peripheral Definitions		*/
#define UART5       ((USART_RegDef_t *)UART5_BASEADDR)  /*!<UART5 Peripheral Definitions		*/
#define USART6      ((USART_RegDef_t *)USART6_BASEADDR) /*!<USART5 Peripheral Definitions		*/

/*
 * Peripheral Definitions for EXTI
 */
#define EXTI        ((EXTI_RegDef_t *)EXTI_BASEADDR)	/*!<EXTI Peripheral Definitions			*/

/*
 * Peripheral Definitions for SYSCFG
 */
#define SYSCFG      ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)/*!<SYSCFG Peripheral Definitions		*/


/*
 * Clock enable Macros for GPIOx
 */
#define GPIOA_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 0))	/*!<GPIOA Peripheral Clock Enable		*/
#define GPIOB_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 1))	/*!<GPIOB Peripheral Clock Enable		*/
#define GPIOC_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 2))	/*!<GPIOC Peripheral Clock Enable		*/
#define GPIOD_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 3))	/*!<GPIOD Peripheral Clock Enable		*/
#define GPIOE_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 4))	/*!<GPIOE Peripheral Clock Enable		*/
#define GPIOF_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 5))	/*!<GPIOF Peripheral Clock Enable		*/
#define GPIOG_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 6))	/*!<GPIOG Peripheral Clock Enable		*/
#define GPIOH_PR_CLK_EN()    (RCC->AHB1ENR |= (1 << 7))	/*!<GPIOH Peripheral Clock Enable		*/

/*
 * Clock enable Macros for SYSCFG
 */
#define SYSCFG_PCLK_EN()     (RCC->APB2ENR |= (1 << 14))/*!<SYSCFG Peripheral Clock Enable		*/

/*
 * Clock enable Macros for SPIx
 */
#define SPI1_PR_CLK_EN()     (RCC->APB2ENR |= (1 << 12))/*!<SPI1 Peripheral Clock Enable		*/
#define SPI2_PR_CLK_EN()     (RCC->APB1ENR |= (1 << 14))/*!<SPI2 Peripheral Clock Enable		*/
#define SPI3_PR_CLK_EN()     (RCC->APB1ENR |= (1 << 15))/*!<SPI3 Peripheral Clock Enable		*/

/*
 * Clock enable Macros for I2Cx
 */
#define I2C1_PR_CLK_EN()     (RCC->APB1ENR |= (1 << 21))/*!<I2C1 Peripheral Clock Enable		*/
#define I2C2_PR_CLK_EN()     (RCC->APB1ENR |= (1 << 22))/*!<I2C2 Peripheral Clock Enable		*/
#define I2C3_PR_CLK_EN()     (RCC->APB1ENR |= (1 << 23))/*!<I2C3 Peripheral Clock Enable		*/

/*
 * Clock enable Macros for USARTx
 */
#define USART1_PR_CLK_EN()   (RCC->APB2ENR |= (1 << 4)) /*!<USART1 Peripheral Clock Enable		*/
#define USART2_PR_CLK_EN()   (RCC->APB1ENR |= (1 << 17))/*!<USART2 Peripheral Clock Enable		*/
#define USART3_PR_CLK_EN()   (RCC->APB1ENR |= (1 << 18))/*!<USART3 Peripheral Clock Enable		*/
#define UART4_PR_CLK_EN()    (RCC->APB1ENR |= (1 << 19))/*!<UART4 Peripheral Clock Enable		*/
#define UART5_PR_CLK_EN()    (RCC->APB1ENR |= (1 << 20))/*!<UART5 Peripheral Clock Enable		*/
#define USART6_PR_CLK_EN()   (RCC->APB2ENR |= (1 << 5))	/*!<USART5 Peripheral Clock Enable		*/

/*
 * GPIOx peripheral RESET Macros
 */
#define GPIOA_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0));}while(0)  /*!<GPIOA Peripheral Reset	*/
#define GPIOB_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1));}while(0)  /*!<GPIOB Peripheral Reset	*/
#define GPIOC_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2));}while(0)  /*!<GPIOC Peripheral Reset	*/
#define GPIOD_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3));}while(0)  /*!<GPIOD Peripheral Reset	*/
#define GPIOE_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4));}while(0)  /*!<GPIOE Peripheral Reset	*/
#define GPIOF_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5));}while(0)  /*!<GPIOF Peripheral Reset	*/
#define GPIOG_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6));}while(0)  /*!<GPIOG Peripheral Reset	*/
#define GPIOH_REG_RST()    do{(RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7));}while(0)  /*!<GPIOH Peripheral Reset	*/

/*
 * SPIx peripheral RESET Macros
 */
#define SPI1_REG_RST()    do{(RCC->APB2RSTR |= (1 << 12));  (RCC->APB2RSTR &= ~(1 << 12));}while(0) /*!<SPI1 Peripheral Reset	*/
#define SPI2_REG_RST()    do{(RCC->APB1RSTR |= (1 << 14));  (RCC->APB2RSTR &= ~(1 << 14));}while(0) /*!<SPI2 Peripheral Reset	*/
#define SPI3_REG_RST()    do{(RCC->APB1RSTR |= (1 << 15));  (RCC->APB2RSTR &= ~(1 << 15));}while(0) /*!<SPI3 Peripheral Reset	*/

/*
 * I2Cx peripheral RESET Macros
 */
#define I2C1_REG_RST()    do{(RCC->APB1RSTR |= (1 << 21));  (RCC->APB1RSTR &= ~(1 << 21));}while(0) /*!<I2C1 Peripheral Reset	*/
#define I2C2_REG_RST()    do{(RCC->APB1RSTR |= (1 << 22));  (RCC->APB1RSTR &= ~(1 << 22));}while(0) /*!<I2C2 Peripheral Reset	*/
#define I2C3_REG_RST()    do{(RCC->APB1RSTR |= (1 << 23));  (RCC->APB1RSTR &= ~(1 << 23));}while(0) /*!<I2C3 Peripheral Reset	*/

/*
 * USARTx peripheral RESET Macros
 */
#define USART1_REG_RST()    do{(RCC->APB2RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4));}while(0)   /*!<USART1 Peripheral Reset	*/
#define USART2_REG_RST()    do{(RCC->APB1RSTR |= (1 << 17));  (RCC->APB1RSTR &= ~(1 << 17));}while(0) /*!<USART2 Peripheral Reset	*/
#define USART3_REG_RST()    do{(RCC->APB1RSTR |= (1 << 18));  (RCC->APB1RSTR &= ~(1 << 18));}while(0) /*!<USAR33 Peripheral Reset	*/
#define UART4_REG_RST()     do{(RCC->APB1RSTR |= (1 << 19));  (RCC->APB1RSTR &= ~(1 << 19));}while(0) /*!<UART4 Peripheral Reset	*/
#define UART5_REG_RST()     do{(RCC->APB1RSTR |= (1 << 20));  (RCC->APB1RSTR &= ~(1 << 20));}while(0) /*!<UART5 Peripheral Reset	*/
#define USART6_REG_RST()    do{(RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5));}while(0)   /*!<USART6 Peripheral Reset	*/

/*
 * Clock disable Macros for GPIOx
 */
#define GPIOA_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 0))	/*!<GPIOA Peripheral Clock Disable		*/
#define GPIOB_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 1))	/*!<GPIOB Peripheral Clock Disable		*/
#define GPIOC_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 2))	/*!<GPIOC Peripheral Clock Disable		*/
#define GPIOD_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 3))	/*!<GPIOD Peripheral Clock Disable		*/
#define GPIOE_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 4))	/*!<GPIOE Peripheral Clock Disable		*/
#define GPIOF_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 5))	/*!<GPIOF Peripheral Clock Disable		*/
#define GPIOG_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 6))	/*!<GPIOG Peripheral Clock Disable		*/
#define GPIOH_PR_CLK_DI()    (RCC->AHB1ENR &= ~(1 << 7))	/*!<GPIOH Peripheral Clock Disable		*/

/*
 * Clock disable Macros for SYSCFG
 */
#define SYSCFG_PCLK_EN()     (RCC->APB2ENR &= ~(1 << 14))/*!<SYSCFG Peripheral Clock Disable		*/

/*
 * Clock disable Macros for SPIx
 */
#define SPI1_PR_CLK_DI()     (RCC->APB2ENR &= ~(1 << 12))	/*!<SPI1 Peripheral Clock Disable		*/
#define SPI2_PR_CLK_DI()     (RCC->APB1ENR &= ~(1 << 14))	/*!<SPI2 Peripheral Clock Disable		*/
#define SPI3_PR_CLK_DI()     (RCC->APB1ENR &= ~(1 << 15))	/*!<SPI3 Peripheral Clock Disable		*/

/*
 * Clock disable Macros for I2Cx
 */
#define I2C1_PR_CLK_DI()     (RCC->APB1ENR &= ~(1 << 21))	/*!<I2C1 Peripheral Clock Disable		*/
#define I2C2_PR_CLK_DI()     (RCC->APB1ENR &= ~(1 << 22))	/*!<I2C2 Peripheral Clock Disable		*/
#define I2C3_PR_CLK_DI()     (RCC->APB1ENR &= ~(1 << 23))	/*!<I2C3 Peripheral Clock Disable		*/

/*
 * Clock disable Macros for USARTx
 */
#define USART1_PR_CLK_DI()   (RCC->APB2ENR &= ~(1 << 4))	/*!<USART1 Peripheral Clock Disable		*/
#define USART2_PR_CLK_DI()   (RCC->APB1ENR &= ~(1 << 17))	/*!<USART2 Peripheral Clock Disable		*/
#define USART3_PR_CLK_DI()   (RCC->APB1ENR &= ~(1 << 18))	/*!<USART3 Peripheral Clock Disable		*/
#define UART4_PR_CLK_DI()    (RCC->APB1ENR &= ~(1 << 19))	/*!<UART4 Peripheral Clock Disable		*/
#define UART5_PR_CLK_DI()    (RCC->APB1ENR &= ~(1 << 20))	/*!<UART5 Peripheral Clock Disable		*/
#define USART6_PR_CLK_DI()   (RCC->APB2ENR &= ~(1 << 5))	/*!<USART6 Peripheral Clock Disable		*/

/*
 * MACRO for GPIO to code conversion
 */
#define GPIO_BASEADDR_TO_CODE(x)    ((x == GPIOA)?0:\
				     (x == GPIOB)?1:\
				     (x == GPIOC)?2:\
				     (x == GPIOD)?3:\
		                     (x == GPIOE)?4:\
			             (x == GPIOF)?5:\
				     (x == GPIOG)?6:\
				     (x == GPIOH)?7:0)		/*!<GPIO Peripheral Coding 0 to 7		*/

/*
 * IRQ No on EXTI line
 * depends on every MCU
 */
#define IRQ_NO_EXTI0          6   /*!<IRQ Number for External Interrupt 0 				*/
#define IRQ_NO_EXTI1          7	  /*!<IRQ Number for External Interrupt 1 				*/
#define IRQ_NO_EXTI2	      8   /*!<IRQ Number for External Interrupt 2 				*/
#define IRQ_NO_EXTI3          9   /*!<IRQ Number for External Interrupt 3 				*/
#define IRQ_NO_EXTI4          10  /*!<IRQ Number for External Interrupt 4 				*/
#define IRQ_NO_EXTI9_5        23  /*!<IRQ Number for External Interrupt 5 to 9 				*/
#define IRQ_NO_EXTI15_10      40  /*!<IRQ Number for External Interrupt 10 to 15 			*/

#define IRQ_NO_SPI1          35	  /*!<IRQ Number for SPI1						*/
#define IRQ_NO_SPI2          36   /*!<IRQ Number for SPI2				 		*/
#define IRQ_NO_SPI3          51   /*!<IRQ Number for SPI3				 		*/

#define IRQ_NO_I2C1_EV      31    /*!<IRQ Number for I2C 1 Event						*/
#define IRQ_NO_I2C1_ER      32    /*!<IRQ Number for I2C 1 Error						*/
#define IRQ_NO_I2C2_EV      33	  /*!<IRQ Number for I2C 2 Event						*/
#define IRQ_NO_I2C2_ER      34	  /*!<IRQ Number for I2C 2 Error						*/
#define IRQ_NO_I2C3_EV      72    /*!<IRQ Number for I2C 3 Event						*/
#define IRQ_NO_I2C3_ER      73    /*!<IRQ Number for I2C 3 Error						*/

/*
 * SPI register: bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA        0		/*!<Clock phase							*/
#define SPI_CR1_CPOL        1		/*!<Clock polarity						*/
#define SPI_CR1_MSTR        2		/*!<Master selection						*/
#define SPI_CR1_BR          3		/*!<Baud rate control						*/
#define SPI_CR1_SPE         6		/*!<SPI enable							*/
#define SPI_CR1_LSBFIRST    7		/*!<Frame format						*/
#define SPI_CR1_SSI         8		/*!<Internal slave select					*/
#define SPI_CR1_SSM         9		/*!<Software slave management					*/
#define SPI_CR1_RXONLY      10		/*!<Receive only mode enable					*/
#define SPI_CR1_DFF         11		/*!<Data frame format						*/
#define SPI_CR1_CRCNEXT     12		/*!<CRC transfer next						*/
#define SPI_CR1_CRCEN       13		/*!<Hardware CRC calculation enable				*/
#define SPI_CR1_BIDIOE      14		/*!<Output enable in bidirectional mode				*/
#define SPI_CR1_BIDIMODE    15		/*!<Bidirectional data mode enable				*/

/*
 * SPI register: bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN        0	/*!<Rx buffer DMA enable					*/
#define SPI_CR2_TXDMAEN        1	/*!<Tx buffer DMA enable					*/
#define SPI_CR2_SSOE           2	/*!<SS output enable						*/
#define SPI_CR2_FRF            4	/*!<Frame format						*/
#define SPI_CR2_ERRIE          5	/*!<Error interrupt enable					*/
#define SPI_CR2_RXNEIE         6	/*!<RX buffer not empty interrupt enable			*/
#define SPI_CR2_TXEIE          7	/*!<Tx buffer empty interrupt enable				*/

/*
 * SPI register: bit position definitions SPI_SR
 */
#define SPI_SR_RXNE         0		/*!<Receive buffer not empty					*/
#define SPI_SR_TXE          1		/*!<Transmit buffer empty					*/
#define SPI_SR_CHSIDE       2		/*!<Channel side						*/
#define SPI_SR_UDR          3		/*!<Underrun flag						*/
#define SPI_SR_CRCERR       4		/*!<CRC error flag						*/
#define SPI_SR_MODF         5		/*!<Mode fault							*/
#define SPI_SR_OVR          6		/*!<Overrun flag						*/
#define SPI_SR_BSY          7		/*!<Busy flag							*/
#define SPI_SR_FRE          8		/*!<Frame Error							*/

/*
 * I2C register: bit position definitions I2C_CR1
 */
#define I2C_CR1_PE           0		/*!<Peripheral enable						*/
#define I2C_CR1_SMBUS        1		/*!<: SMBus mode						*/
#define I2C_CR1_NOSTRETCH    7		/*!<Clock stretching disable (Slave mode)			*/
#define I2C_CR1_START        8		/*!<Start generation						*/
#define I2C_CR1_STOP         9		/*!<Stop generation						*/
#define I2C_CR1_ACK          10		/*!<Acknowledge enable						*/
#define I2C_CR1_SWRST        15		/*!<Software reset						*/

/*
 * I2C register: bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ           0	/*!<Peripheral clock frequency					*/
#define I2C_CR2_ITERREN        8	/*!<Error interrupt enable					*/
#define I2C_CR2_ITEVTEN        9	/*!<Event interrupt enable					*/
#define I2C_CR2_ITBUFEN        10	/*!<Buffer interrupt enable					*/
#define I2C_CR2_DMAEN          11	/*!<DMA requests enable						*/
#define I2C_CR2_LAST           12	/*!<DMA last transfer						*/




/*
 * I2C register: bit position definitions I2C_SR1
 */
#define I2C_SR1_SB           0		/*!<Start bit (Master mode)					*/
#define I2C_SR1_ADDR         1		/*!<Address sent (master mode)/matched (slave mode)		*/
#define I2C_SR1_BTF          2		/*!<Byte transfer finished					*/
#define I2C_SR1_ADD10        3		/*!<10-bit header sent (Master mode)				*/
#define I2C_SR1_STOPF        4		/*!<Stop detection (slave mode)					*/
#define I2C_SR1_RXNE         6		/*!<Data register not empty (receivers)				*/
#define I2C_SR1_TXE          7		/*!<Data register empty (transmitters)				*/
#define I2C_SR1_BERR         8		/*!<Bus error 							*/
#define I2C_SR1_ARLO         9		/*!<Arbitration lost (master mode)				*/
#define I2C_SR1_AF           10		/*!<Acknowledge failure						*/
#define I2C_SR1_OVR          11		/*!<Overrun/Underrun						*/
#define I2C_SR1_PECERR       12		/*!<PEC Error in reception					*/
#define I2C_SR1_TIMEOUT      14		/*!<Timeout or Tlow error					*/
#define I2C_SR1_SMBALERT     15		/*!<SMBus alert							*/

/*
 * I2C register: bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL           0		/*!<Master/slave						*/
#define I2C_SR2_BUSY          1		/*!<Bus busy							*/
#define I2C_SR2_TRA           2		/*!<Transmitter/receiver					*/
#define I2C_SR2_GENCALL       4		/*!<General call address (Slave mode) 				*/
#define I2C_SR2_DUALF         7		/*!<Dual flag (Slave mode)					*/

/*
 * I2C register: bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR           0		/*!<Clock control register in Fm/Sm mode (Master mode)		*/
#define I2C_CCR_DUTY          14	/*!<Fm mode duty cycle						*/
#define I2C_CCR_FS            15	/*!<I2C master mode selection					*/

/*
 * USART register: bit position definitions USART_CR1
 */
#define USART_CR1_RE        2		/*!<Receiver enable						*/
#define USART_CR1_TE        3		/*!Transmitter enable						*/
#define USART_CR1_IDLEIE    4		/*!<IDLE interrupt enable					*/
#define USART_CR1_RXNEIE    5		/*!<RXNE interrupt enable					*/
#define USART_CR1_TCIE      6		/*!<Transmission complete interrupt enable			*/
#define USART_CR1_TXEIE     7		/*!<TXE interrupt enable					*/
#define USART_CR1_PS        9		/*!<Parity selection						*/
#define USART_CR1_PCE       10		/*!<Parity control enable					*/
#define USART_CR1_M         12		/*!<Word length							*/
#define USART_CR1_UE        13		/*!<USART enable						*/
#define USART_CR1_OVER8     15		/*!<Oversampling mode						*/

/*
 * USART register: bit position definitions USART_CR2
 */
#define USART_CR2_STOP     12		/*!<STOP bits							*/

/*
 * USART register: bit position definitions USART_CR3
 */
#define USART_CR3_EIE      0		/*!<Error interrupt enable					*/
#define USART_CR3_RTSE     8		/*!<RTS enable							*/
#define USART_CR3_CTSE     9		/*!<CTS enable							*/
#define USART_CR3_CTSIE    10		/*!<CTS interrupt enable					*/

/*
 * USART register: bit position definitions USART_SR
 */
#define USART_SR_FE       1			/*!<Framing error					*/
#define USART_SR_NF       2			/*!<Noise detected flag					*/
#define USART_SR_ORE      3			/*!<Overrun error					*/
#define USART_SR_IDLE     4			/*!<IDLE line detected					*/
#define USART_SR_RXNE     5			/*!<Read data register not empty			*/
#define USART_SR_TC       6			/*!<Transmission complete				*/
#define USART_SR_TXE      7			/*!<Transmit data register empty			*/
#define USART_SR_CTS      9			/*!<CTS flag						*/

/*
 * Generic MACROs
 */
#define ENABLE                1
#define DISABLE               0
#define SET       			  ENABLE
#define RESET      			  DISABLE
#define GPIO_PIN_SET          ENABLE
#define GPIO_PIN_RESET        DISABLE
#define FLAG_SET			  SET
#define FLAG_RESET			  RESET

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
