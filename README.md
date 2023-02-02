# Driver Development Codes for ARM Cortex M4 microcontroller using STM32 development board
Understanding how to write drivers is a vital skill that embedded software developers need to know. There are normally two sort of drivers in an embedded system: drivers for peripheral devices linked to the microcontroller and drivers for external devices coneected via an interface like SPI, I2C or UART. These days manufactureers offer sample drivers for their chips that can be used as it is or may need to be modified for production. 
This repository contains driver files for GPIO, SPI, I2C and UART peipherals of STM32F446RE microcontroller. 
# Project Architecture
Driver layer is required to operate peripherals from application program. With the help of drivers layer it is very easy for the user to configure the peripherals and perform different operations on them to get the results. 
![Blank board](https://user-images.githubusercontent.com/124070100/216399758-6cef157c-7bfc-4b35-8ef5-6f06d7e89fde.png)
Driver layer includes different source and header file. 
## stm32f446xx 
This header file is processor specific header files, it contains all necessary programing libraries, generic macros, memory addresses. 
