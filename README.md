![RealTemp Logo](https://user-images.githubusercontent.com/21014451/187782653-f4151c87-eb0a-4d6f-b1ec-8fff1e7c7f93.png)

# RealTemp
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/rohanperi/RealTemp)
![GitHub issues](https://img.shields.io/github/issues-raw/rohanperi/RealTemp)

A series of LEDs that tell you how warm your room is - Made with an STM32F7, TMP102 sensor, I2C, and FreeRTOS

Demo - https://drive.google.com/file/d/1CxCEilgngU1GPsF856IsTzhS2-I3HScR/view

# Installation
 
1. Clone the repository by doing ```git clone https://github.com/rohanperi/RealTemp.git``` in your terminal.
2. Make sure you have ST Cube MX IDE: https://www.st.com/en/development-tools/stm32cubeide.html 
3. Have your Microcontroller (MCU) ready, I used an STM32F767 but other MCUs from ST can work too!
4. Open this project in Cube MX, making sure your MCU is connected to your PC, then build and run the code

# Usage

In order to see useful outputs, you must create the circuit:

![RealTemp](https://user-images.githubusercontent.com/21014451/187785385-76615142-3646-4cf1-8b4d-174c90cde3ea.jpg)

This video can be used to help wire the TMP102 Sensor: https://www.youtube.com/watch?v=isOekyygpR8

# Parts

MCU: https://www.st.com/en/microcontrollers-microprocessors/stm32f767zi.html 

TMP102 Sensor: https://www.sparkfun.com/products/13314


