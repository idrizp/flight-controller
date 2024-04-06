# Flight Controller

A simple flight-controller microcontroller built on top of the STM32. 

## Components used:
- CC2500 - The transceiver used for the flight controller.
- ICP-10100 - The barometer used for altitude checking. Not too precise, but it should work for a hobbyist project.
- ICM-42688-P - The accelerometer and gyroscope 

## Driver Status:

- CC2500 - Completed (uses SPI)
- ICP-10100 - Completed (uses I2C)
- ICM-42688-P - Completed (uses SPI)

## TODO:
I plan to include the gerber file in the future, as well as the PCB schematic so others can build this project on their own. It should be relatively cheap. I went with the STM32L051XX series as it seems to be the cheapest for now. 
