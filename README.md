STM32F103 USB peripheral expander
----

This project is a USB peripheral expander for the stm32f103.
It's made for stm32f103c8t6 but with some modification depending
on the flash density and clocks can be used for any stm32f10x.

The aim is to export the stm32 peripherals to a host under a
simple USB api, so the host can access the stm32's peripherals and
use them. It is fully configurable and it's possible to add/remove
peripherals on the fly.

> This is a project in progress.

The aim is to be able to control these peripherals using a gui
app on your desktop (Linux/Windows/Macos).

## Supported peripherals
Currently the supported peripherals are the following:

* GPIOs (I/O)
    * PA8
    * PA9
    * PB2
    * PB8
    * PB9
    * PB10
    * PB11
    * PB12
    * PB13
    * PB14
    * PB15
    * PC13
    * PC14
    * PC15

* ADC1 channels:
    * ADC1_IN0 - PA0
    * ADC1_IN1 - PA1
    * ADC1_IN4 - PA4
    * ADC1_IN5 - PA4
    * ADC1_IN6 - PA5
    * ADC1_IN7 - PA6
    * ADC1_IN8 - PB1
    * ADC1_IN9 - PB2

* SPI1
    * NSS - PA15
    * SCK - PB3
    * MISO - PB4
    * MOSI - PB5

* USART1
    * TX - PA9
    * RX - PA10

* USART2
    * TX - PA2
    * RX - PA3

* USB - Dual CDC device

## TODO:
#### peripherals:
These are the peripherals that are not implemented yet

* I2C1 w/ DMA
    * SDA - PB7
    * SCL - PB6

#### GUI
A gui application that exports these peripherals to the OS

## FW details
* `CMSIS version`: 5.3.0
* `StdPeriph Library version`: 3.6.1
* `STM3 USB Driver version`: 4.1.0

