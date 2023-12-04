# Si443x ISM Transceiver Library for Arduino

This library version is an enhancement of the [origial GitHub project](https://github.com/ADiea/si4432) by ahmetipkin. 

It can be an alternative to the [RadioHead library](http://www.airspayce.com/mikem/arduino/RadioHead) when you are focused on the Si443x and need a specific combination of parameters that are not supported by the RadioHead library (e.g. Manchester encoding).

## Boards

Example of boards with the Silicon Labs Si4430/31/32 chip:

- 2.54mm pitch 433MHz   
  ![Si4432-433MHz-2 54mm](https://user-images.githubusercontent.com/6020549/163330289-770357cd-5bb4-4030-9347-edd0da6f21d3.JPG)

- 1.27mm pitch 433MHz   
  ![Si4432-433MHz-1 27mm](https://user-images.githubusercontent.com/6020549/170854841-ab5318ae-7b31-4d11-98d9-719f48a49c36.JPG)  
  [Pinout](http://www.dorji.com/docs/data/DRF4432F20.pdf)   

## Features

- Support floating point values for frequency (MHz) and baud rate (kbps).

- Select modulation type GFSK (default) or OOK.

- Select Manchester encoding (default off).

- Select packet handling (default on).

- Select transmit power (default max).

- Select blocking (default) or non-blocking transmit.

- Select idle mode from standby, sleep, ready (default) and tune.

- Boot config callback for individual chip settings (e.g. antenna switch)

- Boot SyncWord validation:  
    Sync Word 3 0x2D.  
    Sync Word 2 0xD4.  
    Sync Word 1 0x00.  
    Sync Word 0 0x00.  

- Use SPI transactions. Legacy SPI communication works fine on ATMega with fixed CPU frequency, but doesn't work on CPUs like STM32 due to overclocking.   

- Configurable CS pin parameter.   

- Example code for transmit and receive.

## Breaking Changes

- Support pin 0, use 0xFF when pin is not available.

- Removed delay in turnOn() to allow non-blocking wakeup.

- Removed receive processing from sendPacket function.

- Removed waitForPacket function.

## Wiring

|Si4432||UNO|MEGA|ESP8266|SAMD21|
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC|--|3.3V(*1)|3.3V|3.3V|3.3V|
|GND|--|GND|GND|GND|GND|
|SCK|--|D13(*2)|D52(*2)|IO14|SCK|
|MISO|--|D12|D50|IO12|MISO|
|MOSI|--|D11(*2)|D51(*2)|IO13|MOSI|
|/SEL|--|D10(*2)|D10(*2)|IO15|(*3)|
|SDN|--|D7(*2)|D7(*2)|IO4|(*3)|
|/IRQ|--|D2|D2|IO5|(*3)|

(*1)   
__The Si443X requires up to 30 mA and may not work reliably when supplied from the MCU on-board 3v3 voltage regulator.__   
UNO's 3.3V output can supply 50 mA but the output current capacity of UNO-compatible devices is typically lower than that of official products.   

(*2)   
Si4432 is not 5V tolerant. You need to level shift between 5V and 3.3V, e.g. with this [level shifter](https://www.ti.com/lit/ds/symlink/txs0108e.pdf?ts=1647593549503).   

(*3)
Configurable via constructor.