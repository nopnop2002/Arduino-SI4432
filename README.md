# Arduino-SI443
Arduino library for SILICON LABS SI443x.   

I made it with reference to [this](https://github.com/ADiea/si4432).   

- 2.54mm pitch 433MHz   
 ![SI4432-433MHz-2 54mm](https://user-images.githubusercontent.com/6020549/163330289-770357cd-5bb4-4030-9347-edd0da6f21d3.JPG)

- 1.27mm pitch 433MHz   
 ![SI4432-433MHz-1 27mm](https://user-images.githubusercontent.com/6020549/170854841-ab5318ae-7b31-4d11-98d9-719f48a49c36.JPG)
 Pinout is [here](http://www.dorji.com/docs/data/DRF4432F20.pdf).   

# Changes from the original

- Changed legacy SPI communication to SPI communication with transactions.   

- Added CS pin parameters to the constructor.   

- Removed receive processing from sendPacket function.   

- Removed waitForPacket function.   

- Added SyncWord validation.   
 Sync Word 3 reset value is 0x2D.   
 Sync Word 2 reset value is 0xD4.   

- Added example code.   



# Wireing
|SI4432||UNO|MEGA|ESP8266|
|:-:|:-:|:-:|:-:|:-:|
|VCC|--|3.3V(*1)|3.3V|3.3V|
|GND|--|GND|GND|GND|
|SCK|--|D13(*2)|D52(*2)|IO14|
|MISO|--|D12|D50|IO12|
|MOSI|--|D11(*2)|D51(*2)|IO13|
|SEL|--|D10(*2)|D10(*2)|IO15|
|SDN|--|D7(*2)|D7(*2)|IO4|
|IRQ|--|D2|D2|IO5|

(*1)   
UNO's 3.3V output can only supply 50mA.   
In addition, the output current capacity of UNO-compatible devices is smaller than that of official products.   
__So this module may not work normally when supplied from the on-board 3v3.__   

(*2)   
SI4432 is not 5V tolerant.   
You need level shift from 5V to 3.3V.   
I used [this](https://www.ti.com/lit/ds/symlink/txs0108e.pdf?ts=1647593549503) for a level shift.   

