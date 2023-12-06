![RoboBoard-ESP32](https://github.com/LaskaKit/RoboBoard-ESP32/blob/main/img/LaskaKit-roboboard-esp32-2.jpg)

# LaskaKit RoboBoard ESP32
Build a robot! Don't worry, it's nothing complicated. It's easy with our RoboBoard-ESP32. The RoboBoard has everything you need for a simple robot on it, and you can make it into a much more complex robot over time thanks to the expansion connectors for both the I2C and SPI bus. 

You can connect a lot of other peripherals. 

The basis of the whole board is ESP32 - it hides a great performance, the possibility to use Wi-Fi or Bluetooth interface and also a large community of DIY makers who have already invented many projects with it. It is not something unknown - ESP32 is one of the most well-known and used modules ever. 

The RoboBoard-ESP32 has a ready-to-use programmer (CH9102F), just plug the USB-C cable into the board and your computer, start the Arduino IDE and you can start programming. The converter-programmer will take care of uploading the program to the ESP32. 

The RoboBoard-ESP32 can drive up to 4 DC motors using PWM, thanks to the TB6612 driver. The maximum continuous current through one channel is 1A. The TB6612 DC motor driver is controlled via the PCA9685, which is controlled via the I2C bus. So you only need two wires (I2C, SDA - GPIO21 and SCL - GPIO22) to drive up to 4 DC motors. 

If you don't want to use DC motors or on the contrary you would like to add Servos to them, RoboBoard-ESP32 is ready for this option as well. 
You can connect up to 8 servo motors - what you use the servo motors for is up to you. The individual servo motors are controlled by GPIO16 to GPIO19, then GPIO25 and GPIO26 and lastly GPIO32 and GPIO33. 

What else have we prepared on the board? We have fitted a buzzer, you can control it via GPIO27. Then we have fitted two I2C ushup connectors to connect I2C sensors that are powered by 3.3V (SDA - GPIO21 and SCL - GPIO22) and one SPI ushup to connect other sensors or displays powered by 3.3V (CS - GPIO15, SDI - GPIO13, CLK - GPIO14, SDO - GPIO12).

For the battery, we have a standard 3-pin JST-XH-3P connector where you plug in 2S LiPol batteries - for example this one https://www.laskakit.cz/rc-lipol-baterie-2s-1000mah-7-4v-20c-jst-bec/

The motors are powered via the LMR14050 step-down switching inverter from the 2S battery, which can deliver up to 5A. 

The battery voltage, via the voltage divider, is fed and measured by GPIO34. 

Sample codes will be prepared on our github at https://github.com/LaskaKit/RoboBoard-ESP32/tree/main/SW
