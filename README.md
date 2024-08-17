# GyroRaycaster
Gamelike raycaster where viewer can be moved and rotated by gyroscope sensor. When the viewer has reached the exit to the outer world the games ends.

![screenshot](/assets/images/Screenshot.png) 

Video https://youtu.be/0g54CI1bC_A

## Hardware
Arduino Uno/Nano with gyroscope sensor MPU6050 and SSD1306 OLED 128x64 pixel display

![photo](/assets/images/GyroRaycaster.jpg) 

![breadboard](/assets/images/Breadboard.svg) 

Running on breadboard was not very stable for me over time (occassionally i2c-bus freezes). The same circuit soldered on board ran stable.

![schema](/assets/images/Schema.svg) 

# Can I use a SH1106 display instead of a SSD1306?
Official: No

The project was designed for a SSD1306 display.

To get the project working on a SH1106 display (as requested on https://www.youtube.com/watch?v=0g54CI1bC_A)
there are some challenges to be resolved :  
 1) The library Adafruit_SSD1306 (I used v2.5.7) does not support the SH1106 display => Adafruit_SH110X is needed (I used v2.1.10)
 2) The library Adafruit_SH110X seems to have a higher memory footprint then the Adafruit_SSD1306 library => Project would compile, but does not run (=Black screen)
 3) Removing of display strings to reduce memory consumption in this special [SH1106 version](/inofficial/GyroRaycasterSH1106/GyroRaycasterSH1106.ino) got the project working on a SH1106 display
 => Use a SSD1306 display for this project or be able to fix it yourself.

![screenshot](/assets/images/SH1106.png) 
