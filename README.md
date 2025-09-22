# Apple II Disk Drive Arduino

## Introduction

This project allows you to hook an old Apple //c (with ROM 255 which does not have support for Smartport) to an Arduino connected to an SD card which acts as an Apple 2 Disk drive. Here are some key facts:

- It can be used on old ROM 255 Apple //c.
- It uses a standard Arduino Nano (no 5V/3.3V headaches)
- For the moment it's **read-only**.
- The disk images must be uploaded in `NIB` format (more about this later)
- You can use the switches to select any `.nib` file in the SD card
- I use a SSD1306 display to show the images on the SD card, which one you are using and the track you are reading.
- Has an LED to show disk access (this is mostly for debugging purposes)


## References

- **SmartportSD project** - There are many iterations of this project that allows you to connect your Apple //c to an SD card. The problem is that all those version require you to have an Apple //c that supports Smartport. Unforunately I have a 255 version ROM in my Apple so none of them work for me. I use their cycle count code to send data to the IWM on the Apple //c.
- [SDFat](https://github.com/greiman/SdFat) - Arduino FAT16/FAT32 exFAT Library. This is used to access the SD card from the Arduino. Currently on `v2.3.0`.
- [SSD1306_minimal](https://github.com/kirknorthrop/SSD1306_minimal) - Most libraries that use the SSD1306 use a lot of memory. In the Arduino you only have 2k of RAM. The sector buffer takes up ~416 bytes, the SD library more so I need a very lightweight version of the drivers for the display. I modified this last version for my usage. At this point I don't remember what I modified.
- [SoftI2CMaster](https://github.com/felias-fogg/SoftI2CMaster) - For some reason I can't remember I wasn't able to use the standard i2c Arduino library. Maybe because of memory usage or maybe because the standard pins were occupied (or maybe both). For whatever reason I ended up using this one. Currently running `v2.1.9`

## Circuit

The following diagram shows you how to hook up the circuit.

![Circuit diagram](/img/a2diskarduino-layout.png)

To edit: https://circuitcanvas.com/p/3vo5y2um8k7dad4r851?canvas=layout

This is the pinout I use as reference for the Arduino Nano

![Pinout](/img/arduino-pinout.jpeg)
