# Apple II Disk Drive

## Introduction

This project allows you to hook an old Apple //c (with ROM 255 which does not have support for Smartport) to an Arduino connected to an SD card which acts as an Apple 2 Disk drive. Here are some key facts:

- It can be used on old ROM 255 Apple //c.
- It uses a standard Arduino Nano (no 5V/3.3V headaches)
- For the moment it's read-only.
- The disk images must be uploaded in `NIB` format (more about this later)
- You can use the switches to select any `.nib` file in the SD card
- I use a SSD1306 display to show the images on the SD card, which one you are using and the track you are reading.
- Has an LED to show disk access (this is mostly for debugging purposes)


## References

- **SmartportSD project** - There are many iterations of this project that allows you to connect your Apple //c to an SD card. The problem is that all those version require you to have an Apple //c that supports Smartport. Unforunately I have a 255 version ROM in my Apple so none of them work for me. I use their cycle count code to send data to the IWM on the Apple //c.
- [SoftI2CMaster](https://github.com/felias-fogg/SoftI2CMaster) - Yet another software I2C Arduino library. Needed because we are using the I2C pins for the SDCard.
- [SDFat](https://github.com/greiman/SdFat) - Arduino FAT16/FAT32 exFAT Library
- [SSD1306_minimal](https://github.com/kirknorthrop/SSD1306_minimal) - I modified this last version for my usage. At this point I don't remember what I modified.


## Circuit

The following diagram shows you how to hook up the circuit.

![Circuit diagram](/img/a2diskarduino-layout.png)

To edit: https://circuitcanvas.com/p/3vo5y2um8k7dad4r851?canvas=layout
