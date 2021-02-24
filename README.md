# MECH5030_Arduino
*The following repository is for the use of a University of Leeds project within MECH5030 2020-2021.*

The project uses an Arduino, BLE and IMU Sensors to gather data from multiple sensors and transfer them using the BLE communication protocol.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

To compile and upload the code there are a series of libraries that must be installed. The required list resides at the top of each *.ino* file. For example:

```c
#include <ArduinoBLE.h>         // Library for Nano built-in BLE chip
#include <Arduino_LSM9DS1.h>    // Library for Nano built-in IMU chip
#include <Adafruit_LSM6DS33.h>  // Library for external IMU accel and gyro chip
#include <Adafruit_LIS3MDL.h>   // Library for external IMU magnetometer chip
#include <Adafruit_Sensor.h>    // Required for the Adafruit library
```
### Installing

To install the required libraries into the Arduino IDE there are two methods

**1. Using Arduino's Library Manager**
&nbsp;&nbsp;&nbsp;&nbsp;Go to tools -> Manage Libraries... (or CTRL + SHIFT + I). With the library manager open simply search for desired library and then click the install button.
&nbsp;&nbsp;&nbsp;&nbsp;**Note:** If asked to install additional libraries it is advised to choose YES otherwise the installed library may not work as it should.
   
**2. Manually downloading the Library Folder**
&nbsp;&nbsp;&nbsp;&nbsp;Find the library that you require installing (Heres the online library for [Adafruit_LSM6DS.h](https://github.com/adafruit/Adafruit_LSM6DS)). Once downloaded onto the PC simply move it in to your folder for Arduino libraries **/Arduino/libraries/*.
&nbsp;&nbsp;&nbsp;&nbsp;**Note:** This folder should already have been made when installing the Arduino IDE software onto your PC and typically resides within *C:/Users/YOUR_USER/Documents/*.
## Authors

* **Matthew Fisher** - *el16m2f*
