//
//  MECH5030_Sensors.h
//  
//
//  Created by Matthew Fisher on 27/03/2021.
//

#ifndef MECH5030_Sensors_h
#define MECH5030_Sensors_h

#define IMU2_AG_ADDRESS     0x6A
#define IMU2_M_ADDRESS      0x1C
#define IMU3_AG_ADDRESS     0X6B
#define IMU3_M_ADDRESS      0X1E

#define RAD_TO_DEG      57.2958

#include "Arduino.h"
#include <Arduino_LSM9DS1.h>    // Library for Nano built-in IMU chip
#include <Adafruit_LSM6DS33.h>  // Library for external IMU accel and gyro chip
#include <Adafruit_LIS3MDL.h>   // Library for external IMU magnetometer chip
#include <Adafruit_Sensor.h>

/**
 * A data struct to hold 3 D.o.F values in variables x, y, and z.
 * example useage:
 *    xyzStruct accel_reading;
 *    accel_reading.x = 9.81;
 *    Serial.println(accel_reading.x);
 */
struct xyzData {
    float x;
    float y;
    float z;
};

/**
 * A data struct to hold readings for a 9 D.o.F IMU values in variables accel, gyro, and mag.
 * example useage:
 *    imuStruct imu_readings;
 *    imu_readings.accel.x = 9.81;
 *    Serial.println(imu_readings.accel.x);
 */
struct imuData {
    xyzData accel;
    xyzData gyro;
    xyzData mag;
};

struct threeImuData {
    long timestamp;
    imuData imu1;
    imuData imu2;
    imuData imu3;
};

class MECH5030_Sensors{
  private:
    LSM9DS1Class *_imu1 = &IMU;
    Adafruit_LSM6DS33 *_imu2_ag;
    Adafruit_LIS3MDL *_imu2_m;
    Adafruit_LSM6DS33 *_imu3_ag;
    Adafruit_LIS3MDL *_imu3_m;
    
  public:
    
    bool accelEnabled = true; // Whether data should be collected for accelerometers
    bool gyroEnabled; // Whether data should be collected for gyroscopes
    bool magEnabled; // Whether data should be collected for magnetometers
    
    /** Constructor */
    MECH5030_Sensors(void);
    
    /** Constructor */
    ~MECH5030_Sensors(void);
    
    bool init(void);

    void getAccelerometer(bool state);
    void getGyrometer(bool state);
    void getMagnetometer(bool state);
    
    String getDataAsString(String delim, unsigned char dec_precision);
    threeImuData getData();
    
    
  private:
    
    bool init_ext_imu_ag(Adafruit_LSM6DS33 *_imu);
    
    bool init_ext_imu_m(Adafruit_LIS3MDL *_imu);
    
    // Struct variables to hold the imu data
    imuData _imu1_data;
    imuData _imu2_data;
    imuData _imu3_data;
};

#endif /* MECH5030_Sensors_h */
