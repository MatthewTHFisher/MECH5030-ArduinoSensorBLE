//
//  MECH5030_Sensors.cpp
//  
//
//  Created by Matthew Fisher on 27/03/2021.
//

#include "MECH5030_Sensors.h"

MECH5030_Sensors::MECH5030_Sensors(void) :
    _imu2_ag(new Adafruit_LSM6DS33),
    _imu2_m(new Adafruit_LIS3MDL),
    _imu3_ag(new Adafruit_LSM6DS33),
    _imu3_m(new Adafruit_LIS3MDL)
{}

MECH5030_Sensors::~MECH5030_Sensors(void)
{
    delete _imu2_ag;
    delete _imu2_m;
    delete _imu3_ag;
    delete _imu3_m;
}


bool MECH5030_Sensors::init(void)
{
    // BEGIN I2C on ALL IMUS
    Serial.println("Attemping initalisation of IMUS...");
    if (!_imu1->begin())             { return false; }
    Serial.println("IMU1 Init");
    if (!_imu2_ag->begin_I2C(IMU2_AG_ADDRESS))  { return false; }
    Serial.println("IMU2 AG Init");
    if (!_imu2_m->begin_I2C(IMU2_M_ADDRESS))   { return false; }
    Serial.println("IMU2 M Init");
    if (!_imu3_ag->begin_I2C(IMU3_AG_ADDRESS))  { return false; }
    Serial.println("IMU3 AG Init");
    if (!_imu3_m->begin_I2C(IMU3_M_ADDRESS))   { return false; }
    Serial.println("IMU3 M Init");
    
    init_ext_imu_ag(_imu2_ag);
    init_ext_imu_m(_imu2_m);
    init_ext_imu_ag(_imu3_ag);
    init_ext_imu_m(_imu3_m);
    
    return true;
}

// Function to initialise the Accel and Gyro chip of an external IMU
bool MECH5030_Sensors::init_ext_imu_ag(Adafruit_LSM6DS33 *_imu)
{

    // Accelerometer setup to match that found on the Nano
    _imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
    _imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

    // Gyroscope setup to match that found on the Nano
    _imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
    _imu->setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    
    return true;
}

// Function to initialise the Mag chip of an external IMU
bool MECH5030_Sensors::init_ext_imu_m(Adafruit_LIS3MDL *_imu)
{
    
    // Magnetometer setup to match that found on the Nano
    _imu->setDataRate(LIS3MDL_DATARATE_20_HZ);
    _imu->setRange(LIS3MDL_RANGE_4_GAUSS);
    
    return true;
}

void MECH5030_Sensors::getAccelerometer(bool state)
{
    accelEnabled = state;
}

void MECH5030_Sensors::getGyrometer(bool state)
{
    gyroEnabled = state;
}

void MECH5030_Sensors::getMagnetometer(bool state)
{
    magEnabled = state;
}

String MECH5030_Sensors::getDataAsString(String delim, unsigned char dec_precision)
{
    threeImuData data = getData();
    
    String str = String(data.timestamp);

    // Adds data for IMU 1 (The onboard imu)
    if(accelEnabled){ str = str + delim + String(data.imu1.accel.x, dec_precision) + delim + String(data.imu1.accel.y, dec_precision) + delim + String(data.imu1.accel.z, dec_precision); }
    if(gyroEnabled){ str = str + delim + String(data.imu1.gyro.x, dec_precision) + delim + String(data.imu1.gyro.y, dec_precision) + delim + String(data.imu1.gyro.z, dec_precision); }
    if(magEnabled){ str = str + delim + String(data.imu1.mag.x, dec_precision) + delim + String(data.imu1.mag.y, dec_precision) + delim + String(data.imu1.mag.z, dec_precision); }

    // Adds data for IMU 2
    if(accelEnabled){ str = str + delim + String(data.imu2.accel.x, dec_precision) + delim + String(data.imu2.accel.y, dec_precision) + delim + String(data.imu2.accel.z, dec_precision); }
    if(gyroEnabled){ str = str + delim + String(data.imu2.gyro.x, dec_precision) + delim + String(data.imu2.gyro.y, dec_precision) + delim + String(data.imu2.gyro.z, dec_precision); }
    if(magEnabled){ str = str + delim + String(data.imu2.mag.x, dec_precision) + delim + String(data.imu2.mag.y, dec_precision) + delim + String(data.imu2.mag.z, dec_precision); }

    // Adds data for IMU 3
    if(accelEnabled){ str = str + delim + String(data.imu3.accel.x, dec_precision) + delim + String(data.imu3.accel.y, dec_precision) + delim + String(data.imu3.accel.z, dec_precision); }
    if(gyroEnabled){ str = str + delim + String(data.imu3.gyro.x, dec_precision) + delim + String(data.imu3.gyro.y, dec_precision) + delim + String(data.imu3.gyro.z, dec_precision); }
    if(magEnabled){ str = str + delim + String(data.imu3.mag.x, dec_precision) + delim + String(data.imu3.mag.y, dec_precision) + delim + String(data.imu3.mag.z, dec_precision); }

    return str;
}

// Returns data in the format (m/s^s, deg/s, uT)
threeImuData MECH5030_Sensors::getData(void)
{
    // Log the current timestamp of the Arduino
    long timestamp = millis();
    
    // IMU 1 Data Read (G's, deg/s, uT)
    // >> Data collection for the Nano's built-in IMU
    if (_imu1->accelerationAvailable() && accelEnabled)  {
        _imu1->readAcceleration(_imu1_data.accel.x, _imu1_data.accel.y, _imu1_data.accel.z);

        //Onboard IMU-accelerometer requires converting to change from G's into M/S^2
        _imu1_data.accel.x = _imu1_data.accel.x * SENSORS_GRAVITY_STANDARD;
        _imu1_data.accel.y = _imu1_data.accel.y * SENSORS_GRAVITY_STANDARD;
        _imu1_data.accel.z = _imu1_data.accel.z * SENSORS_GRAVITY_STANDARD;
    }
    
    // Onboard IMU-Gyrometer reads data in
    if (_imu1->gyroscopeAvailable() && gyroEnabled) {
        _imu1->readGyroscope(_imu1_data.gyro.x, _imu1_data.gyro.y, _imu1_data.gyro.z);
        
        _imu1_data.gyro.x = _imu1_data.gyro.x - imu1GyroOffsets.x;
        _imu1_data.gyro.y = _imu1_data.gyro.y - imu1GyroOffsets.y;
        _imu1_data.gyro.z = _imu1_data.gyro.z - imu1GyroOffsets.z;
    }
    
    // Onboard IMU-Magnetometer reads data in uT
    if (_imu1->magneticFieldAvailable() && magEnabled) {
        _imu1->readMagneticField(_imu1_data.mag.x, _imu1_data.mag.y, _imu1_data.mag.z);
    }
    
    // Temporary variables to hold the data of the external IMUs
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sensors_event_t mag;
    
    // IMU 2 Data Read (M/S^2, rad/s, uT)
    if (accelEnabled || gyroEnabled) {    //Since accel and gyro are on the same chip we must do both together
      _imu2_ag->getEvent(&accel, &gyro, &temp);
      _imu2_data.accel.x = accel.acceleration.x;
      _imu2_data.accel.y = accel.acceleration.y;
      _imu2_data.accel.z = accel.acceleration.z;
      _imu2_data.gyro.x = (gyro.gyro.x*RAD_TO_DEG) - imu2GyroOffsets.x;
      _imu2_data.gyro.y = (gyro.gyro.y*RAD_TO_DEG) - imu2GyroOffsets.y;
      _imu2_data.gyro.z = (gyro.gyro.z*RAD_TO_DEG) - imu2GyroOffsets.z;
    }
    if (magEnabled) {    // If we don't need to retrieve mag data we won't waste time doing this
      _imu2_m->getEvent(&mag);
      _imu2_data.mag.x = mag.magnetic.x;
      _imu2_data.mag.y = mag.magnetic.y;
      _imu2_data.mag.z = mag.magnetic.z;
    }
    
    // IMU 3 Data Read (M/S^2, rad/s, uT)
    if (accelEnabled || gyroEnabled) {    //Since accel and gyro are on the same chip we must do both together
      _imu3_ag->getEvent(&accel, &gyro, &temp);
      _imu3_data.accel.x = accel.acceleration.x;
      _imu3_data.accel.y = accel.acceleration.y;
      _imu3_data.accel.z = accel.acceleration.z;
      _imu3_data.gyro.x = (gyro.gyro.x*RAD_TO_DEG) - imu3GyroOffsets.x;
      _imu3_data.gyro.y = (gyro.gyro.y*RAD_TO_DEG) - imu3GyroOffsets.y;
      _imu3_data.gyro.z = (gyro.gyro.z*RAD_TO_DEG) - imu3GyroOffsets.z;
    }
    if (magEnabled) {    // If we don't need to retrieve mag data we won't waste time doing this
      _imu3_m->getEvent(&mag);
      _imu3_data.mag.x = mag.magnetic.x;
      _imu3_data.mag.y = mag.magnetic.y;
      _imu3_data.mag.z = mag.magnetic.z;
    }
    
    return {
        timestamp,
        _imu1_data.accel.x, _imu1_data.accel.y, _imu1_data.accel.z, _imu1_data.gyro.x, _imu1_data.gyro.y, _imu1_data.gyro.z, _imu1_data.mag.x, _imu1_data.mag.y, _imu1_data.mag.z,
        _imu2_data.accel.x, _imu2_data.accel.y, _imu2_data.accel.z, _imu2_data.gyro.x, _imu2_data.gyro.y, _imu2_data.gyro.z, _imu2_data.mag.x, _imu2_data.mag.y, _imu2_data.mag.z,
        _imu3_data.accel.x, _imu3_data.accel.y, _imu3_data.accel.z, _imu3_data.gyro.x, _imu3_data.gyro.y, _imu3_data.gyro.z, _imu3_data.mag.x, _imu3_data.mag.y, _imu3_data.mag.z
        };
    
}

