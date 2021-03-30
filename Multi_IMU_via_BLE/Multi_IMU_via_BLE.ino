/*    Author(s): M. Fisher
 *    University of Leeds
 *    Date: 23/02/2021
 *    
 *    Created for MECH5030 Team Project
 *    
 *    This program is used for the Arduino Nano 33 BLE to record IMU data and transfer
 *    the data via a BLE.
 *    
 *    The program has 5 useable characteristics:
 *       1 - Start/Stop IMU recordings
 *       2 - Set the reading frequency of the IMU
 *       3 - Set the number of digits after the decimal point
 *       4 - Send IMU data via a string containing timestamp, accel, gyro, mag
 *       5 - Turn on which IMU data should be sent of accel, gyro, mag
 */

#include <ArduinoBLE.h>         // Library for Nano built-in BLE chip
#include "Libraries/MECH5030_Sensors.h"
#include "Libraries/MECH5030_Sensors.cpp"

#define BLE_LOCAL_NAME "LeedsMechEngWearable"
#define IMU_SERVICE_UUID                  "f57fd14d-86fc-4d25-a03b-0009f23d4bfd"  // "C3D0" (Previous value)
#define SENSOR_CALIBRATION_SERVICE_UUID   "2b53a4b1-33b9-48e4-945c-b7c03f8b7819"  

MECH5030_Sensors sensors;

 // BLE 9-DoF IMU Service
BLEService imuService(IMU_SERVICE_UUID);
BLEService sensorCalibrationService(SENSOR_CALIBRATION_SERVICE_UUID);

// BLE IMU Characteristic list
BLEBoolCharacteristic         startImuCharacteristic            ("49E8", BLERead | BLEWriteWithoutResponse | BLEWrite);   // 0 (False) or 1 (True), to start recording data
BLEUnsignedCharCharacteristic setReadingPrecisionCharacteristic ("98B2", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The number of units after the decimal point
BLEUnsignedCharCharacteristic setReadingFrequencyCharacteristic ("AE89", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The frequency of posting results to BLE in Hz
BLEBoolCharacteristic         accelEnabledCharacteristic        ("1A55", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether acceleration is being tracked
BLEBoolCharacteristic         gyroEnabledCharacteristic         ("3901", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether gyroscope is being tracked
BLEBoolCharacteristic         magEnabledCharacteristic          ("552B", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether magnetometer is being tracked
BLEStringCharacteristic       imuStringCharacteristic           ("785F", BLERead | BLENotify, 1000);

BLEFloatCharacteristic        imu1GyroXOffsetCharacteristic     ("6410", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu1GyroYOffsetCharacteristic     ("6411", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu1GyroZOffsetCharacteristic     ("6412", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu2GyroXOffsetCharacteristic     ("6413", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu2GyroYOffsetCharacteristic     ("6414", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu2GyroZOffsetCharacteristic     ("6415", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu3GyroXOffsetCharacteristic     ("6416", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu3GyroYOffsetCharacteristic     ("6417", BLERead | BLEWriteWithoutResponse | BLEWrite);
BLEFloatCharacteristic        imu3GyroZOffsetCharacteristic     ("6418", BLERead | BLEWriteWithoutResponse | BLEWrite);

BLEBoolCharacteristic         setBleAdvertisingCharacteristic   ("E21A", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The BLE's advertising state

volatile long previousMillis = 0;      // Variable used within the main loop timer
volatile bool imu_read = false;        // Whether to read and send IMU data or not
bool ble_connected = false;            // Whether a BLE device is connected or not
bool ble_advertising = false;          // Whether the BLE is being advertised

// Adjustable variables to adjust the imu recording
volatile unsigned char dec_precision = 3;    // Number pf digits after the decimal point
volatile unsigned char imu_frequency = 10;   // Frequency that results are recorded (Hz)

// Function definitions. These can be found later on in the file
bool ble_init();
void write_rgb_led();
void crash_sequence();

void setup() {
  // initialize serial communication
  // If no serial is connected it will continue anyway
  Serial.begin(250000);
  
  // Initialise the 4 LED peripherals attached to the Arduino board
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  write_rgb_led(255,0,0);   // Set RGB red to signal thats its setting up

  Serial.println("Initialising Arduino for IMU and Bluetooth...");
  
  /**
   * The below is peripheral setup for all the IMUs
   * If it fails to setup any peripheral then an endless crash loop is entered flashing a warning LED.
   * For the program to work all of the peripherals must be correctly plugged in!
   */
  if (!sensors.init()) { Serial.println("Failed to initalise IMUs...");  crash_sequence(); }


  if(!ble_init()){ Serial.println("Failed to initalise BLE...");  crash_sequence(); }
  
  // Signal to show that the device setup correctly
  for (int i = 0; i<=4; i++) {
    write_rgb_led(0,255,0);
    delay(200);
    write_rgb_led(0,0,0);
    delay(200);
  }

  delay(500);
  Serial.println("Bluetooth device active, waiting for connection...");
}

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {    
    while (central.connected()) {
      
      long currentMillis = millis();

      // Timer check to so that code is only completed to time of imu_frequency
      if ( (imu_read == true) && (currentMillis - previousMillis >= (1000/imu_frequency)) ) {
        previousMillis = currentMillis;

        String dataStr = sensors.getDataAsString(",", dec_precision);

        // Once all of the data is added to the string it is written to the imuString characteristic
        imuStringCharacteristic.writeValue(dataStr);
        
      }
    }
  }else{
    // If nothing is connected an orange light flashes to signal its ready to connect
    write_rgb_led(255,165,0);
    delay(500);
    write_rgb_led(0,0,0);
    delay(500);

    if(!ble_advertising){
      // If nothing conencted and BLE isn't being advertised
      // Start to advertise again
      BLE.advertise();
      ble_advertising = true;
      setBleAdvertisingCharacteristic.writeValue(ble_advertising);
    }
    
  }
}

bool ble_init() {
  if (!BLE.begin()) { Serial.println("starting BLE failed!"); return false; }

  BLE.setLocalName("LeedsMechEngWearable");  // The name that will be visible when searching for the device
  
  // Set callback functions to be called when an event such as a BLE device connects occurs
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // <- Operations for imuService
  BLE.setAdvertisedService(imuService);      // Advertise the service UUID so that the service can be accessed

  // Add the characteristics to the service
  imuService.addCharacteristic(startImuCharacteristic);
  imuService.addCharacteristic(imuStringCharacteristic);
  imuService.addCharacteristic(setReadingPrecisionCharacteristic);
  imuService.addCharacteristic(setReadingFrequencyCharacteristic);
  imuService.addCharacteristic(accelEnabledCharacteristic);
  imuService.addCharacteristic(gyroEnabledCharacteristic);
  imuService.addCharacteristic(magEnabledCharacteristic);
  imuService.addCharacteristic(setBleAdvertisingCharacteristic);

  // Add the imu service
  BLE.addService(imuService); 

  // Set initial values for characteristics
  startImuCharacteristic.writeValue(imu_read);
  imuStringCharacteristic.writeValue("");
  setReadingPrecisionCharacteristic.writeValue(dec_precision);
  setReadingFrequencyCharacteristic.writeValue(imu_frequency);
  accelEnabledCharacteristic.writeValue(sensors.accelEnabled);
  gyroEnabledCharacteristic.writeValue(sensors.gyroEnabled);
  magEnabledCharacteristic.writeValue(sensors.magEnabled);
  setBleAdvertisingCharacteristic.writeValue(ble_advertising);

  // Set callback functions to be called when a characteristics is written too
  startImuCharacteristic.setEventHandler(BLEWritten, imuStartCharacteristicWritten);
  setReadingPrecisionCharacteristic.setEventHandler(BLEWritten, imuPrecisionCharacteristicWritten);
  setReadingFrequencyCharacteristic.setEventHandler(BLEWritten, imuFrequencyCharacteristicWritten);
  accelEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  gyroEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  magEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  setBleAdvertisingCharacteristic.setEventHandler(BLEWritten, setBleAdvertisingCharacteristicWritten);

  // <- Operations for sensorCalibrationService
  BLE.setAdvertisedService(sensorCalibrationService);
  
  sensorCalibrationService.addCharacteristic(imu1GyroXOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu1GyroYOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu1GyroZOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu2GyroXOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu2GyroYOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu2GyroZOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu3GyroXOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu3GyroYOffsetCharacteristic);
  sensorCalibrationService.addCharacteristic(imu3GyroZOffsetCharacteristic);

  // Set initial values for characteristics
  imu1GyroXOffsetCharacteristic.writeValue(sensors.imu1GyroOffsets.x);
  imu1GyroYOffsetCharacteristic.writeValue(sensors.imu1GyroOffsets.y);
  imu1GyroZOffsetCharacteristic.writeValue(sensors.imu1GyroOffsets.z);
  imu2GyroXOffsetCharacteristic.writeValue(sensors.imu2GyroOffsets.x);
  imu2GyroYOffsetCharacteristic.writeValue(sensors.imu2GyroOffsets.y);
  imu2GyroZOffsetCharacteristic.writeValue(sensors.imu2GyroOffsets.z);
  imu3GyroXOffsetCharacteristic.writeValue(sensors.imu3GyroOffsets.x);
  imu3GyroYOffsetCharacteristic.writeValue(sensors.imu3GyroOffsets.y);
  imu3GyroZOffsetCharacteristic.writeValue(sensors.imu3GyroOffsets.z);

  imu1GyroXOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu1GyroYOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu1GyroZOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu2GyroXOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu2GyroYOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu2GyroZOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu3GyroXOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu3GyroYOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);
  imu3GyroZOffsetCharacteristic.setEventHandler(BLEWritten, imuGyroOffsetCharacteristicWritten);

  BLE.addService(sensorCalibrationService);

  // start advertising the BLE signal
  BLE.advertise();
  ble_advertising = true;
  setBleAdvertisingCharacteristic.writeValue(ble_advertising);

  return true;
}

// Function called when a BLE device conencts
void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());

  // Set RGB LED to Blue to signal a succesful connection
  write_rgb_led(0, 0, 255);

  ble_connected = true;

  // Since a device is connected, stop advertising the BLE signal
  BLE.stopAdvertise();
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());

  // Turn the RGB LED off to signal nothing connected
  //write_rgb_led(0,0,0);

  ble_connected = false;

  // Since no devices are connected, re-advertise
  //BLE.advertise();
}

// Function called when imuStart characteristic is written, it alters whether the IMU should be read or not.
void imuStartCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.value()[0] == 0) { 
    Serial.println("Stopping IMU recordings..."); 
    imu_read = false;
  } else { 
    Serial.println("Starting IMU recordings..."); 
    imu_read = true; 
  }
}

// Function called when imuPrecision characteristic is written, it alters the number of decimal points sent via BLE
void imuPrecisionCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("IMU Precision written: ");
  Serial.println(characteristic.value()[0]);
  
  dec_precision = characteristic.value()[0];
}

// Function called when imuFrequency characteristic is written, it alters how often the IMU is read
// Note: Mag has a max frequency of 20hz
void imuFrequencyCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("IMU Frequency written: ");
  Serial.println(characteristic.value()[0]);
  
  imu_frequency = characteristic.value()[0];
}

// Function called when a imuSetup char is written eg (accelEnabledCharacteristic)
// The function changes what data should be read/sent via BLE
void imuSetupCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  bool flag = true;

  // If value written is a 0 than the flag is set to false
  if (characteristic.value()[0] == 0) {
    flag = false;
  }

  // If statements to determine which characteristic was written to
  if (characteristic.uuid() == accelEnabledCharacteristic.uuid()) {
    sensors.getAccelerometer(flag);
    
    if(flag){  Serial.println("Accelerometer enabled!"); }
    else{  Serial.println("Accelerometer disabled!"); }
    
  }else if (characteristic.uuid() == gyroEnabledCharacteristic.uuid()) {
    sensors.getGyrometer(flag);
    
    if(flag){  Serial.println("Gyrometer enabled!"); }
    else{  Serial.println("Gyrometer disabled!"); }
    
  }else if (characteristic.uuid() == magEnabledCharacteristic.uuid()) {
    sensors.getMagnetometer(flag);
    
    if(flag){  Serial.println("Magnetometer enabled!"); }
    else{  Serial.println("Magnetometer disabled!"); }
    
  }
}


// Function called when a imuSetup char is written eg (accelEnabledCharacteristic)
// The function changes what data should be read/sent via BLE
void setBleAdvertisingCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  
  if (characteristic.value()[0] == 0) {
    // If value written is a 0 than stop ble advertising
    BLE.stopAdvertise();
    ble_advertising = false;
  }else{
    // Else start ble advertising
    BLE.advertise();
    ble_advertising = true;
  }

}

void imuGyroOffsetCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  float float_value = *((float*)(characteristic.value()));

  if (characteristic.uuid() == imu1GyroXOffsetCharacteristic.uuid()) {
    sensors.imu1GyroOffsets.x = float_value;
    Serial.print("Set IMU 1 Gyro X Offset to: ");
  } else if (characteristic.uuid() == imu1GyroYOffsetCharacteristic.uuid()) {
    sensors.imu1GyroOffsets.y = float_value;
    Serial.print("Set IMU 1 Gyro Y Offset to: ");
  } else if (characteristic.uuid() == imu1GyroZOffsetCharacteristic.uuid()) {
    sensors.imu1GyroOffsets.z = float_value;
    Serial.print("Set IMU 1 Gyro Z Offset to: ");
  } else if (characteristic.uuid() == imu2GyroXOffsetCharacteristic.uuid()) {
    sensors.imu2GyroOffsets.x = float_value;
    Serial.print("Set IMU 2 Gyro X Offset to: ");
  } else if (characteristic.uuid() == imu2GyroYOffsetCharacteristic.uuid()) {
    sensors.imu2GyroOffsets.y = float_value;
    Serial.print("Set IMU 2 Gyro Y Offset to: ");
  } else if (characteristic.uuid() == imu2GyroZOffsetCharacteristic.uuid()) {
    sensors.imu2GyroOffsets.z = float_value;
    Serial.print("Set IMU 2 Gyro Z Offset to: ");
  } else if (characteristic.uuid() == imu3GyroXOffsetCharacteristic.uuid()) {
    sensors.imu3GyroOffsets.x = float_value;
    Serial.print("Set IMU 3 Gyro X Offset to: ");
  } else if (characteristic.uuid() == imu3GyroYOffsetCharacteristic.uuid()) {
    sensors.imu3GyroOffsets.y = float_value;
    Serial.print("Set IMU 3 Gyro Y Offset to: ");
  } else if (characteristic.uuid() == imu3GyroZOffsetCharacteristic.uuid()) {
    sensors.imu3GyroOffsets.z = float_value;
    Serial.print("Set IMU 3 Gyro Z Offset to: ");
  } 
  
  Serial.println(float_value,6);
}


// The crash sequence entered when an error is found.
void crash_sequence() {
  // Reset the RGB LED
  write_rgb_led(0,0,0);

  // A loop that plays 3 quick flashes and then 3 long flashes
  while(1){
    for (unsigned char i = 0; i<=3; i++) {
      write_rgb_led(255,0,0);
      delay(160);
      write_rgb_led(0,0,0);
      delay(160);
    }
    for (unsigned char i = 0; i<=3; i++) {
      write_rgb_led(255,0,0);
      delay(350);
      write_rgb_led(0,0,0);
      delay(350);
    }
  }
}

// Simple function to write the RGB LED on with different amounts of brightness.
// Note: The input takes a range of 0-255. (0 - Off) (255 - Max Brightness)
void write_rgb_led(unsigned char red, unsigned char green, unsigned char blue) {

  if (red > 255) { red = 255; }
  if (green > 255) { green = 255; }
  if (blue > 255) { blue = 255; }
  
  analogWrite(LEDR, 255-red);
  analogWrite(LEDG, 255-green);
  analogWrite(LEDB, 255-blue);
}
