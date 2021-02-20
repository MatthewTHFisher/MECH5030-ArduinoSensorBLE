/*    Author(s): M. Fisher
 *    University of Leeds
 *    
 *    This program is used for the Arduino Nano 33 BLE to record IMU data and transfer
 *    the data via a BLE
 *    
 *    The program has 4 useable characteristics:
 *       1 - Start/Stop IMU recordings
 *       2 - Set the reading frequency of the IMU
 *       3 - Set the number of digits after the decimal point
 *       4 - Send IMU data via a string containing timestamp, accel, gyro, magne
 */

#include <ArduinoBLE.h>         // Library for built-in BLE module
#include <Arduino_LSM9DS1.h>    // Library for Nano built-in IMU
#include <Adafruit_LSM6DS33.h>  // Library for external IMU accel and gyro chip
#include <Adafruit_LIS3MDL.h>   // Library for external IMU magnetometer chip
#include <Adafruit_Sensor.h>

Adafruit_LSM6DS33 imu_2_ag;
Adafruit_LIS3MDL  imu_2_m;
Adafruit_LSM6DS33 imu_3_ag;
Adafruit_LIS3MDL  imu_3_m;

 // BLE 9-DoF IMU Service
BLEService imuService("C3D0");

// BLE Characteristic list
BLEBoolCharacteristic   startImuCharacteristic            ("49E8", BLERead | BLEWriteWithoutResponse | BLEWrite);   // 0 (False) or 1 (True), to start recording data
BLEIntCharacteristic    setReadingPrecisionCharacteristic ("98B2", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The number of units after the decimal point
BLEIntCharacteristic    setReadingFrequencyCharacteristic ("AE89", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The frequency of posting results to BLE in Hz
BLEBoolCharacteristic   accelEnabledCharacteristic        ("1A55", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether acceleration is being tracked
BLEBoolCharacteristic   gyroEnabledCharacteristic         ("3901", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether gyroscope is being tracked
BLEBoolCharacteristic   magEnabledCharacteristic          ("552B", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether magnetometer is being tracked
BLEStringCharacteristic imuStringCharacteristic           ("785F", BLERead | BLENotify, 1000);

BLEDescriptor batteryLevelDescriptor("0x2901", "millis");

struct xyzStruct {
  float x;
  float y;
  float z;
};

struct imuStruct {
  xyzStruct accel;
  xyzStruct gyro;
  xyzStruct mag;
};

// Objects to hold the imu data
imuStruct imu_1_data;
imuStruct imu_2_data;
imuStruct imu_3_data;

long previousMillis = 0;
bool imu_read = false;       // Whether to read and send IMU data or not
bool ble_connected = false;

bool get_accel = true;   // Whether data should be collected for accelerometers
bool get_gyro = true;    // Whether data should be collected for gyroscopes
bool get_mag = false;    // Whether data should be collected for magnetometers

String delim = ",";

// Adjustable variables to adjust the imu recording
int dec_precision = 3;    // Number pf digits after the decimal point
int imu_frequency = 10;   // Frequency that results are recorded (Hz)

// Function definitions. These can be found later on in the file
void init_ext_imu_ag(Adafruit_LSM6DS33 imu);  //Function to initialise external accel & gyro chip
void init_ext_imu_m(Adafruit_LIS3MDL imu);    //Function to initialise external magnetometer chip
void write_rgb_led();
void crash_sequence();


void setup() {
  Serial.begin(250000);    // initialize serial communication
  //while(!Serial) { delay(50); }
  Serial.println("Initialising Arduino for IMU and Bluetooth...");

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  if (!IMU.begin()) { Serial.println("Failed to initalise IMU 1...");                       crash_sequence(); }
  if (!imu_2_ag.begin_I2C(0x6A)) { Serial.println("Failed to find external IMU 2 accel&gyro chip..."); crash_sequence(); }
  if (!imu_3_ag.begin_I2C(0x6B)) { Serial.println("Failed to find external IMU 3 accel&gyro chip..."); crash_sequence(); }
  if (!imu_2_m.begin_I2C(0x1C)) { Serial.println("Failed to find external IMU 2 mag chip..."); crash_sequence(); }
  if (!imu_3_m.begin_I2C(0x1E)) { Serial.println("Failed to find external IMU 3 mag chip..."); crash_sequence(); }
  if (!BLE.begin()) { Serial.println("starting BLE failed!");                             crash_sequence(); }

  // Set accel & gyro for external IMUs
  init_ext_imu_ag(imu_2_ag);
  init_ext_imu_m(imu_2_m);
  init_ext_imu_ag(imu_3_ag);
  init_ext_imu_m(imu_3_m);

  BLE.setLocalName("LeedsMechEngWearable");
  BLE.setAdvertisedService(imuService); // add the service UUID

  startImuCharacteristic.addDescriptor(batteryLevelDescriptor);

  // Add the characteristics to the service
  imuService.addCharacteristic(startImuCharacteristic);
  imuService.addCharacteristic(imuStringCharacteristic);
  imuService.addCharacteristic(setReadingPrecisionCharacteristic);
  imuService.addCharacteristic(setReadingFrequencyCharacteristic);
  imuService.addCharacteristic(accelEnabledCharacteristic);
  imuService.addCharacteristic(gyroEnabledCharacteristic);
  imuService.addCharacteristic(magEnabledCharacteristic);
  
  BLE.addService(imuService); // Add the imu service

  // Set initial values for characteristics
  startImuCharacteristic.writeValue(imu_read);
  imuStringCharacteristic.writeValue("");
  setReadingPrecisionCharacteristic.writeValue(dec_precision);
  setReadingFrequencyCharacteristic.writeValue(imu_frequency);
  accelEnabledCharacteristic.writeValue(get_accel);
  gyroEnabledCharacteristic.writeValue(get_gyro);
  magEnabledCharacteristic.writeValue(get_mag);

  // Set functions to be called when an event occurs
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Set functions to be called when a characteristics are written too
  startImuCharacteristic.setEventHandler(BLEWritten, imuStartCharacteristicWritten);
  setReadingPrecisionCharacteristic.setEventHandler(BLEWritten, imuPrecisionCharacteristicWritten);
  setReadingFrequencyCharacteristic.setEventHandler(BLEWritten, imuFrequencyCharacteristicWritten);
  accelEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  gyroEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  magEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  

  // start advertising the BLE signal
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connection...");

  if (startImuCharacteristic.hasDescriptor("0x2901")) {
    Serial.println("Char has description descriptor...");
  }
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());

  ble_connected = true;

  // Since a device is connected, stop advertising the BLE signal
  BLE.stopAdvertise();
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  
  write_rgb_led(0,0,0);

  ble_connected = false;

  // Since no devices are connected, re-advertise
  BLE.advertise();
}


void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {    
    while (central.connected()) {
      
      write_rgb_led(0,0,255);
      
      long currentMillis = millis();
      
      if ( (imu_read == true) && (currentMillis - previousMillis >= (1000/imu_frequency)) ) {
        previousMillis = currentMillis;

        // data retreival for the Nano's built-in IMU
        if (IMU.accelerationAvailable() && get_accel)  { 
          IMU.readAcceleration(imu_1_data.accel.x, imu_1_data.accel.y, imu_1_data.accel.z); 

          //Onboard IMU-accelerometer requires converting to change from G's into M/S^2
          imu_1_data.accel.x = imu_1_data.accel.x * SENSORS_GRAVITY_STANDARD;
          imu_1_data.accel.y = imu_1_data.accel.y * SENSORS_GRAVITY_STANDARD;
          imu_1_data.accel.z = imu_1_data.accel.z * SENSORS_GRAVITY_STANDARD;
        }
        if (IMU.gyroscopeAvailable() && get_gyro) { 
          IMU.readGyroscope(imu_1_data.gyro.x, imu_1_data.gyro.y, imu_1_data.gyro.z); 
          }
        if (IMU.magneticFieldAvailable() && get_mag) { 
          IMU.readMagneticField(imu_1_data.mag.x, imu_1_data.mag.y, imu_1_data.mag.z); 
          }
          
        // Variables to hold the data of the external IMUs
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        sensors_event_t mag;

        // Data collection for IMU 2
        if (get_accel || get_gyro) {    //Since accel and gyro are on the same chip we must do both together
          imu_2_ag.getEvent(&accel, &gyro, &temp);
          imu_2_data.accel.x = accel.acceleration.x;
          imu_2_data.accel.y = accel.acceleration.y;
          imu_2_data.accel.z = accel.acceleration.z;
          imu_2_data.gyro.x = gyro.gyro.x;
          imu_2_data.gyro.y = gyro.gyro.y;
          imu_2_data.gyro.z = gyro.gyro.z;
        }
        if (get_mag) {    // If we don't need to retrieve mag data we won't waste time doing this
          imu_2_m.getEvent(&mag);
          imu_2_data.mag.x = mag.magnetic.x;
          imu_2_data.mag.y = mag.magnetic.y;
          imu_2_data.mag.z = mag.magnetic.z;
        }

        // Data collection for IMU 3
        if (get_accel || get_gyro) {    //Since accel and gyro are on the same chip we must do both together
          imu_3_ag.getEvent(&accel, &gyro, &temp);
          imu_3_data.accel.x = accel.acceleration.x;
          imu_3_data.accel.y = accel.acceleration.y;
          imu_3_data.accel.z = accel.acceleration.z;
          imu_3_data.gyro.x = gyro.gyro.x;
          imu_3_data.gyro.y = gyro.gyro.y;
          imu_3_data.gyro.z = gyro.gyro.z;
        }
        if (get_mag) {    // If we don't need to retrieve mag data we won't waste time doing this
          imu_3_m.getEvent(&mag);
          imu_3_data.mag.x = mag.magnetic.x;
          imu_3_data.mag.y = mag.magnetic.y;
          imu_3_data.mag.z = mag.magnetic.z;
        }
        
       
        int imu_to_print = 0;
        switch (imu_to_print){
          case 1:
            Serial.println(currentMillis);
            if(get_accel){
              Serial.print(imu_1_data.accel.x, dec_precision); Serial.print(",");
              Serial.print(imu_1_data.accel.y, dec_precision); Serial.print(",");
              Serial.print(imu_1_data.accel.z, dec_precision); Serial.print(",");
            }
            if (get_gyro){
              Serial.print(imu_1_data.gyro.x, dec_precision); Serial.print(",");
              Serial.print(imu_1_data.gyro.y, dec_precision); Serial.print(",");
              Serial.print(imu_1_data.gyro.z, dec_precision); Serial.print(",");
            }
            if (get_mag){
              Serial.print(imu_1_data.mag.x, dec_precision); Serial.print(",");
              Serial.print(imu_1_data.mag.y, dec_precision); Serial.print(",");
              Serial.print(imu_1_data.mag.z, dec_precision); 
            }
            Serial.print("\n");
            break;
          case 2:
            Serial.println(currentMillis);
            if(get_accel){
              Serial.print(imu_2_data.accel.x, dec_precision); Serial.print(",");
              Serial.print(imu_2_data.accel.y, dec_precision); Serial.print(",");
              Serial.print(imu_2_data.accel.z, dec_precision); Serial.print(",");
            }
            if (get_gyro){
              Serial.print(imu_2_data.gyro.x, dec_precision); Serial.print(",");
              Serial.print(imu_2_data.gyro.y, dec_precision); Serial.print(",");
              Serial.print(imu_2_data.gyro.z, dec_precision); Serial.print(",");
            }
            if (get_mag){
              Serial.print(imu_2_data.mag.x, dec_precision); Serial.print(",");
              Serial.print(imu_2_data.mag.y, dec_precision); Serial.print(",");
              Serial.print(imu_2_data.mag.z, dec_precision); 
            }
            Serial.print("\n");
            break;
          case 3:
            Serial.println(currentMillis);
            if(get_accel){
              Serial.print(imu_3_data.accel.x, dec_precision); Serial.print(",");
              Serial.print(imu_3_data.accel.y, dec_precision); Serial.print(",");
              Serial.print(imu_3_data.accel.z, dec_precision); Serial.print(",");
            }
            if (get_gyro){
              Serial.print(imu_3_data.gyro.x, dec_precision); Serial.print(",");
              Serial.print(imu_3_data.gyro.y, dec_precision); Serial.print(",");
              Serial.print(imu_3_data.gyro.z, dec_precision); Serial.print(",");
            }
            if (get_mag){
              Serial.print(imu_3_data.mag.x, dec_precision); Serial.print(",");
              Serial.print(imu_3_data.mag.y, dec_precision); Serial.print(",");
              Serial.print(imu_3_data.mag.z, dec_precision); 
            }
            Serial.print("\n");
            break;
          default:
            break;
        }
        
        String str = String(currentMillis);

        // Adds data for IMU 1 (The onboard imu)
        if(get_accel){ str = str + delim + String(imu_1_data.accel.x, dec_precision) + delim + String(imu_1_data.accel.y, dec_precision) + delim + String(imu_1_data.accel.z, dec_precision); }
        if(get_gyro){ str = str + delim + String(imu_1_data.gyro.x, dec_precision) + delim + String(imu_1_data.gyro.y, dec_precision) + delim + String(imu_1_data.gyro.z, dec_precision); }
        if(get_mag){ str = str + delim + String(imu_1_data.mag.x, dec_precision) + delim + String(imu_1_data.mag.y, dec_precision) + delim + String(imu_1_data.mag.z, dec_precision); }

        // Adds data for IMU 2 
        if(get_accel){ str = str + delim + String(imu_2_data.accel.x, dec_precision) + delim + String(imu_2_data.accel.y, dec_precision) + delim + String(imu_2_data.accel.z, dec_precision); }
        if(get_gyro){ str = str + delim + String(imu_2_data.gyro.x, dec_precision) + delim + String(imu_2_data.gyro.y, dec_precision) + delim + String(imu_2_data.gyro.z, dec_precision); }
        if(get_mag){ str = str + delim + String(imu_2_data.mag.x, dec_precision) + delim + String(imu_2_data.mag.y, dec_precision) + delim + String(imu_2_data.mag.z, dec_precision); }

        // Adds data for IMU 3
        if(get_accel){ str = str + delim + String(imu_3_data.accel.x, dec_precision) + delim + String(imu_3_data.accel.y, dec_precision) + delim + String(imu_3_data.accel.z, dec_precision); }
        if(get_gyro){ str = str + delim + String(imu_3_data.gyro.x, dec_precision) + delim + String(imu_3_data.gyro.y, dec_precision) + delim + String(imu_3_data.gyro.z, dec_precision); }
        if(get_mag){ str = str + delim + String(imu_3_data.mag.x, dec_precision) + delim + String(imu_3_data.mag.y, dec_precision) + delim + String(imu_3_data.mag.z, dec_precision); }
        
        imuStringCharacteristic.writeValue(str);
        
      }
    }
  }else{
    write_rgb_led(255,165,0);
    delay(500);
    write_rgb_led(0,0,0);
    delay(500);
  }
}

void imuStartCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.value()[0] == 0) { 
    Serial.println("Stopping IMU recordings..."); 
    imu_read = false;
  } else { 
    Serial.println("Starting IMU recordings..."); 
    imu_read = true; 
  }
}

void imuPrecisionCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("IMU Precision written: ");
  Serial.println(characteristic.value()[0]);
  dec_precision = characteristic.value()[0];
}

void imuFrequencyCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("IMU Frequency written: ");
  Serial.println(characteristic.value()[0]);
  imu_frequency = characteristic.value()[0];
}

void imuSetupCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  bool flag = true;
  if (characteristic.value()[0] == 0) {
    flag = false;
  }
  
  if (characteristic.uuid() == accelEnabledCharacteristic.uuid()) {
    get_accel = flag;
    if(flag){  Serial.println("Accelerometer enabled!"); }
    else{  Serial.println("Accelerometer disabled!"); }
  }else if (characteristic.uuid() == gyroEnabledCharacteristic.uuid()) {
    get_gyro = flag;
    if(flag){  Serial.println("Gyrometer enabled!"); }
    else{  Serial.println("Gyrometer disabled!"); }
  }else if (characteristic.uuid() == magEnabledCharacteristic.uuid()) {
    get_mag = flag;
    if(flag){  Serial.println("Magnetometer enabled!"); }
    else{  Serial.println("Magnetometer disabled!"); }
  }
}

void init_ext_imu_ag(Adafruit_LSM6DS33 imu){

  // Accelerometer setup to match that found on the Nano
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  // Gyroscope setup to match that found on the Nano
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
}

void init_ext_imu_m(Adafruit_LIS3MDL imu){
  imu.setDataRate(LIS3MDL_DATARATE_20_HZ);
  imu.setRange(LIS3MDL_RANGE_4_GAUSS);
}

void crash_sequence() {
  write_rgb_led(0,0,0);
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }
}

void write_rgb_led(int red, int green, int blue) {

  if (red > 255) { red = 255; }
  if (green > 255) { green = 255; }
  if (blue > 255) { blue = 255; }
  
  analogWrite(LEDR, 255-red);
  analogWrite(LEDG, 255-green);
  analogWrite(LEDB, 255-blue);
}
