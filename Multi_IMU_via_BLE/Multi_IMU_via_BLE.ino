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

#include <ArduinoBLE.h>         // Library for built-in BLE module
#include <Arduino_LSM9DS1.h>    // Library for Nano built-in IMU
#include <Adafruit_LSM6DS33.h>  // Library for external IMU accel and gyro chip
#include <Adafruit_LIS3MDL.h>   // Library for external IMU magnetometer chip
#include <Adafruit_Sensor.h>

/**
 * imu2_ag and imu2_m are objects for the 2nd IMU sensor (found on the upper arm).
 * imu3_ag and imu3_m are objects for the 2nd IMU sensor (found on the lower arm).
 * 
 * imuX_ag represents the IMUs Accel and Gyro chip -> Adafruit_LSM6DS33
 * imuX_m represents the IMUs Mag chip -> Adafruit_LIS3MDL
 */
Adafruit_LSM6DS33 imu_2_ag;
Adafruit_LIS3MDL  imu_2_m;
Adafruit_LSM6DS33 imu_3_ag;
Adafruit_LIS3MDL  imu_3_m;

 // BLE 9-DoF IMU Service
BLEService imuService("C3D0");

// BLE IMU Characteristic list
BLEBoolCharacteristic         startImuCharacteristic            ("49E8", BLERead | BLEWriteWithoutResponse | BLEWrite);   // 0 (False) or 1 (True), to start recording data
BLEUnsignedCharCharacteristic setReadingPrecisionCharacteristic ("98B2", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The number of units after the decimal point
BLEUnsignedCharCharacteristic setReadingFrequencyCharacteristic ("AE89", BLERead | BLEWriteWithoutResponse | BLEWrite);   // The frequency of posting results to BLE in Hz
BLEBoolCharacteristic         accelEnabledCharacteristic        ("1A55", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether acceleration is being tracked
BLEBoolCharacteristic         gyroEnabledCharacteristic         ("3901", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether gyroscope is being tracked
BLEBoolCharacteristic         magEnabledCharacteristic          ("552B", BLERead | BLEWriteWithoutResponse | BLEWrite);   // Whether magnetometer is being tracked
BLEStringCharacteristic       imuStringCharacteristic           ("785F", BLERead | BLENotify, 1000);

/**
 * A data struct to hold 3 D.o.F values in variables x, y, and z.
 * example useage:
 *    xyzStruct accel_reading;
 *    accel_reading.x = 9.81;
 *    Serial.println(accel_reading.x);
 */
struct xyzStruct {
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
struct imuStruct {
  xyzStruct accel;
  xyzStruct gyro;
  xyzStruct mag;
};

// Struct variables to hold the imu data
imuStruct imu_1_data;
imuStruct imu_2_data;
imuStruct imu_3_data;

volatile long previousMillis = 0;      // Variable used within the main loop timer
volatile bool imu_read = false;        // Whether to read and send IMU data or not
bool ble_connected = false;   // Whether a BLE device is connected or not

volatile bool get_accel = true;   // Whether data should be collected for accelerometers
volatile bool get_gyro = true;    // Whether data should be collected for gyroscopes
volatile bool get_mag = false;    // Whether data should be collected for magnetometers

// The delimeter used in the data string passed via BLE
String delim = ",";

// Adjustable variables to adjust the imu recording
volatile unsigned char dec_precision = 3;    // Number pf digits after the decimal point
volatile unsigned char imu_frequency = 10;   // Frequency that results are recorded (Hz)

// Function definitions. These can be found later on in the file
void init_ext_imu_ag(Adafruit_LSM6DS33 imu);  //Function to initialise external accel & gyro chip
void init_ext_imu_m(Adafruit_LIS3MDL imu);    //Function to initialise external magnetometer chip
void write_rgb_led();
void crash_sequence();


void setup() {
  // initialize serial communication
  // if no serial is connected it will continue anyway
  Serial.begin(250000);
  unsigned char counter = 0;
  while(!Serial){
    delay(50);
    counter++;
    if (counter>=13){ break; }
  }
  
  Serial.println("Initialising Arduino for IMU and Bluetooth...");

  // Initialise the 4 LED peripherals attached to the Arduino board
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  /**
   * The below is peripheral setup for all the IMUs
   * If it fails to setup any peripheral then an endless crash loop is entered flashing a warning LED.
   * For the program to work all of the peripherals must be correctly plugged in!
   */
  if (!IMU.begin()) { Serial.println("Failed to initalise IMU 1...");                                   crash_sequence(); }
  if (!imu_2_ag.begin_I2C(0x6A)) { Serial.println("Failed to find external IMU 2 accel&gyro chip...");  crash_sequence(); }
  if (!imu_3_ag.begin_I2C(0x6B)) { Serial.println("Failed to find external IMU 3 accel&gyro chip...");  crash_sequence(); }
  if (!imu_2_m.begin_I2C(0x1C)) { Serial.println("Failed to find external IMU 2 mag chip...");          crash_sequence(); }
  if (!imu_3_m.begin_I2C(0x1E)) { Serial.println("Failed to find external IMU 3 mag chip...");          crash_sequence(); }

  // Setup accel, gyro and mag data rate and range for external IMUs
  init_ext_imu_ag(imu_2_ag);
  init_ext_imu_m(imu_2_m);
  init_ext_imu_ag(imu_3_ag);
  init_ext_imu_m(imu_3_m);


  // >> Start of BLE Setup

  // Initialise the BLE module found on the Arduino board
  if (!BLE.begin()) { Serial.println("starting BLE failed!"); crash_sequence(); }

  BLE.setLocalName("LeedsMechEngWearable");  // The name that will be visible when searching for the device
  BLE.setAdvertisedService(imuService);      // Advertise the service UUID so that the service can be accessed

  // Add the characteristics to the service
  imuService.addCharacteristic(startImuCharacteristic);
  imuService.addCharacteristic(imuStringCharacteristic);
  imuService.addCharacteristic(setReadingPrecisionCharacteristic);
  imuService.addCharacteristic(setReadingFrequencyCharacteristic);
  imuService.addCharacteristic(accelEnabledCharacteristic);
  imuService.addCharacteristic(gyroEnabledCharacteristic);
  imuService.addCharacteristic(magEnabledCharacteristic);

  // Add the imu service
  BLE.addService(imuService); 

  // Set initial values for characteristics
  startImuCharacteristic.writeValue(imu_read);
  imuStringCharacteristic.writeValue("");
  setReadingPrecisionCharacteristic.writeValue(dec_precision);
  setReadingFrequencyCharacteristic.writeValue(imu_frequency);
  accelEnabledCharacteristic.writeValue(get_accel);
  gyroEnabledCharacteristic.writeValue(get_gyro);
  magEnabledCharacteristic.writeValue(get_mag);

  // Set callback functions to be called when an event such as a BLE device connects occurs
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Set callback functions to be called when a characteristics is written too
  startImuCharacteristic.setEventHandler(BLEWritten, imuStartCharacteristicWritten);
  setReadingPrecisionCharacteristic.setEventHandler(BLEWritten, imuPrecisionCharacteristicWritten);
  setReadingFrequencyCharacteristic.setEventHandler(BLEWritten, imuFrequencyCharacteristicWritten);
  accelEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  gyroEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);
  magEnabledCharacteristic.setEventHandler(BLEWritten, imuSetupCharacteristicWritten);

  // start advertising the BLE signal
  BLE.advertise();

  // >> End of BLE Setup

  // Signal to show that the device setup correctly
  for (int i = 0; i<=5; i++) {
    write_rgb_led(0,255,0);
    delay(250);
    write_rgb_led(0,0,0);
    delay(250);
  }

  Serial.println("Bluetooth device active, waiting for connection...");
}

// Function called when a BLE device conencts
void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());

  // Set RGB LED to Blue to signal a succesful connection
  write_rgb_led(0,0,255);

  ble_connected = true;

  // Since a device is connected, stop advertising the BLE signal
  BLE.stopAdvertise();
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());

  // Turn the RGB LED off to signal nothing connected
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
      
      long currentMillis = millis();

      // Timer check to so that code is only completed to time of imu_frequency
      if ( (imu_read == true) && (currentMillis - previousMillis >= (1000/imu_frequency)) ) {
        previousMillis = currentMillis;

        // >> Data collection for the Nano's built-in IMU
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

        // >> Data collection for IMU 2
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

        
        // By default no data is printed to the console but for debugging purposes
        //  imu_to_print can be set to print over serial.
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

        /**
         * To send data via BLE the data is condensed into one long string where the very first element is the timestamp
         * e.g. Timestamp,imu1_accelX,imu1_accelY,imu1_accelZ,imu1_gyroX ... imu3_magY,imu3_magZ
         * Note: 'delim' is a variable defined in the heading and is used to seperate the values with a ',' for example.
         */
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

        // Once all of the data is added to the string it is written to the imuString characteristic
        imuStringCharacteristic.writeValue(str);
        
      }
    }
  }else{
    // If nothing is connected an orange light flashes to signal its ready to connect
    write_rgb_led(255,165,0);
    delay(500);
    write_rgb_led(0,0,0);
    delay(500);
  }
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

// Function to initialise the Accel and Gyro chip of an external IMU
void init_ext_imu_ag(Adafruit_LSM6DS33 imu){

  // Accelerometer setup to match that found on the Nano
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  // Gyroscope setup to match that found on the Nano
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
}

// Function to initialise the Mag chip of an external IMU
void init_ext_imu_m(Adafruit_LIS3MDL imu){
  imu.setDataRate(LIS3MDL_DATARATE_20_HZ);
  imu.setRange(LIS3MDL_RANGE_4_GAUSS);
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
