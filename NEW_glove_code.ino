// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <bluefruit.h>
#include <math.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
#include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
//#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset

// BLE service and characteristics
BLEUart bleuart; // BLE UART service

// Button on glove
#define BUTTON_PIN 7

float offst_x = 0, offst_y = 0, offst_z = 0;
float agx, agy, agz; // Global frame accelerations
float ax, ay, az; // raw accelerometer data
float vx = 0, vy = 0, vz = 0; // Velocity
float px = 0, py = 0, pz = 0; // Position
unsigned long lastUpdate = 0;
float xdat[40]; // 10 sets of 4 integer data from accelerometer_x-axis
float ydat[40]; // 10 sets of 4 integer data from accelerometer_y-axis
float zdat[40]; // 10 sets of 4 integer data from accelerometer_z-axis
int cval = 0;

// Detection of EEPROM or SDFat storage
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal; // For nRF52840 sense etc.
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  // Initialize Bluefruit
  Bluefruit.begin();
  Bluefruit.setName("Feather nRF52840 Sense");

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz

  //##################################################################################
  // Set up and start BLE UART service
  bleuart.begin();
  
  // Start advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising

  Serial.println("Bluetooth device active, waiting for connections...");
  //##################################################################################
  // Initialize the button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Use the internal pull-up resistor
}

// Callback for received data
void bleuart_rx_callback(uint16_t conn_handle) {
  uint8_t buffer[20];
  size_t len = bleuart.read(buffer, sizeof(buffer));
  
  Serial.print("Received: ");
  Serial.write(buffer, len);
  Serial.println();
}

void loop() {
  float roll, pitch, yaw;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;

  // Compute delta time
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0; //DeltaTime in sec
  lastUpdate = currentTime;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

  // get yaw, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  // Convert accelerometer readings from local to global frame using the orientation || Rotation Matrix ||


  //agx = ax * cos(pitch) * cos(yaw) + ay * (sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw)) + az * (cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw)) - offst_x;
  //agy = ax * cos(pitch) * sin(yaw) + ay * (sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw)) + az * (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw)) - offst_y;
  //agz = -ax * sin(pitch) + ay * sin(roll) * cos(pitch) + az * cos(roll) * cos(pitch) - offst_z;

  //agx = -ax - ax*cos((M_PI/180)*pitch) - az*sin((M_PI/180)*pitch) - ax*cos((M_PI/180)*yaw) + ay*sin((M_PI/180)*yaw); //try switching radians/degrees
  //agy = -ay*cos((M_PI/180)*roll) + az*sin((M_PI/180)*roll) - ay - ax*sin((M_PI/180)*yaw) - ay*cos((M_PI/180)*yaw); //try switching radians/degrees
  //agz = -ay*sin((M_PI/180)*roll) - az*cos((M_PI/180)*roll) + ax*sin((M_PI/180)*pitch) - az*cos((M_PI/180)*pitch) - az; //try switching radians/degrees

  // Check button state
  bool buttonPressed = !digitalRead(BUTTON_PIN);
  if (buttonPressed) {
    Serial.println("Lay glove flat on table");
    cval = 0;
    while (abs(accel.acceleration.x) + abs(accel.acceleration.y) >= 1) {
      accelerometer->getEvent(&accel);
      Serial.println(abs(accel.acceleration.x) + abs(accel.acceleration.y));
      delay(100);
    }
    while (cval < 10) {
      accelerometer->getEvent(&accel);
      zdat[cval] = accel.acceleration.z - 9.81;
      delay(50);
      cval += 1;
      offst_z += accel.acceleration.z - 9.81;
    }
    offst_z = offst_z/10;
    Serial.print("offst_z: ");
    Serial.println(offst_z);
    delay(1000);

    Serial.println("rotate the glove 90 degrees clockwise");
    cval = 0;
    while (abs(accel.acceleration.x) + abs(accel.acceleration.z) >= 1) {
      accelerometer->getEvent(&accel);
      Serial.println(abs(accel.acceleration.x) + abs(accel.acceleration.z));
      Serial.println(abs(accel.acceleration.x) + abs(accel.acceleration.z) + abs(accel.acceleration.y));
      delay(100);
    }
    while (cval < 10) {
      accelerometer->getEvent(&accel);
      Serial.println(abs(accel.acceleration.x) + abs(accel.acceleration.z) + abs(accel.acceleration.y));
      ydat[cval] = accel.acceleration.y - 9.81;
      offst_y += accel.acceleration.y - 9.81;
      delay(50);
      cval += 1;
      Serial.println(cval);
    }
    offst_y = offst_y/10;
    delay(1000);

    Serial.println("put the glove fingers facing up");
    cval = 0;
    while (abs(accel.acceleration.z) + abs(accel.acceleration.y) >= 1) {
      accelerometer->getEvent(&accel);
      Serial.println(abs(accel.acceleration.z) + abs(accel.acceleration.y));
      delay(100);
    }
    while (cval < 10) {
      accelerometer->getEvent(&accel);
      xdat[cval] = accel.acceleration.x - 9.81;
      delay(50);
      cval += 1;
      offst_x += accel.acceleration.x - 9.81;
    }
    offst_x = offst_x/10;
    delay(1000);
  }
  // Subtract gravity from the global Z acceleration
  //agz += 9.81; // since rotated to global plane this is valid
  // Integrate acceleration to get velocity
  //Serial.print("ac X: "); Serial.print(ax); Serial.print(" m/s^2, ");
  //Serial.print("ac Y: "); Serial.print(ay); Serial.print(" m/s^2, ");
  //Serial.print("ac Z: "); Serial.print(az); Serial.println(" m/s^2");
  //Serial.print("Rotated: acc X: "); Serial.print(agx); Serial.print(" m/s^2, ");
  //Serial.print("acc Y: "); Serial.print(agy); Serial.print(" m/s^2, ");
  //Serial.print("acc Z: "); Serial.print(agz); Serial.println(" m/s^2");
  //vx += agx * dt;
  //vy += agy * dt;
  //vz += agz * dt;

  // Integrate velocity to get displacement
  //px += vx * dt;
  //py += vy * dt;
  //pz += vz * dt;
  // Printing displacement
  //Serial.print("Position X: "); Serial.print(px); Serial.print(" m, ");
  //Serial.print("Position Y: "); Serial.print(py); Serial.print(" m, ");
  //Serial.print("Position Z: "); Serial.print(pz); Serial.println(" m");

  // print the yaw, pitch and roll
  Serial.print("Orientation: ");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4);  

  // Calculate gravity vector from quaternion
  //float gravity_x = 2 * (qx * qz - qw * qy);
  //float gravity_y = 2 * (qw * qx + qy * qz);
  //float gravity_z = qw * qw - qx * qx - qy * qy + qz * qz;

  // Remove gravity component from accelerometer data
  //float linear_accel_x = ax - gravity_x;//*9.81;
  //float linear_accel_y = ay - gravity_y;//*9.81;
  //float linear_accel_z = az - gravity_z;//*9.81;

  // Output the linear acceleration
  //Serial.print("Linear Acceleration X: "); Serial.println(linear_accel_x);
  //Serial.print("Linear Acceleration Y: "); Serial.println(linear_accel_y);
  //Serial.print("Linear Acceleration Z: "); Serial.println(linear_accel_z);

  // Transform the linear acceleration to the global frame using the quaternion
  float global_accel_x = (1 - 2*qy*qy - 2*qz*qz) * ax + 2*(qx*qy - qw*qz) * ay + 2*(qx*qz + qw*qy) * az;
  float global_accel_y = 2*(qx*qy + qw*qz) * ax + (1 - 2*qx*qx - 2*qz*qz) * ay + 2*(qy*qz - qw*qx) * az;
  float global_accel_z = 2*(qx*qz - qw*qy) * ax + 2*(qy*qz + qw*qx) * ay + (1 - 2*qx*qx - 2*qy*qy) * az - 9.81;

  // Output the global acceleration
  Serial.print("Global Acceleration X: "); Serial.println(global_accel_x);
  Serial.print("Global Acceleration Y: "); Serial.println(global_accel_y);
  Serial.print("Global Acceleration Z: "); Serial.println(global_accel_z);

  // Transmit data via BLE UART
  char tempStr[64];
  snprintf(tempStr, sizeof(tempStr), "Accel: sum=%.2f", abs(accel.acceleration.x) + abs(accel.acceleration.y) + abs(accel.acceleration.z));
  bleuart.print(tempStr);

  // Call the RX callback to check for received data
  //bleuart_rx_callback(bleuart.connHandle());
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif
  delay(50);
}