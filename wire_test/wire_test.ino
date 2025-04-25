#include <Wire.h>
#include <LSM6DS3.h>

uint16_t errorsAndWarnings = 0;

// Create LSM6DS3 object
LSM6DS3Core myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

// Data buffer for both accelerometer and gyroscope data
int16_t imuData[6]; // 3 accelerometer and 3 gyroscope data points

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for the serial monitor to be ready
  
  Serial.println("Starting LSM6DS3 sensor...");
  
  // Call .beginCore() to configure the IMU
  if (myIMU.beginCore() != 0) {
      Serial.print("\nDevice Error.\n");
  } else {
      Serial.print("\nDevice OK.\n");
  }
  
  uint8_t dataToWrite = 0;  // Temporary variable

  // Setup the accelerometer and gyroscope
  dataToWrite = 0; // Start fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
  
  // Write the patched together data
  errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set the ODR bit for both accelerometer and gyroscope
  errorsAndWarnings += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  dataToWrite = 0; // Start fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz; // Gyro ODR at 104Hz
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps; // Gyro Full Scale at 500dps

  // Write the gyroscope settings to CTRL2_G
  errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

  Serial.println("Sensor initialized. Starting data acquisition at 100 Hz...");
}

void loop() {
    // Read both accelerometer and gyroscope data in one call
    // Accelerometer data starts at LSM6DS3_ACC_GYRO_OUTX_L_XL, and gyroscope data starts at LSM6DS3_ACC_GYRO_OUTX_L_G
    // Since both are contiguous, we can read them together starting at LSM6DS3_ACC_GYRO_OUTX_L_XL (0x28)
    errorsAndWarnings += myIMU.readRegisterRegion((uint8_t*)imuData, LSM6DS3_ACC_GYRO_OUTX_L_G, 12); // Read 12 bytes

    // Print the accelerometer data (in counts)
    Serial.print("\nAccelerometer Counts:\n");
    Serial.print(" X = ");
    Serial.println(imuData[0]); // X-axis accelerometer data
    Serial.print(" Y = ");
    Serial.println(imuData[1]); // Y-axis accelerometer data
    Serial.print(" Z = ");
    Serial.println(imuData[2]); // Z-axis accelerometer data

    // Print the gyroscope data (in counts)
    Serial.print("\nGyroscope Counts:\n");
    Serial.print(" X = ");
    Serial.println(imuData[3]); // X-axis gyroscope data
    Serial.print(" Y = ");
    Serial.println(imuData[4]); // Y-axis gyroscope data
    Serial.print(" Z = ");
    Serial.println(imuData[5]); // Z-axis gyroscope data

    // Print total errors/warnings
    Serial.println();
    Serial.print("Total reported Errors and Warnings: ");
    Serial.println(errorsAndWarnings);
  
    delay(10); // Delay for 10ms to achieve 100Hz update rate
}
