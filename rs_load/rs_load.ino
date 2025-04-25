#include <bluefruit.h>
#include <ble_gap.h>
#include <Wire.h> 
#include "SPI.h"
#include "Adafruit_TinyUSB.h"
#include "utility/adafruit_fifo.h"
#include "LSM6DS3.h"
#include <HX711.h>

/* 
    RS_LOAD
    Bosch BMI270 IMU

    normal mode allows 104 Hz - target for both accel and gryo to start with
    G_HM_MODE bit in CTRL7_G (16h). set to 1 for normal and low power mode
    XL_HM_MODE bit in CTRL6_C (15h). set to 1 for normal and low power mode

*/
//*********************************************************************************
// RS LOAD - external load cell / acceleromter monitoring
// BLE Services and Characteristics
//*********************************************************************************

#define IDENTIFY_DURATION 5000 // 5 seconds
#define LED_PIN LED_BUILTIN 

// IMU Details
#define LSM6DS3_ADDR 0x6A
#define OUTX_L_G      0x22  // Start of gyro data

#define DOUT  2  // Data pin
#define CLK   3  // Clock pin


// not yet used
#define TIMESTAMP_L   0x40  // Lower byte of timestamp
#define TIMESTAMP_H   0x41  // Upper byte of timestamp
#define TIMESTAMP_2   0x42  // Middle byte of timestamp

//Create instance of LSM6DS3Core
LSM6DS3Core myIMU(I2C_MODE, LSM6DS3_ADDR);    //I2C device address 0x6A
HX711 scale; // load cell

// used to check although we are not using buffers
bool _tx_buffered   = false;
Adafruit_FIFO* _tx_fifo  = NULL;
bool streamNotifyEnabled = false;
uint8_t stream_command = 0x00; //default to stop stream - only used in case there is more control given to the device
bool tick_flag = false;
uint32_t timestamp;   // Timestamp
unsigned long previousMillis = 0;  // Last time data was read
const long interval = 10;  // 100 Hz (10 ms interval)
float load = 0;
uint16_t errorsAndWarnings = 0;
int number_of_values = 6; //three axes for accel and gyro

//*********************************************************************************
// RS LOAD - external load cell / acceleromter monitoring
// BLE Services and Characteristics
//*********************************************************************************
#define MEASUREMENT_SERVICE_UUID         "f9300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f9300002-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_IDENTIFY_CHAR_UUID        "f9300004-67d2-4b32-9a68-5f3d93a8b6a5"

// measurement service 
BLEService measurementService = BLEService(MEASUREMENT_SERVICE_UUID); // Custom Sensor Service UUID
BLECharacteristic streamDataChar = BLECharacteristic(MEASUREMENT_NOTIFY_CHAR_UUID); // Notify Characteristic (Sensor Data)
BLECharacteristic identifyChar = BLECharacteristic(SENSOR_IDENTIFY_CHAR_UUID); // Write Characteristic (Identify - Flash LED)

// battery service - uses the standard uuid for battery service
//BLEService batteryService = BLEService(0x180F);  // Battery Service UUID
//BLECharacteristic batteryChar = BLECharacteristic(0x2A19); // Battery Level Characteristic

#define BLE_ADVERTISING_FLAGS (BLE_GAP_ADV_FLAGS_CONNECTABLE | BLE_GAP_ADV_FLAGS_SCANNABLE)

byte enableInt = 0;
bool isStreaming = false;
uint32_t lastSampleTime = 0;
uint8_t batteryLevel = 100; // Example initial battery level (in percent)

// Callback for control characteristic (start/stop streaming)
void controlCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
//void controlCallback(BLECharacteristic *chr, uint8_t *data, uint16_t length) {
    switch (data[0]){
      case 0x01: //stream all
        stream_command = 0x01;
        enableInt = 1;  // andres enable flag
        number_of_values = 7; //default for all axis and mag
        break;
      
      case 0x02: // stream only imu
        stream_command = 0x02;
        number_of_values = 6; //default for all axis and mag
        enableInt = 1;  // andres enable flag
        break;

      case 0x03: // stream only load
        stream_command = 0x03;
        enableInt = 1;  // andres enable flag
        number_of_values = 6; //default for all axis and mag
        break;
      
      case 0x00:
        stream_command = 0x00;
        enableInt = 0;
        break;

    }
}

// use this to make sure that the settings are correct
void connect_callback(uint16_t conn_handle)
{
    Serial.print("connect callback");
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  // request PHY changed to 2MB
  conn->requestPHY();

  // request to update data length
  conn->requestDataLengthUpdate();
    
  // request mtu exchange
  conn->requestMtuExchange(247);  //dont think this is needed for this as only sending 14 bytes max

  // request connection interval of 7.5 ms
  conn->requestConnectionParameter(6); // in unit of 1.25 -> 12 gives 15ms 
  //conn->requestConnectionParameter(40);

  // delay a bit for all the request to complete
  delay(1000);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

// dont need this but it will ensure that the notification has been set
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) {
  if (chr->uuid == streamDataChar.uuid) {
    streamNotifyEnabled = (cccd_value & BLE_GATT_HVX_NOTIFICATION);

    if (streamNotifyEnabled) {
      Serial.println("Notifications ENABLED");
    } else {
      Serial.println("Notifications DISABLED");
    }
  }
}


// Callback for identify characteristic (flash LED)
void identifyCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
  Serial.println("Identify Callback Triggered");
  if (data[0] == 0x01) {
        uint32_t startMillis = millis();
        
        while (millis() - startMillis < IDENTIFY_DURATION) {
            digitalWrite(LED_PIN, HIGH);  // Turn LED on
            delay(100);                   // Wait for 500 milliseconds (0.5 seconds)
            digitalWrite(LED_PIN, LOW);   // Turn LED off
            delay(100);                   // Wait for 500 milliseconds
        }
        digitalWrite(LED_PIN, HIGH);// ensure led is off
    }
}


//*********************************************************************************
// RS LOAD - external load cell / acceleromter monitoring
// Setup 
//*********************************************************************************
void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    while (!Serial);  // Wait for serial to be ready

    //Call .beginCore() to configure the IMU - low level example
    if (myIMU.beginCore() != 0) {
        Serial.print("\nDevice Error.\n");
    } else {
        Serial.print("\nDevice OK.\n");
    }

    // Initialize HX711
    //scale.begin(DOUT, CLK);
    // Calibrate the load cell if necessary
    //scale.set_scale(2280.f);  // Adjust based on your load cell (calibration factor)
    //scale.tare();  // Zero the scale

    // set up the imu
    // Software Reset
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x01);
    delay(100);// Wait for reset

    // Enable BDU in CTRL3_C (0x12), bit 6
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x44);  // Set BDU and IF_INC, clear BLE


    // --- Disable filters ---
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00); // Disable gyro filters
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x00); // 0x00: Disable low-pass filter

    uint8_t dataToWrite = 0;  //Temporary variable
    dataToWrite = 0; //Start Fresh!
    dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;  // 2,4,8,16 g options
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
    
    dataToWrite = 0; // Start fresh!
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz; // Gyro ODR at 104Hz
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps; // Gyro Full Scale at 500dps

    // Write the gyroscope settings to CTRL2_G
    errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

    // Initialize BLE'
    //Bluefruit.Advertising.clearData();  // Clear old advertisement data
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    Bluefruit.Periph.setConnInterval(6, 12);
    Bluefruit.begin();
    Bluefruit.setName("RS_LOAD");  // should be the advertising name

    measurementService.begin();
    // Start Services & Characteristics
    streamDataChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_WRITE);  // Allows both notify and write
    streamDataChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    streamDataChar.setMaxLen(247);  // Same as setting max mtu, must be done here as connection sets it first
    streamDataChar.setCccdWriteCallback(cccd_callback);  // This still handles client subscribe requests
    streamDataChar.setWriteCallback(controlCallback);  // This handles writes to control the streaming
    streamDataChar.begin();

    identifyChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ); // write only
    identifyChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    identifyChar.setFixedLen(1);
    identifyChar.setWriteCallback(identifyCallback);
    identifyChar.begin();

    // Add Device Address to Advertising Packet
    uint8_t macAddress[6];
    Bluefruit.getAddr(macAddress);  // attempts to include the device address so that they can be uniquely identified
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, macAddress, 6);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);;

    //SPI.begin();
    // SysTick_Config( (F_CPU/sampleRate)*TICK_INTERVAL_MS );
    //SysTick_Config(SystemCoreClock / 100);  // 100 Hz  use this as the 100hz timer potentially
}

//*********************************************************************************
// RS LOAD - external load cell / acceleromter monitoring
// send data functions (UART sim)
//*********************************************************************************

bool notifyEnabled(uint16_t conn_hdl)
{
  return streamDataChar.notifyEnabled(conn_hdl);
}


bool flushTXD(uint16_t conn_hdl)
{
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  VERIFY(conn);

  uint16_t const gatt_mtu = conn->getMtu() - 3;
  uint8_t* ff_data = (uint8_t*) rtos_malloc( gatt_mtu );
  VERIFY(ff_data);

  uint16_t len = _tx_fifo->read(ff_data, gatt_mtu);
  bool result = true;

  if ( len )
  {
    result = streamDataChar.notify(conn_hdl, ff_data, len);
  }

  rtos_free(ff_data);

  return result;
}


size_t writeDataOverBLE(uint16_t conn_hdl, const uint8_t *content, size_t len)
{
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  VERIFY(conn, 0);

  // skip if not enabled
  if ( !notifyEnabled(conn_hdl) ) return 0;

  // notify right away if txd buffered is not enabled
  if ( !(_tx_buffered && _tx_fifo) )
  {
    //Serial.println("notifying right away");
    return streamDataChar.notify(conn_hdl, content, len) ? len : 0;
    //return len;
  }else
  {
    Serial.println("did not notify right away");
    uint16_t written = _tx_fifo->write(content, len);

    // Not up to GATT MTU, notify will be sent later by TXD timer handler
    if ( _tx_fifo->count() < (conn->getMtu() - 3) )
    {
      return len;
    }
    else
    {
      // TX fifo has enough data, send notify right away
      VERIFY( flushTXD(conn_hdl), 0);

      // still more data left, send them all
      if ( written < len )
      {
        VERIFY(streamDataChar.notify(conn_hdl, content+written, len-written), written);
      }

      return len;
    }
  }
}

extern "C"
{
void SysTick_Handler(void)
{
    //if(enableInt == 1){} if enabled set the tick_flag
    tick_flag = true;
  
}
} // extern C


//*********************************************************************************
// RS LOAD - external load cell / acceleromter monitoring
// Main loop
//*********************************************************************************
void loop() {
  //enableInt = 1;
  //stream_command =0x02;

  if (enableInt == 1){
      unsigned long currentMillis = millis();  // Get current time
      if (currentMillis - previousMillis >= interval) {  // Every 10 ms (100 Hz)
          previousMillis = currentMillis;  // Save time of last read

          // imu data buffer
          uint8_t dataBuffer[12];  // 12 bytes: 6 for accel, 6 for gyro, 
          int16_t rawData[number_of_values]; // 3 accel 3 gyro and load
      /*
        case 0x01 // stream all
        case 0x02 // only imu
        case 0x03 // only load
      */

          switch(stream_command){
                case 0x01: // stream all  
                    
                    errorsAndWarnings += myIMU.readRegisterRegion((uint8_t*)dataBuffer, OUTX_L_G, 12); // Read 12 bytes
                    // Read accelerometer data (3 axes)
                    
                    rawData[0] = (int16_t)(dataBuffer[1] << 8 | dataBuffer[0]);
                    rawData[1] = (int16_t)(dataBuffer[3] << 8 | dataBuffer[2]);
                    rawData[2] = (int16_t)(dataBuffer[5] << 8 | dataBuffer[4]);

                    // Read gyroscope data (3 axes)
                    rawData[3] = (int16_t)(dataBuffer[7] << 8 | dataBuffer[6]);
                    rawData[4] = (int16_t)(dataBuffer[9] << 8 | dataBuffer[8]);
                    rawData[5] = (int16_t)(dataBuffer[11] << 8 | dataBuffer[10]);

                    // Store load cell data in rawData (6th, 7th, 8th position)
                    if (scale.is_ready()) {
                      float load = scale.get_units(10);
                      int16_t scaledLoad = (int16_t)(load * 100);  // scale to keep 2 decimal places
                      rawData[6] = scaledLoad;
                    }

                    break; 
                case 0x02:

                  errorsAndWarnings += myIMU.readRegisterRegion((uint8_t*)dataBuffer, OUTX_L_G, 12); // Read 12 bytes

                  // Read gyro data (3 axes)
                  rawData[0] = (int16_t)(dataBuffer[1] << 8 | dataBuffer[0]);
                  rawData[1] = (int16_t)(dataBuffer[3] << 8 | dataBuffer[2]);
                  rawData[2] = (int16_t)(dataBuffer[5] << 8 | dataBuffer[4]);

                  // Read acc data (3 axes)
                  rawData[3] = (int16_t)(dataBuffer[7] << 8 | dataBuffer[6]);
                  rawData[4] = (int16_t)(dataBuffer[9] << 8 | dataBuffer[8]);
                  rawData[5] = (int16_t)(dataBuffer[11] << 8 | dataBuffer[10]);
                  
                  break;
        
        }// end swithc
        // send data out as a notification
        writeDataOverBLE(Bluefruit.connHandle(), (uint8_t*)rawData, sizeof(rawData));
        //below is used when sending multiple packets in a notification - here we are sampling slowly so should be ok as above  
        //writeDataOverBLE(Bluefruit.connHandle(), ptrOutBuffer, OUTATIME * (number_of_values*2));  // 160 bytes = 20 samples * 8 bytes
            
    }// timer 100hz
  }//end enable int if

}// end loop
