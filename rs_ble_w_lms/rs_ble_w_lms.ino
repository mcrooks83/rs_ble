#include <bluefruit.h>
#include <ble_gap.h>
#include <Wire.h> 
#include "SPI.h"
//#include "SdFat_Adafruit_Fork.h"
//#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"

// for flashTransport definition
//#include "flash_config.h"

#include "utility/adafruit_fifo.h"

//BLE settings
#define IDENTIFY_DURATION 5000 // 5 seconds
#define LED_PIN LED_BUILTIN // not sure

// BLE Services and Characteristics
#define MEASUREMENT_SERVICE_UUID         "f8300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f8300002-67d2-4b32-9a68-5f3d93a8b6a5"
//#define SENSOR_CONTROL_CHAR_UUID         "f8300003-67d2-4b32-9a68-5f3d93a8b6a5" # maybe use this for axis selection
#define SENSOR_IDENTIFY_CHAR_UUID        "f8300004-67d2-4b32-9a68-5f3d93a8b6a5"

//an option may be to select an axis via a characteristic in this service.

// measurement service 
BLEService measurementService = BLEService(MEASUREMENT_SERVICE_UUID); // Custom Sensor Service UUID
BLECharacteristic streamDataChar = BLECharacteristic(MEASUREMENT_NOTIFY_CHAR_UUID); // Notify Characteristic (Sensor Data)
//BLECharacteristic controlChar = BLECharacteristic(SENSOR_CONTROL_CHAR_UUID); // Write Characteristic (Start/Stop)
BLECharacteristic identifyChar = BLECharacteristic(SENSOR_IDENTIFY_CHAR_UUID); // Write Characteristic (Identify - Flash LED)

// battery service - uses the standard uuid for battery service
BLEService batteryService = BLEService(0x180F);  // Battery Service UUID
BLECharacteristic batteryChar = BLECharacteristic(0x2A19); // Battery Level Characteristic

#define TICK_INTERVAL_MS    1
BLEDis bledis; // device information
//BLEUart bleuart;

uint32_t sampleRate = 6144; // hZ internal sample rate (I think)
byte avgTime = 0;
int32_t  avgValueX = 0;
int32_t  avgValueY = 0;
int32_t  avgValueZ = 0;
byte enableInt = 0;  // this could be the same as isStreaming
const int cSelect2 = 7;
const uint16_t bufferSize = 16000;

// acceleration data buffers
int16_t   accelerationx[bufferSize];
int16_t   accelerationy[bufferSize];
int16_t   accelerationz[bufferSize];
int16_t   acceleration_magnitude[bufferSize];

int16_t  spiBuffer[3];
uint8_t * ptrspiBuffer = (uint8_t *) &spiBuffer;
uint16_t dBufferIn = 0;
uint16_t dBufferOut = 0;


uint32_t lastSampleTime = 0;
uint8_t batteryLevel = 100; // Example initial battery level (in percent)
bool _tx_buffered   = false;
Adafruit_FIFO* _tx_fifo  = NULL;

bool streamNotifyEnabled = false;

//*******************************************************************************************************************
//Interrupt code. This runs periodically, set by sampleRate variable
//const uint16_t OUTATIME = 80;  // Define OUTATIME (number of samples per notification)

//*******************************************************************************************************************
//Interrupt code. This runs periodically, set by sampleRate variable
extern "C"
{
void SysTick_Handler(void)
{

  if(enableInt == 1){
  //LSM6DSO32 read - consider wrapping this into a read accleromter function
  digitalWrite(cSelect2, LOW);  //low selects the chip
  
    SPI.transfer(0xA8);
    
    ptrspiBuffer[0] = SPI.transfer(0x00);
    ptrspiBuffer[1] = SPI.transfer(0x00);
    ptrspiBuffer[2] = SPI.transfer(0x00);
    ptrspiBuffer[3] = SPI.transfer(0x00);
    ptrspiBuffer[4] = SPI.transfer(0x00);
    ptrspiBuffer[5] = SPI.transfer(0x00);

  digitalWrite(cSelect2, HIGH);   

    //timeBuffer[dBufferIn] = timeForBuffer;


    //Take average of 3 readings

    avgValueX = avgValueX + spiBuffer[0];
    avgValueY = avgValueY + spiBuffer[1];
    avgValueZ = avgValueZ + spiBuffer[2];

    avgTime++;
    
    if (avgTime == 3){
      avgTime = 0;
      
      avgValueX = avgValueX / 3;
      avgValueY = avgValueY / 3;
      avgValueZ = avgValueZ / 3;

      acceleration_magnitude[dBufferIn] = sqrt(
          avgValueX * avgValueX +
          avgValueY * avgValueY +
          avgValueZ * avgValueZ
      );
      accelerationx[dBufferIn] = avgValueX;
      accelerationy[dBufferIn] = avgValueY;
      accelerationz[dBufferIn] = avgValueZ;
      
      avgValueX = 0;
      avgValueY = 0;
      avgValueZ = 0;

      dBufferIn++;
      if (dBufferIn == bufferSize){dBufferIn = 0;}
      
    }
    
  }
}
} // extern C


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
// Callback for control characteristic (start/stop streaming)
void controlCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
    Serial.println("control Callback Triggered: ");
    for (uint16_t i = 0; i < len; i++) {
        Serial.print(data[i], HEX); // or DEC/BIN depending on what you want
        Serial.print(" ");
    }
    Serial.println();

    if (data[0] == 0x01) {
        enableInt = 1;  // andres enable flag

    } else if (data[0] == 0x00) {
        enableInt = 0;
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
        digitalWrite(LED_PIN, HIGH); 
    }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
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
  conn->requestMtuExchange(247);

  // request connection interval of 7.5 ms
  conn->requestConnectionParameter(6); // in unit of 1.25 -> 12 gives 15ms 
  //conn->requestConnectionParameter(40);

  // delay a bit for all the request to complete
  delay(1000);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    //set pin as an output
    pinMode ( cSelect2, OUTPUT );
    // set acceleromter chip high to deselect
    digitalWrite ( cSelect2, 1 );

    // on board led to indentify with
    pinMode(LED_PIN, OUTPUT);

    // Initialize BLE'
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    Bluefruit.Periph.setConnInterval(6, 12);
    //Bluefruit.Periph.setConnInterval(20, 40);

    Bluefruit.begin();
    Bluefruit.setName("RS_VAG");  // should be the advertising name
    //bledis.setManufacturer("Right Step");
    //bledis.setModel("RS VAG V1.0.0");

    // Start Services & Characteristics
    measurementService.begin();

    // Add TXD Characteristic

    //streamDataChar.setProperties(CHR_PROPS_NOTIFY);  // sets the char to be notify that a client can subscribe to
    //streamDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    //streamDataChar.setMaxLen( 247 );  //same as setting max mtu but must be done here as connection sets it in the first place
    //.setFixedLen(247);
    //streamDataChar.setUserDescriptor("vag_data");
    //streamDataChar.setCccdWriteCallback(cccd_callback);
    //streamDataChar.begin();
    

    //controlChar.setProperties(CHR_PROPS_WRITE);  // write only
    //controlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    //controlChar.setFixedLen(1);
    //controlChar.setWriteCallback(controlCallback);
    //controlChar.begin();

    //combine the notify and write
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

    // Setup Battery Service
    batteryService.begin();
    batteryChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    batteryChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    batteryChar.setFixedLen(1);
    batteryChar.begin();

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
    Bluefruit.Advertising.start(0);
    

    // not sure what this does?
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }
    
    analogReference(AR_INTERNAL);
    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    //start SPI bus
    SPI.begin();

    //Software reset LSM6DSO32
    digitalWrite(cSelect2, LOW);
        SPI.transfer(0x12); //CTRL3_C register address
        SPI.transfer(0x05);
    digitalWrite(cSelect2, HIGH); 

    //set up the multi-byte read and register sync
    digitalWrite(cSelect2, LOW);
      SPI.transfer(0x12);
      SPI.transfer(0x44); // BDU = 1, IF_INC = 1
    digitalWrite(cSelect2, HIGH);

    delay(100);

    //sets ODR etc
    digitalWrite(cSelect2, LOW);
        SPI.transfer(0x10); //CTRL1_XL
        SPI.transfer(0xA0); //6.66 kHz ±4 g
    digitalWrite(cSelect2, HIGH); 

    digitalWrite(cSelect2, LOW);
        SPI.transfer(0x17); //CTRL8_XL (17h)
        //Low-pass filter settings
        //SPI.transfer(0x00); //odr/4
        //SPI.transfer(0x20); //odr/10
        //SPI.transfer(0x40); //odr/20
        //SPI.transfer(0x60); //odr/45
        //SPI.transfer(0x80); //odr/100
        //SPI.transfer(0xA0); //odr/200
        //SPI.transfer(0xC0); //odr/400
        SPI.transfer(0xE0); //odr/800
    digitalWrite(cSelect2, HIGH); 

    SysTick_Config( (F_CPU/sampleRate)*TICK_INTERVAL_MS );

    //bufferTXD(true);
}



void bufferTXD(bool enable)
{
  _tx_buffered = enable;

  if ( enable )
  {
    // Create FIFO for TXD
    if ( _tx_fifo == NULL )
    {
      _tx_fifo = new Adafruit_FIFO(1);
      _tx_fifo->begin( Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH) );
    }
  }else
  {
    if ( _tx_fifo ) delete _tx_fifo;
  }
}

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

void loop() {

    if (enableInt==1) {  //adding the is notification enabled check screws it up
        //uint16_t OUTATIME = 80;

        // connection interval 12 (15ms) -> now lowered to 6
        uint16_t OUTATIME = 10;  //works at 20, 30 and 40 - 10 crashes at 15ms but works at 7.5
        int number_of_values = 4; // x, y,z and mag
        uint16_t Xout = dBufferIn / OUTATIME;
        if( Xout * OUTATIME != dBufferOut){
        
            //int16_t outBuffer[OUTATIME];
            int16_t outBuffer[OUTATIME * 3];  //3 axis
            uint8_t * ptrOutBuffer = (uint8_t *) &outBuffer;
        
            for(uint16_t i = 0; i < OUTATIME; i++){
                //outBuffer[i] = acceleration_magnitude[dBufferOut + i];
                outBuffer[i*number_of_values + 0] = accelerationx[dBufferOut + i];
                outBuffer[i*number_of_values + 1] = accelerationy[dBufferOut + i];
                outBuffer[i*number_of_values + 2] = accelerationz[dBufferOut + i];
                outBuffer[i*number_of_values + 3] = acceleration_magnitude[dBufferOut + i];
            }

            // taken from the ble uart service
            //writeDataOverBLE(Bluefruit.connHandle(), ptrOutBuffer, OUTATIME * 2);  // OUTATIME * 2 for 2 bytes per int16_t
            writeDataOverBLE(Bluefruit.connHandle(), ptrOutBuffer, OUTATIME * (number_of_values*2));  // 240 bytes = 40 samples * 6 bytes
            
            // Update the buffer out index
            dBufferOut = dBufferOut + OUTATIME;
            if (dBufferOut == bufferSize){dBufferOut = 0;}
        }
    }
}
