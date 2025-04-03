#include <bluefruit.h>

#define PACKET_SIZE 16  // size of the data packet 
#define SAMPLE_RATE 2000  // 2000 Hz vag data rate
#define IDENTIFY_DURATION 5000 // 5 seconds
#define LED_PIN LED_BUILTIN // not sure

// BLE Services and Characteristics

#define MEASUREMENT_SERVICE_UUID         "f8300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f8300002-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_CONTROL_CHAR_UUID         "f8300003-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_IDENTIFY_CHAR_UUID        "f8300004-67d2-4b32-9a68-5f3d93a8b6a5"

// measurement service 
BLEService measurementService = BLEService(MEASUREMENT_SERVICE_UUID); // Custom Sensor Service UUID
BLECharacteristic streamDataChar = BLECharacteristic(MEASUREMENT_NOTIFY_CHAR_UUID); // Notify Characteristic (Sensor Data)
BLECharacteristic controlChar = BLECharacteristic(SENSOR_CONTROL_CHAR_UUID); // Write Characteristic (Start/Stop)
BLECharacteristic identifyChar = BLECharacteristic(SENSOR_IDENTIFY_CHAR_UUID); // Write Characteristic (Identify - Flash LED)

// battery service - uses the standard uuid for battery service
BLEService batteryService = BLEService(0x180F);  // Battery Service UUID
BLECharacteristic batteryChar = BLECharacteristic(0x2A19); // Battery Level Characteristic

#define BLE_ADVERTISING_FLAGS (BLE_GAP_ADV_FLAGS_CONNECTABLE | BLE_GAP_ADV_FLAGS_SCANNABLE)


bool isStreaming = false;
uint32_t lastSampleTime = 0;
uint8_t batteryLevel = 100; // Example initial battery level (in percent)

// Callback for control characteristic (start/stop streaming)
void controlCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
//void controlCallback(BLECharacteristic *chr, uint8_t *data, uint16_t length) {
    if (data[0] == 0x01) {
        isStreaming = true;

        /*
            here the actual code to read the sensor is executed
        */

    } else {
        isStreaming = false;
    }
}

// Callback for identify characteristic (flash LED)
void identifyCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
  Serial.println("Identify Callback Triggered");
//void identifyCallback(BLECharacteristic *chr, uint8_t *data, uint16_t length) {
    if (data[0] == 0x01) {
        uint32_t startMillis = millis();
        
        while (millis() - startMillis < IDENTIFY_DURATION) {
            digitalWrite(LED_PIN, HIGH);  // Turn LED on
            delay(500);                   // Wait for 500 milliseconds (0.5 seconds)
            digitalWrite(LED_PIN, LOW);   // Turn LED off
            delay(500);                   // Wait for 500 milliseconds
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    // Initialize BLE'
    //Bluefruit.Advertising.clearData();  // Clear old advertisement data
    
    Bluefruit.begin();
    Bluefruit.setName("RS_VAG");  // should be the advertising name


    // Start Services & Characteristics
    measurementService.begin();

    streamDataChar.setProperties(CHR_PROPS_NOTIFY);  // sets the char to be notify that a client can subscribe to
    streamDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    streamDataChar.setFixedLen(PACKET_SIZE);
    streamDataChar.begin();

    controlChar.setProperties(CHR_PROPS_WRITE);  // write only
    controlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    controlChar.setFixedLen(1);
    controlChar.setWriteCallback(controlCallback);
    controlChar.begin();

    identifyChar.setProperties(CHR_PROPS_WRITE); // write only
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
  
    
    // Start Advertising -> dont advertise services. they are discoverable but can only be used on connect
    //Bluefruit.Advertising.addService(measurementService);
    //Bluefruit.Advertising.addService(batteryService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.start(0);
    //Bluefruit.Advertising.start(0, BLE_ADVERTISING_CONNECTABLE);
}


//simulate some sensor data - taken from the internet
void generateSensorData(uint8_t* data) {
    for (int i = 0; i < 8; i++) {  // 8 int16 values = 16 bytes
        int16_t sensorValue = random(-32768, 32767);  // Simulated sensor data
        data[2 * i] = sensorValue & 0xFF;  // Little-endian format
        data[2 * i + 1] = (sensorValue >> 8) & 0xFF;
    }
}

void loop() {
    if (isStreaming && streamDataChar.notifyEnabled()) {
        uint32_t now = micros();
        if (now - lastSampleTime >= (1000000 / SAMPLE_RATE)) { // Every 0.5 ms
            lastSampleTime = now;

            uint8_t data[PACKET_SIZE];
            generateSensorData(data);  
            streamDataChar.notify(data, PACKET_SIZE);
        }
    }

    // Periodically notify battery level
    static uint32_t lastBatteryUpdate = 0;
    if (millis() - lastBatteryUpdate > 1000) { // Update every second
        lastBatteryUpdate = millis();

        batteryLevel = (batteryLevel > 0) ? batteryLevel - 1 : 100; // Simulate battery drain
        batteryChar.notify(&batteryLevel, sizeof(batteryLevel)); // Notify the client
    }
}
