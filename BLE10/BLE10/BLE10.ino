
#include <bluefruit.h>
#include <ble_gap.h>
#include <Wire.h> 
#include "SPI.h"
//#include "SdFat_Adafruit_Fork.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"

// for flashTransport definition
#include "flash_config.h"


//must copy Adafruit_nRFCrypto and Bluefruit52Lib from adafruit nrt52 to seeed nrf52 folder


#define expFlashCS D1
// Built from the P25Q16H datasheet.
// https://gitlab.com/arduino5184213/seeed_xiao_nrf52840/flash_speedtest/-/tree/master
SPIFlash_Device_t const P25Q16SU {
  .total_size = (1UL << 21), // 2MiB
  .start_up_time_us = 10000, // Don't know where to find that value
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,
  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02, // Datasheet p. 27
  .has_sector_protection = 1,   // Datasheet p. 27
  .supports_fast_read = 1,      // Datasheet p. 29
  .supports_qspi = 1,           // Obviously
  .supports_qspi_writes = 1,    // Datasheet p. 41
  .write_status_register_split = 1, // Datasheet p. 28
  .single_status_byte = 0,      // 2 bytes
  .is_fram = 0,                 // Flash Memory
};

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

FatFile root;
FatFile file;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Check if flash is formatted
bool fs_formatted = false;

// Set to true when PC write to flash
bool fs_changed = true;;



// Interval between systick event
#define TICK_INTERVAL_MS    1


//BLEDfu  bledfu;  // OTA DFU service
BLEDis bledis; // device information
BLEUart bleuart;
BLEBas  blebas;  // battery

uint32_t sampleRate = 6144; // hZ 
//uint32_t sampleRate = 1024; // hZ 
byte avgTime = 0;
int32_t  avgValueX = 0;
int32_t  avgValueY = 0;
int32_t  avgValueZ = 0;
//const int led2 = 13;
//const int debugPin = 5;
byte enableInt = 0;
const int cSelect2 = 7;

const uint16_t bufferSize = 12000;

//uint32_t  timeForBuffer = 1;
//unsigned long timeBuffer[bufferSize];
//uint8_t * ptrTimeBuffer = (uint8_t *) &timeBuffer;

int16_t   accelerationx[bufferSize];
int16_t   accelerationy[bufferSize];
int16_t   accelerationz[bufferSize];

//int16_t   accelerationx2[bufferSize];
//int16_t   accelerationy2[bufferSize];
//int16_t   accelerationz2[bufferSize];

int16_t  spiBuffer[3];
uint8_t * ptrspiBuffer = (uint8_t *) &spiBuffer;

uint16_t dBufferIn = 0;
uint16_t dBufferOut = 0;

char activeCommand = 'X';
char CommandA = 0;
char CommandB = 0;



//*******************************************************************************************************************
//Interrupt code. This runs periodically, set by sampleRate variable
extern "C"
{
void SysTick_Handler(void)
{

  if(enableInt == 1){
    


  //LSM6DSO32 read
  digitalWrite(cSelect2, LOW);
  
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

      accelerationx[dBufferIn] = avgValueX;
      accelerationy[dBufferIn] = avgValueY;
      accelerationz[dBufferIn] = avgValueZ;
      
      avgValueX = 0;
      avgValueY = 0;
      avgValueZ = 0;

      dBufferIn++;
      if (dBufferIn == bufferSize){dBufferIn = 0;}
      
    }
    
    

//    accelerationx[dBufferIn] = spiBuffer[0];
//    accelerationy[dBufferIn] = spiBuffer[1];
//    accelerationz[dBufferIn] = spiBuffer[2];


    //timeForBuffer++;
    


    //digitalWrite(debugPin, LOW);
    //digitalWrite ( led2, !digitalRead(led2) );
  }
}
} // extern C


//*******************************************************************************************************************
//BLE stuff
void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();


  

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  //Bluefruit.Advertising.addUuid(uuid);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  // request PHY changed to 2MB
  conn->requestPHY();

  // request to update data length
  conn->requestDataLengthUpdate();
    
  // request mtu exchange
  conn->requestMtuExchange(247);

  // request connection interval of 7.5 ms
  //conn->requestConnectionParameter(6); // in unit of 1.25

  // delay a bit for all the request to complete
  delay(1);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;


}
//*******************************************************************************************************************
void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;
  
  //rxLastTime = millis();
  
  //// first packet
  //if ( rxCount == 0 )
  //{
  //  rxStartTime = millis();
  //}

  //uint32_t count = bleuart.available();

  //rxCount += count;
  //bleuart.flush(); // empty rx fifo

  // Serial.printf("RX %d bytes\n", count);
}
////*******************************************************************************************************************
void bleuart_notify_callback(uint16_t conn_hdl, bool enabled)
{
  if ( enabled )
  {
    //Serial.println("Send a key and press enter to start test");
  }
}
//*******************************************************************************************************************
//char* getUserInput(void)
//{
//  static char inputs[64+1];
//  memset(inputs, 0, sizeof(inputs));
//
//  // wait until data is available
//  while( Serial.available() == 0 ) { delay(1); }
//
//  uint8_t count=0;
//  do
//  {
//    count += Serial.readBytes(inputs+count, 64);
//  } while( (count < 64) && Serial.available() );
//
//  return inputs;
//}



//file system stuff
//*******************************************************************************************************************
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and 
// return number of copied bytes (must be multiple of block size) 
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize) {
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize) {


  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void) {
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fs_changed = true;

}
//*******************************************************************************************************************
void batLevel() {
  // 760 = 1v
  uint16_t ADCRead;
  uint8_t batOUT = 100;
  ADCRead = analogRead(32);


    if(ADCRead < 3116){batOUT = 90;}
    if(ADCRead < 3040){batOUT = 80;}
    if(ADCRead < 2964){batOUT = 70;}
    if(ADCRead < 2888){batOUT = 60;}
    if(ADCRead < 2812){batOUT = 50;}
    if(ADCRead < 2736){batOUT = 40;}
    if(ADCRead < 2660){batOUT = 30;}
    if(ADCRead < 2584){batOUT = 20;}
    if(ADCRead < 2508){batOUT = 10;}
    if(ADCRead < 2432){batOUT = 0;}

  
    blebas.write(batOUT);

}
//*******************************************************************************************************************
void setup(void)
{  
  Serial.begin(115200);

  pinMode ( cSelect2, OUTPUT );
  digitalWrite ( cSelect2, 1 );
  
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);

  
  //Serial.begin(460800);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb



  //flash.begin();
  flash.begin(&P25Q16SU, 1);

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size()/512, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  usb_msc.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Init file system on the flash
  fs_formatted = fatfs.begin(&flash);

  //Read config files
  char sensorName[10];
  if (fatfs.exists("name.cfg")) {
    file.open("name.cfg");
    
      
      file.fgets(sensorName, sizeof(sensorName));

    file.close(); 
  }

//  byte filter = 0x0C;
//  if (fatfs.exists("filter.cfg")) {
//    file.open("filter.cfg");
//    
//      char line[20];
//      file.fgets(line, sizeof(line));
//      String row1 = String(line);
//      //row1.toUpperCase();
//      int inChar = row1.toInt();
//      if (inChar != 0){
//        switch (inChar) {
//          case 160: 
//            filter = 0x08;
//            break;
//          case 320:
//            filter = 0x09;
//            break;
//          case 640: 
//            filter = 0x0A;
//            break;
//          case 1280: 
//            filter = 0x0B;
//            break;
//          }
//      }
//    file.close(); 
//  }




  


  
//    // VBAT
//    31, // D32 is P0.31 (VBAT)
//    #define PIN_VBAT                (32)    // Read the BAT voltage.
//    pinMode(VBAT_ENABLE, OUTPUT);
//    digitalWrite(VBAT_ENABLE, HIGH);








  
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12); // 7.5 - 15 ms
  Bluefruit.setName(sensorName);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Your company");
  bledis.setModel("Your device");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  //bleuart.setRxCallback(bleuart_rx_callback);
  //bleuart.setNotifyCallback(bleuart_notify_callback);

  // Set up and start advertising
  startAdv();


  // Set the analog reference 
  analogReference(AR_INTERNAL_1_8);


//  AR_DEFAULT,
//  AR_INTERNAL,          // 0.6V Ref * 6 = 0..3.6V
//  AR_INTERNAL_3_0,      // 0.6V Ref * 5 = 0..3.0V
//  AR_INTERNAL_2_4,      // 0.6V Ref * 4 = 0..2.4V
//  AR_INTERNAL_1_8,      // 0.6V Ref * 3 = 0..1.8V
//  AR_INTERNAL_1_2,      // 0.6V Ref * 2 = 0..1.6V
//  AR_VDD4               // VDD/4 REF * 4 = 0..VDD
  

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  SPI.begin();
  //SPI.beginTransaction(settings);

  

    //Software reset LSM6DSO32
  digitalWrite(cSelect2, LOW);
    SPI.transfer(0x12); //CTRL3_C
    SPI.transfer(0x05);
  digitalWrite(cSelect2, HIGH); 

  delay(100);


//    //Accelerometer ODR selection
//    //Accelerometer full-scale selection
//    //No second low-pass filter
//  digitalWrite(cSelect2, LOW);
//    SPI.transfer(0x10); //CTRL1_XL
//    SPI.transfer(0xA4); //6.66 kHz ±32 g
//  digitalWrite(cSelect2, HIGH); 


//    //Accelerometer ODR selection
//    //Accelerometer full-scale selection
//    //Enable second low-pass filter
//  digitalWrite(cSelect2, LOW);
//    SPI.transfer(0x10); //CTRL1_XL
//    SPI.transfer(0xA6); //6.66 kHz ±32 g
//  digitalWrite(cSelect2, HIGH); 

//    //Accelerometer ODR selection
//    //Accelerometer ±4 g selection
//    //Enable second low-pass filter
//  digitalWrite(cSelect2, LOW);
//    SPI.transfer(0x10); //CTRL1_XL
//    SPI.transfer(0xA2); //6.66 kHz ±4 g
//  digitalWrite(cSelect2, HIGH); 

    //Accelerometer ODR selection
    //Accelerometer ±4 g selection
    //No second low-pass filter
  digitalWrite(cSelect2, LOW);
    SPI.transfer(0x10); //CTRL1_XL
    SPI.transfer(0xA0); //6.66 kHz ±4 g
  digitalWrite(cSelect2, HIGH); 


//    //Accelerometer ODR selection
//    //Accelerometer ±4 g selection
//    //No second low-pass filter
//  digitalWrite(cSelect2, LOW);
//    SPI.transfer(0x10); //CTRL1_XL
//    SPI.transfer(0x90); //3.33 kHz ±4 g
//  digitalWrite(cSelect2, HIGH); 





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
  
//  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_uS, TimerHandler);
//  ISR_Timer.setInterval(TIMER_INTERVAL_xS,  doingSomething1);
  SysTick_Config( (F_CPU/sampleRate)*TICK_INTERVAL_MS );



}



//*******************************************************************************************************************

void loop(void)
{  
  enableInt = 1;
  uint32_t USB_Status = NRF_POWER->USBREGSTATUS;

  
  
    
      //BLE connected. Stream the data.
      //Send data out OUTATIME Bytes at time.
      //uint16_t OUTATIME = 80;
      uint16_t OUTATIME = 120;
      uint16_t Xout = dBufferIn / OUTATIME;
      if( Xout * OUTATIME != dBufferOut){


        //float magnitude = 0;
        //magnitude = magnitude + sq(calcOffset);
        //magnitude = sqrt(magnitude) * .1F;
        
    
     
        int16_t  outBuffer[OUTATIME];
        uint8_t * ptrOutBuffer = (uint8_t *) &outBuffer;
    
        for(uint16_t i = 0; i < OUTATIME; i++){
          outBuffer[i] = accelerationz[dBufferOut + i];
        }



          switch (activeCommand) {
            case 'X': 
              for(uint16_t i = 0; i < OUTATIME; i++){
                outBuffer[i] = accelerationx[dBufferOut + i];
              }
              bleuart.write(ptrOutBuffer,OUTATIME * 2); 
              break;
            case 'Y':
              for(uint16_t i = 0; i < OUTATIME; i++){
                outBuffer[i] = accelerationy[dBufferOut + i];
              }
              bleuart.write(ptrOutBuffer,OUTATIME * 2); 
              break;
            case 'Z': 
              for(uint16_t i = 0; i < OUTATIME; i++){
                outBuffer[i] = accelerationz[dBufferOut + i];
              }
              bleuart.write(ptrOutBuffer,OUTATIME * 2); 
              break;
            case 'M': 
              for(uint16_t i = 0; i < OUTATIME; i++){
                //calculate magnitude
                int32_t magnitude = 0;
                magnitude = magnitude + sq(accelerationx[dBufferOut + i]);
                magnitude = magnitude + sq(accelerationy[dBufferOut + i]);
                magnitude = magnitude + sq(accelerationz[dBufferOut + i]);
                magnitude = sqrt(magnitude);
                //shift the result to fit more of it into int16_t
                magnitude = magnitude - 32767;
                magnitude = constrain(magnitude, -32768, 32767);
                outBuffer[i] = magnitude;
              }
              bleuart.write(ptrOutBuffer,OUTATIME * 2); 
              break;
            }





          
//        if(USB_Status == 0){
//          bleuart.write(ptrOutBuffer,OUTATIME * 2); 
//        }else{
//          Serial.write(ptrOutBuffer,OUTATIME * 2); 
//        }
        
       
        dBufferOut = dBufferOut + OUTATIME;
        if (dBufferOut == bufferSize){
          dBufferOut = 0;
          batLevel();
          }



      

    
    
  }

      // Get commands from BLEUART
      while ( bleuart.available() )
      {
        uint8_t ch;
        ch = (uint8_t) bleuart.read();
        //Serial.write(ch);
        //Command issued?
        if ( ch==10 or ch==13){
          ///Convert to uppercase
          if (CommandA > 96){CommandA = CommandA - 32;}
          switch (CommandA) {
            case 'S': 
              ///Convert to uppercase
              if (CommandB > 96){CommandB = CommandB - 32;}
              activeCommand = CommandB;
              break;
            case 'F': 
              //Change filter
                //Stop the interupt to get access to the SPI
                enableInt = 0;
                delay(1);
              if(CommandB == '1'){
                  //Accelerometer ODR selection
                  //Accelerometer ±4 g selection
                  //Enable second low-pass filter
                digitalWrite(cSelect2, LOW);
                  SPI.transfer(0x10); //CTRL1_XL
                  SPI.transfer(0xA2); //6.66 kHz ±4 g
                digitalWrite(cSelect2, HIGH); 

//                digitalWrite(cSelect2, LOW);
//                  SPI.transfer(0x17); //CTRL8_XL (17h)
//                  //Low-pass filter settings
//                  //SPI.transfer(0x00); //odr/4
//                  //SPI.transfer(0x20); //odr/10
//                  //SPI.transfer(0x40); //odr/20
//                  //SPI.transfer(0x60); //odr/45
//                  //SPI.transfer(0x80); //odr/100
//                  //SPI.transfer(0xA0); //odr/200
//                  //SPI.transfer(0xC0); //odr/400
//                  SPI.transfer(0xE0); //odr/800
//                digitalWrite(cSelect2, HIGH); 

                
              }else{
                  //Accelerometer ODR selection
                  //Accelerometer ±4 g selection
                  //No second low-pass filter
                digitalWrite(cSelect2, LOW);
                  SPI.transfer(0x10); //CTRL1_XL
                  SPI.transfer(0xA0); //6.66 kHz ±4 g
                digitalWrite(cSelect2, HIGH); 
              }

              enableInt = 1;
              break;
            }
          CommandA = 0;
          CommandB = 0;
        }else{
          CommandA = CommandB;
          CommandB = ch;
        }
    

      }

      
  
  
}
