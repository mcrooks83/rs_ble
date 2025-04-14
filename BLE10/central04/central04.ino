/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 * This sketch demonstrate the central API(). A additional bluefruit
 * that has bleuart as peripheral is required for the demo.
 */
#include <bluefruit.h>

BLEClientUart clientUart; // bleuart client

uint32_t rx_count = 0;

const uint16_t bufferSize = 10000;
uint8_t inBuf[bufferSize];

uint16_t dBufferIn = 0;
uint16_t dBufferOut = 0;


// Custom UUID used to differentiate this device.
// Use any online UUID generator to generate a valid UUID.
// Note that the byte order is reversed ... CUSTOM_UUID
// below corresponds to the follow value:
// df67ff1a-718f-11e7-8cf7-a6006ad3dba0
//const uint8_t CUSTOM_UUID[] =
//{
//    0xA0, 0xDB, 0xD3, 0x6A, 0x00, 0xA6, 0xF7, 0x8C,
//    0xE7, 0x11, 0x8F, 0x71, 0x1A, 0xFF, 0x67, 0xDF
//};
//BLEUuid uuid = BLEUuid(CUSTOM_UUID);

void setup()
{
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Config the connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);  
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);  

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);  
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }else
  {      
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{



  if ( clientUart.discover(conn_handle) )
  {


    clientUart.enableTXD();


  }else
  {

    
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  

}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */ 
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  
//  if ( uart_svc.available() ){
//    uint16_t serB = uart_svc.available();
//    if( serB > 255){serB = 255;}
//    uint8_t inBuf[serB];
//    uart_svc.readBytes(inBuf, serB);
//    Serial.write(inBuf, serB);
//  }

  while ( uart_svc.available() )
  {
    inBuf[dBufferIn] = uart_svc.read();
    dBufferIn++;
    if (dBufferIn == bufferSize){dBufferIn = 0;}
  }


}

void loop()
{
      //Send data out OUTATIME bytes at time to avoid USB delays.
      uint16_t OUTATIME = 100;
      uint16_t Xout = dBufferIn / OUTATIME;
      if( Xout * OUTATIME != dBufferOut){
    
        //uint8_t inBuf[bufferSize];
        uint8_t  outBuffer[OUTATIME];
    
        for(uint16_t i = 0; i < OUTATIME; i++){
    
  
          outBuffer[i] = inBuf[dBufferOut + i];
    
   
        }
    
        Serial.write(outBuffer,OUTATIME); 
        
        dBufferOut = dBufferOut + OUTATIME;
        if (dBufferOut == bufferSize){dBufferOut = 0;}
      }
      // Get Serial input and send to Peripheral
      if ( Serial.available() )
      {
        //delay(2); // delay a bit for all characters to arrive
        
        char str[20+1] = { 0 };
        Serial.readBytes(str, 20);
        
        clientUart.print( str );
      }

      
}
