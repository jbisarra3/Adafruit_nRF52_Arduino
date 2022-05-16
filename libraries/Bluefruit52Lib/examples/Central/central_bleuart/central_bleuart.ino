/*********************************************************************
Sketch: bleuart_DoseCheck
Joel Bisarra
May 13, 2022

This sketch programs an AdaFruit Feather nRF52840 Express as a central 
BLE device to relay commands between an EmStat Pico device and the PSTrace
software. Note, an additional nRF52840 device is used as a peripheral to
send and receive commands from the EmStat Pico device

Hardware Info:
  Adafruit nRF52840 Express
  
Notes: 
  Based on central_bleuart.ino, example code from Adafruit nRF52 Arduino
  library. This code actsed as a central device to transfer information between the 
  Serial port and BLE peripheral.

Revision History:
  - 13 May, 2022 -  Initial Revision from example code: Removed battery info client
                    and Serial print statements, increase MTU size, and changed
                    data reception loop to store all characters in a string
                    before printing instead of printing each character individually
                    to allow us to break up commands
*********************************************************************/


#include <bluefruit.h>

BLEClientDis  clientDis;  // device information client
BLEClientUart clientUart; // bleuart client


void setup()
{
  Serial.begin(115200);

  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  // Configure DIS client
  clientDis.begin();

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
  }
  else
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
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  //Check if connection has been discovered before enabling TXD
  if ( clientUart.discover(conn_handle) &&  clientDis.discover(conn_handle))
  {
    //Below snippet was used to increase the MTU of the BLE connection, which 
    //is the maximum length of a packet of information in the BLE connection.
    //This was done to prevent the BLE from splitting up commands into 
    //separate packets
    
    // request PHY changed to 2MB
    conn->requestPHY();

    // request to update data length
    conn->requestDataLengthUpdate();
    
    // request mtu exchange
    conn->requestMtuExchange(247);
    
    // delay a bit for all the request to complete
    delay(1000);
    
    clientUart.enableTXD();
  }
  else
  {
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
  char ble_str[240] = {0};
  int i = 0;
  //Receive all characters from Peripheral and send to Serial
  while ( uart_svc.available())
  {
    delay(1);
    ble_str[i] = (char) uart_svc.read();
    i++;
  }
  Serial.write(ble_str);
}

void loop()
{
  
  if ( Bluefruit.Central.connected() )
  {
     //Not discovered yet
    if ( clientUart.discovered() )
    {
      // Get single command from Serial input and send to Peripheral
      char serial_str[240] = {0};
      int i = 0;
      while( Serial.available())
      {
        delay(1);
        char serial_char = (char) Serial.read();
        serial_str[i] = serial_char;
        i++;
        
        //Newline separates commands in Serial input 
        //Needed in case PSTrace sends multiple commands at once
        if(serial_char == '\n')
        {
          break;
        }
      }
      clientUart.write(serial_str);
    }
  }
  
}
