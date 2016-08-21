
/* Copyright (c) 2014, Nordic Semiconductor ASA

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include <SPI.h>              //For SPI communication between ATMega and NRF8001
#include <EEPROM.h>           //For Bit-Banging ATMega
#include <lib_aci.h>          //ACI library for NRF8001 communication
#include <aci_setup.h>        //ACI library setup file
#include "uart_over_ble.h"    //BLE UART UUID declarations and helper functions
#include <avr/power.h>        //Used to disable certain ATMega functions while in "on" state    
#include "services.h"         //NRF8001 setup file with pipe declarations and setup messages
#include "LowPower.h"         //Helper library for putting ATMega in a sleep state

/**
  Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
  This would mean that the setup cannot be changed once put in.
  However this removes the need to do the setup of the nRF8001 on every reset.
*/

//MCP3208 pin assignments
#define SELPIN A3 //Selection Pin (PC3)
#define DATAOUT A1// PC0
#define DATAIN  A2// PC2 
#define SPICLOCK  A0// PC1

//BLE pipe assignments
#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
static services_pipe_type_mapping_t
services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
#define NUMBER_OF_PIPES 0
static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

/* Store the setup for the nRF8001 in the flash of the AVR to save on RAM */
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

// aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;

/*
  Temporary buffers for sending ACI commands
*/
static hal_aci_evt_t  aci_data;
//static hal_aci_data_t aci_cmd;

/*
  Timing change state variable
*/
static bool timing_change_done = false;

/*
  Used to test the UART TX characteristic notification
*/
static uart_over_ble_t uart_over_ble;                                         
static uint8_t uart_buffer[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t uart_buffer_len = 14;

static uint16_t conn_interval = 416; //.51s (416*1.25ms) (4.16s Total interval with slave)
static uint16_t slave_latency = 8; //Only respond to every 8th conn_interval
static uint16_t supp_timeout = 1200; //connection timeout disconnect after 12s (1200*10ms)

bool connectedState = false; //Connection with client boolean
bool connParamState = false; //Has slave and client agreed on the new connection parameters.

//After initial connection aci event, conn parameters get changed to the fastest possible to exchange
//a series of debug logs. This ensures that this process is complete
uint8_t debugInfoState = 0; 

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  //Serial.println("ERROR ");
  //Serial.println(file);
  //Serial.println(": ");
  //Serial.println(line);
  //Serial.println("\n");
  while (1);
}

/*
  Description:
  This program provides pressure sensor measurement, digitization and BLE communication protocols for the Mercury Patch.
  Present program is confirmed to operate with a Samsung Android Phone and its related BLE stack serving as a client. Connection
  parameters and related ATMega sleeping intervals may need to be altered if a different client is selected for end user use. 

  Version History:          Date                  Notes
  Mercury_Patch_V3          2016-08-21            7-day <25mAh target met, code cleaned of debug components.
                                                  Sleep mode and transmission functional, Advertising may need to be sped up to
                                                  reduce the number of connection request timeouts create by the 1s advertising interval
  Mercury_Patch_V2          2016-06-23            Transmission of data implemented, power savings testing for connected and 
                                                  advertising states started, but not presently functioning.
  
*/
void setup(void)
{

  //set pin modes of MCP3208
  pinMode(SELPIN, OUTPUT);
  pinMode(DATAOUT, INPUT);
  pinMode(DATAIN, OUTPUT);
  pinMode(SPICLOCK, OUTPUT);
  
  //disable  MCP3208 to start with
  digitalWrite(SELPIN, HIGH);
  digitalWrite(DATAIN, LOW);
  digitalWrite(SPICLOCK, LOW);


  //Set Atmega Serial rate, 38400 optimum for error % when debugging, able to get away with 57600 with some ascii text being lost.
  //Serial.begin(57600);
  
  /*
    Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*) setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /*
    Tell the ACI library, the MCU to nRF8001 pin connections.
    The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = 5; //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.rdyn_pin   = 2; //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
  //SPI_CLOCK_DIV16 = 1MHz SPI speed

  aci_state.aci_pins.reset_pin              = 19; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupt Enabled 
  aci_state.aci_pins.interrupt_number       = 1; //Interrupt number, highest is device reset, then external inputs

  //Initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the ////Serial
  lib_aci_init(&aci_state, false);

  //1s delay to ensure completion of ATMega and NRF8001 setup, may not be needed.
  delay(1000);
}

//Initializes BLE UART communication
void uart_over_ble_init(void)
{
  uart_over_ble.uart_rts_local = true;
}

//Helper function for transmitting data from slave to client.
bool uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
  {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status)
    {
      aci_state.data_credit_available--;
    }
  }
  return status;
}

/*
 * UART_process_control used for handling rx inputs from master/client. They are not
 * presently used in the code.  Currently no control is being transmitted from the android app.
 */
bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    //Serial.println(*byte, HEX);
    switch (*byte)
    {
      /*
        Queues a ACI Disconnect to the nRF8001 when this packet is received.
        May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
          Parameters:
          None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        status = true;
        break;

      /*
        Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        conn_params = (aci_ll_conn_params_t *)(byte + 1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                               conn_params->max_conn_interval,
                               conn_params->slave_latency,
                               conn_params->timeout_mult);
        status = true;
        break;

      /*
        Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
          Parameters:
          None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;

      /*
        Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
          Parameters:
          None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }
  return status;
}

/*
 * 
 * BLE primary loop for handling ACI events
 * 
 */
void aci_loop()
{
  //Serial.println("Entering aci_loop() function...");
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;  //Retrieve ACI event codes

    switch (aci_evt->evt_opcode)
    {
      /**
        As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
        {
          aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
          switch (aci_evt->params.device_started.device_mode)
          {
            case ACI_DEVICE_SETUP:
              /**
                When the device is in the setup mode
              */
              //Serial.println(F("EDSS"));
              setup_required = true;
              break;

            case ACI_DEVICE_STANDBY:
              //Serial.println(F("EDST"));
              //Looking for a client by sending radio advertisements
              //Code will repeat until a connection event is received.
              if (aci_evt->params.device_started.hw_error)
              {
                delay(20); //Magic number used to make sure the HW error event is handled correctly.(Nordic Smeiconductor)
              }
              else
              {
                lib_aci_connect(240/* in seconds */, 0x1000 /* advertising interval 1s*/);
                //Serial.println(F("AS"));
              }
              break;
          }
        }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          /*ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
            TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
            all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
            */
          //Serial.println(F("ACI Command "));
          //Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          //Serial.println(F("Evt Cmd respone: Status "));
          //Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
                                 (uint8_t *) & (aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        //Serial.println(F("EC"));
        connectedState = true; //Switch connected state boolean to true
        uart_over_ble_init(); //Initizialize BLE UART
        aci_state.data_credit_available = aci_state.data_credit_total; //Retrieve available data credits from NRF8001 (2 total)

        /*
          Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
           /*Retrieves status of  the 15 pipes established to be used by the NRF8001 in the "services.h" setup file and
           * confirms that they are available and functional for use.
           * 
           * Previous code use pipe status to initiate the change connection parameter timning requests, this created 
           * conflicts with the debug info sharing started immediately upon a connected state with the phone client.
           */
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          //Serial.println(F("EP"));
        }
        break;

      case ACI_EVT_TIMING:
           /*Occurs whenever a successful change connection parameter timing request has occured.
           * This helps ensure that even that the connection parameters rewquested by the Patch are accepted
           * 
           */
           
       //Serial.println(F("ELIC"));
       if (aci_evt->params.timing.conn_rf_interval != conn_interval && debugInfoState==1){
          lib_aci_change_timing(conn_interval, conn_interval, slave_latency, supp_timeout); //Requests a connection parameter timning change to the client
          //Serial.println(F("TC"));       
       }

       //Once client has accepted user connection timing parameters,setup UART BLE for the given timing parameters
       else if(aci_evt->params.timing.conn_rf_interval == conn_interval && debugInfoState ==2){
          lib_aci_set_local_data(&aci_state,
             PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
             (uint8_t *) & (aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
             PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
          //Serial.println(F("TSET"));
          connParamState = true;
       }
           
        debugInfoState++; //Flow control for handling conection timing changes
        break;

      case ACI_EVT_DISCONNECTED:
        connectedState = false; //Reset connection with client boolean
        connParamState = false; //Reset connection parameters accepted boolaent
        debugInfoState = 0;     //Reset flow control for handling connection parameter timing change requests
        timing_change_done = false;  //Deprecated
        //Serial.println(F("ED"));
        lib_aci_connect(240/* in seconds */, 0x1000 /* advertising interval 1s*/);  //Restart advertising
        //Serial.println(F("AS"));
        break;

      case ACI_EVT_DATA_RECEIVED:
        //Serial.println(F("Pipe Number: "));
        //Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          //Serial.println(F(" Data(Hex) : "));
          for (int i = 0; i < aci_evt->len - 2; i++)
          {
            //Serial.println((char)aci_evt->params.data_received.rx_data.aci_data[i]);
            uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
            //Serial.println(F(" "));
          }

          uart_buffer_len = aci_evt->len - 2;
          //Serial.println(F(""));
          if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
          {
            /*Do this to test the loopback otherwise comment it out */
          }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        /*
         * Occurs whenever a data credit that was in use, such as those used to transmit a data packet are released
         * back to the NRF8001 to be used again.
         */
      
         //Serial.println(F("D"));
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        //Serial.println(F("ACI Evt Pipe Error: Pipe #:"));
        //Serial.println(aci_evt->params.pipe_error.pipe_number, DEC);
        //Serial.println(F("  Pipe Error Code: 0x"));
        //Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        //Serial.println(F("HW:"));
        //Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          //Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        lib_aci_connect(240/* in seconds */, 0x1000 /* advertising interval 1s*/); //Restart advertising
        //Serial.println(F("AS"));
        break;

    }
  }
  
  else 
  {
    /*
     * Enters the else statement if the aci_loop() is called and no aci_events ar available to be processed.
     */

    if (connectedState ==false){
      //Serial.println(F("SL"));
      gotoSleep();
    }
    
    else if (connectedState==true &&connParamState==true &&(aci_state.data_credit_available >= 2)){
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); 
      //Serial.println(F("PS"));
      lowPowerState(); //Turn uneeded ATMega functions off
      sensor_transmit_16bits(); //Transmit data

    }
    else if (connectedState==true &&connParamState==true &&(aci_state.data_credit_available < 2)){
      //Serial.println(F("AR"));
      LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF); 
      LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
      lowPowerState(); //Turn uneeded ATMega functions off
    }
    
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
     It indicates that do_aci_setup() should be called. The flag should be cleared if
     do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
  */
  if (setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
      
    }
  }
}

/*
 * 
 * Main iteration loop
 */

void loop() {
  //Process any ACI commands or events
  aci_loop();

  }

//Transmit ADC channels 1-7
void sensor_transmit_16bits()
{
  //MCP3208 iterates channels 1-8 instead of 0-7
  for (int g = 1; g < 8; g++)
  {
   read_adc(g);
  }
  
  uart_tx(uart_buffer, uart_buffer_len); //Make transmission
}

//Read one of the ADC channels
void read_adc(int channel) {
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)
  uint8_t MSBtop = 0; //Most Signficant Byte
  uint8_t LSBtop = 0; //Least Significant Byte
  
  //allow channel selection
  commandbits |= ((channel - 1) << 3);

  digitalWrite(SELPIN, LOW); //Enable MCP3208 ADC
  
  // setup bits to be written to MCP3208
  for (int i = 7; i >= 3; i--) {
    digitalWrite(DATAIN, commandbits & 1 << i);
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }

  //Writes to 2 bytes, 2 clock cycles to finish processing setup bits written to MCP3208
  digitalWrite(SPICLOCK, HIGH);   
  digitalWrite(SPICLOCK, LOW);
  digitalWrite(SPICLOCK, HIGH);
  digitalWrite(SPICLOCK, LOW);

  //read MSB bits from adc
  for (int i = 3; i >= 0; i--) {
    digitalWrite(SPICLOCK, HIGH);
    bitWrite(MSBtop,i,digitalRead(DATAOUT));
    digitalWrite(SPICLOCK, LOW);
  }
  
  //read LSB bits from adc
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SPICLOCK, HIGH);
    bitWrite(LSBtop,i,digitalRead(DATAOUT));
    digitalWrite(SPICLOCK, LOW);
  }

  //write uart buffer
  uart_buffer[channel*2-2] = MSBtop;
  uart_buffer[channel*2-1] = LSBtop;
  digitalWrite(SELPIN, HIGH); //turn off device
}

/*
 * Sleep Functions
 * 
 */

//For pin interupt, just a safety handle
void wakeOnInterrupt()
 {
  // don't need the external interrupts any more
 }

void gotoSleep(){
    attachInterrupt(0, wakeOnInterrupt, FALLING);
    
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0); 
    ////Serial.println(F("up"));
}

void lowPowerState(){
      ADCSRA = 0;
      power_adc_disable();
      power_timer0_disable();// Timer 0
      power_timer1_disable();// Timer 1
      power_timer2_disable();// Timer 2
}

 
