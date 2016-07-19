
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

#include <SPI.h>
#include <EEPROM.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "uart_over_ble.h"
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "services.h"
/**
  Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
  This would mean that the setup cannot be changed once put in.
  However this removes the need to do the setup of the nRF8001 on every reset.
*/
#define SELPIN A3 //Selection Pin (PC3)
#define DATAOUT A1// PC0
#define DATAIN  A2// PC2 
#define SPICLOCK  A0// PC1

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
static uint8_t uart_buffer[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t uart_buffer_len = 20;
static uint8_t dummychar = 0;

static uint16_t maxConn = 300;
static uint16_t minConn = 300;
static uint16_t slaveLat = 0;
static uint16_t superTime = 1800;
long lastTime = 0;
long sendingInterval = 250; // sending interval in miliseconds
bool connectedState = false;

volatile boolean extInterrupt;
volatile boolean wdtInterrupt;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while (1);
}

/*
  Description:

  In this template we are using the BTLE as a UART and can send and receive packets.
  The maximum size of a packet is 20 bytes.
  When a command it received a response(s) are transmitted back.
  Since the response is done using a Notification the peer must have opened it(subscribed to it) before any packet is transmitted.
  The pipe for the UART_TX becomes available once the peer opens it.
  See section 20.4.1 -> Opening a Transmit pipe
  In the master control panel, clicking Enable Services will open all the pipes on the nRF8001.

  The ACI Evt Data Credit provides the radio level ack of a transmitted packet.
*/
void setup(void)
{

  
  ADCSRA = 0;  // disable ADC
  power_adc_disable(); // ADC converter
  power_usart0_disable();// Serial (USART) 
  power_twi_disable(); // TWI (I2C)
  
  //set pin modes of MCP3208
  pinMode(SELPIN, OUTPUT);
  pinMode(DATAOUT, INPUT);
  pinMode(DATAIN, OUTPUT);
  pinMode(SPICLOCK, OUTPUT);
  
  //disable device to start with
  digitalWrite(SELPIN, HIGH);
  digitalWrite(DATAIN, LOW);
  digitalWrite(SPICLOCK, LOW);


  Serial.begin(115200);
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
  aci_state.aci_pins.interrupt_number       = 1; //Interrupt two = external input on pin D2, ie RDYN pin

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);
  delay(2000);

  
  lib_aci_change_timing(minConn,maxConn,slaveLat,superTime);
                               
}

//Initializes 
void uart_over_ble_init(void)
{
  uart_over_ble.uart_rts_local = true;
}

//Helper function for transmitting data from slave to client.
bool uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1))
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
 * presently used in the code.
 */
bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    Serial.println(*byte, HEX);
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


void aci_loop()
{
  //Serial.println("Entering aci_loop() function...");
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;

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
              Serial.println(F("Evt Device Started: Setup"));
              setup_required = true;
              break;

            case ACI_DEVICE_STANDBY:
              Serial.println(F("Evt Device Started: Standby"));
              //Looking for an iPhone by sending radio advertisements
              //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
              if (aci_evt->params.device_started.hw_error)
              {
                delay(20); //Magic number used to make sure the HW error event is handled correctly.
              }
              else
              {
                lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
                Serial.println(F("Advertising started"));
              }
              break;
          }
        }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
                                 (uint8_t *) & (aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        connectedState = true;
        Serial.println(F("Evt Connected"));
        uart_over_ble_init();
        timing_change_done = false;
        aci_state.data_credit_available = aci_state.data_credit_total;

        /*
          Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
          // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        Serial.println(F("Evt link connection interval changed"));
        lib_aci_set_local_data(&aci_state,
                               PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                               (uint8_t *) & (aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                               PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        connectedState = false;
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        break;

      case ACI_EVT_DATA_RECEIVED:
        Serial.print(F("Pipe Number: "));
        Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          Serial.print(F(" Data(Hex) : "));
          for (int i = 0; i < aci_evt->len - 2; i++)
          {
            Serial.print((char)aci_evt->params.data_received.rx_data.aci_data[i]);
            uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
            Serial.print(F(" "));
          }

          uart_buffer_len = aci_evt->len - 2;
          Serial.println(F(""));
          if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
          {
            /*Do this to test the loopback otherwise comment it out
            */
            /*
              if (!uart_tx(&uart_buffer[0], aci_evt->len - 2))
              {
              Serial.println(F("UART loopback failed"));
              }
              else
              {
              Serial.println(F("UART loopback OK"));
              }
            */
          }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started"));
        break;

    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
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

//Main loop()
void loop() {
  //Process any ACI commands or events
   aci_loop();
   gotoSleep();
//   
  if (connectedState && wdtInterrupt) {
    sensor_transmit_16bits();
  }
}

//Transmit ADC channels 1-7
void sensor_transmit_16bits()
{
  for (int g = 1; g < 8; g++)
  {
   read_adc(g);
  }
  
  lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, uart_buffer, uart_buffer_len);
}

//Read one of the ADC channels
void read_adc(int channel) {
  unsigned int temp_adcvalue;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)
  uint8_t MSBtop = 0;
  uint8_t LSBtop = 0;
  
  //allow channel selection
  commandbits |= ((channel - 1) << 3);

  digitalWrite(SELPIN, LOW); //Select adc
  // setup bits to be written
  for (int i = 7; i >= 3; i--) {
    digitalWrite(DATAIN, commandbits & 1 << i);
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }

  digitalWrite(SPICLOCK, HIGH);   //ignores 2 null bits
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

//For pin interupt
void wakeOnInterrupt ()
 {
 extInterrupt = true;
 detachInterrupt (0);  // don't need the external interrupts any more
 }

//Setup Watchdog timer
void myWatchdogEnable() {  // turn on watchdog timer; interrupt mode every 0.5s
 MCUSR = 0;
 WDTCSR |= B00011000;
 WDTCSR = B01000101;
}

//Sleep function
void gotoSleep(void)
{
   ADCSRA = 0;                    //disable the ADC
   noInterrupts ();     // timed sequences follow
//   EIFR = bit (INTF0);  // clear flag for interrupt 0
//   attachInterrupt (0, wakeOnInterrupt, FALLING);
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // if (connectedState){
   myWatchdogEnable();
  // }
   
   wdtInterrupt = false;
   extInterrupt = false;
   sleep_enable();
   byte mcucr1 = MCUCR | bit(BODS) | bit(BODSE); //turn off the brown-out detector while sleeping
   byte mcucr2 = mcucr1 & ~bit(BODSE);
   MCUCR = mcucr1; //timed sequence
   MCUCR = mcucr2; //BODS stays active for 3 cycles, sleep instruction must be executed while it's active
   interrupts ();      // need interrupts now
   sleep_cpu();                   //go to sleep
   sleep_disable();               //wake up here
}

//Watchdog interrupt
ISR(WDT_vect) {
 wdtInterrupt = true;
 wdt_disable();
}

 
