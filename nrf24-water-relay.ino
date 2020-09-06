//
//
//
//

#define DEBUG_PRINT
#include "debug.h"

// "RF24" by TMRh20
//   https://tmrh20.github.io/RF24/
#include <printf.h>
#include <RF24.h>

#define RELAY_ON               LOW
#define RELAY_OFF              HIGH

#define PIN_PULSE_1            2              // PIN receiving interrupt from flow sensor #1.
#define INT_PULSE_1            0              // Interrupt #0 is defined for PIN 2 on Arduino Nano.

#define PIN_PULSE_2            3              // PIN receiving interrupt from flow sensor #2.
#define INT_PULSE_2            1              // Interrupt #1 is defined for PIN 3 on Arduino Nano.

#define PIN_RELAY_1            4              // PIN for turning relay #1 on/off
#define PIN_RELAY_2            5              // PIN for turning relay #2 on/off

#define PIN_RF24_CSN           9              // CSN PIN for RF24 module.
#define PIN_RF24_CE           10              // CE PIN for RF24 module.

#define NRF24_CHANNEL         1               // 0 ... 125
#define NRF24_CRC_LENGTH      RF24_CRC_16     // RF24_CRC_DISABLED, RF24_CRC_8, RF24_CRC_16 for 16-bit
#define NRF24_DATA_RATE       RF24_250KBPS    // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define NRF24_PAYLOAD_SIZE    32              // Max. 32 bytes.
#define NRF24_PA_LEVEL        RF24_PA_LOW     // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX    
#define NRF24_RETRY_DELAY     5               // Delay bewteen retries, 1..15.  Multiples of 250µs.
#define NRF24_RETRY_COUNT     15              // Number of retries, 1..15.

byte tx_addr[6] = "7LD57";                    // Transmission address for readings.
byte rx_addr[6] = "1WTRC";                    // Address for getting ON/OFF command.
byte payload[32];                             // Buffer for payload data.

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);        // NRF24L01 radio.

#define TX_DELAY_SECONDS      10              // Delay between transmission of status.
unsigned long tx_ms = 0;                      // Millisecond value for last transmission.
unsigned long tx_count = 0;                   // Number of transmissions.

const unsigned char tx_protocol = 0x06;       // Transmission protocol number.
const unsigned char sensor1_id  = 0x01;       // Identification of sensor #1.
const unsigned char sensor2_id  = 0x02;       // Identification of sensor #2.

#define ALIVE_DELAY_SECONDS    2              // Delay between debugging output showing device is alive.
unsigned long alive_ms = 0;                   // Millisecond value for last output.

#define C1  11                                // Constant used when converting puses to litres/min. for sensor #1.
#define C2  11                                // Constant used when converting puses to litres/min. for sensor #2.
unsigned long flow_ms = 0;                    // Determine when last flow measurement started.
volatile unsigned int pulse1 = 0;             // Pulses counted by interrupt routine for flow sensor #1.
volatile unsigned int pulse2 = 0;             // Pulses counted by interrupt routine for flow sensor #1.
float total_litres1 = 0.0;                    // Total litres since last reboot.
float total_litres2 = 0.0;                    // Total litres since last reboot.


void setup() 
{
  // Initialize debugging.
  debug_begin();
  debug_println(F("\n\nWater Relay, v1.0"));

  // Initialize relay to OFF.
  debug_println(F("- R1"));
  pinMode(PIN_RELAY_1, OUTPUT);
  digitalWrite(PIN_RELAY_1, RELAY_OFF);

  // Initialize relay to OFF.
  debug_println(F("- R2"));
  pinMode(PIN_RELAY_2, OUTPUT);
  digitalWrite(PIN_RELAY_2, RELAY_OFF);

  debug_println(F("- P1"));
  pinMode(PIN_PULSE_1, INPUT);
  digitalWrite(PIN_PULSE_1, HIGH);
  
  debug_println(F("- P2"));
  pinMode(PIN_PULSE_2, INPUT);
  digitalWrite(PIN_PULSE_2, HIGH);
  
  // Initialize NRF24L01 module.
  debug_println(F("- NRF24"));
  nrf24_setup();
  debug_nrf24(radio);

  debug_println("- INT");
  flow_ms = millis();
  attachInterrupt(INT_PULSE_1, count_pulse_1, FALLING);
  attachInterrupt(INT_PULSE_2, count_pulse_2, FALLING);

  // Start listening for commands ...
  radio.startListening();

  // Get milliseconds controlling status transmission and alive notifications.
  tx_ms = millis();
  alive_ms = millis();
  debug_println("DONE");
}

void loop() 
{
  
  if (radio.available()) 
  {
    
    // Read payload.
    radio.read(&payload, sizeof(payload));

    // Write payload for debugging.
    #ifdef DEBUG_PRINT        
    // Data available.
    debug_print(F("\nRCV: "));
    for (int i = 0; i < sizeof(payload); i++) 
    {
      if (payload[i] <= 0x0F) debug_print('0'); 
      debug_print(payload[i], HEX); 
      debug_print(' ');
    }
    debug_println();
    #endif
      
    if (payload[0] == 0x04) 
    {
      // We have received a message with protocol id = 0x04.
      debug_print(F("MSG 0x04: "));
      debug_print(F("#1:")); debug_print(payload[1]); debug_print(F(", #2:")); debug_println(payload[2]);

      // Update relays.
      update_relay(PIN_RELAY_1, payload[1]);
      update_relay(PIN_RELAY_2, payload[2]);
      
      // Status have changed. Send status.
      send_status();
    }
    else 
    {
      debug_print(F("INV"));
    }
  }
  
  // Transmit status and sensor readings.
  if (millis() - tx_ms > (unsigned long)(TX_DELAY_SECONDS * 1000L)) 
  {
    tx_ms = millis();
    send_status();    
  }
  
  // Print debug information to check that Arduino loop is running.
  if (millis() - alive_ms > (ALIVE_DELAY_SECONDS * 1000L)) 
  {
    alive_ms = millis();  
    debug_print('.');    
  }
}

void count_pulse_1() 
{
  pulse1++;  
}

void count_pulse_2() 
{
  pulse2++;  
}

bool update_relay(int relay, byte value) 
{
  
  if (relay != PIN_RELAY_1 && relay != PIN_RELAY_2) return false;
  
  if (value == 0) 
    digitalWrite(relay, RELAY_OFF);
  else if (value == 1) 
    digitalWrite(relay, RELAY_ON); 
  else if (value == 2) 
  {
    if (digitalRead(relay) == RELAY_ON) 
      digitalWrite(relay, RELAY_OFF);
    else
      digitalWrite(relay, RELAY_ON);
  }
  else 
    return false;
    
  return true;  
}

void send_status() 
{
  int rc;
  unsigned char relay1, relay2;
  
  // Stop listening in order to be able to send.
  radio.stopListening();
  radio.flush_tx();
  
  // Output the next transmission number.
  debug_print(F("\nXMIT: "));
  debug_println(++tx_count);
  
  // Read status of relays.
  relay1 = digitalRead(PIN_RELAY_1) == RELAY_OFF ? 0 : 1;
  relay2 = digitalRead(PIN_RELAY_2) == RELAY_OFF ? 0 : 1;

  // Disable interrupts to prevent us getting interrupts while calculating the flow.
  detachInterrupt(INT_PULSE_1);
  detachInterrupt(INT_PULSE_2);

  // How many milliseconds since last reading?
  unsigned long flow_time = millis() - flow_ms;

  // Report time in milliseconds and number of pulses on each flow sensor.
  debug_print("Flow: ms="); debug_print(flow_time); debug_print(", p1="); debug_print(pulse1); debug_print(", p2="); debug_println(pulse2);
  
  //
  // Calculate flow rate.
  //
  // F = C * Q 
  //  F is the pulse frequency in Hz (pulses per second).
  //  C is a constant typically published in datasheet of flow sensor.
  //  Q is the flow rate in litres per minute (L/min).
  //
  // So, we calculate the flow as:
  // Q = F / C
  //
  
  // Calculate flow rate #1
  float F1 = (1000.0 / flow_time) * pulse1;                   // Pulses per second.
  float Q1 = F1 / C1;                                         // Litres per minute.
  float L1 = Q1 / (60000.0 / flow_time);                      // Litres this duty cycle.
  total_litres1 += L1;                                        // Total litres since last reboot.
  
  // Calculate flow rate #2
  float F2 = (1000.0 / flow_time) * pulse2;                   // Pulses per second.
  float Q2 = F2 / C2;                                         // Litres per minute.
  float L2 = Q2 / (60000.0 / flow_time);                      // Litres this duty cycle.
  total_litres2 += L2;                                        // Total litres since last reboot.
  
  // Report flow rate and total millilitres.
  debug_print("Q1="); debug_print(Q1); debug_print(", L1="); debug_print(L1); debug_print(", T1="); debug_println(total_litres1);
  debug_print("Q2="); debug_print(Q2); debug_print(", L2="); debug_print(L2); debug_print(", T2="); debug_println(total_litres2);
  
  // Reset pulse counts.
  pulse1 = 0;           
  pulse2 = 0;           
  
  // Note start of new duty cycle.
  flow_ms = millis();
  
  // Enable interrupts.
  attachInterrupt(INT_PULSE_1, count_pulse_1, FALLING);
  attachInterrupt(INT_PULSE_2, count_pulse_2, FALLING);
  
  // Pack payload for relay/sensor #1.
  int offset = 0;
  memcpy(payload + offset, &tx_protocol, sizeof(tx_protocol)); offset += sizeof(tx_protocol);  
  memcpy(payload + offset, &tx_count, sizeof(tx_count)); offset += sizeof(tx_count);
  
  memcpy(payload + offset, &sensor1_id, sizeof(sensor1_id)); offset += sizeof(sensor1_id);
  memcpy(payload + offset, &relay1, sizeof(relay1)); offset += sizeof(relay1);  
  memcpy(payload + offset, &flow_time, sizeof(flow_time)); offset += sizeof(flow_time);
  memcpy(payload + offset, &Q1, sizeof(Q1)); offset += sizeof(Q1);
  memcpy(payload + offset, &L1, sizeof(L1)); offset += sizeof(L1);
  memcpy(payload + offset, &total_litres1, sizeof(total_litres1)); offset += sizeof(total_litres1);

  // Send payload #1
  rc = nrf24_send(payload, offset, 5);
  if (rc > -1) 
  {
    debug_print(F("RTY1=")); debug_println(rc);
  }
  else 
    debug_println(F("ERR1"));
  
  // Pack payload for relay/sensor #2.
  offset = sizeof(tx_protocol) + sizeof(tx_count);
  memcpy(payload + offset, &sensor2_id, sizeof(sensor2_id)); offset += sizeof(sensor2_id);
  memcpy(payload + offset, &relay2, sizeof(relay2)); offset += sizeof(relay2);
  memcpy(payload + offset, &flow_time, sizeof(flow_time)); offset += sizeof(flow_time);
  memcpy(payload + offset, &Q2, sizeof(Q2)); offset += sizeof(Q2);
  memcpy(payload + offset, &L2, sizeof(L2)); offset += sizeof(L2);
  memcpy(payload + offset, &total_litres1, sizeof(total_litres1)); offset += sizeof(total_litres1);
  
  // Send payload #2
  rc = nrf24_send(payload, offset, 5);
  if (rc > -1) 
  {
    debug_print(F("RTY2=")); debug_println(rc);
  }
  else 
    debug_println(F("ERR2"));
  
  // Go back to listening.
  radio.startListening();   
}
  
int nrf24_send(byte *buf, int bytes, int retries)
{
  int max_retries = retries;
  
  while (retries > 0)
  {
    delay((max_retries - retries) * 50);
    if (radio.write(payload, bytes))
      break;
    retries--;      
  }

  if (retries == 0)
    return -1;
  else
    return max_retries - retries;
}

void nrf24_setup()
{
  radio.begin();
  
  radio.setAutoAck(true);                 
  radio.enableDynamicPayloads();          
  radio.setPALevel(NRF24_PA_LEVEL);
  radio.setRetries(NRF24_RETRY_DELAY, NRF24_RETRY_COUNT);              
  
  radio.setDataRate(NRF24_DATA_RATE);          
  radio.setChannel(NRF24_CHANNEL);
  radio.setCRCLength(NRF24_CRC_LENGTH);
  radio.setPayloadSize(NRF24_PAYLOAD_SIZE);
  
  radio.openWritingPipe(tx_addr);  
  radio.openReadingPipe(1, rx_addr);
  
  radio.stopListening();                  
}
