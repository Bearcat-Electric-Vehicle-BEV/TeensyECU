#include "bev_pins.h"

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
unsigned long xlastwaketime = 0;

//TODO find better name
void func(const CAN_message_t &msg){
  // static unsigned long xlastwaketime;
  
  if (abs(millis() - (int)xlastwaketime) > 2000)
  {
    digitalWrite(PIN_ECU_OK, HIGH);
    digitalWrite(PIN_PRECHARGE_FINISHED, LOW);
    
    Serial.print("FAILURE");
    while (1)
    {
      //TODO: broadcast failure
    }
  }
  
  unsigned int shutdown = msg.buf[0]; //shutdown
  unsigned int precharge = msg.buf[1]; //precharge
  unsigned int speaker = msg.buf[2]; //speaker

  if (shutdown)
  {
    digitalWrite(PIN_ECU_OK, HIGH);
  }
  else
  {
    digitalWrite(PIN_ECU_OK, LOW);
  }

  if (precharge)
  {
    noInterrupts();
    // delay(1500);
    digitalWrite(PIN_PRECHARGE_FINISHED, HIGH);
    interrupts();
  }
  else
  {
    digitalWrite(PIN_PRECHARGE_FINISHED, LOW);
  }

  if (speaker)
  {
    //TODO drive speaker
    //digitalWrite(PIN_SPEAKER, HIGH);
  }
  
  xlastwaketime = millis();
}

void canSniff(const CAN_message_t &msg) {
  func(msg);
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } 
  Serial.println();
}

void setup(){
    Serial.begin(9600); 
    delay(400);

    // Set Pin Modes
    pinMode(PIN_ECU_OK, OUTPUT);
    pinMode(PIN_PRECHARGE_FINISHED, INPUT);
    pinMode(PIN_SHUTDOWN_TTL_OK, INPUT);
    pinMode(PIN_CAN_TRANS_STDBY, OUTPUT);

    digitalWrite(PIN_CAN_TRANS_STDBY, LOW);


    // initialize CAN
    Can0.begin();
    Can0.setBaudRate(500000);
    Can0.setMaxMB(16);

    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    
    Can0.onReceive(canSniff);
    Can0.mailboxStatus();
    
    xlastwaketime = millis();
}

// DAQ Loop
void loop() {
  Can0.events();
}