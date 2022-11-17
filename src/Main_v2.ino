#include "bev_can.h"
#include "bev_logger.h"
#include "bev_etc.h"
#include "bev_state.h"
#include "bev_pins.h"
#include "bev_state.h"
#include "bev_faults.h"



void setup(){
    Serial.begin(9600); 
    delay(400);

    // Set Pin Modes
    pinMode(PIN_VEHICLE_PWR, INPUT);
    pinMode(PIN_Brake_Light, OUTPUT);
    pinMode(PIN_ECU_OK, OUTPUT);
    pinMode(PIN_PRECHARGE_FINISHED, INPUT);
    pinMode(PIN_READY_TO_GO, OUTPUT);
    pinMode(PIN_SHUTDOWN_TTL_OK, INPUT);
    pinMode(PIN_CAN_TRANS_STDBY, OUTPUT);
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);
    pinMode(PIN_ACCEL_0, INPUT);
    pinMode(PIN_ACCEL_1, INPUT);
    pinMode(PIN_BRAKE_POS, INPUT);
    pinMode(PIN_SPEAKER, OUTPUT);
    pinMode(PIN_RESET, INPUT);
    pinMode(PIN_FORWARD_SWITCH, INPUT);
    pinMode(PIN_FORWARD_ENABLE, OUTPUT);

    // initialize CAN
    CANInit();

    // DAQ Loop
    while (1)
    {
        //
    }
    

    for(;;) {}
}


void loop() {}