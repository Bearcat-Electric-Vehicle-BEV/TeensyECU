#ifndef BEV_PINS_H
#define BEV_PINS_H

typedef uint8_t pin_t;

#define PIN_VEHICLE_PWR 2
#define PIN_Brake_Light 5
#define PIN_ECU_OK 9
#define PIN_PRECHARGE_FINISHED 10
#define PIN_READY_TO_GO 7 // changed from 11
#define PIN_SHUTDOWN_TTL_OK 8 // changed from 24 from 12
#define PIN_CAN1_RX 23
#define PIN_CAN1_TX 22
#define PIN_CAN_TRANS_STDBY 33
#define PIN_SDA 19
#define PIN_SCL 18
#define PIN_ACCEL_0 17
#define PIN_ACCEL_1 16
#define PIN_BRAKE_POS 40
#define PIN_SOC 39
#define PIN_SPEAKER 37
#define PIN_RESET 36
#define PIN_FORWARD_SWITCH 28
#define PIN_FORWARD_ENABLE 29

// Wade
// analog sensor inputs
#define PIN_TIRE_RF_TEMP 24
#define PIN_TIRE_LF_TEMP 25
#define PIN_TIRE_RR_TEMP 26
#define PIN_TIRE_LR_TEMP 27
// brake temps?
#define PIN_BRAKE_PRESSURE 41
#define PIN_STEERING_ANGLE 38
#define PIN_SUS_RF_POS 21
#define PIN_SUS_LF_POS 20
#define PIN_SUS_RR_POS 15
#define PIN_SUS_LR_POS 14

#endif // BEV_PINS_H