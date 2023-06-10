#include "SPI.h"
#include "SPI_MSTransfer_MASTER.h"
#include <FlexCAN_T4.h>
#include "temp_lookup.h"
#include <SD.h>

/* Free rtos includes */
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
//FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
// CAN_message_t msg;
SPI_MSTransfer_MASTER<&SPI, 10, 0x1235> mySPI1235;

/* Freertos definitions */
TaskHandle_t pxBMSTaskHandle;
void vBMSTask(void * pvParameters);

void setup() {
  //Serial.begin(115200);
  SPI.begin();
  pinMode(0, OUTPUT);
  pinMode(33, OUTPUT);
  digitalWrite(0, HIGH);
  digitalWrite(33, LOW);
  mySPI1235.begin();

  Can0.begin();
  Can0.setBaudRate(500000);
  //Can0.setMaxMB(16);

  //Can0.enableFIFO();
  // Can0.setFIFOFilter(REJECT_ALL);
  // Can0.setFIFOFilter(0, M192_Command_Message_CANID, STD);
  // Can0.setFIFOFilter(1, MSGID_0X6B2_CANID, STD);
  //Can0.enableFIFOInterrupt();
    
  //Can0.onReceive(CanSniff);

  
  xTaskCreate(vBMSTask, "BMS", 4096, nullptr, 5, &pxBMSTaskHandle);

  /** @note Execution doesn't go beyond scheduler */
  vTaskStartScheduler();

  for(;;) {}
    
}

void SendMessage(unsigned id, uint8_t value) {

    CAN_message_t msg;
    
    msg.id = id;
    msg.flags.extended = 0;
    msg.len = 1;
    msg.buf[0] = value;

    Can0.write(msg);

}

void SendMessage(unsigned id, uint8_t* value, int len) {

    CAN_message_t msg;

    if (value == nullptr)
    {
        return;
    }
    msg.id = id;
    msg.flags.extended = 0;
    msg.len = 1;
    for (int i=0; i<len; i++)
    {
      msg.buf[i] = value[i];
    }
    Can0.write(msg);
}

void WriteToSD(const CAN_message_t &msg, const char* fname){
    static char buffer[100];

    if (!SD.begin(BUILTIN_SDCARD)) {
      return;
    }

    CAN2Str(msg, buffer, 100);

    File dataFile = SD.open(fname, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(buffer);
      dataFile.close();
    }

}

void WriteToSD(const char* dataString, const char* fname){
    if (!SD.begin(BUILTIN_SDCARD)) {
        return;
    }

    if (dataString == NULL || fname == NULL) {
        return;
    }

    File dataFile = SD.open(fname, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }

}

void CAN2Str(const CAN_message_t &msg, char *buffer, size_t len) {

    snprintf(buffer, len, "MB:%d OVERRUN:%d LEN:%d EXT:%d TS:%05d ID:%04lX BUFFER: ", 
              msg.mb, msg.flags.overrun, msg.len, msg.flags.extended, 
              msg.timestamp, msg.id);
    
    /* Adding char '0' to numeric returns ascii value */
    char tmpBuf[msg.len] = {0};
    for ( uint8_t i = 0; i < msg.len; i++ ) {
        tmpBuf[i] = msg.buf[i] + '0';
    }

    /* Append to buffer */
    strncat(buffer, tmpBuf, msg.len);

}

void loop() {}

#define DEBUG_ADDR 0x69
int counter = 0;
uint8_t debug_buffer[8] = {0};
uint16_t address[7] = {0x00, 0x800, 0x1000, 0x1800, 0x2000, 0x2800, 0x3000};

void vBMSTask(__attribute__((unused)) void * pvParameters) {
	
	/* Free rtos execution rate */
	TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    const TickType_t ImplausibilityTime = pdMS_TO_TICKS(50);
	
	// Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

		// Infinite loop
   for( ;; )
    {
	
  int8_t min = 0;
  int8_t max = 0;
  int8_t average = 0;
  int sum = 0;

  uint8_t MinChanID = 0;
  uint8_t MaxChanID = 0;
  uint16_t rxbuff = 0x0;

  for (int i = 0; i < 7; i++){
    uint16_t txbuff = address[i];
    rxbuff = mySPI1235.transfer16(&txbuff, 1, 1);
    rxbuff = (rxbuff >> 4) & 0xff;
    //Serial.println(i);
    //Serial.println(rxbuff);
    // SendMessage(DEBUG_ADDR+1, i);
    //SendMessage(DEBUG_ADDR+4, rxbuff);
    
    //This section forces rxbuff to be within our array size
    int index = rxbuff - 39; 
    if (index < 0){
      index = 0;
    }
    if (index > 57){
      index = 57;
    }
    int8_t temp = lookup[index];

    //Serial.println(temp);
    if (i == 0){
      min = temp;
      max = temp;

      MinChanID = i;
      MaxChanID = i;
    }

    if (temp < min){
      min = temp;
      MinChanID = i;
    }
    if (temp > max){
      max = temp;
      MaxChanID = i;
    }

  //SendMessage(DEBUG_ADDR+1, rxbuff);

    sum += temp;
  }

  average = sum / 7;

  CAN_message_t msg;

  msg.id = 0x1839F380;
  msg.len = 8;
  msg.flags.extended = 1;
  msg.flags.remote   = 0;
  msg.flags.overrun  = 0;
  msg.flags.reserved = 0;
  Can0.mailboxStatus(); 

  //How do we want to display which channel is the hottest?
  msg.buf[0] = 0x00;//  rxbuff(0x00)  Thermistor module number
  msg.buf[1] = min; //min;
  msg.buf[2] = max; //max;
  msg.buf[3] = average; //average;
  msg.buf[4] = 0x07;//Number of thermistors enabled   (7)
  msg.buf[5] = 0x01;//Highest thermistor ID on the module
  msg.buf[6] = 0x00;//Lowest thermistor ID on the module

  // uint8_t checksum = (0x01 + 0x01 + rxbuff + rxbuff + rxbuff);//min + max + average);

  //Checksum 8-bit (sum of all bytes + ID + length) 
  uint8_t checksum = (min + max + average + 0x07 + 0x01 + 0x00 + 0 + 8); 
  // also try 0x41
  msg.buf[7] = checksum;

  Can0.write(msg);
  Can0.events();
  counter += 1;
  if (counter >= 100){
    WriteToSD(msg, "CanLogger.txt");
    counter = 0;
  }
  
  //BMS expects CAN message every 100 ms
  vTaskDelayUntil(&xLastWakeTime, xFrequency);

	} // end of infinite loop
	
	// bad if outside infinite loop 
	configASSERT(NULL);
  
}