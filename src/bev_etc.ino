/**
 * @file bev_etc.ino
 * @author Marshal Stewart (stewa2m3@mail.uc.edu)
 * @brief 
 * @version v1.1
 * @date 2022-07-03
 * 
 * ETC APIs and general throttle control functions
 * 
 * $$$$$$$\  $$$$$$$$\ $$\    $$\
 * $$  __$$\ $$  _____|$$ |   $$ |
 * $$ |  $$ |$$ |      $$ |   $$ |
 * $$$$$$$\ |$$$$$\    \$$\  $$  |
 * $$  __$$\ $$  __|    \$$\$$  /
 * $$ |  $$ |$$ |        \$$$  /
 * $$$$$$$  |$$$$$$$$\    \$  /   
 * \_______/ \________|    \_/    
 * Copyright University of Cincinnati 2022
 * 
 */

#include "bev_etc.h"
#include "bev_can.h"
#include "bev_pins.h"
#include "bev_state.h"
#include "bev_logger.h"
#include "etc_curve.h"

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @paragraph APPS-Detection APPS Detection
 * The pedals are read through the ADC, depending on what our actual input 
 * voltage is this will change. This is also dependent on what ADC resolution
 * is being used. The Teensy 4.1 uses 10-bit ADC resolution, by default, but
 * 12-bit resolution can be enabled. The following is to accomidate these.
 *
 * @todo Once Teensy Board is revisioned, should add checking of 3.3V src ping
 *      for max ADC resolution
 * 
 * @todo Check what ADC bit resolution is being used
 */


/**
 * @brief Obstraction of Pedal, makes reading ETC Task easier
 */
Pedal::Pedal(pin_t _pin,  uint8_t _src_type) : pin(_pin), src_type(_src_type) {};

/**
 * @brief Reads current pedal position and returns float of position %
 * 
 * @return adc_t (float) Percent Press 0-1
 * 
 * @note ACCEL_MAX is used here
 */
adc_t Pedal::ReadPedal(void)
{
    switch(src_type)
    {
      case DIGITAL:
        pos = digitalRead(pin);
        break;
      case ANALOG_0V_3V:
        pos = analogRead(pin);
        ratio = (float)pos / ACCEL_MAX;
        break;
      case ANALOG_3V_0V:
        pos = analogRead(pin);
        ratio = 1 - ((float)pos / 650);
        break;
      case ANALOG_0V_5V:
        pos = analogRead(pin);
        ratio = (float)pos / ACCEL_MAX;
        break;
      default:
        // Log.error("Invalid Pedal Src Type: %d", src_type);
        break;
    }

    return pos;

}

/**
 * @brief Overloaded (-) operator to allow easier readability in ETC task
 */
float Pedal::operator-(const Pedal &pedal)
{
    return (this->ratio - pedal.ratio);
}


/**
 * @brief Get Ratio Function for Pedal
 */
float Pedal::GetRatio()
{
  return ratio;
}

/**
 * @brief 
 * 
 * @return uint32_t 
 *
 * @todo May want to define the table precision
 */
uint32_t Pedal::GetRatio_u32()
{
    return (uint32_t)(ratio * 100);
}

/** @todo for ADC may want larger threshold */
bool Pedal::Applied(void)
{
    return (pos > 0);
}

/**
 * @brief Applies ETC Curve to request
 * 
 * @param request Nm * 10
 */
static void ETC_Update(torque_t *request)
{
  extern const torque_t lookup[TABLE_SIZE];

  *request = lookup[*request];
}

/**
 * ETC
 * 
 * @param pedal_0 input
 * @param request output
 * 
 * @return code_t
 * 
 * @brief
 * 
 */
code_t ETC(Pedal *Pedal0, torque_t *request) {

    /** Reset Request */
    *request = 0;
  
    if (Pedal0->GetRatio_u32() >= TABLE_SIZE)
    {
      return FAIL;
    }

    #ifndef SPEED_MODE
    ETC_Update(request);  // Writes to request
    #else
    *request = Pedal0->GetRatio_u32();
    #endif

    /** @todo Check for restrictions */
    // If request > DCL or something, calc projected current draw?

    return OK;
}

/**
 * @brief 
 * 
 * @todo Convert tasks to statically allocated
 */
TaskHandle_t pxETCTaskHandle;

/**
 * @brief Read pedal, perform ETC calc, send cmd
 *  
 * @author Marshal
 * @date 6/27/22 
 * 
 * @details 10 ms periodic high priority task
 * 
 * @param pvParameters (unused)
 * 
 * @todo Validate timing, can't take longer than 10ms or else starvation if
 * so split to two tasks
 * 
 */
void vETCTask(__attribute__((unused)) void * pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    const TickType_t ImplausibilityTime = pdMS_TO_TICKS(100);

     // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    /** @note BRAKE is digital, however could be analog in future */
    Pedal Accel_0(PIN_ACCEL_0, ANALOG_0V_5V);
    Pedal Accel_1(PIN_ACCEL_1, ANALOG_3V_0V);
    Pedal Brake(PIN_BRAKE_POS, DIGITAL);

    bool InvalidAPPS = false;

    /** @todo utilize validity bits */
    torque_t Request = 0;
    
    /** @todo Needs used */
    bool __attribute__((unused)) RequestValid = false;

    CmdParameters_t cmd;

    cmd.Torque_Limit_Command = 0;   /** @todo need to find valid value */
    cmd.RollingCounter = 0;
    #ifndef SPEED_MODE
    cmd.Speed_Mode_Enabled = 0;  /** @note Speed Mode is disabled */
    #else
    cmd.Speed_Mode_Enabled = 1;
    #endif
    cmd.InvertedEnable = 0;
    cmd.Direction_Command = 1;
    cmd.Speed_Command = 0;
    cmd.Torque_Command = 0;

    for( ;; )
    {

      #ifdef DEBUG_BEV
      // Serial.println("ETC Task");
      #endif

      if (!CheckState(READY_TO_GO_WAIT))
      {
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
          continue;
      }

      /** @todo Need way of determining if first time enabling motor */
      // SendInverterEnable();

      /** Detect if pedals are connected */
      Accel_0.ReadPedal();
      Accel_1.ReadPedal();
      /// @todo Brake is a high priority, maybe add interrupt 
      Brake.ReadPedal();

      #ifdef DEBUG_BEV
        // char buffer[40] = {0};
        // snprintf(buffer, 40, "%f,%f", Accel_0.GetRatio(), Accel_1.GetRatio());
        // Log.info(buffer);
      #endif

      /** @note FSAE T.4.2.6 Page 62 */
      if (abs(Accel_0 - Accel_1) > 0.10)
      {
          if (InvalidAPPS)
          {
              /** @todo Proper fault detection and system failure */
              cmd.InvertedEnable = 0;
              ChangeState(SHUTDOWN, "Pedals outside threshold for over 100ms");
          }

          /** @note FSAE T.4.2.4 Page 62 
           * Can be outside the threshold for 100ms
           */
          InvalidAPPS = true;
          vTaskDelayUntil(&xLastWakeTime, ImplausibilityTime);

      }
      else if (Brake.Applied() && (Accel_0.Applied() || Accel_1.Applied()))
      {
          /** @todo Too Harsh? */
          cmd.InvertedEnable = 0;
          ChangeState(TS_DISABLE, "Brake applied while pedal also applied");
      }

      /** Applying ETC Algorithm */
      if (ETC(&Accel_0, &Request) != OK)
      {
          RequestValid = false;
          cmd.Torque_Command = 0x0; 
      }
      else 
      {
          /** @todo Want cleaner way than writing to random global */
          RequestValid = true;
          cmd.InvertedEnable = 1;
          #ifndef SPEED_MODE
          cmd.Torque_Command = Request;
          #else
          cmd.Speed_Command = (Request * 12);  // TODO can scale rn is a percentage
          #endif
      }

      if (cmd.RollingCounter & 0xF)
      {
        cmd.RollingCounter = 0;
      }
      else
      {
        cmd.RollingCounter = cmd.RollingCounter + 1;
      }

      SendCommandMessage(&cmd);

      // Wait for the next cycle.
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    configASSERT(NULL);
 
}