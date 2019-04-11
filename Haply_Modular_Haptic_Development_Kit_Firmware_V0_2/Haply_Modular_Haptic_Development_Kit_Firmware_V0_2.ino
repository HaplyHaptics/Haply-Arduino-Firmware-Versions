/**
 ************************************************************************************************
 * @file       Haply_Arduino_Firmware.ino
 * @author     Steve Ding, Colin Gallacher
 * @version    V1.0.0
 * @date       30-July-2017
 * @brief      Haply board firmware for encoder and sensor read and torque write using on-board 
 *             actuator ports
 ************************************************************************************************
 * @attention
 *
 *
 ************************************************************************************************
 */

/* includes ************************************************************************************/ 
#include <stdlib.h>
#include <Encoder.h>
#include <pwm01.h>
#include "Haply_Modular_Haptic_Development_Kit_Firmware_V0_2.h"


/* Actuator, Encoder, Sensors parameter declarations *******************************************/
actuator actuators[TOTAL_ACTUATOR_PORTS];
encoder encoders[TOTAL_ACTUATOR_PORTS];
sensor digitalSensors[DIGITAL_PINS];
sensor analogSensors[ANALOG_PINS];


/* Actuator Status and Command declarations ****************************************************/

/* Address of device that sent data */
byte deviceAddress;

/* communication interface control, defines type of instructions recieved */
byte cmdCode;

/* communication interface control, defines response to send */
byte replyCode = 3;

/* Iterator and debug definitions **************************************************************/
long lastPublished = 0;


/* main setup and loop block  *****************************************************************/

/**
 * Main setup function, defines parameters and hardware setup
 */
void setup() {
  SerialUSB.begin(0);
}


/**
 * Main loop function
 */
void loop() {

  if(micros() - lastPublished >= 50){

    lastPublished = micros();

    if(SerialUSB.available() > 0){

      cmdCode = command_instructions();

      switch(cmdCode){
        case 0:
          deviceAddress = reset_haply(actuators, encoders, analogSensors, digitalSensors);
          break;
        case 1:
          deviceAddress = setup_device(actuators, encoders, analogSensors, digitalSensors);
          break;
        case 2:
          deviceAddress = write_states(actuators);
          replyCode = 1;
          break;
        default:
          break;
      }
    }

    switch(replyCode){
      case 0:
        break;
      case 1:
        read_states(encoders, digitalSensors, analogSensors, deviceAddress);
        replyCode = 3;
        break;
      default:
        break;
    }
  }
}


