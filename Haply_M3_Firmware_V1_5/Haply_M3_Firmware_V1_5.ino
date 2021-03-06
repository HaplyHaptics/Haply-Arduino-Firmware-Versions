/**
 ************************************************************************************************
 * @file       Haply_M3_Firmware.ino
 * @author     Steve Ding, Colin Gallacher
 * @version    V1.5.0
 * @date       11-December-2018
 * @brief      Haply M3 board firmware for encoder and sensor read and torque write using 
 *             on-board actuator ports
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
#include "Haply_M3_Firmware_V1_5.h"


/* Actuator, Encoder, Sensors parameter declarations *******************************************/
actuator actuators[TOTAL_ACTUATOR_PORTS];
encoder encoders[TOTAL_ACTUATOR_PORTS];
pwm pwmPins[PWM_PINS];
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
  reset_device(actuators, encoders, analogSensors, pwmPins);
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
          deviceAddress = reset_haply(actuators, encoders, analogSensors, pwmPins);
          break;
        case 1:
          deviceAddress = setup_device(actuators, encoders, analogSensors, pwmPins);
          break;
        case 2:
          deviceAddress = write_states(pwmPins, actuators);
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
        read_states(encoders, analogSensors, deviceAddress);
        replyCode = 3;
        break;
      default:
        break;
    }
  }
}


