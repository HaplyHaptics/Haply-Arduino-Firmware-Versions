/**
 ************************************************************************************************
 * @file       Haply_M0_Firmware.ino
 * @author     Steve Ding, Colin Gallacher
 * @version    V0.3.0
 * @date       11-December-2018
 * @brief      Haply M0 board firmware for encoder and sensor read and torque write using 
 *             on-board actuator ports
 ************************************************************************************************
 * @attention
 *
 *
 ************************************************************************************************
 */


#include <stdlib.h>
#include <Encoder.h>
#include "ADC_Boost.h"
#include "PWM_Arduino_Zero.h"
#include "Haply_M0_Firmware_V0_3.h"



/* Actuator, Encoder, Sensors parameter declarations *******************************************/
actuator actuators[TOTAL_ACTUATOR_PORTS];
encoder encoders[TOTAL_ACTUATOR_PORTS];
pwm pwmPins[PWM_PINS];
sensor analogSensors[ANALOG_PINS];


/* Actuator Status and Command declarations ****************************************************/

/* Address of device that sent data */
char deviceAddress;

/* communication interface control, defines type of instructions recieved */
char cmdCode;

/* communication interface control, defines response to send */
char replyCode = 3;

/* Iterator and debug definitions **************************************************************/
long lastPublished = 0;
long currentState = 0;


/* main setup and loop block  *****************************************************************/

/**
 * Main setup function, defines parameters and hardware setup
 */
void setup() {
  ADC_Boost();
  SerialUSB.begin(0);
}



/**
 * Main loop function
 */
void loop() {
  currentState = micros();
  
  if(currentState - lastPublished >= 50){

    lastPublished = currentState;

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
