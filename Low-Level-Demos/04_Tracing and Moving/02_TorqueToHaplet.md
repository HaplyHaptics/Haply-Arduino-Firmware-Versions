# Mouse Tracker: Torques to Haplet
We understand a bit about how to send torques to our Haplet, let's take that concept a little bit further; let's have our Haplet follow our mouse as we move it along the screen. We'll start in familiar territory, sending the states of our encoders to Processing and recieving the correct torque back to send to our motors. We'll focus on the Processing side of things in the next lesson.

## Code
For this task, we'll need higher precision values for passing data between Processing and Arduino. To accomplish this, we'll make use of those functions we went over in previous lessons for sending high precision floats over the serial port.

```C
#include<stdlib.h>
#include <Encoder.h>
#include <pwm01.h>

// define the encoder pins
Encoder myEnc1(28, 29); // J2
Encoder myEnc2(24, 25); // J3

// define PWM related variables
uint32_t pwmPin1=9; //J2
uint32_t dirPin1= 26;
uint32_t pwmPin2=8; //J3
uint32_t dirPin2= 22;
int32_t  pwm_freq = 40000; //Hz  
 
// forces to be received
byte inbuf[8]; //different way than previously used to initialize an 8 byte array

// forces
byte outbuf[8];
byte outbuf1[4];
byte outbuf2[4];
float th1, th2, tau1, tau2; // angles and torques to be applied

void setup() {
  SerialUSB.begin(115200); //faster baud rate to ensure data transfer is fast enough
  
  // setup encoders
  float offset = 0.0;
  myEnc1.write((180.0 - offset) * 13856.0 / 360.0);
  myEnc2.write(-offset * 13856.0 / 360.0);
  
  // setup PWMs for motors
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pwm_set_resolution(12);
  pwm_setup(pwmPin1 , pwm_freq, 1);  // Pin 8 freq set to "pwm_freq2" on clock
  pwm_setup(pwmPin2, pwm_freq, 1);  // Pin 9 freq set to "pwm_freq2" on clock B
}

// Data input and output loop control
long last_recieve = 0;

void loop() {

  // Reading angles from Encoders - turning into degrees
  th1 = 360.0/13824.0*myEnc1.read();
  th2 = 360.0/13824.0*myEnc2.read();

// sending angles to processing  
FloatToBytes(th1, outbuf1);
FloatToBytes(th2, outbuf2);
ArrayCopy(outbuf1, 0, outbuf, 0, 4);
ArrayCopy(outbuf2, 0, outbuf, 4, 4);
SerialUSB.write(outbuf,8);

  
  // Receiving torques from Processing
if( micros() - last_recieve >= 1000){ // loop control implimentation
  last_recieve = micros();
    if(SerialUSB.available() > 0){
    SerialUSB.readBytes(inbuf, 8); // receiving torques as an array
  }
}

//turning array into floats
byte inbuf1[4];
byte inbuf2[4];
ArrayCopy(inbuf, 0, inbuf1, 0, 4);
ArrayCopy(inbuf, 4, inbuf2, 0, 4);

tau1 = BytesToFloat(inbuf1);
tau2 = BytesToFloat(inbuf2);

    // command the motors (PWMs)
    if (tau1 <= 0) {
      digitalWrite(dirPin1, HIGH);
    }
    else {
      digitalWrite(dirPin1, LOW);
    }
  
    if (tau2 <=  0) {
      digitalWrite(dirPin2, HIGH);
    }
    else {
      digitalWrite(dirPin2, LOW);
    }
    
    tau1 = abs(tau1);
    tau2 = abs(tau2);
    if (tau1>1){
      tau1=1;
    }
    if (tau2>1){
      tau2=1;
    }
    int duty1=4095*tau1;
    int duty2=4095*tau2;
    
    pwm_write_duty( pwmPin1, duty1 );  
    pwm_write_duty( pwmPin2, duty2 );  
  
}


/**
 * Union definition for floating point and integer representation conversion
 */
typedef union{
    long val_l;
    float val_f;
} ufloat;

/**
 * Translates a 32-bit floating point into an array of four bytes
 * 
 * @note     None
 * @param    val: 32-bit floating point
 * @param    segments: array of four bytes
 * @return   None 
 */
void FloatToBytes(float val, byte segments[]){
    ufloat temp;

    temp.val_f = val;

    segments[3] = (byte)((temp.val_l >> 24) & 0xff);
    segments[2] = (byte)((temp.val_l >> 16) & 0xff);
    segments[1] = (byte)((temp.val_l >> 8) & 0xff);
    segments[0] = (byte)((temp.val_l) & 0xff);
}


/**
 * Translates an array of four bytes into a floating point
 * 
 * @note     None
 * @param    segment: the input array of four bytes
 * @return   Translated 32-bit floating point 
 */
float BytesToFloat(byte segments[]){
    ufloat temp;

    temp.val_l = (temp.val_l | (segments[3] & 0xff)) << 8;
    temp.val_l = (temp.val_l | (segments[2] & 0xff)) << 8;
    temp.val_l = (temp.val_l | (segments[1] & 0xff)) << 8;
    temp.val_l = (temp.val_l | (segments[0] & 0xff)); 

    return temp.val_f;
}

/**
 * Copies elements from one array to another
 * 
 * @note     None
 * @param    src: The source array to be copied from
 * @param    src_index: The starting index of the source array
 * @param    dest: The destination array to be copied to
 * @param    dest_index: The starting index of the destination array
 * @param    len: Number of elements to be copied
 * @return   None 
 */
void ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len ){
    for(int i = 0; i < len; i++){
        dest[dest_index + i] = src[src_index + i];
    }
}
```
The overall flow of the code here is, in a sense, the intended flow in the field of haptics. We're using our Haply board as a straightforward microcontroller and saving the calculations for Processing on our computer. Computers have much more processing power, so as simulations and animations become more intensive this becomes the optimal way to operate.  The controller's job is to output the current state of the end effector, and expect to recieve torques which it then applys to the devices. Running this process on a loop and letting the computer handle the calculations makes an efficient way to run complex simulations.

## Next Steps
The firmware (Arduino) side of the code is complete, now we just need to program what we want our software (Processing) to do. This will involve completeing the force calculations and the animation updates. Move on to the [next lesson](../04_Tracing%20and%Moving/03_MouseTrackerToTorque) to see how this is done!


