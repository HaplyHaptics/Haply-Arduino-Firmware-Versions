/**
 ************************************************************************************************
 * @file       Haply_Arduino_Firmware.h
 * @author     Steve Ding, Colin Gallacher
 * @version    V1.5.0
 * @date       30-July-2017
 * @brief      Haply board firmware header 
 ************************************************************************************************
 * @attention
 *
 *
 ************************************************************************************************
 */

/* maximum number of ports available for on board actuator control ***************************************************/
#define TOTAL_ACTUATOR_PORTS		  4

/* maximum number of available input pins ****************************************************************************/
#define	PWM_PINS				          14
#define ANALOG_PINS					      12	

/* number of setup parameters per encoder ****************************************************************************/
#define ENCODER_PARAMETERS			  2


/* encoder pin definitions *******************************************************************************************/
#define ENCPIN1_1   			        28 // J3
#define ENCPIN1_2   			        29

#define ENCPIN2_1   			        24 // J2
#define ENCPIN2_2   			        25

#define ENCPIN3_1   			        36 // J4
#define ENCPIN3_2   			        37  

#define ENCPIN4_1   			        32 // J5
#define ENCPIN4_2   			        33 


/* PWM definitions block *********************************************************************************************/
#define PWMPIN1					          9
#define DIRPIN1					          26

#define PWMPIN2					          8
#define DIRPIN2					          22

#define PWMPIN3					          6
#define DIRPIN3					          34

#define PWMPIN4					          7
#define DIRPIN4					          30

#define PWMFREQ                   40000
#define MAX_TORQUE                0.123 //Nm
#define PWM_OUTPUT_RESOLUTION     4095


/* Rotation direction definitions ************************************************************************************/
#define CW                        0
#define CCW                       1


/* actuator struct definitions ***************************************************************************************/
typedef struct motor{
	int hostAddress;

  int rotationalDirection;
	int pwmPin;
	int dirPin;	
}actuator;


/* encoder struct definitions ****************************************************************************************/
typedef struct enc{
	int hostAddress;

  int rotationalDirection;
	float EncOffset;
	float EncResolution;
	
	Encoder *Enc;	
}encoder;


/* sensor struct definitions *****************************************************************************************/
typedef struct sen{
	int hostAddress;
	int sensorPin;
}sensor;


/* pwm struct definitions ********************************************************************************************/
typedef struct pulseWidth{
  int hostAddress;
  int pulse;
  int pwmPin;
}pwm;


/* communication function definitions ********************************************************************************/
byte command_instructions();
byte reset_haply(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]);
byte setup_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]);
byte write_states(actuator actuators[]);
void read_states(encoder encoders[], sensor analog_sensors[], byte device_address);

/* system setup command function definitions *************************************************************************/
void reset_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]);
void setup_sensors(sensor analog[], byte sensors_active, byte device_address);
void setup_pwms(pwm pulse[], byte pwms_active, byte device_address);
void setup_encoders(encoder encoders[], byte device_address, byte motors_active[], byte encoder_parameters[]);
void setup_actuators(actuator actuators[], byte device_address, byte motors_active[]);

/* component initialization function definitions *********************************************************************/
void initialize_analog(sensor *analog, byte sensor_port, byte device_address);
void initialize_pwm(pwm *pulse, byte pwm_pin, byte device_address);
void initialize_encoder(encoder *enc, byte device_address, byte parameters[], int enc1, int enc2);
void initialize_actuator(actuator *mtr, byte device_address, int pwm, int dir);

/* state control function definitions ********************************************************************************/
void create_torque(actuator *mtr, float torque);
void create_pulse(actuator actuators[], pwm *pin);
float read_analog_sensor(sensor *analog);
float read_digital_sensor(sensor *digital);
float read_encoder_value(encoder *enc);

/* parsing function definitions **************************************************************************************/
byte mtr_parameters(byte sequence[], actuator actuators[]);
byte enc_parameters(byte sequence[], encoder encoders[]);

/* Helper function definitions ***************************************************************************************/
void FloatToBytes(float val, byte segments[]);
float BytesToFloat(byte segments[]);
void ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len );


/* communication functions *******************************************************************************************/

byte command_instructions(){
	return SerialUSB.read();
}


byte reset_haply(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]){
	
	byte deviceAddress = SerialUSB.read();
	
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		actuators[i].hostAddress = 0;
		encoders[i].hostAddress = 0;
	}
	
	for(int i = 0; i < PWM_PINS; i++){
		pulse[i].hostAddress = 0;
    pulse[i].pulse = 0;
    analogWrite(pulse[i].pwmPin, pulse[i].pulse);
	}
	
	for(int i = 0; i < ANALOG_PINS; i++){
		analog[i].hostAddress = 0;
	}
	
	return deviceAddress;
}


byte setup_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]){

  reset_device(actuators, encoders, analog, pulse);
	
	byte motorsActive[4];
	byte encodersActive[4];
	pwm_set_resolution(12);
	
	byte deviceAddress = SerialUSB.read();
	byte motorNumbers = mtr_parameters(motorsActive, actuators);
	byte encoderNumbers = enc_parameters(encodersActive, encoders);
	
	byte sensorsActive = SerialUSB.read();
	setup_sensors(analog, sensorsActive, deviceAddress);

  byte pwmsActive = SerialUSB.read();
  setup_pwms(pulse, pwmsActive, deviceAddress);

	if(encoderNumbers > 0){
		byte encoderParameters[encoderNumbers * ENCODER_PARAMETERS * 4];
		
		SerialUSB.readBytes(encoderParameters, encoderNumbers * ENCODER_PARAMETERS * 4);
		setup_encoders(encoders, deviceAddress, encodersActive, encoderParameters);
	}

  setup_actuators(actuators, deviceAddress, motorsActive);
  
	return deviceAddress;
}


byte write_states(pwm pwmPins[], actuator actuators[]){
	
	int dataLength = 0;
	byte segments[4];
	byte deviceAddress = SerialUSB.read();

  for(int i = 0; i < PWM_PINS; i++){
    if(pwmPins[i].hostAddress == deviceAddress ){
      pwmPins[i].pulse = SerialUSB.read();
      create_pulse(actuators, &pwmPins[i]);
    }
  }
  
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		if(actuators[i].hostAddress == deviceAddress){
			dataLength++;
		}
	}
	
	byte torqueValues[4 * dataLength];
	SerialUSB.readBytes(torqueValues, 4 * dataLength);
	
	int j = 0;
	
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		
		if(actuators[i].hostAddress == deviceAddress){
			ArrayCopy(torqueValues, j, segments, 0, 4);
			j = j + 4;
			
			create_torque(&actuators[i], BytesToFloat(segments));
		}
	}
	
	return deviceAddress;
}


void read_states(encoder encoders[], sensor analogSensors[], byte deviceAddress){

	int dataLength = 0;
	float value;
	byte segments[4];
	
	for(int i = 0; i < ANALOG_PINS; i++){
		if(analogSensors[i].hostAddress == deviceAddress){
			dataLength++;
		}
	}
	
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		if(encoders[i].hostAddress == deviceAddress){
			dataLength++;
		}
	}
	
	byte stateValues[4 * dataLength + 1];
	
	stateValues[0] = deviceAddress;
	
	int j = 1;
	for(int i = 0; i < ANALOG_PINS; i++){
		if(analogSensors[i].hostAddress == deviceAddress){
			value = read_analog_sensor(&analogSensors[i]);
			FloatToBytes(value, segments);
			ArrayCopy(segments, 0, stateValues, j, 4);
			j = j + 4;
		}
	}
  
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		if(encoders[i].hostAddress == deviceAddress){
			value = read_encoder_value(&encoders[i]);
			FloatToBytes(value, segments);
			ArrayCopy(segments, 0, stateValues, j, 4);
			j = j + 4;
		}
	}
	
	SerialUSB.write(stateValues, 4 * dataLength + 1);
}



/* system setup command functions ************************************************************************************/
void reset_device(actuator actuators[], encoder encoders[], sensor analog[], pwm pulse[]){
  
  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    actuators[i].hostAddress = 0;
    encoders[i].hostAddress = 0;
  }

  for(int i = 0; i < PWM_PINS; i++){
    pulse[i].hostAddress = 0;
    pulse[i].pulse = 0;
    analogWrite(pulse[i].pwmPin, pulse[i].pulse);
  }

  for(int i = 0; i < ANALOG_PINS; i++){
    analog[i].hostAddress = 0;
  }
}


void setup_sensors(sensor analog[], byte sensorsActive, byte deviceAddress){

		byte sensorPort;
		
		for(int i = 0; i < sensorsActive; i++){
			
			sensorPort = SerialUSB.read();
			
			if((sensorPort >= 54) && (sensorPort <= 65)){
				initialize_analog(&analog[sensorPort - 54], sensorPort, deviceAddress);
			}
		}
}


void setup_pwms(pwm pulse[], byte pwmsActive, byte deviceAddress){

    byte pwmPin;

    for(int i = 0; i < pwmsActive; i++){

      pwmPin = SerialUSB.read();

      if((pwmPin >= 0) && (pwmPin <= 13)){
        initialize_pwm(&pulse[pwmPin], pwmPin, deviceAddress);
      }
    }
}


void setup_encoders(encoder encoders[], byte deviceAddress, byte encodersActive[], byte encoderParameters[]){
	
	byte segments[4 * ENCODER_PARAMETERS];
	
	int j = 0;
	
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		if(encodersActive[i] > 0){
			switch(i){
				case 0:
					ArrayCopy(encoderParameters, j, segments, 0, 4*ENCODER_PARAMETERS);
					initialize_encoder(&encoders[i], deviceAddress, segments, ENCPIN1_1, ENCPIN1_2);
					j = j + 4*ENCODER_PARAMETERS;		
					break;		
				case 1:
					ArrayCopy(encoderParameters, j, segments, 0, 4*ENCODER_PARAMETERS);
					initialize_encoder(&encoders[i], deviceAddress, segments, ENCPIN2_1, ENCPIN2_2);
					j = j + 4*ENCODER_PARAMETERS;
					break;
				case 2:
					ArrayCopy(encoderParameters, j, segments, 0, 4*ENCODER_PARAMETERS);
					initialize_encoder(&encoders[i], deviceAddress, segments, ENCPIN3_1, ENCPIN3_2);
					j = j + 4*ENCODER_PARAMETERS;
					break;
				case 3:
					ArrayCopy(encoderParameters, j, segments, 0, 4*ENCODER_PARAMETERS);
					initialize_encoder(&encoders[i], deviceAddress, segments, ENCPIN4_1, ENCPIN4_2);
					j = j + 4*ENCODER_PARAMETERS;
					break;
			}
		}
	}
}

void setup_actuators(actuator actuators[], byte deviceAddress, byte motorsActive[]){

  int j = 0;
  
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		if(motorsActive[i] > 0){
			switch(i){
				case 0:
					initialize_actuator(&actuators[i], deviceAddress, PWMPIN1, DIRPIN1);
          j++;
					break;
				case 1:
					initialize_actuator(&actuators[i], deviceAddress, PWMPIN2, DIRPIN2);
					j++;
					break;
				case 2:
					initialize_actuator(&actuators[i], deviceAddress, PWMPIN3, DIRPIN3);
          j++;
					break;
				case 3:
					initialize_actuator(&actuators[i], deviceAddress, PWMPIN4, DIRPIN4);
          j++;
					break;
			}
		}
	}
}



/* component initialization functions ********************************************************************************/

void initialize_analog(sensor *analog, byte sensorPort, byte deviceAddress){
	
	analog->hostAddress = deviceAddress;
	analog->sensorPin = sensorPort;
			
	pinMode(analog->sensorPin, INPUT);
	
}
 
 
void initialize_pwm(pwm *pulse, byte pwmPin, byte deviceAddress){
	
	pulse->hostAddress = deviceAddress;
	pulse->pwmPin = pwmPin;
			
	pinMode(pulse->pwmPin, OUTPUT);

  if(pwmPin == 6 || pwmPin == 7 || pwmPin == 8 || pwmPin == 9){
    pwm_setup(pulse->pwmPin, PWMFREQ, 1);
  }
}


void initialize_encoder(encoder *enc, byte deviceAddress, byte parameters[], int enc1, int enc2){
	
	int i = 0;
	byte segments[4];
	
	enc->hostAddress = deviceAddress;
	
	ArrayCopy(parameters, i, segments, 0, 4);
	enc->EncOffset = BytesToFloat(segments);
	i = i + 4;
	
	ArrayCopy(parameters, i, segments, 0, 4);
	enc->EncResolution = BytesToFloat(segments);

  if(enc->rotationalDirection > 0){
    int temp = enc1;
    enc1 = enc2;
    enc2 = temp;
  }

  enc->Enc = new Encoder(enc1, enc2);
	enc->Enc->write(enc->EncOffset * enc->EncResolution / 360);
}


void initialize_actuator(actuator *mtr, byte deviceAddress, int pwm, int dir){
	
	mtr->hostAddress = deviceAddress;
  
	mtr->pwmPin = pwm;
	mtr->dirPin = dir;

  if(dir == DIRPIN2 || dir == DIRPIN4){
    mtr->rotationalDirection = 1 - mtr->rotationalDirection;
  }
	
	pinMode(mtr->pwmPin, OUTPUT);
	pinMode(mtr->dirPin, OUTPUT);
	
	pwm_setup(mtr->pwmPin, PWMFREQ, 1);
}



/* state control functions *******************************************************************************************/

void create_torque(actuator *mtr, float torque){
	
	int duty;
  bool output = HIGH;

  if(mtr->rotationalDirection == 0){
    output = !output;  
  }
	
	if(torque <= 0){
		digitalWrite(mtr->dirPin, output);
	}
	else{
		digitalWrite(mtr->dirPin, !output);
	}
	
	torque = abs(torque);
	
	if(torque > MAX_TORQUE){
		torque = MAX_TORQUE;
	}
	
	duty = PWM_OUTPUT_RESOLUTION * torque / MAX_TORQUE;
	
	pwm_write_duty(mtr->pwmPin, duty);
}


void create_pulse(actuator actuators[], pwm *pin){

  int check = 1; 
  
  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    if(actuators[i].pwmPin == pin->pwmPin){
      check = 0;
    }
  }

  if(check){

    if(pin->pwmPin == 6 || pin->pwmPin == 7 || pin->pwmPin == 8 || pin->pwmPin == 9){
      int duty = pin->pulse * 4095 / 255;
      pwm_write_duty(pin->pwmPin, duty);
    }
    else{
      analogWrite(pin->pwmPin, pin->pulse);
    }
  }
}



float read_analog_sensor(sensor *analog){
	float value;
	value = analogRead(analog->sensorPin);
	
	return value;
}

float read_digital_sensor(sensor *digital){
	float value;
	value = digitalRead(digital->sensorPin);
	
	return value;
}

float read_encoder_value(encoder *enc){
	float thDegrees;
	thDegrees = 360.0 * enc->Enc->read()/enc->EncResolution;
	
	return thDegrees;
}



/* parsing functions *************************************************************************************************/

byte mtr_parameters(byte sequence[], actuator actuators[]){
	
	byte motorUse = SerialUSB.read();
	
	int motorCount = 0;
	
	for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
		sequence[i] = motorUse & 0x01;
		motorUse = motorUse >> 1;
		
		if(sequence[i] == 1){
			motorCount++;
		}
	}

  for(int i = 0; i < motorCount; i++){
    actuators[i].rotationalDirection = SerialUSB.read();
  }
	
	return (byte)motorCount;
}


byte enc_parameters(byte sequence[], encoder encoders[]){

  byte encoderUse = SerialUSB.read();

  int encoderCount = 0;

  for(int i = 0; i < TOTAL_ACTUATOR_PORTS; i++){
    sequence[i] = encoderUse & 0x01;
    encoderUse = encoderUse >> 1;

    if(sequence[i] == 1){
      encoderCount++;
    }
  }

  for(int i = 0; i < encoderCount; i++){
    encoders[i].rotationalDirection = SerialUSB.read();
  }
  
  return (byte)encoderCount;
}




/* Helper functions **************************************************************************************************/

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

	


