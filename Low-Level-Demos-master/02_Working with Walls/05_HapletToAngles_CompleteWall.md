# The Great Wall of Varying Height
We're nearly done with the creation of our virtual environment, consisting of a spring wall and a dashpot. The only thing left to do is to vary the height of our wall, this will be the finishing touch on our system and allow us to "pull" on our end effector by changing it's environment rather than it's position.

## Setup
We'll be adding a third potentiometer for this setup, but if you only have two thats no problem. Simply comment out the code that controls either spring constant or dampening parameter. Just make sure that the hardcoded value is well tuned!

<img src="Images/IMG_3512.JPG">

## Code
Here, we'll be using a potentiometer for control of height, that means we'll need a 3-byte array to send data. Notice the similarities of methodology in sending our 3-byte array compared to the 2-byte array we've used before.
```C
#include <stdlib.h>
#include <Encoder.h>
#include <pwm01.h>

//pin assignment
#define ENCJ21 24
#define ENCJ22 25
#define ENCJ2pwm 8
#define ENCJ2dir 22
#define ENCJ31 28
#define ENCJ32 29
#define ENCJ3pwm 9
#define ENCJ3dir 26
#define POTk_W A0
#define POTb_W A1
#define POTh_W A2
#define TIMER 1000 //upper bound to feel realistic (micros)

//rates defined
#define BAUD 9600
uint32_t PWMfreq=40000; //unsigned integer, 32 bits. more direct way to declare data type

//data to pass to processing: motor angles and wall height
byte buf[] = {0,0,0};

//angles of arms, in degrees and radians
float th1_deg;
float th2_deg;
float th1;
float th2;

//defined dimensions - in processing will be scaled up x2000. This was empirically determined
float l= 0.05; //first arm length
float d= 0.02; //distance at base
float L= 0.065; //second arm length

//wall parameters - in processing will be scaled up x2000. This was empirically determined
float y_W=0.05; // wall's position - will be changed
int h_W; // for reading wall height

//encoder pins
Encoder myEnc1(ENCJ21, ENCJ22);
Encoder myEnc2(ENCJ31, ENCJ32);

//K and B parameters
float POTb, POTk;
float k_W; //Pot input ranges from 0 to 1023 (10 bit reading), so it will be mapped
float b_W; //Dampening parameter, will be mapped
double y_E_old;
float t_old;

void setup() {
SerialUSB.begin(BAUD);
//inital values for arms- always start flat
myEnc1.write(180.0*13856.0/360.0);
myEnc2.write(0);

//Pot input resolution
analogReadResolution(10);

//PWM motor init
pinMode(ENCJ2pwm, OUTPUT);
pinMode(ENCJ3pwm, OUTPUT);
pinMode(ENCJ2dir, OUTPUT);
pinMode(ENCJ3dir, OUTPUT);
pwm_set_resolution(12);
pwm_setup(ENCJ2pwm, PWMfreq, 1);
pwm_setup(ENCJ3pwm, PWMfreq, 1);

}

void loop() {
// time must be included due to differential term for dampening constant
float t = micros();
//setting tuning parameters
POTk= analogRead(POTk_W);
POTb= analogRead(POTb_W);
k_W= map(POTk, 0, 1023, 0, 5000);
//k_W=400.0;

//Dampening constant
b_W = map(POTb, 0, 1023, 0, 1000);
//b_W = 420.0;

//Wall height change, storing output from H_W into pot1
h_W = analogRead(POTh_W);
h_W = map(h_W, 0, 1023, 0, 255);
//Map function for floats. map() function does not work for floats
float percent = h_W/255.0;
y_W= 0.04 + percent *(0.07-0.04);

//angle determination
th1_deg = 360.0/13824.0*myEnc1.read();
th2_deg = 360.0/13824.0*myEnc2.read();
th1 = th1_deg*3.14159/180.0;
th2 = th2_deg*3.14159/180.0;

buf[0]=round(th1_deg);
buf[1]=round(th2_deg)+80;
buf[2]=round(h_W); //to be multiplied by 2000 for scaling

SerialUSB.write(buf,3); // to send to processing

// Forward Kinematics
float c1=cos(th1);
float c2=cos(th2);
float s1=sin(th1);
float s2=sin(th2);
float xA=l*c1;
float yA=l*s1;
float xB=d+l*c2;
float yB=l*s2;
float R=pow(xA,2) +pow(yA,2);
float S=pow(xB,2)+pow(yB,2);
float M=(yA-yB)/(xB-xA);
float N=0.5*(S-R)/(xB-xA);
float a=pow(M,2)+1;
float b=2*(M*N-M*xA-yA);
float c=pow(N,2)-2*N*xA+R-pow(L,2);
float Delta=pow(b,2)-4*a*c;
double y_E=(-b+sqrt(Delta))/(2*a); //brings to cm
double x_E=M*y_E+N;

// Jacobian elements J^T=[J11,J12;J21,J22]
float phi1=acos((x_E-l*c1)/L);
float phi2=acos((x_E-d-l*c2)/L);
float s21=sin(phi2-phi1);
float s12=sin(th1-phi2);
float s22=sin(th2-phi2);
float J11=-(s1*s21+sin(phi1)*s12)/s21;
float J12=(c1*s21+cos(phi1)*s12)/s21;
float J21=sin(phi1)*s22/s21;
float J22=-cos(phi1)*s22/s21;

if(t-t_old>=TIMER){ //loop control

// contact force with the virtual wall
float f_x=0;
float f_y=0;
float p_W=y_E-y_W; // penetration amount into the wall
if (p_W>0) { // if statement encapsulates below
f_y=-k_W*p_W-b_W*(10000*(y_E-y_E_old)/(t-t_old)); //adding damper term here

// torques to be rendered: tau=J^T*f
float tau1=J11*f_x+J12*f_y;
float tau2=J21*f_x+J22*f_y;
tau2=-tau2;

//directional motor command
if (tau1 <=0){
digitalWrite(ENCJ2dir, HIGH);
}
else{
digitalWrite(ENCJ2dir, LOW);
}

if (tau2 <=0){
digitalWrite(ENCJ3dir, HIGH);
}
else{
digitalWrite(ENCJ3dir, LOW);
}
//torque to apply
tau1= abs(tau1);
tau2= abs(tau2);
if (tau1>1){
tau1=1;
}
if (tau2>1){
tau2=1;
}
//torque to duty cycle
int duty1=4095*tau1;
int duty2=4095*tau2;

pwm_write_duty(ENCJ2pwm, duty1);
pwm_write_duty(ENCJ3pwm, duty2);
}
else{ //if no force on wall
pwm_write_duty(ENCJ2pwm, 0);
pwm_write_duty(ENCJ3pwm, 0);
}
t_old=t;
y_E_old= y_E;
}
}
```
Another slight difference in our code here is the use of our map function for the case of the wall height. Because the map() function is not made to work with floating point values, we made one ourselves. This is the only time we'll have to do this for this set of lessons, but an important takeaway here is that not all functions work for all datatypes.

## Next Steps
Now the only thing left to do is visualize our wall correctly, since our Processing code from previous lessons is expecting a 2-byte array and isn't coded to vary the height of the wall. Move on to the [final lesson](../02_Working%20with%20Walls/06_AnglesToHaplet_MovingWall_PDE.md) in this section to see how this is done!
