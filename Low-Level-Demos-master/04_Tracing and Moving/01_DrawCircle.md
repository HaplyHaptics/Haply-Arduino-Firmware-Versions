# Drawing A Circle

## Setup
For this lesson set we won't be using the breadboard, only the Haplet. So we only need our power input and USB port connection to the computer.

## Coding Philosophy

The overall principle of coding here will be to first parameterize a circle with time. We'll do this using the millis() function. Once we've defined our circle, the next step should look familiar. Remember the way we defined our wall force by using distance "into" the wall, similar to a spring force equation? Exact same concept here, we would like our force __to__ the circle to be scaling with distance, so we can imagine our pulling force as a spring dragging our end effector with it! The only real difference between the wall of the past and the circle present here is the direction of force.

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

// Serial COM port buffer variable
byte buf[]={0,0}; // define a buffer array with a byte of data in each entry
// Somewhat low resolution ok, as this is just for visualization in processing

// angular variables - from encoder
float th1_degree;
float th2_degree;
float th1; // in radians
float th2; // in radians

// Link dimensions
float l=0.05; // in meters
float L=0.065;
float d=0.02;

// Circular Dimensions
float x_C,y_C;
float r=0.025;
float x_0=0.01;
float y_0=0.08;
float omega=2*PI/10.0;

// Gain variables
float k = 150.0;

void setup() {
SerialUSB.begin(9600);

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

void loop() {
float timer = millis()/1000.0; // timer for parameterizing circle

// circle trajectory -
x_C=r*cos(omega*timer)+x_0;
y_C=r*sin(omega*timer)+y_0;

// read the angles from encoder measurements
th1_degree = 360.0/13824.0*myEnc1.read();
th2_degree = 360.0/13824.0*myEnc2.read();
th1=th1_degree*3.14159/180.0;
th2=th2_degree*3.14159/180.0;

// send the angles to the processing sketch via the serial port
buf[0]=round(th1_degree);
buf[1]=round(th2_degree)+80; // we add 80 degrees just to make sure we don't hit any negative angle to pass as unsigned byte (we will subtract 80 when visualizing!)
SerialUSB.write(buf ,2);

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
// Result of forward kinematics - X and Y of the endpoint
float y_E=(-b+sqrt(Delta))/(2*a);
float x_E=M*y_E+N;

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

//Componential distance calculation

float dist_X = x_C-x_E;
float dist_Y = y_C-y_E;

//Force calculation

float f_x = k*dist_X;
float f_y = k*dist_Y;

// torques to be rendered: tau=J^T*f
float tau1=J11*f_x+J12*f_y;
float tau2=J21*f_x+J22*f_y;
tau1=-tau1;

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
```
Something to take note of here is the use of only a k value; no dampening parameter is used. This is not a huge deal, we just need to carefully tune our k value so as not to overshoot our target. If you'd like an extra exercise, try adding a dampening parameter similar to previous lessons. It should add a little bit of smoothness to your circle tracking if done correctly!

We can visualize this using processing sketches from [previous lessons](../01_Getting%20Started/04_AnglesToHaplet_PDE.md). See below for the action shot!

<img src="Images/Circle.gif" width ="200px">

## Next Steps
If you didn't add a damper in the code, thats no problem. Try adding your own *physical* damper, by either putting a higher friction material underneath the end effector, or by pushing down on the top of the end effector. See how much that smooths out the movement? I've always found this really interesting, as it is a true combination of simulation code with the surrounding environment.

There are still alot of things you can mess with in this code. Try changing the shape you paramaterize to, or the starting points of the shape. These are both shown the "circle trajectory" part of the code. See what other shapes you can draw!
