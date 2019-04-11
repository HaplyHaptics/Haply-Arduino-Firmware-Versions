# Haplet to Angles

## Reading Angles from Haplet
With a basic understanding of how to visualize output angles from potentiometers, its time to work with something more reflective of the data being sent: the Haplet. The motors at the base of the haplet can output angles, making real time tracking of the haplet possible.

You'll note that the Haplet has a Haply board built in, see the image below for setup.
<img src="Images/klaglmnccmgmpnid.jpg" width ="50%" height="50%">

## Code and Motor Considerations
Because we are working with motors with built in encoders, the encoder library will be used. Documentation for this library is available [here](https://www.pjrc.com/teensy/td_libs_Encoder.html). Compared to code from the previous sections, you'll note that some code is shown before the  setup() and loop() functions, defining a number of variables. This is more common practice, and helps to keep code clear and concise.
```c
// Encoder library necessary for haplet motor operation
#include <Encoder.h>

// Setting pins and baud rate for haply board
#define ENCJ21 28
#define ENCJ22 29
#define ENCJ31 24
#define ENCJ32 25
#define BAUD 9600

// defining 2 byte buffer for receiving angles
byte buf[] = {0,0};

// defining angles
float th1;
float th2;

// defining MyEnc 1 and 2 to match up with pins as defined above
Encoder myEnc1(ENCJ21, ENCJ22);
Encoder myEnc2(ENCJ31, ENCJ32);

void setup() {
  SerialUSB.begin(BAUD);
  // starting angles of haplet must be at 180 and 0 degrees
  myEnc1.write(180.0*13856.0/360.0);
  myEnc2.write(0);
  
}

void loop() {
  // reading angles as degrees, 13824.0 is an empirically found constant
  th1 = 360.0/13824.0*myEnc1.read();
  th2 = 360.0/13824.0*myEnc2.read();

  // putting angles into byte arrays to send
  buf[0]=round(th1);
  buf[1]=round(th2)+80;
  // +80 is to prevent negative values in datatype conversion, will be subtracted in processing
  
  SerialUSB.write(buf,2);

  delay(5);

}
```
Note the similarities and differences between this code and code used in the previous lesson. A 2-byte buffer array is used to send the encoded angle data. Use of a buffer array is used commonly as a method to send and receive data, however there is flexibility in implimentation. Here, we don't require a high resolution of the data to draw the device so we pass a single byte that represents each angle. The downside is that we can only pass an angle with an 8 bit resolution, or about 1.4 degrees (360 degrees / 256 unique values possible with 8 bits). However, here the most important lesson is that we must understand what values are being passed and then interpreted between the computer and the robot. If the robot is sending an unsigned byte (range 0-255) the Processing sketch must know this and interpret the data the same way. Since Processing only uses signed data types, we have to do a little data jujitsu to make sure that both sides of the serial communication are on the same page.

An important distinction is that the data here are angles encoded from the motors, rather than voltages read from potentiometers. The treatment of this data is slightly more complex, we can already see in this code that unit conversion is used. Additionally, you'll notice that a +80 constant appears in the treatment of theta 2. This comes about from the inability of Java (and therefore Processing) to handle unsigned byte values. In practice, this means that sending negative angle values can become a source of error in the transmission of data. If you try to print out the angles being sent from the Haplet when it's arms and therefore motor are at an angle below the zero point, you'll see negative values! To combat this, we add 80 degrees, which is sufficient to keep all angles in the range of the motor greater than zero. On Processing the only necessary step in this treatment is to subtract 80 degrees from the final angle value. But we'll get to that in the next lesson.

<img src="https://github.com/HaplyHaptics/Educational-Resources/blob/_Elie/01_Getting%20Started/Images/onnpkbpekbgkbojd.jpg" width="35%" height="35%">
Photo of an angle in which the Haplet would output a negative angle value in our code. Notice how this angle is about 80 degrees?

## Next Steps
Similarly to previous lessons, we are sending a 2-byte array from the Haply board to our computer. In next steps we'll use processing again to visualize the Haply board, move on to ["AnglesToHaplet_PDE.md"](../01_Getting%20Started/04_AnglesToHaplet_PDE.md) to see how it's done!
