Sending and Recieving Floats
============================

This is a brief intermission of working with the Haplet to a quick
lesson series on some very useful functions; we'll use them in later
lessons. The gist of this is to send floats back and forth between the
Processing "side" and Arduino "side" with as much numerical precision as
possible. The two lessons below will outline the functions utilized to
do this -

Recommended Order
-----------------

1.  [Send / Recieve
    PDE](../03_Sending%20and%20Recieving%20Floats/01_SendAndReceiveFloats_PDE.md)

2.  [Send/ Recieve
    Arduino](../03_Sending%20and%20Recieving%20Floats/02_SendAndReceiveFloats.md)

Sending and Recieving - Processing
==================================

Let's begin on the processing side of things. The code here matches up
with the code that we'll use on the Arduino side of things; we'll be
sending a number to Arduino over the serial port, verifying that it was
recieved and then sending it back.

Code
----

The functions at the very bottom of the code play a crucial role here,
they're what we'll be using in later lessons.

``` C
// sending numbers between arduino and processing
// relevant functions at bottom of code

import processing.serial.*;
Serial myPort;

float test_f = 5.0; //test number 1 to be turned into bytes
float test_g = 6.0; //test number 2 to be turned into bytes
// numbers turned into bytes will be copied into one larger array
byte[] test_a = new byte[4]; // array defined to hold test_f and test_g
byte[] test_b = new byte[8]; // larger array to store bytes for sending
byte[] test_c = new byte[4];
float received1; // floats for numbers upon return
float received2;


void setup(){
  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort.buffer(8); // we'll be sending an 8 byte array (test_b) over the serial port
  
}


void draw(){
  test_a = FloatToBytes(test_f);
  System.arraycopy(test_a, 0, test_b, 0, 4);
  test_a = FloatToBytes(test_g);
  System.arraycopy(test_a, 0, test_b, 4, 4);
  myPort.write(test_b); //sending test be over port
   
  // now that bytes have been sent to arduino, the code below
  // will test to ensure that they have been properly treated 
  // and returned back
  if (myPort.available() > 0) {
    myPort.readBytes(test_b); // reading bytes into larger array
    System.arraycopy(test_b, 0, test_a, 0, 4); // splitting larger arry into 
    System.arraycopy(test_b, 4, test_c, 0, 4); // two smaller arrays
        
    // converting arrays into floats
    received1 = BytesToFloat(test_a);
    received2 = BytesToFloat(test_c);
    
    //printing floats    
    println(received1, received2);
  }
  
  
}

/**
   * Translates a float point number to its raw binary format and stores it across four bytes
   *
   * @param    val floating point number
   * @return   array of 4 bytes containing raw binary of floating point number
   */ 
    private byte[] FloatToBytes(float val){
  
        byte[] segments = new byte[4];
  
        int temp = Float.floatToRawIntBits(val);
  
        segments[3] = (byte)((temp >> 24) & 0xff);
        segments[2] = (byte)((temp >> 16) & 0xff);
        segments[1] = (byte)((temp >> 8) & 0xff);
        segments[0] = (byte)((temp) & 0xff);

        return segments;
  
    }


  /**
   * Translates a binary of a float point to actual float point
   *
   * @param    segment array containing raw binary of floating point
   * @return   translated floating point number
   */     
    private float BytesToFloat(byte[] segment){
  
        int temp = 0;
  
        temp = (temp | (segment[3] & 0xff)) << 8;
        temp = (temp | (segment[2] & 0xff)) << 8;
        temp = (temp | (segment[1] & 0xff)) << 8;
        temp = (temp | (segment[0] & 0xff)); 
  
        float val = Float.intBitsToFloat(temp);
  
        return val;
}
```

What is important to get from this lesson isn't the functions themselves
as much as how to use them. Make sure to understand how the floats we're
sending over are organized into the arrays to be sent across the serial
port.

Next Steps
----------

So we have our code to run in Processing on our computers, now we need
to write to corresponding firmware on our Arduino boards! Move to the
[next
lesson](../03_Sending%20and%20Recieving%20Floats/02_SendAndReceiveFloats.md)
to see how it is done!

Sending and Recieving - Arduino
===============================

In the previous set of code we sent two numbers to our Arduino board,
now it's time to write the corresponding firmware on the Arduino board
to complete the process.

Code
----

The process here should look familiar to what was written for the
Processing side - at least in the process of copying and moving floats
in array form around. The syntax of course will look slightly different.

``` C
// sending numbers between arduino and processing
// relevant functions at bottom of code

byte input1[8]; //recieving byte array
byte input2[4]; //byte array to store 1 number to turn into a float
byte input3[4]; //byte array to store 1 number to turn into a float

float fp1; //floats to be made
float fp2;

void setup() {
  SerialUSB.begin(115200);

}

void loop() {
if (SerialUSB.available() > 0) {
  SerialUSB.readBytes(input1, 8); //reading incoming byte array into large byte array
ArrayCopy(input1, 0, input2, 0, 4); // copying large array into smaller byte arrays
  fp1 = BytesToFloat(input2); // turning smaller byte array into float
ArrayCopy(input1, 4, input2, 0, 4);
  fp2 = BytesToFloat(input2);

  
if (fp1 - 5.0 < 0.1 && fp2 - 6.0< 0.1) { // loop ensures accuracy of number to what was sent
  FloatToBytes(2.0, input2); // Returning floats to byte arrays
  FloatToBytes(3.0, input3);
  ArrayCopy(input2, 0, input1, 0, 4); // Copying byte arrays into larger array
  ArrayCopy(input3, 0, input1, 4, 4);
  SerialUSB.write(input1, 8); // Sending larger array to processing
}
}


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
 * @param     segment: the input array of four bytes
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

Next Steps
----------

By the end of these last two lessons, the important takeaway is how to
use the functions at the bottom of the code. As it is essentially a
library, we'll only be using them when high precision is needed for
sending data back and forth - the level of precision afforded by these
functions won't play a difference in tasks like tracking the haplet in a
Processing sketch.
