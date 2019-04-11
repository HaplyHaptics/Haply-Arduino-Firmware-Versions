# Sending and Recieving - Processing
Let's begin on the processing side of things. The code here matches up with the code that we'll use on the Arduino side of things; we'll be sending a number to Arduino over the serial port, verifying that it was recieved and then sending it back.

## Code

The functions at the very bottom of the code play a crucial role here, they're what we'll be using in later lessons.

```C
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

What is important to get from this lesson isn't the functions themselves as much as how to use them. Make sure to understand how the floats we're sending over are organized into the arrays to be sent across the serial port.

## Next Steps
So we have our code to run in Processing on our computers, now we need to write to corresponding firmware on our Arduino boards! Move to the [next lesson](../03_Sending%20and%20Recieving%20Floats/02_SendAndReceiveFloats.md) to see how it is done!

    
