# Sending and Recieving - Arduino
In the previous set of code we sent two numbers to our Arduino board, now it's time to write the corresponding firmware on the Arduino board to complete the process.

## Code
The process here should look familiar to what was written for the Processing side - at least in the process of copying and moving floats in array form around. The syntax of course will look slightly different.

```C
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
## Next Steps

By the end of these last two lessons, the important takeaway is how to use the functions at the bottom of the code. As it is essentially a library, we'll only be using them when high precision is needed for sending data back and forth - the level of precision afforded by these functions won't play a difference in tasks like tracking the haplet in a Processing sketch.
