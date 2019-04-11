# Angles to Kite

## Processing Angles to Animation
In the previous lesson we set up our bread/ Haply board combination and sent analog readings from two potentiometers as a 2-byte array along the serial port. Now that we have this array coming in, its time to write some code to start a kite visualization of the incoming data.

We'll do this using [Processing](https://processing.org/), which is based in Java rather than C. Processing was originally developed at MIT as a programming framework for visual artists, and is a great open source resource for visualization. It has since expanded due to the open source community to include many other open source tools such and audio and physics engines. It has a similar structure to [Arduino](https://www.arduino.cc/) code, you'll see below there is a similar setup() function. The draw() function is identical to the loop() function in that it continuously loops, but is different in that it refreshes at 50 Hz. This is because it was designed with visualization in mind; 50 Hz around the threshold at which the human eye sees animations become continuous.

There isn't much setup in this step, simply attach the haply board to your computer via USB.

## Kite Visualization Code

```C
// Animates a Kite (Symmetric 5-bar) Linkage, based on two bytes
// that are read from the serial port and interpreted as angular positions
// of the two input joints.

import processing.serial.*;

Serial myPort;        // The serial port

//Kite dimensions in pixels. May vary based on your screen size
//In later examples we'll work in meters. My screen is 1440 x 900px, meaning
//that my screen is around 5 pixels per mm. This means that l and L will be
//20 and 30 mm, respectively
int l=100; // length l in pixels
int L=150; // length L in pixels

// define buffers to retrieve angle byte arrays from arduino side
float inByte1;
float inByte2;

// define angles for calculations
float th1;
float th2;

// values to be found from angle calculations
float r;
float psi;
float alpha;

// Defines kite as graphic object
PShape kite;

void setup () {
  // set the window size:
  size(600, 400,P2D);

  // List all the available serial ports
  printArray(Serial.list()); // Uncomment to check out the right number for myPort

  // Open whatever port is the one you're using. (Uncomment the previous command)
  myPort = new Serial(this, Serial.list()[1], 9600);


  // set inital background:
  background(255);
  
  // Here we initialize our graphic object to draw the kite
  kite = createShape();
  kite.beginShape();
  kite.fill(255);
  kite.stroke(0);
  kite.strokeWeight(2);
  
  // Put all the vertices on O as an initial position
  kite.vertex(width/2, 2*height/3); // Create O (Vertex 0)
  kite.vertex(width/2, 2*height/3); // Create A (Vertex 1) 
  kite.vertex(width/2, 2*height/3); // Create E (Vertex 2) 
  kite.vertex(width/2, 2*height/3); // Create B (Vertex 3) 
    
  kite.endShape(CLOSE);
  
  
}

void draw () {
  
   // Expand array size to the number of bytes you expect
   // We sent a 2 byte array of pot values from our initial code in Arduino,
   // So we are expecting a 2 byte array to come over the serial port
  byte[] inBuffer = new byte[2];
  
  while (myPort.available() > 0) {
    inBuffer = myPort.readBytes(); // define inBuffer such that bytes sent from arduino side can be stored
    myPort.readBytes(inBuffer); // read from arduino
    if (inBuffer != null) { //if statement will trigger provided data is available
      
      //As a note: angles are by default in radians
      // Convert the potentiometer readings to unsigned bytes (no direct unsigned datatype in Java) and then map to desired range
      inByte1=(inBuffer[0]>=0) ? float(inBuffer[0])/255.0*PI : float(inBuffer[0]+256)/255.0*PI; // one-liner if-else
      inByte2=(inBuffer[1]>=0) ? float(inBuffer[1])/255.0*PI : float(inBuffer[1]+256)/255.0*PI; // one-liner if-else
      
      // Define angles as distance from the horizantal zero value
      th1=PI-inByte1; // Depending on how the potentiometer1 is facing the screen
      th2=PI-inByte2; // Depending on how the potentiometer2 is facing the screen
      
      background(255); // To clean up the leftovers of previous drawings
      
      
      kite.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from potentiometer input
      kite.setVertex(3,width/2+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2=0
      
      // calculations of psi, alpha, and r. values are needed for animation
      psi=(th1+th2)/2;
      alpha=(th1-th2)/2;
      r=l*cos(alpha)+sqrt(pow(L,2)-pow(l*sin(alpha),2));
       
      kite.setVertex(2,width/2+r*cos(psi), 2*height/3-r*sin(psi)); // Vertex E
      
      shape(kite); // Display the shape
    }
  }
}
```
## Visualization Complete!
While Arduino code needs to be flashed onto the Haply board, Processing code runs natively on your PC. Once you have the data transmitting from the previous lesson, run the processing code to start the visualization.

It should look something like this! You can move the head of the kite by turning the knobs of the potentiometers.

<img src = "Images/Kite_gif.gif">

