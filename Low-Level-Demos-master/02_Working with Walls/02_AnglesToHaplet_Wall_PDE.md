# Visualizing the Basic Wall

We have built a wall which is invisible in the real world, but feels like a wall to our touch. Cool eh? Now, all we need to do is visualize this wall on our computers. We've done similar tasks before in processing, the only addition here is the creation of a straight line at the appropriate height across the animation to represent the wall.

# Code
Notice how the same method is used to send and receive data as in previous lessons. Sending and recieving data between the Haply board and the computer is always a crucial part of Haptics, so it's important to understand how this works and how the data is treated to ensure accuracy in the process.
```C
// Animates a Kite (Symmetric 5-bar) Linkage, based on two bytes
// that are read from the serial port and interpreted as angular positions
// of the two input joints.
// Includes a wall at a set height

import processing.serial.*;

Serial myPort;

//Haplet dimensions in pixels
int l=100;
int L=150;
int d=40;

//Define wall dimension
int y_wall=100;

// define buffers to retrieve byte arrays from arduino side
float inByte1;
float inByte2;

// define angles for calculations
float th1;
float th2;

// required graphic objects
PShape kite, circle1, circle2;

void setup () {
  // set the window size:
  size(600, 400,P2D);

  // List all the available serial ports
  printArray(Serial.list()); // Uncomment to check out the right number for myPort

  // Open whatever port is the one you're using. (Uncomment the previous command)
  myPort = new Serial(this, Serial.list()[1], 9600);
  
  // set inital background:
  background(255);
  
  // create kite
  kite = createShape();
  kite.beginShape();
  kite.fill(255);
  kite.stroke(0);
  kite.strokeWeight(2);
  
  // Put all the vertices on O or C as an initial position
  kite.vertex(width/2, 2*height/3); // Create O (Vertex 0)
  kite.vertex(width/2, 2*height/3); // Create A (Vertex 1) 
  kite.vertex(width/2, 2*height/3); // Create E (Vertex 2) 
  kite.vertex(width/2+d, 2*height/3); // Create B (Vertex 3) 
  kite.vertex(width/2+d, 2*height/3); // Create C (Vertex 4)  
  kite.endShape(CLOSE);
  
  // draw little circles on motorized joints
  circle1 = createShape(ELLIPSE, width/2, 2*height/3, d/5, d/5); // little circle on O (motor 1)
  circle1.setStroke(color(0));
  circle2 = createShape(ELLIPSE, width/2+d, 2*height/3, d/5, d/5); // little circle on c (motor 2)
  circle2.setStroke(color(0));
  
  
}

void draw () {
  
   // Expand array size to the number of bytes you expect
  byte[] inBuffer = new byte[2];
  
  while (myPort.available() > 0) {
    inBuffer = myPort.readBytes(); // define inBuffer such that bytes sent from arduino side can be stored
    myPort.readBytes(inBuffer); // read from arduino
    if (inBuffer != null) { //if statement will trigger provided data is available
      
      // Convert the serial port readings to unsigned bytes (Remember bytes are unsigned in arduino and signed in processing!)
      inByte1=(inBuffer[0]>=0) ? float(inBuffer[0]) : float(inBuffer[0]+256); // one-liner if-else
      inByte2=(inBuffer[1]>=0) ? float(inBuffer[1]) : float(inBuffer[1]+256); // one-liner if-else
      
      th1=PI/180.0*inByte1;      //convert to radians
      th2=PI/180.0*(inByte2-80); //convert to radians (after subtracting 80 degrees that we had added on the ".ino" side.) 
      
      // forward kinematics
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
      float y_E=(-b+sqrt(Delta))/(2*a);
      float x_E=M*y_E+N;
      
      // Update graphics
      background(255); // To clean up the left-overs of drawings from the previous loop!
      kite.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
      kite.setVertex(3,width/2+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
      kite.setVertex(2,width/2+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations
      shape(kite); // Display the kite
      shape(circle1);
      shape(circle2);
      //draw wall
      stroke(0);
      line(0,2*height/3-y_wall,width,2*height/3-y_wall);
    }
  }
}
```
See the last few lines of code? Thats all it took to draw a wall, we defined the height as a global variable and drawing is a simple command.

See below for an animation of what your wall should look and behave like.

<img src="Images/BasicWall.gif">

# Next Steps
So now that we understand the basics of creating and visualizing a simple wall, its time to tweak things a bit. Remember when we said that walls are just really really stiff springs? What if we change the stiffness of that spring? In the next lesson we'll show how this can be done using a potentiometer, allowing you to change a wall into a spring in real time.
