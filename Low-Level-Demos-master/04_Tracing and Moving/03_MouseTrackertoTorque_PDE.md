# Mouse Tracker to Torque
As said before, the general rule to follow in the context of haptics is to let the computer handle all the, well, computation! This includes the animation steps as well, so that all that is left is the data and application of force to the end effector. This is best handled by the microcontroller, or in our case the Haply board. We went over the Haply board in the previous lesson; here we'll be focusing our attention on the Processing side of things.

## Code
Similar to last lesson, in the code below we are using functions to send a recieve floats with high precision. The process of drawing in the code is similar to previous lessons as well.

```C
import processing.serial.*;
import java.lang.Math;

Serial myPort;

// Haplet link lengths, 50 is pixels per cm, 100 cm to meter
float l=0.05*50*100; // length l in pixels
float L=0.065*50*100; // length L in pixels
float d=0.02*50*100; // length d in pixels

// Mouse position
float x_m,y_m;

// required graphic objects
PShape kite, circle1, circle2;

// data to be input
float bufin1;
float bufin2;
float th1, th2;

void setup () {
  // set the window size:
  size(1000, 700,P2D);
  
    // List all the available serial ports
  printArray(Serial.list()); // Uncomment to check out the right number for myPort

  // Open whatever port is the one you're using. (Uncomment the previous command)
  myPort = new Serial(this, Serial.list()[1], 9600);

  // set buffer size to 8 bytes
  myPort.buffer(8);
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
  byte[] bufin = new byte[8];
  byte[] bufin_1 = new byte[4];
  byte[] bufin_2 = new byte[4];
  
  // recieving angles from haplet
  bufin = myPort.readBytes();
  myPort.readBytes(bufin);
  if (bufin != null) {
    System.arraycopy(bufin, 0, bufin_1, 0, 4); // copy first angle (4 bytes of data) into bufin_1
    System.arraycopy(bufin, 4, bufin_2, 0 ,4); // repeat process for second angle, into buffin_2
    bufin1 = BytesToFloat(bufin_1); //convert angle bytes into floats for calculations
    bufin2 = BytesToFloat(bufin_2);

      // haplet angles to haplet position
      th1 = bufin1*PI/180.0; //in radians
      th2 = bufin2*PI/180.0;
      
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
      // Result of forward kinematics - X and Y of the endpoint
      float y_h=(-b+sqrt(Delta))/(2*a);
      float x_h=M*y_h+N; // SI units 

      // Jacobian values
      float phi1=acos((x_h-l*c1)/L);
      float phi2=acos((x_h-d-l*c2)/L);
      float s21=sin(phi2-phi1);
      float s12=sin(th1-phi2);
      float s22=sin(th2-phi2);
      float J11=-(s1*s21+sin(phi1)*s12)/s21;
      float J12=(c1*s21+cos(phi1)*s12)/s21;
      float J21=sin(phi1)*s22/s21;
      float J22=-cos(phi1)*s22/s21;
  
      // trajectory - mousepointer
      x_m = mouseX-500; 
      y_m = -mouseY+470;
  
      // Torques from difference in haplet and mouse, set gain, calculate force
 
      float dist_X = x_m-x_h;
      float dist_Y = y_m-y_h;
  
      float k = .006; // tuning parameter found empirically
  
      float f_x = k*dist_X;
      float f_y = k*dist_Y; 
  
      // torques to be rendered: tau=J^T*f
      float tau1=J11*f_x+J12*f_y;
      float tau2=J21*f_x+J22*f_y;
      tau1=-tau1;
      
      // Sending torques to arduino side
      byte[] tau_1 = new byte[4];
      byte[] tau_2 = new byte[4];
      byte[] bufout = new byte [8];
      
      // changing torque values into byte arrays
      tau_1 = FloatToBytes(tau1);
      tau_2 = FloatToBytes(tau2);
      // copying torques (as byte arrays) into bufout to send
      System.arraycopy(tau_1, 0, bufout, 0, 4);
      System.arraycopy(tau_2, 0, bufout, 4, 4);
      
      myPort.write(bufout);
  
 // Update graphics
      background(255); // To clean up the left-overs of drawings from the previous loop!
      kite.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
      kite.setVertex(3,width/2+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
      kite.setVertex(2,width/2+x_h, 2*height/3-y_h); // Vertex E from Fwd Kin calculations
      shape(kite); // Display the kite
      shape(circle1);
      shape(circle2);
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
Comparing this code to that which was used on the Arduino side to draw a circle, you'll see that the methodology to calculate force is identical. By taking the angles which were recieved from the Haplet and applying forward kinematics, we are able to calculate forces and then torques to send back to the Haplet. We know from our previous lesson that the Haplet is well equipped to apply these to the end effector.

See below to see this code in action!

<img src="Images/Tracker.gif" width ="200px">

## Next Steps
Running this code, you'll see that there are areas of error with the tracking. Based on previous lessons, what are some ways to improve this code? Remember the methodologies used to apply a damper to the system, you'll find that any of these will improve the tracking.
