import controlP5.*;
import oscP5.*;
import netP5.*;
import geomerative.*;


OscP5 oscP5;
NetAddress myRemoteLocation;
ControlP5 cp5;
PFont f;

String chord = "Major";

float radius;

float sphereMaxRadius = 100;
float sphereMinRadius = 20;

float maxw = 20;

float x,y,z;  //coordinates of the shape (translation)
float x2, y2, z2;
float x3, y3, z3;
float x4, y4, z4;

float ph2 = 0;
float ph3 = 2*PI/3;
float ph4 = 4*PI/3;

float w = 0.5*(2*PI);

float r,g,b; //color of the shape

void setup() {
  size(800,800,P3D);
  frameRate(30);
  
  f = createFont( "Arial", 30);

  
  
  radius = width/4;
  
  x = width/2;
  y = height/2;
  z = -100;
  
  x2 = x+radius*cos(ph2);
  y2 = y-radius*sin(ph2);
  z2 = z;
  
  
  x3 = x+radius*cos(ph3);
  y3 = y-radius*sin(ph3);
  z3 = z;
  
  x4 = x+radius*cos(ph4);
  y4 = y-radius*sin(ph4);
  z4 = z;
  
  println(x2);
  println(y2);
  
  
  r = 200;
  g = 50;
  b = 100;
  
  
  
  /* start oscP5, listening for incoming messages at port 12000 */
  oscP5 = new OscP5(this,7563);
  
  /* myRemoteLocation is a NetAddress. a NetAddress takes 2 parameters,
   * an ip address and a port number. myRemoteLocation is used as parameter in
   * oscP5.send() when sending osc packets to another computer, device, 
   * application. usage see below. for testing purposes the listening port
   * and the port of the remote location address are the same, hence you will
   * send messages back to this sketch.
   */
  myRemoteLocation = new NetAddress("127.0.0.1",7563);
}

void draw() {
  background(0);
  directionalLight(255, 255, 255, 80, 100, -100);
  pointLight(255, 255, 255, 200, -200, 0);
  rectMode(CENTER);

  noStroke();
  
  
  pushMatrix();
  translate(x,y+cos(ph2)*5,z);
  //rotateX(PI/8);
  //rotateY(PI/8);
  fill(r, g, b);
  sphere(80);
  popMatrix();
  
  pushMatrix();
  fill(255, 255, 255);
  translate(x,y+300,0);
  textFont(f);
  textAlign(CENTER);
  text(chord, 0, 0);
  popMatrix();
  
  
  pushMatrix();
  translate(x2, y2, z2);
  //rotateX(PI/8);
  //rotateY(PI/8);
   fill(255, 0, 0);
  sphere(map(r, 0, 255, sphereMinRadius, sphereMaxRadius));
  popMatrix();
  
  
  pushMatrix();
  translate(x3, y3, z3);
  //rotateX(PI/8);
  //rotateY(PI/8);
  fill(0, 255, 0);
  sphere(map(g, 0, 255, sphereMinRadius, sphereMaxRadius));
  popMatrix();
  
  pushMatrix();
  translate(x4, y4, z4);
  //rotateX(PI/8);
  //rotateY(PI/8);
    fill(0, 0, 255);
  sphere(map(b, 0, 255, sphereMinRadius, sphereMaxRadius));
  popMatrix();
  
  
  
  ph2 = ph2 + w/30;
  if (ph2 > 2*PI)
    ph2 = ph2 - 2*PI;
    
  ph3 = ph3 + w/30;
  if (ph3 > 2*PI)
    ph3 = ph3 - 2*PI;
  
  ph4 = ph4 + w/30;
  if (ph4 > 2*PI)
    ph4 = ph4 - 2*PI;
    
    
  x2 = x+radius*cos(ph2);
  y2 = y-radius*sin(ph2);
  
  x3 = x+radius*cos(ph3);
  y3 = y-radius*sin(ph3);
  
  x4 = x+radius*cos(ph4);
  y4 = y-radius*sin(ph4);
    


}
void mousePressed() {
  /* in the following different ways of creating osc messages are shown by example */
  OscMessage myMessage = new OscMessage("/colour");
  
  myMessage.add(33.0);
  myMessage.add(44.0);
  myMessage.add(200.5);

  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}


/* incoming osc message are forwarded to the oscEvent method. */
void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  print("### received an osc message.");
  print(" addrpattern: "+theOscMessage.addrPattern());
  println(" typetag: "+theOscMessage.typetag());
  
  
  
  //If address of oscMessage is /colour then change the shape and colour visualized
  if (theOscMessage.checkAddrPattern("/colour"))
  {
    
    //get rgb values
    r = theOscMessage.get(0).floatValue();
    g = theOscMessage.get(1).floatValue();
    b = theOscMessage.get(2).floatValue();
  }
  
  
  //If address of oscMessage is /distance then change the size of the shape
  if (theOscMessage.checkAddrPattern("/distance"))
  {
    
    //get distance (invert sign, assuming distance in the message is passed as a positive value)
    w = -theOscMessage.get(0).floatValue()*maxw/100;
  }
  
  if (theOscMessage.checkAddrPattern("/chord"))
  {
    chord = theOscMessage.get(0).stringValue();
  }
  
  
}
