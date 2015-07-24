import processing.serial.*;
import controlP5.*;

ControlP5 cp5;
Serial serialPort;
 
// Side view visualization offsets
int Xoffset = 200;
int Yoffset = 590;

// Top view visualization offsets
int TXoffset = 800;
int TYoffset = 575;

final float a1 = 198; // Shoulder-to-elbow "bone" length
final float a2 = 220; // Elbow-to-wrist "bone" length
final float a3 = 20;  // Wrist to plate length

final float j1maxSpeed = 25.0;

// To increase precision, example position 21.32 degs
// comes in as 2132, and needs to be scaled down
final float jointScaler = 100;

// Visualized robot joint angles
float[] vTheta = new float[4];

// Robot joint angle setpoints
float[] rsTheta = new float[4];

// Robot near real time joint angles
float[] rTheta = new float[4];

// Robot theta presets
float[][] presetTheta = new float[32][4];

float j0kp = 8.0;
float j1kp = 1.0;
float j2kp = 3.0;
float j3kp = 8.0;
float lastj0output = 0.0;

float globalToolAngle = 0.0;
float maxSpeed = 30.0;
float controlPosX = 0;
float controlPosY = 500;
boolean updateKinematics = false;
boolean inverseSolution = true;
boolean j0snap = true;
boolean motorsEnabled = false;
boolean connectedToRobot = false;
boolean elbowup = true;
boolean setPointReached = false;
boolean drivePath = false;
boolean validPose = true;
boolean forceToolAngle = false;
double lastControlTime = 0.0;
int pressed = 0;
int presetCount = 0;
int nextSequencePosition = 0;
String textValue = "";

Knob j0sp;
Toggle enableMotors;
Toggle connectSerial;
Toggle driveSequence;
Toggle forceToolPose;
Textfield j3sp;
Textfield globalSpeed;
DropdownList drop0;
DropdownList drop1;

void setup() {
  size(1100, 700, P3D);
  background(157, 6, 50);
  fill(255);
  PFont font = createFont("arial",15);
  PFont bfont = createFont("arial",65);
  textFont(bfont);
  text("Initializing", 400, 325);
  textFont(font);
  
  cp5 = new ControlP5(this); 
  setupCP5();
   
  refreshSerial();
}
 
void draw() {
  background(249,249,249);
  strokeWeight(1);
  fill(color(249,249,249));
  rect(995, 15, 90, 670, 7);
  rect(800, 15, 185, 170, 7);
  strokeWeight(4);
  fill(color(50,50,50));
  
  text("Robot local joint position: " + rTheta[0] + ", " + rTheta[1] + ", " + rTheta[2] + ", " + rTheta[3], 3, 14);
  text("Robot local joint setpoint: " + rsTheta[0] + ", " + rsTheta[1] + ", " + rsTheta[2] + ", " + rsTheta[3], 3, 28);
  text("New local joint setpoint: " + vTheta[0] + ", " + vTheta[1] + ", " + vTheta[2] + ", " + vTheta[3] , 3, 42);
  text("Robot right side", 150, 640);
  text("Robot top", 770, 640);
 
  if (drivePath) {
    // Automatic drive enabled, driving through pose presets
    text("Sequential drive enabled", 400, 325);
    if (setPointReached) {
      println("Driving to setpoint " + nextSequencePosition);
      arrayCopy(presetTheta[nextSequencePosition] , rsTheta);
      nextSequencePosition++;
      if (nextSequencePosition > (presetCount-1)) nextSequencePosition = 0; 
      setPointReached = false;
    }
  } else if (mouseButton == RIGHT) {
    controlPosX = mouseX-Xoffset;
    controlPosY = -mouseY+Yoffset;
    updateKinematics = true;
  }

  // Do inverse kinematics and calculate joint thetas
  if (updateKinematics && !drivePath) {
    inverseSolution = get_angles(controlPosX, controlPosY);   
    updateKinematics = false;
  }
  
  if (!inverseSolution) {
    fill(0);
    text("Inverse kinematic solution not found", 400, 325);
    cp5.getController("driveRobot").setLock(true);
    cp5.getController("driveRobot").setColorForeground(color(180,180,180));
  } else if (!validPose) {
    fill(0);
    text("Invalid robot pose", 400, 325);
    cp5.getController("driveRobot").setLock(true);
    cp5.getController("driveRobot").setColorForeground(color(180,180,180));
  } else {
    cp5.getController("driveRobot").setLock(false);
    cp5.getController("driveRobot").setColorForeground(color(0,150,136));
  }
  
  if (motorsEnabled && ((millis() - lastControlTime) > 10)) {
    lastControlTime = millis();
    driveJoints();
    text("Motors and drive logic enabled", 500, 14);
  } else text("Motors and drive logic disabled", 500, 14);
  
  if (connectedToRobot) {
    text("Connected to robot", 500, 28);
  }
  
  // Get visualization coordinates and draw robot
  if (drivePath) {
    drawSide(rTheta);
    drawTop(rTheta);
  } else {
    drawSide(vTheta);
    drawTop(vTheta);
  } 

  delay(5);
}

void driveJoints() {
  //Calculate errors for joints and find biggest
  float[] jointErr = new float[4];
  float[] jointSpeed = new float[4];
  float maxErr = 0.0;
  
  for (int i = 0; i < 4; i++) {
    jointErr[i] = rsTheta[i] - rTheta[i];
  }
  
  for (int i = 0; i < 4; i++) {
    if (abs(jointErr[i]) > maxErr) maxErr = abs(jointErr[i]);
  }
  
  for (int i = 0; i < 4; i++) {
    jointSpeed[i] = abs(jointErr[i]/maxErr)*maxSpeed;
    if (jointSpeed[i] < 1.0) jointSpeed[i] = 1.0;
  }
  
  if (jointSpeed[1] > j1maxSpeed) {
    for (int i = 0; i < 4; i++) {
      jointSpeed[i] = (j1maxSpeed/jointSpeed[1])*jointSpeed[i];
      if (jointSpeed[i] < 1.0) jointSpeed[i] = 1.0;
    }
  }
  
  if (forceToolAngle) {
    float toolTheta = - rTheta[1] - rTheta[2] + rsTheta[3] + rsTheta[1] + rsTheta[2];
    toolTheta = constrain(toolTheta, -90.0, 90.0);
    jointErr[3] = toolTheta - rTheta[3];
    jointSpeed[3] = 90.0;
  }
  
  // Apply constraints and P control, robot accepts deg/s
  // Joint 0 position controller, output velocity
  float j0output = constrain(j0kp * jointErr[0], -jointSpeed[0], jointSpeed[0]);
  
  // Accelerate robot base
  if (abs(lastj0output) < abs(j0output)) {
    j0output = j0output*0.04 + lastj0output*0.96;
    lastj0output = j0output;
  }
  
  // Joint 1,2,3 position controller, output velocity
  float j1output = constrain(j1kp * jointErr[1], -jointSpeed[1], jointSpeed[1]);
  float j2output = constrain(j2kp * jointErr[2], -jointSpeed[2], jointSpeed[2]);
  float j3output = constrain(j3kp * jointErr[3], -jointSpeed[3], jointSpeed[3]);
  
  // Pass controller output to robot as deg/s * 100
  //println("Commanding robot: A" + (int)(j0output*100) + " B" + (int)(j1output*100) + " C0" + " D0" + "\n");
  serialPort.write("A"+ (int)(j0output*100) + " B" + (int)(j1output*100) + " C" + (int)(j2output*100) + " D" + (int)(j3output*100) + "\n");
  
  if (maxErr < 0.6) {
    println("Set point reached");
    setPointReached = true;
  }
}
 
// Given target(Px, Py) solve for theta1, theta2 with inverse kinematics
boolean get_angles(float Px, float Py) {
  float c2 = 0.0; 
  float s2 = 0.0;
  // Fit control input to robot constraints
  if (sqrt(pow(Px, 2) + pow(Py, 2)) > (a1 + a2)) {
    float inputAngle =atan2(Py, Px);
    Px = (a1+a2-0.0001)*cos(inputAngle);
    Py = (a1+a2-0.0001)*sin(inputAngle);
  }
  
  // Find theta2 where c2 = cos(theta2) and s2 = sin(theta2)
  c2 = (pow(Px, 2) + pow(Py, 2) - pow(a1, 2) - pow(a2, 2))/(2*a1*a2); // is btwn -1 and 1
 
  if (elbowup == false) {
    s2 = sqrt(1 - pow(c2, 2)); // sqrt can be + or -, and each corresponds to a different orientation
  }
  else if (elbowup == true) {
    s2 = -sqrt(1 - pow(c2, 2));
  }
  
  // Solve for the angle in degrees, automatically in correct quadrant
  vTheta[2] = degrees(atan2(s2, c2)); 
 
  // Find theta1 where c1 = cos(theta1) and s1 = sin(theta1)
  vTheta[1] = degrees(atan2(-a2*s2*Px + (a1 + a2*c2)*Py, (a1 + a2*c2)*Px + a2*s2*Py)) - 90.0;
  vTheta[3] = - vTheta[1] - vTheta[2] + globalToolAngle;
  
  // Robustness
  if ((Float.isNaN(vTheta[1])) || (Float.isNaN(vTheta[2])) || (Float.isNaN(vTheta[3]))) {
    return false;
  } else {
    if (!((vTheta[1] < -90.0) || (vTheta[2] > 120.0) || (vTheta[2] < -120.0) || (vTheta[3] < - 90.0) || (vTheta[3] > 90.0))) validPose = true;
    else validPose = false;
    return true;  
  }
}
 
void drawSide(float[] theta) {
  pushMatrix();
  rotateX(PI); // Make y axis point up 
  translate(Xoffset, -Yoffset); // Position arm side view
  fill(204);
  float joint1X = a1*cos(radians(theta[1]+90.0));
  float joint1Y = a1*sin(radians(theta[1]+90.0));
 
  float joint2X = a1*cos(radians(theta[1]+90.0)) + a2*cos(radians(theta[1]+90.0+theta[2]));
  float joint2Y = a1*sin(radians(theta[1]+90.0)) + a2*sin(radians(theta[1]+90.0+theta[2]));
  
  float joint3X1 = joint2X + a3*cos(radians(theta[1]+90.0+theta[2]+theta[3])) - 13*cos(radians(theta[1]+90.0+theta[2]+theta[3]+90.0));
  float joint3Y1 = joint2Y + a3*sin(radians(theta[1]+90.0+theta[2]+theta[3])) - 13*sin(radians(theta[1]+90.0+theta[2]+theta[3]+90.0));
  
  float joint3X2 = joint3X1 + 26*cos(radians(theta[1]+90.0+theta[2]+theta[3]+90.0));
  float joint3Y2 = joint3Y1 + 26*sin(radians(theta[1]+90.0+theta[2]+theta[3]+90.0));
  
  triangle(0, 0, 40, -25, -40, -25);
  line(0.0, 0.0, joint1X, joint1Y);
  fill(color(86,217,45)); //Bright green
  ellipse(joint1X, joint1Y, 20, 20);
  line(joint1X, joint1Y, joint2X, joint2Y);
  fill(color(237, 46, 6)); //Bright red
  ellipse(joint2X, joint2Y, 20, 20);
  line(joint3X1, joint3Y1, joint3X2, joint3Y2);
  popMatrix();
}

void drawTop(float[] theta) { 
  pushMatrix();
  rotateX(PI); // Make y axis point up
  translate(TXoffset, -TYoffset); // Position arm top view
  fill(204);
  
  rect(-43, -43, 86, 86, 7);

  float j1projX = a1*cos(radians(theta[1]+90))*cos(radians(theta[0]+90));
  float j1projY = a1*cos(radians(theta[1]+90))*sin(radians(theta[0]+90));
  
  line(0, 0, j1projX, j1projY);
  fill(color(86,217,45)); //Bright green
  ellipse(j1projX, j1projY, 20, 20);
  
  float j2projX = j1projX + a2*cos(radians(theta[1]+90+theta[2]))*cos(radians(theta[0]+90));
  float j2projY = j1projY + a2*cos(radians(theta[1]+90+theta[2]))*sin(radians(theta[0]+90));
  
  fill(color(237, 46, 6)); //Bright red
  line(j1projX, j1projY, j2projX, j2projY);
  ellipse(j2projX, j2projY, 20, 20);
  popMatrix();
}


void controlEvent(ControlEvent theEvent) {
  if(theEvent.isAssignableFrom(Textfield.class)) {
    /*println("controlEvent: accessing a string from controller '"
            +theEvent.getName()+"': "
            +theEvent.getStringValue()
            );*/
  }
  
  if (theEvent.getName() == "Global tool orientation") {
    if (!Float.isNaN(float(theEvent.getStringValue()))) {
      float toolInput = float(theEvent.getStringValue());
      println("New tool setpoint " + toolInput);
      globalToolAngle = toolInput;
      updateKinematics = true;
    } else println("Illegal new tool setpoint, ignoring");    
  }  else if (theEvent.getName() == "Global max deg/s") {
    if (!Float.isNaN(float(theEvent.getStringValue()))) {
      float speed = float(theEvent.getStringValue());
      if ((speed <= 60.0) && (speed >= 0.0)) {
        println("New global max speed " + speed);
        maxSpeed = speed;
      } else println("New global max speed out of range");
    } else println("New global max deg/s is NaN");    
  }
}

public void flipElbow() {
  println("Flipping elbow");
  elbowup = !elbowup;
  updateKinematics = true;
}

public void snapJ0() {
  println("Toggling Joing 0 snap");
  if (j0snap) j0sp.snapToTickMarks(false);
  else j0sp.snapToTickMarks(true);
  j0snap = !j0snap;
}

public void loadRobotTheta() {
  println("Loading thetas from robot");
  arrayCopy(rTheta, vTheta);
  j0sp.setValue(rTheta[0]);
}

public void zeroJoints() {
  println("Zeroing joint thetas");
  vTheta = new float[4];
  globalToolAngle = 0.0;
  controlPosX = 0;
  controlPosY = 500;
  j0sp.setValue(0);
  j3sp.setText("0");
}

public void driveRobot() {
  if (connectedToRobot && motorsEnabled) {
    println("Driving robot to visualized setpoints");
    arrayCopy(vTheta, rsTheta);
  
    // Prepare j0 for new acceleration
    lastj0output = 0.0;
  } else println("ERR: Failed to begin move, check connection and motor state"); 
}

public void enableMotors(boolean theFlag) {
  if (connectedToRobot) {
    if (theFlag) {
      println("Enabling robot motors");
      motorsEnabled = true;
      lastj0output = 0.0;
      serialPort.write("T \n");
    } else {
      println("Disabling robot motors");
      motorsEnabled = false;
      serialPort.write("F \n");
    }
  } else {
    println("ERR: Must be connected to robot");
    enableMotors.setValue(false);
  }
}

public void refreshSerial() {
  print("Available ports: ");
  println(Serial.list());
  drop0.clear();
  for (int i=0;i<Serial.list().length;i++) {
    drop0.addItem(Serial.list()[i], i);
  }
  drop0.setValue(Serial.list().length - 1);
}

public void connectSerial(boolean theFlag) {
  if (theFlag) {
    print("Connecting to ");
    println(Serial.list()[(int)drop0.getValue()]);
    serialPort = new Serial(this, Serial.list()[(int)drop0.getValue()], 9600);
    serialPort.bufferUntil('\n');
    println("Connection established");
    connectedToRobot = true;
  } else {
    println("Disconnected from robot");
    serialPort.stop();
    serialPort = null;
    connectedToRobot = false;
  }
}

public void zeroController() {
    if (connectedToRobot) {
    println("Zeroing robot axis counters");
    serialPort.write("Z \n");
  } else println("ERR: Must be connected to robot");
}

public void loadPreset() {
  if ((int)drop1.getValue() > 0) {
    println("Loading new robot pose preset");
    arrayCopy(presetTheta[(int)drop1.getValue() - 1], vTheta);
  } 
}

public void savePreset() {
  println("Saving robot pose preset");
  if ((int)drop1.getValue() == 0) {
    //New preset
    arrayCopy(vTheta, presetTheta[presetCount]);
    presetCount++;
    drop1.addItem("Pose " + presetCount, presetCount);
  } else {
    //Overwrite preset
    arrayCopy(vTheta, presetTheta[(int)drop1.getValue() - 1]);
  }
}

public void driveSequence(boolean theFlag) {
  if (motorsEnabled && connectedToRobot && theFlag && (presetCount > 0)) {
    println("Begin drive sequence");
    nextSequencePosition = 0;
    setPointReached = true;
    drivePath = true;
    cp5.getController("driveRobot").setLock(true);
    cp5.getController("driveRobot").setColorForeground(color(180,180,180));
  } else if (!theFlag) {
    println("End drive sequence");
    drivePath = false;
    if (inverseSolution && validPose) {
      cp5.getController("driveRobot").setLock(false);
      cp5.getController("driveRobot").setColorForeground(color(0,150,136));
    }
  } else {
    println("ERR: could not being drive sequence");
    driveSequence.setValue(false);
  }
}

public void forceToolPose(boolean theFlag) {
  forceToolAngle = theFlag;
}

void j0sp(int theValue) {
  //println("Setting new Joint 0 theta "+theValue);
  vTheta[0] = theValue;
}

void serialEvent(Serial serialPort) {
  String inString = serialPort.readStringUntil('\n');
  String items[] = split(inString, '\t');
 // println("Got:" + inString);
  
  if (connectedToRobot) {
    if (inString.indexOf("INFO") != -1) {
      // Getting and printing robot information directly
      println(inString);
    } else if (inString.indexOf("ERR") != -1) {
      // Robot error! 
      println(inString);
    } else if (items.length == 4) {
      // Got valid axis pos information
      rTheta[0] = int(items[0]) / jointScaler; 
      rTheta[1] = int(trim(items[1])) / jointScaler;
      rTheta[2] = int(trim(items[2])) / jointScaler;
      rTheta[3] = int(trim(items[3])) / jointScaler;
    } else {
      println("Robot communication failure, force disconnect");
      serialPort.stop();
      serialPort = null;
      connectSerial.setValue(false);
      connectedToRobot = false;
    }
  }
}

void setupCP5() {
  cp5.setColorForeground(color(0,150,136));
  cp5.setColorBackground(color(180,180,180));
  cp5.setColorLabel(color(255,255,255)); 
  cp5.setColorValue(255); //Text entry color
  cp5.setColorActive(color(150,231,45));

  cp5.addBang("loadPreset")
     .setPosition(900,20)
     .setSize(80,40)
     .setCaptionLabel("\nLoad axis pose\n       preset")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.TOP)
     ;   
     
  cp5.addBang("savePreset")
     .setPosition(900,80)
     .setSize(80,40)
     .setCaptionLabel("\nSave axis pose\n       preset")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.TOP)
     ;   
     
 driveSequence = cp5.addToggle("driveSequence")
     .setPosition(900,140)
     .setSize(80,40)
     .setLabel("Drive presets \n  in sequence")
     ; 
     
  controlP5.Label driveSequenceLabel = driveSequence.captionLabel();
  driveSequenceLabel.style().marginTop = -33; //move upwards (relative to button size)
  driveSequenceLabel.style().marginLeft = 11; //move to the right
     
  drop1 = cp5.addDropdownList("jointPresetList")
     .setPosition(808, 37)
     .setSize(80,60)
     .setColorLabel(color(0,0,0))
     ;
     
  customizePreset(drop1);
     
  cp5.addBang("flipElbow")
     .setPosition(1000,20)
     .setSize(80,40)
     .setCaptionLabel("Flip robot elbow")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
     ;   
     
  cp5.addBang("snapJ0")
     .setPosition(1000,80)
     .setSize(80,40)
     .setCaptionLabel("\nToggle robot base\n  snap to position")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.TOP)
     ;   
     
  cp5.addBang("loadRobotTheta")
     .setPosition(1000,140)
     .setSize(80,40)
     .setCaptionLabel("\n Load axis pose\n    from robot")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.TOP)
     ;  
     
  forceToolPose = cp5.addToggle("forceToolPose")
     .setPosition(1000,200)
     .setSize(80,40)
     .setLabel("IK drive tool pose")
     ;
    
  controlP5.Label tpLabel = forceToolPose.captionLabel();
  tpLabel.style().marginTop = -30; //move upwards (relative to button size)
  tpLabel.style().marginLeft = 4; //move to the right 
     
  cp5.addBang("zeroJoints")
     .setPosition(1000,260)
     .setSize(80,40)
     .setCaptionLabel("Reset robot pose")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
     ; 
     
  cp5.addBang("driveRobot")
     .setPosition(1000,320)
     .setSize(80,40)
     .setCaptionLabel("\n Drive robot\nto new pose")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.TOP)
     ; 
     
  enableMotors = cp5.addToggle("enableMotors")
     .setPosition(1000,380)
     .setSize(80,40)
     .setLabel("Enable motors")
     ; 
     
  controlP5.Label enableMotorsLabel = enableMotors.captionLabel();
  enableMotorsLabel.style().marginTop = -28; //move upwards (relative to button size)
  enableMotorsLabel.style().marginLeft = 11; //move to the right
     
  cp5.addBang("zeroController")
     .setPosition(1000,440)
     .setSize(80,40)
     .setCaptionLabel("Zero Robot State")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
     ; 
  
  cp5.addBang("refreshSerial")
     .setPosition(1000,500)
     .setSize(80,40)
     .setCaptionLabel("\n    Refresh\n serial ports")
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.TOP)
     ;    
     
  connectSerial = cp5.addToggle("connectSerial")
     .setPosition(1000,560)
     .setSize(80,40)
     .setLabel("Connect to robot")
     ; 
     
  controlP5.Label connectSerialLabel = connectSerial.captionLabel();
  connectSerialLabel.style().marginTop = -28; //move upwards (relative to button size)
  connectSerialLabel.style().marginLeft = 5; //move to the right
  
  j3sp = cp5.addTextfield("Global tool orientation")
     .setPosition(5,48)
     .setText("0")
     .setWidth(100)
     .setAutoClear(false)
     .setColorLabel(color(0,0,0))
     ;
     
  globalSpeed = cp5.addTextfield("Global max deg/s")
     .setPosition(5,90)
     .setText(str(maxSpeed))
     .setWidth(100)
     .setAutoClear(false)
     .setColorLabel(color(0,0,0))
     ;
     
  drop0 = cp5.addDropdownList("serialPortList")
     .setPosition(1000, 620)
     .setSize(80,60)
     .setColorLabel(color(0,0,0))
     ;
  
  customizeSerial(drop0);
     
  j0sp = cp5.addKnob("j0sp")
    .setRange(180,-180)
    .setValue(0)
    .setPosition(TXoffset-40, TYoffset-40)
    .setRadius(40)
    .setNumberOfTickMarks(60)
    .setTickMarkLength(4)
    .snapToTickMarks(true)
    .setViewStyle(Knob.LINE)
    .setAngleRange(2*PI)
    .setStartAngle(HALF_PI)
    .setColorValueLabel(color(0,150,136))
    .setColorBackground(color(0,150,136))
    .setColorForeground(color(255,255,255));
    ;
}

void customizeSerial(DropdownList ddl) {
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  ddl.captionLabel().set("SERIAL PORT");
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;
}

void customizePreset(DropdownList ddl) {
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  ddl.setHeight(20*8);
  ddl.captionLabel().set("JOINT PRESET");
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;
  ddl.addItem("New preset", 0);
}


