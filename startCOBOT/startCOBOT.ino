// comments are not complete for subsystem funciton calls. They will be when I have extensively tested them when I am sure that I am satisfied with encoder behavior
// encoder could stand to be optimized but given our previous complete lack of success we have given it a forgiving threshold.
// results are as expected

// Revisions to come


#include <PID_v1.h>
#include "CytronMotorDriver.h"
#include <Stepper.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Adafruit_ILI9341.h>
#include <TouchScreen.h>


#define MotEnable 9 //Motor Enamble pin Runs on PWM signal
#define MotDir  6  // Motor Direction pin


//Touchscreen X+ X- Y+ Y- pins
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 47   // can be a digital pin
#define XP 43   // can be a digital pin

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define TFT_CS 53
#define TFT_DC 46


#define FRAME_X 210
#define FRAME_Y 180
#define FRAME_W 100
#define FRAME_H 50



#define ENCODER0PINA         20      // this pin needs to support interrupts
#define ENCODER0PINB         17      // this pin does not need to support interrupts
#define CLOCKWISE            1       // direction constant
#define COUNTER_CLOCKWISE    2       // direction constant
#define REVOLUTIONS          10      // 30 = a distance of 2 inches
#define CRPM                 35      // for carriage motor speed
#define TRPM                 30      // for torque motor speed
#define SRPM                 200     // for stepper motor speed
#define DELAYMILLI           250     // set millisecond delay (used for torque and carriage)
#define DELAYMCRO            10      // set microsecond delay (used for stepper)
#define MAPval               90      // forgiving precision for encoder that maps 0 - 2000 to 0 - MAPval
#define CPR                  2000    // Encoder Cycles per revolution.
#define STOP                 0       
#define SAMPLETIME           1       // set sample time to refresh PID in milliseconds
// next is a control variable that is used as a bounds check for the position array
short next = 0; 
// scale each bolt location to allow a threshold greater than one degree precision 
int bolt1 = map(1000,0, 2000, 0, MAPval);
int bolt2 = map(1500,0, 2000, 0, MAPval);
int bolt3 = map(500,0, 2000, 0, MAPval);
int bolt4 = map(750,0, 2000, 0, MAPval);
int bolt5 = map(1750,0, 2000, 0, MAPval);
int bolt6 = map(250,0, 2000, 0, MAPval);
int bolt7 = map(1250,0, 2000, 0, MAPval);

// stuff those values into this array to iterate over
int positions[8] = {bolt1,bolt2, bolt3, bolt4, bolt5, bolt6, bolt7, 0};

// when all indices == 2 the COBOT has completed 3 passes. As of 4/04/19 this is unused but it shall be implemented in the next revision
int pass[8] = {0, 0, 0, 0, 0, 0, 0, 1};

// variables modified by interrupt handler must be declared as volatile
volatile long encoder0Position = 0;
volatile long interruptsReceived = 0;
 
// track last position so we know whether it's worth printing new output, used when debugging
int previousPosition = 0;

// logic control
bool locationFound = false;
bool keepLooking   = true;
bool stopForever   = false;
bool notStarted    = true;
// track direction: 0 = counter-clockwise; 1 = clockwise
short currentDirection = CLOCKWISE;

// Number of steps per output rotation // TODO: should change to a #define and all caps. 
const int stepsPerRevolution = 200;

// PID //TODO: explain constants
double kp = .5 , ki = 1 , kd = .5; //kd prev vals: 1; 0.01;  kp prev vals: = 5, 1 KP = 2 BADD (WHEN KI = 1 AND KD = .5)
// BEST SO FAR might be kp = .5, ki = 1, kd = .5
// see details in ardiono PID library files 
double input = 0, output = 0, setpoint = 0;
// touch screen object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Create Instance of PID library
// Using Arduinos PID library. PID is a proportional–integral–derivative control method that automatically applies accurate and responsive correction to a control function
// Our control function is gathering data from an interrupt connected to an optical encoder to determine when we have arrived at the next bolt location
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT); 

// Configure the motor driver.
// Create Instance of CytronMD library for torque motor
CytronMD torqueMotor(PWM_DIR, 10, 7);  // PWM = Pin 10, DIR = Pin 7.
// Create Instance of CytronMD library for carriage motor
CytronMD carriageMotor(PWM_DIR, 9, 6);  // PWM = Pin 9, DIR = Pin 6.

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 35, 37, 39, 41);
// draw and color frame for button
void drawFrame()
{
//  x-location, y-location, width, hieght, color
tft.drawRect(60, 50, 200, 100, ILI9341_BLACK);
tft.fillRect(60, 50, 200, 100, ILI9341_GREEN);
}
void activateTorqueMotor()
{
   tft.setCursor(15,7);
   tft.fillScreen(ILI9341_BLUE);

   tft.println("Torque...");
   torqueMotor.setSpeed(TRPM);
   delay(1000);
   torqueMotor.setSpeed(STOP);
   delay(DELAYMILLI);
   // keepLooking = true;
 articulationSystemBackward();

}
void articulationSystemBackward()
{
  tft.setCursor(15,7);
  tft.fillScreen(ILI9341_BLUE);
  tft.println("Articulation System");
  tft.println("\n Moving Backwards");
  
  myStepper.setSpeed(SRPM);
  for(int i = 0; i < REVOLUTIONS; i++)
  {
          myStepper.step(-stepsPerRevolution);
          delayMicroseconds(DELAYMCRO);

  }
startCarriageMotor();

}

void startCarriageMotor()
 {
//  tft.println("\n\n\n");
//  tft.println("About to go look for location of next bolt...");
//  
//  carriageMotor.setSpeed(CRPM);
//  delay(DELAYMILLI);
//  tft.println("Leaving.");
  keepLooking = true;
 }
 void articulationSystemForward()
{
  tft.fillScreen(ILI9341_BLUE);
  tft.setCursor(15,7);
  myStepper.setSpeed(SRPM);
  tft.println("Articulation System");
  tft.println("\n Moving Forward..");
  for(int i = 0; i < REVOLUTIONS; i++)
  {
  
    myStepper.step(stepsPerRevolution);
    delayMicroseconds(DELAYMCRO);
  }
  tft.fillScreen(ILI9341_BLUE);
activateTorqueMotor();
}
void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01;  // set 31KHz PWM to prevent motor noise
// torque pwm and dir pins.. just dont have #defined names. // TODO: make them #defined 
  pinMode(10, OUTPUT);
  pinMode(7, OUTPUT);
// stepper motor pins.. just dont have #defined names. // TODO: make them #defined 
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(41, OUTPUT);
// carriage motor pins. Will make names more unique/obvious when I define other pins
  pinMode(MotEnable, OUTPUT);
  pinMode(MotDir, OUTPUT); 

  Serial.begin(9600); //initialize serial communication to enable diagnostic output

  pinMode(ENCODER0PINA, INPUT_PULLUP); 
  pinMode(ENCODER0PINB, INPUT_PULLUP);

  digitalWrite(ENCODER0PINA, HIGH); 
  digitalWrite(ENCODER0PINB, HIGH); 

  attachInterrupt(3, updateEncoder, RISING); 
    
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(SAMPLETIME);  // refresh rate of PID controller in milliseconds
  myPID.SetOutputLimits(-CRPM, CRPM); //here change in value reflect change in speed limits of motor 
  tft.begin();

  tft.fillScreen(ILI9341_BLUE);
  // origin = left,top landscape (USB left upper)
  tft.setRotation(1); 
  tft.setCursor(15,7);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("   Touch button to start");
  drawFrame();
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(4);
  tft.println("\n \n    START");



}

void loop()
{
    // Retrieve a point  
  TSPoint p = ts.getPoint();
  // When user presses anywhere on the screen, we will start
  // We are pretending for now that the button is implemented but since its the only touch data we need
  // this is fine for now 
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  {   
     

     if(notStarted)
     {
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      notStarted = false;
      activateTorqueMotor();  
     }
  }


if (locationFound)
  {
     tft.fillScreen(ILI9341_BLUE);

//    tft.println("FOUND LOCATION?");
//    tft.print(encoder0Position, DEC);
//    tft.print("\t");
//    tft.print(currentDirection == CLOCKWISE ? "clockwise" : "counter-clockwise");

    // did we really find the location? If not let PID handle it, otherwise we got lucky so here we are 
    // Dont' worry Reed there is no offset this is actual location compared to a location PID would find acceptable
    if(abs(map(encoder0Position, 0, 2000, 0, MAPval)) == positions[next])
    {
       // reset location found  and disable other functions that are not desired when we are at a bolt location
       locationFound = false;
       keepLooking = false;
       // debugging and data gathering 
//       tft.println("YES! position that mapped was:  ");
//       tft.println(encoder0Position);
       tft.println("cycles completed: ");
       tft.println(interruptsReceived,DEC);

       // dont go out of bounds in our positions array! C be dangerous
       if(next == 7)
       {
        next = 0;
       }
       else
       {
        next++;
       }
       // This code will be used to test on the floor with the chain and i dont want the COBOT to eat my hand so I am letting it visit each bolt one time
       // and it should visit each bolt after 5,441 cycles but I dont trust the cobot so im making sure this dude stops. He is guilty until proven innocent
       if(interruptsReceived > 5500)
       {
          stopForever = true;
       }
       // debugging and data gathering 

       // Move forward to bring socket to bolt unless we are done
       if(!stopForever)
       {
        tft.println("Arrived at bolt:  ");
        tft.println(next);
        articulationSystemForward();
       }
    }
    else
    {
//      tft.print("Nope.. not good enough value: \t");
//      tft.println((map(encoder0Position, 0, 2000, 0, MAPval)));
//      tft.print("actual value (that was mapped): \t");
//      tft.println(encoder0Position);
    }
  }
   // debug stuff
  if (encoder0Position !=  previousPosition && keepLooking == true)
  {

//    tft.println("Current Position: ");
//    tft.print(encoder0Position, DEC);
//    tft.print("\t");
//    tft.print(currentDirection == CLOCKWISE ? "clockwise" : "counter-clockwise");
//    tft.print("\t");
//    tft.println(interruptsReceived, DEC);

      previousPosition = encoder0Position;

  }

  // The encoder position is the control signal being used for PID signal 
  // (input gets mathed and then output tells PID stuff)
  input = encoder0Position;            

  myPID.Compute();  // calculate new output
  
  // If stopForever is true then you know.. we wanna stop forever
  if(stopForever)
  {
    tft.fillScreen(ILI9341_BLUE);
    carriageMotor.setSpeed(0);
    tft.println("COBOT done!");
    
  }
// only do this stuff if COBOT is not busy with other stuff
if(keepLooking == true)
{
  setpoint = positions[next]; 
    pwmOut(output);  
}
  
  
}
void pwmOut(int out)
{                               
  if (out > 0)
  { 
   // tft.println("setting motor speed to: ");
  //  tft.println(out);   
    analogWrite(MotEnable, out);         // Enabling motor enable pin to reach the desire angle
    forward();                           // calling motor to move forward
  }
  else
  {
  //  tft.println("setting motor speed to: ");
//tft.println(out); 
    analogWrite(MotEnable, abs(out));                        
    reverse();                            // calling motor to move reverse
  }
  
  

}


 
 



void updateEncoder()
{
// read both inputs
  int a = digitalRead(ENCODER0PINA);
  int b = digitalRead(ENCODER0PINB);
 
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    encoder0Position--;
    currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    encoder0Position++;
    currentDirection = CLOCKWISE;
  }
 
  // track 0 to 1999
  encoder0Position = encoder0Position % CPR;

// changing positions[next] to setspeed MAY improve accuracy. but I gotta think more. 
if(abs((map(encoder0Position, 0, 2000, 0, MAPval))) == positions[next])
 {
  locationFound = true;
  // This stays
  carriageMotor.setSpeed(STOP);
  delay(DELAYMILLI);
 }

  interruptsReceived++;

}

void forward ()
{
  digitalWrite(MotDir, HIGH); 
  
}

void reverse ()
{
  digitalWrite(MotDir, LOW); 
 
  
}
// I dont even use this plus it doesn't do what I want it to. It stays for now
void finish ()
{
  tft.print("Stopping");
  digitalWrite(MotDir, LOW); 
 digitalWrite(MotEnable, LOW); 
  
}
