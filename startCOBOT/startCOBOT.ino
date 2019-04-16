// comments are not complete for subsystem funciton calls. They will be when I have extensively tested them when I am sure that I am satisfied with encoder behavior
// encoder could stand to be optimized but given our previous complete lack of success we have given it a forgiving threshold.
// results are as expected

// Revisions to come


#include <PID_v1.h>
#include "CytronMotorDriver.h"
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Adafruit_ILI9341.h>
#include <TouchScreen.h>

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
// min and max touch screen pressure to react to
#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define TFT_CS 53
#define TFT_DC 46

// frame coordinates and dimention for drawing rectangles on LCD for buttons
#define FRAME_X 210
#define FRAME_Y 180
#define FRAME_W 100
#define FRAME_H 50

// Motor Pins
#define STEPDIR              26      // Stepper Motor Direction pin (D2 on PCB)
#define STEPPWM              7       // Stepper Motor PWM signal
#define CARPWM               9       // Carriage Motor PWM signal
#define CARDIR               6       // Carriage Motor Direction pin
#define TRQPWM               10      // Torque Motor PWM signal
#define TRQDIR               27      // Torque Motor Direction pin (D3 on PCB)


#define ENCODER0PINA         20      // this pin needs to support interrupts
#define ENCODER0PINB         17      // this pin does not need to support interrupts
#define CLOCKWISE            1       // direction constant
#define COUNTER_CLOCKWISE    2       // direction constant
#define CRPM                 55      // for carriage motor speed
#define CLIMBSPEED           50      // give cobot a boost when climbing up 
#define TRPM                 30      // for torque motor speed
#define DELAYMILLI           250     // set millisecond delay (used for torque and carriage)
#define DELAYMCRO            300      // set microsecond delay (used for stepper)
#define MAPval               90      // forgiving precision for encoder that maps 0 - 2000 to 0 - MAPval
#define CPR                  2000    // Encoder Cycles per revolution.
#define STOP                 0       
#define SAMPLETIME           1       // set sample time to refresh PID in milliseconds
// next is a control variable that is used as a bounds check for the position array
short next = 0; 
// scale each bolt location to allow a threshold greater than one degree precision 
// these values relate to the remainer cycles actually expected
int bolt1 = map(1632,0, 2000, 0, MAPval); // 3632
int bolt2 = map(1432,0, 2000, 0, MAPval); // 5432
int bolt3 = map(1065,0, 2000, 0, MAPval); // 9065
int bolt4 = map(697,0, 2000, 0, MAPval);  // 129697
int bolt5 = map(888,0, 2000, 0, MAPval);  // 16888
int bolt6 = map(688,0, 2000, 0, MAPval);  // 18688
int bolt7 = map(321,0, 2000, 0, MAPval);  // 22321
//// I have a different idea...that I might try next..
//int bolt1 = map(1000,0, 2000, 0, MAPval);
//int bolt2 = map(1500,0, 2000, 0, MAPval);
//int bolt3 = map(500,0, 2000, 0, MAPval);
//int bolt4 = map(750,0, 2000, 0, MAPval);
//int bolt5 = map(1750,0, 2000, 0, MAPval);
//int bolt6 = map(250,0, 2000, 0, MAPval);
//int bolt7 = map(1250,0, 2000, 0, MAPval);

// stuff those values into this array to iterate over
int positions[8] = {bolt1,bolt2, bolt3, bolt4, bolt5, bolt6, bolt7, 0};

// when all indices == 2 the COBOT has completed 3 passes. As of 4/04/19 this is unused but it shall be implemented in the next revision
int pass[8] = {0, 0, 0, 0, 0, 0, 0, 1};
int reqRevolutions[8]= {1, 2, 4, 6, 8, 9, 11, 0};
int REVS = 1600;      
// variables modified by interrupt handler must be declared as volatile
volatile long encoder0Position = 0;
volatile long interruptsReceived = 0;
volatile int revolutions = 0;
 
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
const int stepsPerRevolution = 1600;

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
CytronMD torqueMotor(PWM_DIR, TRQPWM, TRQDIR);  // PWM = Pin 10, DIR = Pin 7.
// Create Instance of CytronMD library for carriage motor
CytronMD carriageMotor(PWM_DIR, CARPWM, CARDIR);  // PWM = Pin 9, DIR = Pin 6.

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
 

   tft.println("Torque...");
   torqueMotor.setSpeed(50);
   delay(1000);
   torqueMotor.setSpeed(STOP);
   delay(DELAYMILLI);
   tft.fillScreen(ILI9341_BLUE);

 articulationSystemBackward();

}
void articulationSystemBackward()
{
   
  tft.println("Articulation System");
  tft.println("\n Moving Backwards");
  
  
  digitalWrite(STEPDIR, HIGH); //Pull direction pin low to move "backward"
  digitalWrite(35, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(37, HIGH);
  for(int i = 0; i < 15; i++)
  {
    for(int x = 0; x < REVS; x++)  //Loop the forward stepping enough times for motion to be visible
    {
      digitalWrite(STEPPWM,HIGH); //Trigger one step forward
      delayMicroseconds(DELAYMCRO);
      digitalWrite(STEPPWM,LOW); //Pull step pin low so it can be triggered again
      delayMicroseconds(DELAYMCRO);
     }
  }
  delay(1);
  digitalWrite(35, LOW); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(37, LOW);
// clear screen
tft.fillScreen(ILI9341_BLUE);
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
  tft.println("Articulation System");
  tft.println("\n Moving Forward");
  torqueMotor.setSpeed(TRPM);
  delay(1);
  
  digitalWrite(STEPDIR, LOW); //Pull direction pin low to move "forward"
  digitalWrite(35, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(37, HIGH);
  for(int i = 0; i < 15; i++)
  { 
    for(int x = 0; x < REVS; x++)  //Loop the forward stepping enough times for motion to be visible
    {
      digitalWrite(STEPPWM,HIGH); //Trigger one step forward
      delayMicroseconds(DELAYMCRO);
      digitalWrite(STEPPWM,LOW); //Pull step pin low so it can be triggered again
      delayMicroseconds(DELAYMCRO);
    }
  }
  delay(1);
  digitalWrite(35, LOW); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(37, LOW);
  torqueMotor.setSpeed(STOP);
  delay(1);
  // reset LCD for reading output
  tft.fillScreen(ILI9341_BLUE);
  tft.setCursor(15,7);
  // we have arrived at the bolt, now call torque to tighten
  activateTorqueMotor();
}
void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01;  // set 31KHz PWM to prevent motor noise
// torque pwm and dir pins.. just dont have #defined names. // TODO: make them #defined 
  pinMode(TRQPWM, OUTPUT);
  pinMode(TRQDIR, OUTPUT);
// stepper motor pins for direction and pulse width modulation 
  pinMode(STEPDIR, OUTPUT);
  pinMode(STEPPWM, OUTPUT);;
    // enables motors
// carriage motor pins. Will make names more unique/obvious when I define other pins
  pinMode(CARPWM, OUTPUT);
  pinMode(CARDIR, OUTPUT); 

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
  while(notStarted)
  {
      // Retrieve a point  
  TSPoint p = ts.getPoint();
  // When user presses anywhere on the screen, we will start
  // We are pretending for now that the button is implemented but since its the only touch data we need
  // this is fine for now 
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  {
      tft.fillScreen(ILI9341_BLUE);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      notStarted = false;
      activateTorqueMotor(); 
   }
 }



}

void loop()
{



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
       if(interruptsReceived > 25000)
       {
          stopForever = true;
       }
       // debugging and data gathering 

       // Move forward to bring socket to bolt unless we are done
       if(!stopForever)
       {
        tft.println("Arrived at bolt:  ");
        tft.println(next);
        tft.println("cycles completed: ");
        tft.println(interruptsReceived,DEC);
        // if we are at these bolt locations the cobot is climbing and needs to be given enough power to get going
        // being nice didnt work so the lower bound speed is gonna be 50
        if(next == 1 || next == 6)
        {
          myPID.SetOutputLimits(50, 70); // why not.. GO COBOT GO!!
          myPID.Compute();  // calculate new output maybe this is good maybe not 
          tft.println("output for speed is maybe:  ");
          tft.println(output);
        }
        else if(next == 4)
        {
          myPID.SetOutputLimits(-CRPM, CRPM);
          myPID.Compute();  // calculate new output
        }
        delay(2000);        
        tft.fillScreen(ILI9341_BLUE);
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
    finish(); 
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
    tft.println("setting motor speed to: ");
    tft.println(out);   
    analogWrite(CARPWM, out);         // Enabling motor enable pin to reach the desire angle
    tft.println("current bolt value is: ");
    tft.println(next); 
    forward();                           // calling motor to move forward
    tft.fillScreen(ILI9341_BLUE);
  }
  else
  {
    tft.println("setting motor speed to: ");
    tft.println(out); 
    tft.println("current bolt value is: ");
    tft.println(next); 
    analogWrite(CARPWM, abs(out));                        
    reverse();                            // calling motor to move reverse
    tft.fillScreen(ILI9341_BLUE);
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
  
  if(abs(encoder0Position) == 2000)
  {
    revolutions++;
  }

  // track 0 to 1999
  encoder0Position = encoder0Position % CPR;

// map encoder value to location with forgiving threshold and make sure we have completed enough revolutions for this bolt
if((abs((map(encoder0Position, 0, 1999, 0, MAPval))) == positions[next]) && (revolutions == reqRevolutions[next]) )
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
  digitalWrite(CARDIR, HIGH); 
  
}

void reverse ()
{
  digitalWrite(CARDIR, LOW); 
}
void finish ()
{

  carriageMotor.setSpeed(0);
  delay(250);
  tft.setCursor(15,7);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("COBOT done!");
  tft.println("cycles completed: ");
  tft.println(interruptsReceived,DEC);

  
}
