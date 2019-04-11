//This example implements a simple sliding On/Off button. The example
// demonstrates drawing and touch operations.
//
//Thanks to Adafruit forums member Asteroid for the original sketch!
//
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <TouchScreen.h>
#include "CytronMotorDriver.h"

//pin definition
#define XP        43  // x+
#define XN        A2 //x-
#define YN        47 // y-
#define YP        A3 //y+
#define DC        46
#define MISO      50
#define MOSI      51
#define CLK       52
#define CS        53

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define CRPM                 35      // for carriage motor speed
#define DELAYMILLI           250     // set millisecond delay (used for torque and carriage)
#define GOTIME               5      // time to keep carriage motor on
#define FRAME_X              210
#define FRAME_Y              180
#define FRAME_W              100
#define FRAME_H              50

#define REDBUTTON_X FRAME_X
#define REDBUTTON_Y FRAME_Y
#define REDBUTTON_W (FRAME_W/2)
#define REDBUTTON_H FRAME_H

#define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
#define GREENBUTTON_Y FRAME_Y
#define GREENBUTTON_W (FRAME_W/2)
#define GREENBUTTON_H FRAME_H


//#define TFT_CS 10
//#define TFT_DC 9

// Create Instance of CytronMD library for carriage motor
CytronMD carriageMotor(PWM_DIR, 9, 6);  // PWM = Pin 9, DIR = Pin 6.

//initial speed
unsigned short theSpeed = 0;
bool notStarted    = true;


// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, I need to verify 
TouchScreen ts = TouchScreen(XP, YP, XN, YN, 300);


Adafruit_ILI9341 tft = Adafruit_ILI9341(CS, DC);


// draw and color frame for button
void drawFrame()
{
//  x-location, y-location, width, hieght, color
tft.drawRect(60, 50, 200, 100, ILI9341_BLACK);
tft.fillRect(60, 50, 200, 100, ILI9341_GREEN);
}

void setup(void)
{
  
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  Serial.begin(9600);
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
      startCarriageMotor();
   }
 }

  
}

void startCarriageMotor()
{
   
   tft.println("Motor  turning is on...");
   for(int i = 0; i < GOTIME; i++)
   {
       carriageMotor.setSpeed(CRPM);
       delay(DELAYMILLI);
   }
      
       tft.println("motor is  TURNING OFF...");
       carriageMotor.setSpeed(0);
       delay(DELAYMILLI);
}

void loop()
{
//   // Retrieve a point  
//  TSPoint p = ts.getPoint();
//
//  // See if there's any  touch data for us
//  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
//  {   
//    // Scale using the calibration #'s
//    // and rotate coordinate system
//    p.x = map(p.x, TS_MINY, TS_MAXY, 0, tft.height());
//    p.y = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());
//    int y = tft.height() - p.x;
//    int x = p.y;
//
//    if (RecordOn)
//    {
//      if((x > REDBUTTON_X) && (x < (REDBUTTON_X + REDBUTTON_W)))
//      {
//        if ((y > REDBUTTON_Y) && (y <= (REDBUTTON_Y + REDBUTTON_H)))
//        {
//          Serial.println("Red btn hit"); 
//          redBtn();
//          startCarriageMotor();
//        }
//      }
//    }
//    else //Record is off (RecordOn == false)
//    {
//      if((x > GREENBUTTON_X) && (x < (GREENBUTTON_X + GREENBUTTON_W)))
//      {
//        if ((y > GREENBUTTON_Y) && (y <= (GREENBUTTON_Y + GREENBUTTON_H)))
//        {
//          Serial.println("Green btn hit"); 
//          greenBtn();
//          
//        }
//      }
//    }
//
//    Serial.println(RecordOn);
//  }  
}
