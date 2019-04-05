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
#define XP        43
#define XN        A2
#define YN        47
#define YP        A3
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

#define CRPM                 30      // for carriage motor speed
#define DELAYMILLI           250     // set millisecond delay (used for torque and carriage)

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
CytronMD carriageMotor(PWM_DIR, 9, 10);  // PWM = Pin 9, DIR = Pin 10.

//initial speed
unsigned short theSpeed = 0;



// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, I need to verify 
TouchScreen ts = TouchScreen(XP, YP, XN, YN, 300);


Adafruit_ILI9341 tft = Adafruit_ILI9341(CS, DC); // hope there is not actually a sep cs and dc

boolean RecordOn = false;




void drawFrame()
{
  tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, ILI9341_BLACK);
}

void redBtn()
{ 
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, ILI9341_RED);
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, ILI9341_BLUE);
  drawFrame();
  tft.setCursor(GREENBUTTON_X + 6 , GREENBUTTON_Y + (GREENBUTTON_H/2));
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.println("ready");
  RecordOn = false;
}

void greenBtn()
{
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, ILI9341_GREEN);
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, ILI9341_BLUE);
  drawFrame();
  tft.setCursor(REDBUTTON_X + 6 , REDBUTTON_Y + (REDBUTTON_H/2));
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("GO");
  RecordOn = true;
}

void setup(void)
{
  Serial.begin(9600);
  tft.begin();

  tft.fillScreen(ILI9341_BLUE);
  // origin = left,top landscape (USB left upper)
  tft.setRotation(1); 
  redBtn();
  
}

void startCarriageMotor()
{
   Serial.println("\n\n\n");
   Serial.println("motor  turning is on...");
   for(int i = 0; i < 5; i++)
   {
       Serial.println("\n\n\n");
       Serial.println("motor still is on...");
       carriageMotor.setSpeed(CRPM);
       delay(DELAYMILLI);
   }
       Serial.println("\n\n\n");
       Serial.println("motor is  TURNING OFF...");
       carriageMotor.setSpeed(0);
       delay(DELAYMILLI);
}

void loop()
{
   // Retrieve a point  
  TSPoint p = ts.getPoint();

  // See if there's any  touch data for us
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  {   
    // Scale using the calibration #'s
    // and rotate coordinate system
    p.x = map(p.x, TS_MINY, TS_MAXY, 0, tft.height());
    p.y = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());
    int y = tft.height() - p.x;
    int x = p.y;

    if (RecordOn)
    {
      if((x > REDBUTTON_X) && (x < (REDBUTTON_X + REDBUTTON_W)))
      {
        if ((y > REDBUTTON_Y) && (y <= (REDBUTTON_Y + REDBUTTON_H)))
        {
          Serial.println("Red btn hit"); 
          redBtn();
          startCarriageMotor();
        }
      }
    }
    else //Record is off (RecordOn == false)
    {
      if((x > GREENBUTTON_X) && (x < (GREENBUTTON_X + GREENBUTTON_W)))
      {
        if ((y > GREENBUTTON_Y) && (y <= (GREENBUTTON_Y + GREENBUTTON_H)))
        {
          Serial.println("Green btn hit"); 
          greenBtn();
          
        }
      }
    }

    Serial.println(RecordOn);
  }  
}
