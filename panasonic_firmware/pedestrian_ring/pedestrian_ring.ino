// Sketch to control the color of the eyes based on the position of the dynamixels

#include "Timer.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <ros.h>
#include <std_msgs/UInt8.h>

// Pin for NeoPixel Data
#define PIN            8

// Number of pixels in the ring
#define NUMPIXELS      24

// NeoPixel Library setup
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ROS stuff

ros::NodeHandle nh;

uint8_t r, g, b;
//int color_prev[3] = {0, 0, 0};

int intensity = 0;

bool all_equal(int *prev_msgs){

  for (int i=0; i < 2; i++)
  {
    if (prev_msgs[i] != prev_msgs[i+1])
    {
      return false;
    }
  }
  return true;
}

void locationCb(const std_msgs::UInt8& msg_in)
{
//  color_prev[0] = color_prev[1];
//  color_prev[1] = color_prev[2];
//  color_prev[2] = msg_in.data;
  
//  if (all_equal(color_prev)) {
  
    if (msg_in.data < 2){
      r = 0; g = 1; b = 0;
    }
    else if (msg_in.data >= 2 && msg_in.data < 4) {
      r = 1; g =1; b = 0;
    }
    else {
      r = 1; g =0; b =0;
    }

//  }
}

ros::Subscriber<std_msgs::UInt8> sub("/number_pedestrians", locationCb);

// Timer
Timer t;

// Auxiliary function definitions
void setLight();
void breath();
void updateLight(int bright);

/***********************************************************************************************************/
// Main functions
void setup()
{

  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  
  Serial.begin(57600);

  pixels.begin(); // This initializes the NeoPixel library.

  t.every(60, setLight);

  nh.initNode();
  nh.subscribe(sub);

  r = 0;
  g = 1;
  b = 0;
}

void loop()
{
  t.update();
  nh.spinOnce();
  delay(20);
}


// Auxiliary functions

void setLight()
{
  updateLight(100);  
  
}

void breath()
{
  static bool flag = true;

  if (flag){
    if (++intensity>50) flag = false;
  } 
  else{
    if (--intensity<1) flag = true;
  }
}

void updateLight(int bright)
{
  for (int i=0; i<NUMPIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(r*bright, g*bright, b*bright));
  }
  pixels.show();
}

