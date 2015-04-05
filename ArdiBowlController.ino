

/*
 Title: Silo Bowl Controller
 Description:
 This code shall control the bowl production machine.
 Several modes are available:
 1) trapezoidal
 - on "Start" edge, and not "Stop"
 - Ramp up using a set acceleration
 - Ramp to set velocity.
 - On "Stop", ramp down using set velocity.
 2) ZigZag
 - Zigzag mode is selected by pressing the mode button
 while the machine is not running.
 - ZZ_1 will cause the machine to ramp in either direction a set number of times
 - ZZ_2 will cause the machine to ramp in either direction
 twice as many times.
 - ZZ_3 will cause the machine to ramp in either direction
 twice as many times as ZZ_2.
 3) if the emergency stop button is pressed, the machine is clamped to zero speed.  
 */
#include <Bounce2.h>
#include <Trigger.h>

// set pin numbers:
#define START_BTN 7   // the number of the start button.
#define STOP_BTN 6  // the number of the stop button.
#define MODE_BTN 5;    // the number of the mode button.

#define TRAP_LED 13    // the number of the trapezoidal LED pin
#define ZZ_1_Led 12     // the number of the zz_1 LED pin
#define ZZ_2_Led 11     // the number of the zz_2 LED pin
#define ZZ_3_Led 10     // the number of the zz_3 LED pin

#define VELO_ANAL A5    // the velocity input pin. 

#define SETUP 0
#define TRAPEZOIDAL 100
#define ZZ_1 200
#define ZZ_2 300
#define ZZ_3 400
#define ZZ_INF 500

// Instantiate a Bounce object
Bounce startBtn = Bounce(); 
Bounce stopBtn = Bounce();
Bounce modeBtn = Bounce();

int lastStartButton = false;
int stopButtonState = false;
int lastStopButton = false;
int lastModeButton = false;

// Variables will change:
int modeState = 1;        // the current state of the output 
int state = SETUP;        // the current reading from the input pin

Trigger startTrig = Trigger();
Trigger modeTrig = Trigger();

void setup() {
  // setup up buttons an debounce them
  pinMode(START_BTN,INPUT);   
  startBtn.attach(START_BTN);
  startBtn.interval(20);       // interval in ms

  pinMode(STOP_BTN,INPUT);
  stopBtn.attach(STOP_BTN);
  stopBtn.interval(20);       // interval in ms

  pinMode(MODE_BTN,INPUT);
  //modeBtn.attach(MODE_BTN);
  //modeBtn.interval(20);       // interval in ms

  pinMode(TRAP_LED,OUTPUT);
  pinMode(ZZ_1_Led,OUTPUT);
  pinMode(ZZ_2_Led,OUTPUT);
  pinMode(ZZ_2_Led,OUTPUT);
}

void loop() {
  //Always update the denounced inputs
  startBtn.update();
  stopBtn.update();
  modeBtn.update();

  // set up the triggers.
  startTrig.update(startBtn.read());
  modeTrig.update(modeBtn.read());

  //always read the stop button.
  if(!stopBtn.read())
    state = SETUP;

  switch (state) {
  case SETUP:
    // make sure the output is always set low
    // SET OUTPUT TO ZERO.
    if(modeTrig.Rising)

      //do something when var equals 1
      break;

  case TRAPEZOIDAL:
    //do something when var equals 2
    break;

  case ZZ_1:
    //do something when var equals 2
    break;

  case ZZ_2:
    //do something when var equals 2
    break;

  case ZZ_3:
    //do something when var equals 2
    break;

  case ZZ_INF:
    //do something when var equals 2
    break;

  default: 

    state = SETUP;
    // if nothing else matches, do the default
    // default is optional
  }
}


