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
const int START_BTN = 7;   // the number of the start button.
const int STOP_BTN = 6;  // the number of the stop button.
const int MODE_BTN = 5;    // the number of the mode button.

const int TRAP_LED = 13;    // the number of the trapezoidal LED pin
const int ZZ_1_LED = 12;     // the number of the zz_1 LED pin
const int ZZ_2_LED = 11;     // the number of the zz_2 LED pin
const int ZZ_3_LED = 10;     // the number of the zz_3 LED pin

const int VELO_ANAL = A5;    // the velocity input pin.

const int SETUP = 0;
const int TRAPEZOIDAL = 100;
const int ZZ_1 = 200;
const int ZZ_2 = 300;
const int ZZ_3 = 400;
const int ZZ_INF = 500;

// Instantiate a Bounce object
Bounce startBtn = Bounce();
Bounce stopBtn = Bounce();
Bounce modeBtn = Bounce();

boolean stopBtnState = false;
//
boolean _prevVarState;

// Variables will change:
int modeState = 100;        // the current state of the output
int state = SETUP;        // the current reading from the input pin

Trigger startTrig = Trigger();
Trigger modeTrig = Trigger();

void setup() {
  // setup up buttons an debounce them
  pinMode(START_BTN, INPUT_PULLUP);
  startBtn.attach(START_BTN);
  startBtn.interval(50);       // interval in ms

  pinMode(MODE_BTN, INPUT_PULLUP);
  modeBtn.attach(MODE_BTN);
  modeBtn.interval(50);       // interval in ms

  pinMode(STOP_BTN, INPUT_PULLUP);
  stopBtn.attach(STOP_BTN);
  stopBtn.interval(50);       // interval in ms

  pinMode(TRAP_LED, OUTPUT);
  pinMode(ZZ_1_LED, OUTPUT);
  pinMode(ZZ_2_LED, OUTPUT);
  pinMode(ZZ_3_LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  //Always update the debounced inputs
  startBtn.update();
  stopBtn.update();
  modeBtn.update();
  stopBtnState = stopBtn.read();
  // get the bool states for the triggers.
  //invert because using pullups
  startTrig.update(startBtn.read());
  modeTrig.update(modeBtn.read());
  
  //update(modeBtn.read());
  
  String msg;
  if (modeBtn.read())
    msg = "true";
  else
    msg = "false";

  comment("mode button state is: " + msg);
//  if (modeTrig._prevVarState)
//    comment("Trigger _prevState  is true");
//  else
//    comment("Trigger _prevState  is false");

  //always read the stop button.
  if (!stopBtnState)
    state = SETUP;

  switch (state) {
    case SETUP:
      // make sure the output is always set low
      // SET OUTPUT TO ZERO.
      if (modeTrig.Falling)
      {

        // change the modestate
        if (modeState < 500)
          modeState += 100;
        else
          modeState = 100;
      }
      setStateLamp(modeState);
      comment(String(modeState));
      //delay(3000);
      if (startTrig.Rising & !stopBtn.read())

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

void setStateLamp(int modeState)
{
  switch (modeState) {
    case TRAPEZOIDAL:
      digitalWrite(TRAP_LED, HIGH);
      digitalWrite(ZZ_1_LED, LOW);
      digitalWrite(ZZ_2_LED, LOW);
      digitalWrite(ZZ_3_LED, LOW);
      break;
    case ZZ_1:
      digitalWrite(TRAP_LED, LOW);
      digitalWrite(ZZ_1_LED, HIGH);
      digitalWrite(ZZ_2_LED, LOW);
      digitalWrite(ZZ_3_LED, LOW);
      break;
    case ZZ_2:
      digitalWrite(TRAP_LED, LOW);
      digitalWrite(ZZ_1_LED, LOW);
      digitalWrite(ZZ_2_LED, HIGH);
      digitalWrite(ZZ_3_LED, LOW);
      break;
    case ZZ_3:
      digitalWrite(TRAP_LED, LOW);
      digitalWrite(ZZ_1_LED, LOW);
      digitalWrite(ZZ_2_LED, LOW);
      digitalWrite(ZZ_3_LED, HIGH);
      break;
    case ZZ_INF:
      digitalWrite(TRAP_LED, LOW);
      digitalWrite(ZZ_1_LED, HIGH);
      digitalWrite(ZZ_2_LED, HIGH);
      digitalWrite(ZZ_3_LED, HIGH);
      break;
  }
}

void comment(String comment)
{
  Serial.println(comment);
}

//void update(boolean boolVar)
//{
//  //get the falling or rising.
//  if (boolVar != _prevVarState)
//  {
//    if (boolVar)
//      comment("Rising");
//    else
//      comment("Falling");
//  }
//  else
//    comment("Else");
//  //delay(2000);
//  _prevVarState = boolVar;
//}
