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

// set pin numbers:
const int startButton = 7;   // the number of the start button.
const int stopButton = 6;    // the number of the stop button.
const int modeButton = 5;    // the number of the mode button.

const int trapezoidalLed = 13;    // the number of the trapezoidal LED pin
const int ZZ_1_Led = 12;     // the number of the zz_1 LED pin
const int ZZ_2_Led = 11;     // the number of the zz_2 LED pin
const int ZZ_3_Led = 10;     // the number of the zz_3 LED pin

const int veloInput = A5;    // the velocity input pin. 

// Variables will change:
int modeState = 1;        // the current state of the output 
/*
  Trapezoidal mode is 1;
  ZZ_1 mode is 2;
  ZZ_2 mode is 3;
  ZZ_3 mode is 4;
  Constant trapezoidal is 5;
*/
int state = 0;               // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  pinMode(startButton, INPUT);   // the number of the start button.
  pinMode(stopButton,INPUT);
  pinMode(modeButton,INPUT);
  pinMode(trapezoidalLed,OUTPUT);
  pinMode(ZZ_1_Led,OUTPUT);
  pinMode(ZZ_2_Led,OUTPUT);
  pinMode(ZZ_2_Led,OUTPUT);
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }
  
  // set the LED:
  digitalWrite(ledPin, ledState);

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
}

