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

/*
	TODO: Get the zigger working properly.
*/
#include <Bounce2.h>
#include <Trigger.h>

// set pin numbers:
const int START_BTN = 7;   // the number of the start button.
const int STOP_BTN = 6;  // the number of the stop button.
const int MODE_BTN = 5;    // the number of the mode button.

const int TRAP_LED = 13;     // the number of the trapezoidal LED pin
const int ZZ_1_LED = 12;     // the number of the zz_1 LED pin
const int ZZ_2_LED = 11;     // the number of the zz_2 LED pin
const int ZZ_3_LED = 10;     // the number of the zz_3 LED pin
const int MOTOR_OUTPUT = 9;  // PWM pin for output to motor

const int VELO_ANAL = A5;    // the velocity input pin.

// define state numbers
const int SETUP = 0;
const int TRAPEZOIDAL = 100;
const int ZZ_1 = 200;
const int ZZ_2 = 300;
const int ZZ_3 = 400;
const int ZZ_INF = 500;

// define the debounce time.
const int DEBOUNCE = 25;

// Instantiate a Bounce object
Bounce startBtn = Bounce();
Bounce stopBtn = Bounce();
Bounce modeBtn = Bounce();

boolean stopBtnState = false;

// Variables will change:
int modeState = 100;        // the current state of the output
int state = SETUP;        // the current reading from the input pin

Trigger startTrig = Trigger();
Trigger modeTrig = Trigger();

// counter and elapsed time for zigzagging
unsigned long elapsedTime;
const long zigzagTime = 2000; // zigzag time.
int zigzagCtr;
int motorSetpoint; // use this for the zigzag testing.
int analIn;


void setup() {
	// setup up buttons an debounce them
	pinMode(START_BTN, INPUT_PULLUP);
	startBtn.attach(START_BTN);
	startBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(MODE_BTN, INPUT_PULLUP);
	modeBtn.attach(MODE_BTN);
	modeBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(STOP_BTN, INPUT_PULLUP);
	stopBtn.attach(STOP_BTN);
	stopBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(TRAP_LED, OUTPUT);
	pinMode(ZZ_1_LED, OUTPUT);
	pinMode(ZZ_2_LED, OUTPUT);
	pinMode(ZZ_3_LED, OUTPUT);
	pinMode(MOTOR_OUTPUT, OUTPUT);
	//start the serial for the comment functuion to work.
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

	//always read the stop button.
	if (!stopBtnState)
		state = SETUP;

	switch (state) {
	case SETUP:
		// make sure the output is always set low
		// SET OUTPUT TO ZERO.
		//analogWrite(MOTOR_OUTPUT, 0);

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
		// keep the zigzagCtr clamped and ready.
		zigzagCtr = 0;
		elapsedTime = 0;
		// slightlty counter intuitively, stopBtn is normally true (pullup resistor)
		if (startTrig.Falling & stopBtn.read())
			state = modeState;

		analIn = analogRead(VELO_ANAL);   // read the input pin
		motorSetpoint = analIn; // keep the motor setpoint.
		analogWrite(MOTOR_OUTPUT, analIn / 4);

		break;

	case TRAPEZOIDAL:
		comment("Trapezoidal");
		break;

	case ZZ_1:
		comment("ZZ_1");
		zigzagMethod(3);
		break;

	case ZZ_2:
		comment("ZZ_2");
		zigzagMethod(6);
		break;

	case ZZ_3:
		comment("ZZ_3");
		zigzagMethod(9);
		break;

	case ZZ_INF:
		comment("ZZ_INF");
		zigzagMethod(99);
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

void zigzagMethod(int numberOfZigs)
{

	if (numberOfZigs = 99) // special case for infinite zigzag
	{
		zigger(numberOfZigs);
	}
	else if (zigzagCtr < numberOfZigs) // if it needs to do the zigzag motion
	{
		// if elapsed time hasn't been used this run
		if (elapsedTime = 0)
			elapsedTime = millis();
		// check to see if it's time to 'zig'.
		zigger(numberOfZigs);

	}
	else if (zigzagCtr = numberOfZigs)// should get here after the zigzags finish.
		comment("Done all my zigzags");

	else // if not a recognised value, go to setup state.
		state = SETUP;

	comment("Number of zigs: " + String(zigzagCtr));

}

void zigger(int numberOfZigs)
{
	long zzt;
	//first zigzag, only do half the time.
	if (zigzagCtr = 0)
	{
		zzt = zigzagTime / 2;
		analogWrite(MOTOR_OUTPUT, motorSetpoint);
	}
	else
		zzt = zigzagTime;

	//after the zigzag timer has elapsed, reverse and increment the counter.
	if (millis() - elapsedTime > zigzagTime)
	{
		if (numberOfZigs != 99)
			zigzagCtr++;
		
		motorSetpoint = 255 - motorSetpoint;
		analogWrite(MOTOR_OUTPUT, motorSetpoint);
		elapsedTime = 0;
		//TODO - change output direction
	}
}