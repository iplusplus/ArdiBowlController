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




/*
NOT NEEDED FOR INDUSTRIAL SHIELDS
const int TRAP_LED = 13;     // the number of the trapezoidal LED pin
const int ZZ_1_LED = 12;     // the number of the zz_1 LED pin
const int ZZ_2_LED = 11;     // the number of the zz_2 LED pin
const int ZZ_3_LED = 10;     // the number of the zz_3 LED pin

*/

// set digital input pin numbers:
const int START_BTN = 4;	// I0.3 the number of the start button.
const int STOP_BTN = 8;		// I0.2  the number of the stop button.
const int MODE_BTN = 12;	// I0.1 the number of the mode button.

//set analogue input pins.
const int VELO_ANAL = A1;    // I0.8 the velocity input pin.
const int TIME_ANAL = A0;    // I0.9 the velocity input pin.

// set motor output pin number.
const int ANAL_10V_REF = 11;	// Q0.1 just set to max to provide 10V refernce voltage.
const int MOTOR_OUTPUT = 13;	// Q0.0 PWM pin for output to motor

// set drive start and direction pins.
const int MOTOR_RUN = 3;		// Q0.6 (SET 10VDC) motor runnnig
const int MOTOR_DIRECTION = 5;	// Q0.5 (SET 10VDC) motor direction.

// define state numbers
#define SETUP 0
#define TRAPEZOIDAL 100
#define ZZ_1 200
#define ZZ_2 300
#define ZZ_3 400
#define ZZ_INF 500
#define ZZ_1_NUM 3 // number of zigzagz, ZZ_1
#define ZZ_2_NUM 6 // number of zigzagz, ZZ_2
#define ZZ_3_NUM 9 // number of zigzagz, ZZ_3
#define ZZ_INF_NUM 99 //DON'T CHANGE!

// define the zigzagtime
#define ZIGZAG_TIME 2000 // zigzag time.

// define the debounce time.
#define DEBOUNCE 25

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
unsigned long startTime;

int zigzagCtr;
int motorSetpoint;	// use this for the zigzag testing.
int spdIn;			// grab the speed pot.
int timeIn;			// grab the time pot.
boolean motorDir;   // motor direction.
boolean firstTrap;  // use this flag to indicate whether the trap is called or not.


void setup() {
	// setup up buttons an debounce them
	pinMode(START_BTN, INPUT);
	startBtn.attach(START_BTN);
	startBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(MODE_BTN, INPUT);
	modeBtn.attach(MODE_BTN);
	modeBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(STOP_BTN, INPUT);
	stopBtn.attach(STOP_BTN);
	stopBtn.interval(DEBOUNCE);       // interval in ms

	//pinMode(TRAP_LED, OUTPUT);
	//pinMode(ZZ_1_LED, OUTPUT);
	//pinMode(ZZ_2_LED, OUTPUT);
	//pinMode(ZZ_3_LED, OUTPUT);
	pinMode(MOTOR_OUTPUT, OUTPUT);
	pinMode(MOTOR_DIRECTION, OUTPUT);
	pinMode(MOTOR_RUN, OUTPUT);
	pinMode(ANAL_10V_REF, OUTPUT);
	//start the serial for the comment functuion to work.
	Serial.begin(115200);
}

void loop() {
	//Always update the debounced inputs
	startBtn.update();
	stopBtn.update();
	modeBtn.update();
	stopBtnState = stopBtn.read();
	// get the bool states for the triggers.
	startTrig.update(startBtn.read());
	modeTrig.update(modeBtn.read());

	//always read the stop button and set to SETUP if pressed.
	if (!stopBtnState)
		state = SETUP;

	switch (state) {
	case SETUP:
		// make sure the output is always set low, and not running.
		analogWrite(MOTOR_OUTPUT, 0);
		analogWrite(ANAL_10V_REF, 255);
		digitalWrite(MOTOR_RUN, LOW);
		// reset motor direction in setup state if necessary
		if (motorDir)
		{
			motorDir = false; //make sure the initial directionis repeatable.
			digitalWrite(MOTOR_RUN, LOW);
		}
		
		
		//handle mode changing
		if (modeTrig.Falling)
		{
			// change the modestate
			if (modeState < 500)
				modeState += 100;
			else
				modeState = 100;
		}
		//setStateLamp(modeState);
		//comment(String(modeState));
		// keep the zigzagCtr clamped and ready.
		zigzagCtr = 0;
		startTime = 0;
		firstTrap = true; // setup the trapezoidal move ready.

		// slightlty counter intuitively, stopBtn is normally true (pullup resistor)
		if (startTrig.Falling & stopBtn.read())
			state = modeState;
		
		// read the velocity input and map to percentage 0 - 139.5% (139.5% is max motor)
		spdIn = map(analogRead(VELO_ANAL),0,1023,0,122*1.395);	

		// read the time and map to 500ms to 3000ms
		timeIn = map(analogRead(TIME_ANAL), 0, 1023, 250, 1500);		// read the time input

		// motor setpoint.  as spdIn.
		motorSetpoint = spdIn; // spdIn (as %) * single percent output.
		comment("spdIN : " + String(spdIn));
		comment("motorSetpoint : " + String(motorSetpoint));
		//comment("time : " + String(timeIn));
		break;

	case TRAPEZOIDAL:
		comment("Trapezoidal");
		// just write the output to the motor output.
		if (firstTrap)
		{
			startMotor(motorSetpoint, false);
			//comment(String(motorSetpoint));
		}
		break;

	case ZZ_1:
		//comment("ZZ_1");
		zigzagMethod(ZZ_1_NUM);
		break;

	case ZZ_2:
		//comment("ZZ_2");
		zigzagMethod(ZZ_2_NUM);
		break;

	case ZZ_3:
		//comment("ZZ_3");
		zigzagMethod(ZZ_3_NUM);
		break;

	case ZZ_INF:
		//comment("ZZ_INF");
		zigzagMethod(ZZ_INF_NUM);
		break;

	default:

		state = SETUP;
		// if nothing else matches, do the default
		// default is optional
	}
}

//set the lamp states based upon the mode
//void //setStateLamp(int modeState)
//{
//	switch (modeState) {
//	case TRAPEZOIDAL:
//		digitalWrite(TRAP_LED, HIGH);
//		digitalWrite(ZZ_1_LED, LOW);
//		digitalWrite(ZZ_2_LED, LOW);
//		digitalWrite(ZZ_3_LED, LOW);
//		break;
//	case ZZ_1:
//		digitalWrite(TRAP_LED, LOW);
//		digitalWrite(ZZ_1_LED, HIGH);
//		digitalWrite(ZZ_2_LED, LOW);
//		digitalWrite(ZZ_3_LED, LOW);
//		break;
//	case ZZ_2:
//		digitalWrite(TRAP_LED, LOW);
//		digitalWrite(ZZ_1_LED, LOW);
//		digitalWrite(ZZ_2_LED, HIGH);
//		digitalWrite(ZZ_3_LED, LOW);
//		break;
//	case ZZ_3:
//		digitalWrite(TRAP_LED, LOW);
//		digitalWrite(ZZ_1_LED, LOW);
//		digitalWrite(ZZ_2_LED, LOW);
//		digitalWrite(ZZ_3_LED, HIGH);
//		break;
//	case ZZ_INF:
//		digitalWrite(TRAP_LED, LOW);
//		digitalWrite(ZZ_1_LED, HIGH);
//		digitalWrite(ZZ_2_LED, HIGH);
//		digitalWrite(ZZ_3_LED, HIGH);
//		break;
//	}
//}

// wraps the Serial.prinrln for extrra laziness
void comment(String comment)
{
	Serial.println(comment);
}

void zigzagMethod(int numberOfZigs)
{
	if (numberOfZigs == ZZ_INF_NUM)				// special case for infinite zigzag
		zigger(numberOfZigs);
	else if (zigzagCtr < numberOfZigs)	// if it needs to do the zigzag motion
		zigger(numberOfZigs); 
	else if (zigzagCtr == numberOfZigs)	// should get here after the zigzags finish.
		comment("Done all my zigzags");
		// stop and return to 
	else								// if not a recognised value, go to setup state.
		state = SETUP;

}

// handles timing each zigzag, incrementing the counter and changing motor direction.
void zigger(int numberOfZigs)
{
	unsigned long currTime = millis();	// get current time
	long zzt;							// temp var for holding correct time
	//comment("elapsed time: " + String(currTime - startTime));

	// if elapsed time hasn't been used this run, get it.
	if (startTime == 0)
		startTime = currTime;

	//first zigzag, only do half the time (as not so much distance to go).
	if (zigzagCtr == 0)
	{
		zzt = timeIn / 2;
		startMotor(motorSetpoint, false);
	}
	else
		zzt = timeIn;

	//comment("ZZ Time: " + String(zzt));
	//after the zigzag timer has elapsed, reverse and increment the counter.
	if (currTime - startTime > zzt)
	{
		//when completed, increment zigzagCtr.
		zigzagCtr++;
		// change output direction.
		motorDir = !motorDir;
		digitalWrite(MOTOR_DIRECTION, motorDir);
		if (motorDir)
			comment("MOTOR DIRECTION: TRUE");
		else
			comment("MOTOR DIRECTION: FALSE");
		//analogWrite(MOTOR_OUTPUT, motorSetpoint);
		//reset elapsed time for next zigzag.
		startTime = 0;
	}

	// if infinite zigzags, keep zigzag ctr at 1.
	if (numberOfZigs == ZZ_INF_NUM && zigzagCtr > 1)
		zigzagCtr = 1;

}

void startMotor(int motorSetpoint, boolean direction)
{
	//set output speed and start the motor in the right direction.
	analogWrite(MOTOR_OUTPUT, motorSetpoint);
	if (direction)
		digitalWrite(MOTOR_DIRECTION, HIGH);
	else
		digitalWrite(MOTOR_DIRECTION, LOW);

	//start the motor.
	digitalWrite(MOTOR_RUN, HIGH);
}