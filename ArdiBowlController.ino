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
//#include <Wire.h>
//#include <LCD03.h>

/*
NOT NEEDED FOR INDUSTRIAL SHIELDS
const int TRAP_LED = 13;     // the number of the trapezoidal LED pin
const int ZZ_1_LED = 12;     // the number of the zz_1 LED pin
const int ZZ_2_LED = 11;     // the number of the zz_2 LED pin
const int ZZ_3_LED = 10;     // the number of the zz_3 LED pin

*/

// set digital input pin numbers:
const int DRV_FAULT = A5;	// I0.4 drv fault relay feedback
const int START_BTN = 4;	// I0.3 the number of the start button.
const int STOP_BTN = 8;		// I0.2  the number of the stop button.
const int MODE_BTN = 12;	// I0.1 the number of the mode button.

// define xFader deadband range as percentage 
#define xFADER_DEAD_START 50
#define xFADER_DEAD_END 200

//set analogue input pins.
const int FADER_ANAL = A2;	// I0.7	the crossfader input pin.
const int VELO_ANAL = A1;   // I0.8 the velocity input pin.
const int TIME_ANAL = A0;   // I0.9 the velocity input pin.

// set motor output pin number.
const int ANAL_10V_REF = 11;	// Q0.1 just set to max to provide 10V refernce voltage.
const int MOTOR_OUTPUT = 13;	// Q0.0 PWM pin for output to motor

// set drive start and direction pins.
const int MOTOR_RUN = 6;		// Q0.4  motor runnnig
const int MOTOR_DIRECTION = 5;	// Q0.5 motor direction.

// define state numbers
#define SETUP 0
#define TRAPEZOIDAL 100
#define ZZ_1 200
#define ZZ_2 300
#define ZZ_3 400
#define ZZ_INF 500
#define X_FADER 600
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
boolean drvFaultState = false;

// Variables will change:
int modeState = 100;        // the current state of the output
int state = SETUP;        // the current reading from the input pin

Trigger startTrig = Trigger();
Trigger modeTrig = Trigger();

// counter and elapsed time for zigzagging
unsigned long startTime;

int zigzagCtr;
//int motorSetpoint;	// use this for the zigzag testing.
int spdIn;			// grab the speed pot.
int timeIn;			// grab the time pot.
int veloAnalIn;		// grab unscaled pot.
int vel;			// used in X_FADER.
int faderIn;		// grab the xFader.
boolean motorDir;   // motor direction.
boolean firstTrap;  // use this flag to indicate whether the trap is called or not.

//LCD03 lcd; // set up the LCD connection

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

	// configure digital output pins where necessa
	pinMode(MOTOR_DIRECTION, OUTPUT);
	pinMode(MOTOR_RUN, OUTPUT);

	// configure analogue output pins (config same as above).
	pinMode(MOTOR_OUTPUT, OUTPUT);
	pinMode(ANAL_10V_REF, OUTPUT);

	//start the serial for the comment function to work.
	Serial.begin(115200);

	//initialise LCD Screen
	//LCDSetup();
}

void loop() {
	//Always update the debounced inputs
	startBtn.update();
	stopBtn.update();
	modeBtn.update();

	stopBtnState = stopBtn.read();

	// if the Drive fault signal is 5Vdc (read into analogue port) then true
	drvFaultState = (analogRead(DRV_FAULT) > 240);

	// get the bool states for the triggers.
	startTrig.update(startBtn.read());
	modeTrig.update(modeBtn.read());

	// read the velocity input and map to percentage 0 - 139.5% (139.5% is max motor)
	veloAnalIn = analogRead(VELO_ANAL);
	comment("Velo anal in is: " + String(veloAnalIn));
	/*
	scale from 0 to 139.5%.  127 is about half of 255.
	5V to the drive is 100% speed (2860). 139.5% is max speed
	*/
	spdIn = map(veloAnalIn, 0, 1023, 0, 127 * 1.395);
	comment("speedIn is " + String(spdIn));


	//always read the stop button and and set to SETUP if pressed.
	if (!stopBtnState || !drvFaultState)
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
			motorDir = false; //make sure the initial direction is repeatable.
			digitalWrite(MOTOR_RUN, LOW);
		}

		//handle mode changing
		if (modeTrig.Falling)
		{
			// change the modestate
			if (modeState < ZZ_INF)
				modeState += 100;
			else
				modeState = 100;
		}

		// keep the zigzagCtr clamped and ready.
		zigzagCtr = 0;
		startTime = 0;
		firstTrap = true; // setup the trapezoidal move ready.

		// slightlty counter intuitively, stopBtn is normally true 
		if (startTrig.Falling & stopBtn.read())
			state = modeState;

		comment("state is: " + String(modeState));

		//modestate test for X_FADER mode.
		if (state == X_FADER)
		{
			int v = map(faderIn, 0, 1023, 0, 255);
			comment("velocity in x_fader: " + String(v));

			if (v > xFADER_DEAD_START && v < xFADER_DEAD_END)
				;
			else
				// hold in setup if not in deadband.
				state = SETUP;
		}

		// read the velocity input and map to percentage 0 - 139.5% (139.5% is max motor)
		//spdIn = map(analogRead(VELO_ANAL),0,1023,0,122*1.395);	
		comment("bowl velocity is " + String(spdIn));

		// read the time and map to 500ms to 3000ms
		timeIn = map(analogRead(TIME_ANAL), 0, 1023, 250, 1500);		// read the time input

		break;

	case TRAPEZOIDAL:
		//comment("Trapezoidal");
		// just write the output to the motor output.
		if (firstTrap)
			startMotor(spdIn, false);

		updatMotorSpd(spdIn);
		comment("bowl velocity is " + String(spdIn));
		break;

	case ZZ_1:
		//comment("ZZ_1");
		zigzagMethod(ZZ_1_NUM);
		updatMotorSpd(spdIn);
		comment("bowl velocity is " + String(spdIn));
		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		break;

	case ZZ_2:
		//comment("ZZ_2");
		zigzagMethod(ZZ_2_NUM);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case ZZ_3:
		//comment("ZZ_3");
		zigzagMethod(ZZ_3_NUM);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case ZZ_INF:
		//comment("ZZ_INF");
		zigzagMethod(ZZ_INF_NUM);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case X_FADER:
		// make sure in zero speed before starting
		vel = map(faderIn, 0, 1023, 0, spdIn);
		comment("xfader vel: " + String(vel));
		if (vel < spdIn / 2)
			startMotor(vel, false);
		else if (vel > spdIn / 2)
			startMotor(vel, true);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		break;

	default:

		state = SETUP;
		// if nothing else matches, do the default
		// default is optional
	}
}

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

	// if elapsed time hasn't been used this run, get it.
	if (startTime == 0)
		startTime = currTime;

	//first zigzag, only do half the time (as not so much distance to go).
	if (zigzagCtr == 0)
	{
		zzt = timeIn / 2;
		startMotor(spdIn, false);
	}
	else
		zzt = timeIn;

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

void updatMotorSpd(int _motorSetPoint)
{
	//set output speed
	comment("_MotorSeptoint " + String(_motorSetPoint));
	analogWrite(MOTOR_OUTPUT, _motorSetPoint);

}
//
//
//// initial screen
//void LCDSetup(){
//	// Initialise a 20x4 LCD
//	lcd.begin(20, 4);
//	delay(10000);
//	comment("LCD Setup entered");
//	
//	// Turn on the backlight
//	lcd.backlight();
//
//	//set up screen and print.
//	lcd.setCursor(5);
//	lcd.print("SILOSTUDIO");
//	lcd.setCursor(42);
//	lcd.print("Attua Aparicio &");
//	lcd.setCursor(64);
//	lcd.print("Oscar Wanless");
//}
//
//// initial screen
//void lcdSetupMode(int currentState, int selectedState, int MotorRpm, int ZigzagTime, int RemainingZigs){
//	// Clear the last screen down
//	lcd.clear();
//	lcd.home();
//
//	//write the required info to the screen
//	switch (currentState) {
//	case SETUP:
//		lcd.setCursor(2);
//		lcd.print("Mode: ");
//		// print selected mode
//		if (selectedState == TRAPEZOIDAL)
//			lcd.print("No zig-zag");
//		else if (selectedState == ZZ_1)
//			lcd.print("3 zig-zags");
//		else if (selectedState == ZZ_2)
//			lcd.print("6 zig-zags");
//		else if (selectedState == ZZ_3)
//			lcd.print("9 zig-zags");
//		else if (selectedState == ZZ_INF)
//			lcd.print("Endless zig-zags");
//
//		//set up screen and print.
//		lcdWriteSpeed(3, MotorRpm);
//		lcdWriteTime(4, ZigzagTime);
//		break;
//
//	case TRAPEZOIDAL:
//		lcdPrintMachineRunning();
//		lcd.setCursor(25);
//		lcd.print("No zig-zag");
//		lcdWriteSpeed(4, MotorRpm);
//		break;
//
//	case ZZ_1: //allow multiple fall through to get same result for these three.
//	case ZZ_2:
//	case ZZ_3:
//		//Generic zig zag print
//		lcdPrintMachineRunning();
//		lcd.setCursor(24);
//		lcd.print("ZZs left: " + String(RemainingZigs));
//		lcdWriteSpeed(3, MotorRpm);
//		lcdWriteTime(4, ZigzagTime);
//		break;
//
//	case ZZ_INF:
//		//Infinite zig zag print
//		lcdPrintMachineRunning();
//		lcd.setCursor(22);
//		lcd.print("Endless zig-zags");
//		lcdWriteSpeed(3, MotorRpm);
//		lcdWriteTime(4, ZigzagTime);
//		break;
//
//	default:
//		lcd.setCursor(20);
//		lcd.print("ERROR: UNKNOWN MODE!");
//	}
//}
//
//void lcdWriteSpeed(int line, int MotorRpm){
//	lcd.setCursor(getCursorNumber(line) + 2);
//	lcd.print("SPEED: " + String(MotorRpm) + "rpm");
//}
//
//void lcdWriteTime(int line, int ZigzagTime){
//	lcd.setCursor(getCursorNumber(line));
//	lcd.print("ZIGZAG TIME: " + String(ZigzagTime) + "ms");
//}
//
//// get the start cursor number based upon the line number
//int getCursorNumber(int line){
//
//	if (line < 5)
//		return (line - 1) * 20;
//	else
//		return 0;
//}
//
//// Print "MACHINE RUNNING"
//void lcdPrintMachineRunning(){
//	lcd.setCursor(2);
//	lcd.print("MACHINE RUNNING");
//}
