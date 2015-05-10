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
#include <Wire.h>
#include <LCD03.h>

//MEGA PINS
int I0_09 = A2;	// select the Analog (0-10Vdc)
int I0_10 = A3;	// select the Analog (0-10Vdc)
int I0_11 = A4;	// select the Analog (0-10Vdc)
int I0_12 = A5;	// select the Analog (0-10Vdc)

int I0_03 = 25; // select 24VDC
int I0_02 = 24;	// select 24VDC
int I0_01 = 23;	// select 24VDC
int I0_00 = 22;	// select 24VDC


//Configuration Digital OUT (24Vdc)
int Q0_07 = 6;     // select the Analog (0-10Vdc) 
int Q0_05 = 4;     // select the Analog (0-10Vdc) 
int Q0_00 = 36;     // select the Analog (0-10Vdc) 
int Q0_01 = 37;     // select the Analog (0-10Vdc) 

// set digital input pin numbers:
const int DRV_FAULT = I0_03;// I0.3 drv fault relay feedback
const int START_BTN = I0_02;// I0.2 the number of the start button.
const int STOP_BTN = I0_01;	// I0.1  the number of the stop button.
const int MODE_BTN = I0_00;	// I0.0 the number of the mode button.

// define xFader deadband range 
#define xFADER_DEAD_START 450
#define xFADER_DEAD_END 550

//set analogue input pins.
const int FADER_ANAL = I0_12;	// I0.7	the crossfader input pin.
const int VELO_ANAL = I0_09;   // I0.8 the velocity input pin.
const int TIME_ANAL = I0_10;   // I0.9 the velocity input pin.

// set motor output pin number.
const int ANAL_10V_REF = Q0_07;	// Q0.1 just set to max to provide 10V refernce voltage.
const int MOTOR_OUTPUT = Q0_05;	// Q0.0 PWM pin for output to motor

//TODO - add these to OneNote and to 
// set drive start and direction pins.
const int MOTOR_RUN = Q0_00;		// Q0.4  motor runnnig
const int MOTOR_DIRECTION = Q0_01;	// Q0.5 motor direction.

// define state numbers
#define SETUP 0
#define TRAPEZOIDAL 100
#define ZZ_INF 200
#define ZZ_PULSE 300
#define X_FADER 400
#define ZZ_1 500
#define ZZ_2 600
#define ZZ_3 700

// define the number of zigzags
#define ZZ_1_NUM 3 // number of zigzagz, ZZ_1
#define ZZ_2_NUM 6 // number of zigzagz, ZZ_2
#define ZZ_3_NUM 9 // number of zigzagz, ZZ_3
#define ZZ_INF_NUM 99 //DON'T CHANGE!

// define the zigzagtime
#define ZIGZAG_TIME 2000 // zigzag time.

// define the debounce time.
#define DEBOUNCE 25

#define SCREEN_UPDATE_PERIOD 333 // time in ms between updating screen. avoid too much work!
#define SMOOTHING_ARRAY_SIZE 100 // size of array to smooth over.

// Instantiate a Bounce object
Bounce startBtn = Bounce();
Bounce stopBtn = Bounce();
Bounce modeBtn = Bounce();
Bounce drvFlt = Bounce();

// read once, use throughout
boolean stopBtnState = false;
boolean drvFaultState = false;

// Variables will change:
int modeState = TRAPEZOIDAL;        // the current state of the output
int state = SETUP;        // the current reading from the input pin

Trigger startTrig = Trigger();
Trigger modeTrig = Trigger();

// counter and elapsed time for zigzagging
unsigned long startTime;

int zigzagCtr;
int xfaderFixSpdIn;	// use this for the zigzag testing.
int spdIn;			// grab the speed pot.
int arSpdIn[SMOOTHING_ARRAY_SIZE];		// array for smoothing.
int timeIn;			// grab the time pot.
int artimeIn[SMOOTHING_ARRAY_SIZE];		// array for smoothing.
int veloAnalIn;		// grab unscaled pot.
int vel;			// used in X_FADER.
int faderIn;		// grab the xFader.
int arFaderIn[SMOOTHING_ARRAY_SIZE];		// array for smoothing.
unsigned long lastScreenUpdateTime; // hold the current screen time to regulate updates.
boolean motorDir;   // motor direction.
boolean firstTrap;  // use this flag to indicate whether the trap is called or not.

LCD03 lcd; // set up the LCD connection

void setup() {
	// configure digital output pins where necessary
	pinMode(MOTOR_RUN, OUTPUT);
	digitalWrite(MOTOR_RUN, LOW); // set this low straight away

	pinMode(MOTOR_DIRECTION, OUTPUT);

	// configure analogue output pins (config same as above).
	pinMode(MOTOR_OUTPUT, OUTPUT);
	pinMode(ANAL_10V_REF, OUTPUT);

	// setup up buttons and debounce them
	pinMode(START_BTN, INPUT);
	startBtn.attach(START_BTN);
	startBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(MODE_BTN, INPUT);
	modeBtn.attach(MODE_BTN);
	modeBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(STOP_BTN, INPUT);
	stopBtn.attach(STOP_BTN);
	stopBtn.interval(DEBOUNCE);       // interval in ms

	pinMode(DRV_FAULT, INPUT);
	drvFlt.attach(DRV_FAULT);
	drvFlt.interval(DEBOUNCE);       // interval in ms
	//pinMode(FADER_ANAL, INPUT);
	//start the serial for the comment function to work.
	Serial.begin(115200);

	//initialise LCD Screen
	LCDSetup();
}

void loop() {
	//Always update the debounced inputs
	startBtn.update();
	stopBtn.update();
	modeBtn.update();
	drvFlt.update();

	stopBtnState = stopBtn.read();

	// get the drive fault state (TRUE = NO FAULT)
	drvFaultState = drvFlt.read();
	comment("SrtB: " + String(startBtn.read()) + " MD: " + String(modeBtn.read()) + " DF:" + String(drvFaultState) + " StpB: " + String(stopBtnState));

	// get the bool states for the triggers.
	startTrig.update(startBtn.read());
	modeTrig.update(modeBtn.read());

	// read the velocity input and map to percentage 0 - 139.5% (139.5% is max motor)
	veloAnalIn = analogRead(VELO_ANAL);
	//comment("Velo anal in is: " + String(veloAnalIn));
	//comment("last screen time: " + String(lastScreenUpdateTime));
	/*
		scale spdIn from 0 to 139.5%.  127.5 is about half of 255 (so 5V output).
		5V to the drive is 100% speed (2860rpm). 3989rpm (139.5%) is max speed
		So 127.5 * 1.395 = 177.8 = max drive speed
		*/
	//smooth the input
	spdIn = map(returnInputAverage(arSpdIn, veloAnalIn, SMOOTHING_ARRAY_SIZE), 0, 1023, 0, 178);
	//comment("speedIn is " + String(spdIn));

	// read the time and map to 500ms to 3000ms
	//smooth the input
	timeIn = map(returnInputAverage(artimeIn, analogRead(TIME_ANAL), SMOOTHING_ARRAY_SIZE), 0, 1023, 250, 1500);		// read the time input

	// update the screen
	if (updateScreen())
	{	// pass the correct speed based upon the
		comment("Selected State " + String(modeState));
		if (state != X_FADER)
			lcdSetupMode(state, modeState, spdIn, timeIn, zigzagCtr);
		else
			lcdSetupMode(state, 0, vel, 0, 0);
	}

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
			if (modeState < ZZ_3)
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

		//modestate test for X_FADER mode.
		if (state == X_FADER)
		{
			xfaderFixSpdIn = spdIn; // don't allow max speed to change during xFader
			int v = analogRead(FADER_ANAL); //

			// hold in setup if not in deadband.
			if (!(v > xFADER_DEAD_START && v < xFADER_DEAD_END))
				state = SETUP;
		}

		break;

	case TRAPEZOIDAL:
		// just write the output to the motor output.
		if (firstTrap)
			startMotor(spdIn, false);

		updatMotorSpd(spdIn);
		//comment("bowl velocity is " + String(spdIn));
		break;

	case ZZ_INF:
		// continuous zigzags
		zigzagMethod(ZZ_INF_NUM);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case ZZ_PULSE:
		// continuous zigzags
		zzPulseMethod();

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case ZZ_1:
		// three zig zags
		zigzagMethod(ZZ_1_NUM);
		updatMotorSpd(spdIn);
		//comment("bowl velocity is " + String(spdIn));
		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		break;

	case ZZ_2:
		// six zigzags
		zigzagMethod(ZZ_2_NUM);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case ZZ_3:
		// nine zig zags
		zigzagMethod(ZZ_3_NUM);

		if (startTrig.Falling)
			state = TRAPEZOIDAL;

		updatMotorSpd(spdIn);
		break;

	case X_FADER:
		// use a cross fader to change between modes
		// make sure in zero speed before starting
		faderIn = analogRead(FADER_ANAL);
		vel = map(faderIn, 0, 1023, 0, xfaderFixSpdIn);
		//comment("xfader vel: " + String(vel));

		// pick direction of travel based upon master speed override.
		if (vel < xfaderFixSpdIn / 2)
			startMotor(vel, false);
		else if (vel > xfaderFixSpdIn / 2)
			startMotor(vel, true);

		// allow trapezoidal break condition
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

// does the zigzag pulses.
void zzPulseMethod(){
	unsigned long currTime = millis();	// get current time
	long zzt;							// temp var for holding correct time

	// if elapsed time hasn't been used this run, get it.
	if (startTime == 0)
		startTime = currTime;

	// if zigzagCtr is zero, run.
	if (zigzagCtr == 0)
		digitalWrite(MOTOR_DIRECTION, motorDir);
	else if (zigzagCtr == 1) // if it's one, stop.
	{
		analogWrite(MOTOR_OUTPUT, 0);
		digitalWrite(MOTOR_RUN, LOW);
	}
	else
		state = SETUP; // shouoldn't be able to get here.  Reset

	//after the section has elapsed, do next section.
	if (currTime - startTime > zzt)
	{
		//when completed, increment zigzagCtr.
		zigzagCtr++;
		startTime = 0;
	}

	// if zizagCtr is > 1, reset.
	if (zigzagCtr > 1)
		zigzagCtr = 0;
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

// update the screen
boolean updateScreen()
{
	if (millis() - lastScreenUpdateTime > SCREEN_UPDATE_PERIOD)
	{
		lastScreenUpdateTime = millis();
		return true;
	}

	return false;
}

// initial screen
void LCDSetup(){
	// Initialise a 20x4 LCD
	lcd.begin(20, 4);
	delay(1000);
	comment("LCD Setup entered");

	// Turn on the backlight
	lcd.backlight();

	//set up screen and print.
	lcd.setCursor(5);
	lcd.print("SILOSTUDIO");
	lcd.setCursor(42);
	lcd.print("Attua Aparicio &");
	lcd.setCursor(64);
	lcd.print("Oscar Wanless");
	delay(5000);
}

// initial screen
void lcdSetupMode(int currentState, int selectedState, int MotorRpm, int ZigzagTime, int RemainingZigs){
	// Clear the last screen down
	lcd.clear();
	lcd.home();

	//write the required info to the screen
	switch (currentState) {
	case SETUP:
		lcd.setCursor(2);
		lcd.print("Mode: ");
		// print selected mode
		if (selectedState == TRAPEZOIDAL)
			lcd.print("No zig-zag");
		else if (selectedState == ZZ_INF)
			lcd.print("Zig-zags");
		else if (selectedState == ZZ_PULSE)
			lcd.print("Pulsing");
		else if (selectedState == X_FADER)
			lcd.print("-fader");
		else if (selectedState == ZZ_1)
			lcd.print("3 zig-zags");
		else if (selectedState == ZZ_2)
			lcd.print("6 zig-zags");
		else if (selectedState == ZZ_3)
			lcd.print("9 zig-zags");

		//set up screen and print.
		lcdWriteSpeed(3, MotorRpm);
		lcdWriteTime(4, ZigzagTime);
		break;

	case TRAPEZOIDAL:
		lcdPrintMachineRunning();
		lcd.setCursor(25);
		lcd.print("No zig-zag");
		lcdWriteSpeed(4, MotorRpm);
		break;

	case ZZ_1: //allow multiple fall through to get same result for these three.
	case ZZ_2:
	case ZZ_3:
		//Generic zig zag print
		lcdPrintMachineRunning();
		lcd.setCursor(24);
		lcd.print("ZZs left: " + String(RemainingZigs));
		lcdWriteSpeed(3, MotorRpm);
		lcdWriteTime(4, ZigzagTime);
		break;

	case ZZ_INF:
		//Infinite zig zag print
		lcdPrintMachineRunning();
		lcd.setCursor(22);
		lcd.print("Endless zig-zags");
		lcdWriteSpeed(3, MotorRpm);
		lcdWriteTime(4, ZigzagTime);
		break;

	case ZZ_PULSE:
		//Infinite zig zag print
		lcdPrintMachineRunning();
		lcd.setCursor(22);
		lcd.print("      Pulsing");
		lcdWriteSpeed(3, MotorRpm);
		lcdWriteTime(4, ZigzagTime);
		break;

	case X_FADER:
		//Infinite zig zag print
		lcdPrintMachineRunning();
		lcd.setCursor(26);
		lcd.print("X-FADER");
		lcdWriteSpeed(4, MotorRpm);

		break;

	default:
		lcd.setCursor(20);
		lcd.print("ERROR: UNKNOWN MODE!");
	}
}

void lcdWriteSpeed(int line, int MotorRpm){
	lcd.setCursor(getCursorNumber(line) + 2);
	lcd.print("SPEED: " + String(MotorRpm) + "rpm");
}

void lcdWriteTime(int line, int ZigzagTime){
	lcd.setCursor(getCursorNumber(line));
	lcd.print("ZIGZAG TIME: " + String(ZigzagTime) + "ms");
}

// get the start cursor number based upon the line number
int getCursorNumber(int line){

	if (line < 5)
		return (line - 1) * 20;
	else
		return 0;
}

// Print "MACHINE RUNNING"
void lcdPrintMachineRunning(){
	lcd.setCursor(2);
	lcd.print("MACHINE RUNNING");
}


int returnInputAverage(int inputArray[], int currentValue, int numreadings)
{
	long total = 0;
	//shuffle all the values down a place and put the new value in.  collect the total..
	for (int i = 0; i < numreadings - 1; i++)
	{
		inputArray[i] = inputArray[i + 1];
		total += inputArray[i];
	}
	//add the currentvalue to the array.
	inputArray[numreadings - 1] = currentValue;
	//add the current value.
	//comment(String(total));
	total += currentValue;
	//return the avearge.  Crude smoothing of the input signal.
	if (total < numreadings)
		return 0;
	else
		return (total / numreadings);

}