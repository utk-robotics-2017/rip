// Auto-generated by ArduinoGen

#include "CmdMessenger.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include <Wire.h>

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

const char LED = 13;

Adafruit_BNO055 bno055s [1] = {
	Adafruit_BNO055(40)
};

enum RIPenum : int16_t
{
	kAcknowledge = 0,
	kError = 1,
	kUnknown = 2,
	kSetLed = 3,
	kPing = 4,
	kPingResult = 5,
	kPong = 6,
	kGetYaw = 7,
	kGetYawResult = 8,
	kGetPitch = 9,
	kGetPitchResult = 10,
	kGetRoll = 11,
	kGetRollResult = 12,
	kGetRate = 13,
	kGetRateResult = 14,
	kGetCalibrationStatus = 15,
	kGetCalibrationStatusResult = 16
};

void setup()
{
	// Init LED pin
	pinMode(LED, OUTPUT);

	// Initialize Serial Communication
	Serial.begin(115200);

	attachCommandCallbacks();

	bno055s[0].begin();
	bno055s[0].setExtCrystalUse(true);

	// Flash led 3 times at the end of setup
	for(int i = 0; i < 3; i++)
	{
		digitalWrite(LED, HIGH);
		delay(250);
		digitalWrite(LED, LOW);
		delay(250);
	}
}

void loop()
{
	// Process incoming serial data, and perform callbacks
	cmdMessenger.feedinSerialData();


}

//Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
	cmdMessenger.attach(unknownCommand);
	cmdMessenger.attach(kPing, ping);
	cmdMessenger.attach(kSetLed, setLed);
	cmdMessenger.attach(kGetYaw, getYaw);
	cmdMessenger.attach(kGetPitch, getPitch);
	cmdMessenger.attach(kGetRoll, getRoll);
	cmdMessenger.attach(kGetRate, getRate);
	cmdMessenger.attach(kGetCalibrationStatus, getCalibrationStatus);
}

// Called when a received command has no attached function
void unknownCommand()
{
	cmdMessenger.sendBinCmd(kError, kUnknown);
}

// Called upon initialization of Spine to check connection
void ping()
{
	cmdMessenger.sendBinCmd(kAcknowledge, kPing);
	cmdMessenger.sendBinCmd(kPingResult, kPong);
}

// Callback function that sets led on or off
void setLed()
{
	// Read led state argument, interpret string as boolean
	bool ledState = cmdMessenger.readBoolArg();
	digitalWrite(LED, ledState);
	cmdMessenger.sendBinCmd(kAcknowledge, kSetLed);
}

void getYaw() {
	int indexNum = cmdMessenger.readBinArg<int>();
	if(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {
		cmdMessenger.sendBinCmd(kError, kGetYaw);
		return;
	}
	float rv;
	/* Get a new sensor event */
	sensors_event_t event;
	bno055s[indexNum].getEvent(&event);
	
	// In degrees
	rv = event.orientation.x;
	cmdMessenger.sendBinCmd(kAcknowledge, kGetYaw);
	cmdMessenger.sendCmdStart(kGetYawResult);
	cmdMessenger.sendCmdBinArg(rv);
	cmdMessenger.sendCmdEnd();
}

void getPitch() {
	int indexNum = cmdMessenger.readBinArg<int>();
	if(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {
		cmdMessenger.sendBinCmd(kError, kGetPitch);
		return;
	}
	float rv;
	/* Get a new sensor event */
	sensors_event_t event;
	bno055s[indexNum].getEvent(&event);
	
	// In degrees
	rv = event.orientation.y;
	cmdMessenger.sendBinCmd(kAcknowledge, kGetPitch);
	cmdMessenger.sendCmdStart(kGetPitchResult);
	cmdMessenger.sendCmdBinArg(rv);
	cmdMessenger.sendCmdEnd();
}

void getRoll() {
	int indexNum = cmdMessenger.readBinArg<int>();
	if(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {
		cmdMessenger.sendBinCmd(kError, kGetRoll);
		return;
	}
	float rv;
	/* Get a new sensor event */
	sensors_event_t event;
	bno055s[indexNum].getEvent(&event);
	
	// In degrees
	rv = event.orientation.z;
	cmdMessenger.sendBinCmd(kAcknowledge, kGetRoll);
	cmdMessenger.sendCmdStart(kGetRollResult);
	cmdMessenger.sendCmdBinArg(rv);
	cmdMessenger.sendCmdEnd();
}

void getRate() {
	int indexNum = cmdMessenger.readBinArg<int>();
	if(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {
		cmdMessenger.sendBinCmd(kError, kGetRate);
		return;
	}
	float rv;
	/* Get a new sensor event */
	imu::Vector<3> vec = bno055s[indexNum].getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	
	// In degrees
	rv = vec.x;
	cmdMessenger.sendBinCmd(kAcknowledge, kGetRate);
	cmdMessenger.sendCmdStart(kGetRateResult);
	cmdMessenger.sendCmdBinArg(rv);
	cmdMessenger.sendCmdEnd();
}

void getCalibrationStatus() {
	int indexNum = cmdMessenger.readBinArg<int>();
	if(!cmdMessenger.isArgOk() || indexNum < 0 || indexNum > 1) {
		cmdMessenger.sendBinCmd(kError, kGetCalibrationStatus);
		return;
	}
	char rv;
	rv = bno055s[indexNum].isFullyCalibrated();
	cmdMessenger.sendBinCmd(kAcknowledge, kGetCalibrationStatus);
	cmdMessenger.sendCmdStart(kGetCalibrationStatusResult);
	cmdMessenger.sendCmdBinArg(rv);
	cmdMessenger.sendCmdEnd();
}
