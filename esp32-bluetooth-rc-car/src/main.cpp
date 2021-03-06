#include <Arduino.h>
#include "BluetoothSerial.h"
#include "DFRobot_Fermion.h"
#include "AsyncDelay.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

DFRobot_Fermion mainMotor(13, 14);
DFRobot_Fermion steeringMotor(32, 33);
int mainMotorSpeed = 25;
const int MAX_DUTY_CYCLE = 255;
const int FLASH_BT_NOT_CONNECTED = 1000;
const int FLASH_BT_CONNECTED = 250;
AsyncDelay delay_status;
AsyncDelay delay_10ms;
AsyncDelay dead_mans_switch;

typedef enum
{
	FORWARD = 70,
	FORWARD_LEFT = 71,
	BACKWARDS_LEFT = 72,
	FORWARD_RIGHT = 73,
	BACKWARDS_RIGHT = 74,
	BACKWARDS = 66,
	STOP = 83,
	LEFT = 76,
	RIGHT = 82,
	SPEED_1 = 49,
	SPEED_2 = 50,
	SPEED_3 = 51,
	SPEED_4 = 52,
	SPEED_5 = 53,
	SPEED_6 = 54,
	SPEED_7 = 55,
	SPEED_8 = 56,
	SPEED_9 = 57,
	SPEED_10 = 113
} DIRECTIONS;

BluetoothSerial SerialBT;

void setup()
{
	Serial.begin(115200);
	SerialBT.begin("RC Car"); // Bluetooth device name
	Serial.println("The device started, now you can pair it with bluetooth!");
	mainMotor.init(0, 20000, 8);
	steeringMotor.init(1, 20000, 8);
	delay_status.start(FLASH_BT_NOT_CONNECTED, AsyncDelay::MILLIS);
	dead_mans_switch.start(1000, AsyncDelay::MILLIS);
	pinMode(LED_BUILTIN, OUTPUT);
}

void takeAction(int c)
{
	switch (c)
	{
	case FORWARD:
		mainMotor.forward(mainMotorSpeed);
		steeringMotor.brake();
		break;
	case FORWARD_LEFT:
		mainMotor.forward(mainMotorSpeed);
		steeringMotor.reverse(MAX_DUTY_CYCLE);
		break;
	case FORWARD_RIGHT:
		mainMotor.forward(mainMotorSpeed);
		steeringMotor.forward(MAX_DUTY_CYCLE);
		break;
	case STOP:
		mainMotor.brake();
		steeringMotor.brake();
		break;
	case BACKWARDS:
		mainMotor.reverse(mainMotorSpeed);
		steeringMotor.brake();
		break;
	case BACKWARDS_LEFT:
		mainMotor.reverse(mainMotorSpeed);
		steeringMotor.reverse(MAX_DUTY_CYCLE);
		break;
	case BACKWARDS_RIGHT:
		mainMotor.reverse(mainMotorSpeed);
		steeringMotor.forward(MAX_DUTY_CYCLE);
		break;
	case LEFT:
		mainMotor.brake();
		steeringMotor.reverse(MAX_DUTY_CYCLE);
		break;
	case RIGHT:
		mainMotor.brake();
		steeringMotor.forward(MAX_DUTY_CYCLE);
		break;
	case SPEED_1:
		mainMotorSpeed = 25;
		break;
	case SPEED_2:
		mainMotorSpeed = 50;
		break;
	case SPEED_3:
		mainMotorSpeed = 75;
		break;
	case SPEED_4:
		mainMotorSpeed = 100;
		break;
	case SPEED_5:
		mainMotorSpeed = 125;
		break;
	case SPEED_6:
		mainMotorSpeed = 150;
		break;
	case SPEED_7:
		mainMotorSpeed = 175;
		break;
	case SPEED_8:
		mainMotorSpeed = 200;
		break;
	case SPEED_9:
		mainMotorSpeed = 225;
		break;
	case SPEED_10:
		mainMotorSpeed = 250;
		break;
	default:
		Serial.printf("%d ", c);
		break;
	}
}

void loop()
{
	static int lastC = 0;
	if (SerialBT.available())
	{
		dead_mans_switch.restart();
		int c = SerialBT.read();
		if (c != lastC)
		{
			lastC = c;
			takeAction(c);
		}
	}
	if(dead_mans_switch.isExpired())
	{
		dead_mans_switch.repeat();
		takeAction(STOP);
	}
	if (delay_status.isExpired())
	{
		delay_status.start(SerialBT.connected() ? FLASH_BT_CONNECTED : FLASH_BT_NOT_CONNECTED, AsyncDelay::MILLIS);
		digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == HIGH ? LOW : HIGH);
	}
}
