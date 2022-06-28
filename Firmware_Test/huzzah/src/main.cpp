#include <Arduino.h>
#include <TMC2209.h>

#define EN_PIN 14   // Enable
#define DIR_PIN 15  // Direction
#define STEP_PIN 32 // Step

HardwareSerial &serial_stream = Serial1;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;

// Instantiate TMC2209
TMC2209 stepper_driver;

void setup()
{
    delay(4000);
    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(EN_PIN, LOW);

    Serial.begin(SERIAL_BAUD_RATE);

    stepper_driver.setup(serial_stream);

    if (stepper_driver.isSetupAndCommunicating())
    {
        Serial.println("Stepper driver is setup and communicating!");
        Serial.println("Try turning driver power off to see what happens.");
    }
    else if (stepper_driver.isCommunicatingButNotSetup())
    {
        Serial.println("Stepper driver is communicating but not setup!");
        Serial.println("Running setup again...");
        stepper_driver.setup(serial_stream);
    }
    else
    {
        Serial.println("Stepper driver is not communicating!");
        Serial.println("Try turning driver power on to see what happens.");
    }

    stepper_driver.enable();

    stepper_driver.setHoldCurrent(100);
}

void loop()
{
}