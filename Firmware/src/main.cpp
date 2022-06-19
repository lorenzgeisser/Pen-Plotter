#include <Arduino.h>
#include "point.h"
#include "sdCard.h"
#include "machine.h"
#include <Servo.h>

#define FILENAME "fisch.txt"

enum State
{
    START,
    INIT_MACHINE,
    READY,
    HOMING,
    DRAWING,
    FAULT,
    FINISHED
};
Servo myservo1;

State currentState = START;

bool isFirst = true;
bool isFinished = false;

Machine machine;

String lastErrorMessage;

void setup(void)
{
    Serial.begin(115200);

    myservo1.attach(5);

    myservo1.write(4);
    delay(4000);
    myservo1.write(65);
    delay(4000);

    myservo1.write(4);
    delay(4000);
    myservo1.write(65);
    delay(4000);

    myservo1.write(4);
    delay(4000);
    myservo1.write(65);
    delay(4000);

    myservo1.write(4);
    delay(4000);
    myservo1.write(65);
    delay(4000);

    myservo1.write(4);
    delay(4000);
    myservo1.write(65);
    delay(4000);

    while (1)
        ;

    delay(3000);
}

void setState(State newState)
{
    currentState = newState;

    isFinished = false;
    isFirst = true;
}

void loop()
{

    switch (currentState)
    {
    case START:
        if (isFirst)
        {
            Serial.println("START");
        }

        setState(INIT_MACHINE);

        break;

    case INIT_MACHINE:

        machine.init();

        machine.enableStepperMotors();

        sdCard.begin();

        setState(HOMING);

        break;

    case HOMING:
        if (isFirst)
        {
            Serial.println("HOMING");
        }

        if (!machine.homeLinAxis(isFinished, isFirst))
        {
            setState(FAULT);

            break;
        }

        if (isFinished)
        {
            setState(DRAWING);
        }

        break;

    case READY:
        if (isFirst)
        {
            Serial.println("READY");
            isFirst = false;
        }

        break;

    case DRAWING:

        if (isFirst)
        {
            Serial.println("DRAWING");
        }

        if (!machine.draw(isFinished, isFirst, FILENAME))
        {
            setState(FAULT);

            break;
        }

        if (isFinished)
        {
            setState(READY);
        }

        break;

    case FINISHED:
        if (isFirst)
        {
            Serial.println("FINISHED");
            isFirst = false;
            machine.disableStepperMotors();
        }

    case FAULT:
        if (isFirst)
        {
            isFirst = false;

            machine.disableStepperMotors();
            Serial.println("FAULT");
        }

        break;

    default:
        currentState = FAULT;
        break;
    }
}