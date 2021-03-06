// -------------- INCLUDES --------------
#include <Arduino.h>
#include "point.h"
#include "sdCard.h"
#include "machine.h"


// -------------- ENUMS --------------
enum State
{
    INIT,
    INIT_MACHINE,
    READY,
    HOMING,
    DRAWING,
    FAULT,
    FINISHED
};


// -------------- CONSTANTS --------------
const String FileName =  "gcode.txt";
const State StartState = INIT;


// -------------- GLOBAL VARIABLES --------------
// Flags
bool isFirst = true;
bool isFinished = false;


Machine machine;
State currentState = StartState;
String lastErrorMessage;


// -------------- FUNCTIONS --------------
// Changes state of state machine
void setState(State newState)
{
    currentState = newState;

    isFinished = false;
    isFirst = true;
}


// -------------- SETUP --------------
void setup(void)
{
    Serial.begin(115200);

    delay(3000);
}


// -------------- LOOP --------------
void loop()
{

    switch (currentState)
    {
    case INIT:
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

        if (!machine.draw(isFinished, isFirst, FileName))
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