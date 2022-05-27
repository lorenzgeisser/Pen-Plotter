#include <Arduino.h>
#include "point.h"
#include "sdCard.h"
#include "machine.h"

#define PIN_SD_CS A1

#define FILENAME "fisch.txt"

enum State
{
    START,
    READY,
    HOMING,
    DRAWING,
    FAULT,
    FINISHED
};

State currentState = START;

bool isFirst = true;
bool isFinished = false;


Machine machine;

void failState(void)
{
    Serial.println("Failstate");

    machine.setPen(false);
    machine.disableMotors();

    while (true)
    {
    }
}

void setup(void)
{
    Serial.begin(115200);

    delay(3000);

    // Init SD Card
    Serial.print("Init SD");
    if (sdCard.begin(PIN_SD_CS))
    {
        Serial.println(" --> OK");
    }
    else
    {
        Serial.println(" --> Fail");
        failState();
    }

    // Open file
    Serial.print("Open file");
    if (sdCard.openFile(FILENAME))
    {
        Serial.println(" --> OK");
    }
    else
    {
        Serial.println(" --> Fail");
        failState();
    }

    // Init motors
    Serial.print("Init motors");
    if (machine.begin(1000, 50, 1000, 50))
    {
        Serial.println(" --> OK");
    }
    else
    {
        Serial.println(" --> Fail");
        failState();
    }

    // Init servo
    Serial.print("Init servo");
    if (machine.initServo(PIN_SERVO))
    {
        Serial.println(" --> OK");
    }
    else
    {
        Serial.println(" --> Fail");
        failState();
    }

    // Init switch
    Serial.print("Init switch");
    if (machine.initSwitch(PIN_ENDSWITCH))
    {
        Serial.println(" --> OK");
    }
    else
    {
        Serial.println(" --> Fail");
        failState();
    }

    machine.enableMotors();

    // // Home machine
    // Serial.print("Home machine");
    // if (machine.home())
    // {
    //     Serial.println(" --> OK");
    // }
    // else
    // {
    //     Serial.println(" --> Fail");
    //     failState();
    // }

    // // Start drawing
    // Serial.println("Start drawing:");

    // Point nextPoint = Point(10, 10, false);
    // Point lastPoint = Point(10, 10, false);

    // lastPoint.updatePoint();

    // while (sdCard.getNextPoint(&nextPoint))
    // {
    //     nextPoint.updatePoint();
    //     Serial.print("next Point: ");
    //     Serial.print("X = ");
    //     Serial.print(nextPoint.x);
    //     Serial.print("  Y = ");
    //     Serial.print(nextPoint.y);
    //     Serial.print("  draw = ");
    //     Serial.println(nextPoint.draw);

    //     machine.moveTo(nextPoint, lastPoint);

    //     lastPoint = nextPoint;
    //     lastPoint.updatePoint();
    // }
    // Serial.println("Finished drawing");

    // // Close file
    // Serial.print("Close file");
    // if (sdCard.closeFile())
    // {
    //     Serial.println(" --> OK");
    // }
    // else
    // {
    //     Serial.println(" --> Fail");
    //     failState();
    // }

    // // Home machine
    // Serial.print("Home machine");
    // if (machine.home())
    // {
    //     Serial.println(" --> OK");
    // }
    // else
    // {
    //     Serial.println(" --> Fail");
    //     failState();
    // }

    // machine.disableMotors();
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

        setState(HOMING);

        break;

    case HOMING:
        if (isFirst)
        {
            Serial.println("HOMING");
        }

        if (!machine.home(isFinished, isFirst))
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
        }

    case FAULT:
        if (isFirst)
        {
            isFirst = false;

            machine.disableMotors();
            Serial.println("FAULT");
        }

        break;

    default:
        currentState = FAULT;
        break;
    }
}