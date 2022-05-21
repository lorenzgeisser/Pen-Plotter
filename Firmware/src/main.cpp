#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "point.h"

#define PIN_MOT_ROT_EN 6
#define PIN_MOT_ROT_DIR 2
#define PIN_MOT_ROT_STP 3

#define PIN_MOT_LIN_EN 6
#define PIN_MOT_LIN_DIR 10
#define PIN_MOT_LIN_STP A0

#define PIN_ENDSWITCH 4
#define PIN_SERVO 5

#define SPEED_HOMING_FAST 2000
#define SPEED_HOMING_SLOW 100

#define MOTOR_STEPS_FULL 200 * 16
#define MOTOR_STEPS_DEG ((MOTOR_STEPS_FULL) / 360.0)

#define STEPS_FULL_ROTATION 200 * 16
#define STEPS_HALF_ROTATION 200 * 16 / 2

#define STEPS_PER_MM (8060 / 102.0)

#define MAX_DISTANCE_BETWEEN_MICROPOINTS 1

AccelStepper motor_turn(AccelStepper::DRIVER, PIN_MOT_ROT_STP, PIN_MOT_ROT_DIR);
AccelStepper motor_linear(AccelStepper::DRIVER, PIN_MOT_LIN_STP, PIN_MOT_LIN_DIR);

MultiStepper motors;

Servo myservo;

void initMotors(void)
{
    motor_turn.setPinsInverted(false, false, true);
    motor_turn.setEnablePin(PIN_MOT_ROT_EN);

    motor_linear.setPinsInverted(false, false, true);
    motor_linear.setEnablePin(PIN_MOT_LIN_EN);

    motor_linear.setMaxSpeed(1000.0);
    motor_linear.setAcceleration(50.0);

    motor_turn.setMaxSpeed(1000.0);
    motor_turn.setAcceleration(50.0);

    motors.addStepper(motor_turn);
    motors.addStepper(motor_linear);

    motor_turn.enableOutputs();
    motor_linear.enableOutputs();
}
void setPen(bool active)
{
    if (active)
    {
        myservo.write(100);
        // delay(200); // waits 15ms for the servo to reach the position
    }
    else
    {
        myservo.write(70); // tell servo to go to position in variable 'pos'
        // delay(200);        // waits 15ms for the servo to reach the position
    }
}
void home(void)
{
    setPen(false);
    motor_linear.setMaxSpeed(SPEED_HOMING_FAST);

    long newAbsSteps[] = {0, 10000};

    motors.moveTo(newAbsSteps);

    while (digitalRead(PIN_ENDSWITCH))
    {
        motors.run();
    }

    motor_linear.setCurrentPosition(0);

    delay(500);

    motor_linear.setMaxSpeed(SPEED_HOMING_FAST);

    long newAbsSteps2[] = {0, -10000};

    motors.moveTo(newAbsSteps2);

    while (!digitalRead(PIN_ENDSWITCH))
    {
        motors.run();
    }

    motor_linear.setCurrentPosition(0);
    delay(500);

    motor_linear.setMaxSpeed(SPEED_HOMING_SLOW);

    long newAbsSteps3[] = {0, 10000};

    motors.moveTo(newAbsSteps3);

    while (digitalRead(PIN_ENDSWITCH))
    {
        motors.run();
    }

    motor_linear.setCurrentPosition(0);

    motor_turn.setCurrentPosition(3 * MOTOR_STEPS_FULL);

    motor_linear.setMaxSpeed(SPEED_HOMING_FAST);

    motor_linear.setCurrentPosition(5480);
}

void initServo(void)
{
    myservo.attach(PIN_SERVO); // attaches the servo on pin 9 to the servo object
}

void initSwitch(void)
{
    pinMode(PIN_ENDSWITCH, OUTPUT);
}

long getNumberOfMicropoints(double distancePoints)
{
    double tempNumber = abs(distancePoints);
    long number = 0;

    while (tempNumber > MAX_DISTANCE_BETWEEN_MICROPOINTS)
    {
        number++;
        tempNumber -= MAX_DISTANCE_BETWEEN_MICROPOINTS;
    }

    if (tempNumber > 0)
    {
        number++;
    }

    return number;
}

double getAngleOfMicropoints(Point newPoint, Point currentPoint)
{
    double denta_x = newPoint.x - currentPoint.x;
    double denta_y = newPoint.y - currentPoint.y;

    double angleRad;

    if (abs(denta_x) < 0.001)
    {
        angleRad = 0.5 * PI;
    }
    else if (abs(denta_y) < 0.001)
    {
        angleRad = 0;
    }
    else
    {
        angleRad = atan(abs(denta_y) / abs(denta_x));
    }

    return angleRad;
}

Point getCurrentMicropoint(long currentMicropoint, long numberOfMicropoints, double angleOfMicropoints, Point newPoint, Point currentPoint)
{

    double denta_x = newPoint.x - currentPoint.x;
    double denta_y = newPoint.y - currentPoint.y;
    if (currentMicropoint == numberOfMicropoints)
    {
        return newPoint;
    }
    else
    {
        double newDenta_x = cos(angleOfMicropoints) * currentMicropoint * MAX_DISTANCE_BETWEEN_MICROPOINTS;
        double newDenta_y = sin(angleOfMicropoints) * currentMicropoint * MAX_DISTANCE_BETWEEN_MICROPOINTS;

        double x = currentPoint.x;

        if (denta_x > 0)
        {
            x += newDenta_x;
        }
        else if (denta_x < 0)
        {
            x -= newDenta_x;
        }

        double y = currentPoint.y;

        if (denta_y > 0)
        {
            y += newDenta_y;
        }
        else if (denta_y < 0)
        {
            y -= newDenta_y;
        }

        return Point(x, y, newPoint.draw);
    }
}

long getFastestRotAbs(Point nextPoint)
{
    while (motor_turn.currentPosition() > STEPS_FULL_ROTATION)
    {
        motor_turn.setCurrentPosition(motor_turn.currentPosition() - STEPS_FULL_ROTATION);
    }

    while (motor_turn.currentPosition() < (-1 * STEPS_FULL_ROTATION))
    {
        motor_turn.setCurrentPosition(motor_turn.currentPosition() + STEPS_FULL_ROTATION);
    }

    long currentPositionRot = motor_turn.currentPosition();

    long nextPositionRot = nextPoint.angle * MOTOR_STEPS_DEG;

    long deltaMove = nextPositionRot - currentPositionRot;

    if (deltaMove > STEPS_HALF_ROTATION)
    {
        deltaMove -= STEPS_FULL_ROTATION;
    }
    else if (deltaMove < (-1 * STEPS_HALF_ROTATION))
    {
        deltaMove += STEPS_FULL_ROTATION;
    }

    return currentPositionRot + deltaMove;
}

long getLinPos(Point nextPoint)
{
    return STEPS_PER_MM * nextPoint.radius;
}

void moveTo(Point nextPoint, Point lastPoint = Point(0, 0, false))
{
    setPen(nextPoint.draw);
    if (nextPoint.draw && (nextPoint.getDistanceToPoint(lastPoint) > MAX_DISTANCE_BETWEEN_MICROPOINTS))
    {
        long numberOfMicropoints = getNumberOfMicropoints(nextPoint.getDistanceToPoint(lastPoint));

        double angleRadMicropoints = getAngleOfMicropoints(nextPoint, lastPoint);

        for (long i = 0; i < numberOfMicropoints; i++)
        {
            Point micropoint = getCurrentMicropoint(i, numberOfMicropoints, angleRadMicropoints, nextPoint, lastPoint);

            long newAbsSteps[] = {getFastestRotAbs(micropoint), getLinPos(micropoint)};

            motors.moveTo(newAbsSteps);
            motors.runSpeedToPosition();
        }

        long newAbsSteps[] = {getFastestRotAbs(nextPoint), getLinPos(nextPoint)};

        motors.moveTo(newAbsSteps);
        motors.runSpeedToPosition();
    }
    else
    {

        long newAbsSteps[] = {getFastestRotAbs(nextPoint), getLinPos(nextPoint)};

        motors.moveTo(newAbsSteps);
        motors.runSpeedToPosition();
    }
}

bool getNextPoint(File *myFile, Point *p_, double middle_x, double middle_y)
{
    File temp = SD.open("temp.txt", FILE_WRITE);

    temp.close();

    // Check to see if the file exists:

    if (SD.exists("temp.txt"))
    {

        Serial.println("temp.txt exists.");
    }
    else
    {

        Serial.println("temp.txt doesn't exist.");
    }

    // delete the file:

    Serial.println("Removing temp.txt...");

    SD.remove("temp.txt");

    temp = SD.open("temp.txt", FILE_WRITE);


    temp.write("hello");
        temp.write("\n");
        
    temp.write("hoi");
        temp.write("\n");


    temp.close();

    while (myFile->available())
    {
        String currentChar = myFile->readStringUntil('\n');

        // Serial.println(currentChar);

        if (currentChar.startsWith("G1") || currentChar.startsWith("G0"))
        {
            for (int i = 0; i < currentChar.length(); i++)
            {
                char singleCHar = currentChar.charAt(i);

                switch (singleCHar)
                {
                case 'X':
                    p_->x = currentChar.substring(i + 1).toDouble() - middle_x;
                    break;

                case 'Y':
                    p_->y = currentChar.substring(i + 1).toDouble() - middle_y;
                    break;

                case 'Z':
                    if (currentChar.substring(i + 1).toDouble() > 0)
                    {
                        p_->draw = false;
                    }
                    else
                    {
                        p_->draw = true;
                    }
                    break;

                default:
                    break;
                }
            }

            return true;
        }
    }

    return false;
}

void setup(void)
{
    Serial.begin(115200);

    delay(3000);

    if (!SD.begin(A1))
    {
        Serial.println("SD initialization failed!");
        while (true)
            ;
    }

    String filename = "fisch.txt";
    File myFile;

    myFile = SD.open(filename, FILE_READ);

    if (!myFile)
    {
        Serial.println("error opening " + filename);
        while (true)
            ;
    }
    Point nextPoint = Point(10, 10, false);

    double min_x = 1000000;
    double max_x = -1000000;
    double min_y = 1000000;
    double max_y = -1000000;

    Serial.println(" start ");

    int cnt = 0;

    while (getNextPoint(&myFile, &nextPoint, 0, 0))
    {
        if (nextPoint.draw)
        {
            cnt++;
            if (nextPoint.x < min_x)
            {
                min_x = nextPoint.x;
            }
            if (nextPoint.x > max_x)
            {
                max_x = nextPoint.x;
            }

            if (nextPoint.y < min_y)
            {
                min_y = nextPoint.y;
            }
            if (nextPoint.y > max_y)
            {
                max_y = nextPoint.x;
            }

            Serial.print("x = ");
            Serial.print(nextPoint.x);
            Serial.print("  ");

            Serial.print("y = ");
            Serial.print(nextPoint.y);
            Serial.print("  ");

            Serial.print("draw = ");
            Serial.print(nextPoint.draw);
            Serial.println("  ");
        }
    }

    Serial.println(cnt);

    Serial.print("min x = ");
    Serial.print(min_x);
    Serial.print("  ");

    Serial.print("max x = ");
    Serial.print(max_x);
    Serial.print("  ");

    Serial.print("min y = ");
    Serial.print(min_y);
    Serial.print("  ");

    Serial.print("max y = ");
    Serial.print(max_y);
    Serial.print("  ");

    double delta_x = max_x - min_x;

    double middle_x = min_x + (delta_x / 2);

    double delta_y = max_y - min_y;
    double middle_y = min_y + (delta_y / 2);

    myFile.close();

    myFile = SD.open(filename, FILE_READ);

    if (!myFile)
    {
        Serial.println("error opening " + filename);
        while (true)
            ;
    }

    Serial.println("Init Servo:");
    initServo();
    setPen(false);

    Serial.println("Init Motors:");
    initMotors();

    Serial.println("Init Switch:");
    initSwitch();

    Serial.println("Home:");
    home();

    motor_turn.setMaxSpeed(800);

    Serial.println(" start drawing");

    Point lastPoint(0, 0, false);

    lastPoint.updatePoint();

    Point nextPoint2 = Point(10, 10, false);

    while (getNextPoint(&myFile, &nextPoint2, middle_x, middle_y))
    {
        nextPoint2.updatePoint();
        // Serial.println("next Point: ");
        // Serial.print("X = ");
        // Serial.println(nextPoint2.x);
        // Serial.print("Y = ");
        // Serial.println(nextPoint2.y);
        // Serial.print("draw = ");
        // Serial.println(nextPoint2.draw);

        moveTo(nextPoint2, lastPoint);

        lastPoint = nextPoint2;
        lastPoint.updatePoint();
    }

    home();
}

void loop()
{
}