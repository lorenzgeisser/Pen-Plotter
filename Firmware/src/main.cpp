#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

#include "point.h"

#define PIN_MOT_L_EN 6
#define PIN_MOT_L_DIR 3
#define PIN_MOT_L_STP 2

#define PIN_MOT_R_EN 12
#define PIN_MOT_R_DIR 11
#define PIN_MOT_R_STP 10

#define PIN_ENDSWITCH 4
#define PIN_SERVO 5

#define SPEED_HOMING_FAST 2000
#define SPEED_HOMING_SLOW 100

#define MOTOR_STEPS_FULL 200 * 16
#define MOTOR_STEPS_DEG ((MOTOR_STEPS_FULL) / 360.0)

#define STEPS_FULL_ROTATION 200 * 16
#define STEPS_HALF_ROTATION 200 * 16 / 2

#define NUMBER_POINTS 97

#define STEPS_PER_MM (8060 / 102.0)

#define MAX_DISTANCE_BETWEEN_MICROPOINTS 1

Point points[NUMBER_POINTS] = {
    Point(30, -10),
    Point(30, 10),
    Point(10, 10),
    Point(10, 30),
    Point(-10, 30),
    Point(-10, 10),
    Point(-30, 10),
    Point(-30, -10),
    Point(-10, -10),
    Point(-10, -30),
    Point(10, -30),
    Point(10, -10),
    Point(30, -10),
    Point(10, -10),
    Point(10, -30),
    Point(-10, -30),
    Point(-10, -10),
    Point(-30, -10),
    Point(-30, 10),
    Point(-10, 10),
    Point(-10, 30),
    Point(10, 30),
    Point(10, 10),
    Point(30, 10),
    Point(30, -10),
        Point(30, 10),
    Point(10, 10),
    Point(10, 30),
    Point(-10, 30),
    Point(-10, 10),
    Point(-30, 10),
    Point(-30, -10),
    Point(-10, -10),
    Point(-10, -30),
    Point(10, -30),
    Point(10, -10),
    Point(30, -10),
    Point(10, -10),
    Point(10, -30),
    Point(-10, -30),
    Point(-10, -10),
    Point(-30, -10),
    Point(-30, 10),
    Point(-10, 10),
    Point(-10, 30),
    Point(10, 30),
    Point(10, 10),
    Point(30, 10),
    Point(30, -10),
        Point(30, 10),
    Point(10, 10),
    Point(10, 30),
    Point(-10, 30),
    Point(-10, 10),
    Point(-30, 10),
    Point(-30, -10),
    Point(-10, -10),
    Point(-10, -30),
    Point(10, -30),
    Point(10, -10),
    Point(30, -10),
    Point(10, -10),
    Point(10, -30),
    Point(-10, -30),
    Point(-10, -10),
    Point(-30, -10),
    Point(-30, 10),
    Point(-10, 10),
    Point(-10, 30),
    Point(10, 30),
    Point(10, 10),
    Point(30, 10),
    Point(30, -10),
        Point(30, 10),
    Point(10, 10),
    Point(10, 30),
    Point(-10, 30),
    Point(-10, 10),
    Point(-30, 10),
    Point(-30, -10),
    Point(-10, -10),
    Point(-10, -30),
    Point(10, -30),
    Point(10, -10),
    Point(30, -10),
    Point(10, -10),
    Point(10, -30),
    Point(-10, -30),
    Point(-10, -10),
    Point(-30, -10),
    Point(-30, 10),
    Point(-10, 10),
    Point(-10, 30),
    Point(10, 30),
    Point(10, 10),
    Point(30, 10),
    Point(30, -10)
};

AccelStepper motor_turn(AccelStepper::DRIVER, PIN_MOT_L_DIR, PIN_MOT_L_STP);
AccelStepper motor_linear(AccelStepper::DRIVER, PIN_MOT_R_DIR, PIN_MOT_R_STP);

MultiStepper motors;

Servo myservo;

void initMotors(void)
{
    motor_turn.setPinsInverted(false, false, true);
    motor_turn.setEnablePin(PIN_MOT_L_EN);

    motor_linear.setPinsInverted(false, false, true);
    motor_linear.setEnablePin(PIN_MOT_R_EN);

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
        delay(200); // waits 15ms for the servo to reach the position
    }
    else
    {
        myservo.write(70); // tell servo to go to position in variable 'pos'
        delay(200);        // waits 15ms for the servo to reach the position
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

        return Point(x, y);
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

void moveTo(Point nextPoint, bool draw, Point lastPoint = Point(0, 0))
{
    setPen(draw);
    if (draw && (nextPoint.getDistanceToPoint(lastPoint) > MAX_DISTANCE_BETWEEN_MICROPOINTS))
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
void setup(void)
{
    Serial.begin(115200);

    delay(3000);

    Serial.println("Points:");
    for (int i = 0; i < NUMBER_POINTS; i++)
    {
        Serial.print("Point ");
        Serial.print(i);

        Serial.print(" x=");
        Serial.print(points[i].x);

        Serial.print(" y=");
        Serial.print(points[i].y);

        Serial.print(" radius=");
        Serial.print(points[i].radius);

        Serial.print(" angle=");
        Serial.println(points[i].angle);
    }
    Serial.println();

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

    for (int i = 0; i < NUMBER_POINTS; i++)
    {
        Serial.println();

        Serial.print("Point ");
        Serial.println(i);
        if (i >= 1)
        {
            moveTo(points[i], true, points[i - 1]);
        }
        else
        {
            moveTo(points[i], false);
        }
    }

    home();
}

void loop()
{
}