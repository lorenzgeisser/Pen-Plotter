#ifndef MACHINE_H
#define MACHINE_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
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

AccelStepper motor_turn;
AccelStepper motor_linear;

MultiStepper motors;

Servo myservo;

class Machine
{
public:
    bool begin(float speedLinear, float accelerationLinear, float speedRot, float accelerationRot)
    {
        motor_turn = AccelStepper(AccelStepper::DRIVER, PIN_MOT_ROT_STP, PIN_MOT_ROT_DIR);
        motor_linear = AccelStepper(AccelStepper::DRIVER, PIN_MOT_LIN_STP, PIN_MOT_LIN_DIR);

        motor_turn.setPinsInverted(false, false, true);
        motor_turn.setEnablePin(PIN_MOT_ROT_EN);

        motor_linear.setPinsInverted(false, false, true);
        motor_linear.setEnablePin(PIN_MOT_LIN_EN);

        motor_linear.setMaxSpeed(speedLinear);
        motor_linear.setAcceleration(accelerationLinear);

        motor_turn.setMaxSpeed(speedRot);
        motor_turn.setAcceleration(accelerationRot);

        motors.addStepper(motor_turn);
        motors.addStepper(motor_linear);

        disableMotors();

        return true;
    }

    bool enableMotors(void)
    {
        motor_turn.enableOutputs();
        motor_linear.enableOutputs();

        return true;
    }

    bool disableMotors(void)
    {
        motor_turn.disableOutputs();
        motor_linear.disableOutputs();

        return true;
    }

    bool setPen(bool active)
    {
        if (active)
        {
            myservo.write(100);
        }
        else
        {
            myservo.write(70);
        }

        return true;
    }

    bool initServo(int pin)
    {
        myservo.attach(pin);

        return true;
    }

    bool home(void)
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

        return true;
    }

    bool initSwitch(int pin)
    {
        pinMode(pin, INPUT);

        return true;
    }

    void moveTo(Point nextPoint, Point lastPoint = Point(0, 0, false))
    {
        setPen(nextPoint.draw);

        if (nextPoint.draw != lastPoint.draw)
        {
            delay(500);
        }

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

private:
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
};

#endif