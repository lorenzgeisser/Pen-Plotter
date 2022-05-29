#ifndef MACHINE_H
#define MACHINE_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include "point.h"

// Pins
const int PinMotorRotEn = 6;
const int PinMotorRotDir = 2;
const int PinMotorRotStp = 3;

const int PinMotorLinEn = 6;
const int PinMotorLinDir = 10;
const int PinMotorLinStp = A0;

const int PinLimitSwitch = 4;

const int PinServoMotor = 5;

// Motor speed and acceleration
const float SpeedMotorRot = 1000.0;
const float AccelerationMotorRot = 50.0;

const float SpeedMotorLin = 1000.0;
const float AccelerationMotorLin = 50.0;

// Machine parameters
const double MotorMicrosteps = 16.0;
const double MotorStepsFullRotation = 200.0 * MotorMicrosteps;
const double MotorStepsHalfRotation = MotorStepsFullRotation / 2.0;

const double MotorRotStepsPerDeg = MotorStepsFullRotation / 360.0;
const double MotorLinStepsPerMm = 79.0196;

const double DistanceMaxBetweenMicropointsMm = 1.0;

const double DistanceMaxHomingMm = 300;

const double DistanceLimitSwitchToMiddleMm = 70.5;

const int TimeToMovePenMs = 1000;

// Definition homing point
Point homePoint(DistanceLimitSwitchToMiddleMm, 0);

// Instances
AccelStepper motor_turn;
AccelStepper motor_linear;
MultiStepper motors;
SdCard sdCard;
Servo myservo;

class Machine
{
public:
    // Initializes the stepper motors, the servo motor and the limit switch
    void init(void)
    {
        // Initialize stepper motors
        motor_turn = AccelStepper(AccelStepper::DRIVER, PinMotorRotStp, PinMotorRotDir);
        motor_linear = AccelStepper(AccelStepper::DRIVER, PinMotorLinStp, PinMotorLinDir);

        motor_turn.setPinsInverted(false, false, true);
        motor_turn.setEnablePin(PinMotorRotEn);

        motor_linear.setPinsInverted(false, false, true);
        motor_linear.setEnablePin(PinMotorLinEn);

        // Configure motor speed an acceleration
        motor_linear.setMaxSpeed(SpeedMotorLin);
        motor_linear.setAcceleration(AccelerationMotorLin);

        motor_turn.setMaxSpeed(SpeedMotorRot);
        motor_turn.setAcceleration(AccelerationMotorRot);

        // Add stepper motors to the milti stepper object
        motors.addStepper(motor_turn);
        motors.addStepper(motor_linear);

        // Disable stepper motors
        disableStepperMotors();

        // Clear motor queue to stop motors
        clearMotorQueue();

        // Initialize servo motor
        initServo(PinServoMotor);

        // Initialize limit switch
        initSwitch(PinLimitSwitch);
    }

    // Enables stepper motors
    void enableStepperMotors(void)
    {
        motor_turn.enableOutputs();
        motor_linear.enableOutputs();
    }

    // Disables stepper motors
    void disableStepperMotors(void)
    {
        motor_turn.disableOutputs();
        motor_linear.disableOutputs();
    }

    // Homes linear axis using limit switch
    bool homeLinAxis(bool &isFinished, bool &isFirst)
    {
        static uint32_t millisAtSetPenMs;

        if (isFirst)
        {
            // Reset isFirst
            isFirst = false;

            // Move pen up
            setPen(false);

            // Store current time in millis
            millisAtSetPenMs = millis();

            // Move linear motor until switch is pressed
            setMotorTarget(0, DistanceMaxHomingMm * MotorLinStepsPerMm);
        }

        // Wait until pen is moved to position
        if (millis() < (millisAtSetPenMs + TimeToMovePenMs))
        {
            // Return without error
            return true;
        }

        if (!motors.run())
        {
            // => The linear axis has not reached the limit switch

            // Return with error
            return false;
        }

        // Check if limit switch is pressed
        if (!digitalRead(PinLimitSwitch))
        {
            // => Limit switch is pressed

            // Set absolute position of linear motor
            motor_linear.setCurrentPosition(homePoint.x * MotorLinStepsPerMm);

            isFinished = true;

            // Return without error
            return true;
        }

        // Return without error
        return true;
    }

    void clearMotorQueue(void)
    {
        setMotorTarget(motor_turn.currentPosition(), motor_linear.currentPosition());
    }

    bool draw(bool &isFinished, bool &isFirst, String filename)
    {
        static bool pointFinished;

        static Point nextPoint;
        static Point lastPoint;
        static Point currentMicropoint;

        static long numberOfMicropoints;
        static double angleRadMicropoints;

        static long currentMicroPointIndex;

        static uint32_t millisWait = 0;

        if (isFirst)
        {
            // Reset isFirst
            isFirst = false;

            // Set last point and next point to home point
            lastPoint = homePoint;
            nextPoint = homePoint;

            // Open file
            if (!sdCard.openFile(filename))
            {
                return false;
            }

            // Reset variables
            pointFinished = true;

            clearMotorQueue();
        }

        if (pointFinished)
        {
            // Store last point
            lastPoint = nextPoint;

            // Get next point
            if (!sdCard.getNextPoint(&nextPoint))
            {
                // => No point found
                isFinished = true;

                // Close file
                sdCard.closeFile();

                return true;
            }

            // Update points
            nextPoint.updatePoint();
            lastPoint.updatePoint();

            // Get number of micropoints
            numberOfMicropoints = getNumberOfMicropoints(nextPoint.getDistanceToPoint(lastPoint));

            // Calculation of the angle of micro points when they are needed
            angleRadMicropoints = getAngleOfMicropoints(nextPoint, lastPoint);
            currentMicroPointIndex = 0;

            currentMicropoint.updatePoint();

            // Set pen for next point
            setPen(nextPoint.draw);

            // Wait for pen to move
            if (nextPoint.draw != lastPoint.draw)
            {
                millisWait = millis() + TimeToMovePenMs;
            }
            else
            {
                millisWait = 0;
            }

            pointFinished = false;
        }

        if (millis() < millisWait)
        {
            return true;
        }

        if (!motors.run())
        {
            if (currentMicroPointIndex <= numberOfMicropoints)
            {

                Serial.println("new micro");

                // => Get new micro point
                currentMicropoint = getCurrentMicropoint(currentMicroPointIndex, numberOfMicropoints, angleRadMicropoints, nextPoint, lastPoint);
                currentMicropoint.updatePoint();

                setMotorTarget(getFastestRotAbs(currentMicropoint), getLinPos(currentMicropoint));

                currentMicroPointIndex++;
            }
            else
            {
                Serial.println("next one please");

                pointFinished = true;
            }
        }

        return true;
    }

private:
    // Sets pen position
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

    // Sets new absolute motor target mosition
    void setMotorTarget(long rot, long lin)
    {
        long tempSteps[] = {rot, lin};
        motors.moveTo(tempSteps);
    }

    bool initServo(int pin)
    {
        myservo.attach(pin);

        return true;
    }

    bool initSwitch(int pin)
    {
        pinMode(pin, INPUT);

        return true;
    }

    long getNumberOfMicropoints(double distancePoints)
    {
        double tempNumber = abs(distancePoints);
        long number = 0;

        while (tempNumber > DistanceMaxBetweenMicropointsMm)
        {
            number++;
            tempNumber -= DistanceMaxBetweenMicropointsMm;
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
            double newDenta_x = cos(angleOfMicropoints) * currentMicropoint * DistanceMaxBetweenMicropointsMm;
            double newDenta_y = sin(angleOfMicropoints) * currentMicropoint * DistanceMaxBetweenMicropointsMm;

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
        // prenvent overflow of currentPosition
        while (motor_turn.currentPosition() > MotorStepsFullRotation)
        {
            motor_turn.setCurrentPosition(motor_turn.currentPosition() - MotorStepsFullRotation);
        }

        // change negative currentPosition to positive
        while (motor_turn.currentPosition() < 0)
        {
            motor_turn.setCurrentPosition(motor_turn.currentPosition() + MotorStepsFullRotation);
        }

        long currentPositionRot = motor_turn.currentPosition();

        long nextPositionRot = nextPoint.angle * MotorRotStepsPerDeg;

        long deltaMove = nextPositionRot - currentPositionRot;

        if (deltaMove > MotorStepsHalfRotation)
        {
            deltaMove -= MotorStepsFullRotation;
        }
        else if (deltaMove < (-1 * MotorStepsHalfRotation))
        {
            deltaMove += MotorStepsFullRotation;
        }

        return currentPositionRot + deltaMove;
    }

    long getLinPos(Point nextPoint)
    {
        return MotorLinStepsPerMm * nextPoint.radius;
    }
};

#endif