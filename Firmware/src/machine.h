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

#define MOTOR_STEPS_FULL 200 * 16
#define MOTOR_STEPS_DEG ((MOTOR_STEPS_FULL) / 360.0)

#define STEPS_FULL_ROTATION 200 * 16
#define STEPS_HALF_ROTATION 200 * 16 / 2

#define STEPS_PER_MM (8060 / 102.0)

#define MAX_DISTANCE_BETWEEN_MICROPOINTS 4

#define MAX_DISTANCE_HOMING_MM 300

#define DISTANCE_SWITCH_MOVE_AWAY_MM 1

#define DISTANCE_SWITCH_TO_MIDDLE_MM 70.5

#define TIME_TO_MOVE_PEN_MS 500

AccelStepper motor_turn;
AccelStepper motor_linear;

MultiStepper motors;

SdCard sdCard;

Servo myservo;

Point homePoint(DISTANCE_SWITCH_TO_MIDDLE_MM, 0);

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

    void setMotorTarget(long rot, long lin)
    {
        long tempSteps[] = {rot, lin};
        motors.moveTo(tempSteps);
    }

    bool home(bool &isFinished, bool &isFirst)
    {
        if (isFirst)
        {
            // Reset isFirst
            isFirst = false;

            // Move pen up
            setPen(false);

            // Reset absolute positions of motors
            motor_linear.setCurrentPosition(0);
            motor_turn.setCurrentPosition(0);

            // Move linear motor until switch is pressed
            setMotorTarget(0, MAX_DISTANCE_HOMING_MM * STEPS_PER_MM);
        }

        if (!digitalRead(PIN_ENDSWITCH))
        {
            // The linear axis has reached the limit switch

            // Set absolute position of linear motor
            motor_linear.setCurrentPosition(homePoint.x * STEPS_PER_MM);

            isFinished = true;
        }
        else if (!motors.run())
        {
            // The linear axis has not reached the limit switch

            return false;
        }

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
                delay(TIME_TO_MOVE_PEN_MS);
            }

            pointFinished = false;
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

    bool initSwitch(int pin)
    {
        pinMode(pin, INPUT);

        return true;
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
        // prenvent overflow of currentPosition
        while (motor_turn.currentPosition() > STEPS_FULL_ROTATION)
        {
            motor_turn.setCurrentPosition(motor_turn.currentPosition() - STEPS_FULL_ROTATION);
        }

        // change negative currentPosition to positive
        while (motor_turn.currentPosition() < 0)
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