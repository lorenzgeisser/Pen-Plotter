#ifndef POINT_H
#define POINT_H

#include <Arduino.h>

class Point
{
public:
    double x;
    double y;
    bool inCenter;
    double angle;
    double radius;

    Point(double _x, double _y)
    {
        updatePoint(_x, _y);
    }

    void updatePoint(double _x, double _y)
    {
        x = _x;
        y = _y;

        updateAngle();
        updateRadius();
    }

    double getDistanceToPoint(Point point)
    {
        double delta_x = abs(x - point.x);
        double delta_y = abs(y - point.y);

        return sqrt(sq(delta_x) + sq(delta_y));
    }

private:
    void updateRadius(void)
    {
        double delta_x = abs(0 - x);
        double delta_y = abs(0 - y);

        radius = sqrt(sq(delta_x) + sq(delta_y));
    }

    void updateAngle(void)
    {
        inCenter = false;

        // special cases
        if (x > 0 && y == 0)
        {
            // point is on the positive X axis
            angle = 0;
        }
        else if (x < 0 && y == 0)
        {
            // point is on the negative X axis
            angle = 180;
        }
        else if (x == 0 && y > 0)
        {
            // point is on the positive Y axis
            angle = 90;
        }
        else if (x == 0 && y < 0)
        {
            // point is on the negative Y axis
            angle = 270;
        }
        else if (x == 0 && y == 0)
        {
            // point is in the center
            inCenter = true;
        }
        else
        {
            // angle calculation
             angle = atan(abs(y) / abs(x)) * 180.0 / PI;

            if (x < 0 && y > 0)
            {
                // point is in quadrant 2
                angle = 180-angle ;
            }
            else if (x < 0 && y < 0)
            {
                // point is in quadrant 3
                angle += 180 ;
            }
            else if (x > 0 && y < 0)
            {
                // point is in quadrant 4
                angle = 360-angle ;
            }
        }
    }
};

#endif