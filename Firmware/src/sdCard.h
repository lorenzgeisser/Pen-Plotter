#ifndef SDCARD_H
#define SDCARD_H

#include "point.h"
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>

#define PIN_SD_CS A1

class SdCard
{
public:
    bool begin(void)
    {
        return SD.begin(PIN_SD_CS);
    }

    bool openFile(String _filename)
    {
        myFile = SD.open(_filename, FILE_READ);

        return myFile;
    }

    bool closeFile(void)
    {
        myFile.close();

        return true;
    }

    bool getNextPoint(Point *p_)
    {
        while (myFile.available())
        {
            String currentChar = myFile.readStringUntil('\n');

            if (currentChar.startsWith("G1"))
            {
                // new coordinates
                for (int i = 0; i < currentChar.length(); i++)
                {
                    char singleCHar = currentChar.charAt(i);

                    switch (singleCHar)
                    {
                    case 'X':
                        p_->x = currentChar.substring(i + 1).toDouble();
                        break;

                    case 'Y':
                        p_->y = currentChar.substring(i + 1).toDouble();
                        break;

                    default:
                        break;
                    }
                }

                return true;
            }
            else if (currentChar.startsWith("M300"))
            {
                // new pen command
                double value = currentChar.substring(6).toDouble();

                if (value == 30)
                {
                    // pen down
                    p_->draw = true;

                    return true;
                }
                else if (value == 50)
                {
                    // pen up
                    p_->draw = false;

                    return true;
                }
            }
        }

        return false;
    }

private:
    File myFile;
};

#endif