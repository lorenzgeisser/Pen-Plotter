#include <Arduino.h>
/**************************************************

HomeX sketch for sensorless homing stepper
TMC2209 with Teensy 4.0
Based on Simple sketch by Teemu Mäntykallio
homes on power up.
send 'x' followed by int to begin stepping.
send 'h' to re-home.
RX on pin 7, TX on pin 8
PDN_UART pin on tmc2209 connected to pin 7
1 k resistor between teensy pins 7 and 8
motor direction controlled by uart:
    driver.shaft(true or false);
**************************************************/

#include <TMCStepper.h>

#define EN_PIN 14 // Enable
//#define DIR_PIN 3 // Direction
#define STEP_PIN 32         // Step
#define STALL_PIN_X 15      // Teensy pin that diag pin is attached to
#define SERIAL_PORT Serial2 // HardwareSerial port for Teensy 4.0
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // Match to your driver

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 50 // [0..255]
int stepTime = 160;
bool startup = true; // set false after homing

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
bool shaftVal = false;
bool stalled_X = false;

long offset = 0;

void stallInterruptX()
{ // flag set when motor stalls
if(millis()>3000+offset){
      Serial.println("stalled");
        delay(3000);
        offset = millis();
}
      
}
void motor(int steps, int stepDelay)
{
    digitalWrite(EN_PIN, LOW);
    driver.shaft(shaftVal);
    for (int i = 0; i < steps; i++)
    {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
        if (stalled_X)
        {
            i = steps;
        }
    }
    digitalWrite(EN_PIN, HIGH);
}
void homeX()
{
    int homeDelay = 160;
    int backSteps = 5000;
    Serial.println("fast homing x");
    shaftVal = true;
    while (!stalled_X)
    { // fast home x
        motor(500000, homeDelay);
    }
    stalled_X = false;
    delay(1000);
    Serial.println("backing off");
    shaftVal = false;
    motor(backSteps, homeDelay);

    Serial.println("slow homing x");
    shaftVal = true;
    while (!stalled_X)
    { // slow home x
        motor(1000, homeDelay * 2);
    }
    stalled_X = false;
    delay(1000);
    Serial.println("backing off");
    shaftVal = false;
    motor(backSteps, homeDelay);
}

void setup()
{

    Serial.begin(115200);

    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    // shaft direction controlled through uart: driver.shaft(true or false)
    pinMode(STALL_PIN_X, INPUT);

    SERIAL_PORT.begin(115200); // HW UART drivers

    driver.begin(); // SPI: Init CS pins and possible SW SPI pins
    driver.toff(4); // Enables driver in software, changed from 5
    driver.blank_time(24);
    driver.rms_current(300); // Set motor RMS current
    driver.microsteps(16);   // Set microsteps to 1/16th

    // driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
    // driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
    driver.pwm_autoscale(true); // Needed for stealthChop
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.shaft(true);
    // TCOOLTHRS needs to be set for stallgaurd to work //
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.SGTHRS(STALL_VALUE);
    attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING);
    digitalWrite(EN_PIN, LOW); // Enable driver in hardware
}

void loop()
{
   digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(100);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(100);
}
