#include <LMotorController.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define MIN_ABS_SPEED 30

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// PID Variables
double setpoint = 0.0;  // Adjust for better balance
double input, output;

// PID Gains (Adjust based on testing)
double Kp = 45;   //60 2.2 270 30   50,3.5,230   55,3.1,250 --> 30
double Kd = 1.5;
double Ki = 150;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor Setup
int ENA = 5;
int IN1 = 7;
int IN2 = 8;
int IN3 = 9;
int IN4 = 10;
int ENB = 6;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 0.9, 0.9);

void setup() {
    Wire.begin();
    Serial.begin(115200);

    if (!bno.begin()) {
        Serial.println("Error: BNO055 not detected!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    // Setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(2);
    pid.SetOutputLimits(-255, 255);
}

void loop() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    input = euler.y();  // Read pitch angle

    pid.Compute();  // Run PID calculation

    motorController.move(-output, MIN_ABS_SPEED);  // Move motors
}
