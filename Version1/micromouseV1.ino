/*******************************************************
* 10x10cm Grid System
*******************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <VL53L0X.h>

// =====================================================
// ENUMS
// =====================================================
enum Action { 
    MOVE_FORWARD, 
    TURN_LEFT, 
    TURN_RIGHT, 
    TURN_BACK 
};

// =====================================================
// HARDWARE SETTINGS & CONVERSIONS
// =====================================================
#define CELL_SIZE_MM     100.0
#define TRACK_WIDTH_MM   95.0  // Distance between left and right wheels
#define WHEEL_DIA_MM     45.0
#define GEAR_RATIO       50.0
#define ENCODER_PPR      3.0

// Calculations
const float TICKS_PER_REV = ENCODER_PPR * GEAR_RATIO;
const float WHEEL_CIRCUM = WHEEL_DIA_MM * PI;
const float TICKS_PER_MM = TICKS_PER_REV / WHEEL_CIRCUM;

long mmToTicks(float mm) { return (long)(mm * TICKS_PER_MM); }
long angleToTicks(float deg) { 
    float turnDistance = (TRACK_WIDTH_MM * PI) * (deg / 360.0);
    return mmToTicks(turnDistance); 
}

// =====================================================
// PINS
// =====================================================
// Motor Driver (TB6612FNG)
#define STBY_PIN   9
#define AIN1_PIN   11   // Left motor
#define AIN2_PIN   4
#define PWMA_PIN   5
#define BIN1_PIN   6    // Right motor
#define BIN2_PIN   7
#define PWMB_PIN   8

// Encoders (Nano Int0/Int1 on pins 2 and 3)
#define ENC_L_A    3
#define ENC_L_B    A3
#define ENC_R_A    2
#define ENC_R_B    A2

// VL53L0X XSHUT pins
#define XSHUT_FRONT        12
#define XSHUT_FRONT_LEFT   A1
#define XSHUT_FRONT_RIGHT  A0
#define XSHUT_LEFT         10
#define XSHUT_RIGHT        13

// =====================================================
// GLOBALS & OBJECTS
// =====================================================
Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

VL53L0X sensorF;
VL53L0X sensorFL;
VL53L0X sensorFR;
VL53L0X sensorL;
VL53L0X sensorR;

// Distance Readings (mm)
int df = 500, dfl = 500, dfr = 500, dl = 500, dr = 500;

// Speeds & PID
#define FORWARD_SPEED 130
#define MAX_SPEED 130
float Kp_wall = 0.8; 
float Kd_wall = 3.5;

// =====================================================
// MOTOR CONTROL (TB6612FNG)
// =====================================================
void setMotor(int in1, int in2, int pwmPin, int speed) {
    bool dir = speed > 0;
    digitalWrite(in1, dir);
    digitalWrite(in2, !dir);
    
    // If speed is 0, brake
    if (speed == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwmPin, abs(speed));
}

void stopMotors() {
    setMotor(AIN1_PIN, AIN2_PIN, PWMA_PIN, 0);
    setMotor(BIN1_PIN, BIN2_PIN, PWMB_PIN, 0);
}

void resetEncoders() {
    leftEncoder.write(0);
    rightEncoder.write(0);
}

// =====================================================
// SENSOR INITIALIZATION
// =====================================================
void initSensors() {
    digitalWrite(XSHUT_FRONT, LOW);
    digitalWrite(XSHUT_FRONT_LEFT, LOW);
    digitalWrite(XSHUT_FRONT_RIGHT, LOW);
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);
    delay(10);

    digitalWrite(XSHUT_FRONT, HIGH); delay(10);
    sensorF.init(); sensorF.setAddress(0x2A); sensorF.startContinuous();

    digitalWrite(XSHUT_FRONT_LEFT, HIGH); delay(10);
    sensorFL.init(); sensorFL.setAddress(0x2B); sensorFL.startContinuous();

    digitalWrite(XSHUT_FRONT_RIGHT, HIGH); delay(10);
    sensorFR.init(); sensorFR.setAddress(0x2C); sensorFR.startContinuous();

    digitalWrite(XSHUT_LEFT, HIGH); delay(10);
    sensorL.init(); sensorL.setAddress(0x2D); sensorL.startContinuous();

    digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
    sensorR.init(); sensorR.setAddress(0x2E); sensorR.startContinuous();
}

void readSensors() {
    df  = sensorF.readRangeContinuousMillimeters();
    dfl = sensorFL.readRangeContinuousMillimeters();
    dfr = sensorFR.readRangeContinuousMillimeters();
    dl  = sensorL.readRangeContinuousMillimeters();
    dr  = sensorR.readRangeContinuousMillimeters();
}

// =====================================================
// MOVEMENT FUNCTIONS
// =====================================================
void moveForwardPID(int mm) {
    resetEncoders();
    int prevError = 0;
    long targetTicks = mmToTicks(mm);

    while (true) {
        readSensors();
        
        int error = 0;
        bool leftWall = dl < 80;
        bool rightWall = dr < 80;

        if (leftWall && rightWall) {
            error = dl - dr; 
        } else if (leftWall) {
            error = dl - 40; 
        } else if (rightWall) {
            error = 40 - dr; 
        }

        int correction = Kp_wall * error + Kd_wall * (error - prevError);
        prevError = error;

        int lSpeed = constrain(FORWARD_SPEED - correction, -MAX_SPEED, MAX_SPEED);
        int rSpeed = constrain(FORWARD_SPEED + correction, -MAX_SPEED, MAX_SPEED);

        setMotor(AIN1_PIN, AIN2_PIN, PWMA_PIN, lSpeed);
        setMotor(BIN1_PIN, BIN2_PIN, PWMB_PIN, rSpeed);

        long traveled = (abs(leftEncoder.read()) + abs(rightEncoder.read())) / 2;
        if (traveled >= targetTicks) break;
        
        if (df < 30) break; 
    }
    stopMotors();
}

void rotate(int dir, float angle) {
    resetEncoders();
    long ticks = angleToTicks(angle);
    
    while ((abs(leftEncoder.read()) + abs(rightEncoder.read())) / 2 < ticks) {
        setMotor(AIN1_PIN, AIN2_PIN, PWMA_PIN, dir * FORWARD_SPEED);
        setMotor(BIN1_PIN, BIN2_PIN, PWMB_PIN, -dir * FORWARD_SPEED);
    }
    stopMotors();
}

void reverseUntilOpen() {
    int prevError = 0;
    
    // Keep moving backward until either the left or right wall disappears
    while (true) {
        readSensors();
        
        // If either sensor reads > 80mm, a wall is missing (intersection found)
        if (dl > 80 || dr > 80) {
            break;
        }

        // Calculate centering error
        int error = 0;
        bool leftWall = dl < 80;
        bool rightWall = dr < 80;

        if (leftWall && rightWall) {
            error = dl - dr; 
        } else if (leftWall) {
            error = dl - 40; 
        } else if (rightWall) {
            error = 40 - dr; 
        }

        // Calculate PID correction
        int correction = Kp_wall * error + Kd_wall * (error - prevError);
        prevError = error;

        // Apply correction to reverse speeds
        // Note: The + and - are swapped compared to forward movement 
        // to steer the rear of the robot correctly!
        int lSpeed = constrain(-FORWARD_SPEED + correction, -MAX_SPEED, 0);
        int rSpeed = constrain(-FORWARD_SPEED - correction, -MAX_SPEED, 0);

        setMotor(AIN1_PIN, AIN2_PIN, PWMA_PIN, lSpeed);
        setMotor(BIN1_PIN, BIN2_PIN, PWMB_PIN, rSpeed);
    }
    
    // Stop once an opening is detected
    stopMotors();
}
// =====================================================
// DECISION ENGINE
// =====================================================
Action decideNextMove() {
    bool wallFront = df < 80;
    bool wallLeft  = dl < 80;
    bool wallRight = dr < 80;

    if (!wallFront) return MOVE_FORWARD;
    if (!wallLeft && wallRight) return TURN_LEFT;
    if (!wallRight && wallLeft) return TURN_RIGHT;
    if (!wallLeft && !wallRight) return TURN_LEFT; 
    return TURN_BACK; 
}

// =====================================================
// SETUP & LOOP
// =====================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); 

    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH); 
    pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT); pinMode(PWMA_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT); pinMode(PWMB_PIN, OUTPUT);

    pinMode(XSHUT_FRONT, OUTPUT);
    pinMode(XSHUT_FRONT_LEFT, OUTPUT);
    pinMode(XSHUT_FRONT_RIGHT, OUTPUT);
    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);

    initSensors();
    delay(1000); 
}

void loop() {
    readSensors();
    Action action = decideNextMove();

    switch (action) {
        case MOVE_FORWARD:
            moveForwardPID(CELL_SIZE_MM);
            break;
        case TURN_LEFT:
            rotate(-1, 90);
            break;
        case TURN_RIGHT:
            rotate(1, 90);
            break;
        case TURN_BACK:
            reverseUntilOpen();
            break;
    }
    delay(100); 
}
