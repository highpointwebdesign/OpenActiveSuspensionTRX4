#include <Wire.h>
#include <Servo.h>

// Servo limits and calibration offsets
#define SCALE 10
#define SERVO_MIN 70
#define SERVO_MAX 110
#define SERVO_MID 90
#define SERVO_DIFF 0
#define X_CAL 0.0
#define Y_CAL 0.0
#define Z_CAL 0.0

// RC input settings
#define RC_CHANNEL_PIN 6
#define LOW_POSITION 1000
#define HIGH_POSITION 2000
#define NEUTRAL_POSITION 1500

// Servo objects
Servo frontLeft, frontRight, rearLeft, rearRight;
int servoAngles[4]; 

void setup() {
    initializeSystem();
    attachServos();
}

void loop() {
    double acx, acy, acz;
    int mode = suspensionMode();

    if (mode == 0) { // Active suspension
        readAccelerations(acx, acy, acz);
        calculateServoAngles(acx, acy, acz);
    } else {
        setFixedPosition(mode);
    }
    
    adjustServos(servoAngles);
    delay(10);
}

void initializeSystem() {
    Wire.begin();
    Serial.begin(115200);
    setupMPU6050();
}

void setupMPU6050() {
    writeMPU6050(0x6b, 0x00); // Wake up MPU6050
    writeMPU6050(0x1c, 0b00001000); // Set accelerometer resolution
    writeMPU6050(0x1a, 0b00000110); // Configure filter
}

void attachServos() {
    pinMode(RC_CHANNEL_PIN, INPUT); // Setup RC input pin
    frontRight.attach(3);
    rearLeft.attach(4);
    frontLeft.attach(5);
    rearRight.attach(2);
}

void readAccelerations(double &acx, double &acy, double &acz) {
    acx = getConvertedAcceleration(0x3b) - X_CAL;
    acy = getConvertedAcceleration(0x3d) - Y_CAL;
    acz = getConvertedAcceleration(0x3f) - Z_CAL;
}

double getConvertedAcceleration(uint8_t address) {
    return readTwoBytes(address) / 819.2;
}

void calculateServoAngles(double acx, double acy, double acz) {
    servoAngles[0] = SERVO_MID + (int)(-acz + acy) * SCALE;
    servoAngles[1] = SERVO_MID + (int)(acz + acy) * SCALE;
    servoAngles[2] = SERVO_MID + (int)(-acz - acy) * SCALE;
    servoAngles[3] = SERVO_MID + (int)(acz - acy) * SCALE;
}

void adjustServos(int angles[]) {
    applyCalibrationAndLimits(angles);
    rearRight.write(180 - angles[0]);
    frontRight.write(angles[1]);
    rearLeft.write(angles[2]);
    frontLeft.write(180 - angles[3]);
}

void applyCalibrationAndLimits(int angles[]) {
    for (int i = 0; i < 4; i++) {
        angles[i] = constrain(angles[i], SERVO_MIN, SERVO_MAX);
    }
    angles[0] -= SERVO_DIFF;
    angles[1] += SERVO_DIFF;
    angles[2] += SERVO_DIFF;
    angles[3] -= SERVO_DIFF;
}

void writeMPU6050(uint8_t address, uint8_t value) {
    Wire.beginTransmission(0x68);
    Wire.write(address);
    Wire.write(value);
    int error = Wire.endTransmission();
    if (error) {
        Serial.print("I2C error: ");
        Serial.println(error);
    }
}

int16_t readTwoBytes(uint8_t address) {
    Wire.beginTransmission(0x68);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 2);
    if (Wire.available() == 2) {
        return (Wire.read() << 8) | Wire.read();
    }
    return 0; // Default value or error handling
}

int suspensionMode() {
    int pwmValue = pulseIn(RC_CHANNEL_PIN, HIGH, 25000); // Read the PWM signal
    if (pwmValue <= LOW_POSITION + 100) { // Adjust threshold as needed
        return 1; // Low position
    } else if (pwmValue >= HIGH_POSITION - 100) {
        return 2; // High position
    } else {
        return 0; // Active suspension
    }
}

void setFixedPosition(int mode) {
    int angle = SERVO_MID;
    if (mode == 1) { // Set all to lowest position
        angle = SERVO_MIN;
    } else if (mode == 2) { // Set all to highest position
        angle = SERVO_MAX;
    }
    
    for (int i = 0; i < 4; i++) {
        servoAngles[i] = angle;
    }
}
