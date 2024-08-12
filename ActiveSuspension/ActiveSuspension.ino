#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <ESP32Servo.h>

#define SCALE 10 // servo "degrees" per newton
#define SERVO_MIN 70
#define SERVO_MAX 110
#define SERVO_MID 90
#define SERVO_DIFF 0
#define ANGLE_THRESHOLD 2 // degrees

#define STABLE 0
#define REALISTIC 1

int suspensionMode; // This will be loaded from Preferences
float responseFactor = 0.5; // Default to a realistic response

Preferences preferences; // Preferences object for non-volatile storage
Servo fl, fr, rl, rr; // front-left, front-right, rear-left, rear-right

int d[4]; // for storing angles in "degrees"
int previousAngles[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID}; // stores previous angles

double acx, acy, acz; 

// Calibration offsets
double X_CAL, Y_CAL, Z_CAL;

volatile bool mpuInterrupt = false; // flag to indicate MPU6050 data ready
AsyncWebServer server(80); // Correct server type and port

// Function declarations
void w(uint8_t address, uint8_t value);
void readAccelerometerData(int16_t &ax, int16_t &ay, int16_t &az);

void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

void loadPreferences() {
  X_CAL = preferences.getDouble("X_CAL", 0.0);
  Y_CAL = preferences.getDouble("Y_CAL", 0.0);
  Z_CAL = preferences.getDouble("Z_CAL", 0.0);
  suspensionMode = preferences.getInt("suspensionMode", REALISTIC); // Load the last used suspension mode
}

void savePreference(const char* key, double value) {
  preferences.putDouble(key, value);
}

void saveIntPreference(const char* key, int value) {
  preferences.putInt(key, value);
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  preferences.begin("suspension", false); // Initialize Preferences with namespace "suspension"
  loadPreferences(); // Load saved preferences

  w(0x6b, 0x00); // disable MPU6050 sleep
  w(0x1c, 0b00001000); // maximum accelerometer resolution
  w(0x1a, 0b00000110); // filter setup

  rr.attach(2); // GPIO pins for the servos
  fr.attach(3);
  rl.attach(4);
  fl.attach(5);

  // Attach interrupt for MPU6050 data ready
  pinMode(21, INPUT_PULLUP); // Assume INT pin of MPU6050 is connected to GPIO 21
  attachInterrupt(digitalPinToInterrupt(21), dmpDataReady, FALLING);

  // Wi-Fi setup
  WiFi.softAP("YOURACCESSPOINT", "YOURPASSWORD");
  Serial.println("WiFi AP started with SSID: YOURACCESSPOINT");

  // Global CORS configuration
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  // Web server setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Use the files on your phone to control the suspension.");
  });

  // API to load settings
  server.on("/api/load", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"X_CAL\":" + String(X_CAL) + ",";
    json += "\"Y_CAL\":" + String(Y_CAL) + ",";
    json += "\"Z_CAL\":" + String(Z_CAL) + ",";
    json += "\"suspensionMode\":" + String(suspensionMode);
    json += "}";
    request->send(200, "application/json", json);
  });

  // API to save individual settings
  server.on("/api/save", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("key") && request->hasParam("value")) {
      String key = request->getParam("key")->value();
      String valueStr = request->getParam("value")->value();

      if (key == "suspensionMode") {
        suspensionMode = valueStr.toInt();
        saveIntPreference("suspensionMode", suspensionMode);
      } else {
        double value = valueStr.toDouble();
        savePreference(key.c_str(), value);
      }

      loadPreferences(); // Reload preferences after saving
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  // API to load failsafe settings
  server.on("/api/failsafe", HTTP_GET, [](AsyncWebServerRequest *request){
    // Reset to default/failsafe settings
    preferences.putDouble("X_CAL", 0.0);
    preferences.putDouble("Y_CAL", 0.0);
    preferences.putDouble("Z_CAL", 0.0);
    preferences.putInt("suspensionMode", REALISTIC); // Default to realistic mode
    loadPreferences(); // Reload preferences
    request->send(200, "text/plain", "Failsafe settings loaded");
  });

  // API to reset MPU6050
  server.on("/api/reset_mpu", HTTP_GET, [](AsyncWebServerRequest *request){
    w(0x6b, 0x00); // Reset MPU6050
    request->send(200, "text/plain", "MPU6050 reset");
  });

  server.begin(); // Start the server

  // Dynamic calibration - read initial values
  int16_t ax, ay, az;
  readAccelerometerData(ax, ay, az);
  X_CAL = ax / 819.2;
  Y_CAL = ay / 819.2;
  Z_CAL = az / 819.2;
}

void loop() {
  if (mpuInterrupt) { // Check if MPU6050 data is ready
    mpuInterrupt = false; // Reset interrupt flag

    int16_t ax, ay, az;
    readAccelerometerData(ax, ay, az);

    acx = ax / 819.2 - X_CAL;
    acy = ay / 819.2 - Y_CAL;
    acz = az / 819.2 - Z_CAL;

    Serial.print(acx);
    Serial.print("   ");
    Serial.print(acy);
    Serial.print("   ");
    Serial.print(acz);
    Serial.println();

    if (suspensionMode == STABLE) {
      // Active suspension - car tries to counteract its inertia and compensate its angle
      d[0] = SERVO_MID + (-acz + acy) * SCALE; // rr
      d[1] = SERVO_MID + (acz + acy) * SCALE;  // fr
      d[2] = SERVO_MID + (-acz - acy) * SCALE; // rl
      d[3] = SERVO_MID + (acz - acy) * SCALE;  // fl
    } else if (suspensionMode == REALISTIC) {
      // More realistic simulation - allows some degree of rocking motion
      d[0] = SERVO_MID + (-acz + acy) * SCALE * responseFactor; // rr
      d[1] = SERVO_MID + (acz + acy) * SCALE * responseFactor;  // fr
      d[2] = SERVO_MID + (-acz - acy) * SCALE * responseFactor; // rl
      d[3] = SERVO_MID + (acz - acy) * SCALE * responseFactor;  // fl
    }

    wa(d);
  }
}

// Function to write byte to MPU6050
void w(uint8_t address, uint8_t value) { 
  Wire.beginTransmission(0x68);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read accelerometer data
void readAccelerometerData(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

// Function to write angles to servos, apply calibration and limit max points
void wa(int d[]) { 
  // FIRST - apply diagonal calibration
  d[0] = d[0] - SERVO_DIFF;
  d[1] = d[1] + SERVO_DIFF;
  d[2] = d[2] + SERVO_DIFF;
  d[3] = d[3] - SERVO_DIFF;

  // SECOND - remove overshoots
  for (int i = 0; i < 4; i++) {
    if (d[i] < SERVO_MIN) {
      d[i] = SERVO_MIN;
    } else if (d[i] > SERVO_MAX) {
      d[i] = SERVO_MAX;
    }
  }

  // THIRD - check for significant change before updating servos
  for (int i = 0; i < 4; i++) {
    if (abs(d[i] - previousAngles[i]) >= ANGLE_THRESHOLD) {
      // Invert angle if necessary
      int servoAngle = (i == 0 || i == 3) ? 180 - d[i] : d[i];
      
      // Write to servo
      switch (i) {
        case 0: rr.write(servoAngle); break;
        case 1: fr.write(servoAngle); break;
        case 2: rl.write(servoAngle); break;
        case 3: fl.write(servoAngle); break;
      }
      
      previousAngles[i] = d[i];
    }
  }
}
