#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>

Preferences preferences;
#define LED_BUILTIN 2

WebServer server(80);

// Function prototypes
void readSettings();
void setupWiFi();
void setupServer();
void setupMPU6050();
double gc(uint8_t address);
void w(uint8_t address, uint8_t value);
void wa(int d[]);
void flashLED(int flashes);

Servo fl, fr, rl, rr;

void flashLED(int flashes) {
  // for (int i = 0; i < flashes; i++) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(500);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(500);
  // }
  
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  preferences.begin("settings", false);

  readSettings();
  setupWiFi();
  setupServer();
  setupMPU6050();
}

void loop() {
  server.handleClient();

  double acx = gc(0x3b) - preferences.getFloat("vs_X_CAL", 0.0);
  double acy = gc(0x3d) - preferences.getFloat("vs_Y_CAL", 0.0);
  double acz = gc(0x3f) - preferences.getFloat("vs_Z_CAL", 0.0);

  Serial.print(acx);
  Serial.print("   ");
  Serial.print(acy);
  Serial.print("   ");
  Serial.print(acz);
  Serial.println();

  int d[4];
  d[0] = preferences.getInt("vs_SERVO_MID", 90) + (-acz + acy) * preferences.getInt("vs_SCALE", 10);
  d[1] = preferences.getInt("vs_SERVO_MID", 90) + (acz + acy) * preferences.getInt("vs_SCALE", 10);
  d[2] = preferences.getInt("vs_SERVO_MID", 90) + (-acz - acy) * preferences.getInt("vs_SCALE", 10);
  d[3] = preferences.getInt("vs_SERVO_MID", 90) + (acz - acy) * preferences.getInt("vs_SCALE", 10);

  wa(d);
  delay(10);
}

void readSettings() {
  Serial.println("Settings read from Preferences:");
  Serial.printf("vs_rideheight = %.1f\n", preferences.getFloat("vs_rideheight", 1.0));
  Serial.printf("vs_multiplier = %.1f\n", preferences.getFloat("vs_multiplier", 1.0));
  Serial.printf("vs_balance = %.1f\n", preferences.getFloat("vs_balance", 0.0));
  Serial.printf("vs_range = %.1f\n", preferences.getFloat("vs_range", 0.0));
  Serial.printf("vs_speed = %.1f\n", preferences.getFloat("vs_speed", 1.5));
  Serial.printf("vs_damping = %.1f\n", preferences.getFloat("vs_damping", 2.0));
  Serial.printf("vs_SCALE = %d\n", preferences.getInt("vs_SCALE", 10));
  Serial.printf("vs_SERVO_MIN = %d\n", preferences.getInt("vs_SERVO_MIN", 70));
  Serial.printf("vs_SERVO_MAX = %d\n", preferences.getInt("vs_SERVO_MAX", 110));
  Serial.printf("vs_SERVO_MID = %d\n", preferences.getInt("vs_SERVO_MID", 90));
  Serial.printf("vs_SERVO_DIFF = %d\n", preferences.getInt("vs_SERVO_DIFF", 0));
  Serial.printf("vs_X_CAL = %.1f\n", preferences.getFloat("vs_X_CAL", 0.0));
  Serial.printf("vs_Y_CAL = %.1f\n", preferences.getFloat("vs_Y_CAL", 0.0));
  Serial.printf("vs_Z_CAL = %.1f\n", preferences.getFloat("vs_Z_CAL", 0.0));

  flashLED(1);
}

void setupWiFi() {
  String STASSID = preferences.getString("adm_STASSID", "CAMELOT");
  String STAPassword = preferences.getString("adm_STAPassword", "bluedaisy347");

  WiFi.begin(STASSID.c_str(), STAPassword.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected to ");
  Serial.println(STASSID);
  IPAddress localIP = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(localIP);

  // Update the IP address in preferences
  preferences.putString("adm_ipaddress", localIP.toString());
}

void setupServer() {
  server.on("/data", HTTP_GET, [](){
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");

    String json = "{";
    json += "\"adm_STASSID\":\"" + preferences.getString("adm_STASSID", "CAMELOT") + "\",";
    json += "\"adm_STAPassword\":\"" + preferences.getString("adm_STAPassword", "bluedaisy347") + "\",";
    json += "\"adm_ipaddress\":\"" + preferences.getString("adm_ipaddress", "0.0.0.0") + "\",";
    json += "\"vs_rideheight\":" + String(preferences.getFloat("vs_rideheight", 1.0)) + ",";
    json += "\"vs_multiplier\":" + String(preferences.getFloat("vs_multiplier", 1.0)) + ",";
    json += "\"vs_balance\":" + String(preferences.getFloat("vs_balance", 0.0)) + ",";
    json += "\"vs_range\":" + String(preferences.getFloat("vs_range", 0.0)) + ",";
    json += "\"vs_speed\":" + String(preferences.getFloat("vs_speed", 1.5)) + ",";
    json += "\"vs_damping\":" + String(preferences.getFloat("vs_damping", 2.0)) + ",";
    json += "\"vs_SCALE\":" + String(preferences.getInt("vs_SCALE", 10)) + ",";
    json += "\"vs_SERVO_MIN\":" + String(preferences.getInt("vs_SERVO_MIN", 70)) + ",";
    json += "\"vs_SERVO_MAX\":" + String(preferences.getInt("vs_SERVO_MAX", 110)) + ",";
    json += "\"vs_SERVO_MID\":" + String(preferences.getInt("vs_SERVO_MID", 90)) + ",";
    json += "\"vs_SERVO_DIFF\":" + String(preferences.getInt("vs_SERVO_DIFF", 0)) + ",";
    json += "\"vs_X_CAL\":" + String(preferences.getFloat("vs_X_CAL", 0.0)) + ",";
    json += "\"vs_Y_CAL\":" + String(preferences.getFloat("vs_Y_CAL", 0.0)) + ",";
    json += "\"vs_Z_CAL\":" + String(preferences.getFloat("vs_Z_CAL", 0.0));
    json += "}";

    server.send(200, "application/json", json);
    flashLED(1);
  });

  server.on("/update", HTTP_POST, [](){
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");

    if (server.hasArg("value")) {
      String paramValue = server.arg("value");
      int separatorIndex = paramValue.indexOf('=');
      if (separatorIndex != -1) {
        String setting = paramValue.substring(0, separatorIndex);
        String value = paramValue.substring(separatorIndex + 1);
        updateSetting(setting, value);
        server.send(200, "application/json", "{\"message\":\"Setting updated successfully\"}");
      } else {
        server.send(400, "application/json", "{\"message\":\"Invalid setting format\"}");
        flashLED(3);
      }
    } else {
      server.send(400, "application/json", "{\"message\":\"No value parameter provided\"}");
      flashLED(3);
    }
  });

  server.begin();
}

void updateSetting(const String& setting, const String& value) {
  if (setting.startsWith("vs_")) {
    preferences.putFloat(setting.c_str(), value.toFloat());
  } else {
    preferences.putString(setting.c_str(), value);
  }

  flashLED(1);
}

void setupMPU6050() {
  Wire.begin();
  w(0x6b, 0x00); // disable mpu6050 sleep
  w(0x1c, 0b00001000); // maximum accelerometer resolution
  w(0x1a, 0b00000110); // filter setup

  fl.attach(2);
  fr.attach(3);
  rl.attach(4);
  rr.attach(5);
}

void w(uint8_t address, uint8_t value) {
  Wire.beginTransmission(0x68);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

double gc(uint8_t address) {
  Wire.beginTransmission(0x68);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2);
  return (Wire.read() << 8 | Wire.read()) / 819.2;
}

void wa(int d[]) {
  d[0] = d[0] - preferences.getInt("vs_SERVO_DIFF", 0);
  d[1] = d[1] + preferences.getInt("vs_SERVO_DIFF", 0);
  d[2] = d[2] + preferences.getInt("vs_SERVO_DIFF", 0);
  d[3] = d[3] - preferences.getInt("vs_SERVO_DIFF", 0);

  for (int i = 0; i < 4; i++) {
    if (d[i] < preferences.getInt("vs_SERVO_MIN", 70)) {
      d[i] = preferences.getInt("vs_SERVO_MIN", 70);
    } else if (d[i] > preferences.getInt("vs_SERVO_MAX", 110)) {
      d[i] = preferences.getInt("vs_SERVO_MAX", 110);
    }
  }

  d[0] = 180 - d[0];
  d[1] = d[1];
  d[2] = d[2];
  d[3] = 180 - d[3];

  rr.write(d[0]);
  fr.write(d[1]);
  rl.write(d[2]);
  fl.write(d[3]);
}


