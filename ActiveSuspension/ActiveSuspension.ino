#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

MPU6050 mpu;
Servo frontLeftServo, frontRightServo, rearLeftServo, rearRightServo;
Preferences preferences;
// Create a web server object that listens on port 80
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Network credentials
  //  Station
  const char* ssidSTA = "YOURNETWORK";
  const char* passwordSTA = "YOURPASSWORD";
  // Access Point
  const char* ssidAP = "ACCESSPOINT";
  const char* passwordAP = "ACCESSPOINTPASSWORD";

//  variable declarations
  int midPoint = 90;
  float multiplier = 1.0;
  float balance = 0.0;
  int rangeMin = 0;
  int rangeMax = 180;
  int reactionSpeed = 100;

void setup() {

// Initialize serial communication
 Serial.begin(115200);
 delay(1000);

//  GPIO Designation
 frontLeftServo.attach(13);
 frontRightServo.attach(12);
 rearLeftServo.attach(14);
 rearRightServo.attach(27);

 preferences.begin("suspension", false);
 midPoint = preferences.getInt("midPoint", 90);
 multiplier = preferences.getFloat("multiplier", 1.0);
 balance = preferences.getFloat("balance", 0.0);
 rangeMin = preferences.getInt("rangeMin", 0);
 rangeMax = preferences.getInt("rangeMax", 180);
 reactionSpeed = preferences.getInt("reactionSpeed", 100);

 // Set up WiFi as Access Point
 WiFi.mode(WIFI_AP_STA);
 WiFi.softAP(ssidAP, passwordAP);

 // Print the IP address of the AP
 IPAddress apIP = WiFi.softAPIP();
 Serial.print("AP IP address: ");
 Serial.println(apIP);

 // Setting up STA mode
 WiFi.begin(ssidSTA, passwordSTA);

 // Wait for STA connection
 Serial.println("Connecting to WiFi.");
 while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.println(".");
 }
 // Output STA IP address and hostname
 Serial.print("Connected to WiFi. IP address: ");
 Serial.println(WiFi.localIP());
 Serial.print("Hostname: ");
 Serial.println(WiFi.getHostname());

 // Define the root path handling function
 server.on("/", handleRoot);
 server.begin();
 Serial.println("HTTP server started");

 webSocket.begin();
 webSocket.onEvent(webSocketEvent);
 Serial.println("WebSocket server started");
}

void loop() {
 // Handle client requests
 server.handleClient();
 webSocket.loop();
}

void handleRoot() {
 String html = "<html><body>";
 html += "<h1>Suspension Control</h1>";
 html += "<form>";
 html += "Mid Point: <button onclick=\"updateValue('midPoint', -1)\" id=\"decrementMidPoint\">-</button>";
 html += "<input type=\"text\" id=\"midPoint\" value=\"" + String(midPoint) + "\" readonly>";
 html += "<button onclick=\"updateValue('midPoint', 1)\" id=\"incrementMidPoint\">+</button><br>";
 html += "Multiplier: <button onclick=\"updateValue('multiplier', -0.05)\" id=\"decrementMultiplier\">-</button>";
 html += "<input type=\"text\" id=\"multiplier\" value=\"" + String(multiplier) + "\" readonly>";
 html += "<button onclick=\"updateValue('multiplier', 0.05)\" id=\"incrementMultiplier\">+</button><br>";
 html += "Balance: <button onclick=\"updateValue('balance', -0.1)\" id=\"decrementBalance\">-</button>";
 html += "<input type=\"text\" id=\"balance\" value=\"" + String(balance) + "\" readonly>";
 html += "<button onclick=\"updateValue('balance', 0.1)\" id=\"incrementBalance\">+</button><br>";
 html += "Range Min: <button onclick=\"updateValue('rangeMin', -1)\" id=\"decrementRangeMin\">-</button>";
 html += "<input type=\"text\" id=\"rangeMin\" value=\"" + String(rangeMin) + "\" readonly>";
 html += "<button onclick=\"updateValue('rangeMin', 1)\" id=\"incrementRangeMin\">+</button><br>";
 html += "Range Max: <button onclick=\"updateValue('rangeMax', -1)\" id=\"decrementRangeMax\">-</button>";
 html += "<input type=\"text\" id=\"rangeMax\" value=\"" + String(rangeMax) + "\" readonly>";
 html += "<button onclick=\"updateValue('rangeMax', 1)\" id=\"incrementRangeMax\">+</button><br>";
 html += "Reaction Speed: <button onclick=\"updateValue('reactionSpeed', -10)\" id=\"decrementReactionSpeed\">-</button>";
 html += "<input type=\"text\" id=\"reactionSpeed\" value=\"" + String(reactionSpeed) + "\" readonly>";
 html += "<button onclick=\"updateValue('reactionSpeed', 10)\" id=\"incrementReactionSpeed\">+</button><br>";
 html += "</form>";
 html += "<script>";
 html += "var ws = new WebSocket('ws://' + location.hostname + ':81/');";
 html += "ws.onmessage = function(event) {";
 html += " var data = event.data.split(',');";
 html += " data.forEach(function(item) {";
 html += "  var pair = item.split(':');";
 html += "  var element = document.getElementById(pair[0]);";
 html += "  if (element) {";
 html += "   element.value = pair[1];";
 html += "   document.getElementById('decrement' + pair[0].charAt(0).toUpperCase() + pair[0].slice(1)).disabled = false;";
 html += "   document.getElementById('increment' + pair[0].charAt(0).toUpperCase() + pair[0].slice(1)).disabled = false;";
 html += "  } else {";
 html += "   console.error('Element not found: ' + pair[0]);";
 html += "  }";
 html += " });";
 html += "};";
 html += "function updateValue(id, increment) {";
 html += " var element = document.getElementById(id);";
 html += " var newValue = parseFloat(element.value) + increment;";
 html += " if (id === 'midPoint' || id === 'rangeMin' || id === 'rangeMax') {";
 html += "  newValue = Math.max(0, Math.min(180, newValue));";
 html += " } else if (id === 'multiplier') {";
 html += "  newValue = Math.max(0.05, Math.min(2.0, newValue));";
 html += " } else if (id === 'balance') {";
 html += "  newValue = Math.max(-1.0, Math.min(1.0, newValue));";
 html += " } else if (id === 'reactionSpeed') {";
 html += "  newValue = Math.max(10, Math.min(200, newValue));";
 html += " }";
 html += " element.value = newValue.toFixed(2);";
 html += " var message = id + ':' + newValue;";
 html += " ws.send(message);";
 html += " document.getElementById('decrement' + id.charAt(0).toUpperCase() + id.slice(1)).disabled = true;";
 html += " document.getElementById('increment' + id.charAt(0).toUpperCase() + id.slice(1)).disabled = true;";
 html += "}";
 html += "</script>";
 html += "</body></html>";
 server.send(200, "text/html", html);
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
 if (type == WStype_TEXT) {
  String message = String((char *)payload);
  Serial.print("Received message: ");
  Serial.println(message);

  // Split the message into key-value pairs
  int start = 0;
  int end = message.indexOf(',');

  while (end != -1) {
   String pair = message.substring(start, end);
   int delimiter = pair.indexOf(':');
   String key = pair.substring(0, delimiter);
   String value = pair.substring(delimiter + 1);

   Serial.print("Received key: ");
   Serial.print(key);
   Serial.print(", value: ");
   Serial.println(value);

   if (key == "midPoint") {
    midPoint = value.toInt();
    preferences.putInt("midPoint", midPoint);
    Serial.println("midPoint updated");
   } else if (key == "multiplier") {
    multiplier = value.toFloat();
    preferences.putFloat("multiplier", multiplier);
    Serial.println("multiplier updated");
   } else if (key == "balance") {
    balance = value.toFloat();
    preferences.putFloat("balance", balance);
    Serial.println("balance updated");
   } else if (key == "rangeMin") {
    rangeMin = value.toInt();
    preferences.putInt("rangeMin", rangeMin);
    Serial.println("rangeMin updated");
   } else if (key == "rangeMax") {
    rangeMax = value.toInt();
    preferences.putInt("rangeMax", rangeMax);
    Serial.println("rangeMax updated");
   } else if (key == "reactionSpeed") {
    reactionSpeed = value.toInt();
    preferences.putInt("reactionSpeed", reactionSpeed);
    Serial.println("reactionSpeed updated");
   }

   start = end + 1;
   end = message.indexOf(',', start);
  }

  // Handle the last key-value pair (or the only one if there is no comma)
  String pair = message.substring(start);
  int delimiter = pair.indexOf(':');
  String key = pair.substring(0, delimiter);
  String value = pair.substring(delimiter + 1);

  Serial.print("Received key: ");
  Serial.print(key);
  Serial.print(", value: ");
  Serial.println(value);

  if (key == "midPoint") {
   midPoint = value.toInt();
   preferences.putInt("midPoint", midPoint);
   Serial.println("midPoint updated");
  } else if (key == "multiplier") {
   multiplier = value.toFloat();
   preferences.putFloat("multiplier", multiplier);
   Serial.println("multiplier updated");
  } else if (key == "balance") {
   balance = value.toFloat();
   preferences.putFloat("balance", balance);
   Serial.println("balance updated");
  } else if (key == "rangeMin") {
   rangeMin = value.toInt();
   preferences.putInt("rangeMin", rangeMin);
   Serial.println("rangeMin updated");
  } else if (key == "rangeMax") {
   rangeMax = value.toInt();
   preferences.putInt("rangeMax", rangeMax);
   Serial.println("rangeMax updated");
  } else if (key == "reactionSpeed") {
   reactionSpeed = value.toInt();
   preferences.putInt("reactionSpeed", reactionSpeed);
   Serial.println("reactionSpeed updated");
  }

  // Broadcast the updated values to all connected clients
  webSocket.broadcastTXT(message);
 }
}
