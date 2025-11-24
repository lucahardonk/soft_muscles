#include "WiFiS3.h"
char ssid[] = "iot_wireless";  // your network SSID (name)
char pass[] = "Wireless_iot_234123";  // your network password
#define SENSORPIN 7
int count = 0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
const unsigned long debounceDelay = 250;  // the debounce time in milliseconds
int lastSensorValue = LOW;  // Stores the last sensor value to detect changes

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(SENSORPIN, INPUT);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);  // Halts further execution if the WiFi module is not found
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  server.begin();
  printWifiStatus();
}

void loop() {
  WiFiClient client = server.available();  // Listen for incoming clients
  if (client) {
    Serial.println("New Client");
    processClient(client);
    client.stop();
    Serial.println("Client disconnected");
  }

  int sensorValue = digitalRead(SENSORPIN);

  // Edge detection to increment the count
  if (sensorValue != lastSensorValue) {
    if (millis() - lastDebounceTime > debounceDelay) {
      // reset the debouncing timer
      lastDebounceTime = millis();

      lastSensorValue = sensorValue;
      if (sensorValue == LOW) {
        count++;
        Serial.println(count);
        delay(100);
      }
    }
  }

  
}

void processClient(WiFiClient &client) {
  boolean currentLineIsBlank = true;
  String currentLine = "";

  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);

      if (c == '\n') {
        if (currentLineIsBlank) {
          sendResponse(client, currentLine);
          break;
        } else {
          currentLineIsBlank = true;
        }
      } else if (c != '\r') {
        currentLineIsBlank = false;
        currentLine += c;
      }
    }
  }
}

void sendResponse(WiFiClient &client, String &currentLine) {
  if (currentLine.indexOf("GET /count") >= 0) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type: application/json");
    client.println();
    client.println("{\"count\":" + String(count) + "}");
  } else {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type: text/html");
    client.println();
    client.println("<!DOCTYPE html><html><head><title>Repetition Counts</title>");
    client.println("<script>setInterval(function() { fetch('/count').then(response => response.json()).then(data => document.getElementById('count').innerText = data.count); }, 1000);</script>");
    client.println("<style>body { font-family: Arial, sans-serif; align-items: center; justify-content: center; height: 100vh; margin: 0; background-color: #f0f0f0; } h1 { color: #333; } p { font-size: 2em; color: #666; }</style>");
    client.println("</head><body><h1>Repetition Counts</h1><p>Current value: <span id='count'>" + String(count) + "</span></p></body></html>");
  }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
