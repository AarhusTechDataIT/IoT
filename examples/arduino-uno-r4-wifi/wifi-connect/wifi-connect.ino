/*
  WiFi Connection Sketch (Corrected for UNO R4 WiFi)
  This script connects your Arduino UNO R4 WiFi to a local WiFi network.
*/

// Include the correct library for the UNO R4 WiFi module
#include <WiFiS3.h>

// --- Replace with your network credentials ---
const char* WIFI_SSID = "4G Wi-Fi 3Danmark-1CBA";
const char* WIFI_PASS = "aircraft";
// -----------------------------------------

int status = WL_IDLE_STATUS; // the WiFi radio's status

void setup() {
  // Initialize Serial for debugging output
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect

  // Check for the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Don't continue
    while (true);
  }

  // Attempt to connect to the WiFi network
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    // Wait 5 seconds for connection
    delay(5000);
    Serial.print(".");
  }

  // You're connected!
  Serial.println("\nConnected to WiFi! ✅");
  printWifiStatus();
}

void loop() {
  // The loop is empty because we only need to connect once in setup().
  delay(10000); 
}

void printWifiStatus() {
  // Print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print the IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}