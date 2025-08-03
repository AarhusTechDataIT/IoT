/*
  UDP Send/Receive Sketch (Corrected for UNO R4 WiFi)
  This script sends a message to another Arduino on the network and listens for incoming messages.
*/

#include <WiFiS3.h>
#include <WiFiUdp.h>

// --- Replace with your network credentials ---
char ssid[] = "YOUR_NETWORK_NAME"; // your network SSID (name)
char pass[] = "YOUR_PASSWORD";   // your network password
// -----------------------------------------

unsigned int localPort = 2390;      // Local port to listen on

// --- CHANGE THIS to the IP of the Arduino you want to send to ---
IPAddress remoteIP(192, 168, 1, 101); // Example IP address
// ----------------------------------------------------------------

char packetBuffer[255];             // Buffer to hold incoming packets
char replyPacket[] = "Hello from my Arduino! 👋"; // Message to send

WiFiUDP Udp;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Connect to WiFi
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("\nConnected to WiFi");
  IPAddress ip = WiFi.localIP();
  Serial.print("My IP address: ");
  Serial.println(ip);

  // Begin listening for UDP packets
  Udp.begin(localPort);
  Serial.print("Listening on port ");
  Serial.println(localPort);
}

void loop() {
  // --- Sending a message every 5 seconds ---
  Serial.println("Sending UDP packet...");
  Udp.beginPacket(remoteIP, localPort);
  Udp.write((uint8_t*)replyPacket, strlen(replyPacket));
  Udp.endPacket();
  
  // --- Checking for an incoming message ---
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // Read the packet into the buffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.print("Contents: ");
    Serial.println(packetBuffer);
  }
  
  delay(5000); // Wait 5 seconds before repeating
}