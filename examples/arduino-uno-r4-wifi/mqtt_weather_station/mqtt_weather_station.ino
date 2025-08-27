#include <WiFiS3.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>

// --- START OF CONFIGURATION ---

// WiFi Settings
const char* WIFI_SSID = "SkyLab Academy";
const char* WIFI_PASS = "SkyLab_Academy";

// MQTT Broker Settings
const char* MQTT_SERVER = "10.71.106.119"; // Raspberry Pi IP Addresss
const int   MQTT_PORT = 1883;
const char* MQTT_TOPIC = "home/livingroom/bme280"; // The topic to publish to
const char* MQTT_CLIENT_ID = "foo-bar-42-definitely-globally-unique-client-ish"; // A unique client ID

// --- END OF CONFIGURATION ---

WiFiClient wifiClient;
PubSubClient client(wifiClient);
Adafruit_BME280 bme;

unsigned long lastMsg = 0;
const int MSG_INTERVAL = 10000; // Send data every 10 seconds

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  // --- Connect to WiFi First, Before Anything Else ---
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // 1. Wait for the connection to be established
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  // 2. NEW: Explicitly wait for a valid IP address
  Serial.print("Waiting for IP address...");
  while (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // --- Now Initialize Other Services ---
  Serial.println("Initializing BME280 sensor...");
  if (!bme.begin(0x76)) { // I2C address, may be 0x77
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("Sensor initialized.");

  Serial.println("Setting up MQTT server...");
  client.setServer(MQTT_SERVER, MQTT_PORT);
  Serial.println("Setup complete.");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // This keeps the MQTT client connection alive

  // Using millis() for non-blocking delay
  unsigned long now = millis();
  if (now - lastMsg > MSG_INTERVAL) {
    lastMsg = now;

    // Read sensor values
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F; // hPa

    // Create a JSON payload
    char jsonPayload[128];
    snprintf(jsonPayload, sizeof(jsonPayload),
             "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
             temperature, humidity, pressure);

    // Publish the message
    Serial.print("Publishing message: ");
    Serial.println(jsonPayload);
    client.publish(MQTT_TOPIC, jsonPayload);
  }
}