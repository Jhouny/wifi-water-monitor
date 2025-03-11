#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// Loads credentials and settings from config.h
#include "config.h"

// Replace with your endpoint
String endpoint = "http://" + String(SERVER_IP_ADDRESS) + "/update";

int RELAY_PIN = 2;

WiFiClient client;
HTTPClient http;

unsigned long currentTime = 0;

unsigned long lastRequest = 0;
const unsigned long requestInterval = 5000; // Request every 5 seconds

unsigned long lastSerialOutput = 0;
const unsigned long SerialOutputDelay = 1000;  // Print every 1 seconds

void setup() {
	// Start serial communication for debugging
	Serial.begin(115200);
	delay(10);

	// Set pin mode for relay
	pinMode(RELAY_PIN, OUTPUT);
	digitalWrite(RELAY_PIN, LOW);

	// Connect to Wi-Fi
	Serial.println("Connecting to Wi-Fi...");
	// WiFi.setOutputPower(10);		// Set Wi-Fi power to 10 dBm
	WiFi.begin(WIFI_1_SSID, WIFI_1_PASSW);

  	while (WiFi.status() != WL_CONNECTED) {
		delay(50);
		Serial.print(".");
		yield();
	}
  
	Serial.println("\nWi-Fi connected!");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
}

void loop() {
	currentTime = millis();

	// Log connection strength
	if (currentTime - lastSerialOutput > SerialOutputDelay) {
		lastSerialOutput = currentTime;
		Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
	}

	if ((currentTime - lastRequest) > requestInterval) { 
		lastRequest = currentTime;
		if (WiFi.status() == WL_CONNECTED) {
			Serial.println("Sending request...");
			http.begin(client, endpoint);
			int httpCode = http.GET(); // Send GET request

			if (httpCode > 0) {  // If request was successful
				String payload = http.getString();
				
				// Parse JSON
				StaticJsonDocument<256> doc;
				DeserializationError error = deserializeJson(doc, payload);

				if (!error) {
					float volume = doc["volume"];   
					float distance = doc["distance"];

					Serial.print("Volume: ");
					Serial.println(volume);
					Serial.print("Distance: ");
					Serial.println(distance);

					if (volume < 50)
						digitalWrite(RELAY_PIN, HIGH);
					else if (volume > 95)
						digitalWrite(RELAY_PIN, LOW);
					
					Serial.printf("Set relay to: %d\n", digitalRead(RELAY_PIN));

				} else { Serial.println("Failed to parse JSON!"); }
			} else { Serial.println("HTTP request failed!"); }
			
			http.end(); // Close connection
		} else { Serial.println("Wi-Fi Disconnected!"); }
	}

	delay(200);
	yield();
}