#include <HCSR04.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>

// Loads credentials and settings from config.h
#include "config.h"

ESP8266WiFiMulti wifiMulti;   // WiFi connection manager

ESP8266WebServer server(80);

HCSR04 hc(D5, D6);  // Initialisation of HCSR04 (trig pin, echo pin)

float volumePercentage;

long currentTime = 0;

unsigned long lastLEDBlink = 0;
int blinkDelay = 0;

const int movingAverageSize = 5;     // Size of the moving average
float distances[movingAverageSize];  // Array to store distance readings
int ind = 0;                         // Index for circular buffer
float currentDistance = WATER_MIN_DISTANCE;
float sum = 0;                       // Sum of the distances for moving average
float averageDistance = 0;           // Moving average of the distances
unsigned long lastDistanceUpdate = 0;
const unsigned long DistanceUpdateDelay = 1000;  // Update every 1 second

// Serial output messages delay
unsigned long lastSerialOutput = 0;
const unsigned long SerialOutputDelay = 3000;  // Print every 3 seconds

void handleRoot();
void handleUpdate();
void handleNotFound();

void setup() {
  Serial.begin(115200);  // Start the Serial communication to send messages to the computer
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);  // Set the LED pin as output

  // Initialize the distances array
  for (int i = 0; i < movingAverageSize; i++) {
    distances[i] = 0;
  }

  wifiMulti.addAP(WIFI_1_SSID, WIFI_1_PASSW);  // add Wi-Fi networks you want to connect to

  Serial.println("Connecting ...");
  while (wifiMulti.run() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    delay(50);
    Serial.print('.');
    yield();
  }
  
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());     // Tell us what network we're connected to
  Serial.print("IP address:\t");   //
  Serial.println(WiFi.localIP());  // Send the IP address of the ESP8266 to the computer

  server.on("/", handleRoot);          // Call the 'handleRoot' function when a client requests URI "/"
  server.on("/update", handleUpdate);  // Regularly updates the client with up-to-date data
  server.onNotFound(handleNotFound);   // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

  server.begin();  // Actually start the server
  Serial.println("HTTP server started");

  // Set the initial time
  currentTime = millis();
}

void loop() {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Web Server Section ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    WiFi.disconnect();
    WiFi.reconnect();
    while (wifiMulti.run() != WL_CONNECTED) {
      delay(50);
      Serial.print(".");
      yield();
    }
    Serial.println("\nReconnected to Wi-Fi. New IP address: " + WiFi.localIP().toString());
  }

  // Handle incoming clients
  server.handleClient();

  // Update the current time
  currentTime = millis();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Water Level Section ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  
  // Update moving average
  if (currentTime - lastDistanceUpdate > DistanceUpdateDelay) {
    lastDistanceUpdate = currentTime;
    currentDistance = hc.dist();                // Measure distance
    sum -= distances[ind];                      // Subtract the oldest value from the sum
    distances[ind] = currentDistance;           // Replace it with the new value
    sum += currentDistance;                     // Add the new value to the sum
    ind = (ind + 1) % movingAverageSize;        // Update the index circularly
    averageDistance = sum / movingAverageSize;  // Calculate moving average
    volumePercentage = map(averageDistance, WATER_MAX_DISTANCE, WATER_MIN_DISTANCE, 0, 1000) / 10.0;
    // Blink LED proportionally to the distance
    blinkDelay = map(constrain(volumePercentage, 0, 100), 0, 100, 500, 40);   // Map the percentage to a frequency between 2Hz and 25Hz
  }

  // Print the average distance
  if (currentTime - lastSerialOutput > SerialOutputDelay) {
    lastSerialOutput = currentTime;
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.print(" cm | ");
    Serial.print("Avg distance: ");
    Serial.print(averageDistance);
    Serial.print(" cm | ");
    Serial.print("Volume %: ");
    Serial.println(volumePercentage);
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LED Blink Section ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  if (currentTime - lastLEDBlink > blinkDelay/2) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Toggle the LED on/off
    lastLEDBlink = currentTime;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  yield();
}

void handleRoot() {
  String html = R"rawliteral(
        <!DOCTYPE html>
        <html>
            <head>
                <meta name='viewport' content='width=device-width, initial-scale=1'>
                <link rel='icon' href='data'>
                <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
                <style>
                    html { 
                        font-family: Helvetica; 
                        display: inline-block; 
                        margin: 0px auto; 
                        text-align: center; 
                        text-decoration: none; 
                        font-size: 30px; 
                        margin: 2px; 
                        cursor: pointer;
                    }

                    .lid {
                        width: 60px;
                        height: 6px;
                        position: relative;
                        margin: 0 auto;
                        background-color: black;
                    }

                    .walls {
                        width: 200px; /* Set the width of the container */
                        height: 300px; /* Barrel height */
                        position: relative;
                        margin: 0 auto; 
                        clip-path: polygon(25% 0%, 75% 0%, 95% 15%, 95% 85%, 75% 100%, 25% 100%, 5% 85%, 5% 15%);
                        background-color: black;
                    }

                    .container {
                        width: 196px; /* Set the width of the container */
                        height: 296px; /* Barrel height */
                        top: 2px;
                        left: 2px;
                        position: relative;
                        overflow: hidden; /* Ensures water stays inside */
                        clip-path: polygon(25% 0%, 75% 0%, 95% 15%, 95% 85%, 75% 100%, 25% 100%, 5% 85%, 5% 15%);
                        box-shadow: 0 5px 10px rgba(0, 0, 0, 0.2); /* Add a subtle shadow for depth */
                        background-color: #e6f7ff; /* Light blue background to represent the empty part */
                    }
            
                    .water {
                        position: absolute;
                        bottom: 0; /* Start filling from the bottom */
                        width: 100%; /* Fill the width of the container */
                        background-color: #0077cc; /* Darker blue to represent water */
                        transition: height 0.5s ease; /* Smooth transition when the fill percentage changes */
                    }
            
                    .percentage {
                        position: absolute;
                        width: 100%;
                        text-align: center;
                        top: 50%;
                        transform: translateY(-50%);
                        font-size: 24px;
                        font-weight: bold;
                        color: #002266; /* Same color as water for consistency */
                    }

                    .minorText {
                      font-size:12px;
                      color: #666666;
                      text-align: center;
                      font-weight: 100;
                      margin-bottom: -10px;
                    }
                </style>
            </head>
            <body>
                <h2>Monitor de Reservatorio de Agua</h1>

                <div class="lid"></div>
                <div class="walls">
                    <div class="container">
                        <div class="water" id="waterFill" style="height: 50%;"></div>
                        <div class="percentage" id="percentageText">50%</div>
                    </div>
                </div>

                <p class="minorText">Ultimo update: <span id="lastUpdate"></span></p>
                <p class="minorText">Distancia: <span id="distance"></span></p>
                
                <script>
                    function updateChart() {
                        fetch('/update')
                            .then(response => response.json())
                            .then(data => {
                                const now = new Date().toLocaleTimeString();

                                const waterFill = document.getElementById('waterFill');
                                const percentageText = document.getElementById('percentageText');
                                const lastUpdateText = document.getElementById('lastUpdate');
                                const distanceText = document.getElementById('distance');
                                
                                // Set the height of the water based on the percentage
                                waterFill.style.height = data.volume + '%';

                                // Constrain the volume range to make it not exceed the 0-100% marks
                                const waterVolume = Math.max(0, Math.min(100, data.volume));

                                // Update the text to show the percentage
                                percentageText.textContent = waterVolume + '%';
                                lastUpdateText.textContent = now;
                                distanceText.textContent = data.distance;
                            })
                            .catch(err => console.error('Error fetching data:', err));
                    }

                    updateChart();
                    setInterval(updateChart, 5000); // Update every 5 seconds
                </script>
            </body>
        </html>
    )rawliteral";

  server.send(200, "text/html", html);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void handleUpdate() {
  String json = R"rawliteral({
    "volume": )rawliteral" + String(volumePercentage) + R"rawliteral(,
    "distance": )rawliteral" + String(averageDistance) + R"rawliteral(
  })rawliteral";
  server.send(200, "application/json", json);
}