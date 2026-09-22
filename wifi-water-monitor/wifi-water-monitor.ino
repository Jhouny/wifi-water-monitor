#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <time.h>
#include <math.h>

#include "config.h"

#ifndef ULTRASONIC_TRIG_PIN
#define ULTRASONIC_TRIG_PIN 4
#endif
#ifndef ULTRASONIC_ECHO_PIN
#define ULTRASONIC_ECHO_PIN 5
#endif

constexpr uint32_t STORAGE_MAGIC = 0x57415452;
constexpr uint16_t STORAGE_VERSION = 1;
constexpr uint32_t MAX_RECORDS = 43200;
constexpr uint32_t RECORDS_PER_HOUR = 60;
constexpr uint32_t RECORD_SIZE = 12;
constexpr size_t DATA_BYTES = static_cast<size_t>(MAX_RECORDS) * RECORD_SIZE;
constexpr uint32_t MIN_VALID_UNIX_TIME = 1700000000UL;
constexpr unsigned long SENSOR_INTERVAL_MS = 5000;
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
constexpr unsigned long NTP_RETRY_INTERVAL_MS = 3600000;

struct __attribute__((packed)) WaterRecord {
  uint64_t timestamp;
  float level;
};
static_assert(sizeof(WaterRecord) == 12, "WaterRecord must remain exactly 12 bytes");

struct __attribute__((packed)) StorageMetadata {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t oldest;
  uint32_t nextWrite;
  uint32_t count;
  uint32_t generation;
  uint32_t recordsChecksum;
  uint32_t crc;
};
static_assert(sizeof(StorageMetadata) == 32, "Unexpected metadata layout");

WebServer server(80);
File dataFile;
StorageMetadata metadata = {};
WaterRecord pending[RECORDS_PER_HOUR];
uint32_t pendingCount = 0;
bool filesystemReady = false;
bool storageHealthy = false;
bool clockReady = false;
bool sensorReady = false;
float currentDistance = NAN;
float currentLevel = NAN;
float minuteLevelSum = 0;
uint16_t minuteLevelCount = 0;
uint64_t activeMinute = 0;
unsigned long lastSensorRead = 0;
unsigned long lastWiFiRetry = 0;
unsigned long lastNtpAttempt = 0;
unsigned long lastSerialDiagnostic = 0;

uint32_t crc32(const uint8_t *bytes, size_t length, uint32_t crc = 0xFFFFFFFF) {
  while (length--) {
    crc ^= *bytes++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

uint32_t metadataCrc(const StorageMetadata &value) {
  return crc32(reinterpret_cast<const uint8_t *>(&value), sizeof(StorageMetadata) - sizeof(value.crc));
}

bool validMetadata(const StorageMetadata &value) {
  return value.magic == STORAGE_MAGIC && value.version == STORAGE_VERSION &&
         value.oldest < MAX_RECORDS && value.nextWrite < MAX_RECORDS &&
         value.count <= MAX_RECORDS && value.crc == metadataCrc(value);
}

String metadataPath(uint8_t slot) {
  return slot == 0 ? "/history.meta.0" : "/history.meta.1";
}

bool readMetadata(uint8_t slot, StorageMetadata &value) {
  File file = LittleFS.open(metadataPath(slot), "r");
  if (!file || file.read(reinterpret_cast<uint8_t *>(&value), sizeof(value)) != sizeof(value)) return false;
  file.close();
  return validMetadata(value);
}

bool writeMetadata() {
  StorageMetadata next = metadata;
  next.magic = STORAGE_MAGIC;
  next.version = STORAGE_VERSION;
  next.crc = metadataCrc(next);
  StorageMetadata first;
  StorageMetadata second;
  bool firstValid = readMetadata(0, first);
  bool secondValid = readMetadata(1, second);
  uint8_t activeSlot = firstValid && (!secondValid || first.generation >= second.generation) ? 0 : 1;
  File file = LittleFS.open(metadataPath(activeSlot ^ 1), "w");
  if (!file || file.write(reinterpret_cast<const uint8_t *>(&next), sizeof(next)) != sizeof(next)) {
    if (file) file.close();
    return false;
  }
  file.flush();
  file.close();
  metadata = next;
  return true;
}

bool readRecord(uint32_t index, WaterRecord &record) {
  if (!dataFile || index >= MAX_RECORDS || !dataFile.seek(static_cast<size_t>(index) * RECORD_SIZE, SeekSet)) return false;
  return dataFile.read(reinterpret_cast<uint8_t *>(&record), sizeof(record)) == sizeof(record);
}

bool validRecord(const WaterRecord &record) {
  return record.timestamp >= MIN_VALID_UNIX_TIME && isfinite(record.level) && record.level >= 0 && record.level <= 100;
}

bool initializeStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS initialization failed");
    return false;
  }
  size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  if (freeBytes < DATA_BYTES + 8192) {
    Serial.printf("LittleFS does not have enough space: %u bytes free, %u required\n", static_cast<unsigned>(freeBytes), static_cast<unsigned>(DATA_BYTES + 8192));
    return false;
  }
  dataFile = LittleFS.open("/history.bin", "r+");
  if (!dataFile) {
    dataFile = LittleFS.open("/history.bin", "w+");
    if (!dataFile) return false;
    if (!dataFile.seek(DATA_BYTES - 1, SeekSet) || dataFile.write(static_cast<uint8_t>(0)) != 1) {
      dataFile.close();
      return false;
    }
    dataFile.flush();
  }
  StorageMetadata first;
  StorageMetadata second;
  bool firstValid = readMetadata(0, first);
  bool secondValid = readMetadata(1, second);
  if (!firstValid && !secondValid) {
    memset(&metadata, 0, sizeof(metadata));
    metadata.generation = 1;
    if (!writeMetadata()) return false;
    Serial.println("No valid metadata; starting a new history");
  } else {
    metadata = firstValid && (!secondValid || first.generation >= second.generation) ? first : second;
    if (metadata.count > 0 && recordsChecksum(metadata.oldest, metadata.count) != metadata.recordsChecksum) {
      Serial.println("History checksum mismatch; discarding corrupted history");
      metadata.oldest = 0;
      metadata.nextWrite = 0;
      metadata.count = 0;
      metadata.recordsChecksum = 0;
      metadata.generation++;
      if (!writeMetadata()) return false;
    }
  }
  return true;
}

float levelFromDistance(float distance) {
  if (!isfinite(distance) || distance < WATER_MIN_DISTANCE || distance > WATER_MAX_DISTANCE || WATER_MAX_DISTANCE <= WATER_MIN_DISTANCE) return NAN;
  return constrain((WATER_MAX_DISTANCE - distance) * 100.0f / (WATER_MAX_DISTANCE - WATER_MIN_DISTANCE), 0.0f, 100.0f);
}

float readDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
  return duration == 0 ? NAN : duration * 0.0343f / 2.0f;
}

bool unixTimeIsValid() {
  return static_cast<uint64_t>(time(nullptr)) >= MIN_VALID_UNIX_TIME;
}

uint32_t recordsChecksum(uint32_t start, uint32_t count) {
  uint32_t result = 0;
  WaterRecord record;
  for (uint32_t index = 0; index < count; ++index) {
    if (!readRecord((start + index) % MAX_RECORDS, record)) continue;
    result = crc32(reinterpret_cast<const uint8_t *>(&record), sizeof(record), result ^ 0xFFFFFFFF);
  }
  return result;
}

bool flushPending() {
  if (!filesystemReady || pendingCount == 0) return true;
  StorageMetadata before = metadata;
  for (uint32_t index = 0; index < pendingCount; ++index) {
    uint32_t slot = metadata.nextWrite;
    if (!dataFile.seek(static_cast<size_t>(slot) * RECORD_SIZE, SeekSet) || dataFile.write(reinterpret_cast<const uint8_t *>(&pending[index]), sizeof(WaterRecord)) != sizeof(WaterRecord)) {
      metadata = before;
      Serial.println("History batch write failed; keeping metadata unchanged");
      return false;
    }
    metadata.nextWrite = (metadata.nextWrite + 1) % MAX_RECORDS;
    if (metadata.count < MAX_RECORDS) metadata.count++;
    metadata.oldest = metadata.count == MAX_RECORDS ? metadata.nextWrite : 0;
  }
  dataFile.flush();
  metadata.generation++;
  metadata.recordsChecksum = recordsChecksum(metadata.oldest, metadata.count);
  if (!writeMetadata()) {
    Serial.println("History metadata commit failed; reboot will recover the previous committed batch");
    metadata = before;
    return false;
  }
  pendingCount = 0;
  Serial.printf("Committed %u history records\n", static_cast<unsigned>(metadata.count));
  return true;
}

void finishMinute(uint64_t minute) {
  if (minuteLevelCount == 0) return;
  if (pendingCount >= RECORDS_PER_HOUR) flushPending();
  if (pendingCount < RECORDS_PER_HOUR) {
    pending[pendingCount++] = {minute * 60, minuteLevelSum / minuteLevelCount};
    Serial.printf("Minute record buffered: timestamp=%llu level=%.2f (%u pending)\n",
                  static_cast<unsigned long long>(pending[pendingCount - 1].timestamp),
                  pending[pendingCount - 1].level, static_cast<unsigned>(pendingCount));
  }
  minuteLevelSum = 0;
  minuteLevelCount = 0;
}

void sampleSensor() {
  float distance = readDistance();
  float level = levelFromDistance(distance);
  if (!isfinite(level)) {
    sensorReady = false;
    return;
  }
  sensorReady = true;
  currentDistance = distance;
  currentLevel = level;
  if (!unixTimeIsValid()) return;
  clockReady = true;
  uint64_t minute = static_cast<uint64_t>(time(nullptr)) / 60;
  if (activeMinute == 0) activeMinute = minute;
  if (minute != activeMinute) {
    finishMinute(activeMinute);
    if (minute / 60 != activeMinute / 60) flushPending();
    activeMinute = minute;
  }
  minuteLevelSum += level;
  minuteLevelCount++;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED || millis() - lastWiFiRetry < WIFI_RETRY_INTERVAL_MS) return;
  lastWiFiRetry = millis();
  WiFi.begin(WIFI_1_SSID, WIFI_1_PASSW);
  Serial.printf("Connecting to Wi-Fi SSID '%s'...\n", WIFI_1_SSID);
}

void configureNetwork() {
#if USE_STATIC_IP
  IPAddress localIp;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress primaryDns;
  IPAddress secondaryDns;
  bool valid = localIp.fromString(STATIC_IP_ADDRESS) && gateway.fromString(STATIC_GATEWAY_ADDRESS) &&
               subnet.fromString(STATIC_SUBNET_ADDRESS) && primaryDns.fromString(STATIC_PRIMARY_DNS) &&
               secondaryDns.fromString(STATIC_SECONDARY_DNS);
  if (!valid || !WiFi.config(localIp, gateway, subnet, primaryDns, secondaryDns)) {
    Serial.println("Static network configuration failed");
  } else {
    Serial.print("Static IP configured: ");
    Serial.println(localIp);
  }
#else
  Serial.println("Using DHCP for Wi-Fi address");
#endif
}

void logWiFiState() {
  Serial.printf("Wi-Fi status=%d, RSSI=%d dBm\n", static_cast<int>(WiFi.status()),
                WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Wi-Fi gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("HTTP URL: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
  }
}

void maintainClock() {
  if (unixTimeIsValid()) {
    clockReady = true;
    return;
  }
  if (millis() - lastNtpAttempt < NTP_RETRY_INTERVAL_MS) return;
  lastNtpAttempt = millis();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

String jsonNumber(float value, uint8_t decimals = 2) {
  return isfinite(value) ? String(value, static_cast<unsigned int>(decimals)) : "null";
}

void handleCurrent() {
  String json = "{\"distance\":" + jsonNumber(currentDistance) + ",\"level\":" + jsonNumber(currentLevel) +
                ",\"connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
                ",\"clockValid\":" + String(clockReady ? "true" : "false") + ",\"sensorValid\":" + String(sensorReady ? "true" : "false") +
                ",\"timestamp\":" + String(static_cast<unsigned long>(time(nullptr))) + "}";
  server.send(200, "application/json", json);
}

void handleHistory() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "{\"points\":[");
  uint64_t cutoff = unixTimeIsValid() ? static_cast<uint64_t>(time(nullptr)) - 86400 : 0;
  WaterRecord record;
  bool first = true;
  for (uint32_t offset = 0; offset < metadata.count; ++offset) {
    uint32_t slot = (metadata.oldest + offset) % MAX_RECORDS;
    if (!readRecord(slot, record) || !validRecord(record) || record.timestamp < cutoff) continue;
    if (!first) server.sendContent(",");
    first = false;
    server.sendContent("{\"timestamp\":" + String(static_cast<unsigned long>(record.timestamp)) + ",\"level\":" + String(record.level, 2) + "}");
    yield();
  }
  for (uint32_t index = 0; index < pendingCount; ++index) {
    const WaterRecord &pendingRecord = pending[index];
    if (pendingRecord.timestamp < cutoff || !validRecord(pendingRecord)) continue;
    if (!first) server.sendContent(",");
    first = false;
    server.sendContent("{\"timestamp\":" + String(static_cast<unsigned long>(pendingRecord.timestamp)) +
                       ",\"level\":" + String(pendingRecord.level, 2) + "}");
  }
  server.sendContent("]}");
}

void handleDownload() {
  server.sendHeader("Content-Disposition", "attachment; filename=water-history.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "timestamp,level\n");
  WaterRecord record;
  for (uint32_t offset = 0; offset < metadata.count; ++offset) {
    uint32_t slot = (metadata.oldest + offset) % MAX_RECORDS;
    if (readRecord(slot, record) && validRecord(record)) server.sendContent(String(static_cast<unsigned long>(record.timestamp)) + "," + String(record.level, 2) + "\n");
    if ((offset & 31) == 0) yield();
  }
  for (uint32_t index = 0; index < pendingCount; ++index) {
    const WaterRecord &pendingRecord = pending[index];
    if (validRecord(pendingRecord)) {
      server.sendContent(String(static_cast<unsigned long>(pendingRecord.timestamp)) + "," +
                         String(pendingRecord.level, 2) + "\n");
    }
  }
}

void handleRoot() {
  server.send(200, "text/html", R"rawliteral(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Water monitor</title><style>
:root{color-scheme:light;font-family:system-ui,sans-serif;background:#eef4f6;color:#17324d}body{max-width:900px;margin:0 auto;padding:24px}header{display:flex;justify-content:space-between;align-items:center;gap:16px}h1{font-size:clamp(1.5rem,4vw,2.5rem);margin:0}.status{font-size:.9rem}.ok{color:#16825d}.bad{color:#b03a3a}.layout{display:grid;grid-template-columns:minmax(220px,320px) 1fr;gap:28px;align-items:center;margin-top:26px}.tank{height:320px;position:relative;clip-path:polygon(25% 0,75% 0,96% 12%,96% 88%,75% 100%,25% 100%,4% 88%,4% 12%);background:#aac2c9;overflow:hidden;border:6px solid #17324d}.water{position:absolute;bottom:0;width:100%;height:0;background:linear-gradient(#38bfd2,#0876a5);transition:height 900ms cubic-bezier(.2,.8,.2,1)}.wave{position:absolute;top:-9px;left:-10%;width:120%;height:18px;border-radius:50%;background:#7de0df;opacity:.8}.tank-label{position:absolute;inset:0;display:grid;place-items:center;font-size:2.4rem;font-weight:700;color:#17324d;text-shadow:0 1px #fff}.stats{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.stat{padding:16px;background:#fff;border:1px solid #c7d7dc;border-radius:8px}.stat b{display:block;font-size:1.5rem;margin-top:5px}.graph{margin-top:26px;padding:18px;background:#fff;border:1px solid #c7d7dc;border-radius:8px}canvas{display:block;width:100%;height:220px}button{padding:10px 14px;border:0;border-radius:6px;background:#17324d;color:#fff;cursor:pointer}small{color:#58717d}.error{color:#b03a3a;min-height:1.4em}@media(max-width:650px){body{padding:16px}.layout{grid-template-columns:1fr}.tank{height:260px}}
  </style></head><body><header><h1>Water monitor</h1><span class="status" id="status">Loading...</span></header><main class="layout"><div class="tank"><div class="water" id="water"><div class="wave"></div></div><div class="tank-label" id="level">--%</div></div><section><div class="stats"><div class="stat">Distance<b id="distance">--</b><small>centimetres</small></div><div class="stat">Updated<b id="updated">--</b><small>browser local time</small></div></div><p class="error" id="error"></p><button id="download">Download CSV</button></section></main><section class="graph"><h2>Last 24 hours</h2><canvas id="chart"></canvas></section><script>
  const $=id=>document.getElementById(id),canvas=$("chart"),ctx=canvas.getContext("2d");function draw(points){const d=canvas.getBoundingClientRect(),ratio=devicePixelRatio||1;canvas.width=d.width*ratio;canvas.height=d.height*ratio;ctx.scale(ratio,ratio);const w=d.width,h=d.height,p=24;ctx.clearRect(0,0,w,h);ctx.strokeStyle="#c7d7dc";ctx.fillStyle="#58717d";ctx.font="12px system-ui";[0,25,50,75,100].forEach(v=>{let y=h-p-(h-2*p)*v/100;ctx.beginPath();ctx.moveTo(p,y);ctx.lineTo(w-p,y);ctx.stroke();ctx.fillText(v+"%",2,y+4)});if(!points.length)return;ctx.strokeStyle="#0876a5";ctx.lineWidth=3;ctx.beginPath();points.forEach((x,i)=>{let px=p+(w-2*p)*i/Math.max(1,points.length-1),py=h-p-(h-2*p)*Math.max(0,Math.min(100,x.level))/100;i?ctx.lineTo(px,py):ctx.moveTo(px,py)});ctx.stroke();ctx.fillStyle="#58717d";ctx.textAlign="center";[0,Math.floor((points.length-1)/2),points.length-1].forEach(i=>{let px=p+(w-2*p)*i/Math.max(1,points.length-1);ctx.fillText(new Date(points[i].timestamp*1000).toLocaleTimeString([], {hour:"2-digit",minute:"2-digit"}),px,h-4)});ctx.textAlign="start"}async function update(){try{let[c,h]=await Promise.all([fetch('/api/current'),fetch('/api/history')]);if(!c.ok||!h.ok)throw Error('Request failed');let current=await c.json(),history=await h.json(),level=current.level==null?0:Math.max(0,Math.min(100,current.level));$("water").style.height=level+'%';$("level").textContent=current.level==null?'--':level.toFixed(1)+'%';$("distance").textContent=current.distance==null?'--':current.distance.toFixed(1);$("updated").textContent=current.timestamp?new Date(current.timestamp*1000).toLocaleTimeString():'--';$("status").textContent=current.connected?(current.clockValid?'Connected':'Connected, waiting for clock'):'Disconnected';$("status").className=current.connected?'status ok':'status bad';$("error").textContent=current.sensorValid?'':'No valid sensor reading';draw(history.points)}catch(e){$("error").textContent='Unable to load data';$("status").textContent='Connection error';$("status").className='status bad'}}$("download").onclick=()=>location.href='/download';update();setInterval(update,5000);addEventListener('resize',update);
  </script></body></html>)rawliteral");
  }

  void setup() {
      Serial.begin(115200);
      unsigned long serialWaitStart = millis();
      while (!Serial && millis() - serialWaitStart < 10000) {
        delay(10);
      }
      Serial.println("ESP32-C3 water monitor booting");
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    configureNetwork();
    connectWiFi();
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    filesystemReady = initializeStorage();
    storageHealthy = filesystemReady;
    server.on("/", handleRoot);
    server.on("/api/current", handleCurrent);
    server.on("/api/history", handleHistory);
    server.on("/download", handleDownload);
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
    server.begin();
    Serial.println("HTTP server started");
    logWiFiState();
  }

  void loop() {
    connectWiFi();
    maintainClock();
    server.handleClient();
    if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
      lastSensorRead = millis();
      sampleSensor();
    }
    if (millis() - lastSerialDiagnostic >= 5000) {
      lastSerialDiagnostic = millis();
      Serial.printf("alive; Wi-Fi=%s; IP=%s; level=%.2f; distance=%.2f\n",
                    WiFi.status() == WL_CONNECTED ? "connected" : "disconnected",
                    WiFi.localIP().toString().c_str(), currentLevel, currentDistance);
    }
    yield();
  }
