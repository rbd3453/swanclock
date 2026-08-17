#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>

// --- FIRMWARE VERSION & OTA CONFIGURATION ---
const String CURRENT_VERSION = "1.0";
const String VERSION_URL = "https://raw.githubusercontent.com/rbd3453/swanclock/main/ota/version.txt";
const String FIRMWARE_URL = "https://raw.githubusercontent.com/rbd3453/swanclock/main/ota/firmware.bin";

// --- HARDWARE PINS (Digit 1 Local Motor & Sensor) ---
const int motorPin1 = 13;
const int motorPin2 = 12;
const int motorPin3 = 14;
const int motorPin4 = 27;
const int hallSensorPin = 26; 

// --- MOTOR CONFIGURATION ---
AccelStepper stepper(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);

const int STEPS_PER_REV = 4096; // Standard 1/64 reduction
const int TOTAL_FLAPS = 45; 
const float STEPS_PER_FLAP = (float)STEPS_PER_REV / (float)TOTAL_FLAPS; // 91.02222

bool isHome = false;
int currentFlapPosition = 0;
long absoluteFlapCount = 0; // Tracks total lifetime flaps to eliminate math drift

// --- TIMER VARIABLES ---
const int START_MINUTES = 108;
int currentMinutes = START_MINUTES;
int currentSeconds = 0;
bool isPaused = false;
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

WebServer server(80);

// --- HTML & JAVASCRIPT DEBUG DASHBOARD ---
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { background-color: #0b0b0b; color: #00ff66; font-family: monospace; text-align: center; padding-top: 3vh; }
    h1 { font-size: 22px; letter-spacing: 2px; margin-bottom: 5px; }
    .card { background: #141414; border: 1px solid #00ff66; border-radius: 8px; padding: 15px; margin: 12px auto; width: 85%; max-width: 400px; box-shadow: 0 0 10px rgba(0,255,102,0.1); }
    .status-val { font-size: 24px; font-weight: bold; color: #ffffff; letter-spacing: 1px; margin-top: 5px; }
    button { background-color: #222; color: #00ff66; border: 2px solid #00ff66; padding: 12px 20px; font-size: 16px; font-family: monospace; cursor: pointer; border-radius: 6px; margin: 5px; width: 100%; font-weight: bold; }
    button:active { background-color: #00ff66; color: #000; }
    .btn-exec { background-color: #003311; font-size: 20px; padding: 15px; }
    .btn-pause { background-color: #332200; border-color: #ffaa00; color: #ffaa00; }
    .btn-ota { background-color: #001a33; border-color: #00ccff; color: #00ccff; }
    input[type="number"] { background: #000; border: 1px solid #00ff66; color: #00ff66; padding: 10px; font-size: 18px; font-family: monospace; text-align: center; width: 60%; border-radius: 4px; margin-bottom: 10px; }
    input[type="range"] { width: 90%; margin: 15px 0; accent-color: #00ff66; }
    label { display: block; font-size: 12px; margin-bottom: 5px; color: #88ffbb; letter-spacing: 1px; }
    .input-row { display: flex; justify-content: space-between; gap: 10px; margin-bottom: 15px; }
    .input-group { flex: 1; }
    .input-group input { width: 80%; }
    .ota-status { font-size: 12px; margin-top: 8px; color: #88ffbb; min-height: 16px; }
  </style>
</head>
<body>
  <h1>SWAN STATION TERMINAL</h1>
  <p style="color: #666; font-size: 11px;">LIVE SYSTEM DIAGNOSTICS</p>
  
  <div class="card">
    <label>COUNTDOWN TIMER</label>
    <div id="timerDisplay" class="status-val">--:--</div>
  </div>

  <div class="card">
    <label>DIGIT 1 CURRENT FLAP</label>
    <div id="flapDisplay" class="status-val">FLAP --</div>
  </div>

  <div class="card">
    <button class="btn-exec" onclick="sendCmd('/execute')">EXECUTE (RESET 108)</button>
  </div>

  <div class="card">
    <button class="btn-pause" onclick="sendCmd('/togglePause')">PAUSE / RESUME</button>
  </div>

  <div class="card">
    <label>MOTOR SPEED CONTROL</label>
    <span style="font-size: 12px; color:#fff;" id="speedLabel">800 Steps/Sec</span>
    <input type="range" id="speedSlider" min="300" max="1000" step="50" value="800" onchange="submitSpeed()">
  </div>

  <div class="card">
    <label>DEBUG OVERRIDE: TARGET & ROTATIONS</label>
    <div class="input-row">
      <div class="input-group">
        <label>TARGET FLAP (0-44)</label>
        <input type="number" id="flapInput" min="0" max="44" value="1">
      </div>
      <div class="input-group">
        <label>EXTRA ROTATIONS</label>
        <input type="number" id="rotationsInput" min="0" max="10" value="0">
      </div>
    </div>
    <button onclick="submitFlap()">SET POSITION (AUTO-PAUSES TIMER)</button>
  </div>

  <div class="card">
    <label>FIRMWARE MANAGEMENT</label>
    <button class="btn-ota" onclick="checkUpdate()">CHECK FOR FIRMWARE UPDATES</button>
    <div id="otaStatus" class="ota-status"></div>
  </div>

  <script>
    function sendCmd(url) {
      fetch(url).catch(err => console.log(err));
    }
    
    function submitFlap() {
      let val = document.getElementById('flapInput').value;
      let rot = document.getElementById('rotationsInput').value;
      fetch('/setFlap?val=' + val + '&rot=' + rot).catch(err => console.log(err));
    }

    function submitSpeed() {
      let val = document.getElementById('speedSlider').value;
      document.getElementById('speedLabel').innerText = val + " Steps/Sec";
      fetch('/setSpeed?val=' + val).catch(err => console.log(err));
    }

    function checkUpdate() {
      let statusDiv = document.getElementById('otaStatus');
      statusDiv.innerText = "CHECKING FOR UPDATES...";
      fetch('/check-update')
        .then(res => res.text())
        .then(msg => {
          statusDiv.innerText = msg;
        })
        .catch(err => {
          statusDiv.innerText = "UPDATE CHECK FAILED";
          console.log(err);
        });
    }

    setInterval(() => {
      fetch('/status')
        .then(res => res.json())
        .then(data => {
          document.getElementById('timerDisplay').innerText = data.timer;
          document.getElementById('flapDisplay').innerText = "FLAP " + data.flap;
        })
        .catch(err => console.log(err));
    }, 500);
  </script>
</body>
</html>
)rawliteral";

// --- DRIFT-FREE FLAP MATH WITH EXTRA ROTATIONS ---
void goToFlap(int targetFlap, int extraRotations = 0) {
  if (targetFlap < 0 || targetFlap >= TOTAL_FLAPS) return;
  
  // If we are already there AND no extra rotations are requested, do nothing
  if (targetFlap == currentFlapPosition && extraRotations == 0) return; 
  
  int flapsToAdvance = targetFlap - currentFlapPosition;
  if (flapsToAdvance < 0) {
    flapsToAdvance += TOTAL_FLAPS; 
  }
  
  // Add the extra full rotations (45 flaps per rotation)
  int totalFlapsToMove = flapsToAdvance + (extraRotations * TOTAL_FLAPS);
  
  absoluteFlapCount += totalFlapsToMove;
  long targetSteps = -1 * (long)(absoluteFlapCount * STEPS_PER_FLAP);
  stepper.moveTo(targetSteps);
  
  // Update internal tracker to the new resting spot
  currentFlapPosition = targetFlap;
}

void findHome() {
  Serial.println("[SYSTEM] SEEKING HOME POSITION...");
  stepper.setSpeed(-400); 

  while (digitalRead(hallSensorPin) == HIGH) {
    stepper.runSpeed();
  }

  stepper.setCurrentPosition(0);
  stepper.setSpeed(-400);
  while (digitalRead(hallSensorPin) == LOW) {
    stepper.runSpeed();
  }

  absoluteFlapCount = 8; 
  long homeSteps = -1 * (long)(absoluteFlapCount * STEPS_PER_FLAP);
  stepper.setCurrentPosition(homeSteps);
  currentFlapPosition = 8; 
  
  Serial.println("\n[SUCCESS] HOME LOCKED. INDEXED TO FLAP 8.");
}

// --- VERSION COMPARISON HELPER ---
bool isNewerVersion(const String& serverVer, const String& currentVer) {
  float serverFloat = serverVer.toFloat();
  float currentFloat = currentVer.toFloat();
  if (serverFloat > 0 && currentFloat > 0) {
    return serverFloat > currentFloat;
  }
  return (serverVer != currentVer) && (serverVer.length() > 0);
}

// --- REMOTE HTTPS OTA UPDATES ---
bool checkForOTAUpdates() {
  Serial.println("\n[OTA] Checking for firmware updates...");
  Serial.printf("[OTA] Current Firmware Version: %s\n", CURRENT_VERSION.c_str());

  WiFiClientSecure client;
  client.setInsecure(); // Critical: GitHub rotates SSL certificates

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // Critical: Follow raw GitHub CDN/redirects

  if (!http.begin(client, VERSION_URL)) {
    Serial.println("[OTA] Error: Unable to connect to version URL.");
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] Version check failed, HTTP response code: %d\n", httpCode);
    http.end();
    return false;
  }

  String serverVersion = http.getString();
  serverVersion.trim();
  http.end();

  Serial.printf("[OTA] Server Version: %s\n", serverVersion.c_str());

  if (!isNewerVersion(serverVersion, CURRENT_VERSION)) {
    Serial.println("[OTA] Device is already running the latest firmware.");
    return false;
  }

  Serial.printf("[OTA] New version available (%s > %s). Initiating download...\n", serverVersion.c_str(), CURRENT_VERSION.c_str());

  if (!http.begin(client, FIRMWARE_URL)) {
    Serial.println("[OTA] Error: Unable to connect to firmware binary URL.");
    return false;
  }
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int fwHttpCode = http.GET();
  if (fwHttpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] Firmware download request failed, HTTP response code: %d\n", fwHttpCode);
    http.end();
    return false;
  }

  int contentLength = http.getSize();
  Serial.printf("[OTA] Binary size: %d bytes\n", contentLength);

  if (contentLength <= 0) {
    Serial.println("[OTA] Error: Invalid firmware content length.");
    http.end();
    return false;
  }

  if (!Update.begin(contentLength)) {
    Serial.println("[OTA] Error: Insufficient space on partition for OTA update.");
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t written = 0;
  uint8_t buff[1024] = { 0 };
  int lastProgress = -1;

  Serial.println("[OTA] Writing firmware to flash partition...");

  while (http.connected() && (written < (size_t)contentLength)) {
    size_t availableBytes = stream->available();
    if (availableBytes > 0) {
      size_t bytesToRead = (availableBytes < sizeof(buff)) ? availableBytes : sizeof(buff);
      if ((written + bytesToRead) > (size_t)contentLength) {
        bytesToRead = (size_t)contentLength - written;
      }
      int bytesRead = stream->readBytes(buff, bytesToRead);
      if (bytesRead > 0) {
        Update.write(buff, bytesRead);
        written += bytesRead;

        int progress = (int)((written * 100) / contentLength);
        if (progress != lastProgress && (progress % 10 == 0 || progress == 100)) {
          Serial.printf("[OTA] Progress: %d%%\n", progress);
          lastProgress = progress;
        }
      }
    }
    yield();
  }

  http.end();

  if (written == (size_t)contentLength) {
    if (Update.end(true)) {
      Serial.println("[OTA] Update successfully written to flash partition!");
      Serial.println("[OTA] Rebooting ESP32 into new firmware...");
      delay(500);
      ESP.restart();
      return true;
    } else {
      Serial.printf("[OTA] Update error during finalization. Error #: %d\n", Update.getError());
      return false;
    }
  } else {
    Serial.printf("[OTA] Incomplete payload received (%u of %d bytes). Aborting.\n", written, contentLength);
    Update.abort();
    return false;
  }
}

// --- WEB SERVER ENDPOINTS ---
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleStatus() {
  char timerBuf[16];
  if (isPaused) {
    snprintf(timerBuf, sizeof(timerBuf), "PAUSED %03d:%02d", currentMinutes, currentSeconds);
  } else {
    snprintf(timerBuf, sizeof(timerBuf), "%03d:%02d", currentMinutes, currentSeconds);
  }

  String json = "{";
  json += "\"timer\":\"" + String(timerBuf) + "\",";
  json += "\"flap\":" + String(currentFlapPosition);
  json += "}";

  server.send(200, "application/json", json);
}

void handleExecute() {
  currentMinutes = START_MINUTES;
  currentSeconds = 0;
  isPaused = false;
  Serial.println("\n[SYSTEM] > 4 8 15 16 23 42");
  Serial.println("[SYSTEM] > OVERRIDE ACCEPTED. RESETTING TO 108:00\n");
  
  // Have it do 2 dramatic extra full spins when reset, just like the show!
  goToFlap(1, 2); 
  server.send(200, "text/plain", "OK");
}

void handleTogglePause() {
  isPaused = !isPaused;
  Serial.println(isPaused ? "[SYSTEM] TIMER PAUSED." : "[SYSTEM] TIMER RESUMED.");
  server.send(200, "text/plain", "OK");
}

void handleSetFlap() {
  if (server.hasArg("val")) {
    int target = server.arg("val").toInt();
    int rot = 0;
    
    // Check if the secondary extra rotations parameter was passed
    if (server.hasArg("rot")) {
      rot = server.arg("rot").toInt();
    }
    
    isPaused = true; 
    Serial.printf("[DEBUG] Manual override: Moving to Flap %d with %d Extra Rotations\n", target, rot);
    
    goToFlap(target, rot);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing Parameter");
  }
}

void handleSetSpeed() {
  if (server.hasArg("val")) {
    int newSpeed = server.arg("val").toInt();
    if (newSpeed < 100) newSpeed = 100;
    if (newSpeed > 1200) newSpeed = 1200;
    
    stepper.setMaxSpeed(newSpeed);
    stepper.setAcceleration(newSpeed / 2); 
    
    Serial.printf("[SYSTEM] Motor speed updated to: %d steps/sec\n", newSpeed);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing Parameter");
  }
}

void handleCheckUpdate() {
  Serial.println("\n[SYSTEM] Manual OTA check triggered from web dashboard.");
  isPaused = true;

  bool updateSuccess = checkForOTAUpdates();

  if (!updateSuccess) {
    isPaused = false; // Safely resume countdown timer if no update found/applied
    server.send(200, "text/plain", "Firmware is up to date (v" + CURRENT_VERSION + ")");
  } else {
    server.send(200, "text/plain", "Update successful! Rebooting...");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SWAN STATION MASTER BRAIN INITIALIZING ===");

  pinMode(hallSensorPin, INPUT_PULLUP);
  stepper.setMaxSpeed(800);
  stepper.setAcceleration(400);

  findHome();

  // --- CAPTIVE PORTAL / WI-FI PROVISIONING ---
  WiFiManager wm;
  Serial.println("[SYSTEM] Initializing WiFiManager Captive Portal...");
  bool res = wm.autoConnect("Swan Station Setup");
  if (!res) {
    Serial.println("[SYSTEM] Failed to connect to Wi-Fi. Restarting...");
    delay(3000);
    ESP.restart();
  } else {
    Serial.println("\n[SYSTEM] WI-FI CONNECTED!");
    Serial.print("[SYSTEM] TERMINAL IP: ");
    Serial.println(WiFi.localIP());
  }

  server.on("/", handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/execute", HTTP_GET, handleExecute);
  server.on("/togglePause", HTTP_GET, handleTogglePause);
  server.on("/setFlap", HTTP_GET, handleSetFlap);
  server.on("/setSpeed", HTTP_GET, handleSetSpeed); 
  server.on("/check-update", HTTP_GET, handleCheckUpdate);
  server.begin();
  
  Wire.begin(21, 22);

  goToFlap(1);
}

void loop() {
  server.handleClient();
  stepper.run(); 

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (!isPaused) {
      if (currentMinutes >= 100) {
        goToFlap(1); 
      } else if (currentMinutes > 0) {
        goToFlap(40); 
      } else if (currentMinutes == 0 && currentSeconds == 0) {
        goToFlap(41); 
      }

      if (currentMinutes == 0 && currentSeconds == 0) {
        // Hold at zero
      } else {
        if (currentSeconds == 0) {
          currentMinutes--;
          currentSeconds = 59;
        } else {
          currentSeconds--;
        }
      }
    }
  }
}
