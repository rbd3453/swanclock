#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <DFRobotDFPlayerMini.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>

// --- FIRMWARE VERSION & OTA CONFIGURATION ---
const String CURRENT_VERSION = "1.4.37";
const String VERSION_URL = "https://raw.githubusercontent.com/rbd3453/swanclock/main/ota/version.txt";
const String FIRMWARE_URL = "https://raw.githubusercontent.com/rbd3453/swanclock/main/ota/firmware.bin";

// --- HARDWARE PINS (Right-most Local Motor & Sensor) ---
const int motorPin1 = 13;
const int motorPin2 = 12;
const int motorPin3 = 14;
const int motorPin4 = 27;
const int hallSensorPin = 26; 

// --- VOLUME POTENTIOMETER ---
// Pin 1 ➔ 3.3V | Pin 3 ➔ GND | Pin 2 (Signal Wiper) ➔ GPIO 34
const int potPin = 34;
int lastPotVolume = -1;
unsigned long lastPotReadMillis = 0;
const unsigned long potReadInterval = 50; // Check ADC every 50ms for responsive dial tracking

// --- AUDIO (DFPlayer Mini on Serial2) ---
// ESP32 GPIO 16 (RX2) connects to DFPlayer TX
// ESP32 GPIO 17 (TX2) connects to DFPlayer RX
HardwareSerial dfSerial(2);
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;
int currentVolume = 20;

// --- MOTOR CONFIGURATION (Master Drum - Right-most) ---
AccelStepper stepper(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);

const int STEPS_PER_REV = 4096; // Standard 1/64 reduction
const int TOTAL_FLAPS = 45; 
const float STEPS_PER_FLAP = (float)STEPS_PER_REV / (float)TOTAL_FLAPS; // 91.02222

bool isHome = false;
int currentFlapPosition = 0;
long absoluteFlapCount = 0; // Tracks total lifetime flaps to eliminate math drift

// --- DRUM ARRAY TRACKING (5 DRUMS, LEFT TO RIGHT) ---
// Drum 0: Left-most (Slave 4 - I2C 0x04)
// Drum 1: 2nd from Left (Slave 3 - I2C 0x03)
// Drum 2: 3rd from Left (Slave 2 - I2C 0x02)
// Drum 3: 4th from Left / 2nd from Right (Slave 1 - I2C 0x01)
// Drum 4: Right-most (Master Drum - Local ESP32)
int drumFlapState[5] = {0, 0, 0, 0, 0};
int lastDisplayedDigits[5] = {-1, -1, -1, -1, -1}; // Tracks displayed digits to eliminate unnecessary I2C traffic

// --- POWER & MOVEMENT PROFILES ---
// 0 = Eco / Safe Sequential Mode (1.5A Power Supply) - Moves drums one-at-a-time, max ~300mA draw
// 1 = Fast Simultaneous Mode (3.0A+ Power Supply) - Moves drums in parallel
int powerMode = 0; // Default to Safe Sequential Mode

// --- CALIBRATION STORAGE (NVS PREFERENCES) ---
Preferences prefs;
int masterHomeOffsetSteps = (int)(8 * STEPS_PER_FLAP); // Default fallback ~728 steps

void loadMasterCalibration() {
  prefs.begin("calibration", false);
  masterHomeOffsetSteps = prefs.getInt("masterOffset", (int)(8 * STEPS_PER_FLAP));
  prefs.end();
}

void saveMasterCalibration(int newOffset) {
  prefs.begin("calibration", false);
  prefs.putInt("masterOffset", newOffset);
  prefs.end();
  masterHomeOffsetSteps = newOffset;
}

void loadPowerSettings() {
  prefs.begin("settings", false);
  powerMode = prefs.getInt("powerMode", 0);
  prefs.end();
}

void savePowerSettings(int mode) {
  prefs.begin("settings", false);
  prefs.putInt("powerMode", mode);
  prefs.end();
  powerMode = mode;
}

// --- TIMER VARIABLES & STATE MACHINE ---
enum TimerState {
  TIMER_STOPPED,
  TIMER_HOLDING, // Setting drums to target time and holding 5 seconds before counting down
  TIMER_RUNNING
};
TimerState timerState = TIMER_STOPPED;
unsigned long holdStartTime = 0;
const unsigned long HOLD_DURATION_MS = 5000; // 5-second hold at starting time

const int START_MINUTES = 108;
int currentMinutes = START_MINUTES;
int currentSeconds = 0;
bool isPaused = true; // Start paused on boot: drums stay at Home until user initiates
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

WebServer server(80);

// --- OTA ASYNC STATE & LOG BUFFER ---
String otaLogBuffer = "[SYSTEM] Ready for firmware operations.\n";
int otaProgressPercent = -1; // -1 = idle
bool otaRunning = false;
String otaState = "IDLE";
TaskHandle_t otaTaskHandle = NULL;

void logOTA(const String& msg) {
  Serial.println(msg);
  otaLogBuffer += msg + "\n";
  if (otaLogBuffer.length() > 3000) {
    otaLogBuffer = otaLogBuffer.substring(otaLogBuffer.length() - 2000);
  }
}

// --- PHYSICAL FLAP MAPPING ---
// Flap 0: Blank (Home)
// Flaps 1..5: 5 Red Hieroglyphics
// Flaps 6..15: 0-9 (Set 1)
// Flaps 16..25: 0-9 (Set 2)
// Flaps 26..35: 0-9 (Set 3)
// Flaps 36..44: 0-8 (Set 4)

int getNextFlapForDigit(int currentFlap, int digit) {
  if (digit < 0 || digit > 9) return 0;
  
  int candidates[4];
  int numCandidates = 0;
  
  if (6 + digit < 45) candidates[numCandidates++] = 6 + digit;
  if (16 + digit < 45) candidates[numCandidates++] = 16 + digit;
  if (26 + digit < 45) candidates[numCandidates++] = 26 + digit;
  if (36 + digit < 45) candidates[numCandidates++] = 36 + digit;
  
  int bestFlap = candidates[0];
  int minAdvance = 999;
  
  for (int i = 0; i < numCandidates; i++) {
    int flap = candidates[i];
    int advance = flap - currentFlap;
    if (advance < 0) advance += TOTAL_FLAPS;
    if (advance < minAdvance) {
      minAdvance = advance;
      bestFlap = flap;
    }
  }
  return bestFlap;
}

int charToFlap(char c, int currentFlap = 0) {
  if (c >= '0' && c <= '9') {
    return getNextFlapForDigit(currentFlap, c - '0');
  }
  if (c == ' ') return 0; // Blank (Home)
  if (c == '*' || c == '#') return 1; // Default to Hieroglyphic 1
  return 0; // Default to Blank
}

// --- DRIFT-FREE FLAP MATH WITH COIL POWER MANAGEMENT ---
void goToFlap(int targetFlap, int extraRotations = 0) {
  if (targetFlap < 0 || targetFlap >= TOTAL_FLAPS) return;
  if (targetFlap == currentFlapPosition && extraRotations == 0) return; 
  
  int flapsToAdvance = targetFlap - currentFlapPosition;
  if (flapsToAdvance < 0) {
    flapsToAdvance += TOTAL_FLAPS; 
  }
  
  int totalFlapsToMove = flapsToAdvance + (extraRotations * TOTAL_FLAPS);
  absoluteFlapCount += totalFlapsToMove;
  long targetSteps = -1 * (long)(absoluteFlapCount * STEPS_PER_FLAP);
  stepper.enableOutputs(); // Energize coils for motion
  stepper.moveTo(targetSteps);
  currentFlapPosition = targetFlap;
  drumFlapState[4] = targetFlap;
}

// Set a specific drum (0 to 4) to a flap position with error feedback
bool setDrumFlap(int drumIndex, int flap, int extraRot = 0) {
  if (drumIndex < 0 || drumIndex > 4) return false;
  if (flap < 0 || flap >= TOTAL_FLAPS) return false;
  if (drumFlapState[drumIndex] == flap && extraRot == 0) return true; // Already in place

  if (drumIndex == 4) {
    // Drum 4: Master (Right-most)
    goToFlap(flap, extraRot);
    drumFlapState[4] = flap;
    return true;
  } else {
    // Slaves: Drum 0 -> Addr 4, Drum 1 -> Addr 3, Drum 2 -> Addr 2, Drum 3 -> Addr 1
    uint8_t slaveAddr = (uint8_t)(4 - drumIndex);
    Wire.setTimeOut(30);
    Wire.beginTransmission(slaveAddr);
    Wire.write((uint8_t)flap);
    Wire.write((uint8_t)10); // Default RPM
    uint8_t err = Wire.endTransmission(true);
    delay(15); // Give I2C bus time to settle between transmissions
    if (err == 0) {
      drumFlapState[drumIndex] = flap;
      return true;
    } else {
      Serial.printf("[I2C] Failed sending to Drum %d (Addr 0x%02X), error: %d\n", drumIndex, slaveAddr, err);
      return false;
    }
  }
}

// Wait for a drum to complete its move and power off coils (ensures only 1 motor ever active)
void waitForDrum(int drumIndex, unsigned long timeoutMs = 1800) {
  if (drumIndex == 4) {
    unsigned long start = millis();
    while (stepper.distanceToGo() != 0 && (millis() - start < timeoutMs)) {
      stepper.run();
      yield();
    }
    stepper.disableOutputs();
  } else {
    uint8_t slaveAddr = (uint8_t)(4 - drumIndex);
    unsigned long start = millis();
    delay(50); // Brief delay for slave to initiate rotation
    while (millis() - start < timeoutMs) {
      Wire.requestFrom(slaveAddr, (uint8_t)1);
      if (Wire.available()) {
        uint8_t isBusy = Wire.read();
        if (isBusy == 0) {
          break; // Slave is stationary and coils are OFF!
        }
      } else {
        // No response from address (unattached device like 0x04) -> abort immediately, don't hang!
        break;
      }
      delay(30);
      yield();
    }
  }
}

void findHome() {
  Serial.println("[SYSTEM] SEEKING HOME POSITION...");
  stepper.enableOutputs();
  stepper.setMaxSpeed(800);
  stepper.setSpeed(-400); 

  while (digitalRead(hallSensorPin) == HIGH) {
    stepper.runSpeed();
  }

  stepper.setCurrentPosition(0);
  stepper.setSpeed(-400);
  while (digitalRead(hallSensorPin) == LOW) {
    stepper.runSpeed();
  }

  // Move forward by stored calibration offset to land precisely on Blank Flap (Flap 0)
  stepper.moveTo(-1 * masterHomeOffsetSteps);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  stepper.setCurrentPosition(0);
  currentFlapPosition = 0;
  absoluteFlapCount = 0;
  drumFlapState[4] = 0;
  stepper.disableOutputs(); // De-energize coils when stationary
  
  Serial.printf("\n[SUCCESS] HOME LOCKED (Offset: %d steps). INDEXED TO BLANK FLAP (0).\n", masterHomeOffsetSteps);
}

// --- VERSION COMPARISON HELPER ---
bool isNewerVersion(const String& serverVer, const String& currentVer) {
  if (serverVer == currentVer || serverVer.length() == 0) return false;
  int sMajor = 0, sMinor = 0, sPatch = 0;
  int cMajor = 0, cMinor = 0, cPatch = 0;
  sscanf(serverVer.c_str(), "%d.%d.%d", &sMajor, &sMinor, &sPatch);
  sscanf(currentVer.c_str(), "%d.%d.%d", &cMajor, &cMinor, &cPatch);
  if (sMajor != cMajor) return sMajor > cMajor;
  if (sMinor != cMinor) return sMinor > cMinor;
  if (sPatch != cPatch) return sPatch > cPatch;
  return false;
}

// --- BACKGROUND ASYNCHRONOUS OTA TASK ---
void otaTaskFunction(void* pvParameters) {
  otaRunning = true;
  otaProgressPercent = 0;
  otaState = "CONNECTING";
  logOTA("\n[OTA] --- STARTING FIRMWARE UPDATE CHECK ---");
  logOTA("[OTA] Current Firmware Version: v" + CURRENT_VERSION);
  logOTA("[OTA] Connecting to GitHub manifest...");

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);

  if (!http.begin(client, VERSION_URL)) {
    logOTA("[OTA] ERROR: Unable to connect to version URL.");
    otaState = "ERROR";
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    logOTA("[OTA] ERROR: Version check failed, HTTP code: " + String(httpCode));
    http.end();
    otaState = "ERROR";
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }

  String serverVersion = http.getString();
  serverVersion.trim();
  http.end();

  logOTA("[OTA] Server Manifest Version: v" + serverVersion);

  if (!isNewerVersion(serverVersion, CURRENT_VERSION)) {
    logOTA("[OTA] Device is running the latest firmware (v" + CURRENT_VERSION + ").");
    otaState = "UP_TO_DATE";
    otaProgressPercent = -1;
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }

  logOTA("[OTA] New version available (v" + serverVersion + " > v" + CURRENT_VERSION + ")!");
  logOTA("[OTA] Connecting to GitHub binary stream...");
  otaState = "DOWNLOADING";

  if (!http.begin(client, FIRMWARE_URL)) {
    logOTA("[OTA] ERROR: Unable to connect to firmware binary URL.");
    otaState = "ERROR";
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int fwHttpCode = http.GET();
  if (fwHttpCode != HTTP_CODE_OK) {
    logOTA("[OTA] ERROR: Binary download failed, HTTP code: " + String(fwHttpCode));
    http.end();
    otaState = "ERROR";
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }

  int contentLength = http.getSize();
  logOTA("[OTA] Binary size: " + String(contentLength) + " bytes");

  if (contentLength <= 0) {
    logOTA("[OTA] ERROR: Invalid content length received.");
    http.end();
    otaState = "ERROR";
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }

  if (!Update.begin(contentLength)) {
    logOTA("[OTA] ERROR: Insufficient flash partition space for OTA.");
    http.end();
    otaState = "ERROR";
    otaRunning = false;
    vTaskDelete(NULL);
    return;
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t written = 0;
  uint8_t buff[1024] = { 0 };
  int lastProgress = -1;

  logOTA("[OTA] Streaming & flashing to secondary partition...");

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
        otaProgressPercent = progress;
        if (progress != lastProgress && (progress % 10 == 0 || progress == 100)) {
          logOTA("[OTA] Progress: " + String(progress) + "% (" + String(written) + "/" + String(contentLength) + " bytes)");
          lastProgress = progress;
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  http.end();

  if (written == (size_t)contentLength) {
    if (Update.end(true)) {
      logOTA("[OTA] SUCCESS! Firmware successfully flashed.");
      logOTA("[OTA] REBOOTING ESP32 IN 2 SECONDS...");
      otaState = "SUCCESS";
      otaProgressPercent = 100;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      ESP.restart();
    } else {
      logOTA("[OTA] ERROR: Update finalization error #" + String(Update.getError()));
      otaState = "ERROR";
      otaRunning = false;
    }
  } else {
    logOTA("[OTA] ERROR: Incomplete payload received (" + String(written) + "/" + String(contentLength) + " bytes).");
    Update.abort();
    otaState = "ERROR";
    otaRunning = false;
  }

  vTaskDelete(NULL);
}

// ==========================================
// --- HTML: MAIN COUNTDOWN & 5-DRUM PAGE ---
// ==========================================
const char* mainPageHtml = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Swan Station Terminal</title>
  <style>
    body { background-color: #0b0b0b; color: #00ff66; font-family: 'Courier New', monospace; text-align: center; margin: 0; padding: 20px 10px; }
    .container { max-width: 480px; margin: 0 auto; }
    .header-bar { display: flex; justify-content: space-between; align-items: center; padding: 0 5px 15px 5px; border-bottom: 1px solid #1a3322; }
    .title-box { text-align: left; }
    h1 { font-size: 20px; letter-spacing: 2px; margin: 0; color: #00ff66; text-shadow: 0 0 6px rgba(0,255,102,0.4); }
    .subtitle { color: #557766; font-size: 11px; margin-top: 3px; }
    .gear-btn { font-size: 22px; text-decoration: none; color: #00ccff; padding: 6px 10px; border: 1px solid #005577; border-radius: 6px; background: #001a26; transition: 0.2s; }
    .gear-btn:hover { background: #00334d; border-color: #00ccff; }
    .card { background: #141414; border: 1px solid #1a3322; border-radius: 8px; padding: 16px; margin: 15px 0; box-shadow: 0 0 12px rgba(0,0,0,0.8); }
    .timer-val { font-size: 48px; font-weight: bold; color: #ffffff; letter-spacing: 4px; text-shadow: 0 0 10px rgba(255,255,255,0.4); margin: 8px 0; }
    
    /* 5-DRUM SPLIT FLAP STYLING */
    .split-flap-container { display: flex; justify-content: center; align-items: center; gap: 6px; margin: 15px 0 10px 0; }
    .flap-unit { display: flex; flex-direction: column; align-items: center; }
    .flap-unit label { font-size: 9px; color: #668877; margin-bottom: 3px; }
    .flap-box { width: 52px; height: 68px; background: #000; border: 2px solid #00ff66; border-radius: 6px; color: #ffffff; font-size: 34px; font-weight: bold; font-family: monospace; text-align: center; line-height: 68px; box-shadow: inset 0 0 8px rgba(0,255,102,0.2), 0 0 8px rgba(0,0,0,0.8); position: relative; box-sizing: border-box; text-transform: uppercase; }
    .flap-box::before { content: ""; position: absolute; top: 50%; left: 0; right: 0; height: 2px; background: #111; pointer-events: none; }
    .flap-colon { font-size: 36px; font-weight: bold; color: #00ff66; margin-top: 14px; }
    
    button { background-color: #1a1a1a; color: #00ff66; border: 2px solid #00ff66; padding: 12px 18px; font-size: 15px; font-family: monospace; cursor: pointer; border-radius: 6px; margin: 6px 0; width: 100%; font-weight: bold; transition: 0.15s; }
    button:active { background-color: #00ff66; color: #000; }
    .btn-exec { background-color: #003311; font-size: 17px; padding: 14px; border-color: #00ff66; }
    .btn-pause { background-color: #332200; border-color: #ffaa00; color: #ffaa00; }
    .btn-submit-flaps { background-color: #002b3d; border-color: #00ccff; color: #00ccff; margin-top: 10px; }
    
    .input-row { display: flex; justify-content: space-between; gap: 10px; margin-bottom: 10px; }
    .input-group { flex: 1; }
    .input-group label { display: block; font-size: 11px; margin-bottom: 4px; color: #88ffbb; }
    .input-group input { width: 100%; background: #000; border: 1px solid #00ff66; color: #00ff66; padding: 10px; font-size: 18px; font-family: monospace; text-align: center; border-radius: 4px; box-sizing: border-box; }
    .status-msg { font-size: 12px; color: #88ffbb; min-height: 18px; margin-top: 6px; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header-bar">
      <div class="title-box">
        <h1>SWAN STATION</h1>
        <div class="subtitle">PRIMARY CLOCK TERMINAL (v1.4.37)</div>
      </div>
      <a href="/diagnostics" class="gear-btn" title="Settings, Calibration & Diagnostics">⚙️</a>
    </div>

    <!-- MAIN TIMER DISPLAY CARD -->
    <div class="card">
      <div style="font-size: 12px; color: #88ffbb; letter-spacing: 1px;">COUNTDOWN TIMER</div>
      <div id="timerDisplay" class="timer-val">108:00</div>
      <div id="statusSubtext" style="font-size: 12px; color: #888;">HOMED & PAUSED (READY)</div>
      
      <div style="display:flex; gap:10px; margin-top:12px;">
        <button class="btn-exec" style="flex:1;" onclick="sendCmd('/execute')">START (108)</button>
        <button class="btn-pause" style="flex:1;" id="pauseBtn" onclick="sendCmd('/togglePause')">RESUME</button>
      </div>
    </div>

    <!-- 5-DRUM MANUAL SPLIT FLAP INTERFACE -->
    <div class="card">
      <div style="font-size: 12px; color: #88ffbb; letter-spacing: 1px;">SPLIT-FLAP DIGIT OVERRIDE</div>
      <div style="font-size: 11px; color: #666; margin-top: 2px;">Enter digits, letters, or symbols (Left -&gt; Right)</div>
      
      <div class="split-flap-container">
        <div class="flap-unit">
          <label>D1 (MIN 100)</label>
          <input type="text" maxlength="2" id="drum0" class="flap-box" value=" " onclick="this.select()">
        </div>
        <div class="flap-unit">
          <label>D2 (MIN 10)</label>
          <input type="text" maxlength="2" id="drum1" class="flap-box" value=" " onclick="this.select()">
        </div>
        <div class="flap-unit">
          <label>D3 (MIN 1)</label>
          <input type="text" maxlength="2" id="drum2" class="flap-box" value=" " onclick="this.select()">
        </div>
        <div class="flap-colon">:</div>
        <div class="flap-unit">
          <label>D4 (SEC 10)</label>
          <input type="text" maxlength="2" id="drum3" class="flap-box" value=" " onclick="this.select()">
        </div>
        <div class="flap-unit">
          <label>D5 (MASTER)</label>
          <input type="text" maxlength="2" id="drum4" class="flap-box" value=" " onclick="this.select()">
        </div>
      </div>
      
      <button class="btn-submit-flaps" onclick="submit5Drums()">DISPLAY ON PHYSICAL DRUMS</button>
      <div id="drumCmdStatus" class="status-msg" style="color: #00ccff;"></div>
      <div id="profileBadge" style="font-size: 11px; color: #ffaa00; margin-top: 6px;">PROFILE: 1.5A SAFE SEQUENTIAL</div>
    </div>

    <!-- CUSTOM COUNTDOWN STARTER -->
    <div class="card">
      <div style="font-size: 12px; color: #88ffbb; letter-spacing: 1px; margin-bottom: 8px;">CUSTOM COUNTDOWN START</div>
      <div class="input-row">
        <div class="input-group">
          <label>MINUTES (0-999)</label>
          <input type="number" id="startMin" min="0" max="999" value="108">
        </div>
        <div class="input-group">
          <label>SECONDS (0-59)</label>
          <input type="number" id="startSec" min="0" max="59" value="0">
        </div>
      </div>
      <button class="btn-exec" onclick="startCustomTimer()">START NEW COUNTDOWN</button>
    </div>
  </div>

  <script>
    function sendCmd(url) {
      fetch(url).catch(err => console.log(err));
    }

    function submit5Drums() {
      let d0 = document.getElementById('drum0').value;
      let d1 = document.getElementById('drum1').value;
      let d2 = document.getElementById('drum2').value;
      let d3 = document.getElementById('drum3').value;
      let d4 = document.getElementById('drum4').value;
      let statusEl = document.getElementById('drumCmdStatus');
      statusEl.innerText = "Dispatching [" + d0 + " " + d1 + " " + d2 + " : " + d3 + " " + d4 + "] to drums...";
      
      fetch('/setCustomFlaps?d0=' + encodeURIComponent(d0) + '&d1=' + encodeURIComponent(d1) + '&d2=' + encodeURIComponent(d2) + '&d3=' + encodeURIComponent(d3) + '&d4=' + encodeURIComponent(d4))
        .then(res => res.text())
        .then(msg => { statusEl.innerText = msg; })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function startCustomTimer() {
      let m = document.getElementById('startMin').value;
      let s = document.getElementById('startSec').value;
      fetch('/startCountdown?min=' + m + '&sec=' + s)
        .then(res => res.text())
        .catch(err => console.log(err));
    }

    // Fast 200ms status poll
    setInterval(() => {
      fetch('/status')
        .then(res => res.json())
        .then(data => {
          document.getElementById('timerDisplay').innerText = data.timer;
          document.getElementById('pauseBtn').innerText = data.paused ? "RESUME" : "PAUSE";
          document.getElementById('statusSubtext').innerText = data.statusText || (data.paused ? "TIMER PAUSED" : "COUNTING DOWN");
          if (data.powerMode !== undefined) {
            let badge = document.getElementById('profileBadge');
            badge.innerText = data.powerMode == 0 
              ? "PROFILE: 1.5A SAFE SEQUENTIAL (Brownout-Proof)" 
              : "PROFILE: 3.0A+ FAST SIMULTANEOUS";
            badge.style.color = data.powerMode == 0 ? "#ffaa00" : "#00ccff";
          }
        })
        .catch(err => console.log(err));
    }, 200);
  </script>
</body>
</html>
)rawliteral";

// ===============================================
// --- HTML: SETTINGS & DIAGNOSTICS PAGE (/diagnostics) ---
// ===============================================
const char* diagnosticsPageHtml = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Swan Station Diagnostics</title>
  <style>
    body { background-color: #0b0b0b; color: #00ff66; font-family: 'Courier New', monospace; text-align: center; margin: 0; padding: 20px 10px; }
    .container { max-width: 480px; margin: 0 auto; }
    .header-bar { display: flex; justify-content: space-between; align-items: center; padding: 0 5px 15px 5px; border-bottom: 1px solid #1a3322; }
    .title-box { text-align: left; }
    h1 { font-size: 19px; letter-spacing: 2px; margin: 0; color: #00ccff; }
    .back-btn { font-size: 13px; text-decoration: none; color: #00ff66; padding: 6px 12px; border: 1px solid #00ff66; border-radius: 6px; background: #002211; font-weight: bold; transition: 0.15s; }
    .back-btn:hover { background: #004422; }
    .card { background: #141414; border: 1px solid #005577; border-radius: 8px; padding: 15px; margin: 14px 0; text-align: left; box-shadow: 0 0 10px rgba(0,0,0,0.8); }
    .card label { display: block; font-size: 12px; margin-bottom: 6px; color: #00ccff; font-weight: bold; letter-spacing: 1px; }
    button { background-color: #1a1a1a; color: #00ff66; border: 1px solid #00ff66; padding: 10px 16px; font-size: 14px; font-family: monospace; cursor: pointer; border-radius: 6px; margin: 5px 0; width: 100%; font-weight: bold; }
    button:active { background-color: #00ff66; color: #000; }
    .btn-ota { background-color: #001a33; border-color: #00ccff; color: #00ccff; padding: 12px; }
    .btn-audio-play { background-color: #260026; border-color: #ff00ff; color: #ff00ff; }
    .btn-audio-stop { background-color: #260000; border-color: #ff3333; color: #ff3333; }
    input[type="number"], select { background: #000; border: 1px solid #00ccff; color: #00ccff; padding: 9px; font-size: 15px; font-family: monospace; text-align: center; width: 100%; border-radius: 4px; margin-bottom: 8px; box-sizing: border-box; }
    input[type="range"] { width: 100%; margin: 12px 0; accent-color: #00ccff; cursor: pointer; }
    .input-row { display: flex; justify-content: space-between; gap: 10px; margin-bottom: 8px; }
    .input-group { flex: 1; }
    .ota-status { font-size: 11px; margin-top: 6px; color: #88ffbb; min-height: 16px; }
    .console-box { background: #000; border: 1px solid #00ccff; color: #00ccff; height: 110px; overflow-y: scroll; font-size: 11px; padding: 8px; border-radius: 4px; margin-top: 6px; text-align: left; white-space: pre-wrap; word-break: break-all; font-family: monospace; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header-bar">
      <div class="title-box">
        <h1>SETTINGS & CALIBRATION</h1>
        <div style="color: #666; font-size: 11px;">SYSTEM DIAGNOSTICS (v1.4.37)</div>
      </div>
      <a href="/" class="back-btn">← TERMINAL</a>
    </div>

    <!-- POWER & MOVEMENT PROFILE -->
    <div class="card" style="border-color: #ffaa00;">
      <label style="color: #ffaa00;">POWER & MOVEMENT PROFILE</label>
      <div style="font-size: 11px; color: #ffdd88; margin-bottom: 8px;">
        Choose movement behavior based on your 5V power supply capacity.
      </div>
      <div class="input-group">
        <label>ACTIVE POWER PROFILE</label>
        <select id="powerModeSelect" onchange="submitPowerMode()">
          <option value="0">Safe Sequential Mode (1.5A Power Supply)</option>
          <option value="1">Fast Simultaneous Mode (3.0A+ Power Supply)</option>
        </select>
      </div>
      <div id="powerModeStatus" style="font-size: 11px; color: #88ffbb; margin-top: 4px;">
        Safe Mode: Moves drums one-by-one (max ~300mA draw) to eliminate brownout reboots.
      </div>
    </div>

    <!-- DRUM ZERO CALIBRATION TOOL -->
    <div class="card" style="border-color: #00ff66;">
      <label style="color:#00ff66;">DRUM ZERO CALIBRATION TOOL</label>
      <div style="font-size: 11px; color: #88ffbb; margin-bottom: 8px;">
        Jog drum until visible flap is the <b>BLANK FLAP</b> (Home baseline), then click <b>SET AS HOME (BLANK FLAP)</b>.
      </div>
      
      <div class="input-group">
        <label>TARGET DRUM</label>
        <select id="calUnitSelect">
          <option value="0">Master Drum (Right-most - Local)</option>
          <option value="1">Slave 1 (2nd from Right - I2C 0x01)</option>
          <option value="2">Slave 2 (3rd from Right - I2C 0x02)</option>
          <option value="3">Slave 3 (4th from Right - I2C 0x03)</option>
          <option value="4">Slave 4 (5th from Right / Left-most - I2C 0x04)</option>
        </select>
      </div>

      <label style="margin-top: 8px;">JOG STEPS & FLAPS</label>
      <div class="input-row">
        <button style="flex:1;" onclick="jogFlaps(1)">+1 FLAP</button>
        <button style="flex:1;" onclick="jogFlaps(5)">+5 FLAPS</button>
        <button style="flex:1;" onclick="jogFlaps(10)">+10 FLAPS</button>
      </div>
      <div class="input-row">
        <button style="flex:1; font-size: 12px;" onclick="jogSteps(10)">+10 STEPS (FINE TUNE)</button>
      </div>

      <button style="background-color: #00441b; border-color: #00ff66; color: #fff; margin-top: 8px; padding: 12px;" onclick="setAsHome()">SET CURRENT POSITION AS HOME (BLANK FLAP)</button>
      <button style="background-color: #002244; border-color: #00ccff; color: #00ccff; font-size: 12px;" onclick="reHomeDrum()">RE-HOME & VERIFY ALIGNMENT</button>
      <div id="calStatus" class="ota-status" style="color: #ffff88;"></div>
    </div>

    <!-- MANUAL SINGLE DRUM POSITION -->
    <div class="card">
      <label>MANUAL SINGLE DRUM POSITION</label>
      <div class="input-group">
        <label>TARGET DRUM</label>
        <select id="flapUnitSelect">
          <option value="0">Master Drum (Right-most - Local)</option>
          <option value="1">Slave 1 (2nd from Right - I2C 0x01)</option>
          <option value="2">Slave 2 (3rd from Right - I2C 0x02)</option>
          <option value="3">Slave 3 (4th from Right - I2C 0x03)</option>
          <option value="4">Slave 4 (5th from Right / Left-most - I2C 0x04)</option>
        </select>
      </div>
      <div class="input-row">
        <div class="input-group">
          <label>FLAP (0-44)</label>
          <input type="number" id="flapInput" min="0" max="44" value="1">
        </div>
        <div class="input-group">
          <label>EXTRA ROTATIONS</label>
          <input type="number" id="rotationsInput" min="0" max="10" value="0">
        </div>
      </div>
      <button onclick="submitSingleFlap()">MOVE DRUM TO FLAP</button>
      <div id="flapControlStatus" class="ota-status" style="color: #ff88ff;"></div>
    </div>

    <!-- I2C BUS DIAGNOSTICS & SLAVE SCANNER -->
    <div class="card">
      <label>I2C BUS DIAGNOSTICS & SLAVE SCANNER</label>
      <button class="btn-ota" onclick="runI2CScan()">SCAN I2C BUS (0x01 - 0x77)</button>
      <pre id="i2cScanResults" class="console-box" style="color: #88ffbb; border-color: #00ff66;">Ready to scan I2C bus.</pre>
    </div>

    <!-- MOTOR SPEED CONTROL -->
    <div class="card">
      <label>MOTOR SPEED CONTROL</label>
      <div style="display:flex; justify-content:space-between; font-size:12px; color:#fff;">
        <span>Speed</span><span id="speedLabel">800 Steps/Sec</span>
      </div>
      <input type="range" id="speedSlider" min="300" max="1000" step="50" value="800" onchange="submitSpeed()">
    </div>

    <!-- AUDIO SYSTEM (DFPLAYER MP3) -->
    <div class="card">
      <label>AUDIO SYSTEM (DFPLAYER MP3)</label>
      <div id="audioSdStatus" style="font-size: 11px; color: #88ffbb; margin-bottom: 8px;">QUERYING SD CARD...</div>
      <div class="input-row">
        <div class="input-group">
          <label>TRACK SELECT</label>
          <select id="trackSelect" onchange="syncTrackInput()">
            <option value="1">Track 1</option>
          </select>
        </div>
        <div class="input-group">
          <label>CUSTOM #</label>
          <input type="number" id="trackInput" min="1" max="999" value="1" onchange="syncTrackSelect()">
        </div>
      </div>
      <div style="display:flex; gap:10px;">
        <button class="btn-audio-play" style="flex:1;" onclick="playTrack()">PLAY TRACK</button>
        <button class="btn-audio-stop" style="flex:1;" onclick="stopAudio()">STOP</button>
      </div>
      <label style="margin-top: 12px;">VOLUME (LIVE DIAL SYNC): <span id="volumeLabel" style="color:#fff;">20 / 30</span></label>
      <input type="range" id="volumeSlider" min="0" max="30" step="1" value="20" oninput="onSliderInput()" onchange="submitVolume()">
      <div id="audioStatus" class="ota-status" style="color: #ff88ff;"></div>
    </div>

    <!-- FIRMWARE MANAGEMENT & OTA TERMINAL -->
    <div class="card">
      <label>FIRMWARE MANAGEMENT & OTA TERMINAL</label>
      <button class="btn-ota" id="btnOta" onclick="checkUpdate()">CHECK FOR FIRMWARE UPDATES</button>
      <div id="otaProgressContainer" style="display:none; background:#002233; border:1px solid #00ccff; height:14px; border-radius:4px; margin:8px 0;">
        <div id="otaProgressBar" style="background:#00ccff; height:100%; width:0%; transition:width 0.3s ease;"></div>
      </div>
      <label style="margin-top: 8px;">LIVE OTA SERIAL LOGS</label>
      <pre id="otaConsole" class="console-box">[SYSTEM] Ready for firmware operations.</pre>
    </div>
  </div>

  <script>
    function jogFlaps(flaps) {
      let unit = document.getElementById('calUnitSelect').value;
      let statusEl = document.getElementById('calStatus');
      statusEl.innerText = "Jogging Drum " + unit + " by +" + flaps + " flaps...";
      fetch('/calibrateJog?unit=' + unit + '&flaps=' + flaps)
        .then(res => res.text())
        .then(msg => { statusEl.innerText = msg; })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function jogSteps(steps) {
      let unit = document.getElementById('calUnitSelect').value;
      let statusEl = document.getElementById('calStatus');
      statusEl.innerText = "Jogging Drum " + unit + " by +" + steps + " steps...";
      fetch('/calibrateJog?unit=' + unit + '&steps=' + steps)
        .then(res => res.text())
        .then(msg => { statusEl.innerText = msg; })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function setAsHome() {
      let unit = document.getElementById('calUnitSelect').value;
      let statusEl = document.getElementById('calStatus');
      statusEl.innerText = "Saving Blank Flap Home for Drum " + unit + "...";
      fetch('/setHome?unit=' + unit)
        .then(res => res.text())
        .then(msg => { statusEl.innerText = msg; })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function reHomeDrum() {
      let unit = document.getElementById('calUnitSelect').value;
      let statusEl = document.getElementById('calStatus');
      statusEl.innerText = "Re-homing Drum " + unit + "...";
      fetch('/reHome?unit=' + unit)
        .then(res => res.text())
        .then(msg => { statusEl.innerText = msg; })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function submitSingleFlap() {
      let unit = document.getElementById('flapUnitSelect').value;
      let val = document.getElementById('flapInput').value;
      let rot = document.getElementById('rotationsInput').value;
      let statusEl = document.getElementById('flapControlStatus');
      statusEl.innerText = "Moving Drum " + unit + " to Flap " + val + "...";
      fetch('/setFlap?unit=' + unit + '&val=' + val + '&rot=' + rot)
        .then(res => res.text())
        .then(msg => { statusEl.innerText = msg; })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function runI2CScan() {
      let resEl = document.getElementById('i2cScanResults');
      resEl.innerText = "[I2C] Scanning bus addresses 0x01 to 0x77...";
      fetch('/scanI2C')
        .then(res => res.json())
        .then(data => { resEl.innerText = data.logs; })
        .catch(err => { resEl.innerText = "[ERROR] I2C Scan request failed: " + err; });
    }

    function submitSpeed() {
      let val = document.getElementById('speedSlider').value;
      document.getElementById('speedLabel').innerText = val + " Steps/Sec";
      fetch('/setSpeed?val=' + val).catch(err => console.log(err));
    }

    let isUserInteractingSlider = false;

    function playTrack() {
      let track = document.getElementById('trackInput').value;
      fetch('/playTrack?track=' + track).then(res => res.text()).then(msg => {
        document.getElementById('audioStatus').innerText = msg;
      });
    }

    function stopAudio() {
      fetch('/stopAudio').then(res => res.text()).then(msg => {
        document.getElementById('audioStatus').innerText = msg;
      });
    }

    function onSliderInput() {
      let val = document.getElementById('volumeSlider').value;
      document.getElementById('volumeLabel').innerText = val + " / 30";
    }

    function submitVolume() {
      let val = document.getElementById('volumeSlider').value;
      fetch('/setVolume?val=' + val);
    }

    function syncTrackInput() {
      document.getElementById('trackInput').value = document.getElementById('trackSelect').value;
    }

    function syncTrackSelect() {
      let customVal = document.getElementById('trackInput').value;
      let sel = document.getElementById('trackSelect');
      for (let i = 0; i < sel.options.length; i++) {
        if (sel.options[i].value == customVal) {
          sel.selectedIndex = i;
          return;
        }
      }
    }

    function loadAudioInfo() {
      fetch('/getAudioInfo')
        .then(res => res.json())
        .then(data => {
          let sdStatus = document.getElementById('audioSdStatus');
          let sel = document.getElementById('trackSelect');
          if (data.trackCount > 0) {
            sdStatus.innerText = "SD CARD DETECTED (" + data.trackCount + " TRACKS FOUND)";
            sel.innerHTML = "";
            for (let i = 1; i <= data.trackCount; i++) {
              let opt = document.createElement('option');
              opt.value = i;
              opt.innerText = "Track " + i;
              sel.appendChild(opt);
            }
          } else {
            sdStatus.innerText = "SD CARD READY (DIRECT TRACK ACCESS)";
          }
        })
        .catch(err => console.log(err));
    }

    let otaPollInterval = null;
    function checkUpdate() {
      document.getElementById('btnOta').disabled = true;
      document.getElementById('btnOta').innerText = "CHECKING IN BACKGROUND...";
      fetch('/check-update')
        .then(res => res.text())
        .then(msg => { startOtaPolling(); })
        .catch(err => {
          document.getElementById('btnOta').disabled = false;
          document.getElementById('btnOta').innerText = "CHECK FOR FIRMWARE UPDATES";
        });
    }

    function startOtaPolling() {
      if (otaPollInterval) clearInterval(otaPollInterval);
      otaPollInterval = setInterval(pollOtaStatus, 500);
      pollOtaStatus();
    }

    function pollOtaStatus() {
      fetch('/ota-status')
        .then(res => res.json())
        .then(data => {
          let consoleEl = document.getElementById('otaConsole');
          if (data.logs) {
            consoleEl.innerText = data.logs;
            consoleEl.scrollTop = consoleEl.scrollHeight;
          }
          let progressContainer = document.getElementById('otaProgressContainer');
          let progressBar = document.getElementById('otaProgressBar');
          let btnOta = document.getElementById('btnOta');

          if (data.running) {
            btnOta.disabled = true;
            btnOta.innerText = "FIRMWARE FLASHING IN PROGRESS...";
            if (data.progress >= 0) {
              progressContainer.style.display = "block";
              progressBar.style.width = data.progress + "%";
            }
          } else {
            btnOta.disabled = false;
            btnOta.innerText = "CHECK FOR FIRMWARE UPDATES";
            if (data.progress === 100) {
              progressBar.style.width = "100%";
            }
          }
        })
        .catch(err => console.log(err));
    }

    let sliderEl = document.getElementById('volumeSlider');
    sliderEl.addEventListener('mousedown', () => { isUserInteractingSlider = true; });
    sliderEl.addEventListener('touchstart', () => { isUserInteractingSlider = true; });
    sliderEl.addEventListener('mouseup', () => { isUserInteractingSlider = false; });
    sliderEl.addEventListener('touchend', () => { isUserInteractingSlider = false; });

    function submitPowerMode() {
      let mode = document.getElementById('powerModeSelect').value;
      let statusEl = document.getElementById('powerModeStatus');
      statusEl.innerText = "Saving power profile...";
      fetch('/setPowerMode?mode=' + mode)
        .then(res => res.text())
        .then(msg => {
          statusEl.innerText = mode == "0" 
            ? "Safe Mode ACTIVE: Drums move one-at-a-time (max ~300mA draw)." 
            : "Fast Mode ACTIVE: All drums rotate simultaneously (requires 3A+ supply).";
        })
        .catch(err => { statusEl.innerText = "[ERROR] " + err; });
    }

    function loadPowerMode() {
      fetch('/status')
        .then(res => res.json())
        .then(data => {
          if (data.powerMode !== undefined) {
            document.getElementById('powerModeSelect').value = data.powerMode;
            document.getElementById('powerModeStatus').innerText = data.powerMode == 0 
              ? "Safe Mode ACTIVE: Drums move one-at-a-time (max ~300mA draw)." 
              : "Fast Mode ACTIVE: All drums rotate simultaneously (requires 3A+ supply).";
          }
        })
        .catch(err => console.log(err));
    }

    loadPowerMode();
    loadAudioInfo();
    pollOtaStatus();
  </script>
</body>
</html>
)rawliteral";

// ==============================
// --- WEB SERVER ENDPOINTS ---
// ==============================

void handleRoot() {
  server.send(200, "text/html", mainPageHtml);
}

void handleDiagnostics() {
  server.send(200, "text/html", diagnosticsPageHtml);
}

void handleStatus() {
  char timerBuf[16];
  String statusMsg = "";
  if (timerState == TIMER_HOLDING) {
    long elapsed = millis() - holdStartTime;
    int remaining = (int)((HOLD_DURATION_MS - elapsed) / 1000) + 1;
    if (remaining < 1) remaining = 1;
    if (remaining > 5) remaining = 5;
    snprintf(timerBuf, sizeof(timerBuf), "%03d:%02d", currentMinutes, currentSeconds);
    statusMsg = "HOLDING " + String(timerBuf) + " (STARTS IN " + String(remaining) + "s)";
  } else if (isPaused) {
    snprintf(timerBuf, sizeof(timerBuf), "%03d:%02d", currentMinutes, currentSeconds);
    statusMsg = "TIMER PAUSED";
  } else {
    snprintf(timerBuf, sizeof(timerBuf), "%03d:%02d", currentMinutes, currentSeconds);
    statusMsg = "COUNTING DOWN";
  }

  String json = "{";
  json += "\"timer\":\"" + String(timerBuf) + "\",";
  json += "\"statusText\":\"" + statusMsg + "\",";
  json += "\"holding\":" + String(timerState == TIMER_HOLDING ? "true" : "false") + ",";
  json += "\"minutes\":" + String(currentMinutes) + ",";
  json += "\"seconds\":" + String(currentSeconds) + ",";
  json += "\"paused\":" + String(isPaused ? "true" : "false") + ",";
  json += "\"volume\":" + String(currentVolume) + ",";
  json += "\"flap\":" + String(currentFlapPosition) + ",";
  json += "\"d0\":" + String(drumFlapState[0]) + ",";
  json += "\"d1\":" + String(drumFlapState[1]) + ",";
  json += "\"d2\":" + String(drumFlapState[2]) + ",";
  json += "\"d3\":" + String(drumFlapState[3]) + ",";
  json += "\"d4\":" + String(drumFlapState[4]) + ",";
  json += "\"powerMode\":" + String(powerMode);
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetPowerMode() {
  if (server.hasArg("mode")) {
    int mode = server.arg("mode").toInt();
    savePowerSettings(mode);
    Serial.printf("[POWER] Power Profile set to: %s\n", (powerMode == 0 ? "Safe Sequential (1.5A)" : "Fast Simultaneous (3.0A+)"));
    server.send(200, "text/plain", "Power profile saved.");
  } else {
    server.send(400, "text/plain", "Missing mode parameter");
  }
}

void handleSetCustomFlaps() {
  isPaused = true; // Auto-pause countdown when manual flaps are commanded
  timerState = TIMER_STOPPED;

  String dStr[5];
  dStr[0] = server.hasArg("d0") ? server.arg("d0") : " ";
  dStr[1] = server.hasArg("d1") ? server.arg("d1") : " ";
  dStr[2] = server.hasArg("d2") ? server.arg("d2") : " ";
  dStr[3] = server.hasArg("d3") ? server.arg("d3") : " ";
  dStr[4] = server.hasArg("d4") ? server.arg("d4") : " ";

  String resultLog = "";

  for (int i = 0; i < 5; i++) {
    dStr[i].trim();
    if (dStr[i].length() == 0 && drumFlapState[i] == 0) {
      // Drum is already at blank home and input is blank; skip sending to avoid bus delays on unattached drums (e.g. D1)
      continue;
    }
    int flap = 0;
    if (dStr[i].length() == 0) {
      flap = 0;
    } else if (dStr[i].length() == 1) {
      flap = charToFlap(dStr[i].charAt(0), drumFlapState[i]);
    } else {
      flap = dStr[i].toInt();
    }

    bool ok = setDrumFlap(i, flap);
    if (i == 4) {
      resultLog += "D5(M):Flap " + String(flap) + " ";
    } else {
      uint8_t addr = 4 - i;
      resultLog += "D" + String(i + 1) + "(0x0" + String(addr) + "):Flap " + String(flap) + (ok ? " [OK] " : " [ERR] ");
    }
    
    // In Safe Sequential Mode (1.5A), wait for drum to finish and de-energize coils before starting next
    if (powerMode == 0 && ok) {
      waitForDrum(i);
    } else {
      delay(150); // In Fast Mode (3A+), stagger slightly
    }
  }

  for (int k = 0; k < 5; k++) lastDisplayedDigits[k] = -1;

  Serial.printf("[MANUAL] 5-Drum Override: %s\n", resultLog.c_str());
  server.send(200, "text/plain", resultLog.length() > 0 ? resultLog : "All selected drums already in position.");
}

void initiateCountdown(int mins, int secs, bool dramaticMasterRot = false) {
  currentMinutes = mins;
  currentSeconds = secs;
  isPaused = false;
  timerState = TIMER_HOLDING;

  int d0 = (mins / 100) % 10;
  int d1 = (mins / 10) % 10;
  int d2 = mins % 10;
  int d3 = (secs / 10) % 10;
  int d4 = secs % 10;

  Serial.printf("[SYSTEM] Aligning drums to starting time %03d:%02d...\n", mins, secs);

  bool ok0 = setDrumFlap(0, getNextFlapForDigit(drumFlapState[0], d0));
  if (ok0 && powerMode == 0) waitForDrum(0); else delay(100);

  bool ok1 = setDrumFlap(1, getNextFlapForDigit(drumFlapState[1], d1));
  if (ok1 && powerMode == 0) waitForDrum(1); else delay(100);

  bool ok2 = setDrumFlap(2, getNextFlapForDigit(drumFlapState[2], d2));
  if (ok2 && powerMode == 0) waitForDrum(2); else delay(100);

  bool ok3 = setDrumFlap(3, getNextFlapForDigit(drumFlapState[3], d3));
  if (ok3 && powerMode == 0) waitForDrum(3); else delay(100);

  int extra = dramaticMasterRot ? 2 : 0;
  bool ok4 = setDrumFlap(4, getNextFlapForDigit(drumFlapState[4], d4), extra);
  if (ok4 && powerMode == 0) waitForDrum(4);

  lastDisplayedDigits[0] = d0;
  lastDisplayedDigits[1] = d1;
  lastDisplayedDigits[2] = d2;
  lastDisplayedDigits[3] = d3;
  lastDisplayedDigits[4] = d4;

  holdStartTime = millis(); // Start the 5-second countdown hold
  Serial.printf("[SYSTEM] Drums aligned to %03d:%02d. Holding for 5 seconds before countdown begins...\n", mins, secs);
}

void handleStartCountdown() {
  if (server.hasArg("min") && server.hasArg("sec")) {
    int m = server.arg("min").toInt();
    int s = server.arg("sec").toInt();
    if (m < 0) m = 0;
    if (s < 0) s = 0;
    if (s > 59) s = 59;
    initiateCountdown(m, s, false);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing min or sec parameter");
  }
}

void handleExecute() {
  Serial.println("\n[SYSTEM] > 4 8 15 16 23 42");
  Serial.println("[SYSTEM] > OVERRIDE ACCEPTED. RESETTING TO 108:00\n");
  initiateCountdown(START_MINUTES, 0, true);
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
    int rot = server.hasArg("rot") ? server.arg("rot").toInt() : 0;
    int unit = server.hasArg("unit") ? server.arg("unit").toInt() : 0;
    
    isPaused = true; 
    
    if (unit == 0) {
      Serial.printf("[DEBUG] Manual override: Master Drum moving to Flap %d (+%d rot)\n", target, rot);
      goToFlap(target, rot);
      server.send(200, "text/plain", "Master Drum set to Flap " + String(target));
    } else {
      Serial.printf("[DEBUG] Manual override: Sending Flap %d to Slave Unit %d\n", target, unit);
      Wire.setTimeOut(30);
      Wire.beginTransmission((uint8_t)unit);
      Wire.write((uint8_t)target);
      Wire.write((uint8_t)10); // Speed RPM
      uint8_t err = Wire.endTransmission(true);
      if (err == 0) {
        // Update tracking state: Unit 1 -> Drum 3, Unit 2 -> Drum 2, Unit 3 -> Drum 1, Unit 4 -> Drum 0
        int drumIdx = 4 - unit;
        if (drumIdx >= 0 && drumIdx < 5) drumFlapState[drumIdx] = target;
        server.send(200, "text/plain", "Slave " + String(unit) + " set to Flap " + String(target));
      } else {
        server.send(200, "text/plain", "I2C Error code " + String(err) + " on Slave " + String(unit));
      }
    }
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
  if (otaRunning) {
    server.send(200, "text/plain", "Update check already in progress.");
    return;
  }
  otaLogBuffer = "";
  logOTA("[SYSTEM] OTA update check triggered via Web Dashboard.");
  xTaskCreatePinnedToCore(otaTaskFunction, "OTA_Task", 8192, NULL, 1, &otaTaskHandle, 0);
  server.send(200, "text/plain", "OTA check started in background...");
}

void handleOtaStatus() {
  String json = "{";
  json += "\"running\":" + String(otaRunning ? "true" : "false") + ",";
  json += "\"state\":\"" + otaState + "\",";
  json += "\"progress\":" + String(otaProgressPercent) + ",";
  String escapedLogs = otaLogBuffer;
  escapedLogs.replace("\\", "\\\\");
  escapedLogs.replace("\"", "\\\"");
  escapedLogs.replace("\n", "\\n");
  escapedLogs.replace("\r", "");
  json += "\"logs\":\"" + escapedLogs + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// --- AUDIO WEB HANDLERS ---
void handlePlayTrack() {
  int track = 1;
  if (server.hasArg("track")) {
    track = server.arg("track").toInt();
  }
  if (track < 1) track = 1;
  dfPlayer.play(track);
  Serial.printf("[AUDIO] Playing Track %d\n", track);
  server.send(200, "text/plain", "PLAYING TRACK " + String(track));
}

void handleStopAudio() {
  dfPlayer.stop();
  Serial.println("[AUDIO] Audio playback stopped.");
  server.send(200, "text/plain", "AUDIO STOPPED");
}

void handleSetVolume() {
  if (server.hasArg("val")) {
    currentVolume = server.arg("val").toInt();
    currentVolume = constrain(currentVolume, 0, 30);
    lastPotVolume = currentVolume;
    dfPlayer.volume(currentVolume);
    Serial.printf("[AUDIO] Volume set to %d/30\n", currentVolume);
    server.send(200, "text/plain", "VOLUME: " + String(currentVolume) + "/30");
  } else {
    server.send(400, "text/plain", "Missing Parameter");
  }
}

void handleGetAudioInfo() {
  int count = dfPlayer.readFileCounts();
  if (count < 0) count = 0;
  String json = "{";
  json += "\"ready\":" + String(dfPlayerReady ? "true" : "false") + ",";
  json += "\"trackCount\":" + String(count) + ",";
  json += "\"volume\":" + String(currentVolume);
  json += "}";
  server.send(200, "application/json", json);
}

void handleScanI2C() {
  String logOutput = "=== I2C BUS SCAN (GPIO 21 SDA, GPIO 22 SCL) ===\n";
  int count = 0;
  
  Wire.setTimeOut(15);
  Wire.setClock(100000);

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission(true);

    if (error == 0) {
      logOutput += "[SUCCESS] Device found at 0x" + (address < 16 ? String("0") : String("")) + String(address, HEX) + " (Decimal " + String(address) + ")\n";
      count++;
    } else if (error == 4) {
      logOutput += "[ERROR] Unknown I2C error at address 0x" + String(address, HEX) + "\n";
    }
  }

  if (count == 0) {
    logOutput += "[RESULT] No I2C devices detected.\n";
    logOutput += "Check: Common Ground, 3.3V/5V Level Shifter power, and A4/A5 wiring.";
  } else {
    logOutput += "[RESULT] Scan complete: " + String(count) + " device(s) responded!";
  }

  String json = "{";
  json += "\"count\":" + String(count) + ",";
  String escapedLogs = logOutput;
  escapedLogs.replace("\\", "\\\\");
  escapedLogs.replace("\"", "\\\"");
  escapedLogs.replace("\n", "\\n");
  escapedLogs.replace("\r", "");
  json += "\"logs\":\"" + escapedLogs + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleCalibrateJog() {
  int unit = server.hasArg("unit") ? server.arg("unit").toInt() : 0;
  int flaps = server.hasArg("flaps") ? server.arg("flaps").toInt() : 0;
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 0;

  isPaused = true;

  if (unit == 0) {
    // Master Drum (Local ESP32)
    long delta = 0;
    if (flaps > 0) delta = (long)(flaps * STEPS_PER_FLAP);
    if (steps > 0) delta += steps;
    
    stepper.enableOutputs();
    stepper.move(-1 * delta);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    stepper.disableOutputs(); // Power off coils when jog move finishes
    server.send(200, "text/plain", "Master jogged by " + (flaps > 0 ? String(flaps) + " flaps." : String(steps) + " steps."));
  } else {
    // Slave Drum (Arduino Nano over I2C)
    Wire.setTimeOut(30);
    Wire.beginTransmission((uint8_t)unit);
    if (flaps > 0) {
      Wire.write((uint8_t)250); // 0xFA: Jog Flaps
      Wire.write((uint8_t)flaps);
    } else {
      Wire.write((uint8_t)251); // 0xFB: Jog Steps
      Wire.write((uint8_t)steps);
    }
    uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      server.send(200, "text/plain", "Slave " + String(unit) + " jogged " + (flaps > 0 ? String(flaps) + " flaps." : String(steps) + " steps."));
    } else {
      server.send(200, "text/plain", "I2C Error code " + String(err) + " communicating with Slave " + String(unit));
    }
  }
}

void handleSetHome() {
  int unit = server.hasArg("unit") ? server.arg("unit").toInt() : 0;

  if (unit == 0) {
    // Master Drum: Save current offset from Hall trigger
    long currentRelOffset = -1 * stepper.currentPosition();
    masterHomeOffsetSteps = (int)(masterHomeOffsetSteps + currentRelOffset);
    while (masterHomeOffsetSteps < 0) masterHomeOffsetSteps += STEPS_PER_REV;
    masterHomeOffsetSteps = masterHomeOffsetSteps % STEPS_PER_REV;
    saveMasterCalibration(masterHomeOffsetSteps);
    
    stepper.setCurrentPosition(0);
    currentFlapPosition = 0;
    absoluteFlapCount = 0;
    drumFlapState[4] = 0;
    stepper.disableOutputs(); // De-energize coils
    Serial.printf("[CALIBRATION] Master Home saved: %d steps\n", masterHomeOffsetSteps);
    server.send(200, "text/plain", "SUCCESS: Master Blank Flap (Home) saved (" + String(masterHomeOffsetSteps) + " steps).");
  } else {
    // Slave Drum: Send 0xFC (252) to save EEPROM
    Wire.setTimeOut(30);
    Wire.beginTransmission((uint8_t)unit);
    Wire.write((uint8_t)252);
    Wire.write((uint8_t)0);
    uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      server.send(200, "text/plain", "SUCCESS: Slave " + String(unit) + " Blank Flap (Home) saved into EEPROM.");
    } else {
      server.send(200, "text/plain", "I2C Error " + String(err) + " saving to Slave " + String(unit));
    }
  }
}

void handleReHome() {
  int unit = server.hasArg("unit") ? server.arg("unit").toInt() : 0;

  if (unit == 0) {
    findHome();
    server.send(200, "text/plain", "Master Drum re-homed to Blank Flap (0).");
  } else {
    Wire.setTimeOut(30);
    Wire.beginTransmission((uint8_t)unit);
    Wire.write((uint8_t)253);
    Wire.write((uint8_t)0);
    uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      server.send(200, "text/plain", "Slave " + String(unit) + " re-homed to Blank Flap (0).");
    } else {
      server.send(200, "text/plain", "I2C Error " + String(err) + " re-homing Slave " + String(unit));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SWAN STATION MASTER BRAIN INITIALIZING ===");

  loadMasterCalibration();
  loadPowerSettings();

  pinMode(hallSensorPin, INPUT_PULLUP);
  pinMode(potPin, INPUT);
  analogSetPinAttenuation(potPin, ADC_11db);

  // Read initial potentiometer level
  int initialPot = analogRead(potPin);
  currentVolume = constrain(map(initialPot, 15, 4080, 0, 30), 0, 30);
  lastPotVolume = currentVolume;

  stepper.setMaxSpeed(1200);
  stepper.setAcceleration(2500);

  findHome();

  // --- AUDIO INITIALIZATION (DFPlayer Mini) ---
  dfSerial.begin(9600, SERIAL_8N1, 16, 17);
  if (dfPlayer.begin(dfSerial, false, false)) {
    dfPlayerReady = true;
    dfPlayer.volume(currentVolume);
    Serial.printf("[AUDIO] DFPlayer Mini initialized. Volume: %d/30 (Pot ADC: %d)\n", currentVolume, initialPot);
  } else {
    Serial.println("[AUDIO] DFPlayer Mini offline or not responding (will accept commands).");
  }

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

  server.on("/", HTTP_GET, handleRoot);
  server.on("/diagnostics", HTTP_GET, handleDiagnostics);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/setCustomFlaps", HTTP_GET, handleSetCustomFlaps);
  server.on("/startCountdown", HTTP_GET, handleStartCountdown);
  server.on("/execute", HTTP_GET, handleExecute);
  server.on("/togglePause", HTTP_GET, handleTogglePause);
  server.on("/setFlap", HTTP_GET, handleSetFlap);
  server.on("/setSpeed", HTTP_GET, handleSetSpeed); 
  server.on("/calibrateJog", HTTP_GET, handleCalibrateJog);
  server.on("/setHome", HTTP_GET, handleSetHome);
  server.on("/reHome", HTTP_GET, handleReHome);
  server.on("/scanI2C", HTTP_GET, handleScanI2C);
  server.on("/check-update", HTTP_GET, handleCheckUpdate);
  server.on("/ota-status", HTTP_GET, handleOtaStatus);
  server.on("/playTrack", HTTP_GET, handlePlayTrack);
  server.on("/stopAudio", HTTP_GET, handleStopAudio);
  server.on("/setVolume", HTTP_GET, handleSetVolume);
  server.on("/getAudioInfo", HTTP_GET, handleGetAudioInfo);
  server.on("/setPowerMode", HTTP_GET, handleSetPowerMode);
  server.begin();
  
  Wire.begin(21, 22);

  // All drums remain at their Home position (Flap 0 - Blank Flap) on boot
  for (int i = 0; i < 5; i++) {
    drumFlapState[i] = 0;
  }
  Serial.println("[SYSTEM] Boot sequence complete. All drums homed at Flap 0. Timer paused and ready.");
}

void loop() {
  server.handleClient();
  stepper.run(); 
  if (stepper.distanceToGo() == 0) {
    stepper.disableOutputs(); // Immediately de-energize coils when stationary to prevent heating!
  } 

  unsigned long currentMillis = millis();

  // --- NON-BLOCKING POTENTIOMETER READ ---
  if (currentMillis - lastPotReadMillis >= potReadInterval) {
    lastPotReadMillis = currentMillis;
    int rawPot = analogRead(potPin);
    int mappedVol = constrain(map(rawPot, 15, 4080, 0, 30), 0, 30);
    if (mappedVol != lastPotVolume) {
      lastPotVolume = mappedVol;
      currentVolume = mappedVol;
      dfPlayer.volume(currentVolume);
      Serial.printf("[AUDIO] Volume Dial: %d/30 (ADC: %d)\n", currentVolume, rawPot);
    }
  }

  // --- TIMER STATE MACHINE & COUNTDOWN ENGINE ---
  if (timerState == TIMER_HOLDING) {
    if (!isPaused && (millis() - holdStartTime >= HOLD_DURATION_MS)) {
      timerState = TIMER_RUNNING;
      previousMillis = millis(); // Reset 1000ms baseline for 1st countdown tick
      Serial.println("[SYSTEM] 5-second hold complete. COUNTDOWN INITIATING!");
    }
  } else if (timerState == TIMER_RUNNING && !isPaused) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis += interval; // True drift-free 1000ms cadence

      if (currentMinutes == 0 && currentSeconds == 0) {
        // Red Hieroglyphs 1 through 5 (Alarm condition)
        setDrumFlap(0, 1);
        setDrumFlap(1, 2);
        setDrumFlap(2, 3);
        setDrumFlap(3, 4);
        setDrumFlap(4, 5);
        timerState = TIMER_STOPPED;
      } else {
        // Decrement time first
        if (currentSeconds == 0) {
          currentMinutes--;
          currentSeconds = 59;
        } else {
          currentSeconds--;
        }

        int d0 = (currentMinutes / 100) % 10; // Minutes 100s
        int d1 = (currentMinutes / 10) % 10;  // Minutes 10s
        int d2 = currentMinutes % 10;         // Minutes 1s
        int d3 = (currentSeconds / 10) % 10;  // Seconds 10s
        int d4 = currentSeconds % 10;         // Seconds 1s

        // Only command drums when their displayed digit actually changes!
        if (d0 != lastDisplayedDigits[0]) {
          lastDisplayedDigits[0] = d0;
          setDrumFlap(0, getNextFlapForDigit(drumFlapState[0], d0));
        }

        if (d1 != lastDisplayedDigits[1]) {
          lastDisplayedDigits[1] = d1;
          setDrumFlap(1, getNextFlapForDigit(drumFlapState[1], d1));
        }

        if (d2 != lastDisplayedDigits[2]) {
          lastDisplayedDigits[2] = d2;
          setDrumFlap(2, getNextFlapForDigit(drumFlapState[2], d2));
        }

        if (d3 != lastDisplayedDigits[3]) {
          lastDisplayedDigits[3] = d3;
          setDrumFlap(3, getNextFlapForDigit(drumFlapState[3], d3));
        }

        if (d4 != lastDisplayedDigits[4]) {
          lastDisplayedDigits[4] = d4;
          setDrumFlap(4, getNextFlapForDigit(drumFlapState[4], d4));
        }
      }
    }
  }
}
