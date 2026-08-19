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

// --- FIRMWARE VERSION & OTA CONFIGURATION ---
const String CURRENT_VERSION = "1.3.2";
const String VERSION_URL = "https://raw.githubusercontent.com/rbd3453/swanclock/main/ota/version.txt";
const String FIRMWARE_URL = "https://raw.githubusercontent.com/rbd3453/swanclock/main/ota/firmware.bin";

// --- HARDWARE PINS (Digit 1 Local Motor & Sensor) ---
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
    button { background-color: #222; color: #00ff66; border: 2px solid #00ff66; padding: 12px 20px; font-size: 16px; font-family: monospace; cursor: pointer; border-radius: 6px; margin: 5px 0; width: 100%; font-weight: bold; }
    button:active { background-color: #00ff66; color: #000; }
    .btn-exec { background-color: #003311; font-size: 20px; padding: 15px; }
    .btn-pause { background-color: #332200; border-color: #ffaa00; color: #ffaa00; }
    .btn-ota { background-color: #001a33; border-color: #00ccff; color: #00ccff; }
    .btn-audio-play { background-color: #260026; border-color: #ff00ff; color: #ff00ff; }
    .btn-audio-stop { background-color: #260000; border-color: #ff3333; color: #ff3333; }
    input[type="number"], select { background: #000; border: 1px solid #00ff66; color: #00ff66; padding: 10px; font-size: 16px; font-family: monospace; text-align: center; width: 80%; border-radius: 4px; margin-bottom: 10px; box-sizing: border-box; }
    input[type="range"] { width: 90%; margin: 15px 0; accent-color: #00ff66; cursor: pointer; }
    label { display: block; font-size: 12px; margin-bottom: 5px; color: #88ffbb; letter-spacing: 1px; }
    .input-row { display: flex; justify-content: space-between; gap: 10px; margin-bottom: 10px; }
    .input-group { flex: 1; }
    .input-group input, .input-group select { width: 100%; }
    .ota-status { font-size: 12px; margin-top: 8px; color: #88ffbb; min-height: 16px; }
    .console-box { background: #000; border: 1px solid #00ccff; color: #00ccff; height: 130px; overflow-y: scroll; font-size: 11px; padding: 8px; border-radius: 4px; margin-top: 6px; text-align: left; white-space: pre-wrap; word-break: break-all; font-family: monospace; }
    .progress-bar-bg { background: #002233; border: 1px solid #00ccff; border-radius: 4px; height: 16px; overflow: hidden; margin: 8px 0; }
    .progress-bar-fill { background: #00ccff; height: 100%; width: 0%; transition: width 0.3s ease; }
  </style>
</head>
<body>
  <h1>SWAN STATION TERMINAL</h1>
  <p style="color: #666; font-size: 11px;">LIVE SYSTEM DIAGNOSTICS (v1.3.2)</p>
  
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
    <label>AUDIO SYSTEM (DFPLAYER MP3)</label>
    <div id="audioSdStatus" style="font-size: 11px; color: #88ffbb; margin-bottom: 10px;">QUERYING SD CARD...</div>
    <div class="input-row">
      <div class="input-group">
        <label>DETECTED TRACKS</label>
        <select id="trackSelect" onchange="syncTrackInput()">
          <option value="1">Track 1</option>
        </select>
      </div>
      <div class="input-group">
        <label>CUSTOM #</label>
        <input type="number" id="trackInput" min="1" max="999" value="1" onchange="syncTrackSelect()">
      </div>
    </div>
    <button class="btn-audio-play" onclick="playTrack()">PLAY SOUND TRACK</button>
    <button class="btn-audio-stop" onclick="stopAudio()">STOP SOUND</button>
    
    <label style="margin-top: 15px;">AUDIO VOLUME (LIVE DIAL SYNC)</label>
    <span style="font-size: 12px; color:#fff;" id="volumeLabel">20 / 30</span>
    <input type="range" id="volumeSlider" min="0" max="30" step="1" value="20" oninput="onSliderInput()" onchange="submitVolume()">
    <div id="audioStatus" class="ota-status" style="color: #ff88ff;"></div>
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
        <label>TARGET DRUM</label>
        <select id="unitSelect">
          <option value="0">Master (Digit 1 - Hundreds)</option>
          <option value="1">Slave 1 (Digit 2 - Tens Min)</option>
          <option value="2">Slave 2 (Digit 3 - Ones Min)</option>
          <option value="3">Slave 3 (Digit 4 - Tens Sec)</option>
          <option value="4">Slave 4 (Digit 5 - Ones Sec)</option>
        </select>
      </div>
    </div>
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
    <label>FIRMWARE MANAGEMENT & OTA TERMINAL</label>
    <button class="btn-ota" id="btnOta" onclick="checkUpdate()">CHECK FOR FIRMWARE UPDATES</button>
    
    <div id="otaProgressContainer" style="display:none;">
      <div class="progress-bar-bg">
        <div id="otaProgressBar" class="progress-bar-fill"></div>
      </div>
    </div>

    <div style="text-align: left; margin-top: 8px;">
      <label>LIVE OTA SERIAL LOGS</label>
      <pre id="otaConsole" class="console-box">[SYSTEM] Ready for firmware operations.</pre>
    </div>
  </div>

  <script>
    function sendCmd(url) {
      fetch(url).catch(err => console.log(err));
    }
    
    function submitFlap() {
      let val = document.getElementById('flapInput').value;
      let rot = document.getElementById('rotationsInput').value;
      let unit = document.getElementById('unitSelect').value;
      fetch('/setFlap?val=' + val + '&rot=' + rot + '&unit=' + unit).catch(err => console.log(err));
    }

    function submitSpeed() {
      let val = document.getElementById('speedSlider').value;
      document.getElementById('speedLabel').innerText = val + " Steps/Sec";
      fetch('/setSpeed?val=' + val).catch(err => console.log(err));
    }

    let otaPollInterval = null;

    function checkUpdate() {
      document.getElementById('btnOta').disabled = true;
      document.getElementById('btnOta').innerText = "CHECKING IN BACKGROUND...";
      fetch('/check-update')
        .then(res => res.text())
        .then(msg => {
          startOtaPolling();
        })
        .catch(err => {
          console.log(err);
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

          if (data.progress >= 0) {
            progressContainer.style.display = "block";
            progressBar.style.width = data.progress + "%";
          } else {
            progressContainer.style.display = "none";
          }

          if (!data.running) {
            document.getElementById('btnOta').disabled = false;
            document.getElementById('btnOta').innerText = "CHECK FOR FIRMWARE UPDATES";
            if (data.state === "SUCCESS") {
              if (otaPollInterval) clearInterval(otaPollInterval);
              document.getElementById('btnOta').innerText = "UPDATE COMPLETED - REBOOTING...";
            }
          }
        })
        .catch(err => console.log(err));
    }

    function playTrack() {
      let track = document.getElementById('trackInput').value;
      let statusDiv = document.getElementById('audioStatus');
      statusDiv.innerText = "SENDING PLAY CMD (TRACK " + track + ")...";
      fetch('/playTrack?track=' + track)
        .then(res => res.text())
        .then(msg => { statusDiv.innerText = msg; })
        .catch(err => { statusDiv.innerText = "AUDIO CMD FAILED"; console.log(err); });
    }

    function stopAudio() {
      let statusDiv = document.getElementById('audioStatus');
      statusDiv.innerText = "STOPPING AUDIO...";
      fetch('/stopAudio')
        .then(res => res.text())
        .then(msg => { statusDiv.innerText = msg; })
        .catch(err => { statusDiv.innerText = "STOP CMD FAILED"; console.log(err); });
    }

    let isUserInteractingSlider = false;

    function onSliderInput() {
      isUserInteractingSlider = true;
      let val = document.getElementById('volumeSlider').value;
      document.getElementById('volumeLabel').innerText = val + " / 30";
    }

    function submitVolume() {
      let val = document.getElementById('volumeSlider').value;
      document.getElementById('volumeLabel').innerText = val + " / 30";
      fetch('/setVolume?val=' + val)
        .then(res => res.text())
        .then(msg => { 
          document.getElementById('audioStatus').innerText = msg; 
          setTimeout(() => { isUserInteractingSlider = false; }, 300);
        })
        .catch(err => {
          console.log(err);
          isUserInteractingSlider = false;
        });
    }

    function syncTrackInput() {
      document.getElementById('trackInput').value = document.getElementById('trackSelect').value;
    }

    function syncTrackSelect() {
      let val = document.getElementById('trackInput').value;
      let sel = document.getElementById('trackSelect');
      sel.value = val;
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
          if (data.volume !== undefined && !isUserInteractingSlider) {
            document.getElementById('volumeSlider').value = data.volume;
            document.getElementById('volumeLabel').innerText = data.volume + " / 30";
          }
        })
        .catch(err => console.log(err));
    }

    let sliderEl = document.getElementById('volumeSlider');
    sliderEl.addEventListener('mousedown', () => { isUserInteractingSlider = true; });
    sliderEl.addEventListener('touchstart', () => { isUserInteractingSlider = true; });
    sliderEl.addEventListener('mouseup', () => { isUserInteractingSlider = false; });
    sliderEl.addEventListener('touchend', () => { isUserInteractingSlider = false; });

    loadAudioInfo();
    pollOtaStatus();

    // Fast 200ms status poll for live volume knob & countdown tracking
    setInterval(() => {
      fetch('/status')
        .then(res => res.json())
        .then(data => {
          document.getElementById('timerDisplay').innerText = data.timer;
          document.getElementById('flapDisplay').innerText = "FLAP " + data.flap;
          if (data.volume !== undefined && !isUserInteractingSlider) {
            document.getElementById('volumeSlider').value = data.volume;
            document.getElementById('volumeLabel').innerText = data.volume + " / 30";
          }
        })
        .catch(err => console.log(err));
    }, 200);
  </script>
</body>
</html>
)rawliteral";

// Forward declarations
void goToFlap(int targetFlap, int extraRotations = 0);

// --- I2C SLAVE BROADCAST HELPERS ---
void sendSlaveFlap(uint8_t slaveAddress, uint8_t flapIndex, uint8_t speedRPM = 10) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(flapIndex);
  Wire.write(speedRPM);
  Wire.endTransmission();
}

// Convert 0-9 digit to standard Split-Flap character index (30 = '0' ... 39 = '9')
uint8_t digitToFlap(int digit) {
  if (digit >= 0 && digit <= 9) {
    return 30 + digit;
  }
  return 0; // Blank / Space
}

void updateAllDrums(int minutes, int seconds) {
  // Digit 1 (Master Local ESP32 Drum): Hundreds of minutes (Flap 1='1', Flap 0=' ')
  if (minutes >= 100) {
    goToFlap(1);
  } else {
    goToFlap(0);
  }

  // Digit 2 (Slave 1 @ 0x01): Tens of Minutes (0-9)
  int tensMinutes = (minutes % 100) / 10;
  sendSlaveFlap(1, digitToFlap(tensMinutes));

  // Digit 3 (Slave 2 @ 0x02): Ones of Minutes (0-9)
  int onesMinutes = minutes % 10;
  sendSlaveFlap(2, digitToFlap(onesMinutes));

  // Digit 4 (Slave 3 @ 0x03): Tens of Seconds (0-5)
  int tensSeconds = seconds / 10;
  sendSlaveFlap(3, digitToFlap(tensSeconds));

  // Digit 5 (Slave 4 @ 0x04): Ones of Seconds (0-9)
  int onesSeconds = seconds % 10;
  sendSlaveFlap(4, digitToFlap(onesSeconds));
}

// --- DRIFT-FREE FLAP MATH WITH EXTRA ROTATIONS ---
void goToFlap(int targetFlap, int extraRotations) {
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

// --- VERSION COMPARISON HELPER (SEMANTIC MAJOR.MINOR.PATCH) ---
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
  client.setInsecure(); // Critical: GitHub rotates SSL certificates

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
  json += "\"flap\":" + String(currentFlapPosition) + ",";
  json += "\"volume\":" + String(currentVolume);
  json += "}";

  server.send(200, "application/json", json);
}

void handleExecute() {
  currentMinutes = START_MINUTES;
  currentSeconds = 0;
  isPaused = false;
  Serial.println("\n[SYSTEM] > 4 8 15 16 23 42");
  Serial.println("[SYSTEM] > OVERRIDE ACCEPTED. RESETTING TO 108:00\n");
  
  // Have master do 2 dramatic extra full spins when reset, just like the show!
  goToFlap(1, 2); 
  
  // Reset slave drums to 0, 8, 0, 0
  sendSlaveFlap(1, digitToFlap(0));
  sendSlaveFlap(2, digitToFlap(8));
  sendSlaveFlap(3, digitToFlap(0));
  sendSlaveFlap(4, digitToFlap(0));

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
    int unit = 0;
    
    // Check if extra rotations parameter was passed
    if (server.hasArg("rot")) {
      rot = server.arg("rot").toInt();
    }
    // Check if target drum unit was passed (0 = master, 1..4 = slave)
    if (server.hasArg("unit")) {
      unit = server.arg("unit").toInt();
    }
    
    isPaused = true; 
    Serial.printf("[DEBUG] Manual override: Moving Drum %d to Flap %d (extra rot: %d)\n", unit, target, rot);
    
    if (unit == 0) {
      goToFlap(target, rot);
    } else {
      sendSlaveFlap((uint8_t)unit, (uint8_t)target);
    }
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
  if (otaRunning) {
    server.send(200, "text/plain", "Update check already in progress.");
    return;
  }
  otaLogBuffer = ""; // Reset console logs for new run
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SWAN STATION MASTER BRAIN INITIALIZING ===");

  pinMode(hallSensorPin, INPUT_PULLUP);
  pinMode(potPin, INPUT);
  analogSetPinAttenuation(potPin, ADC_11db);

  // Read initial potentiometer level
  int initialPot = analogRead(potPin);
  currentVolume = constrain(map(initialPot, 15, 4080, 0, 30), 0, 30);
  lastPotVolume = currentVolume;

  stepper.setMaxSpeed(800);
  stepper.setAcceleration(400);

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

  server.on("/", handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/execute", HTTP_GET, handleExecute);
  server.on("/togglePause", HTTP_GET, handleTogglePause);
  server.on("/setFlap", HTTP_GET, handleSetFlap);
  server.on("/setSpeed", HTTP_GET, handleSetSpeed); 
  server.on("/check-update", HTTP_GET, handleCheckUpdate);
  server.on("/ota-status", HTTP_GET, handleOtaStatus);
  server.on("/playTrack", HTTP_GET, handlePlayTrack);
  server.on("/stopAudio", HTTP_GET, handleStopAudio);
  server.on("/setVolume", HTTP_GET, handleSetVolume);
  server.on("/getAudioInfo", HTTP_GET, handleGetAudioInfo);
  server.begin();
  
  Wire.begin(21, 22);
  Wire.setClock(100000); // 100kHz standard I2C clock speed for reliable slave response

  updateAllDrums(currentMinutes, currentSeconds);
}

void loop() {
  server.handleClient();
  stepper.run(); 

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

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (!isPaused) {
      if (currentMinutes == 0 && currentSeconds == 0) {
        // Hold at zero / alert state
        goToFlap(41); 
        sendSlaveFlap(1, 41);
        sendSlaveFlap(2, 41);
        sendSlaveFlap(3, 41);
        sendSlaveFlap(4, 41);
      } else {
        if (currentSeconds == 0) {
          currentMinutes--;
          currentSeconds = 59;
        } else {
          currentSeconds--;
        }
        updateAllDrums(currentMinutes, currentSeconds);
      }
    }
  }
}
