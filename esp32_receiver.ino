/*
 * ESP32 — 6-Axis IMU BLE Receiver + WebSocket Dashboard Server
 *
 * Receives BLE notify packets from XIAO nRF52840 (20 samples × 12 bytes)
 * Decodes ax, ay, az (g) and gx, gy, gz (dps)
 * Serves a live 6-graph HTML dashboard over WiFi via WebSocket
 *
 * Libraries needed (Arduino Library Manager):
 *   - "ArduinoJson" by Benoit Blanchon
 *   - "WebSockets" by Markus Sattler  (or ESPAsyncWebServer + AsyncWebSocket)
 *   - ESP32 Arduino core (espressif/arduino-esp32)
 *
 * Board: "ESP32 Dev Module" (or your specific ESP32 board)
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>    // from "WebSockets" library by Markus Sattler
#include <ArduinoJson.h>

// ════════════════════════════════════════════════════════════
//  CONFIGURE THESE
// ════════════════════════════════════════════════════════════
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
// ════════════════════════════════════════════════════════════

// ── BLE UUIDs (must match sender) ────────────────────────────
static BLEUUID serviceUUID("ABCD1234-0000-467A-9538-01F0652C74E0");
static BLEUUID charUUID   ("ABCD1234-0001-467A-9538-01F0652C74E0");

// ── Scale factors (must match sender) ────────────────────────
const float ACCEL_SCALE = 8192.0f;
const float GYRO_SCALE  = 65.536f;

// ── BLE state ─────────────────────────────────────────────────
static BLEClient*            pClient    = nullptr;
static BLERemoteCharacteristic* pChar   = nullptr;
static bool                  bleConnected = false;
static bool                  doConnect    = false;
static BLEAdvertisedDevice*  myDevice     = nullptr;

// ── Stats ─────────────────────────────────────────────────────
volatile uint32_t totalSamples    = 0;
volatile uint32_t lastReportMicros = 0;
volatile float    currentRate     = 0.0f;

// ── WebSocket / HTTP ──────────────────────────────────────────
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ── Forward declarations ──────────────────────────────────────
void sendToWebSocket(float ax, float ay, float az,
                     float gx, float gy, float gz);

// ════════════════════════════════════════════════════════════
//  HTML DASHBOARD (stored in PROGMEM to save RAM)
// ════════════════════════════════════════════════════════════
const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IMU Live Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg:       #0a0e1a;
    --surface:  #111827;
    --border:   #1e2d45;
    --text:     #c8d6e8;
    --muted:    #4a6080;
    --ax: #ff6b6b; --ay: #ffa94d; --az: #ffd43b;
    --gx: #4dabf7; --gy: #74c0fc; --gz: #a5d8ff;
    --grid:     rgba(255,255,255,0.04);
    --font: 'JetBrains Mono', 'Fira Code', monospace;
  }
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--font);
    font-size: 13px;
    min-height: 100vh;
    padding: 16px;
  }
  header {
    display: flex;
    align-items: center;
    gap: 16px;
    margin-bottom: 20px;
    border-bottom: 1px solid var(--border);
    padding-bottom: 14px;
  }
  .logo {
    font-size: 11px;
    letter-spacing: 0.18em;
    text-transform: uppercase;
    color: var(--muted);
  }
  h1 {
    font-size: 15px;
    font-weight: 600;
    letter-spacing: 0.05em;
    color: var(--text);
  }
  .pill {
    margin-left: auto;
    display: flex;
    align-items: center;
    gap: 8px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 20px;
    padding: 4px 12px;
    font-size: 11px;
    color: var(--muted);
  }
  .dot {
    width: 7px; height: 7px;
    border-radius: 50%;
    background: #2d3748;
    transition: background 0.3s;
  }
  .dot.live { background: #51cf66; box-shadow: 0 0 6px #51cf66; }
  .stats {
    display: flex;
    gap: 24px;
    margin-bottom: 18px;
    flex-wrap: wrap;
  }
  .stat {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 16px;
    min-width: 130px;
  }
  .stat-label { font-size: 10px; color: var(--muted); letter-spacing: 0.1em; text-transform: uppercase; margin-bottom: 4px; }
  .stat-value { font-size: 20px; font-weight: 700; color: var(--text); }
  .stat-unit  { font-size: 11px; color: var(--muted); margin-left: 3px; }

  /* 2-column grid on wider screens */
  .grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 14px;
  }
  @media (max-width: 700px) {
    .grid { grid-template-columns: 1fr; }
  }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 14px;
    position: relative;
  }
  .card-header {
    display: flex;
    align-items: baseline;
    gap: 10px;
    margin-bottom: 10px;
  }
  .axis-label {
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.12em;
    text-transform: uppercase;
  }
  .axis-desc { font-size: 10px; color: var(--muted); }
  .live-val {
    margin-left: auto;
    font-size: 14px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
  }
  canvas { display: block; width: 100% !important; height: 120px !important; }

  /* section divider */
  .section-title {
    font-size: 10px;
    letter-spacing: 0.15em;
    text-transform: uppercase;
    color: var(--muted);
    margin: 18px 0 10px;
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .section-title::after {
    content: '';
    flex: 1;
    height: 1px;
    background: var(--border);
  }
</style>
</head>
<body>
<header>
  <div>
    <div class="logo">SEEED XIAO nRF52840 ▸ ESP32</div>
    <h1>6-Axis IMU Live Dashboard</h1>
  </div>
  <div class="pill">
    <div class="dot" id="statusDot"></div>
    <span id="statusText">Connecting…</span>
  </div>
</header>

<div class="stats">
  <div class="stat">
    <div class="stat-label">Sample Rate</div>
    <div class="stat-value" id="rateVal">—</div><span class="stat-unit">Hz</span>
  </div>
  <div class="stat">
    <div class="stat-label">Total Samples</div>
    <div class="stat-value" id="totalVal">0</div>
  </div>
  <div class="stat">
    <div class="stat-label">Packets/s</div>
    <div class="stat-value" id="pktsVal">—</div>
  </div>
</div>

<div class="section-title">Accelerometer</div>
<div class="grid" id="accelGrid">
  <div class="card">
    <div class="card-header">
      <span class="axis-label" style="color:var(--ax)">AX</span>
      <span class="axis-desc">X-axis acceleration</span>
      <span class="live-val" id="val-ax" style="color:var(--ax)">—</span>
      <span class="axis-desc">g</span>
    </div>
    <canvas id="chart-ax"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="axis-label" style="color:var(--ay)">AY</span>
      <span class="axis-desc">Y-axis acceleration</span>
      <span class="live-val" id="val-ay" style="color:var(--ay)">—</span>
      <span class="axis-desc">g</span>
    </div>
    <canvas id="chart-ay"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="axis-label" style="color:var(--az)">AZ</span>
      <span class="axis-desc">Z-axis acceleration</span>
      <span class="live-val" id="val-az" style="color:var(--az)">—</span>
      <span class="axis-desc">g</span>
    </div>
    <canvas id="chart-az"></canvas>
  </div>
</div>

<div class="section-title">Gyroscope</div>
<div class="grid" id="gyroGrid">
  <div class="card">
    <div class="card-header">
      <span class="axis-label" style="color:var(--gx)">GX</span>
      <span class="axis-desc">X-axis angular rate</span>
      <span class="live-val" id="val-gx" style="color:var(--gx)">—</span>
      <span class="axis-desc">dps</span>
    </div>
    <canvas id="chart-gx"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="axis-label" style="color:var(--gy)">GY</span>
      <span class="axis-desc">Y-axis angular rate</span>
      <span class="live-val" id="val-gy" style="color:var(--gy)">—</span>
      <span class="axis-desc">dps</span>
    </div>
    <canvas id="chart-gy"></canvas>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="axis-label" style="color:var(--gz)">GZ</span>
      <span class="axis-desc">Z-axis angular rate</span>
      <span class="live-val" id="val-gz" style="color:var(--gz)">—</span>
      <span class="axis-desc">dps</span>
    </div>
    <canvas id="chart-gz"></canvas>
  </div>
</div>

<script>
// ── Config ────────────────────────────────────────────────────
const MAX_POINTS   = 500;   // rolling window per chart
const CHART_LABELS = Array.from({length: MAX_POINTS}, () => '');

// ── Chart factory ─────────────────────────────────────────────
function makeChart(id, color) {
  const ctx = document.getElementById(id).getContext('2d');
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [...CHART_LABELS],
      datasets: [{
        data: new Array(MAX_POINTS).fill(null),
        borderColor: color,
        borderWidth: 1.2,
        pointRadius: 0,
        tension: 0,
        fill: {
          target: 'origin',
          above: color + '12',
          below: color + '12',
        },
      }]
    },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      plugins: { legend: { display: false }, tooltip: { enabled: false } },
      scales: {
        x: {
          display: false,
          ticks: { display: false },
          grid: { display: false },
          border: { display: false },
        },
        y: {
          display: true,
          grid: { color: 'rgba(255,255,255,0.04)', lineWidth: 1 },
          border: { display: false, dash: [3,3] },
          ticks: {
            color: '#4a6080',
            font: { family: 'monospace', size: 10 },
            maxTicksLimit: 5,
            padding: 6,
          }
        }
      }
    }
  });
}

const CSS = getComputedStyle(document.documentElement);
const clr = k => CSS.getPropertyValue(k).trim();

const charts = {
  ax: makeChart('chart-ax', clr('--ax')),
  ay: makeChart('chart-ay', clr('--ay')),
  az: makeChart('chart-az', clr('--az')),
  gx: makeChart('chart-gx', clr('--gx')),
  gy: makeChart('chart-gy', clr('--gy')),
  gz: makeChart('chart-gz', clr('--gz')),
};

// ── Push a value into a chart ─────────────────────────────────
function pushValue(chart, v) {
  const d = chart.data.datasets[0].data;
  d.push(v);
  if (d.length > MAX_POINTS) d.shift();
  chart.update('none');
}

// ── Stats counters ────────────────────────────────────────────
let totalSamples = 0;
let packetCount  = 0;
let lastRateTime = Date.now();

function updateStats(n) {
  totalSamples += n;
  packetCount++;
  document.getElementById('totalVal').textContent = totalSamples.toLocaleString();
  const now = Date.now();
  if (now - lastRateTime >= 1000) {
    const dt = (now - lastRateTime) / 1000;
    document.getElementById('rateVal').textContent = (totalSamples / dt).toFixed(0);
    document.getElementById('pktsVal').textContent = (packetCount / dt).toFixed(1);
    totalSamples = 0;
    packetCount  = 0;
    lastRateTime = now;
  }
}

// ── WebSocket ─────────────────────────────────────────────────
const dot      = document.getElementById('statusDot');
const statusTx = document.getElementById('statusText');
const WS_PORT  = 81;
const ws = new WebSocket(`ws://${location.hostname}:${WS_PORT}/`);

ws.onopen = () => {
  dot.classList.add('live');
  statusTx.textContent = 'Live';
};
ws.onclose = () => {
  dot.classList.remove('live');
  statusTx.textContent = 'Disconnected';
};
ws.onmessage = (e) => {
  let msg;
  try { msg = JSON.parse(e.data); } catch { return; }

  if (msg.type === 'batch') {
    const samples = msg.samples;          // array of {ax,ay,az,gx,gy,gz}
    for (const s of samples) {
      pushValue(charts.ax, s.ax);
      pushValue(charts.ay, s.ay);
      pushValue(charts.az, s.az);
      pushValue(charts.gx, s.gx);
      pushValue(charts.gy, s.gy);
      pushValue(charts.gz, s.gz);
      document.getElementById('val-ax').textContent = s.ax.toFixed(3);
      document.getElementById('val-ay').textContent = s.ay.toFixed(3);
      document.getElementById('val-az').textContent = s.az.toFixed(3);
      document.getElementById('val-gx').textContent = s.gx.toFixed(1);
      document.getElementById('val-gy').textContent = s.gy.toFixed(1);
      document.getElementById('val-gz').textContent = s.gz.toFixed(1);
    }
    updateStats(samples.length);
  }
};
</script>
</body>
</html>
)rawhtml";

// ════════════════════════════════════════════════════════════
//  BLE CALLBACK
// ════════════════════════════════════════════════════════════
void notifyCallback(BLERemoteCharacteristic* pC,
                    uint8_t* pData, size_t length, bool isNotify)
{
  // Each sample = 6 × int16 = 12 bytes
  uint16_t numSamples = length / 12;
  if (numSamples == 0) return;

  // Build JSON batch for WebSocket
  // Format: {"type":"batch","samples":[{ax,ay,az,gx,gy,gz},...]}
  // Use StaticJsonDocument sized for 20 samples (generous)
  StaticJsonDocument<4096> doc;
  doc["type"] = "batch";
  JsonArray arr = doc.createNestedArray("samples");

  for (uint16_t i = 0; i < numSamples; i++) {
    int16_t axR, ayR, azR, gxR, gyR, gzR;
    memcpy(&axR, pData + i * 12 + 0,  2);
    memcpy(&ayR, pData + i * 12 + 2,  2);
    memcpy(&azR, pData + i * 12 + 4,  2);
    memcpy(&gxR, pData + i * 12 + 6,  2);
    memcpy(&gyR, pData + i * 12 + 8,  2);
    memcpy(&gzR, pData + i * 12 + 10, 2);

    JsonObject s = arr.createNestedObject();
    s["ax"] = roundf((axR / ACCEL_SCALE) * 1000) / 1000.0f;
    s["ay"] = roundf((ayR / ACCEL_SCALE) * 1000) / 1000.0f;
    s["az"] = roundf((azR / ACCEL_SCALE) * 1000) / 1000.0f;
    s["gx"] = roundf((gxR / GYRO_SCALE)  * 10)   / 10.0f;
    s["gy"] = roundf((gyR / GYRO_SCALE)  * 10)   / 10.0f;
    s["gz"] = roundf((gzR / GYRO_SCALE)  * 10)   / 10.0f;
  }

  // Serialize and broadcast to all connected WebSocket clients
  String json;
  serializeJson(doc, json);
  wsServer.broadcastTXT(json);

  // Serial stats
  totalSamples += numSamples;
  uint32_t now = micros();
  if (now - lastReportMicros >= 1000000) {
    float dt   = (now - lastReportMicros) / 1e6f;
    currentRate = totalSamples / dt;
    Serial.printf("BLE rate: %.1f samples/s\n", currentRate);
    totalSamples    = 0;
    lastReportMicros = now;
  }
}

// ════════════════════════════════════════════════════════════
//  BLE SCAN CALLBACK
// ════════════════════════════════════════════════════════════
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.println("Found IMU device!");
      advertisedDevice.getScan()->stop();
      myDevice  = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

bool connectToServer() {
  Serial.printf("Connecting to %s\n", myDevice->getAddress().toString().c_str());
  pClient = BLEDevice::createClient();

  if (!pClient->connect(myDevice)) {
    Serial.println("BLE connect failed");
    return false;
  }
  Serial.println("BLE connected");
  pClient->setMTU(247);

  BLERemoteService* pService = pClient->getService(serviceUUID);
  if (!pService) {
    Serial.println("Service not found");
    pClient->disconnect();
    return false;
  }

  pChar = pService->getCharacteristic(charUUID);
  if (!pChar) {
    Serial.println("Characteristic not found");
    pClient->disconnect();
    return false;
  }

  if (pChar->canNotify()) {
    pChar->registerForNotify(notifyCallback);
    // Write 0x0001 to CCCD to enable notifications
    BLERemoteDescriptor* cccd = pChar->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (cccd) {
      uint8_t val[] = {0x01, 0x00};
      cccd->writeValue(val, 2, true);
    }
    Serial.println("Notifications enabled");
  }

  bleConnected = true;
  return true;
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 IMU Receiver + Dashboard");

  // ── WiFi ──────────────────────────────────────────────────
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nIP: http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("WebSocket: ws://%s:81\n", WiFi.localIP().toString().c_str());

  // ── HTTP server ───────────────────────────────────────────
  httpServer.on("/", HTTP_GET, []() {
    httpServer.send_P(200, "text/html", HTML_PAGE);
  });
  httpServer.begin();

  // ── WebSocket server ──────────────────────────────────────
  wsServer.begin();
  // No callback needed — we only broadcast, not receive

  // ── BLE ───────────────────────────────────────────────────
  BLEDevice::init("IMU_Receiver");
  BLEDevice::setMTU(247);

  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->start(10, false);   // 10 s scan; callback triggers connect

  lastReportMicros = micros();
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  // Handle pending BLE connection
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Ready — streaming to dashboard");
    } else {
      Serial.println("Retrying BLE scan...");
      BLEDevice::getScan()->start(5, false);
    }
    doConnect = false;
  }

  // Check if BLE connection dropped
  if (bleConnected && !pClient->isConnected()) {
    Serial.println("BLE disconnected — rescanning");
    bleConnected = false;
    BLEDevice::getScan()->start(5, false);
  }

  httpServer.handleClient();
  wsServer.loop();
}
