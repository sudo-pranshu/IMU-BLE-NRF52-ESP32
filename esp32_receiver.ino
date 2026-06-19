/*
 * ESP32 — 6-Axis IMU BLE Receiver + Live HTTP Dashboard (SSE)
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  MANDATORY: SET PARTITION SCHEME BEFORE COMPILING           ║
 * ║                                                             ║
 * ║  Arduino IDE 2.x:                                           ║
 * ║    Tools → Partition Scheme →                               ║
 * ║    "Huge APP (3MB No OTA/1MB SPIFFS)"                       ║
 * ║                                                             ║
 * ║  If "Huge APP" is not listed, your board variant may use:   ║
 * ║    "No OTA (Large APP)"  or  "Minimal SPIFFS (Large APPS)"  ║
 * ║                                                             ║
 * ║  WHY: ESP32 BLE alone compiles to ~1.1 MB. Default          ║
 * ║  partition only gives 1.28 MB for code. Huge APP gives 3MB. ║
 * ║  This is unavoidable — not a code bug.                      ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * Libraries needed — ONLY these two (no WebSockets lib needed!):
 *   • ArduinoJson  ← NOT needed anymore, removed
 *   • No extra libs beyond what ships with ESP32 Arduino core
 *
 * How the dashboard works:
 *   HTTP GET /        → serves the HTML page
 *   HTTP GET /stream  → Server-Sent Events stream (text/event-stream)
 *                       Browser's native EventSource API connects here
 *   No WebSockets library needed — SSE is built into WebServer.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <WiFi.h>
#include <WebServer.h>

// ══════════════════════════════════════════════════
//  YOUR WIFI CREDENTIALS
// ══════════════════════════════════════════════════
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
// ══════════════════════════════════════════════════

static BLEUUID serviceUUID("ABCD1234-0000-467A-9538-01F0652C74E0");
static BLEUUID charUUID   ("ABCD1234-0001-467A-9538-01F0652C74E0");

const float ACCEL_SCALE = 8192.0f;
const float GYRO_SCALE  = 65.536f;

static BLEClient*               pClient      = nullptr;
static BLERemoteCharacteristic* pChar        = nullptr;
static bool                     bleConnected = false;
static bool                     doConnect    = false;
static BLEAdvertisedDevice*     myDevice     = nullptr;

// SSE client tracking
static WiFiClient sseClient;
static bool       sseActive = false;

uint32_t totalSamples     = 0;
uint32_t lastReportMicros = 0;

WebServer httpServer(80);

// ══════════════════════════════════════════════════
//  HTML PAGE — minimal, no external JS libs
//  Uses Canvas2D for scrolling line graphs
//  Uses native EventSource for SSE
// ══════════════════════════════════════════════════
const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>IMU Live</title>
<style>
:root{--bg:#0a0e1a;--sf:#111827;--bd:#1e2d45;--tx:#c8d6e8;--mu:#4a6080}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font:13px/1.4 'JetBrains Mono',monospace;padding:12px}
h1{font-size:14px;font-weight:600;margin-bottom:4px}
.sub{font-size:10px;color:var(--mu);letter-spacing:.12em;text-transform:uppercase}
header{display:flex;justify-content:space-between;align-items:center;
  border-bottom:1px solid var(--bd);padding-bottom:12px;margin-bottom:14px}
.pill{display:flex;align-items:center;gap:7px;background:var(--sf);
  border:1px solid var(--bd);border-radius:20px;padding:4px 12px;font-size:11px;color:var(--mu)}
.dot{width:7px;height:7px;border-radius:50%;background:#333;transition:background .4s}
.dot.on{background:#51cf66;box-shadow:0 0 7px #51cf66}
.stats{display:flex;gap:12px;flex-wrap:wrap;margin-bottom:14px}
.st{background:var(--sf);border:1px solid var(--bd);border-radius:8px;padding:9px 14px;min-width:110px}
.sl{font-size:10px;color:var(--mu);text-transform:uppercase;letter-spacing:.08em}
.sv{font-size:18px;font-weight:700}
.sec{font-size:10px;letter-spacing:.15em;text-transform:uppercase;color:var(--mu);
  margin:14px 0 8px;display:flex;align-items:center;gap:8px}
.sec::after{content:'';flex:1;height:1px;background:var(--bd)}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
@media(max-width:600px){.grid{grid-template-columns:1fr}}
.card{background:var(--sf);border:1px solid var(--bd);border-radius:10px;padding:10px}
.ch{display:flex;align-items:baseline;gap:8px;margin-bottom:6px}
.al{font-size:11px;font-weight:700;letter-spacing:.1em}
.ad{font-size:10px;color:var(--mu)}
.lv{margin-left:auto;font-size:14px;font-weight:600}
canvas{display:block;width:100%;height:100px;border-radius:4px;background:#0d1420}
</style></head><body>
<header>
  <div><div class="sub">XIAO nRF52840 → ESP32</div><h1>6-Axis IMU Dashboard</h1></div>
  <div class="pill"><div class="dot" id="dot"></div><span id="st">Waiting…</span></div>
</header>
<div class="stats">
  <div class="st"><div class="sl">Rate</div><div class="sv"><span id="hz">—</span><span style="font-size:11px;color:var(--mu)"> Hz</span></div></div>
  <div class="st"><div class="sl">Samples</div><div class="sv" id="tot">0</div></div>
  <div class="st"><div class="sl">Pkts/s</div><div class="sv" id="pps">—</div></div>
</div>
<div class="sec">Accelerometer</div>
<div class="grid">
  <div class="card"><div class="ch"><span class="al" style="color:#ff6b6b">AX</span><span class="ad">X-axis</span><span class="lv" id="vax" style="color:#ff6b6b">—</span><span class="ad"> g</span></div><canvas id="cax"></canvas></div>
  <div class="card"><div class="ch"><span class="al" style="color:#ffa94d">AY</span><span class="ad">Y-axis</span><span class="lv" id="vay" style="color:#ffa94d">—</span><span class="ad"> g</span></div><canvas id="cay"></canvas></div>
  <div class="card"><div class="ch"><span class="al" style="color:#ffd43b">AZ</span><span class="ad">Z-axis</span><span class="lv" id="vaz" style="color:#ffd43b">—</span><span class="ad"> g</span></div><canvas id="caz"></canvas></div>
</div>
<div class="sec">Gyroscope</div>
<div class="grid">
  <div class="card"><div class="ch"><span class="al" style="color:#4dabf7">GX</span><span class="ad">X-axis</span><span class="lv" id="vgx" style="color:#4dabf7">—</span><span class="ad"> dps</span></div><canvas id="cgx"></canvas></div>
  <div class="card"><div class="ch"><span class="al" style="color:#74c0fc">GY</span><span class="ad">Y-axis</span><span class="lv" id="vgy" style="color:#74c0fc">—</span><span class="ad"> dps</span></div><canvas id="cgy"></canvas></div>
  <div class="card"><div class="ch"><span class="al" style="color:#a5d8ff">GZ</span><span class="ad">Z-axis</span><span class="lv" id="vgz" style="color:#a5d8ff">—</span><span class="ad"> dps</span></div><canvas id="cgz"></canvas></div>
</div>
<script>
// ── Canvas scrolling chart ────────────────────────────────
const N=300; // points in rolling window
function Scope(id,color){
  const cv=document.getElementById(id);
  const cx=cv.getContext('2d');
  const buf=new Float32Array(N);
  let head=0,mn=Infinity,mx=-Infinity,needScale=true;
  function resize(){cv.width=cv.offsetWidth*devicePixelRatio;cv.height=cv.offsetHeight*devicePixelRatio;}
  resize();
  window.addEventListener('resize',resize);
  return{
    push(v){
      buf[head%N]=v; head++;
      if(needScale||head%50===0){
        mn=Math.min(...buf);mx=Math.max(...buf);
        const pad=(mx-mn)*0.15||0.1;mn-=pad;mx+=pad;needScale=false;
      }
      const W=cv.width,H=cv.height;
      cx.clearRect(0,0,W,H);
      // grid lines
      cx.strokeStyle='rgba(255,255,255,0.04)';cx.lineWidth=1;
      for(let i=1;i<4;i++){const y=H*i/4;cx.beginPath();cx.moveTo(0,y);cx.lineTo(W,y);cx.stroke();}
      // zero line
      if(mn<0&&mx>0){
        const zy=H-(0-mn)/(mx-mn)*H;
        cx.strokeStyle='rgba(255,255,255,0.12)';cx.beginPath();cx.moveTo(0,zy);cx.lineTo(W,zy);cx.stroke();
      }
      // data
      cx.beginPath();cx.strokeStyle=color;cx.lineWidth=1.5*devicePixelRatio;
      const range=mx-mn||1;
      for(let i=0;i<N;i++){
        const idx=(head-N+i+N)%N;
        const x=i/N*W;
        const y=H-(buf[idx]-mn)/range*H;
        i===0?cx.moveTo(x,y):cx.lineTo(x,y);
      }
      cx.stroke();
      // fill under
      const lastX=W,lastIdx=(head-1+N)%N;
      const lastY=H-(buf[lastIdx]-mn)/range*H;
      cx.lineTo(lastX,H);cx.lineTo(0,H);
      cx.fillStyle=color+'1a';cx.fill();
      // y labels
      cx.fillStyle='#4a6080';cx.font=`${9*devicePixelRatio}px monospace`;
      cx.fillText(mx.toFixed(2),3*devicePixelRatio,11*devicePixelRatio);
      cx.fillText(mn.toFixed(2),3*devicePixelRatio,H-4*devicePixelRatio);
    }
  };
}

const scopes={
  ax:Scope('cax','#ff6b6b'),ay:Scope('cay','#ffa94d'),az:Scope('caz','#ffd43b'),
  gx:Scope('cgx','#4dabf7'),gy:Scope('cgy','#74c0fc'),gz:Scope('cgz','#a5d8ff')
};

// ── Stats ────────────────────────────────────────────────
let tot=0,pkts=0,smpls=0,last=Date.now();
setInterval(()=>{
  const dt=(Date.now()-last)/1000;
  document.getElementById('hz').textContent=(smpls/dt).toFixed(0);
  document.getElementById('pps').textContent=(pkts/dt).toFixed(1);
  smpls=0;pkts=0;last=Date.now();
},1000);

// ── SSE connection ───────────────────────────────────────
const dot=document.getElementById('dot'),stEl=document.getElementById('st');
const es=new EventSource('/stream');
es.onopen=()=>{dot.classList.add('on');stEl.textContent='Live';}
es.onerror=()=>{dot.classList.remove('on');stEl.textContent='Reconnecting…';}
es.onmessage=e=>{
  // format: "ax,ay,az,gx,gy,gz|ax,ay,az,gx,gy,gz|..."
  const rows=e.data.split('|');
  for(const r of rows){
    if(!r)continue;
    const v=r.split(',');
    if(v.length<6)continue;
    const [ax,ay,az,gx,gy,gz]=v.map(Number);
    scopes.ax.push(ax);scopes.ay.push(ay);scopes.az.push(az);
    scopes.gx.push(gx);scopes.gy.push(gy);scopes.gz.push(gz);
    document.getElementById('vax').textContent=ax.toFixed(3);
    document.getElementById('vay').textContent=ay.toFixed(3);
    document.getElementById('vaz').textContent=az.toFixed(3);
    document.getElementById('vgx').textContent=gx.toFixed(1);
    document.getElementById('vgy').textContent=gy.toFixed(1);
    document.getElementById('vgz').textContent=gz.toFixed(1);
    tot++;smpls++;
  }
  pkts++;
  document.getElementById('tot').textContent=tot.toLocaleString();
};
</script></body></html>
)HTML";

// ══════════════════════════════════════════════════
//  SSE helpers
// ══════════════════════════════════════════════════

// Queue of SSE lines waiting to be flushed (ring buffer)
#define SSE_BUF_LINES 8
static char  sseBuf[SSE_BUF_LINES][128];
static uint8_t sseWrite = 0, sseRead = 0;

// Called from BLE callback (different task) — just enqueue
void enqueueSSE(const char* payload) {
  uint8_t next = (sseWrite + 1) % SSE_BUF_LINES;
  if (next == sseRead) return; // drop if full
  snprintf(sseBuf[sseWrite], 128, "%s", payload);
  sseWrite = next;
}

// Called from loop() — flush queued lines to SSE client
void flushSSE() {
  if (!sseActive || !sseClient.connected()) { sseActive = false; return; }
  while (sseRead != sseWrite) {
    sseClient.print("data: ");
    sseClient.print(sseBuf[sseRead]);
    sseClient.print("\n\n");
    sseRead = (sseRead + 1) % SSE_BUF_LINES;
  }
}

// ══════════════════════════════════════════════════
//  BLE notify callback
// ══════════════════════════════════════════════════
static char pktBuf[512];

void notifyCallback(BLERemoteCharacteristic* pC,
                    uint8_t* pData, size_t length, bool)
{
  uint16_t n = length / 12;
  if (!n) return;

  // Build "ax,ay,az,gx,gy,gz|..." string
  int pos = 0;
  for (uint16_t i = 0; i < n && pos < (int)sizeof(pktBuf)-30; i++) {
    int16_t ar[6];
    memcpy(ar, pData + i * 12, 12);
    float ax=ar[0]/ACCEL_SCALE, ay=ar[1]/ACCEL_SCALE, az=ar[2]/ACCEL_SCALE;
    float gx=ar[3]/GYRO_SCALE,  gy=ar[4]/GYRO_SCALE,  gz=ar[5]/GYRO_SCALE;
    pos += snprintf(pktBuf+pos, sizeof(pktBuf)-pos,
                    "%.3f,%.3f,%.3f,%.1f,%.1f,%.1f|",
                    ax,ay,az,gx,gy,gz);
  }
  pktBuf[pos] = '\0';
  enqueueSSE(pktBuf);

  totalSamples += n;
  uint32_t now = micros();
  if (now - lastReportMicros >= 2000000) {
    Serial.printf("%.0f sps\n", totalSamples / ((now-lastReportMicros)/1e6f));
    totalSamples = 0; lastReportMicros = now;
  }
}

// ══════════════════════════════════════════════════
//  BLE scan callback
// ══════════════════════════════════════════════════
class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    if (d.isAdvertisingService(serviceUUID)) {
      d.getScan()->stop();
      myDevice = new BLEAdvertisedDevice(d);
      doConnect = true;
      Serial.println("Found IMU_6AXIS");
    }
  }
};

bool connectBLE() {
  pClient = BLEDevice::createClient();
  if (!pClient->connect(myDevice)) return false;
  pClient->setMTU(247);
  BLERemoteService* svc = pClient->getService(serviceUUID);
  if (!svc) { pClient->disconnect(); return false; }
  pChar = svc->getCharacteristic(charUUID);
  if (!pChar) { pClient->disconnect(); return false; }
  if (pChar->canNotify()) {
    pChar->registerForNotify(notifyCallback);
    auto* cccd = pChar->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (cccd) { uint8_t v[]={1,0}; cccd->writeValue(v,2,true); }
  }
  bleConnected = true;
  Serial.println("BLE connected + notifications ON");
  return true;
}

// ══════════════════════════════════════════════════
//  HTTP handlers
// ══════════════════════════════════════════════════
void handleRoot() {
  httpServer.send_P(200, "text/html", HTML);
}

void handleStream() {
  // If another client is already connected, drop the old one
  if (sseActive) sseClient.stop();

  sseClient = httpServer.client();
  sseActive = true;

  // Send SSE headers
  sseClient.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/event-stream\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: keep-alive\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "\r\n"
  );
  sseClient.flush();
  // Don't call httpServer.send() — we've taken over the client directly
}

// ══════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 IMU Receiver v3 (SSE, no WebSockets lib)");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.printf("\n\nDashboard → http://%s\n\n", WiFi.localIP().toString().c_str());

  httpServer.on("/",       HTTP_GET, handleRoot);
  httpServer.on("/stream", HTTP_GET, handleStream);
  httpServer.begin();

  BLEDevice::init("IMU_Recv");
  BLEDevice::setMTU(247);
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCB());
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(10, false);

  lastReportMicros = micros();
}

// ══════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════
void loop() {
  if (doConnect) { connectBLE(); doConnect = false; }
  if (bleConnected && !pClient->isConnected()) {
    bleConnected = false;
    Serial.println("BLE lost — rescanning");
    BLEDevice::getScan()->start(5, false);
  }
  httpServer.handleClient();
  flushSSE();
}
