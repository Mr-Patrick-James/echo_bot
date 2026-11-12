#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid     = ".........";
const char* password = "12345678910";

//const char* ssid     = "Nyotok";
//const char* password = "DT27dCF7";

// ── Stepper pins (A4988) ─────────────────────────────────
#define STEP_PIN  5   // D1
#define DIR_PIN   4   // D2

#define BATTERY_PIN A0

// ── Position config ───────────────────────────────────────
const int STEP_DELAY_US  = 1500;   // speed — lower = faster
const int POS_HOME       = 0;      // home — CCW fully down
const int POS_UP         = 1580;   // CW fully up
const int POS_MIN        = 0;
const int POS_MAX        = 99999;  // virtually unlimited

// ── State ─────────────────────────────────────────────────
int  currentPos    = 0;     // current step position
int  targetPos     = 0;     // target step position
bool motorRunning  = false; // continuous mode flag
bool movingToPos   = false; // position mode flag
bool motorDir      = true;  // true = CW (up), false = CCW (down)

unsigned long lastStepTime = 0;
bool stepState = LOW;

ESP8266WebServer server(80);

// ── Move one step toward target ───────────────────────────
void stepTowardTarget() {
  if (currentPos == targetPos) {
    movingToPos = false;
    return;
  }
  bool goUp = (targetPos > currentPos);
  digitalWrite(DIR_PIN, goUp ? HIGH : LOW);
  unsigned long now = micros();
  if (now - lastStepTime >= (unsigned long)STEP_DELAY_US) {
    lastStepTime = now;
    stepState = !stepState;
    digitalWrite(STEP_PIN, stepState);
    if (stepState == HIGH) {  // count on rising edge
      currentPos += goUp ? 1 : -1;
    }
  }
}

// ── Continuous step ───────────────────────────────────────
void stepContinuous() {
  digitalWrite(DIR_PIN, motorDir ? HIGH : LOW);  // HIGH = CW, LOW = CCW
  unsigned long now = micros();
  if (now - lastStepTime >= (unsigned long)STEP_DELAY_US) {
    lastStepTime = now;
    stepState = !stepState;
    digitalWrite(STEP_PIN, stepState);
    if (stepState == HIGH) {
      currentPos += motorDir ? 1 : -1;
      if (currentPos < POS_MIN) { currentPos = POS_MIN; motorRunning = false; }
    }
  }
}

// ── Battery voltage ───────────────────────────────────────
float getBatteryVoltage() {
  int v = analogRead(BATTERY_PIN);
  return ((v / 1023.0) * 3.3) * 3.65;
}

// ── Status JSON ───────────────────────────────────────────
String statusJson() {
  String j = "{";
  j += "\"pos\":"     + String(currentPos);
  j += ",\"target\":" + String(targetPos);
  j += ",\"moving\":" + String((movingToPos || motorRunning) ? "true" : "false");
  j += ",\"pct\":"    + String((int)((float)currentPos / POS_MAX * 100));
  j += ",\"batt\":"   + String(getBatteryVoltage(), 2);
  j += "}";
  return j;
}

// ═══════════════════════════════════════════════════════════
// WEB UI
// ═══════════════════════════════════════════════════════════
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no'>
<title>Trash Lifter</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; user-select: none; }
body { background: #1a1a1a; font-family: sans-serif; color: white; display: flex; flex-direction: column; align-items: center; padding: 16px; }
h2 { color: #38bdf8; font-size: 15px; margin-bottom: 12px; letter-spacing: 1px; }
.card { background: #1e293b; border: 1px solid #334155; border-radius: 16px; padding: 16px; width: 100%; max-width: 400px; margin-bottom: 12px; }
.title { font-size: 11px; font-weight: bold; letter-spacing: 1px; color: #64748b; text-transform: uppercase; margin-bottom: 12px; display: flex; justify-content: space-between; }
.pos-track { position: relative; height: 10px; background: #334155; border-radius: 999px; margin-bottom: 6px; }
.pos-thumb { position: absolute; top: 50%; width: 18px; height: 18px; background: linear-gradient(135deg,#38bdf8,#818cf8); border-radius: 50%; transform: translate(-50%,-50%); transition: left .3s ease; border: 3px solid #0f172a; box-shadow: 0 2px 6px rgba(0,0,0,.5); }
.pos-labels { display: flex; justify-content: space-between; font-size: 10px; color: #64748b; margin-bottom: 12px; }
.stat-row { display: flex; gap: 8px; margin-bottom: 12px; }
.stat { flex: 1; background: #0f172a; border-radius: 10px; padding: 10px 6px; text-align: center; }
.stat-val { font-size: 18px; font-weight: bold; background: linear-gradient(90deg,#38bdf8,#818cf8); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
.stat-lbl { font-size: 9px; color: #64748b; margin-top: 3px; }
.preset-row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 7px; margin-bottom: 10px; }
.pbtn { padding: 14px 4px; font-size: 12px; font-weight: bold; border-radius: 10px; cursor: pointer; outline: none; border: 2px solid; transition: filter .15s; text-align: center; }
.pbtn:active { filter: brightness(1.5); }
.p-home { border-color: #0369a1; background: #0c4a6e; color: #7dd3fc; }
.p-mid  { border-color: #7c3aed; background: #4c1d95; color: #c4b5fd; }
.p-up   { border-color: #15803d; background: #14532d; color: #86efac; }
.divider { border: none; border-top: 1px solid #334155; margin: 10px 0; }
.step-row { display: flex; gap: 6px; align-items: center; margin-bottom: 8px; }
.step-input { flex: 1; background: #0f172a; border: 1px solid #334155; border-radius: 8px; color: #38bdf8; font-size: 14px; padding: 8px; text-align: center; outline: none; }
.go-btn { padding: 8px 16px; background: linear-gradient(90deg,#38bdf8,#818cf8); border: none; border-radius: 8px; color: #0f172a; font-weight: bold; cursor: pointer; font-size: 13px; }
.cont-row { display: flex; gap: 7px; margin-bottom: 8px; }
.c-btn { flex: 1; padding: 12px 4px; font-size: 12px; font-weight: bold; border-radius: 10px; cursor: pointer; outline: none; border: 2px solid; transition: filter .15s; }
.c-btn:active { filter: brightness(1.5); }
.c-cw   { border-color: #0891b2; background: #164e63; color: #67e8f9; }
.c-stop { border-color: #475569; background: #1e293b; color: #94a3b8; }
.c-ccw  { border-color: #7c3aed; background: #4c1d95; color: #c4b5fd; }
.c-cw.on  { background: #0891b2; color: #fff; }
.c-ccw.on { background: #7c3aed; color: #fff; }
.home-btn { width: 100%; padding: 10px; border: 1px solid #334155; border-radius: 8px; background: #0f172a; color: #64748b; font-size: 11px; cursor: pointer; margin-top: 4px; }
.batt { color: #4ecca3; font-size: 12px; font-weight: bold; }
</style>
</head><body>
<h2>&#9883; Trash Lifter Control</h2>

<div class='card'>
  <div class='title'>
    <span>Position</span>
    <span class='batt' id='battV'>&#128267; --V</span>
  </div>
  <div class='pos-track'><div class='pos-thumb' id='thumb'></div></div>
  <div class='pos-labels'>
    <span>HOME<br>0</span>
    <span style='text-align:right'>UP<br>1580</span>
  </div>
  <div class='stat-row'>
    <div class='stat'><div class='stat-val' id='posVal'>0</div><div class='stat-lbl'>current steps</div></div>
    <div class='stat'><div class='stat-val' id='tgtVal'>0</div><div class='stat-lbl'>target steps</div></div>
    <div class='stat'><div class='stat-val' id='pctVal'>0%</div><div class='stat-lbl'>position %</div></div>
  </div>

  <div class='preset-row' style='grid-template-columns:1fr 1fr;'>
    <button class='pbtn p-home' onclick='moveTo(0)'>&#8962; HOME<br><small>0 steps (CCW)</small></button>
    <button class='pbtn p-up'   onclick='moveTo(1580)'>&#9650; UP<br><small>1580 steps (CW)</small></button>
  </div>

  <div class='divider'></div>

  <div class='step-row'>
    <input type='number' class='step-input' id='stepInput' min='0' max='99999' value='0' placeholder='steps (unlimited)'>
    <button class='go-btn' onclick='moveToInput()'>GO</button>
  </div>

  <div class='cont-row'>
    <button id='btnCW'  class='c-btn c-cw'   onclick='toggleCont("cw")'>&#8635; CW</button>
    <button           class='c-btn c-stop' onclick='stopAll()'>&#9646; STOP</button>
    <button id='btnCCW' class='c-btn c-ccw'  onclick='toggleCont("ccw")'>&#8634; CCW</button>
  </div>

  <button class='home-btn' onclick='setHome()'>&#9654; Set Current as HOME (zero position)</button>
</div>

<script>
var MAX_STEPS = 99999;
var contState = 'stop';

function moveTo(steps) {
  document.getElementById('stepInput').value = steps;
  fetch('/moveTo?steps=' + steps).then(r=>r.json()).then(updateUI);
}
function moveToInput() {
  var s = parseInt(document.getElementById('stepInput').value) || 0;
  moveTo(s);
}
function toggleCont(dir) {
  if (contState === dir) {
    contState = 'stop';
    fetch('/stop').then(r=>r.json()).then(updateUI);
    document.getElementById('btnCW').classList.remove('on');
    document.getElementById('btnCCW').classList.remove('on');
  } else {
    contState = dir;
    var url = dir === 'cw' ? '/start_cw' : '/start_ccw';
    fetch(url).then(r=>r.json()).then(updateUI);
    document.getElementById('btnCW').classList.toggle('on', dir==='cw');
    document.getElementById('btnCCW').classList.toggle('on', dir==='ccw');
  }
}
function stopAll() {
  contState = 'stop';
  fetch('/stop').then(r=>r.json()).then(updateUI);
  document.getElementById('btnCW').classList.remove('on');
  document.getElementById('btnCCW').classList.remove('on');
}
function setHome() {
  fetch('/setHome').then(r=>r.json()).then(updateUI);
}
function updateUI(d) {
  if (!d) return;
  var pct = Math.min(100, Math.max(0, (d.pos / MAX_STEPS) * 100));
  document.getElementById('thumb').style.left = pct + '%';
  document.getElementById('posVal').textContent = d.pos;
  document.getElementById('tgtVal').textContent = d.target;
  document.getElementById('pctVal').textContent = d.pct + '%';
  document.getElementById('battV').innerHTML = '&#128267; ' + d.batt + 'V';
}
// Poll status every 400ms
setInterval(function() {
  fetch('/status').then(r=>r.json()).then(updateUI).catch(function(){});
}, 400);
</script>
</body></html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ═══════════════════════════════════════════════════════════
// ROUTES
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  server.on("/", handleRoot);

  // Move to exact step position
  server.on("/moveTo", []() {
    int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 0;
    targetPos   = constrain(steps, POS_MIN, POS_MAX);
    movingToPos = (targetPos != currentPos);
    motorRunning = false;
    server.send(200, "application/json", statusJson());
  });

  // Continuous CW
  server.on("/start_cw", []() {
    motorDir     = true;
    motorRunning = true;
    movingToPos  = false;
    server.send(200, "application/json", statusJson());
  });

  // Continuous CCW
  server.on("/start_ccw", []() {
    motorDir     = false;
    motorRunning = true;
    movingToPos  = false;
    server.send(200, "application/json", statusJson());
  });

  // Stop all
  server.on("/stop", []() {
    motorRunning = false;
    movingToPos  = false;
    targetPos    = currentPos;
    server.send(200, "application/json", statusJson());
  });

  // Set current position as home (zero)
  server.on("/setHome", []() {
    currentPos   = 0;
    targetPos    = 0;
    motorRunning = false;
    movingToPos  = false;
    server.send(200, "application/json", statusJson());
  });

  // Preset positions
  server.on("/goHome", []() { targetPos = POS_HOME; movingToPos = true; motorRunning = false; server.send(200, "application/json", statusJson()); });
  server.on("/goUp",   []() { targetPos = POS_UP;   movingToPos = true; motorRunning = false; server.send(200, "application/json", statusJson()); });

  // Status + battery
  server.on("/status",  []() { server.send(200, "application/json", statusJson()); });
  server.on("/battery", []() { server.send(200, "text/plain", String(getBatteryVoltage(), 2)); });

  server.begin();
  Serial.println("Server started");

  // ── Auto move to UP position on boot ─────────────────
  targetPos   = POS_UP;
  movingToPos = true;
  Serial.println("Auto-moving to UP (1580 steps)...");
}

// ═══════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  if (movingToPos) {
    stepTowardTarget();
  } else if (motorRunning) {
    stepContinuous();
  }
}
