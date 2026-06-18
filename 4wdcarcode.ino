/*
 * Function：
 *  - Clockwise (wallFollow)
 *  - Servo_attack (attack)
 *  - PID Speed Control
 *  - Vive Navigation（Use tx/ty us as coordination）
 *  - Web：
 *      · Mode Switch：WEB / NAV / WALL / STOP
 *      · WEB mode changes speed, turn, pid manually
 *      · Keyboard Control:
 *          W / ↑ : SPEED +50
 *          S / ↓ : SPEED -50
 *          A / ← : TURN  -50（左）
 *          D / → : TURN  +50（右）
 *          Space : SPEED = 0
 *      · Set Navigation Target navTargetX/navTargetY (Vive tx/ty)
 *      · Show Vive Position / Yaw / Target / Mode
 *      · ATTACK Button
 */

#include <Wire.h>
#include <VL53L1X.h>
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "vive510.h"

#include <WiFi.h>
#include <WebServer.h>

// ========== Define Pins ==========
// Motors
#define MOTOR_FL_RPWM  4  //Front Left Forward
#define MOTOR_FL_LPWM  5  //Front Left Backward
#define MOTOR_FR_RPWM  6  //Front Right Forward
#define MOTOR_FR_LPWM  7  //Front Right Backward
#define MOTOR_BL_RPWM  1  //Back Left Forward
#define MOTOR_BL_LPWM  2  //Back Left Backward
#define MOTOR_BR_RPWM  10 //Back Right Forward
#define MOTOR_BR_LPWM  11 //Back Right Backward

// Encoders
#define ENC_FL_A  12
#define ENC_FL_B  13
#define ENC_FR_A  14
#define ENC_FR_B  15
#define ENC_BL_A  16
#define ENC_BL_B  17
#define ENC_BR_A  38
#define ENC_BR_B  48

// TOF
#define TOF_SDA  8
#define TOF_SCL  9
#define TOF_FRONT_SHUT  18
#define TOF_LEFT_SHUT   37

// Servo
#define SERVO_PIN  47

// Vive
#define SIGNALPINL 35
#define SIGNALPINR 36

// TopHat Address
#define TOPHAT_ADDR 0x28

// Last time data got from TopHat
uint8_t tophatLastByte = 0;
bool    tophatOk       = false;
volatile uint8_t wifiPktCountWindow = 0;

// Define: how long communicate with tophat(500ms)
unsigned long lastTophatUpdate = 0;
const unsigned long TOPHAT_PERIOD_MS = 500;   // 500ms（2Hz）

// ========== WiFi & Web ==========
const char* AP_SSID = "WeCar6";
const char* AP_PSWD = "12345678";
WebServer server(80);

// ========== Web HTML ==========
const char* INDEX_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>MEAM510 Car Control</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 10px; }
    h1 { margin-top: 0.2em; }
    h2 { margin: 0.2em 0 0.4em 0; }
    .row { margin: 4px 0; }
    label { display:inline-block; width:110px; }
    input { width:80px; }
    .card {
      border:1px solid #ccc;
      padding:8px;
      margin-bottom:8px;
      border-radius:4px;
    }
    pre { font-size:12px; }
    button { margin-left:4px; }
    .grid2 { display:grid; grid-template-columns:1fr 1fr; gap:4px 16px; }
    .mode-btn {
      margin:2px;
      padding:4px 8px;
      border-radius:4px;
      border:1px solid #888;
      cursor:pointer;
    }
    .mode-btn.active {
      background:#0078ff;
      color:#fff;
      border-color:#0050aa;
    }
    .pill {
      display:inline-block;
      padding:2px 8px;
      border-radius:999px;
      font-size:11px;
      border:1px solid #ccc;
      margin-left:4px;
    }
    .pill.green { background:#d4f5d4; border-color:#6abf69; }
    .pill.red   { background:#ffd6d6; border-color:#ff6b6b; }
    .kbd {
      display:inline-block;
      padding:2px 5px;
      border-radius:3px;
      border:1px solid #aaa;
      font-size:11px;
      margin-right:2px;
      background:#f8f8f8;
    }
  </style>
</head>
<body>
  <h1>MEAM510 Car Web Control</h1>

  <!-- Mode & Navigation Control -->
  <div class="card">
    <h2>Mode & Navigation</h2>
    <div class="row">
      <span>Mode:</span>
      <button id="btnModeWeb"  class="mode-btn" onclick="setMode(0)">WEB</button>
      <button id="btnModeNav"  class="mode-btn" onclick="setMode(1)">NAV (Vive)</button>
      <button id="btnModeWall" class="mode-btn" onclick="setMode(2)">WALL</button>
      <button id="btnModeStop" class="mode-btn" onclick="setMode(3)">STOP</button>
      <span id="modeText" class="pill">-</span>
    </div>
    <div class="row">
      <label>Target X (us):</label>
      <input id="navX" type="number" value="4000" step="10">
    </div>
    <div class="row">
      <label>Target Y (us):</label>
      <input id="navY" type="number" value="2000" step="10">
      <button onclick="applyNav()">Set Target</button>
    </div>
    <div class="row">
      <button onclick="doAttack()">ATTACK</button>
    </div>
    <div class = "row">
      <button onclick="goPoint(1)">Blue Base</button>
      <button onclick="goPoint(2)">Low Tower Red Button</button>
      <button onclick="goPoint(3)">Red Base</button>
      <button onclick="goPoint(4)">High Tower Red Button From Blue Base</button>
      <button onclick="goPoint(5)">High Tower Red Button From Red Base</button>
      <button onclick="goPoint(6)">Red Base Attack Multiple Times</button>
    </div>
    <div class = "row">
      <button onclick="updateStatus()">Refresh Status</button>
    </div>
  </div>

  <!-- Speed & PID + Keyboard Note -->
  <div class="card">
    <h2>Manual Control & PID (WEB Mode + Keyboard)</h2>
    <div class="row">
      <label>Target Speed:</label>
      <input id="speed" type="number" value="300" step="10">
      <span>counts/s</span>
    </div>
    <div class="row">
      <label></label>
      <button onclick="adjustSpeed(50)">Speed +50</button>
      <button onclick="adjustSpeed(-50)">Speed -50</button>
    </div>
    <div class="row">
      <label>Turn:</label>
      <input id="turn" type="number" value="0" step="10">
      <span>+Turn right / -Turn left</span>
    </div>
    <div class="row">
      <label></label>
      <button onclick="adjustTurn(50)">Turn +50</button>
      <button onclick="adjustTurn(-50)">Turn -50</button>
    </div>

    <div class="grid2" style="margin-top:6px;">
      <div>
        <strong>FL</strong><br>
        <label>kp_FL:</label><input id="kpFL" type="number" value="0.8" step="0.01"><br>
        <label>ki_FL:</label><input id="kiFL" type="number" value="0.005" step="0.001">
      </div>
      <div>
        <strong>FR</strong><br>
        <label>kp_FR:</label><input id="kpFR" type="number" value="1.0" step="0.01"><br>
        <label>ki_FR:</label><input id="kiFR" type="number" value="0.005" step="0.001">
      </div>
      <div>
        <strong>BL</strong><br>
        <label>kp_BL:</label><input id="kpBL" type="number" value="1.0" step="0.01"><br>
        <label>ki_BL:</label><input id="kiBL" type="number" value="0.005" step="0.001">
      </div>
      <div>
        <strong>BR</strong><br>
        <label>kp_BR:</label><input id="kpBR" type="number" value="1.0" step="0.01"><br>
        <label>ki_BR:</label><input id="kiBR" type="number" value="0.005" step="0.001">
      </div>
    </div>
    <div class="row" style="margin-top:6px;">
      <button onclick="apply()">Apply</button>
      <button onclick="stopCar()">STOP (Speed=0)</button>
    </div>
    <div class="row" style="font-size:12px; margin-top:4px;">
      <strong>Keyboard(Need Mode=WEB and focus on this page):</strong><br>
      <span class="kbd">W</span>/<span class="kbd">↑</span> Speed +50,
      <span class="kbd">S</span>/<span class="kbd">↓</span> Speed -50,
      <span class="kbd">A</span>/<span class="kbd">←</span> Turn -50,
      <span class="kbd">D</span>/<span class="kbd">→</span> Turn +50,
      <span class="kbd">Space</span> Speed=0
    </div>
  </div>

  <!-- Sensor & Vive Status -->
  <div class="card">
    <h2>Sensors & Vive</h2>
    <div class="row">Front TOF: <span id="distFront">-</span> mm</div>
    <div class="row">Left TOF:  <span id="distLeft">-</span> mm</div>
    <div class="row">
      Vive Pos: X=<span id="viveX">-</span> us,
      Y=<span id="viveY">-</span> us,
      Yaw=<span id="viveYaw">-</span>°
      <span id="viveValid" class="pill">No Vive</span>
    </div>
    <div class="row">
      Nav Target: X=<span id="navXLabel">-</span> us,
      Y=<span id="navYLabel">-</span> us
    </div>
  </div>

  <!-- Motor -->
  <div class="card">
    <h2>Motors</h2>
    <pre id="motors">Loading...</pre>
  </div>

  <script>
    let currentMode = -1;

    function setModeButtons(mode) {
      currentMode = mode;
      const ids = ["btnModeWeb","btnModeNav","btnModeWall","btnModeStop"];
      for (let i=0;i<ids.length;i++){
        const btn = document.getElementById(ids[i]);
        if (!btn) continue;
        if (i === mode) btn.classList.add("active");
        else btn.classList.remove("active");
      }
      const modeText = document.getElementById("modeText");
      const labels = ["WEB","NAV","WALL","STOP"];
      if (mode >=0 && mode < labels.length) {
        modeText.textContent = labels[mode];
      } else {
        modeText.textContent = "-";
      }
    }

    function goPoint(id) {
      fetch("/navPoint?id=" + id)
        .then(r => r.text())
        .then(txt => console.log("navPoint:", txt));
    }

    function apply() {
      const params = new URLSearchParams();
      params.append("speed", document.getElementById("speed").value);
      params.append("turn",  document.getElementById("turn").value);

      params.append("kpFL", document.getElementById("kpFL").value);
      params.append("kiFL", document.getElementById("kiFL").value);
      params.append("kpFR", document.getElementById("kpFR").value);
      params.append("kiFR", document.getElementById("kiFR").value);
      params.append("kpBL", document.getElementById("kpBL").value);
      params.append("kiBL", document.getElementById("kiBL").value);
      params.append("kpBR", document.getElementById("kpBR").value);
      params.append("kiBR", document.getElementById("kiBR").value);

      fetch("/set?" + params.toString()).then(r => r.text()).then(console.log);
    }

    function sendSpeedTurnOnly() {
      const params = new URLSearchParams();
      params.append("speed", document.getElementById("speed").value);
      params.append("turn",  document.getElementById("turn").value);
      fetch("/set?" + params.toString()).then(r => r.text()).then(console.log);
    }

    function adjustSpeed(delta) {
      const speedInput = document.getElementById("speed");
      let val = parseInt(speedInput.value) || 0;
      val += delta;
      speedInput.value = val;
      sendSpeedTurnOnly();
    }

    function adjustTurn(delta) {
      const turnInput = document.getElementById("turn");
      let val = parseInt(turnInput.value) || 0;
      val += delta;
      turnInput.value = val;
      sendSpeedTurnOnly();
    }

    function stopCar() {
      document.getElementById("speed").value = 0;
      document.getElementById("turn").value  = 0;
      sendSpeedTurnOnly();
    }

    function setMode(mode) {
      const params = new URLSearchParams();
      params.append("mode", mode);
      fetch("/set?" + params.toString())
        .then(r => r.text())
        .then(_ => setModeButtons(mode));
    }

    function applyNav() {
      const nx = document.getElementById("navX").value;
      const ny = document.getElementById("navY").value;
      const params = new URLSearchParams();
      params.append("navX", nx);
      params.append("navY", ny);
      params.append("mode", 1); // Switch to NAV Mode
      fetch("/set?" + params.toString())
        .then(r => r.text())
        .then(_ => {
          setModeButtons(1);
        });
    }

    function doAttack() {
      fetch("/attack")
        .then(r => r.text())
        .then(txt => console.log("attack:", txt));
    }

    function updateStatus() {
      fetch("/status")
        .then(r => r.json())
        .then(data => {
          const speedInput = document.getElementById("speed");
          const turnInput  = document.getElementById("turn");
          const kpFlInput  = document.getElementById("kpFL");
          const kiFLInput  = document.getElementById("kiFL");
          const kpFRInput  = document.getElementById("kpFR");
          const kiFRInput  = document.getElementById("kiFR");
          const kpBLInput  = document.getElementById("kpBL");
          const kiBLInput  = document.getElementById("kiBL");
          const kpBRInput  = document.getElementById("kpBR");
          const kiBRInput  = document.getElementById("kiBR");

          document.getElementById("distFront").innerText = data.distFront;
          document.getElementById("distLeft").innerText  = data.distLeft;

          if (document.activeElement != kpFlInput)  kpFlInput.value  = data.kpFL.toFixed(3);
          if (document.activeElement != kiFLInput) kiFLInput.value  = data.kiFL.toFixed(4);
          if (document.activeElement != kpFRInput)  kpFRInput.value  = data.kpFR.toFixed(3);
          if (document.activeElement != kiFRInput) kiFRInput.value  = data.kiFR.toFixed(4);
          if (document.activeElement != kpBLInput)  kpBLInput.value  = data.kpBL.toFixed(3);
          if (document.activeElement != kiBLInput) kiBLInput.value  = data.kiBL.toFixed(4);
          if (document.activeElement != kpBRInput)  kpBRInput.value  = data.kpBR.toFixed(3);
          if (document.activeElement != kiBRInput) kiBRInput.value  = data.kiBR.toFixed(4);

          const txt =
            `Enc (counts): FL=${data.encFL * -1}, FR=${data.encFR}, BL=${data.encBL * -1}, BR=${data.encBR}\n` +
            `Speed (c/s):  FL=${data.speedFL * -1}, FR=${data.speedFR}, BL=${data.speedBL * -1}, BR=${data.speedBR}\n` +
            `TARGET_SPEED=${data.TARGET_SPEED}, Turn=${data.turn}`;
          document.getElementById("motors").innerText = txt;

          document.getElementById("viveX").innerText = data.viveX;
          document.getElementById("viveY").innerText = data.viveY;
          document.getElementById("viveYaw").innerText = data.viveYaw.toFixed(1);

          const vv = document.getElementById("viveValid");
          if (data.viveValid) {
            vv.textContent = "Vive OK";
            vv.classList.remove("red");
            vv.classList.add("green");
          } else {
            vv.textContent = "No Vive";
            vv.classList.remove("green");
            vv.classList.add("red");
          }

          document.getElementById("navXLabel").innerText = data.navX;
          document.getElementById("navYLabel").innerText = data.navY;

          if (typeof data.mode !== "undefined") {
            setModeButtons(data.mode);
          }
        })
        .catch(err => console.log(err));
    }

    // Keyboard Control:
    window.addEventListener("keydown", function(e) {
      if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"," "].includes(e.key)) {
        e.preventDefault();
      }
      if (currentMode !== 0) return;

      switch (e.key) {
        case "w":
        case "W":
        case "ArrowUp":
          adjustSpeed(50);
          break;
        case "s":
        case "S":
        case "ArrowDown":
          adjustSpeed(-50);
          break;
        case "a":
        case "A":
        case "ArrowLeft":
          adjustTurn(-50);
          break;
        case "d":
        case "D":
        case "ArrowRight":
          adjustTurn(50);
          break;
        case " ":
          stopCar();
          break;
      }
    });

    // setInterval(updateStatus, 200);
    // updateStatus();
  </script>
</body>
</html>
)rawliteral";

// ========== Global Variables ==========

Vive510 viveL(SIGNALPINL);
Vive510 viveR(SIGNALPINR);

VL53L1X tofFront;
VL53L1X tofLeft;

int distFront = 4000;
int distLeft  = 4000;
int servoAngle = 90;

// Wall Following 
const int WALL_TARGET = 145;
const int WALL_MIN    = 95;
const int WALL_MAX    = 195;
const int FRONT_ATTACK = 400;
int flag_nearhead = 0;

int TARGET_SPEED = 450;  // Can be changed by using Web

// PID controller
struct PIDController {
  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  int direction = 0;

  float lastError = 0;
  float integral  = 0;
  float output    = 0;
  int pwm         = 0;

  int target = 0;
  int actual = 0;

  float compute(int targetSpeed, int actualSpeed, float dt) {
    target = targetSpeed;
    actual = actualSpeed * direction;

    if (targetSpeed == 0) {
      reset();
      return 0;
    }

    float error = target - actual;
    float deadband = 5;

    if (abs(error) < deadband) {
      error = 0;
    }

    integral += error * dt;
    integral = constrain(integral, -50, 50);

    float derivative = (error - lastError) / dt;
    lastError = error;

    output = kp * error + ki * integral + kd * derivative;
    pwm = pwm + output * 0.1;
    pwm = constrain(pwm, -255, 255);

    int pwm_physical = pwm * direction;
    return pwm_physical;
  }

  void reset() {
    lastError = 0;
    integral  = 0;
    output    = 0;
    pwm       = 0;
  }
};

PIDController pidFL = {0.8f, 0.005f, 0.0f, -1};
PIDController pidFR = {1.0f, 0.005f, 0.0f,  1};
PIDController pidBL = {1.0f, 0.005f, 0.0f, -1};
PIDController pidBR = {1.0f, 0.005f, 0.0f,  1};

// Encoder
int16_t encFL = 0, encFR = 0, encBL = 0, encBR = 0;
int16_t lastEncFL = 0, lastEncFR = 0, lastEncBL = 0, lastEncBR = 0;
int speedFL = 0, speedFR = 0, speedBL = 0, speedBR = 0;

unsigned long lastPIDUpdate = 0;

// Four Control Modes
enum ControlMode {
  MODE_WEB = 0,
  MODE_VIVE_NAV = 1,
  MODE_WALLFOLLOW = 2,
  MODE_STOP = 3
};

int  webTurn        = 0;
int  controlMode    = MODE_STOP;  // Idle is Stop

// ===== Vive =====
float vivePosX = 0.0f;  // tx mid
float vivePosY = 0.0f;  // ty mid
float robotYaw = 0.0f;

bool vivePoseValid = false;
bool havePose      = false;

// Navigation Target（Vive tx/ty）
const float navTargetX_middle_red_button = 4500.0f;//4544.0f;
const float navTargetY_middle_red_button = 4420.0f;//4400.0f;
const float navTargetX_Enemy_red_button  = 4653.0f;
const float navTargetY_Enemy_red_button  = 6500.0f;//6550.0f; 
const float navTargetX_Base_blue_button  = 4500.0f;
const float navTargetY_Base_blue_button  = 1780.0f;
const float navTargetX_High_red_button   = 2560.5f;//2600.5
const float navTargetY_High_red_button   = 4780.0f;//4800.0f;

float navTargetX = 4500.0f;
float navTargetY = 1780.0f;//1800

// Ensure whether it is reached in this period
bool navReached = false;

// ========== Multi-stage navigation (step-by-step) ==========
struct Waypoint {
  float x;
  float y;
};
//Touch Middle Tower(Red Button)
const Waypoint pathMiddle[3] = {
  { 5050.0f, 5000.0f },                    //First Stage: from blue base to middle tower
  { navTargetX_middle_red_button, 5000.0f },                    //Second Stage: pass middle tower
  { navTargetX_middle_red_button, navTargetY_middle_red_button } //Third Stage: Turn around and touch red button(middle tower)
};
//Touch Red Nexus 
const Waypoint pathEnemy[3] = {
  { 5050.0f, 5500.0f },                    
  { navTargetX_Enemy_red_button, 5500.0f },                    
  { navTargetX_Enemy_red_button, navTargetY_Enemy_red_button } 
};
//Touch High Tower(Red Button)
const Waypoint PathHighTower_two[2] = {
  { 2720.0f, 4890.0f },                    
  { navTargetX_High_red_button, navTargetY_High_red_button },                    
};
//Touch High Tower From Red Base
const Waypoint PathHighTower_three[3] = {
  { 3827.5f, 6671.5f },                    // First Stage: Go to the start of ramp
  { 2937.0f, navTargetY_High_red_button },                    // Second Stage: Go to the position of High Tower(Red Button)
  { navTargetX_High_red_button, navTargetY_High_red_button },                    // Third Stage: Turn 90° and hit High Tower(Red Button)
};
//Touch Red Nexus but attack it five times
const Waypoint pathEnemy_Attack[3] = {
  { 5050.0f, 5500.0f },                    
  { navTargetX_Enemy_red_button, 5500.0f },                    
  { navTargetX_Enemy_red_button, navTargetY_Enemy_red_button } 
};


// Whether it is on a path(Consist of many Waypoints）
bool multiNavActive   = false;
int  multiNavIndex    = 0;      // Which Stage
int  multiNavCount    = 0;      // How many Stages
const Waypoint* currentPath = nullptr;  // Which Path(using pointer)

// ========== Attack Several times after reaching red button（pathEnemy_Attack） ==========
bool autoBumpActive   = false;   // Whether it is attacking now
int  autoBumpPhase    = 0;       // 0: forward, 1: backward, 2: stop
int  autoBumpCount    = 0;       // attacking times achieved
unsigned long autoBumpPhaseStart = 0;

const int   AUTO_BUMP_MAX      = 5;     // desired attacking times
const int   AUTO_BUMP_FWD_SPEED  = 250;   // forward speed
const int   AUTO_BUMP_BACK_SPEED = -200;  // backward speed
const int   AUTO_BUMP_FWD_TIME   = 450;   // forward time (ms)
const int   AUTO_BUMP_BACK_TIME  = 400;   // backward time (ms)
const int   AUTO_BUMP_STOP_TIME  = 200;   // stop time (ms)

void startAutoBump() {
  if (autoBumpActive) return;     // If it is attacking, continue
  autoBumpActive = true;
  autoBumpPhase = 0;
  autoBumpCount = 0;
  autoBumpPhaseStart = millis();
  Serial.println("[AutoBump] start auto bump sequence");
}

void updateAutoBump() {
  if (!autoBumpActive) return;

  unsigned long now = millis();
  unsigned long dt  = now - autoBumpPhaseStart;

  switch (autoBumpPhase) {
    case 0: // forward
      drive(AUTO_BUMP_FWD_SPEED, 0);
      if (dt >= AUTO_BUMP_FWD_TIME) {
        autoBumpPhase = 1;
        autoBumpPhaseStart = now;
      }
      break;

    case 1: // backward
      drive(AUTO_BUMP_BACK_SPEED, 0);
      if (dt >= AUTO_BUMP_BACK_TIME) {
        autoBumpPhase = 2;
        autoBumpPhaseStart = now;
      }
      break;

    case 2: // stop
      drive(0, 0);
      if (dt >= AUTO_BUMP_STOP_TIME) {
        autoBumpCount++;
        Serial.print("[AutoBump] bump count = ");
        Serial.println(autoBumpCount);

        if (autoBumpCount >= AUTO_BUMP_MAX) {
          // finished
          autoBumpActive = false;
          drive(0, 0);
          controlMode = MODE_STOP;   // stop
          Serial.println("[AutoBump] all bumps done, STOP");
        } else {
          autoBumpPhase = 0;
          autoBumpPhaseStart = now;
        }
      }
      break;
  }
}


// ========== Servo Control ==========
int angleToDuty(int angle) {
  int pulseWidth = map(angle, 0, 180, 800, 2200);
  return (pulseWidth * 4096) / 20000;
}

void setupServo() {
  Serial.println("Intializing Servo...");
  ledcAttach(SERVO_PIN, 50, 12);
  ledcWrite(SERVO_PIN, angleToDuty(180));//90
  servoAngle = 180;//90
  Serial.println("✓ Servo OK");
}

void moveServo(int angle) {
  servoAngle = constrain(angle, 0, 180);
  ledcWrite(SERVO_PIN, angleToDuty(servoAngle));
}

void stopAllMotors();

unsigned long lastAttack = 0;
void attack() {
  Serial.println("\n===== ATTACK =====");
  stopAllMotors();
  delay(100);

  moveServo(0);
  delay(1500);
  moveServo(180);
  delay(1500);
  moveServo(90);
  delay(1000);

  Serial.println("✓ Attack Finished\n");
}

// ========== Encoder ==========
void setupEncoder(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t config = {};
  config.pulse_gpio_num = pinA;
  config.ctrl_gpio_num  = pinB;
  config.channel        = PCNT_CHANNEL_0;
  config.unit           = unit;
  config.pos_mode       = PCNT_COUNT_INC;
  config.neg_mode       = PCNT_COUNT_DEC;
  config.lctrl_mode     = PCNT_MODE_REVERSE;
  config.hctrl_mode     = PCNT_MODE_KEEP;
  config.counter_h_lim  = 32767;
  config.counter_l_lim  = -32768;

  pcnt_unit_config(&config);
  pcnt_set_filter_value(unit, 500);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void setupEncoders() {
  Serial.println("Initializing Encoders...");
  setupEncoder(PCNT_UNIT_0, ENC_FL_A, ENC_FL_B);
  setupEncoder(PCNT_UNIT_1, ENC_FR_A, ENC_FR_B);
  setupEncoder(PCNT_UNIT_2, ENC_BL_A, ENC_BL_B);
  setupEncoder(PCNT_UNIT_3, ENC_BR_A, ENC_BR_B);
  Serial.println("✓ Four Encoder OK");
}

void readEncoders() {
  pcnt_get_counter_value(PCNT_UNIT_0, &encFL);
  pcnt_get_counter_value(PCNT_UNIT_1, &encFR);
  pcnt_get_counter_value(PCNT_UNIT_2, &encBL);
  pcnt_get_counter_value(PCNT_UNIT_3, &encBR);
}

void calculateSpeeds(float dt) {
  speedFL = (encFL - lastEncFL) / dt;
  speedFR = (encFR - lastEncFR) / dt;
  speedBL = (encBL - lastEncBL) / dt;
  speedBR = (encBR - lastEncBR) / dt;

  lastEncFL = encFL;
  lastEncFR = encFR;
  lastEncBL = encBL;
  lastEncBR = encBR;
}

// ========== Motor (MCPWM) ==========
void setOneMotor(mcpwm_unit_t unit, mcpwm_timer_t timer, int speed) {
  speed = constrain(speed, -255, 255);
  float duty = abs(speed) * 100.0f / 255.0f;

  if (speed > 0) {
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, duty);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
  } else if (speed < 0) {
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, duty);
  } else {
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
  }
}

void setupMotors() {
  Serial.println("Initializing Motors(MCPWM)...");

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_FL_RPWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_FL_LPWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_FR_RPWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_FR_LPWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MOTOR_BL_RPWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, MOTOR_BL_LPWM);

  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR_BR_RPWM);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, MOTOR_BR_LPWM);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;
  pwm_config.cmpr_a    = 0;
  pwm_config.cmpr_b    = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);

  stopAllMotors();
  Serial.println("✓ Four Motors OK (MCPWM)");
}

void setTargetSpeeds(int leftSpeed, int rightSpeed) {
  pidFL.target = leftSpeed;
  pidBL.target = leftSpeed;
  pidFR.target = rightSpeed;
  pidBR.target = rightSpeed;
}

void stopAllMotors() {
  setTargetSpeeds(0, 0);

  setOneMotor(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
  setOneMotor(MCPWM_UNIT_0, MCPWM_TIMER_1, 0);
  setOneMotor(MCPWM_UNIT_0, MCPWM_TIMER_2, 0);
  setOneMotor(MCPWM_UNIT_1, MCPWM_TIMER_0, 0);

  pidFL.reset();
  pidFR.reset();
  pidBL.reset();
  pidBR.reset();
}

void updatePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDUpdate) / 1000.0f;
  if (dt < 0.02f) return;
  lastPIDUpdate = now;

  readEncoders();
  calculateSpeeds(dt);

  int pwmFL = pidFL.compute(pidFL.target, speedFL, dt);
  int pwmFR = pidFR.compute(pidFR.target, speedFR, dt);
  int pwmBL = pidBL.compute(pidBL.target, speedBL, dt);
  int pwmBR = pidBR.compute(pidBR.target, speedBR, dt);

  if (pidFL.target != 0 && abs(pwmFL) < 10) pwmFL = (pwmFL >= 0) ? 10 : -10;
  if (pidFR.target != 0 && abs(pwmFR) < 10) pwmFR = (pwmFR >= 0) ? 10 : -10;
  if (pidBL.target != 0 && abs(pwmBL) < 10) pwmBL = (pwmBL >= 0) ? 10 : -10;
  if (pidBR.target != 0 && abs(pwmBR) < 10) pwmBR = (pwmBR >= 0) ? 10 : -10;

  setOneMotor(MCPWM_UNIT_0, MCPWM_TIMER_0, pwmFL);
  setOneMotor(MCPWM_UNIT_0, MCPWM_TIMER_1, pwmFR);
  setOneMotor(MCPWM_UNIT_0, MCPWM_TIMER_2, pwmBL);
  setOneMotor(MCPWM_UNIT_1, MCPWM_TIMER_0, pwmBR);
}

// forward: Speed，turn: Differential Steering
void drive(int forward, int turn) {
  int leftSpeed  = forward + turn;
  int rightSpeed = forward - turn;
  setTargetSpeeds(leftSpeed, rightSpeed);
}

// ========== TOF ==========
void setupTOF() {
  Serial.println("Initializing TOF...");

  Wire.begin(TOF_SDA, TOF_SCL);
  pinMode(TOF_FRONT_SHUT, OUTPUT);
  pinMode(TOF_LEFT_SHUT,  OUTPUT);
  digitalWrite(TOF_FRONT_SHUT, LOW);
  digitalWrite(TOF_LEFT_SHUT,  LOW);
  delay(10);

  digitalWrite(TOF_FRONT_SHUT, HIGH);
  delay(10);
  tofFront.setTimeout(500);
  if (tofFront.init()) {
    tofFront.setAddress(0x30);
    tofFront.setDistanceMode(VL53L1X::Short);
    tofFront.setMeasurementTimingBudget(20000);
    tofFront.startContinuous(50);
    Serial.println("✓ Front TOF OK");
  }

  digitalWrite(TOF_LEFT_SHUT, HIGH);
  delay(10);
  tofLeft.setTimeout(500);
  if (tofLeft.init()) {
    tofLeft.setAddress(0x31);
    tofLeft.setDistanceMode(VL53L1X::Short);
    tofLeft.setMeasurementTimingBudget(20000);
    tofLeft.startContinuous(50);
    Serial.println("✓ Left TOF OK");
  }
}

void readSensors() {
  static bool inited = false;
  static int disFrontPre = 0;
  static int disLeftPre  = 0;

  int rawFront = tofFront.read();
  int rawLeft  = tofLeft.read();

  if (rawFront > 4000) rawFront = 4000;
  if (rawFront < 10)   rawFront = 0;
  if (rawLeft  > 4000) rawLeft  = 4000;
  if (rawLeft  < 10)   rawLeft  = 0;

  const int TH_front = 50;
  const int TH_left  = 150;

  if (!inited) {
    distFront   = rawFront;
    distLeft    = rawLeft;
    disFrontPre = distFront;
    disLeftPre  = distLeft;
    inited      = true;
    return;
  }

  if (rawFront < 280 && abs(rawFront - disFrontPre) > TH_front) {
    distFront = disFrontPre;
  } else {
    distFront = rawFront;
  }

  distLeft = rawLeft;
  disFrontPre = distFront;
}

// ========== Wall Follow(Add PD Control) ==========
void wallFollow() {
  int fwd = TARGET_SPEED;
  int turn = 0;
  float kp_wallfollow = 0.5;
  float kd_wallfollow = 5.0;
  static int last_error = 0;
  bool isblocked = false;

  if (distFront < 170) isblocked = true;
  else if (distFront > 330 && distLeft < 115) isblocked = false; //125

  if (isblocked) {
    fwd = 0;
    turn = 900;
  } else {
    if (distLeft < WALL_MIN) {
      fwd = TARGET_SPEED * 0.7;
      turn = 145;
    } else if (distLeft > WALL_MAX) {
      fwd = TARGET_SPEED * 0.7;
      turn = -145;
    } else {
      int error = distLeft - WALL_TARGET;
      int derror = error - last_error;
      turn = (int)(-1 * (error * kp_wallfollow + derror * kd_wallfollow));
      last_error = error;
      turn = constrain(turn, -80, 80);
    }
  }
  drive(fwd, turn);
}

// ========== Vive Control ==========
uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else
    middle = (a <= b) ? a : b;
  return middle;
}

void updateVivePose() {
  static uint16_t xL, yL, xR, yR;
  static uint16_t xL0, yL0, xR0, yR0;
  static uint16_t xL1, yL1, xR1, yR1;
  static uint16_t xL2, yL2, xR2, yR2;

  bool okL = (viveL.status() == VIVE_RECEIVING);
  bool okR = (viveR.status() == VIVE_RECEIVING);

  if (!(okL || okR)) {
    vivePoseValid = false;
    viveL.sync(5);
    viveR.sync(5);
    Serial.println("Both Vive no Signal");
    return;
  }

  if (okL) {
    xL2 = xL1; xL1 = xL0;
    yL2 = yL1; yL1 = yL0;
    xL0 = viveL.xCoord();
    yL0 = viveL.yCoord();
    xL  = med3filt(xL0, xL1, xL2);
    yL  = med3filt(yL0, yL1, yL2);
  }

  if (okR) {
    xR2 = xR1; xR1 = xR0;
    yR2 = yR1; yR1 = yR0;
    xR0 = viveR.xCoord();
    yR0 = viveR.yCoord();
    xR  = med3filt(xR0, xR1, xR2);
    yR  = med3filt(yR0, yR1, yR2);
  }

  if (xL < 1000 || xL > 8000) okL = false;
  if (yL < 1000 || yL > 8000) okL = false;
  if (xR < 1000 || xR > 8000) okR = false;
  if (yR < 1000 || yR > 8000) okR = false;

  Serial.println("====== Vive Frame ======");
  if (okL) {
    Serial.print("Left  tx="); Serial.print(xL);
    Serial.print(" ty=");      Serial.println(yL);
  } else {
    Serial.println("Left  invalid");
  }
  if (okR) {
    Serial.print("Right tx="); Serial.print(xR);
    Serial.print(" ty=");      Serial.println(yR);
  } else {
    Serial.println("Right invalid");
  }

  if (!(okL && okR)) {
    vivePoseValid = false;
    Serial.println("Not enough Vive for pose");
    return;
  }

  float posX_new = 0.5f * (float(xL) + float(xR));
  float posY_new = 0.5f * (float(yL) + float(yR));

  float vwx = float(xR) - float(xL);
  float vwy = float(yR) - float(yL);
  float yaw_rad_new = atan2(vwy, vwx);
  float yaw_deg_new = yaw_rad_new * 180.0f / PI;

  if (!havePose) {
    vivePosX = posX_new;
    vivePosY = posY_new;
    robotYaw = yaw_deg_new;
    havePose = true;
    vivePoseValid = true;
  } else {
    float dx = posX_new - vivePosX;
    float dy = posY_new - vivePosY;
    //700
    if (fabs(dx) <= 800 && fabs(dy) <= 800) {
      vivePosX = posX_new;
      vivePosY = posY_new;
      robotYaw = yaw_deg_new;
      vivePoseValid = true;
    } else {
      vivePoseValid = false;
      Serial.println("Vive pose jump too large, ignore frame");
    }
  }

  Serial.print("VivePosX(us)="); Serial.print(vivePosX);
  Serial.print(" VivePosY(us)="); Serial.print(vivePosY);
  Serial.print(" Yaw(deg)=");     Serial.println(robotYaw);
  Serial.println();
}

// ========== Vive Navigation ==========
void navigateToTarget() {
  static float lastAngleErrDeg = 0.0f;
  static float dErrFiltered = 0.0f;
  static unsigned long lastTs = 0;
  navReached = false;

  if (!vivePoseValid) {
    drive(0, 0);
    Serial.println("[Nav] no Vive, stop");
    return;
  }

  float x_robot = vivePosX;
  float y_robot = vivePosY;
  float dx = navTargetX - x_robot;
  float dy = navTargetY - y_robot;
  float dist_error = sqrt(dx*dx + dy*dy);
  const float DIST_TOL = 50.0f;//80

  if (dist_error < DIST_TOL) {
    drive(0, 0);
    navReached = true;
    Serial.println("[Nav] Reached target");
    lastAngleErrDeg = 0;
    dErrFiltered = 0;
    lastTs = millis();
    return;
  }
  //===Angle error====
  float angle_to_target = atan2(dy, dx);
  const float HEADING_OFFSET_DEG = 90.0f;
  float yaw_rad         = (robotYaw+HEADING_OFFSET_DEG) * PI / 180.0f;

  float angle_error     = angle_to_target - yaw_rad;

  while (angle_error >  PI) angle_error -= 2.0f * PI;
  while (angle_error < -PI) angle_error += 2.0f * PI;

  float angle_err_deg = angle_error * 180.0f / PI;

  const float deadZone = 5.0;//3.0
  if(fabs(angle_err_deg) < deadZone) angle_err_deg = 0;

  //====calculate dt=========
  unsigned long now = millis();
  float dt = (now - lastTs) / 1000.0f;
  if(dt <= 0.0f) dt = 0.02f;
  lastTs = now;

  //========Speed Control=======
  const float MAX_FWD  = 550;//450
  const float Kp_dist  = 5;
  float forward_cmd;
  if(dist_error > 50 && dist_error < 100) forward_cmd = Kp_dist * dist_error;
  else  forward_cmd = MAX_FWD;
  
  if (forward_cmd > MAX_FWD) forward_cmd = MAX_FWD;
 //25
  if (fabs(angle_err_deg) > 35.0f) {
    forward_cmd = 0;
  }

  //=====PD Control for turning====
  const float MAX_TURN = 350;//300
  const float Kp_angle = 18.0f;//40.0f
  const float kd_angle = 3.0f; //2.5
  float dErr = (angle_err_deg - lastAngleErrDeg) / dt;
  lastAngleErrDeg = angle_err_deg;
  
  float turn_cmd = -(Kp_angle * angle_err_deg + kd_angle * dErrFiltered);

  if (turn_cmd >  MAX_TURN) turn_cmd =  MAX_TURN;
  if (turn_cmd < -MAX_TURN) turn_cmd = -MAX_TURN;

  drive((int)forward_cmd, (int)turn_cmd);

  Serial.print("[Nav] dist(us)="); Serial.print(dist_error);
  Serial.print(" targetDeg=");     Serial.print(angle_to_target * 180.0f / PI);
  Serial.print(" headingDeg=");    Serial.print(yaw_rad * 180.0f / PI);
  Serial.print(" angErr=");        Serial.print(angle_err_deg);
  Serial.print(" fwd=");           Serial.print(forward_cmd);
  Serial.print(" turn=");          Serial.println(turn_cmd);
}

// ========== Multi-stage Navigation ==========
void runViveNav() {
  if (!multiNavActive || currentPath == nullptr) {
    navigateToTarget();
    return;
  }

  // Have one path
  navigateToTarget();

  if (navReached) {
    multiNavIndex++;
    if (multiNavIndex < multiNavCount) {
      navTargetX = currentPath[multiNavIndex].x;
      navTargetY = currentPath[multiNavIndex].y;

      Serial.print("[SegNav] goto waypoint #");
      Serial.print(multiNavIndex);
      Serial.print(" -> (");
      Serial.print(navTargetX);
      Serial.print(", ");
      Serial.print(navTargetY);
      Serial.println(")");
    } else {
      // Path achieved
      multiNavActive = false;
      Serial.println("[SegNav] path finished");

      if (currentPath == pathEnemy_Attack) {
        // Specific task: needs to attack for several times
        Serial.println("[SegNav] pathEnemy_Attack finished, start auto bumps");
        startAutoBump();
      } else {
        currentPath = nullptr;
        drive(0, 0);
        controlMode = MODE_STOP;   // or MODE_WEB
        Serial.println("[SegNav] stop (no auto bump)");
      }
      currentPath = nullptr;  // clear path
    }
  }
}

//=================TopHat=================
void TopHat_onWifiPacket() {
  if (wifiPktCountWindow < 255) wifiPktCountWindow++; // prevent overflow
}
// Send one byte to TopHat
bool tophat_sendByte(uint8_t data)
{
  Wire.beginTransmission(TOPHAT_ADDR);
  Wire.write(data);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    return true;
  } else {
    Serial.printf("[TopHat] send error: %d\n", error);
    return false;
  }
}

// Read One byte from Tophat
bool tophat_receiveByte(uint8_t &val)
{
  uint8_t bytes = Wire.requestFrom(TOPHAT_ADDR, (uint8_t)1);
  if (bytes > 0) {
    val = Wire.read();
    return true;
  } else {
    Serial.println("[TopHat] recv error: no data");
    return false;
  }
}

// TopHat communication period is TOPHAT_PERIOD_MS(500ms)
void updateTophatI2C() {
  unsigned long now = millis();
  if (now - lastTophatUpdate < TOPHAT_PERIOD_MS) return;
  lastTophatUpdate = now;

  // Get number of Wifi Packet and clear the number of wifi packet in this window 
  uint8_t sendVal = wifiPktCountWindow;
  wifiPktCountWindow = 0;

  bool okSend = tophat_sendByte(sendVal);

  uint8_t recvVal = 0;
  bool okRecv = tophat_receiveByte(recvVal);
  if (okRecv) {
    tophatLastByte = recvVal;
  }
  tophatOk = okSend && okRecv;

  Serial.printf("[TopHat] wifiPkts=%3u | sendOK=%d | recvOK=%d | rx=0x%02X\n",
                sendVal, okSend, okRecv, tophatLastByte);
}


// ========== Web ==========
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
  TopHat_onWifiPacket();
}

void handleStatus() {
  TopHat_onWifiPacket(); 
  String json = "{";
  json += "\"distFront\":" + String(distFront) + ",";
  json += "\"distLeft\":"  + String(distLeft)  + ",";
  json += "\"encFL\":" + String(encFL) + ",";
  json += "\"encFR\":" + String(encFR) + ",";
  json += "\"encBL\":" + String(encBL) + ",";
  json += "\"encBR\":" + String(encBR) + ",";
  json += "\"speedFL\":" + String(speedFL) + ",";
  json += "\"speedFR\":" + String(speedFR) + ",";
  json += "\"speedBL\":" + String(speedBL) + ",";
  json += "\"speedBR\":" + String(speedBR) + ",";
  json += "\"TARGET_SPEED\":" + String(TARGET_SPEED) + ",";
  json += "\"turn\":" + String(webTurn) + ",";
  json += "\"kpFL\":" + String(pidFL.kp, 3) + ",";
  json += "\"kiFL\":" + String(pidFL.ki, 4) + ",";
  json += "\"kpFR\":" + String(pidFR.kp, 3) + ",";
  json += "\"kiFR\":" + String(pidFR.ki, 4) + ",";
  json += "\"kpBL\":" + String(pidBL.kp, 3) + ",";
  json += "\"kiBL\":" + String(pidBL.ki, 4) + ",";
  json += "\"kpBR\":" + String(pidBR.kp, 3) + ",";
  json += "\"kiBR\":" + String(pidBR.ki, 4) + ",";

  json += "\"viveX\":" + String(vivePosX, 0) + ",";
  json += "\"viveY\":" + String(vivePosY, 0) + ",";
  json += "\"viveYaw\":" + String(robotYaw, 1) + ",";
  json += "\"viveValid\":" + String(vivePoseValid ? 1 : 0) + ",";

  json += "\"navX\":" + String(navTargetX, 0) + ",";
  json += "\"navY\":" + String(navTargetY, 0) + ",";
  json += "\"mode\":" + String(controlMode);

  json += "}";
  server.send(200, "application/json", json);
}

void handleSet() {
  TopHat_onWifiPacket();
  Serial.println("[HTTP] /set called");

  if (server.hasArg("speed")) {
    TARGET_SPEED = server.arg("speed").toInt();
    Serial.print("  speed="); Serial.println(TARGET_SPEED);
  }
  if (server.hasArg("turn")) {
    webTurn = server.arg("turn").toInt();
    Serial.print("  turn="); Serial.println(webTurn);
  }

  if (server.hasArg("kpFL")) pidFL.kp = server.arg("kpFL").toFloat();
  if (server.hasArg("kiFL")) pidFL.ki = server.arg("kiFL").toFloat();
  if (server.hasArg("kpFR")) pidFR.kp = server.arg("kpFR").toFloat();
  if (server.hasArg("kiFR")) pidFR.ki = server.arg("kiFR").toFloat();
  if (server.hasArg("kpBL")) pidBL.kp = server.arg("kpBL").toFloat();
  if (server.hasArg("kiBL")) pidBL.ki = server.arg("kiBL").toFloat();
  if (server.hasArg("kpBR")) pidBR.kp = server.arg("kpBR").toFloat();
  if (server.hasArg("kiBR")) pidBR.ki = server.arg("kiBR").toFloat();

  if (server.hasArg("navX")) {
    navTargetX = server.arg("navX").toFloat();
    Serial.print("  navX="); Serial.println(navTargetX);
  }
  if (server.hasArg("navY")) {
    navTargetY = server.arg("navY").toFloat();
    Serial.print("  navY="); Serial.println(navTargetY);
  }

  if (server.hasArg("navX") || server.hasArg("navY")) {
    multiNavActive = false;
    currentPath    = nullptr;
    multiNavIndex  = 0;
    multiNavCount  = 0;
  }

  if (server.hasArg("mode")) {
    int m = server.arg("mode").toInt();
    Serial.print("  request mode="); Serial.println(m);
    if (m >= MODE_WEB && m <= MODE_STOP) {
      controlMode = m;
      if (controlMode == MODE_STOP) {
        TARGET_SPEED = 0;
        webTurn = 0;
        stopAllMotors();
      }
    }
  }

  Serial.print("  controlMode now = ");
  Serial.println(controlMode);

  server.send(200, "text/plain", "OK");
}

void handleAttack() {
  TopHat_onWifiPacket();
  attack();
  server.send(200, "text/plain", "ATTACK");
}

void setupWiFiAndWeb() {
  Serial.println("Start WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PSWD);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/set", handleSet);
  server.on("/attack", handleAttack);

  // Five Points: 1 Blue Base / 2 Middle Tower(Red Button)/ 3 Red Base/ 4.High Tower(Red Button)/ 5.High Tower(Red Button, but from red base)/6. Attack Red Base several times
  server.on("/navPoint", []() {
    TopHat_onWifiPacket();
    if (!server.hasArg("id")) {
      server.send(400, "text/plain", "need id");
      return;
    }

    int id = server.arg("id").toInt();  //1 Blue Base / 2 Middle Tower(Red Button)/ 3 Red Base/ 4.High Tower(Red Button)/ 5.High Tower(Red Button, but from red base)/6. Attack Red Base several times
    Serial.print("[Web] /navPoint id="); Serial.println(id);

    if (id < 1 || id > 6) {
      server.send(400, "text/plain", "bad id");
      return;
    }

    // Assume no navigate autonomously
    multiNavActive = false;
    currentPath    = nullptr;
    multiNavIndex  = 0;
    multiNavCount  = 0;

    if (id == 1) {
      // Blue Base
      navTargetX = navTargetX_Base_blue_button;
      navTargetY = navTargetY_Base_blue_button;
    } else if (id == 2) {
      // Middle Tower(red Button)[Three Stages]
      currentPath    = pathMiddle;
      multiNavActive = true;
      multiNavIndex  = 0;
      multiNavCount  = 3;
      navTargetX = currentPath[0].x;
      navTargetY = currentPath[0].y;
    } else if (id == 3) {
      // Red Base[Three Stages]
      currentPath    = pathEnemy;
      multiNavActive = true;
      multiNavIndex  = 0;
      multiNavCount  = 3;
      navTargetX = currentPath[0].x;
      navTargetY = currentPath[0].y;
    }else if (id == 4) {
      // High Tower(Red Button)[Two Stages]
      currentPath    = PathHighTower_two;
      multiNavActive = true;
      multiNavIndex  = 0;
      multiNavCount  = 2;
      navTargetX = currentPath[0].x;
      navTargetY = currentPath[0].y;
    }else if (id == 5) {
      // High Tower(Red Button but start from Red Base)[Three Stages]
      currentPath    = PathHighTower_three;
      multiNavActive = true;
      multiNavIndex  = 0;
      multiNavCount  = 3;
      navTargetX = currentPath[0].x;
      navTargetY = currentPath[0].y;
    }
    else if (id == 6) {
      // Red Base(Several Attacks)[Three Stages]
      currentPath    = pathEnemy_Attack;
      multiNavActive = true;
      multiNavIndex  = 0;
      multiNavCount  = 3;
      navTargetX = currentPath[0].x;
      navTargetY = currentPath[0].y;
    }

    controlMode = MODE_VIVE_NAV;
    TARGET_SPEED = 450;
    webTurn = 0;

    Serial.print("[Web] Nav to point "); Serial.print(id);
    Serial.print(" ("); Serial.print(navTargetX);
    Serial.print(", ");  Serial.print(navTargetY);
    Serial.print("), multiNavActive=");
    Serial.println(multiNavActive ? "ON" : "OFF");

    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("HTTP server started");
}

// ========== Main ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  viveL.begin();
  viveR.begin();

  Serial.println("\n========================================");
  Serial.println("  Vive Navigation (tx/ty space) + Web + Keyboard");
  Serial.println("========================================\n");

  setupServo();
  delay(300);
  setupMotors();
  delay(300);
  setupEncoders();
  delay(300);
  setupTOF();
  delay(300);
  setupWiFiAndWeb();
  Wire.setClock(40000);
  Serial.println("I2C clock set to 40kHz for TopHat");

  Serial.println("Set up finished");
  lastPIDUpdate = millis();
}

void loop() {
  server.handleClient();

  updatePID();
  updateVivePose();

  updateTophatI2C();

  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate > 50) {
    readSensors();
    lastSensorUpdate = millis();
  }

  static unsigned long lastCtrl = 0;
  if (millis() - lastCtrl > 20) {
    if (autoBumpActive) {
      updateAutoBump();
    } else {
      switch (controlMode) {
        case MODE_WEB:
          drive(TARGET_SPEED, webTurn);
          break;
        case MODE_VIVE_NAV:
          runViveNav();         
          break;
        case MODE_WALLFOLLOW:
          wallFollow();
          break;
        case MODE_STOP:
        default:
          stopAllMotors();
          break;
      }
    }
    lastCtrl = millis();
  }

  if (tophatOk && tophatLastByte == 0) {
    controlMode = MODE_STOP;
    stopAllMotors();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Mode="); Serial.print(controlMode);
    Serial.print("  Front TOF:");  Serial.print(distFront);
    Serial.print(" Left TOF:");   Serial.print(distLeft);
    Serial.print("  SPEED="); Serial.print(TARGET_SPEED);
    Serial.print(" TURN=");   Serial.println(webTurn);
    lastPrint = millis();
  }
}
