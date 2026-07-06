/**
 * ============================================================
 *  Receiver_ESP.ino  —  WiFi BRIDGE + WEB DASHBOARD
 *  Folder: Self-Balancing-Robot\Two_ESP_Run\Receiver_ESP\
 * ============================================================
 *
 *  This ESP32 does ONLY:
 *    1. Creates a WiFi Access Point ("BalanceBot_AP")
 *    2. Receives ESP-NOW packets from the Bot ESP
 *    3. Pushes data to a WebSocket → browser dashboard
 *    4. Hosts the web dashboard (same as original Telemetry.h
 *       but extended with: zone display, live Kp/Ki/Kd readout,
 *       and CSV download with zone column)
 *    5. Forwards gain-change commands back to Bot via ESP-NOW
 *
 *  FIRST STEP:
 *    Upload this to your second ESP32, open Serial at 115200,
 *    and copy the MAC address printed. Paste it into Bot_ESP.ino
 *    as RECEIVER_MAC[].
 *
 *  THEN:
 *    Upload Bot_ESP.ino to the robot ESP.
 *
 *  CONNECT:
 *    WiFi SSID: BalanceBot_AP
 *    Password:  balancebot
 *    Open:      http://192.168.4.1
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_now.h>

// ─────────────────────────────────────────────────────────────
// WiFi AP credentials
// ─────────────────────────────────────────────────────────────
const char* AP_SSID = "BalanceBot_AP";
const char* AP_PASS = "balancebot";

// ─────────────────────────────────────────────────────────────
// ESP-NOW: Bot ESP MAC — filled in by onDataRecv callback
// We also need our own MAC to give to the Bot for command replies
// ─────────────────────────────────────────────────────────────
uint8_t botMac[6] = {0};        // Learned when first packet arrives
bool    botPeerAdded = false;   // Register peer only once

// ─────────────────────────────────────────────────────────────
// TELEMETRY PACKET — MUST MATCH Bot_ESP.ino EXACTLY
// ─────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    float    timeS;
    float    angle;
    float    gyroRate;
    float    targetAngle;
    float    errAngle;
    float    pidOutput;
    float    position;
    float    posError;
    float    encSpeed;
    float    loopHz;
    uint8_t  zone;
    uint8_t  state;
    float    Kp_live;
    float    Ki_live;
    float    Kd_live;
} TelemetryPacket;

// ─────────────────────────────────────────────────────────────
// COMMAND PACKET (Receiver → Bot via ESP-NOW reply)
// ─────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    char    cmdType;   // 'p'=Kp 'i'=Ki 'o'=Kd 'a'=Kp_pos 's'=Ki_pos 'd'=Kd_pos
    //                   'A'=ARM 'D'=DISARM 'C'=CAL
    float   value;
} CommandPacket;

// ─────────────────────────────────────────────────────────────
// Zone names for dashboard
// ─────────────────────────────────────────────────────────────
const char* ZONE_NAMES[] = { "STABLE", "DISTURBED", "RECOVERY", "CUTOFF" };
const char* STATE_NAMES[] = { "DISARMED", "CALIBRATING", "READY", "ARMED" };

// ─────────────────────────────────────────────────────────────
// WEB SERVERS
// ─────────────────────────────────────────────────────────────
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// Ring buffer for latest telemetry (shared between ESP-NOW callback and ws task)
volatile TelemetryPacket latestPkt = {};
volatile bool            newPktAvail = false;
portMUX_TYPE             pktMux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────────────────────────────
// HTML DASHBOARD  (extended from original Telemetry.h)
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot — Two-ESP Dashboard</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:#0f1117;color:#e0e0e0;padding:12px}
h1{font-size:1.3em;color:#7eb8ff;margin-bottom:8px}
h2{font-size:1em;color:#aaa;margin-bottom:6px;border-bottom:1px solid #333;padding-bottom:4px}
.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;flex-wrap:wrap;gap:6px}
#ws{padding:4px 10px;border-radius:12px;font-size:0.8em;font-weight:bold}
#zonebadge{padding:4px 12px;border-radius:12px;font-size:0.85em;font-weight:bold;background:#1b3a2e;color:#80cbc4;min-width:100px;text-align:center}
.on{background:#1b5e20;color:#a5d6a7}.off{background:#b71c1c;color:#ef9a9a}
.g{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:600px){.g{grid-template-columns:1fr}}
.p{background:#1a1d27;border:1px solid #2a2d37;border-radius:8px;padding:12px}
.p.wide{grid-column:1/-1}
.dr{display:flex;justify-content:space-between;align-items:center;padding:3px 0;font-size:0.9em}
.dr label{color:#888;min-width:110px}
.dr span{color:#7eb8ff;font-family:monospace;font-size:1.05em}
.zone-stable{color:#80cbc4!important}.zone-disturbed{color:#ffcc80!important}
.zone-recovery{color:#ef9a9a!important}.zone-cutoff{color:#ff1744!important}
.pr{display:flex;gap:6px;align-items:center;margin:4px 0}
.pr label{min-width:30px;color:#888;font-size:0.85em}
.pr input{width:80px;background:#0f1117;border:1px solid #444;color:#e0e0e0;padding:4px 6px;border-radius:4px;font-size:0.85em}
.pr input:focus{border-color:#7eb8ff;outline:none}
button{padding:6px 14px;border:none;border-radius:5px;cursor:pointer;font-size:0.85em;font-weight:600;transition:0.2s}
.btn-s{background:#1565c0;color:#fff}.btn-s:hover{background:#1976d2}
.btn-g{background:#2e7d32;color:#fff}.btn-g:hover{background:#388e3c}
.btn-r{background:#c62828;color:#fff}.btn-r:hover{background:#d32f2f}
.btn-y{background:#f57f17;color:#fff}.btn-y:hover{background:#f9a825}
.btn-p{background:#6a1b9a;color:#fff}.btn-p:hover{background:#7b1fa2}
.btns{display:flex;gap:6px;flex-wrap:wrap;margin-top:6px}
canvas{width:100%;height:160px;border:1px solid #2a2d37;border-radius:4px;background:#12141c}
#rc{color:#ef5350;font-size:0.85em;margin-top:4px}
.state{font-size:1.1em;font-weight:bold;color:#ffa726;margin:4px 0}
.live-gain{color:#a5d6a7!important}
.pkt-count{font-size:0.75em;color:#555;margin-top:2px}
</style></head><body>

<div class="top">
  <h1>&#9881; BalanceBot — Two-ESP</h1>
  <span id="zonebadge">ZONE: --</span>
  <span id="ws" class="off">DISCONNECTED</span>
</div>

<div class="g">

<!-- Live Data -->
<div class="p">
<h2>&#128203; Live Data</h2>
<div class="dr"><label>State:</label><span id="st" class="state">--</span></div>
<div class="dr"><label>Angle:</label><span id="an">--</span>&deg;</div>
<div class="dr"><label>Gyro Rate:</label><span id="gr">--</span>&deg;/s</div>
<div class="dr"><label>Target Angle:</label><span id="ta">--</span>&deg;</div>
<div class="dr"><label>PID Output:</label><span id="po">--</span></div>
<div class="dr"><label>Position:</label><span id="ps">--</span>&deg;</div>
<div class="dr"><label>Enc Speed:</label><span id="es">--</span></div>
<div class="dr"><label>Loop Hz:</label><span id="hz">--</span></div>
<div class="dr"><label>Zone:</label><span id="zn" class="live-gain">--</span></div>
<div class="pkt-count">Packets: <span id="pkts">0</span></div>
</div>

<!-- Live Gains (read from bot) -->
<div class="p">
<h2>&#128200; Active Gains (from Bot)</h2>
<div class="dr"><label>Kp (live):</label><span id="kp_live" class="live-gain">--</span></div>
<div class="dr"><label>Ki (live):</label><span id="ki_live" class="live-gain">--</span></div>
<div class="dr"><label>Kd (live):</label><span id="kd_live" class="live-gain">--</span></div>
<br>
<h2>&#128308; Data Recording</h2>
<div class="btns">
  <button class="btn-g" id="rb" onclick="toggleRec()">Start Recording</button>
  <button class="btn-p" onclick="dlCSV()">Download CSV</button>
</div>
<div id="rc">0 samples</div>
</div>

<!-- Override Inner PID -->
<div class="p">
<h2>&#128295; Override Inner PID</h2>
<div class="pr"><label>Kp:</label><input type="number" id="kp_a" step="0.5" value="18"><button class="btn-s" onclick="sc('p','kp_a')">Set</button></div>
<div class="pr"><label>Ki:</label><input type="number" id="ki_a" step="5" value="230"><button class="btn-s" onclick="sc('i','ki_a')">Set</button></div>
<div class="pr"><label>Kd:</label><input type="number" id="kd_a" step="0.05" value="0.55"><button class="btn-s" onclick="sc('o','kd_a')">Set</button></div>
</div>

<!-- Override Outer PID -->
<div class="p">
<h2>&#128295; Override Outer PID (Position)</h2>
<div class="pr"><label>Kp:</label><input type="number" id="kp_p" step="0.005" value="0.020"><button class="btn-s" onclick="sc('a','kp_p')">Set</button></div>
<div class="pr"><label>Ki:</label><input type="number" id="ki_p" step="0.001" value="0.008"><button class="btn-s" onclick="sc('s','ki_p')">Set</button></div>
<div class="pr"><label>Kd:</label><input type="number" id="kd_p" step="0.001" value="0.012"><button class="btn-s" onclick="sc('d','kd_p')">Set</button></div>
</div>

<!-- Angle Chart -->
<div class="p wide">
<h2>&#128200; Angle Chart (last 400 samples)</h2>
<canvas id="cv"></canvas>
</div>

</div>

<script>
var sock=null,rec=false,dat=[],chartBuf=[],pktCount=0;
var SN=['DISARMED','CALIBRATING','READY','ARMED'];
var ZN=['STABLE','DISTURBED','RECOVERY','CUTOFF'];
var ZC=['zone-stable','zone-disturbed','zone-recovery','zone-cutoff'];
var ZBG=['#1b3a2e','#3a2e00','#3a1010','#2a0010'];
var ZTC=['#80cbc4','#ffcc80','#ef9a9a','#ff4444'];

function conn(){
  sock=new WebSocket('ws://'+location.hostname+':81/');
  sock.onopen=function(){
    document.getElementById('ws').className='on';
    document.getElementById('ws').textContent='CONNECTED';
  };
  sock.onclose=function(){
    document.getElementById('ws').className='off';
    document.getElementById('ws').textContent='DISCONNECTED';
    setTimeout(conn,2000);
  };
  sock.onmessage=function(e){
    var m=e.data;
    if(m[0]==='D'){
      var v=m.substring(2).split(',');
      var f=v.map(parseFloat);
      // D,time,angle,gyroRate,targetAngle,errAngle,pidOutput,position,posError,encSpeed,loopHz,zone,state,Kp,Ki,Kd
      document.getElementById('an').textContent=f[1].toFixed(2);
      document.getElementById('gr').textContent=f[2].toFixed(1);
      document.getElementById('ta').textContent=f[3].toFixed(2);
      document.getElementById('po').textContent=f[5].toFixed(1);
      document.getElementById('ps').textContent=f[6].toFixed(1);
      document.getElementById('es').textContent=f[8].toFixed(1);
      document.getElementById('hz').textContent=f[9].toFixed(0);
      // Zone
      var zi=parseInt(f[10]); zi=isNaN(zi)?0:Math.min(zi,3);
      var zEl=document.getElementById('zn');
      zEl.textContent=ZN[zi]; zEl.className=ZC[zi];
      var zb=document.getElementById('zonebadge');
      zb.textContent='ZONE: '+ZN[zi];
      zb.style.background=ZBG[zi]; zb.style.color=ZTC[zi];
      // State
      var si=parseInt(f[11]); si=isNaN(si)?0:Math.min(si,3);
      document.getElementById('st').textContent=SN[si];
      // Live gains
      if(f.length>14){
        document.getElementById('kp_live').textContent=f[12].toFixed(2);
        document.getElementById('ki_live').textContent=f[13].toFixed(1);
        document.getElementById('kd_live').textContent=f[14].toFixed(3);
      }
      pktCount++; document.getElementById('pkts').textContent=pktCount;
      chartBuf.push(f[1]);
      if(chartBuf.length>400) chartBuf.shift();
      drawChart();
      if(rec){ dat.push(v.join(',')); document.getElementById('rc').textContent=dat.length+' samples'; }
    }
  };
}
conn();

function sc(c,id){ if(sock&&sock.readyState===1) sock.send(c+document.getElementById(id).value); }

function toggleRec(){
  rec=!rec;
  var b=document.getElementById('rb');
  if(rec){ dat=[]; b.textContent='Stop Recording'; b.className='btn-r'; document.getElementById('rc').textContent='0 samples'; }
  else   { b.textContent='Start Recording'; b.className='btn-g'; }
}

function dlCSV(){
  if(dat.length===0){ alert('No data recorded!'); return; }
  var hdr='time,angle,gyroRate,targetAngle,errAngle,pidOutput,position,posError,encSpeed,loopHz,zone,state,Kp,Ki,Kd\n';
  var csv=hdr+dat.join('\n');
  var a=document.createElement('a');
  a.href='data:text/csv;charset=utf-8,'+encodeURIComponent(csv);
  a.download='balancebot_'+Date.now()+'.csv';
  a.click();
}

function drawChart(){
  var c=document.getElementById('cv');
  var ctx=c.getContext('2d');
  var W=c.width=c.offsetWidth, H=c.height=160;
  ctx.fillStyle='#12141c'; ctx.fillRect(0,0,W,H);
  if(chartBuf.length<2) return;
  var mn=-15,mx=15;
  // Grid lines
  ctx.strokeStyle='#222'; ctx.lineWidth=1;
  for(var a=-10;a<=10;a+=5){
    var yy=H/2-(a/(mx-mn))*H*2;
    ctx.beginPath(); ctx.moveTo(0,yy); ctx.lineTo(W,yy); ctx.stroke();
  }
  ctx.strokeStyle='#333'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(0,H/2); ctx.lineTo(W,H/2); ctx.stroke();
  // Angle trace
  ctx.strokeStyle='#42a5f5'; ctx.lineWidth=1.5; ctx.beginPath();
  for(var i=0;i<chartBuf.length;i++){
    var x=i*W/400, y=H/2-(chartBuf[i]/(mx-mn))*H*2;
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
  // Labels
  ctx.fillStyle='#666'; ctx.font='11px monospace';
  ctx.fillText('+15',2,12); ctx.fillText('0',2,H/2-2); ctx.fillText('-15',2,H-4);
}
</script>
</body></html>
)rawliteral";

// ─────────────────────────────────────────────────────────────
// ESP-NOW receive callback (runs in WiFi event task — ISR-like)
// ─────────────────────────────────────────────────────────────
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataReceived(const esp_now_recv_info *info, const uint8_t *data, int len) {
    const uint8_t *mac = info->src_addr;
#else
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
#endif
    if (len != sizeof(TelemetryPacket)) return;  // wrong size — ignore

    // Learn Bot's MAC on first packet and register as peer for command replies
    if (!botPeerAdded) {
        memcpy(botMac, mac, 6);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, botMac, 6);
        peer.channel = 1;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
        botPeerAdded = true;
        Serial.printf("[ESP-NOW] Bot MAC learned: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      botMac[0],botMac[1],botMac[2],botMac[3],botMac[4],botMac[5]);
    }

    portENTER_CRITICAL(&pktMux);
    memcpy((void*)&latestPkt, data, sizeof(TelemetryPacket));
    newPktAvail = true;
    portEXIT_CRITICAL(&pktMux);
}

// ─────────────────────────────────────────────────────────────
// Send a command back to Bot ESP via ESP-NOW
// ─────────────────────────────────────────────────────────────
void sendCommandToBot(char cmdType, float value = 0.0f) {
    if (!botPeerAdded) {
        Serial.println("[CMD] Bot peer not registered yet — command dropped.");
        return;
    }
    CommandPacket cmd;
    cmd.cmdType = cmdType;
    cmd.value   = value;
    esp_now_send(botMac, (uint8_t*)&cmd, sizeof(cmd));
    Serial.printf("[CMD] Sent to Bot: %c = %.4f\n", cmdType, value);
}

// ─────────────────────────────────────────────────────────────
// WebSocket event handler (receives commands from browser)
// ─────────────────────────────────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type != WStype_TEXT || length < 1) return;
    char *data = (char*)payload;

    if (strcmp(data,"CAL")==0 || strcmp(data,"ARM")==0 || strcmp(data,"DIS")==0) {
        // State commands
        sendCommandToBot(data[0]); // 'C','A','D'
    } else if (length >= 2) {
        char cmd = data[0];
        float val = atof(data+1);
        switch (cmd) {
            case 'p': case 'i': case 'o':  // Inner PID
            case 'a': case 's': case 'd':  // Outer PID
                sendCommandToBot(cmd, val);
                break;
        }
    }
}

// ─────────────────────────────────────────────────────────────
// WiFi/WebSocket background task pinned to Core 0
// ─────────────────────────────────────────────────────────────
void serverTask(void *param) {
    for (;;) {
        httpServer.handleClient();
        wsServer.loop();

        // Check for new ESP-NOW data and broadcast to WebSocket clients
        portENTER_CRITICAL(&pktMux);
        bool hasData = newPktAvail;
        TelemetryPacket local;
        if (hasData) {
            memcpy(&local, (const void*)&latestPkt, sizeof(TelemetryPacket));
            newPktAvail = false;
        }
        portEXIT_CRITICAL(&pktMux);

        if (hasData) {
            char buf[200];
            // D,time,angle,gyroRate,targetAngle,errAngle,pidOutput,
            //   position,posError,encSpeed,loopHz,zone,state,Kp,Ki,Kd
            snprintf(buf, sizeof(buf),
                "D,%.3f,%.3f,%.2f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.0f,%d,%d,%.2f,%.1f,%.3f",
                local.timeS,  local.angle,   local.gyroRate,
                local.targetAngle, local.errAngle, local.pidOutput,
                local.position, local.posError, local.encSpeed, local.loopHz,
                (int)local.zone, (int)local.state,
                local.Kp_live, local.Ki_live, local.Kd_live);
            wsServer.broadcastTXT(buf);
        }

        delay(2);
    }
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n======== RECEIVER ESP — WiFi BRIDGE ========");

    // ── Start as AP ──
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 1);  // channel 1 must match Bot
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    // ── Print our own MAC (copy this into Bot_ESP.ino RECEIVER_MAC) ──
    Serial.print(">>> RECEIVER MAC (paste into Bot_ESP.ino): ");
    Serial.println(WiFi.macAddress());
    Serial.println(">>> Connect browser to: http://192.168.4.1");

    // ── ESP-NOW init ──
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED!");
        while(1);
    }
    esp_now_register_recv_cb(onDataReceived);
    Serial.println("[ESP-NOW] Receiver ready. Waiting for Bot packets...");

    // ── Web servers ──
    httpServer.on("/", [](){
        httpServer.send_P(200, "text/html", INDEX_HTML);
    });
    httpServer.begin();
    wsServer.begin();
    wsServer.onEvent(webSocketEvent);

    // ── Background server task on Core 0 ──
    xTaskCreatePinnedToCore(
        serverTask,   // function
        "serverTask", // name
        8192,         // stack
        NULL,         // param
        1,            // priority
        NULL,         // handle
        0             // Core 0
    );

    Serial.println("============================================\n");
}

// ─────────────────────────────────────────────────────────────
// MAIN LOOP (Core 1 — nearly empty, all work is in tasks)
// ─────────────────────────────────────────────────────────────
void loop() {
    // Nothing heavy here — all done in serverTask on Core 0
    // and ESP-NOW callback is interrupt-driven.
    delay(100);
}
