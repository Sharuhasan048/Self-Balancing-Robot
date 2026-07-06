// webpage.h - Embedded HTML dashboard for BalanceBot Data Logger
#ifndef WEBPAGE_H
#define WEBPAGE_H

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot Dashboard</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:#0f1117;color:#e0e0e0;padding:12px}
h1{font-size:1.3em;color:#7eb8ff;margin-bottom:8px}
h2{font-size:1em;color:#aaa;margin-bottom:6px;border-bottom:1px solid #333;padding-bottom:4px}
.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
#ws{padding:4px 10px;border-radius:12px;font-size:0.8em;font-weight:bold}
.on{background:#1b5e20;color:#a5d6a7}.off{background:#b71c1c;color:#ef9a9a}
.g{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.p{background:#1a1d27;border:1px solid #2a2d37;border-radius:8px;padding:12px}
.p.wide{grid-column:1/-1}
.dr{display:flex;justify-content:space-between;align-items:center;padding:3px 0;font-size:0.9em}
.dr label{color:#888;min-width:90px}
.dr span{color:#7eb8ff;font-family:monospace;font-size:1.05em}
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
</style></head><body>
<div class="top"><h1>&#9881; BalanceBot Dashboard</h1><span id="ws" class="off">DISCONNECTED</span></div>
<div class="g">

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
</div>

<div class="p">
<h2>&#127918; Robot Control</h2>
<div class="btns">
<button class="btn-y" onclick="ws('CAL')">Calibrate</button>
<button class="btn-g" onclick="ws('ARM')">Arm</button>
<button class="btn-r" onclick="ws('DIS')">Disarm</button>
</div>
<br>
<h2>&#128308; Data Recording</h2>
<div class="btns">
<button class="btn-g" id="rb" onclick="toggleRec()">Start Recording</button>
<button class="btn-p" onclick="dlCSV()">Download CSV</button>
</div>
<div id="rc">0 samples</div>
</div>

<div class="p">
<h2>&#128295; Inner Loop (Angle PID)</h2>
<div class="pr"><label>Kp:</label><input type="number" id="kp_a" step="0.5" value="15"><button class="btn-s" onclick="sp('p','kp_a')">Set</button></div>
<div class="pr"><label>Ki:</label><input type="number" id="ki_a" step="5" value="150"><button class="btn-s" onclick="sp('i','ki_a')">Set</button></div>
<div class="pr"><label>Kd:</label><input type="number" id="kd_a" step="0.05" value="0.8"><button class="btn-s" onclick="sp('o','kd_a')">Set</button></div>
<div class="pr"><label>N:</label><input type="number" id="n_a" step="1" value="15"><button class="btn-s" onclick="sp('n','n_a')">Set</button></div>
</div>

<div class="p">
<h2>&#128295; Outer Loop (Position PID)</h2>
<div class="pr"><label>Kp:</label><input type="number" id="kp_p" step="0.05" value="0"><button class="btn-s" onclick="sp('a','kp_p')">Set</button></div>
<div class="pr"><label>Ki:</label><input type="number" id="ki_p" step="0.01" value="0"><button class="btn-s" onclick="sp('s','ki_p')">Set</button></div>
<div class="pr"><label>Kd:</label><input type="number" id="kd_p" step="0.01" value="0"><button class="btn-s" onclick="sp('d','kd_p')">Set</button></div>
</div>

<div class="p wide">
<h2>&#128200; Angle Chart</h2>
<canvas id="cv"></canvas>
</div>
</div>

<script>
var sock=null,rec=false,dat=[],chartBuf=[];
var SN=['DISARMED','CALIBRATING','READY','ARMED'];

function conn(){
  sock=new WebSocket('ws://'+location.hostname+':81/');
  sock.onopen=function(){document.getElementById('ws').className='on';document.getElementById('ws').textContent='CONNECTED';};
  sock.onclose=function(){document.getElementById('ws').className='off';document.getElementById('ws').textContent='DISCONNECTED';setTimeout(conn,2000);};
  sock.onmessage=function(e){
    var m=e.data;
    if(m[0]==='D'){
      var v=m.substring(2).split(',');
      var f=v.map(parseFloat);
      document.getElementById('an').textContent=f[1].toFixed(2);
      document.getElementById('gr').textContent=f[2].toFixed(1);
      document.getElementById('ta').textContent=f[3].toFixed(2);
      document.getElementById('po').textContent=f[5].toFixed(1);
      document.getElementById('ps').textContent=f[6].toFixed(1);
      document.getElementById('es').textContent=f[8].toFixed(1);
      document.getElementById('hz').textContent=f[9].toFixed(0);
      chartBuf.push(f[1]);
      if(chartBuf.length>400)chartBuf.shift();
      drawChart();
      if(rec){dat.push(m.substring(2));document.getElementById('rc').textContent=dat.length+' samples';}
    } else if(m[0]==='S'){
      var si=parseInt(m.substring(2));
      document.getElementById('st').textContent=SN[si]||'??';
    } else if(m[0]==='C'){
      var v=m.substring(2).split(',');
      document.getElementById('kp_a').value=v[0];
      document.getElementById('ki_a').value=v[1];
      document.getElementById('kd_a').value=v[2];
      document.getElementById('n_a').value=v[3];
      document.getElementById('kp_p').value=v[4];
      document.getElementById('ki_p').value=v[5];
      document.getElementById('kd_p').value=v[6];
    }
  };
}
conn();

function ws(m){if(sock&&sock.readyState===1)sock.send(m);}
function sp(c,id){ws(c+document.getElementById(id).value);}

function toggleRec(){
  rec=!rec;
  var b=document.getElementById('rb');
  if(rec){dat=[];b.textContent='Stop Recording';b.className='btn-r';document.getElementById('rc').textContent='0 samples';}
  else{b.textContent='Start Recording';b.className='btn-g';}
}

function dlCSV(){
  if(dat.length===0){alert('No data recorded!');return;}
  var hdr='time,angle,gyroRate,targetAngle,errAngle,pidOutput,position,posError,encSpeed,loopHz\n';
  var csv=hdr+dat.join('\n');
  var a=document.createElement('a');
  a.href='data:text/csv;charset=utf-8,'+encodeURIComponent(csv);
  a.download='balance_log_'+Date.now()+'.csv';
  a.click();
}

function drawChart(){
  var c=document.getElementById('cv');
  var ctx=c.getContext('2d');
  var W=c.width=c.offsetWidth,H=c.height=160;
  ctx.fillStyle='#12141c';ctx.fillRect(0,0,W,H);
  if(chartBuf.length<2)return;
  var mn=-15,mx=15;
  ctx.strokeStyle='#333';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(0,H/2);ctx.lineTo(W,H/2);ctx.stroke();
  ctx.strokeStyle='#222';
  for(var a=-10;a<=10;a+=5){var yy=H/2-(a/(mx-mn))*H*2;ctx.beginPath();ctx.moveTo(0,yy);ctx.lineTo(W,yy);ctx.stroke();}
  ctx.strokeStyle='#42a5f5';ctx.lineWidth=1.5;ctx.beginPath();
  for(var i=0;i<chartBuf.length;i++){
    var x=i*W/400,y=H/2-(chartBuf[i]/(mx-mn))*H*2;
    if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
  }
  ctx.stroke();
  ctx.fillStyle='#666';ctx.font='11px monospace';
  ctx.fillText('+15',2,12);ctx.fillText('0',2,H/2-2);ctx.fillText('-15',2,H-4);
}
</script>
</body></html>
)rawliteral";

#endif
