#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

// --- PINES HARDWARE (ESP32 38-PIN) ---
const int ENA=25, IN1=26, IN2=17, IN3=14, IN4=13, ENB=27;
const int P_FL=33, P_RL=18, P_FR=34, P_RR=35;

// --- CONSTANTES FÍSICAS ---
const float R_RUEDA = 0.0325, L_EJE = 0.165, PPR = 20.0;
const float M_TICK = (2.0 * PI * R_RUEDA) / PPR;

// --- PARÁMETROS MPC Y TRAYECTORIA ---
float q_pos = 5.0, q_theta = 12.0, r_v = 0.4, r_w = 0.2;
float v_crucero = 0.20, escala_ref = 1.0, L_dinamico = 0.165; 

// --- ESTADO DEL ROBOT ---
Adafruit_MPU6050 mpu;
WebServer server(80);
volatile long encFL=0, encRL=0, encFR=0, encRR=0;
long pEncL=0, pEncR=0;
float x=0, y=0, th=0, gZ_off=0;
int outL=0, outR=0;
unsigned long lastT = 0;
bool autoM = false;
String traj = "MARIPOSA";
float t_ref = 0;

void IRAM_ATTR iFL(){encFL++;} void IRAM_ATTR iRL(){encRL++;}
void IRAM_ATTR iFR(){encFR++;} void IRAM_ATTR iRR(){encRR++;}

// --- DASHBOARD WEB ACTUALIZADO ---
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8">
<style>
  body { font-family: 'Segoe UI', sans-serif; background: #0d1117; color: #c9d1d9; margin: 0; padding: 10px; font-size: 12px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 10px; }
  .card { background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 12px; }
  .val { font-family: monospace; color: #58a6ff; font-weight: bold; }
  .slider-container { margin: 8px 0; border-bottom: 1px solid #21262d; padding-bottom: 5px; }
  .slider-label { display: flex; justify-content: space-between; font-weight: bold; }
  .desc { font-size: 0.8em; color: #8b949e; display: block; margin-top: 2px; line-height: 1.1; }
  .math { font-family: 'Courier New', Courier, monospace; color: #f8e3a1; font-size: 0.9em; background: #000; padding: 5px; border-radius: 4px; display: block; margin-top: 5px; }
  button { padding: 8px; margin: 4px; border-radius: 5px; border: none; cursor: pointer; font-weight: bold; width: 31%; font-size: 10px; }
  .btn-auto { width: 97%; background: #238636; color: white; font-size: 1.2em; margin-bottom: 10px; }
  .btn-traj { background: #30363d; color: white; }
  .btn-traj.active { background: #58a6ff; color: #000; }
  .slider { width: 100%; height: 6px; border-radius: 5px; background: #30363d; outline: none; margin: 5px 0; }
  #map { background: #000; border: 1px solid #444; display: block; margin: 10px auto; border-radius: 5px; }
  table { width: 100%; border-collapse: collapse; }
  td { padding: 4px; border-bottom: 1px solid #21262d; }
</style></head>
<body>
  <div class="grid">
    <div class="card">
      <button class="btn-auto" id="toggleBtn" onclick="fetch('/toggle')">START AUTO</button>
      <div style="display: flex; justify-content: space-around;">
        <button class="btn-traj" id="btn_CIRCULO" onclick="setT('CIRCULO')">CÍRCULO</button>
        <button class="btn-traj" id="btn_TREBOL" onclick="setT('TREBOL')">TRÉBOL</button>
        <button class="btn-traj active" id="btn_MARIPOSA" onclick="setT('MARIPOSA')">MARIPOSA</button>
      </div>
      <button style="background:#da3633; color:white; width:97%" onclick="fetch('/reset')">RESET TOTAL</button>
    </div>

    <div class="card">
      <h3>Sintonización de Costos MPC</h3>
      <div class="slider-container">
        <div class="slider-label"><span>QP (Posición)</span><span class="val" id="d_qp">5.0</span></div>
        <span class="desc">Controla la precisión del seguimiento de coordenadas X,Y.</span>
        <input type="range" class="slider" min="0" max="150" value="50" oninput="u('qp',this.value/10)">
      </div>
      <div class="slider-container">
        <div class="slider-label"><span>QT (Theta)</span><span class="val" id="d_qt">12.0</span></div>
        <span class="desc">Controla la orientación del robot respecto a la curva.</span>
        <input type="range" class="slider" min="0" max="200" value="120" oninput="u('qt',this.value/10)">
      </div>
      <div class="slider-container">
        <div class="slider-label"><span>RV (Suavizado V)</span><span class="val" id="d_rv">0.4</span></div>
        <span class="desc">Penaliza aceleraciones lineales bruscas.</span>
        <input type="range" class="slider" min="0" max="100" value="4" oninput="u('rv',this.value/10)">
      </div>
      <div class="slider-container">
        <div class="slider-label"><span>RW (Suavizado W)</span><span class="val" id="d_rw">0.2</span></div>
        <span class="desc">Evita oscilaciones y giros violentos.</span>
        <input type="range" class="slider" min="0" max="100" value="2" oninput="u('rw',this.value/10)">
      </div>
      <div class="slider-container">
        <div class="slider-label"><span>VEL Crucero</span><span class="val" id="d_v">0.20</span></div>
        <span class="desc">Velocidad lineal base para la trayectoria.</span>
        <input type="range" class="slider" min="0" max="100" value="20" oninput="u('v',this.value/100)">
      </div>
      <div class="slider-container">
        <div class="slider-label"><span>ESCALA (a)</span><span class="val" id="d_sc">1.0</span></div>
        <span class="desc">Multiplicador del tamaño de la referencia.</span>
        <input type="range" class="slider" min="5" max="50" value="10" oninput="u('sc',this.value/10)">
      </div>
      <div class="slider-container">
        <div class="slider-label"><span>ANCHO EJE (L)</span><span class="val" id="d_le">0.165</span></div>
        <span class="desc">Distancia efectiva para cálculo cinemático.</span>
        <input type="range" class="slider" min="100" max="300" value="165" oninput="u('le',this.value/1000)">
      </div>
    </div>

    <div class="card">
      <h3>Ecuaciones de Referencia</h3>
      <div id="math_ref">
        <span class="desc"><b>Modo Actual:</b> <span id="traj_label">MARIPOSA</span></span>
        <div id="math_content" class="math">
          r = exp(sin(t)) - 2cos(4t) + sin((2t - π)/24)^5<br>
          x(t) = a * r * cos(t)<br>
          y(t) = a * r * sin(t)
        </div>
      </div>
      <h3 style="margin-top:15px">Telemetría de Componentes</h3>
      <table>
        <tr><td>X / Y [m]</td><td class="val"><span id="vx">0</span>, <span id="vy">0</span></td></tr>
        <tr><td>θ [deg]</td><td class="val"><span id="vth">0</span>°</td></tr>
        <tr><td>Encoders L | R</td><td class="val"><span id="efl">0</span>|<span id="erl">0</span> : <span id="efr">0</span>|<span id="err">0</span></td></tr>
        <tr><td>Accel Z / Temp</td><td class="val"><span id="az">0</span> m/s² | <span id="mt">0</span>°C</td></tr>
        <tr><td>Salida PWM</td><td class="val">L:<span id="pL">0</span> | R:<span id="pR">0</span></td></tr>
      </table>
    </div>

    <div class="card">
      <h3>Monitoreo de Avance (X vs Y)</h3>
      <canvas id="map" width="400" height="400"></canvas>
    </div>
  </div>

<script>
const ctx = document.getElementById("map").getContext("2d");
let path = []; let curT = "MARIPOSA"; let curS = 1.0;

function u(p,v){ 
  document.getElementById('d_'+p).innerText = v;
  if(p=='sc') curS = v;
  fetch(`/setParam?p=${p}&v=${v}`); 
}

function setT(t){
  curT = t;
  document.getElementById('traj_label').innerText = t;
  const content = document.getElementById('math_content');
  if(t === 'MARIPOSA') content.innerHTML = "r = exp(sin(t)) - 2cos(4t) + sin((2t - π)/24)^5<br>x(t) = a * r * cos(t)<br>y(t) = a * r * sin(t)";
  if(t === 'TREBOL') content.innerHTML = "r = a * cos(2t)<br>x(t) = r * cos(t)<br>y(t) = r * sin(t)";
  if(t === 'CIRCULO') content.innerHTML = "x(t) = a * cos(t)<br>y(t) = a * sin(t)";

  document.querySelectorAll('.btn-traj').forEach(b => b.classList.remove('active'));
  document.getElementById('btn_'+t).classList.add('active');
  fetch(`/setTraj?t=${t}`);
}

function draw(d){
  ctx.clearRect(0,0,400,400); ctx.save(); ctx.translate(200, 200);
  ctx.beginPath(); ctx.strokeStyle = "#444"; ctx.setLineDash([5, 5]);

  if(curT === "MARIPOSA") {
    for(let t=0; t<12*Math.PI; t+=0.1){
      let r = curS * (Math.exp(Math.sin(t)) - 2*Math.cos(4*t) + Math.pow(Math.sin((2*t - Math.PI)/24.0), 5));
      ctx.lineTo(r * Math.cos(t) * 20, -r * Math.sin(t) * 20);
    }
  } else if(curT === "TREBOL") {
    for(let t=0; t<2*Math.PI; t+=0.05){
      let r = curS * Math.cos(2*t);
      ctx.lineTo(r * Math.cos(t) * 80, -r * Math.sin(t) * 80);
    }
  } else {
    ctx.arc(0, 0, curS * 60, 0, 2*Math.PI);
  }
  ctx.stroke(); ctx.setLineDash([]);

  path.push({x:d.x, y:d.y}); if(path.length > 800) path.shift();
  ctx.beginPath(); ctx.strokeStyle = "#00ffff"; ctx.lineWidth = 2;
  for(let i=0; i<path.length; i++){ ctx.lineTo(path[i].x*100, -path[i].y*100); }
  ctx.stroke();

  ctx.beginPath(); ctx.arc(d.x*100, -d.y*100, 6, 0, 2*Math.PI); ctx.fillStyle = "red"; ctx.fill();
  ctx.beginPath(); ctx.moveTo(d.x*100, -d.y*100);
  ctx.lineTo((d.x + 0.2*Math.cos(d.th*Math.PI/180))*100, -(d.y + 0.2*Math.sin(d.th*Math.PI/180))*100);
  ctx.strokeStyle = "yellow"; ctx.lineWidth = 3; ctx.stroke();
  ctx.restore();
}

setInterval(()=>{
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('vx').innerText=d.x.toFixed(2); document.getElementById('vy').innerText=d.y.toFixed(2);
    document.getElementById('vth').innerText=d.th.toFixed(1); document.getElementById('efl').innerText=d.fl;
    document.getElementById('erl').innerText=d.rl; document.getElementById('efr').innerText=d.fr;
    document.getElementById('err').innerText=d.rr; document.getElementById('az').innerText=d.az.toFixed(1);
    document.getElementById('mt').innerText=d.mt.toFixed(1); document.getElementById('pL').innerText=d.pL;
    document.getElementById('pR').innerText=d.pR;
    
    let btn = document.getElementById('toggleBtn');
    if(d.auto){ btn.innerText="STOP AUTO"; btn.style.background="#da3633"; }
    else { btn.innerText="START AUTO"; btn.style.background="#238636"; }
    draw(d);
  });
},200);
</script>
</body></html>
)rawliteral";

void drive(float v, float w) {
  float vL = v - (w * L_dinamico / 2.0), vR = v + (w * L_dinamico / 2.0);
  outL = constrain(vL * 3000, -1023, 1023); outR = constrain(vR * 3000, -1023, 1023);
  digitalWrite(IN1, outL>=0?HIGH:LOW); digitalWrite(IN2, outL>=0?LOW:HIGH);
  digitalWrite(IN3, outR>=0?HIGH:LOW); digitalWrite(IN4, outR>=0?LOW:HIGH);
  ledcWrite(ENA, abs(outL)); ledcWrite(ENB, abs(outR));
}

void loopMPC() {
  float xr, yr;
  if (traj == "MARIPOSA") {
    float r = escala_ref * (exp(sin(t_ref)) - 2*cos(4*t_ref) + pow(sin((2*t_ref - PI)/24.0), 5));
    xr = r * cos(t_ref); yr = r * sin(t_ref);
  } else if (traj == "TREBOL") {
    float r = escala_ref * cos(2.0 * t_ref);
    xr = r * cos(t_ref); yr = r * sin(t_ref);
  } else {
    xr = escala_ref * cos(t_ref); yr = escala_ref * sin(t_ref);
  }
  
  float ex = xr - x, ey = yr - y;
  float th_r = atan2(ey, ex), eth = th_r - th;
  while(eth > PI) eth -= 2*PI; while(eth < -PI) eth += 2*PI;
  float v = (q_pos * v_crucero) / (q_pos + r_v);
  float w = (q_theta * eth) / (q_theta + r_w);
  drive(v, w); t_ref += 0.02;
}

void setup() {
  Serial.begin(115200); Wire.begin(21, 22);
  pinMode(IN1, 1); pinMode(IN2, 1); pinMode(IN3, 1); pinMode(IN4, 1);
  ledcAttach(ENA, 20000, 10); ledcAttach(ENB, 20000, 10);
  pinMode(P_FL, 0x05); pinMode(P_RL, 0x05); pinMode(P_FR, 0x01); pinMode(P_RR, 0x01);
  attachInterrupt(P_FL, iFL, 1); attachInterrupt(P_RL, iRL, 1);
  attachInterrupt(P_FR, iFR, 1); attachInterrupt(P_RR, iRR, 1);
  if(mpu.begin()){
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    for(int i=0;i<400;i++){ sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t); gZ_off+=g.gyro.z; delay(2); }
    gZ_off /= 400.0;
  }
  WiFi.softAP("ROBOT_MPC_MASTER", "12345678");
  server.on("/", [](){ server.send_P(200, "text/html", webpage); });
  server.on("/data", [](){
    sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    String j = "{\"x\":"+String(x)+",\"y\":"+String(y)+",\"th\":"+String(th*180/PI)+",\"fl\":"+String(encFL)+",\"rl\":"+String(encRL)+",\"fr\":"+String(encFR)+",\"rr\":"+String(encRR)+",\"az\":"+String(a.acceleration.z)+",\"mt\":"+String(t.temperature)+",\"pL\":"+String(outL)+",\"pR\":"+String(outR)+",\"auto\":"+String(autoM)+"}";
    server.send(200, "application/json", j);
  });
  server.on("/setParam", [](){
    String p=server.arg("p"); float v=server.arg("v").toFloat();
    if(p=="qp") q_pos=v; else if(p=="qt") q_theta=v; else if(p=="rv") r_v=v; else if(p=="rw") r_w=v; else if(p=="v") v_crucero=v; else if(p=="sc") escala_ref=v; else if(p=="le") L_dinamico=v;
    server.send(200);
  });
  server.on("/reset", [](){ x=y=th=t_ref=encFL=encRL=encFR=encRR=pEncL=pEncR=0; server.send(200); });
  server.on("/toggle", [](){ autoM=!autoM; if(!autoM) drive(0,0); server.send(200); });
  server.on("/setTraj", [](){ traj=server.arg("t"); t_ref=0; server.send(200); });
  server.begin(); lastT = millis();
}

void loop() {
  server.handleClient();
  unsigned long n = millis(); float dt = (n-lastT)/1000.0; lastT = n;
  sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
  float gz_f = g.gyro.z - gZ_off; if(abs(gz_f)>0.05) th += gz_f * dt;
  long cL=(encFL+encRL)/2, cR=(encFR+encRR)/2;
  float dC=((cL-pEncL)+(cR-pR))*M_TICK/2.0; pEncL=cL; pEncR=cR;
  x += dC*cos(th); y += dC*sin(th);
  if(autoM) loopMPC();
  delay(20);
}