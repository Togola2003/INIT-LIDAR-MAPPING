// ============================================================================
//  Télécommande servie par le robot lui-même : sur le téléphone, se connecter au Wi-Fi du robot puis
//  ouvrir http://192.168.4.1 dans le navigateur. Rien à installer.
//  Maintenir un bouton = le robot avance tant que le doigt reste appuyé (ordres de 400 ms renouvelés
//  toutes les 250 ms) ; lever le doigt = arrêt immédiat. Le scan du lidar s'affiche en direct.
// ============================================================================
#pragma once

const char PAGE_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot LD06</title>
<style>
:root{--bg:#f5f5f7;--card:#fff;--ink:#1d1d1f;--muted:#6e6e73;--accent:#007aff;--stop:#ff3b30;--line:#e5e5ea}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;user-select:none;-webkit-user-select:none}
body{margin:0;font-family:-apple-system,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--ink);padding:14px}
h1{font-size:20px;margin:4px 0 2px}.muted{color:var(--muted);font-size:13px}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px;margin-top:12px}
canvas{width:100%;aspect-ratio:1;background:#fff;border-radius:12px;display:block}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:4px}
.pad button{height:72px;font-size:28px;border:0;border-radius:16px;background:#f2f2f7;color:var(--ink);touch-action:none}
.pad button.on{background:var(--accent);color:#fff}
.pad .stop{background:var(--stop);color:#fff;font-size:22px;font-weight:600}
input[type=range]{width:100%}
.row{display:flex;justify-content:space-between;align-items:center}
</style></head><body>
<h1>Robot LD06</h1><div class="muted" id="status">connexion…</div>
<div class="card"><canvas id="cv" width="600" height="600"></canvas>
<div class="muted" style="margin-top:6px">Scan en direct · le robot regarde vers le haut · cercles tous les 1 m</div></div>
<div class="card">
<div class="pad">
<button data-m="rotateleft">⟲</button><button data-m="forward">↑</button><button data-m="rotateright">⟳</button>
<button data-m="left">←</button><button class="stop" id="stop">STOP</button><button data-m="right">→</button>
<div></div><button data-m="backward">↓</button><div></div>
</div>
<div class="row" style="margin-top:14px"><span>Vitesse</span><b id="sv">150</b></div>
<input type="range" id="speed" min="60" max="255" value="150">
<div class="muted">Maintenir un bouton pour rouler, lever le doigt pour s'arrêter.</div>
</div>
<script>
const $=id=>document.getElementById(id);let timer=null;
const send=(m)=>fetch('/'+m+'?time=400').catch(()=>{});
function stop(){if(timer){clearInterval(timer);timer=null}document.querySelectorAll('.pad button').forEach(b=>b.classList.remove('on'));fetch('/stop').catch(()=>{})}
document.querySelectorAll('[data-m]').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();stop();b.classList.add('on');send(b.dataset.m);timer=setInterval(()=>send(b.dataset.m),250)});
 ['pointerup','pointerleave','pointercancel'].forEach(t=>b.addEventListener(t,()=>{if(b.classList.contains('on'))stop()}));
});
$('stop').addEventListener('pointerdown',e=>{e.preventDefault();stop()});
$('speed').addEventListener('input',e=>{$('sv').textContent=e.target.value});
$('speed').addEventListener('change',e=>fetch('/speed?v='+e.target.value).catch(()=>{}));
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop()});
const cv=$('cv'),ctx=cv.getContext('2d');
function draw(buf){const v=new DataView(buf);const n=v.getUint16(6,true);const W=cv.width,c=W/2,R=6,s=c/R;
 ctx.clearRect(0,0,W,W);ctx.strokeStyle='#e5e5ea';ctx.lineWidth=1;
 for(let r=1;r<=R;r++){ctx.beginPath();ctx.arc(c,c,r*s,0,7);ctx.stroke()}
 ctx.fillStyle='#1d1d1f';
 for(let k=0;k<n;k++){const o=8+5*k,a=v.getUint16(o,true)/100*Math.PI/180,d=v.getUint16(o+2,true)/1000;
  if(d<=0.02)continue;const x=d*Math.cos(a),y=d*Math.sin(a);ctx.fillRect(c-y*s-2,c-x*s-2,4,4)}
 ctx.fillStyle='#ff3b30';ctx.beginPath();ctx.moveTo(c,c-14);ctx.lineTo(c-9,c+9);ctx.lineTo(c+9,c+9);ctx.fill()}
async function poll(){try{const r=await fetch('/scan');if(r.ok)draw(await r.arrayBuffer())}catch(e){}setTimeout(poll,300)}
async function status(){try{const j=await (await fetch('/status')).json();
 $('status').textContent=`connecté · lidar ${j.points} points, ${j.rpm} tr/min · vitesse ${j.speed}${j.moving?' · en mouvement':''}`;
 $('speed').value=j.speed;$('sv').textContent=j.speed}catch(e){$('status').textContent='robot injoignable : es-tu sur son Wi-Fi ?'}setTimeout(status,1000)}
poll();status();
</script></body></html>)rawliteral";
