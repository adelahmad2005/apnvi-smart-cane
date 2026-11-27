"""
Stage 2 live view in the web browser: the live colour camera with the answer's red box on it.

Test and demo helper, like Stage 1's live view. The picture never freezes: the red box is drawn
on top of the live camera picture (changed 2026-10-12 at Adel's request). Between answers the
fast follower (follower.py, added 2026-10-13) moves the box with the object on every picture,
so it does not jump every 2 seconds. Below the picture: a text box and four buttons:
  Ask once  asks the typed question one time (says "not found" if missing)
  Listening mode  works like the handle button (Adel, 26 Oct): click once to start listening,
                 click again to send; or hold it and let go to send. The Jetson's microphone
                 (the AirPods) records, not the laptop's or phone's. Listening stops by itself
                 after 15 s
  Track     keeps asking it with the newest picture, so the box keeps following the thing
  Stop      stops tracking
If the follower loses the object, the box disappears until the next answer. While tracking,
one missed answer does not hide the box; two in a row do.

A strip under the picture shows Stage 1 at the same time (22 Oct, both stages on one page):
its level (clear, notice, warning, stop), the distance and the camera status. It turns grey
with "no data" if Stage 1 has sent nothing for STAGE1_STALE_S seconds. When Stage 1 sees
something closing in from a side, that edge of the picture turns red and the strip says
"COMING IN FROM THE LEFT" (or right), as on Stage 1's own view.

Next to the camera picture is Stage 3's map (29 Oct, Adel): the map, a dot and an arrow for the
device. It comes from Stage 3's map_view_node on /stage3/map_image, a topic this page listens to
only while a browser has the page open. It says "No map" when Stage 3 is not running.
A phone shows it below the camera picture.
Map dots and zoom (17 Nov, Adel): the map shows only coloured dots, no words. Tap a
dot and the line under the map gives its name ("bin", "ESB117"); a yellow ring marks it.
"Zoom in" (or the mouse wheel) asks Stage 3 to draw the map 2, 4 or 8 times closer (on
/stage3/map_zoom), so dots that sit close get room; "Zoom out" and "Whole map" go back.
Press on the map and drag (mouse or finger, at any zoom) to move it, like a web map: the
picture moves with the pointer at once, and on letting go Stage 3 draws the new view. A drag
is never a tap. Stage 3 sends each dot's place with the picture (/stage3/map_view); this page
passes it on in the X-Map-View header. An open tab with an older page reloads by itself
(the X-Page header), so a restart never leaves an old script running.

Sound through the browser (15 Nov; stage2_node with sound_out:=browser): the page is
the device's mouth and ears. After one click on "Sound on" it plays every sentence the device
says (quieter at Stage 1's warning, cut off at stop) and, while the talk button is held (the
grip's red button, or Listening mode here), records with this computer's microphone and sends
the recording to the Jetson. Browsers only give the microphone to a page opened as
http://localhost:8081 (use `ssh -L 8081:localhost:8081 jetson` from another computer) or over
https; opened by the Jetson's IP address the page still plays sound but cannot record.

Under the buttons, one line says what the device is doing (16 Nov): the guide's
state ("Guiding to the door, 3.2 m to go"), and the last sentence the device said.

Nothing is drawn while no browser has the page open, so it costs almost nothing then.

Open http://localhost:8081, or from a laptop on the same wifi: http://<the Jetson's IP>:8081
Stage 1's own depth view (port 8080) is off by default when started with stage2.launch.py.

Settings (--ros-args -p name:=value):
  color_topic   default /camera/camera/color/image_raw
  port          default 8081
  width         width of the picture sent to the browser, default 640 (smaller = smoother
                over a weak wifi or hotspot)
"""

from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import tempfile
import threading
import time

from apnvi_stage1.hazard_node import NEWEST_PICTURE
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String

from .finder import BOX_COLOUR, BOX_THICKNESS
from .follower import FollowerWorker, TRACK_WIDTH
from .stage2_node import stamp_of

PORT = 8081
WIDTH = 640                             # every frame is scaled to this width (setting: width)
FRAMES_PER_S = 15                       # pictures drawn and sent to the browser per second
JPEG_QUALITY = 70
SHOW_ANSWER_S = 10.0                    # a one-off answer's box stays this long
MISSES_ALLOWED = 1                      # while tracking, "not found" answers that keep the box
MAX_QUESTION_CHARS = 200
WATCHING_S = 3.0                        # a browser asked for a picture this recently = watching
STAGE1_STALE_S = 2.0                    # no Stage 1 message for this long = show "no data"
SIDE_SHOW_S = 1.0                       # a side warning stays on screen this long
SIDE_BAR_PX = 24                        # width of the red bar on that edge of the picture
MAP_TOPIC = '/stage3/map_image'         # Stage 3's map picture (a JPEG)
MAP_STALE_S = 3.0                       # no map picture for this long = show "No map"
MAP_LET_GO_S = 5.0                      # stop listening this long after the last viewer
MAP_VIEW_TOPIC = '/stage3/map_view'     # what the map picture shows, and where its dots are
MAP_ZOOM_TOPIC = '/stage3/map_zoom'     # the page asks Stage 3 to draw closer in
MAP_MAX_ZOOM = 8
MAX_MAP_VIEW_CHARS = 30000              # a bigger view (very many pins) is not passed on
SOUND_WAIT_S = 20.0                     # a browser waits this long for the next sound event
SOUNDS_KEPT = 30                        # sound events (and their files) kept for late browsers
MAX_RECORDING_BYTES = 2000000           # 16 kHz, 16-bit, one channel: about a minute
LEVEL_COLOURS = {'clear': (80, 170, 60), 'notice': (0, 200, 230),      # same as Stage 1's view
                 'warning': (0, 130, 255), 'stop': (40, 40, 220)}
STATUS_COLOURS = {'listening': (0, 200, 230), 'thinking': (0, 130, 255),
                  'tracking': (200, 120, 0), 'ready': (80, 170, 60)}

PAGE = b'''<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>APNVI Stage 2</title>
<style>
 body{background:#222;color:#eee;font-family:sans-serif;margin:0;text-align:center}
 img{max-width:100%}
 #views{display:flex;flex-wrap:wrap;justify-content:center;align-items:flex-start;gap:8px}
 #mapbox{width:min(480px,95vw);min-height:120px;color:#aaa}
 #mapon{display:none}
 #mapwrap{line-height:0;overflow:hidden;aspect-ratio:1/1;background:#5a5a5a}  /* map only */
 #mapmove{position:relative;aspect-ratio:1/1;overflow:hidden}  /* the map square only: the
                                       legend under it must not turn into view (19 Nov) */
 #map{display:block;width:100%;cursor:grab;touch-action:none;user-select:none;
      -webkit-user-select:none;-webkit-user-drag:none}     /* press and drag to move the map */
 #mapbox.big{width:min(1100px,98vw)}           /* 17 Nov (Adel): tap the map to enlarge */
 #mapbox.dragging #map{cursor:grabbing}
 #legwrap{line-height:0;overflow:hidden;aspect-ratio:480/80}   /* the legend: last 80 rows */
 #legend{display:block;width:100%;margin-top:-100%}
 #ring{position:absolute;display:none;width:22px;height:22px;margin:-14px 0 0 -14px;
       border:3px solid #ff0;border-radius:50%;box-shadow:0 0 0 2px #000;pointer-events:none}
 #mapwrap{position:relative}
 #turnbox{position:absolute;right:4px;top:8px;width:36px;display:flex;flex-direction:column;
          align-items:center;gap:3px;background:rgba(40,40,40,.55);border-radius:6px;
          padding:4px 0;line-height:1;z-index:2}             /* 19 Nov: a small lever on the map */
 #turn{writing-mode:vertical-lr;direction:rtl;height:110px;width:18px;margin:0;cursor:pointer}
 #turn0{font-size:10px;padding:1px 2px;margin:0;line-height:1.2}
 #mapbar button{font-size:15px;padding:4px 10px;margin:4px 2px}
 #zlevel{color:#aaa;font-size:14px}
 #pininfo{min-height:1.4em;color:#ff8;padding:2px}
 form{display:flex;gap:8px;justify-content:center;flex-wrap:wrap;padding:12px}
 input{font-size:18px;padding:8px;width:min(480px,90vw)}
 button{font-size:18px;padding:8px 16px;cursor:pointer}
 #msg{min-height:1.5em;color:#aaa}
 #state{min-height:1.5em;color:#8cf;padding:4px}
 #talk{touch-action:none;user-select:none;-webkit-user-select:none;background:#2a6;color:#fff;
       border:none;border-radius:4px}
 #talk.on{background:#c33}
</style></head>
<body>
<div id="views">
 <img id="live" alt="live camera">
 <div id="mapbox"><div id="mapon">
  <div id="mapwrap"><div id="mapmove"><img id="map" alt="map" draggable="false">
   <div id="ring"></div></div>
   <div id="turnbox"><input type="range" id="turn" min="-180" max="180" step="1" value="0"
    title="Drag to turn the map"><button type="button" id="turn0"
    title="Back to north up">0&deg;</button></div></div>
  <div id="legwrap"><img id="legend" alt="legend"></div>
  <div id="mapbar"><button type="button" id="zin">Zoom in</button>
   <button type="button" id="zout">Zoom out</button>
   <button type="button" id="zall">Whole map</button>
   <button type="button" id="zbig">Bigger</button> <span id="zlevel"></span></div>
  <div id="pininfo">Tap a dot to see what it is. Press on the map and drag to move it.</div>
  </div>
  <div id="mapnote">No map (Stage 3 is not running)</div></div>
</div>
<form onsubmit="send('/ask');return false">
 <input id="q" placeholder="where is the exit" autocomplete="off">
 <button type="submit">Ask once</button>
 <button id="talk" type="button" oncontextmenu="return false">Listening mode</button>
 <button type="button" onclick="send('/track')">Track</button>
 <button type="button" onclick="send('/stop')">Stop</button>
 <button id="snd" type="button">Sound on</button>
</form>
<div id="msg"></div>
<div id="state"></div>
<script>
// Ask for the newest picture, show it, then ask for the next one. Pictures never pile up,
// so the video is never behind real life; a slow connection only means fewer per second.
var shown=0, live=document.getElementById('live');
function nextFrame(){
  fetch('/frame?after='+shown,{cache:'no-store'}).then(function(r){
    shown=parseInt(r.headers.get('X-Frame'))||shown; return r.blob();
  }).then(function(b){
    var url=URL.createObjectURL(b);
    live.onload=function(){URL.revokeObjectURL(url); nextFrame();};
    live.src=url;
  }).catch(function(){setTimeout(nextFrame,500);});
}
nextFrame();
// The map (Stage 3) next to the camera: ask ten times a second (it sends 10 at most).
// No answer = "No map". The X-Map-View header says what the picture shows:
// the box in metres, the zoom, and each dot's place in the picture's own pixels. The picture
// is the map with the legend under it: #mapwrap shows the map part, #legwrap the legend.
var mapImg=document.getElementById('map'), mapNote=document.getElementById('mapnote');
var mapOn=document.getElementById('mapon'), mapBox=document.getElementById('mapbox');
var ring=document.getElementById('ring'), pinInfo=document.getElementById('pininfo');
var mapMove=document.getElementById('mapmove'), legImg=document.getElementById('legend');
var mapView=null, picked=null, lastWheel=0, drag=null, held=null, dragged=0;
var PAGE_VERSION='__PAGE_VERSION__';   // the server says its own in X-Page: differs = reload
// 19 Nov (Adel): a lever right of the map turns it. The turn is done here in the browser
// (CSS, round the map's middle); taps, drags and the wheel are turned back before use.
var mapWrap=document.getElementById('mapwrap'), turnIn=document.getElementById('turn');
var turnDeg=0, moveX=0, moveY=0;
try{turnDeg=parseFloat(localStorage.getItem('mapTurn'))||0;}catch(err){}
function unturn(x,y){         // a screen offset, turned back onto the map
  var a=-turnDeg*Math.PI/180;
  return [x*Math.cos(a)-y*Math.sin(a), x*Math.sin(a)+y*Math.cos(a)];
}
function place(){
  var w=mapWrap.clientWidth;
  mapMove.style.transformOrigin=(w/2)+'px '+(w/2)+'px';
  mapMove.style.transform='translate('+moveX+'px,'+moveY+'px) rotate('+turnDeg+'deg)';
}
function setTurn(d){
  turnDeg=Math.max(-180,Math.min(180,d)); turnIn.value=turnDeg;
  turnIn.title='Map turned '+Math.round(turnDeg)+' degrees (drag to turn)';
  document.getElementById('turn0').textContent=Math.round(turnDeg)+'\\u00b0';   // the angle
  try{localStorage.setItem('mapTurn',turnDeg);}catch(err){}
  place();
}
turnIn.oninput=function(){setTurn(parseFloat(turnIn.value)||0);};
document.getElementById('turn0').onclick=function(){setTurn(0);};
window.addEventListener('resize',place);
setTurn(turnDeg);
function spot(e){             // where the pointer is, in the picture's own pixels
  var r=mapWrap.getBoundingClientRect(), w=r.width, k=mapImg.naturalWidth/w;
  var u=unturn(e.clientX-r.left-w/2-moveX, e.clientY-r.top-w/2-moveY);
  return [(u[0]+w/2)*k,(u[1]+w/2)*k];
}
function pinAt(p){            // the dot nearest p, within 16 pixels
  if(!mapView||!mapView.pins)return null;
  var best=null, near=16;
  mapView.pins.forEach(function(q){
    var d=Math.hypot(q.col-p[0],q.row-p[1]); if(d<near){near=d; best=q;}
  });
  return best;
}
function moveBy(dx,dy){moveX=dx||0; moveY=dy||0; place();}
function metres(p){           // the map point (metres) under a picture pixel
  var v=mapView; return [v.x_left+p[0]*v.span/v.size,v.y_top-p[1]*v.span/v.size];
}
function middle(){var v=mapView; return [v.x_left+v.span/2,v.y_top-v.span/2];}
function bigger(){            // 17 Nov (Adel): enlarge the map, or back
  document.getElementById('zbig').textContent=
    mapBox.classList.toggle('big')?'Smaller':'Bigger';
  setTimeout(showView,50);
}
function zoomTo(z,c){
  var body={zoom:Math.max(1,Math.min(8,z))};
  if(c){body.x=c[0]; body.y=c[1];}
  fetch('/map/zoom',{method:'POST',body:JSON.stringify(body)});
}
function showView(){          // after each new picture: the ring on the picked dot, the zoom
  place();                    // the map may just have appeared: turn it round its middle
  var z=mapView?mapView.zoom||1:1;
  document.getElementById('zlevel').textContent=z>1?z.toFixed(1)+' times closer':'';
  var now=null, best=1e9;
  if(picked&&mapView&&mapView.pins)mapView.pins.forEach(function(q){
    var d=Math.hypot(q.x-picked.x,q.y-picked.y);
    if(q.name==picked.name&&d<best){best=d; now=q;}
  });
  if(!now){ring.style.display='none'; return;}
  picked=now;                 // its place in the picture changes when the view changes
  var k=mapImg.clientWidth/mapImg.naturalWidth;
  ring.style.left=(now.col*k)+'px'; ring.style.top=(now.row*k)+'px'; ring.style.display='block';
}
function tell(q){             // say what a tapped dot is: only its name (Adel, 17 Nov)
  picked=q; pinInfo.textContent=String(q.name||'(no name)'); showView();
}
mapImg.onclick=function(e){   // a tap: on a dot says what it is, elsewhere enlarges the map
  if(Date.now()-dragged<300)return;                 // the end of a drag is not a tap
  var q=mapView?pinAt(spot(e)):null;
  if(q){tell(q); return;}
  bigger();
};
mapImg.onmousemove=function(e){
  if(!mapView||drag)return;
  var q=pinAt(spot(e));
  mapImg.style.cursor=q?'pointer':'';
  mapImg.title=q?q.name:'';
};
// Press and drag (mouse or finger, at any zoom) to move the map. The picture moves with the
// pointer at once; on letting go Stage 3 draws the new view, and the shift is kept until it
// comes (1.5 s at most: at the edge of the map the view cannot move further). A press that
// moves more than 6 pixels is a drag, never a tap. The browser's own image drag is off
// (draggable=false, dragstart cancelled), and touch-action:none keeps the finger for us.
mapImg.addEventListener('dragstart',function(e){e.preventDefault();});
mapImg.addEventListener('pointerdown',function(e){
  if(!mapView||(e.pointerType=='mouse'&&e.button!==0))return;
  e.preventDefault();
  drag={x:e.clientX,y:e.clientY,dx:0,dy:0,moved:false}; held=null; moveBy(0,0);
  try{mapImg.setPointerCapture(e.pointerId);}catch(err){}
  mapImg.style.cursor=''; mapBox.classList.add('dragging');
});
mapImg.addEventListener('pointermove',function(e){
  if(!drag)return;
  drag.dx=e.clientX-drag.x; drag.dy=e.clientY-drag.y;
  if(Math.hypot(drag.dx,drag.dy)>6)drag.moved=true;
  if(drag.moved)moveBy(drag.dx,drag.dy);
});
function endDrag(){
  if(!drag)return;
  var d=drag; drag=null; mapBox.classList.remove('dragging');
  if(!d.moved){moveBy(0,0); return;}
  dragged=Date.now();
  var m=mapImg.naturalWidth/mapImg.clientWidth*mapView.span/mapView.size, c=middle();
  var u=unturn(d.dx,d.dy);    // the drag, on the map as drawn (not as turned on screen)
  held={box:mapView.x_left+','+mapView.y_top, until:Date.now()+1500};
  zoomTo(mapView.zoom,[c[0]-u[0]*m,c[1]+u[1]*m]);
}
mapImg.addEventListener('pointerup',endDrag);
mapImg.addEventListener('pointercancel',endDrag);
mapImg.addEventListener('wheel',function(e){
  if(!mapView)return;
  var p=spot(e); if(p[1]>=mapView.size)return;
  e.preventDefault();
  var t=Date.now(); if(t-lastWheel<300)return; lastWheel=t;
  var was=mapView.zoom||1, z=Math.max(1,Math.min(8,e.deltaY<0?was*2:was/2));
  var m=metres(p), c=middle(), f=z/was;            // keep the point under the pointer still
  zoomTo(z,[m[0]+(c[0]-m[0])/f,m[1]+(c[1]-m[1])/f]);
},{passive:false});
document.getElementById('zin').onclick=function(){
  if(mapView)zoomTo((mapView.zoom||1)*2,picked?[picked.x,picked.y]:middle());
};
document.getElementById('zout').onclick=function(){
  if(mapView)zoomTo((mapView.zoom||1)/2,middle());
};
document.getElementById('zall').onclick=function(){zoomTo(1);};
document.getElementById('zbig').onclick=bigger;
function nextMap(){
  fetch('/map',{cache:'no-store'}).then(function(r){
    var page=r.headers.get('X-Page');
    if(page&&page!=PAGE_VERSION){location.reload(); throw 0;}   // a newer page: load it
    if(!r.ok)throw 0;
    var v=null; try{v=JSON.parse(r.headers.get('X-Map-View'));}catch(e){}
    return r.blob().then(function(b){return [b,v];});
  }).then(function(got){
    var url=URL.createObjectURL(got[0]), keep=!legImg.getAttribute('src');
    if(keep)legImg.src=url;                         // the legend never changes: keep the first
    mapImg.onload=function(){
      if(!keep)URL.revokeObjectURL(url);
      mapView=got[1];
      var box=mapView?mapView.x_left+','+mapView.y_top:'';
      if(held&&(box!=held.box||Date.now()>held.until)){held=null; if(!drag)moveBy(0,0);}
      showView();
    };
    mapImg.src=url; mapOn.style.display='block'; mapNote.style.display='none';
  }).catch(function(){mapOn.style.display='none'; mapNote.style.display='block';})
  .then(function(){setTimeout(nextMap,100);});
}
nextMap();
// Listening mode = the handle button. Click once to start and again to send, or hold it and
// let go to send. Start = button held, send = button let go.
var talk=document.getElementById('talk'), talking=false, since=0, timer=null;
function press(down){
  if(down==talking)return;
  talking=down; talk.className=down?'on':'';
  talk.textContent=down?'Listening... click to send':'Listening mode';
  clearTimeout(timer);
  if(down)timer=setTimeout(function(){press(false);},15000);   // the Jetson stops at 15 s
  fetch(down?'/talk/start':'/talk/stop',{method:'POST'}).then(r=>r.text())
   .then(t=>document.getElementById('msg').textContent=t)
   .catch(()=>document.getElementById('msg').textContent='The Jetson did not answer');
}
talk.addEventListener('pointerdown',function(e){
  e.preventDefault();
  if(talking){press(false);}else{since=Date.now();press(true);}
});
talk.addEventListener('pointerup',function(){
  if(talking&&Date.now()-since>600)press(false);       // held: letting go sends
});
// Sound through the browser: this page plays what the device says and records the
// question with this computer's microphone. A browser needs one click before it plays sound.
var ctx=null, heard=-1, playing=null, micOk=false, recording=false, chunks=[], sndOn=false;
var snd=document.getElementById('snd'), msgBox=document.getElementById('msg');
function say(t){msgBox.textContent=t;}
snd.addEventListener('click',function(){
  if(sndOn)return;
  var AC=window.AudioContext||window.webkitAudioContext;
  try{ctx=new AC({sampleRate:16000});}catch(e){ctx=new AC();}
  ctx.resume();
  sndOn=true; snd.textContent='Sound is on'; snd.disabled=true;
  if(navigator.mediaDevices&&navigator.mediaDevices.getUserMedia){
    navigator.mediaDevices.getUserMedia({audio:{echoCancellation:true,noiseSuppression:true}})
     .then(function(stream){
       var src=ctx.createMediaStreamSource(stream), node=ctx.createScriptProcessor(4096,1,1);
       var mute=ctx.createGain(); mute.gain.value=0;
       node.onaudioprocess=function(e){
         if(recording)chunks.push(new Float32Array(e.inputBuffer.getChannelData(0)));
       };
       src.connect(node); node.connect(mute); mute.connect(ctx.destination);
       micOk=true; say('Sound and microphone are on');
     }).catch(function(){say('Sound is on. The microphone was not allowed');});
  }else{
    say('Sound is on. No microphone: open this page as http://localhost:8081 to use it');
  }
  nextSound();
});
function nextSound(){
  fetch('/sound/next?after='+heard,{cache:'no-store'}).then(function(r){
    return r.status==200?r.json():null;
  }).then(function(ev){
    if(ev){heard=ev.seq; soundEvent(ev);}
    nextSound();
  }).catch(function(){setTimeout(nextSound,1000);});
}
function stopSound(){
  if(playing){try{playing.stop();}catch(e){} playing=null;}
}
function soundEvent(ev){
  if(ev.kind=='play'){
    fetch('/sound/file?seq='+ev.seq,{cache:'no-store'}).then(function(r){
      if(!r.ok)throw 0; return r.arrayBuffer();
    }).then(function(data){return ctx.decodeAudioData(data);}).then(function(buffer){
      stopSound();
      var source=ctx.createBufferSource(), gain=ctx.createGain();
      gain.gain.value=ev.volume; source.buffer=buffer;
      source.connect(gain); gain.connect(ctx.destination);
      source.onended=function(){if(playing===source)playing=null;};
      source.start(); playing=source; window.lastPlayed=ev.sentence;
    }).catch(function(){});
  }else if(ev.kind=='stop'){
    stopSound();
  }else if(ev.kind=='listen'){
    if(ev.on){chunks=[]; recording=micOk; if(!micOk)say('No microphone on this page');}
    else{var was=recording; recording=false; if(was&&ev.send)sendRecording();}
  }
}
function sendRecording(){
  var total=0, i, j;
  for(i=0;i<chunks.length;i++)total+=chunks[i].length;
  var all=new Float32Array(total), at=0;
  for(i=0;i<chunks.length;i++){all.set(chunks[i],at); at+=chunks[i].length;}
  chunks=[];
  var rate=ctx.sampleRate, out=all;
  if(rate!=16000){                       // bring it to 16 kHz, which the speech server wants
    var n=Math.floor(total*16000/rate); out=new Float32Array(n);
    for(i=0;i<n;i++){
      var x=i*rate/16000; j=Math.floor(x);
      var a=all[j]||0, b=all[Math.min(j+1,total-1)]||0; out[i]=a+(b-a)*(x-j);
    }
  }
  var view=new DataView(new ArrayBuffer(44+out.length*2));
  function text(o,t){for(var k=0;k<t.length;k++)view.setUint8(o+k,t.charCodeAt(k));}
  text(0,'RIFF'); view.setUint32(4,36+out.length*2,true); text(8,'WAVE'); text(12,'fmt ');
  view.setUint32(16,16,true); view.setUint16(20,1,true); view.setUint16(22,1,true);
  view.setUint32(24,16000,true); view.setUint32(28,32000,true); view.setUint16(32,2,true);
  view.setUint16(34,16,true); text(36,'data'); view.setUint32(40,out.length*2,true);
  for(i=0;i<out.length;i++){
    var v=Math.max(-1,Math.min(1,out[i])); view.setInt16(44+i*2,v<0?v*32768:v*32767,true);
  }
  fetch('/talk/audio',{method:'POST',headers:{'Content-Type':'audio/wav'},body:view.buffer})
   .then(r=>r.text()).then(say).catch(()=>say('The Jetson did not answer'));
}
// What the device is doing, once a second: the guide's state and the last sentence said.
function nextState(){
  fetch('/state',{cache:'no-store'}).then(r=>r.json()).then(function(st){
    document.getElementById('state').textContent=st.line;
  }).catch(function(){}).then(function(){setTimeout(nextState,1000);});
}
nextState();
function send(path){
  var q=document.getElementById('q').value.trim();
  if(path!='/stop'&&!q){document.getElementById('msg').textContent='Type a question first';return;}
  fetch(path,{method:'POST',body:q}).then(r=>r.text())
   .then(t=>document.getElementById('msg').textContent=t)
   .catch(()=>document.getElementById('msg').textContent='The Jetson did not answer');
}
</script>
</body></html>'''
# The page knows its own version; /map tells it the server's, so a tab left open with an older
# page (from before a restart) reloads by itself (Adel saw "undefined" from an old tab, 17 Nov).
PAGE_VERSION = hashlib.sha1(PAGE).hexdigest()[:12]
PAGE = PAGE.replace(b'__PAGE_VERSION__', PAGE_VERSION.encode())


def zoom_words(text):
    """
    Check the page's zoom request and give it as clean JSON for Stage 3, or None if it is bad.

    text: JSON {"zoom": 1 to MAP_MAX_ZOOM, "x": metres, "y": metres}; x and y are optional.
    """
    try:
        asked = json.loads(text)
        zoom = float(asked['zoom'])
        if not math.isfinite(zoom):
            return None
        clean = {'zoom': round(min(max(zoom, 1.0), MAP_MAX_ZOOM), 3)}
        if 'x' in asked or 'y' in asked:
            x, y = float(asked['x']), float(asked['y'])
            if not (math.isfinite(x) and math.isfinite(y)):
                return None
            clean['x'], clean['y'] = round(x, 3), round(y, 3)
    except (ValueError, TypeError, KeyError, AttributeError):
        return None
    return json.dumps(clean)


def view_header(text):
    """Give Stage 3's view JSON if it is safe to send as a header (plain, one line), else None."""
    if not text or len(text) > MAX_MAP_VIEW_CHARS or not text.isascii():
        return None
    if any(c in text for c in '\r\n'):
        return None
    return text


def state_words(guide, said):
    """Say the guide's state in plain words, with the last sentence the device said."""
    state, goal = guide.get('state'), guide.get('goal') or {}
    name, left = goal.get('name', 'the goal'), guide.get('distance_m')
    if state == 'guiding':
        line = 'Guiding to %s' % name
        if left is not None:
            line += ', %.1f m to go' % left
        if guide.get('route_ok') is False:
            line += ' (no route right now)'
    elif state == 'paused':
        line = 'Paused on the way to %s (%s button)' % (name, guide.get('paused_by') or 'a')
    elif state == 'idle':
        line = 'Free mode (no trip)'
    else:
        line = ''
    if said:
        line += (' | ' if line else '') + 'Said: "%s"' % said
    return line


def small_bgr(msg, width):
    """
    Turn a ROS colour picture (rgb8 or bgr8) into a small copy, width pixels wide, in OpenCV order.

    Shrinks straight from the message, without first copying the full 1280 x 720 picture:
    Python programs that copy every full-size camera picture are a known cause of lag on Jetsons.
    """
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'expected an rgb8 or bgr8 picture, got "{msg.encoding}"')
    grid = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    grid = grid[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    height = round(msg.height * width / msg.width)
    small = cv2.resize(grid, (width, height), interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(small, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else small


def fit_width(picture, width=WIDTH):
    """Scale a picture to the given width, keeping its shape."""
    height = round(picture.shape[0] * width / picture.shape[1])
    return cv2.resize(picture, (width, height), interpolation=cv2.INTER_AREA)


def scale_box(box, from_size, to_size):
    """Move a box from one picture size (width, height) to another."""
    sx = to_size[0] / from_size[0]
    sy = to_size[1] / from_size[1]
    left, top, right, bottom = box
    return (round(left * sx), round(top * sy), round(right * sx), round(bottom * sy))


def show_box(result, age_s, tracking):
    """Return True if this result's box should be drawn now."""
    if not result or not result.get('found') or not result.get('box'):
        return False
    return tracking or age_s < SHOW_ANSWER_S


def answer_lines(result, age_s=0.0):
    """Turn the /target/result JSON (as a dict) into two lines of text for the panel."""
    if not result:
        return 'No question yet. Type one below, or hold the button and ask.', ''
    question = f'Asked: "{result.get("question", "")}"'
    seconds = result.get('seconds')
    took = f' in {seconds:.1f} s' if isinstance(seconds, (int, float)) else ''
    if result.get('kind') == 'answer' and result.get('said'):
        problem = result.get('problem')
        return question, f'ANSWER{took}: "{result["said"]}"' + (f' ({problem})' if problem
                                                                else '')
    if result.get('found'):
        follow = ', following' if result.get('follower') else ''
        return question, f'FOUND: {result.get("label") or "it"}{took}{follow}'
    said = f'NOT FOUND{took}'
    if result.get('said') and not result.get('tracking'):
        said += f'. Said: "{result["said"]}"'
    problem = result.get('problem')
    return question, said + (f' ({problem})' if problem else '')


def stage1_strip(stage1, width):
    """
    Draw the Stage 1 strip: level, distance and camera status, coloured by the level.

    stage1 is a dict with level, distance (metres or None), status, side ('left', 'right' or
    None) and fresh (False when Stage 1 has gone quiet).
    """
    strip = np.full((30, width, 3), 60, np.uint8)
    if not stage1.get('fresh'):
        text = 'STAGE 1: no data'
    else:
        level = stage1.get('level') or 'no data'
        strip[:] = LEVEL_COLOURS.get(level, (60, 60, 60))
        text = f'STAGE 1: {level.upper()}'
        if isinstance(stage1.get('distance'), (int, float)) and level != 'clear':
            text += f'   {stage1["distance"]:.2f} m from the front'
        if stage1.get('side') in ('left', 'right'):
            text += f'   COMING IN FROM THE {stage1["side"].upper()}'
        elif stage1.get('status'):
            text += f'   {stage1["status"].replace("_", " ")}'
    cv2.putText(strip, text[:70], (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                cv2.LINE_AA)
    return strip


def render(live_bgr, result, age_s, status, tracking, follow_box=None, width=WIDTH, speed='',
           stage1=None):
    """
    Draw one frame: the live picture with the box on it, and a text panel.

    follow_box is the follower's box (left, top, right, bottom) in live_bgr's pixels. When it
    is given it is drawn instead of the answer's own box, which is a few seconds old.
    result['follower'] is True when the follower took over that answer.
    speed is a short line about how fast each part runs, shown small at the bottom.
    stage1, if given, adds the Stage 1 strip under the picture (see stage1_strip).
    """
    if live_bgr is None:
        picture = np.zeros((round(width * 9 / 16), width, 3), np.uint8)
        cv2.putText(picture, 'waiting for the colour camera...', (20, picture.shape[0] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    else:
        live = np.asarray(live_bgr)
        picture = fit_width(live, width)
        box = None
        if show_box(result, age_s, tracking):
            if follow_box is not None:                  # the follower's up-to-date box
                box = scale_box(follow_box, (live.shape[1], live.shape[0]),
                                (picture.shape[1], picture.shape[0]))
            elif not result.get('follower'):            # no follower: the answer's own box
                size = result.get('size') or [live.shape[1], live.shape[0]]
                box = scale_box(result['box'], size, (picture.shape[1], picture.shape[0]))
            # Follower started but lost the object: no box until the next answer.
        if box is not None:
            left, top, right, bottom = box
            cv2.rectangle(picture, (left, top), (right, bottom), BOX_COLOUR, BOX_THICKNESS)
            label = result.get('label') or ''
            if label:
                y = top - 8 if top > 24 else bottom + 24
                cv2.putText(picture, label, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            BOX_COLOUR, 2, cv2.LINE_AA)

    panel = np.full((108, picture.shape[1], 3), 30, np.uint8)
    state = 'tracking' if tracking else (status or 'ready')
    cv2.rectangle(panel, (0, 0), (150, 90), STATUS_COLOURS.get(state, (120, 120, 120)), -1)
    cv2.putText(panel, state.upper(), (8, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2)
    first, second = answer_lines(result, age_s)
    cv2.putText(panel, first[:55], (162, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    found = bool(result and result.get('found'))
    cv2.putText(panel, second[:60], (162, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (80, 220, 80) if found else (80, 80, 240), 1, cv2.LINE_AA)
    cv2.putText(panel, speed[:90], (8, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (150, 150, 150), 1, cv2.LINE_AA)
    if stage1 and stage1.get('fresh') and stage1.get('side') in ('left', 'right'):
        x0 = 0 if stage1['side'] == 'left' else picture.shape[1] - SIDE_BAR_PX
        cv2.rectangle(picture, (x0, 0), (x0 + SIDE_BAR_PX, picture.shape[0]), (0, 0, 255), -1)
    parts = [picture] + ([stage1_strip(stage1, picture.shape[1])] if stage1 else []) + [panel]
    return np.vstack(parts)


class Stage2View(Node):
    """Collect the newest picture and answer, keep a JPEG of the newest frame, take questions."""

    def __init__(self):
        super().__init__('stage2_view')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('port', PORT)
        self.declare_parameter('width', WIDTH)
        self.declare_parameter('camera_to_front_m', 0.83)   # as the hazard gate
        self.camera_to_front_m = float(self.get_parameter('camera_to_front_m').value)
        self.port = int(self.get_parameter('port').value)
        self.width = int(self.get_parameter('width').value)
        self.live_msg = None                    # newest camera picture, as it came
        self.recent = deque(maxlen=8)           # (time, message) of the last few pictures
        self.live = None                        # small copy of it for the screen
        self.live_drawn = None                  # the message self.live was made from
        self.counts = {'camera': 0, 'sent': 0}  # pictures in from the camera / out to browsers
        self.speed, self.speed_time = '', time.monotonic()
        self.follower = FollowerWorker()        # runs in its own thread, never slows drawing
        if not self.follower.available:
            self.get_logger().warn('No OpenCV tracker found: the box will only move with '
                                   'each answer')
        self.result, self.result_time = None, 0.0
        self.status, self.tracking = 'ready', False
        self.misses = 0                         # "not found" answers in a row while tracking
        self.last_request = 0.0                 # when a browser last asked for a picture
        self.stage1 = {'level': None, 'distance': None, 'status': None}
        self.stage1_time = 0.0                  # when Stage 1 last sent its level
        self.side, self.side_time = None, 0.0   # Stage 1's last side warning, and when
        self.jpeg = None
        self.frame_no = 0                       # goes up by one with every new picture
        self.map_jpeg, self.map_time = None, 0.0    # Stage 3's newest map picture, and when
        self.map_sub = None                     # listening for it, only while a browser watches
        self.map_view_text, self.map_view_sub = None, None  # where its dots are
        self.zoom_pub = self.create_publisher(String, MAP_ZOOM_TOPIC, 10)
        self.lock = threading.Lock()
        self.new_frame = threading.Condition(self.lock)
        self.ask_pub = self.create_publisher(String, '/voice/question', 10)
        self.track_pub = self.create_publisher(String, '/stage2/track', 10)
        self.talk_pub = self.create_publisher(Bool, '/button/pressed', 10)   # = handle button
        self.sounds = deque(maxlen=SOUNDS_KEPT)     # sound events for the browsers, numbered
        self.sound_no = 0
        self.new_sound = threading.Condition()
        self.recording_pub = self.create_publisher(String, '/voice/recording', 10)
        self.create_subscription(String, '/voice/out', self.on_sound, 20)
        self.guide, self.said = {}, ''      # the guide's newest state, the last sentence said
        self.create_subscription(String, '/stage4/guide_state', self.on_guide, 10)
        self.create_subscription(String, '/voice/say', self.on_said, 10)
        self.create_subscription(Image, self.get_parameter('color_topic').value,
                                 self.on_live, NEWEST_PICTURE)
        self.create_subscription(String, '/target/result', self.on_result, 10)
        self.create_subscription(String, '/stage2/status', self.on_status, 10)
        self.create_subscription(String, '/stage2/track', self.on_track, 10)
        self.create_subscription(String, '/hazard/level', self.on_level, 10)
        self.create_subscription(Float32, '/hazard/distance', self.on_distance, 10)
        self.create_subscription(String, '/sensor/status', self.on_sensor, 10)
        self.create_subscription(String, '/hazard/side', self.on_side, 10)
        self.create_timer(1 / FRAMES_PER_S, self.draw)

    @property
    def watching(self):
        """Say if a browser has the page open."""
        return time.monotonic() - self.last_request < WATCHING_S

    def follow_map(self):
        """Listen for Stage 3's map picture only while a browser watches; stop soon after."""
        if self.watching and self.map_sub is None:
            self.map_view_sub = self.create_subscription(String, MAP_VIEW_TOPIC,
                                                         self.on_map_view, 1)
            self.map_sub = self.create_subscription(CompressedImage, MAP_TOPIC, self.on_map, 1)
        elif self.map_sub is not None and time.monotonic() - self.last_request > MAP_LET_GO_S:
            self.destroy_subscription(self.map_sub)
            self.destroy_subscription(self.map_view_sub)
            self.map_sub, self.map_jpeg = None, None
            self.map_view_sub, self.map_view_text = None, None

    def on_map(self, msg):
        self.map_jpeg, self.map_time = bytes(msg.data), time.monotonic()

    def on_map_view(self, msg):
        self.map_view_text = msg.data

    def map_picture(self):
        """Give the newest map picture (JPEG bytes), or None if Stage 3 is not sending one."""
        if time.monotonic() - self.map_time > MAP_STALE_S:
            return None
        return self.map_jpeg

    def map_view(self):
        """Give what the map picture shows, as Stage 3's JSON text, or None."""
        return self.map_view_text

    def zoom_map(self, text):
        """Pass the page's zoom request on to Stage 3; say what was asked."""
        words = zoom_words(text)
        if words is None:
            return 'That zoom was not understood'
        self.zoom_pub.publish(String(data=words))
        return 'Zoom %s' % json.loads(words)['zoom']

    def on_live(self, msg):
        """Keep the newest picture and give a small copy of every one to the follower."""
        self.live_msg = msg
        if not self.watching:
            return                              # nobody watching: no work at all
        try:
            small = small_bgr(msg, TRACK_WIDTH)
        except ValueError as problem:
            self.get_logger().error(str(problem), throttle_duration_sec=10.0)
            return
        stamp = stamp_of(msg)
        stamp = time.monotonic() if stamp is None else stamp
        self.follower.submit_frame(stamp, small)
        self.recent.append((stamp, msg))
        self.counts['camera'] += 1

    def on_result(self, msg):
        try:
            result = json.loads(msg.data)
        except ValueError:
            return
        if result.get('found') and result.get('box') and result.get('size'):
            self.follower.submit_answer(result['box'], result['size'], result.get('stamp'),
                                        result.get('boxes'))
            result['follower'] = self.follower.available
            self.misses = 0
        elif (result.get('tracking') and self.follower.following and self.result
              and self.misses < MISSES_ALLOWED):
            # While tracking, one missed answer does not hide the box: the follower still has
            # the object. Two in a row do: it has probably left the picture.
            self.misses += 1
            return
        else:
            self.follower.stop_following()                   # not found: nothing to follow
            result['follower'] = False
        self.result, self.result_time = result, time.monotonic()

    def on_status(self, msg):
        self.status = msg.data

    def on_level(self, msg):
        self.stage1['level'], self.stage1_time = msg.data, time.monotonic()

    def on_distance(self, msg):
        # 17 Nov (Adel): show it from the device's front, as the hazard gate uses it, not from
        # the camera (0.83 m behind the front). -1 = nothing in range.
        d = msg.data
        self.stage1['distance'] = None if d < 0 else max(0.0, d - self.camera_to_front_m)

    def on_sensor(self, msg):
        self.stage1['status'] = msg.data

    def on_side(self, msg):
        if msg.data in ('left', 'right'):
            self.side, self.side_time = msg.data, time.monotonic()

    def on_track(self, msg):
        self.tracking = bool(msg.data.strip())         # also catches tracking started elsewhere
        if not self.tracking:
            self.follower.stop_following()

    def on_guide(self, msg):
        try:
            self.guide = dict(json.loads(msg.data))
        except (ValueError, TypeError):
            self.guide = {}

    def on_said(self, msg):
        self.said = msg.data

    def state_line(self):
        """Give the line under the buttons: what the guide is doing, and what was said last."""
        return state_words(self.guide, self.said)

    def on_sound(self, msg):
        """Keep a sound event from stage2_node for the browsers (sound_out:=browser)."""
        try:
            event = dict(json.loads(msg.data))
        except (ValueError, TypeError):
            return
        wav = None
        if event.get('kind') == 'play':
            try:
                with open(event.pop('file', ''), 'rb') as file:
                    wav = file.read()
            except OSError:
                return                          # the sound file is gone: nothing to play
        with self.new_sound:
            self.sound_no += 1
            event['seq'] = self.sound_no
            self.sounds.append((event, wav))
            self.new_sound.notify_all()

    def next_sound(self, after, wait_s=SOUND_WAIT_S):
        """
        Give the first sound event numbered above `after`, waiting up to wait_s; else None.

        A browser that has just opened the page (after = -1), or that is ahead of this node
        (the node was restarted), is told the newest number, so it never plays old sentences.
        """
        with self.new_sound:
            if after < 0 or after > self.sound_no:
                return {'kind': 'reset', 'seq': self.sound_no}
            self.new_sound.wait_for(lambda: self.sound_no > after, timeout=wait_s)
            for event, _wav in self.sounds:
                if event['seq'] > after:
                    return event
            if self.sound_no > after:           # older than what is kept: skip ahead
                return {'kind': 'reset', 'seq': self.sound_no}
        return None

    def sound_file(self, number):
        """Give the WAV bytes of the play event with this number, or None."""
        with self.new_sound:
            for event, wav in self.sounds:
                if event['seq'] == number:
                    return wav
        return None

    def take_recording(self, wav):
        """Hand a question recorded by a browser to stage2_node, as a file."""
        if not wav.startswith(b'RIFF') or len(wav) <= 44:
            return 'That recording was empty'
        handle, path = tempfile.mkstemp(prefix='apnvi_browser_', suffix='.wav')
        with os.fdopen(handle, 'wb') as file:
            file.write(wav)
        self.recording_pub.publish(String(data=path))
        return 'Sent (%.1f s)' % ((len(wav) - 44) / 32000.0)

    def ask(self, question):
        """Ask once, like the handle button. Returns the text shown under the buttons."""
        self.ask_pub.publish(String(data=question))
        return f'Asked once: "{question}"'

    def talk(self, pressed):
        """Hold to speak: send what the handle button sends. Returns the text for the page."""
        self.talk_pub.publish(Bool(data=pressed))
        return 'Listening... let go when done' if pressed else 'Sent'

    def track(self, question):
        """Start tracking a question, or stop with an empty one."""
        self.tracking = bool(question)
        if not question:
            self.follower.stop_following()
        self.track_pub.publish(String(data=question))
        return f'Tracking: "{question}"' if question else 'Stopped'

    def draw(self):
        self.follow_map()
        if not self.watching:
            return
        msg = self.live_msg
        if self.follower.following:
            # Show the picture the tracker last finished, not the newest one it is still
            # working on: then the box sits exactly on the object (the video is 1/30 s later).
            done = self.follower.done_stamp
            msg = next((m for stamp, m in reversed(self.recent) if stamp == done), msg)
        if msg is not None and msg is not self.live_drawn:     # a new picture: shrink it once
            self.live, self.live_drawn = small_bgr(msg, self.width), msg
        live = self.live
        follow_box = None
        if live is not None:
            follow_box = self.follower.box_in(live.shape[1], live.shape[0])
        self.update_speed()
        frame = render(live, self.result, time.monotonic() - self.result_time, self.status,
                       self.tracking, follow_box, self.width, self.speed,
                       self.stage1_now())
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            with self.new_frame:
                self.jpeg = encoded.tobytes()
                self.frame_no += 1
                self.new_frame.notify_all()

    def stage1_now(self):
        """Stage 1's latest level, distance, status and side warning, for the strip."""
        now = time.monotonic()
        side = self.side if now - self.side_time < SIDE_SHOW_S else None
        return dict(self.stage1, side=side, fresh=now - self.stage1_time < STAGE1_STALE_S)

    def update_speed(self):
        """Once a second, work out how many pictures each part handles per second."""
        now = time.monotonic()
        seconds = now - self.speed_time
        if seconds < 1.0:
            return
        done, busy_s = self.follower.take_counts()
        tracker = f'tracker {done / seconds:.0f}/s'
        if done:
            tracker += f' ({1000 * busy_s / done:.0f} ms each)'
        self.speed = (f'camera {self.counts["camera"] / seconds:.0f}/s | {tracker} | '
                      f'to browser {self.counts["sent"] / seconds:.0f}/s')
        self.counts = {'camera': 0, 'sent': 0}
        self.speed_time = now


def make_handler(view):
    """
    Web page handler.

    GET /             the page
    GET /frame?after=N  the newest picture, waiting (up to 1 s) until there is one newer
                      than number N. Its number is in the X-Frame header.
    GET /map          Stage 3's newest map picture (JPEG), or 503 if there is none. Its
                      X-Map-View header is Stage 3's view JSON: the box, zoom and the dots
    POST /map/zoom    JSON {"zoom", "x", "y"}: ask Stage 3 to draw the map closer in
    GET /stream       the old non-stop stream (can fall behind on a slow connection)
    POST /ask, /track, /stop  the buttons
    POST /talk/start, /talk/stop  "Hold to speak" pressed and let go
    GET /state        JSON {"line": what the device is doing, in words}
    GET /sound/next?after=N  the next sound event as JSON (waits up to 20 s; 204 = none)
    GET /sound/file?seq=N    the WAV of play event N
    POST /talk/audio         a WAV recorded by the browser's microphone
    """
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802 (name fixed by http.server)
            if self.path.startswith('/frame'):
                self.send_frame()
                return
            if self.path.startswith('/map'):
                self.send_map()
                return
            if self.path.startswith('/sound/'):
                self.send_sound()
                return
            if self.path.startswith('/state'):
                self.reply(200, json.dumps({'line': view.state_line()}).encode('utf-8'),
                           'application/json')
                return
            if self.path != '/stream':
                self.reply(200, PAGE, 'text/html; charset=utf-8')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with view.lock:
                        jpeg = view.jpeg
                    if jpeg:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                                         + jpeg + b'\r\n')
                    time.sleep(1 / FRAMES_PER_S)
            except (BrokenPipeError, ConnectionResetError):
                pass                                # browser tab closed

        def do_POST(self):  # noqa: N802 (name fixed by http.server)
            if self.path == '/talk/audio':
                size = int(self.headers.get('Content-Length') or 0)
                if size <= 0 or size > MAX_RECORDING_BYTES:
                    self.reply(400, b'That recording was empty or too long',
                               'text/plain; charset=utf-8')
                    return
                text = view.take_recording(self.rfile.read(size))
                self.reply(200, text.encode('utf-8'), 'text/plain; charset=utf-8')
                return
            if self.path == '/map/zoom':
                size = min(int(self.headers.get('Content-Length') or 0), 1000)
                text = view.zoom_map(self.rfile.read(size).decode('utf-8', 'ignore'))
                self.reply(200, text.encode('utf-8'), 'text/plain; charset=utf-8')
                return
            length = min(int(self.headers.get('Content-Length') or 0), 4 * MAX_QUESTION_CHARS)
            question = self.rfile.read(length).decode('utf-8', 'ignore').strip()
            question = question[:MAX_QUESTION_CHARS]
            if self.path in ('/talk/start', '/talk/stop'):
                text = view.talk(self.path == '/talk/start')
            elif self.path == '/stop':
                text = view.track('')
            elif not question:
                self.reply(400, b'Type a question first', 'text/plain; charset=utf-8')
                return
            elif self.path == '/ask':
                text = view.ask(question)
            elif self.path == '/track':
                text = view.track(question)
            else:
                self.reply(404, b'Unknown address', 'text/plain; charset=utf-8')
                return
            self.reply(200, text.encode('utf-8'), 'text/plain; charset=utf-8')

        def number(self, key, default):
            """Read a whole number from the address: /sound/next?after=12 -> 12."""
            try:
                return int(self.path.split(key + '=', 1)[1].split('&')[0])
            except (IndexError, ValueError):
                return default

        def send_sound(self):
            """Send the next sound event, or the WAV of one."""
            if self.path.startswith('/sound/next'):
                event = view.next_sound(self.number('after', -1))
                if event is None:
                    self.reply(204, b'', 'application/json')
                else:
                    self.reply(200, json.dumps(event).encode('utf-8'), 'application/json')
                return
            wav = view.sound_file(self.number('seq', -1))
            if wav is None:
                self.reply(404, b'No such sound', 'text/plain; charset=utf-8')
            else:
                self.reply(200, wav, 'audio/wav')

        def send_map(self):
            """Send Stage 3's newest map picture, or say there is none."""
            jpeg = view.map_picture()
            if not jpeg:
                self.reply(503, b'No map', 'text/plain; charset=utf-8')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Page', PAGE_VERSION)
            where = view_header(view.map_view())
            if where:
                self.send_header('X-Map-View', where)
            self.end_headers()
            self.wfile.write(jpeg)

        def send_frame(self):
            """Send the newest picture, once it is newer than the one the browser has."""
            try:
                after = int(self.path.split('after=', 1)[1].split('&')[0])
            except (IndexError, ValueError):
                after = 0
            view.last_request = time.monotonic()    # a browser is watching: start drawing
            with view.new_frame:
                view.new_frame.wait_for(lambda: view.frame_no > after and view.jpeg,
                                        timeout=1.0)
                jpeg, number = view.jpeg, view.frame_no
            if not jpeg:
                self.reply(503, b'No picture yet', 'text/plain; charset=utf-8')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame', str(number))
            self.end_headers()
            self.wfile.write(jpeg)
            view.counts['sent'] += 1

        def reply(self, status, body, content_type):
            """Send a short reply."""
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')   # never keep an old page
            self.send_header('X-Page', PAGE_VERSION)
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass                                    # keep the terminal quiet

    return Handler


def main(args=None):
    """Start the Stage 2 view and its small web server, until Ctrl+C."""
    rclpy.init(args=args)
    view = Stage2View()
    server = ThreadingHTTPServer(('0.0.0.0', view.port), make_handler(view))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    view.get_logger().info(f'Stage 2 view: open http://localhost:{view.port} '
                           f'(or http://<this computer\'s IP>:{view.port}) in a browser')
    try:
        rclpy.spin(view)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        view.follower.close()
        view.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
