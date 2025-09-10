// ====== Cameras ======
let tileIdSeq = 1;

async function loadDevices(){
  const r = await fetch('/devices'); 
  const js = await r.json();
  const sel = document.getElementById('camSel');
  sel.innerHTML='';
  js.devices.forEach(d=>{
    const opt=document.createElement('option');
    opt.value=d.index;
    opt.textContent=`/dev/video${d.index} — ${d.name}`;
    sel.appendChild(opt);
  });
}

document.getElementById('refresh').onclick = loadDevices;
document.getElementById('add').onclick = () => {
  const sel = document.getElementById('camSel');
  if (!sel.value) return;
  addTile(parseInt(sel.value,10));
};

function addTile(index){
  const id = 'tile'+(tileIdSeq++);
  const grid = document.getElementById('grid');

  const card = document.createElement('div');
  card.className='card';
  card.id = id;

  card.innerHTML = `
    <div class="toolbar">
      <div class="row">
        <div class="chip">/dev/video${index}</div>
        <div id="${id}-fps" class="chip">fps: --</div>
        <div id="${id}-res" class="chip">res: --</div>
        <div id="${id}-state" class="chip">idle</div>
      </div>
      <div class="row">
        <button id="${id}-start">Start</button>
        <button id="${id}-stop" disabled>Stop</button>
        <button id="${id}-fit" disabled>Fit</button>
        <button id="${id}-one" disabled>1:1</button>
        <button id="${id}-close">Close</button>
      </div>
    </div>
    <video id="${id}-v" class="vid" autoplay playsinline muted></video>
  `;
  grid.appendChild(card);

  const v   = document.getElementById(`${id}-v`);
  const st  = document.getElementById(`${id}-state`);
  const fps = document.getElementById(`${id}-fps`);
  const res = document.getElementById(`${id}-res`);
  const bStart = document.getElementById(`${id}-start`);
  const bStop  = document.getElementById(`${id}-stop`);
  const bFit   = document.getElementById(`${id}-fit`);
  const bOne   = document.getElementById(`${id}-one`);
  const bClose = document.getElementById(`${id}-close`);

  let pc = null;
  let stopFPS = null;

  function setState(s){ st.textContent = s; }

  async function start(){
    if (pc) return;
    setState('connecting');
    pc = new RTCPeerConnection();
    pc.addTransceiver('video',{direction:'recvonly'});
    pc.ontrack = (e)=>{
      v.srcObject = e.streams[0];
      setState('playing');
      v.onloadedmetadata = ()=>{
        setOneToOne();
        res.textContent = 'res: ' + v.videoWidth + 'x' + v.videoHeight;
        startFPSCounter();
      };
    };
    pc.oniceconnectionstatechange = ()=> setState(pc.iceConnectionState);

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    const r = await fetch('/offer?index='+encodeURIComponent(index), {
      method:'POST', headers:{'Content-Type':'application/sdp'}, body:offer.sdp
    });
    const ans = await r.text();
    await pc.setRemoteDescription({type:'answer', sdp:ans});

    bStart.disabled = true; bStop.disabled=false; bFit.disabled=false; bOne.disabled=false;
  }

  function stop(){
    if (!pc) return;
    pc.getTransceivers().forEach(t=>t.stop&&t.stop());
    pc.close(); pc=null;
    v.srcObject=null;
    setState('stopped');
    fps.textContent='fps: --';
    res.textContent='res: --';
    if (stopFPS) stopFPS();
    bStart.disabled=false; bStop.disabled=true; bFit.disabled=true; bOne.disabled=true;
  }

  function setOneToOne(){
    const w=v.videoWidth||640, h=v.videoHeight||360;
    v.style.width  = w + 'px';
    v.style.height = h + 'px';
  }
  function setFit(){
    v.style.width  = '100%';
    v.style.height = 'auto';
  }

  function startFPSCounter(){
    if (stopFPS) stopFPS();

    if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
      let last=performance.now(), count=0, running=true;
      const onFrame = (_now, _meta)=>{
        if (!running) return;
        count++;
        const now=performance.now();
        if (now-last>=1000) {
          const f=(count*1000)/(now-last);
          fps.textContent = 'fps: ' + f.toFixed(1);
          count=0; last=now;
        }
        v.requestVideoFrameCallback(onFrame);
      };
      v.requestVideoFrameCallback(onFrame);
      stopFPS = ()=>{ running=false; };
      return;
    }

    let last=performance.now(), count=0, rafId=null;
    const tick=()=>{
      if (!v || v.readyState<2) { rafId=requestAnimationFrame(tick); return; }
      count++;
      const now=performance.now();
      if (now-last>=1000) {
        const f=(count*1000)/(now-last);
        fps.textContent = 'fps: ' + f.toFixed(1);
        count=0; last=now;
      }
      rafId=requestAnimationFrame(tick);
    };
    rafId=requestAnimationFrame(tick);
    stopFPS = ()=>{ if (rafId) cancelAnimationFrame(rafId); };
  }

  bStart.onclick = start;
  bStop.onclick  = stop;
  bFit.onclick   = setFit;
  bOne.onclick   = setOneToOne;
  bClose.onclick = ()=>{ stop(); card.remove(); };

  // auto start
  start();
}

loadDevices();

// ====== ROS Overlay + HUD ======
(function initROS(){
  const body = document.getElementById('ros-body');
  const clearBtn = document.getElementById('ov-clear');
  clearBtn.onclick = ()=>{ body.innerHTML = '<div class="empty">Cleared</div>'; };

  function addItem(obj){
    if (!obj || !obj.payload) return;
    const p = obj.payload;
    if (body.querySelector('.empty')) body.innerHTML = '';
    const item = document.createElement('div');
    item.className = 'item';
    const ts = new Date((p.t||Date.now())*1000).toLocaleTimeString();
    const pretty = JSON.stringify(p.data ?? p, null, 2);
    item.innerHTML = `
      <div class="topic">${p.topic || 'topic'}</div>
      <div class="time">${ts}</div>
      <pre style="white-space:pre-wrap; margin:4px 0 0 0;">${pretty}</pre>
    `;
    body.prepend(item);
    const maxItems = 50;
    while (body.children.length > maxItems) body.removeChild(body.lastChild);
  }

  // กฎแม็พไป HUD
  const PIN_RULES = [
    { topic: '/encoder', path: 'data.left',  elem: '#pin-left-enc',  fmt: v => v?.toFixed ? v.toFixed(0) : v },
    { topic: '/encoder', path: 'data.right', elem: '#pin-right-enc', fmt: v => v?.toFixed ? v.toFixed(0) : v },
    { topic: '/odom', path: 'data.pose.pose.position', elem: '#pin-dist',
      fmt: p => {
        if (!p || typeof p.x!=='number' || typeof p.y!=='number') return '—';
        const d = Math.hypot(p.x, p.y); return d.toFixed(2);
      }
    },
    { topic: '/imu', path: 'data.orientation', elem: '#pin-heading',
      fmt: q => {
        if (!q || typeof q.z!=='number' || typeof q.w!=='number') return '—';
        const yaw = 2*Math.atan2(q.z, q.w); return (yaw*180/Math.PI).toFixed(1);
      }
    },
  ];

  function getPath(obj, path){
    if (!obj || !path) return undefined;
    const seg = path.split('.');
    let cur = obj;
    for (const s of seg){
      if (cur && Object.prototype.hasOwnProperty.call(cur, s)){
        cur = cur[s];
      } else {
        return undefined;
      }
    }
    return cur;
  }

  function tryUpdatePins(p){
    let matched = false;
    for (const r of PIN_RULES){
      if (r.topic && p.topic !== r.topic) continue;
      const v = getPath(p, r.path);
      if (typeof v === 'undefined') continue;
      const el = document.querySelector(r.elem);
      if (!el) continue;
      el.textContent = r.fmt ? String(r.fmt(v)) : String(v);
      matched = true;
    }
    return matched;
  }

  // ปุ่ม reset HUD
  document.getElementById('pin-clear').onclick = ()=>{
    ['#pin-left-enc','#pin-right-enc','#pin-dist','#pin-heading'].forEach(sel=>{
      const el = document.querySelector(sel);
      if (el) el.textContent = '—';
    });
  };

  // WebSocket (อันเดียว!)
  function connect(){
    const ws = new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');
    ws.onopen    = ()=> console.log('[WS] open');
    ws.onclose   = ()=> { console.log('[WS] closed, retry...'); setTimeout(connect, 1500); };
    ws.onerror   = (e)=> console.warn('[WS] error', e);
    ws.onmessage = (e)=>{
      try{
        const msg = JSON.parse(e.data);
        if (msg.type !== 'ros' || !msg.payload) return;
        const p = msg.payload;

        // อัปเดต /status (ถ้ามี status-box)
        if (p.topic === '/status'){
          const el = document.getElementById('status-text'); // กล่องเขียวมุมซ้ายบน
          if (el && p.data && typeof p.data.status === 'string') {
            el.textContent = p.data.status;
          }
          // อัปเดต chip ด้านบน (optional)
          const chip = document.getElementById('status');
          if (chip) chip.textContent = p.data?.status ?? 'status';
        }

        // พยายามอัปเดต HUD ตาม PIN_RULES
        const handled = tryUpdatePins(p);

        // ถ้าไม่ match → โผล่ overlay list
        if (!handled) addItem(msg);
      }catch(err){
        console.warn('parse ws msg err', err);
      }
    };
  }
  connect();
})();

// Toggle float/dock ของ sidebar (ค่าเริ่มต้น: dock)
document.getElementById('toggle-float').onclick = ()=>{
  document.body.classList.toggle('float');
  document.getElementById('toggle-float').textContent =
    document.body.classList.contains('float') ? 'Dock Overlay' : 'Float Overlay';
};