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
loadDevices();

async function fetchFormats(index){
  const r = await fetch('/cam_formats?index='+encodeURIComponent(index));
  const js = await r.json();
  if (!js.ok) throw new Error(js.error || 'formats error');
  return js.formats; // { FOURCC: {description, sizes:[{w,h,fps:[...]}] } }
}

function addTile(index){
  const id = 'tile'+(tileIdSeq++);
  const grid = document.getElementById('grid');
  const card = document.createElement('div');
  card.className='card one1';
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
        <select id="${id}-ccSel"  title="FOURCC"></select>
        <select id="${id}-resSel" title="Resolution"></select>
        <select id="${id}-fpsSel" title="FPS"></select>

        <button id="${id}-start">Start</button>
        <button id="${id}-stop" disabled>Stop</button>

        <button id="${id}-fit" disabled>Fit</button>
        <button id="${id}-one" disabled>1:1</button>

        <label style="margin-left:8px;">Zoom
          <input type="range" id="${id}-zoom" min="50" max="300" value="100" style="vertical-align:middle;">
        </label>

        <button id="${id}-close" style="margin-left:auto;">Close</button>
      </div>
    </div>
    <div class="video-wrap" id="${id}-wrap">
      <video id="${id}-v" class="vid" autoplay playsinline muted></video>
    </div>
  `;
  grid.appendChild(card);

  const ccSel  = card.querySelector(`#${id}-ccSel`);
  const resSel = card.querySelector(`#${id}-resSel`);
  const fpsSel = card.querySelector(`#${id}-fpsSel`);

  const v    = document.getElementById(`${id}-v`),
        st   = document.getElementById(`${id}-state`),
        fps  = document.getElementById(`${id}-fps`),
        res  = document.getElementById(`${id}-res`),
        bStart = document.getElementById(`${id}-start`),
        bStop  = document.getElementById(`${id}-stop`),
        bFit   = document.getElementById(`${id}-fit`),
        bOne   = document.getElementById(`${id}-one`),
        bClose = document.getElementById(`${id}-close`),
        rngZoom= document.getElementById(`${id}-zoom`),
        wrap   = document.getElementById(`${id}-wrap`);

  let pc = null, stopFPS = null, zoom = 1.0, mode = 'one1';
  let formats = null;

  function setState(s){ st.textContent = s; }

  function setModeOne(){
    mode = 'one1';
    card.classList.remove('fit');
    card.classList.add('one1');
    wrap.classList.remove('scaled');
    const w = v.videoWidth || 640;
    const h = v.videoHeight|| 360;
    v.style.width  = Math.round(w * zoom) + 'px';
    v.style.height = Math.round(h * zoom) + 'px';
    wrap.style.height = v.style.height;
  }
  function applyFitScale(){
    const naturalW = v.videoWidth || 640;
    const naturalH = v.videoHeight|| 360;
    const wrapWidth = wrap.clientWidth || naturalW;
    wrap.classList.add('scaled');
    wrap.style.transform = `scale(${zoom})`;
    const baseH   = wrapWidth * (naturalH / naturalW);
    const scaledH = baseH * zoom;
    wrap.style.height = Math.round(scaledH) + 'px';
  }
  function setModeFit(){
    mode = 'fit';
    card.classList.remove('one1');
    card.classList.add('fit');
    v.style.width = '100%';
    v.style.height= 'auto';
    applyFitScale();
  }
  function setZoom(val){
    zoom = val;
    if (mode === 'one1') setModeOne();
    else applyFitScale();
  }

  function fillFourcc(){
    ccSel.innerHTML = '';
    const keys = Object.keys(formats);
    keys.forEach(k=>{
      const opt = document.createElement('option');
      opt.value = k;
      opt.textContent = `${k} — ${formats[k].description}`;
      ccSel.appendChild(opt);
    });
    if (keys.includes('MJPG')) ccSel.value = 'MJPG';
  }
  function fillResFor(cc){
    resSel.innerHTML = '';
    if (!formats[cc]) return;
    formats[cc].sizes.forEach(s=>{
      const opt = document.createElement('option');
      opt.value = `${s.w}x${s.h}`;
      opt.textContent = `${s.w}×${s.h}`;
      resSel.appendChild(opt);
    });
  }
  function fillFpsFor(cc, wh){
    fpsSel.innerHTML = '';
    const size = formats[cc]?.sizes.find(s => (s.w+'x'+s.h)===wh);
    if (!size){
      const opt = document.createElement('option'); opt.value='30'; opt.textContent='30 fps';
      fpsSel.appendChild(opt);
      return;
    }
    size.fps.forEach(f=>{
      const opt = document.createElement('option');
      opt.value = f;
      opt.textContent = `${f} fps`;
      fpsSel.appendChild(opt);
    });
  }

  async function initDropdowns(){
    try{
      formats = await fetchFormats(index);
      if (!formats || Object.keys(formats).length===0){
        throw new Error('no formats');
      }
      fillFourcc();
      fillResFor(ccSel.value);
      const firstWH = resSel.options[0]?.value || '640x480';
      resSel.value = firstWH;
      fillFpsFor(ccSel.value, resSel.value);
    }catch(e){
      console.warn('formats fallback', e);
      // fallback minimal
      formats = { MJPG:{description:'Motion-JPEG',sizes:[{w:640,h:360,fps:[30,25,20,15,10]}]} };
      fillFourcc(); fillResFor('MJPG'); fillFpsFor('MJPG','640x360'); 
      ccSel.value='MJPG'; resSel.value='640x360'; fpsSel.value='30';
    }
  }
  initDropdowns();

  ccSel.onchange  = ()=>{ fillResFor(ccSel.value); fillFpsFor(ccSel.value, resSel.value); };
  resSel.onchange = ()=>{ fillFpsFor(ccSel.value, resSel.value); };

  async function start(){
    if (pc) return;
    setState('connecting');
    pc = new RTCPeerConnection();
    pc.addTransceiver('video',{direction:'recvonly'});
    pc.ontrack = (e)=>{
      v.srcObject = e.streams[0];
      setState('playing');
      v.onloadedmetadata = async ()=>{
        res.textContent = 'res: ' + v.videoWidth + 'x' + v.videoHeight;
        setModeOne();
        startFPSCounter();

        try{
          const infoRes = await fetch('/cam_info?index='+encodeURIComponent(index));
          const info = await infoRes.json();
          if (info.ok){
            const applied = `${info.w}x${info.h}@${(info.fps.toFixed ? info.fps.toFixed(1) : info.fps)} ${info.fourcc||''}`;
            let chip = card.querySelector(`#${id}-applied`);
            if (!chip){
              chip = document.createElement('div');
              chip.className = 'chip';
              chip.id = `${id}-applied`;
              res.parentElement.insertBefore(chip, res.nextSibling);
            }
            chip.textContent = 'applied: ' + applied;

            const [selW, selH] = resSel.value.split('x').map(n=>parseInt(n,10));
            const selF = parseFloat(fpsSel.value);
            const selC = ccSel.value.toUpperCase();
            const mismatch = (selW!==info.w) || (selH!==info.h) || (Math.abs(selF - Math.round(info.fps))>1) || (selC!==String(info.fourcc).toUpperCase());
            let warn = card.querySelector(`#${id}-warn`);
            if (mismatch){
              if (!warn){
                warn = document.createElement('div');
                warn.id = `${id}-warn`;
                warn.className = 'chip';
                warn.style.borderColor = '#ff8a00';
                warn.style.color = '#ffcc88';
                warn.style.background = '#2a1a00';
                res.parentElement.appendChild(warn);
              }
              warn.textContent = '⚠ ค่าใช้งานจริงต่างจากที่เลือก (ไดรเวอร์เลือกโหมดที่รองรับให้)';
            } else if (warn){
              warn.remove();
            }
          }
        }catch(e){ console.warn('cam_info error', e); }
      };
    };
    pc.oniceconnectionstatechange = ()=> setState(pc.iceConnectionState);

    const [W,H] = resSel.value.split('x').map(x=>parseInt(x,10));
    const F = parseFloat(fpsSel.value);
    const CC = ccSel.value;

    const qs = new URLSearchParams({ index:String(index) });
    if (W && H){ qs.set('w', String(W)); qs.set('h', String(H)); }
    if (F){ qs.set('fps', String(F)); }
    if (CC){ qs.set('fourcc', CC); }

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    const r = await fetch('/offer?'+qs.toString(), { method:'POST', headers:{'Content-Type':'application/sdp'}, body:offer.sdp });
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
    fps.textContent='fps: --'; res.textContent='res: --';
    const chip = card.querySelector(`#${id}-applied`); if (chip) chip.remove();
    const warn = card.querySelector(`#${id}-warn`);    if (warn) warn.remove();
    if (stopFPS) stopFPS();
    bStart.disabled=false; bStop.disabled=true; bFit.disabled=true; bOne.disabled=true;
  }

  function startFPSCounter(){
    if (stopFPS) stopFPS();
    if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
      let last=performance.now(), count=0, running=true;
      const onFrame=(_n,_m)=>{ if(!running) return; count++; const now=performance.now();
        if(now-last>=1000){ fps.textContent='fps: '+((count*1000)/(now-last)).toFixed(1); count=0; last=now; }
        v.requestVideoFrameCallback(onFrame);
      };
      v.requestVideoFrameCallback(onFrame);
      stopFPS = ()=>{ running=false; };
      return;
    }
    let last=performance.now(), count=0, rafId=null;
    const tick=()=>{ if(!v||v.readyState<2){ rafId=requestAnimationFrame(tick); return; }
      count++; const now=performance.now();
      if(now-last>=1000){ fps.textContent='fps: '+((count*1000)/(now-last)).toFixed(1); count=0; last=now; }
      rafId=requestAnimationFrame(tick);
    };
    rafId=requestAnimationFrame(tick);
    stopFPS = ()=>{ if(rafId) cancelAnimationFrame(rafId); };
  }

  document.getElementById(`${id}-start`).onclick = start;
  document.getElementById(`${id}-stop`).onclick  = stop;
  document.getElementById(`${id}-fit`).onclick   = ()=> setModeFit();
  document.getElementById(`${id}-one`).onclick   = ()=> setModeOne();
  document.getElementById(`${id}-close`).onclick = ()=>{ stop(); card.remove(); };
  document.getElementById(`${id}-zoom`).oninput  = (e)=> setZoom(parseInt(e.target.value,10)/100);

  // ไม่ auto-start — เลือกจากรายการจริงก่อน
}
