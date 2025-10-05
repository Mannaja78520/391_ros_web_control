const $ = s => document.querySelector(s);
const api = (p)=>fetch(p).then(r=>r.json());
const capsBySlot = {};
let knownDevices = [];

function diffDevices(newList){
  const oldSet = new Set(knownDevices.map(d=>d.device));
  const newSet = new Set(newList.map(d=>d.device));
  let changed = false;
  if(oldSet.size !== newSet.size) changed = true;
  if(!changed){
    for(const dev of newSet){
      if(!oldSet.has(dev)){ changed = true; break; }
    }
  }
  knownDevices = newList;
  return changed;
}

async function loadDevices(slot){
  const j = await api('/api/devices');
  const devSel = $('#dev'+slot);
  devSel.innerHTML='';
  const devices = j.devices || [];
  (slot === 1 && diffDevices(devices));
  devices.forEach(d=>{
    const opt=document.createElement('option');
    opt.value=d.device; opt.textContent=`${d.device}${d.name? ' — '+d.name:''}`;
    devSel.appendChild(opt);
  });
}

async function loadCaps(slot){
  const devSel = $('#dev'+slot);
  const fmtSel = $('#fmt'+slot);
  const resSel = $('#res'+slot);
  const fpsSel = $('#fps'+slot);
  fmtSel.innerHTML='';resSel.innerHTML='';fpsSel.innerHTML='';
  const dev = devSel.value;
  const j = await api(`/api/caps?dev=${encodeURIComponent(dev)}`);
  const caps = j.caps || {};
  capsBySlot[slot] = caps;
  Object.keys(caps).forEach(fmt=>{
    const o=document.createElement('option'); o.value=fmt; o.textContent=fmt; fmtSel.appendChild(o);
  });
  onFmtChange(slot, caps);
  return caps;
}

function onFmtChange(slot, caps){
  const fmtSel = $('#fmt'+slot);
  const resSel = $('#res'+slot);
  resSel.innerHTML='';
  const fmt = fmtSel.value;
  const sizes = fmt && caps[fmt] ? Object.keys(caps[fmt]) : [];
  // sort ascending by area to default smallest
  sizes.sort((a,b)=>{const [aw,ah]=a.split('x').map(Number);const [bw,bh]=b.split('x').map(Number);return (aw*ah)-(bw*bh)});
  sizes.forEach(sz=>{const o=document.createElement('option');o.value=sz;o.textContent=sz;resSel.appendChild(o);});
  onResChange(slot, caps);
}

function onResChange(slot, caps){
  const fmtSel = $('#fmt'+slot);
  const resSel = $('#res'+slot);
  const fpsSel = $('#fps'+slot);
  fpsSel.innerHTML='';
  const fmt = fmtSel.value, sz = resSel.value;
  const fpsList = (caps[fmt]?.[sz]||[]).slice();
  // ascending to default to lowest fps at top if desired
  fpsList.sort((a,b)=>a-b);
  fpsList.forEach(f=>{
    const o=document.createElement('option');
    // keep one decimal for readability when needed
    const label = (Math.abs(f - Math.round(f)) < 0.05) ? String(Math.round(f)) : f.toFixed(1);
    o.value=String(f);
    o.textContent=label+" fps";
    fpsSel.appendChild(o);
  });
  // Prefer 20 fps if available
  const opt20 = [...fpsSel.options].find(o=>parseInt(o.value,10)===20);
  if (opt20) fpsSel.value = '20';
}

async function applySlot(slot){
  const sPill = $('#s'+slot);
  const dev = $('#dev'+slot).value;
  const fourcc = $('#fmt'+slot).value;
  const [w,h] = $('#res'+slot).value.split('x').map(Number);
  const fps = parseFloat($('#fps'+slot).value);
  const res = await fetch('/api/apply',{
    method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({slot, device:dev, fourcc, width:w, height:h, fps})
  });
  const j = await res.json();
  if(!res.ok){ sPill.textContent=j.error||'Apply failed'; sPill.className='pill bad'; return; }
  const a=j.applied||{}; sPill.textContent=`${a.width}x${a.height}@${a.fps} ${a.fourcc}`; sPill.className='pill ok';
  // sync selects to actual applied to reflect camera real capabilities
  try{
    const fmtSel = $('#fmt'+slot);
    const resSel = $('#res'+slot);
    const fpsSel = $('#fps'+slot);
    if ([...fmtSel.options].some(o=>o.value===a.fourcc)) fmtSel.value=a.fourcc;
    const sz = `${a.width}x${a.height}`;
    if ([...resSel.options].some(o=>o.value===sz)) resSel.value=sz;
    // pick fps option by nearest numeric value
    let best=null, bestDiff=1e9;
    [...fpsSel.options].forEach(o=>{
      const v = parseFloat(o.value); const d = Math.abs(v - parseFloat(a.fps));
      if (d < bestDiff){ best=o; bestDiff=d; }
    });
    if (best) fpsSel.value = best.value;
  }catch(e){}
}

async function initSlot(slot){
  await loadDevices(slot);
  const caps = await loadCaps(slot);
  const fmtSel = $('#fmt'+slot);
  if([... 
    fmtSel.options].some(o=>o.value==='MJPG')) fmtSel.value='MJPG';
  onFmtChange(slot, caps);
  try{
    const j = await api(`/api/applied?slot=${slot}`); const a=j.applied||{};
    const sPill = $('#s'+slot);
    if(a && a.width){ sPill.textContent=`${a.width}x${a.height}@${a.fps} ${a.fourcc}`; sPill.className='pill ok'; }
    // set current device to applied one if present, and reload caps accordingly
    const devSel = $('#dev'+slot);
    if (a && a.device && [...devSel.options].some(o=>o.value===a.device)){
      devSel.value = a.device;
      const c2 = await loadCaps(slot);
      if ([...fmtSel.options].some(o=>o.value===a.fourcc)) fmtSel.value=a.fourcc;
      onFmtChange(slot, c2);
      const resSel = $('#res'+slot);
      const fpsSel = $('#fps'+slot);
      const sz = `${a.width}x${a.height}`;
      if ([...resSel.options].some(o=>o.value===sz)) resSel.value=sz;
      let best=null, bestDiff=1e9;
      [...fpsSel.options].forEach(o=>{ const v=parseFloat(o.value); const d=Math.abs(v-parseFloat(a.fps)); if(d<bestDiff){best=o;bestDiff=d;} });
      if (best) fpsSel.value=best.value;
    }
  }catch(e){}

  // wire events for cascade updates per slot
  const devSel = $('#dev'+slot);
  const resSel = $('#res'+slot);
  devSel.onchange = async ()=>{
    const c = await loadCaps(slot);
    // prefer MJPG when available
    if ([...fmtSel.options].some(o=>o.value==='MJPG')) fmtSel.value='MJPG';
    onFmtChange(slot, c);
  };
  fmtSel.onchange = ()=> onFmtChange(slot, capsBySlot[slot]||{});
  resSel.onchange = ()=> onResChange(slot, capsBySlot[slot]||{});
}

(async function(){
  await initSlot(1); await initSlot(2); await initSlot(3);
  const refreshAll = async () => {
    const j = await api('/api/devices');
    const devices = j.devices || [];
    if(!diffDevices(devices)) return;
    for(const slot of [1,2,3]){
      const devSel = $('#dev'+slot);
      const current = devSel.value;
      devSel.innerHTML='';
      devices.forEach(d=>{
        const opt = document.createElement('option');
        opt.value = d.device;
        opt.textContent = `${d.device}${d.name? ' — '+d.name:''}`;
        devSel.appendChild(opt);
      });
      if([...devSel.options].some(o=>o.value===current)){
        devSel.value = current;
      }
      const caps = await loadCaps(slot);
      const fmtSel = $('#fmt'+slot);
      if ([...fmtSel.options].some(o=>o.value==='MJPG')) fmtSel.value='MJPG';
      onFmtChange(slot, caps);
    }
  };
  setInterval(refreshAll, 3000);
})();
