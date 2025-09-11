// Minimal WebRTC receiver per slot using /webrtc/offer?slot=N

async function startWebRTC(slot){
  const vid = document.getElementById('vid'+slot);
  const pc = new RTCPeerConnection();
  pc.addTransceiver('video', {direction:'recvonly'});
  pc.ontrack = (e)=>{ vid.srcObject = e.streams[0]; };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);
  const r = await fetch('/webrtc/offer?slot='+slot, {
    method:'POST', headers:{'Content-Type':'application/sdp'}, body:offer.sdp
  });
  const ans = await r.text();
  await pc.setRemoteDescription({type:'answer', sdp:ans});

  // expose for debugging
  window['pc'+slot] = pc;

  // update pill with video size when loaded
  vid.onloadedmetadata = async ()=>{
    const pill = document.getElementById('s'+slot);
    const w = vid.videoWidth, h = vid.videoHeight;
    pill.textContent = `${w}x${h}`;
    pill.className = 'pill ok';
    // optional FPS counter if supported
    if ('requestVideoFrameCallback' in HTMLVideoElement.prototype){
      let last=performance.now(), count=0;
      const tick = ()=>{
        count++;
        const now=performance.now();
        if (now-last>=1000){
          const fps=(count*1000)/(now-last);
          pill.textContent = `${w}x${h} @ ${fps.toFixed(1)}fps`;
          count=0; last=now;
        }
        vid.requestVideoFrameCallback(tick);
      };
      vid.requestVideoFrameCallback(tick);
    }
  };
}

// start 3 streams when DOM ready
document.addEventListener('DOMContentLoaded', ()=>{
  startWebRTC(1); startWebRTC(2); startWebRTC(3);
});

