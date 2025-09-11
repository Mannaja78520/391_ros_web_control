function quat2yaw(q){
  const x=q.x||0,y=q.y||0,z=q.z||0,w=q.w||0;
  const siny_cosp = 2*(w*z + x*y);
  const cosy_cosp = 1-2*(y*y + z*z);
  return Math.atan2(siny_cosp, cosy_cosp);
}

function connectWS(){
  const $ = s => document.querySelector(s);
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onmessage = ev => {
    try{
      const m = JSON.parse(ev.data);
      if(m.topic==='\/status'){
        $('#ros_status').textContent = m.data?.status ?? '-';
        $('#ros_ts').textContent = new Date().toLocaleTimeString();
      } else if(m.topic==='\/encoder'){
        const arr = m.data?.data || [];
        $('#ros_enc').textContent = arr.join(', ');
        $('#ros_ts').textContent = new Date().toLocaleTimeString();
      } else if(m.topic==='\/imu'){
        const q = m.data?.orientation || {}; const yaw = quat2yaw(q);
        $('#ros_imu_yaw').textContent = yaw.toFixed(3);
        $('#ros_ts').textContent = new Date().toLocaleTimeString();
      } else if(m.topic==='\/odom'){
        const p = m.data?.pose?.pose?.position || {}; const q = m.data?.pose?.pose?.orientation || {};
        const yaw = quat2yaw(q);
        $('#ros_odom_pose').textContent = `${(p.x||0).toFixed(3)}, ${(p.y||0).toFixed(3)}, ${yaw.toFixed(3)}`;
        $('#ros_ts').textContent = new Date().toLocaleTimeString();
      }
    }catch(e){ console.warn('bad msg', e); }
  };
  ws.onclose = ()=> setTimeout(connectWS, 1000);
}

// Start ROS WS after DOM ready
document.addEventListener('DOMContentLoaded', connectWS);

