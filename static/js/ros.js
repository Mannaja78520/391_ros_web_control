function quat2yaw(q){
  const x=q.x||0,y=q.y||0,z=q.z||0,w=q.w||0;
  const siny_cosp = 2*(w*z + x*y);
  const cosy_cosp = 1-2*(y*y + z*z);
  return Math.atan2(siny_cosp, cosy_cosp);
}

function connectWS(){
  const $ = s => document.querySelector(s);
  const stamp = () => { const el = $('#ros_ts'); if(el) el.textContent = new Date().toLocaleTimeString(); };
  const setNum = (selector, value, digits = 2, suffix = '') => {
    const el = $(selector);
    if(!el) return;
    const num = Number(value);
    if(Number.isFinite(num)){
      el.textContent = `${num.toFixed(digits)}${suffix}`;
    } else {
      el.textContent = '-';
    }
  };
  const setText = (selector, value) => {
    const el = $(selector);
    if(!el) return;
    el.textContent = (value ?? '-');
  };
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onmessage = ev => {
    try{
      const m = JSON.parse(ev.data);
      let handled = false;
      switch(m.topic){
        case '/status':
          setText('#ros_status', m.data?.status);
          handled = true;
          break;
        case '/controller/ping':
          setNum('#ros_ping', m.data?.value, 0);
          handled = true;
          break;
        case '/battery/voltage':
          setNum('#ros_battery', m.data?.value, 2);
          handled = true;
          break;
        case '/rack/distance':
          setNum('#ros_rack_distance', m.data?.value, 2);
          handled = true;
          break;
        case '/encoder': {
          const arr = Array.isArray(m.data?.data) ? m.data.data : [];
          setText('#ros_enc', arr.length ? arr.join(', ') : '-');
          handled = true;
          break;
        }
        case '/encoder/distance':
          setNum('#ros_encoder_dist', m.data?.value, 2);
          handled = true;
          break;
        case '/robot/stage':
          setText('#ros_stage', m.data?.value);
          handled = true;
          break;
        case '/servo/position':
          setNum('#ros_servo_position', m.data?.value, 1);
          handled = true;
          break;
        case '/seedling/index':
          setNum('#ros_seedling_index', m.data?.value, 0);
          handled = true;
          break;
        case '/imu': {
          const q = m.data?.orientation || {};
          const yaw = quat2yaw(q);
          setText('#ros_imu_yaw', Number.isFinite(yaw) ? yaw.toFixed(3) : '-');
          handled = true;
          break;
        }
        case '/odom': {
          const p = m.data?.pose?.pose?.position || {};
          const q = m.data?.pose?.pose?.orientation || {};
          const yaw = quat2yaw(q);
          const x = Number(p.x||0).toFixed(3);
          const y = Number(p.y||0).toFixed(3);
          const th = Number.isFinite(yaw) ? yaw.toFixed(3) : '-';
          setText('#ros_odom_pose', `${x}, ${y}, ${th}`);
          handled = true;
          break;
        }
        default:
          handled = false;
      }
      if(handled) stamp();
    }catch(e){ console.warn('bad msg', e); }
  };
  ws.onclose = ()=> setTimeout(connectWS, 1000);
}

// Start ROS WS after DOM ready
document.addEventListener('DOMContentLoaded', connectWS);
