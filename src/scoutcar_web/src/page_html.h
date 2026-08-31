#pragma once

// 网页前端（内嵌，编译期打包，与原工程 web_server.cc 的 HTML 内嵌思路一致）
// 实时画面：MJPEG 推流 + 相机切换 + 一个录像按钮 + 状态侧边面板。
// 录像回放与补标导出已移至 Windows 标注平台，车端只保留录制。

static const char * kPageHtml = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
  <meta charset="utf-8">
  <title>scoutcar 实时画面</title>
  <style>
    body { background:#111; color:#eee; font-family:Arial, sans-serif; margin:0; }
    .tabbar { background:#1a1a1a; padding:10px 16px; border-bottom:1px solid #333; }
    .tabbar button { background:#333; color:#eee; border:none; padding:8px 18px;
                     margin-right:8px; border-radius:6px; cursor:pointer; font-size:15px; }
    .tabbar button.on { background:#2a7; }
    .tab { padding:14px 16px; }
    h2 { margin:4px 0 10px; font-size:18px; }
    .status { margin:8px 0; font-size:13px; color:#aaa; }
    .recbtn { padding:10px 26px; font-size:16px; border:none; border-radius:8px;
              cursor:pointer; color:#fff; }
    .recbtn.idle { background:#2a7; }
    .recbtn.rec { background:#d33; }
    .recbtn.off { background:#555; cursor:not-allowed; }
    .dot { display:inline-block; width:12px; height:12px; border-radius:50%;
           background:#333; vertical-align:middle; margin-right:6px; }
    .dot.on { background:#f44; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity:.25; } }
    img.live { border:2px solid #444; max-width:95vw; }
    #reclist { margin:6px 0 12px; }
    #reclist button { display:block; width:100%; text-align:left; background:#222;
                      color:#ddd; border:1px solid #333; border-radius:6px;
                      padding:8px 10px; margin-bottom:6px; cursor:pointer; font-size:14px; }
    #reclist button:hover { background:#2a2a2a; border-color:#2a7; }
    #reclist button.sel { border-color:#2a7; background:#1c3327; }
    #player { margin-top:8px; }
    #pimg { border:2px solid #444; background:#000; }
    .pctl { margin-top:10px; }
    .pctl button { background:#333; color:#eee; border:none; border-radius:6px;
                   padding:8px 14px; margin-right:6px; cursor:pointer; font-size:14px; }
    .pctl button.main { background:#2a7; min-width:72px; }
    .pctl button.export { background:#17a; }
    .pctl button.exportm { background:#a71; }
    #slider { width:100%; margin:10px 0 4px; }
    .frameinfo { font-size:13px; color:#aaa; }
    #toast { color:#8d8; font-size:13px; margin-left:10px; }
    #cam-btns button { background:#333; color:#ddd; border:1px solid #444; border-radius:6px;
                       padding:4px 12px; margin-right:6px; cursor:pointer; font-size:13px; }
    #cam-btns button.on { background:#2a7; color:#fff; border-color:#2a7; }
    #cam-btns button.nosig { opacity:.55; }
    .panel { background:#181818; border:1px solid #333; border-radius:8px;
             padding:10px 14px; min-width:230px; font-size:13px; }
    .panel-sec { margin-bottom:12px; }
    .panel-sec:last-child { margin-bottom:0; }
    .panel-title { color:#2a7; font-weight:bold; margin-bottom:6px; font-size:13px; }
    .kv { line-height:1.8; color:#ccc; white-space:pre-line; }
    .kv b { color:#eee; }
    .good { color:#4d8; } .warn { color:#fc4; } .bad { color:#f66; }
    #width-chart { width:230px; height:105px; background:#101010;
                   border:1px solid #333; border-radius:4px; }
    .legend { color:#888; font-size:11px; margin-top:3px; }
  </style>
</head>
<body>
<!-- ══════════ 实时画面 ══════════ -->
<div id="tab-live" class="tab">
  <h2>实时画面<span style="font-size:13px;color:#aaa;margin-left:10px">感知相机显示同帧叠加（掩膜/边界），状态与调试数值见右侧面板</span></h2>
  <div class="status">
    <span class="dot" id="rec-dot"></span>
    <span id="rec-text">录像未开始</span>
    <button class="recbtn idle" id="rec-btn" onclick="toggleRecord()">● 开始录像</button>
    <span id="live-info" style="margin-left:12px"></span>
  </div>
  <div id="cams" style="margin:6px 0 8px">
    <span style="color:#aaa;font-size:13px">相机：</span>
    <span id="cam-btns"></span>
  </div>
  <div style="display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start">
    <img class="live" src="/video_feed" alt="实时画面" style="max-width:640px">
    <div id="infopanel" class="panel">
      <div class="panel-sec">
        <div class="panel-title">任务状态</div>
        <div id="mi-mission" class="kv">等待数据…</div>
      </div>
      <div class="panel-sec">
        <div class="panel-title">道路边界</div>
        <div id="mi-boundary" class="kv">等待数据…</div>
      </div>
      <div class="panel-sec">
        <div class="panel-title">宽度历史（最近 10 秒）</div>
        <canvas id="width-chart" width="230" height="105"></canvas>
        <div class="legend">原始 <span style="color:#fc4">━</span>　修正 <span style="color:#4d8">━</span>　范围 <span style="color:#777">━</span></div>
      </div>
      <div class="panel-sec">
        <div class="panel-title">系统</div>
        <div id="mi-sys" class="kv"></div>
      </div>
    </div>
  </div>
</div>


<script>
// ══════════ 实时页 ══════════
let recState = { record_enable: false, recording: false, bag_enable: false, bag_recording: false, fps: 0 };
let widthHistory = [];
let lastBoundaryStamp = '';

async function refreshStatus() {
  try {
    const r = await fetch('/api/status');
    recState = await r.json();
  } catch (e) { return; }
  const btn = document.getElementById('rec-btn');
  const dot = document.getElementById('rec-dot');
  const txt = document.getElementById('rec-text');
  const info = document.getElementById('live-info');
  if (!recState.record_enable) {
    btn.disabled = true;
    btn.className = 'recbtn off';
    btn.textContent = '未启用';
    dot.className = 'dot';
    txt.textContent = '录像未启用（record_enable=false）';
  } else {
    btn.disabled = false;
    btn.className = recState.recording ? 'recbtn rec' : 'recbtn idle';
    btn.textContent = recState.recording ? '■ 停止录像' : '● 开始录像';
    dot.className = 'dot ' + (recState.recording ? 'on' : '');
    if (recState.recording) {
      txt.textContent = recState.bag_enable
        ? (recState.bag_recording ? '录像中（双 AVI + rosbag）' : '录像中（rosbag 启动失败）')
        : '录像中（原始 + 叠加双文件）';
    } else {
      txt.textContent = '录像未开始';
    }
  }
  info.textContent = recState.fps > 0 ? ('推流 ' + recState.fps.toFixed(1) + ' fps') : '';
  renderCameras();
  renderInfoPanel();
  updateWidthHistory();
  drawWidthChart();
}

// 侧边信息面板：任务状态 / 道路边界 / 系统（数值来自 /api/status，不再画在图上）
function renderInfoPanel() {
  const m = recState.mission || {};
  const b = recState.boundary || {};
  const turning = m.driving_state === 'TURNING';
  let mh = '状态　<b class="' + (turning ? 'warn' : 'good') + '">' +
           (turning ? '转向中' : '正常') + '</b>';
  document.getElementById('mi-mission').innerHTML = mh;

  let bh;
  if (!b.online) {
    bh = '<span style="color:#766">离线 / 无数据</span>';
  } else {
    const ws = b.width_status || 'NO_WIDTH';
    const wc = ws === 'NORMAL' ? 'good' : (ws === 'TOO_WIDE' ? 'warn' : 'bad');
    const algorithmText = b.algorithm_valid ? '<span class="good">VALID</span>' : '<span class="bad">INVALID / -999</span>';
    const controlText = b.control_ready
      ? '<span class="good">已启用　输出 ' + signed(b.deviation) + '</span>'
      : '<span class="warn">未启用　输出 -999</span>';
    bh = '预瞄　<b>' + (b.preview_mode || '—') + '</b>　y=' + (b.scan_y ?? '—') + '　pt=' + fmt(b.selected_pt, 2) +
         '\n来源　<b>' + (b.boundary_source || '—') + '</b>' +
         '\n\n道路边界　<b>' + val(b.left) + ' / ' + val(b.right) + '</b>' +
         '\n道路宽度　<b>' + val(b.width) + ' px</b>' +
         '\n允许范围　<b>' + val(b.min_width) + ' ~ ' + val(b.max_width) + ' px</b>' +
         '\n宽度状态　<span class="' + wc + '"><b>' + ws + '</b></span>' +
         '\n\n道路中心　<b>' + val(b.road_center) + ' px</b>' +
         '\n算法偏差　<b>' + signed(b.algorithm_deviation) + '</b>' +
         '\n算法状态　' + algorithmText +
         '\n控制状态　' + controlText;
  }
  document.getElementById('mi-boundary').innerHTML = bh;

  document.getElementById('mi-sys').innerHTML =
    '感知帧率　<b>' + (recState.fps > 0 ? recState.fps.toFixed(1) + ' fps' : '—') + '</b>' +
    '\n录像目录　' + (recState.record_dir || '—');
}

function val(v) { return (v === undefined || v === null || v < 0) ? '—' : v; }
function fmt(v, n) { return Number.isFinite(Number(v)) ? Number(v).toFixed(n) : '—'; }
function signed(v) { return v === undefined ? '—' : ((v > 0 ? '+' : '') + v); }

function updateWidthHistory() {
  const b = recState.boundary || {};
  if (!b.online || !b.width) return;
  const key = [b.width,b.deviation,b.scan_y,Date.now() >> 8].join(':');
  if (key === lastBoundaryStamp) return;
  lastBoundaryStamp = key;
  widthHistory.push({t:Date.now(), raw:b.width, fixed:b.width,
                     min:b.min_width, max:b.max_width});
  const cutoff = Date.now() - 10000;
  widthHistory = widthHistory.filter(p => p.t >= cutoff).slice(-80);
}

function drawWidthChart() {
  const c=document.getElementById('width-chart'),ctx=c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  if(widthHistory.length<2) return;
  let values=[]; for(const p of widthHistory) values.push(p.raw,p.fixed,p.min,p.max);
  const lo=Math.max(0,Math.min(...values)-20), hi=Math.max(lo+1,Math.max(...values)+20);
  const t0=Date.now()-10000, x=t=>((t-t0)/10000)*c.width;
  const y=v=>c.height-((v-lo)/(hi-lo))*(c.height-8)-4;
  function line(field,color,width) {
    ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;
    widthHistory.forEach((p,i)=>{const px=x(p.t),py=y(p[field]);i?ctx.lineTo(px,py):ctx.moveTo(px,py);});ctx.stroke();
  }
  line('min','#555',1);line('max','#555',1);line('raw','#fc4',2);line('fixed','#4d8',2);
  ctx.fillStyle='#888';ctx.font='10px Arial';ctx.fillText(Math.round(hi),2,10);ctx.fillText(Math.round(lo),2,c.height-3);
}

// 渲染相机切换按钮（来自 /api/status 的 cameras）
function renderCameras() {
  const box = document.getElementById('cam-btns');
  const cams = recState.cameras || [];
  if (!cams.length) { box.innerHTML = ''; return; }
  let html = '';
  for (const c of cams) {
    const name = c.topic.replace('/camera/', '').replace('/image_raw', '');
    const cls = c.selected ? ' on' : '';
    const nosig = c.has_frame ? '' : ' nosig';
    const mark = c.overlay ? '（叠加）' : '（原始）';
    html += '<button class="' + cls.trim() + nosig + '" onclick="switchCam(\'' + c.topic + '\')">' +
            name + mark + (c.has_frame ? '' : ' ·无信号') + '</button>';
  }
  box.innerHTML = html;
}

async function switchCam(topic) {
  try {
    await fetch('/api/camera/set?topic=' + encodeURIComponent(topic), { method: 'POST' });
    // 短延迟后刷新 /video_feed（服务端已切换，img.src 需重设才会重连）
    await refreshStatus();
    const img = document.querySelector('img.live');
    if (img) { img.src = '/video_feed?' + Date.now(); }
  } catch (e) {}
}

async function toggleRecord() {
  const on = recState.recording;
  try {
    await fetch('/api/record/' + (on ? 'stop' : 'start'), { method: 'POST' });
    if (!on) { await sleep(200); }   // 等片段创建
    await refreshStatus();
  } catch (e) {}
}

function sleep(ms) { return new Promise(res => setTimeout(res, ms)); }

refreshStatus();
setInterval(refreshStatus, 250);
</script>
</body>
</html>
)HTML";
