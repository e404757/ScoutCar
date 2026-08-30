#pragma once

// 网页前端（内嵌，编译期打包，与原工程 web_server.cc 的 HTML 内嵌思路一致）
// 两个页签：
//   实时画面 —— MJPEG 推流 + 一个录像按钮（用户需求：一个按钮起停）
//   录像回放 —— 完整播放器（播放/暂停/进度条/逐帧）+ 导出本帧补标
//
// 回放实现：服务端按帧号出 JPEG（/frame?file=base&idx=N&kind=mask），
// 播放器逐帧取图（本地局域网，640x480 JPEG 每帧几十 KB，足够流畅），
// 天然支持暂停/拖动/逐帧，无进度条精度损失。

static const char * kPageHtml = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
  <meta charset="utf-8">
  <title>scoutcar 实时画面 / 录像回放</title>
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
  </style>
</head>
<body>
<div class="tabbar">
  <button id="tabbtn-live" class="on" onclick="switchTab('live')">实时画面</button>
  <button id="tabbtn-replay" onclick="switchTab('replay')">录像回放</button>
</div>

<!-- ══════════ 实时画面 ══════════ -->
<div id="tab-live" class="tab">
  <h2>实时画面（叠加：掩膜 / 边界 / 任务状态）</h2>
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
  <img class="live" src="/video_feed" alt="实时画面">
</div>

<!-- ══════════ 录像回放 ══════════ -->
<div id="tab-replay" class="tab" style="display:none">
  <h2>录像回放与补标导出</h2>
  <div class="status">回放的是带掩膜的叠加画面（xxx_mask.avi），肉眼找"模型没分割好"的弱帧；
    导出时从原始文件抽同一帧（干净画面）存 JPEG 到补标目录。</div>
  <div id="reclist"></div>
  <div id="player" style="display:none">
    <img id="pimg" width="640" alt="回放帧">
    <div class="pctl">
      <button class="main" id="playbtn" onclick="togglePlay()">播放</button>
      <button onclick="step(-10)">«10</button>
      <button onclick="step(-1)">‹</button>
      <button onclick="step(1)">›</button>
      <button onclick="step(10)">10»</button>
      <button class="export" onclick="exportFrame('raw')">导出本帧(原始·补标)</button>
      <button class="exportm" onclick="exportFrame('mask')">导出本帧(叠加)</button>
      <span id="toast"></span>
    </div>
    <input type="range" id="slider" min="0" max="0" value="0" step="1"
           oninput="onSlider()">
    <div class="frameinfo">帧 <span id="cur">0</span> / <span id="total">0</span>
      &nbsp;·&nbsp; <span id="finfo"></span></div>
  </div>
</div>

<script>
// ══════════ 实时页 ══════════
let recState = { record_enable: false, recording: false, fps: 0 };

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
    txt.textContent = recState.recording ? '录像中（原始 + 叠加双文件）' : '录像未开始';
  }
  info.textContent = recState.fps > 0 ? ('推流 ' + recState.fps.toFixed(1) + ' fps') : '';
  renderCameras();
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
    html += '<button class="' + cls.trim() + nosig + '" onclick="switchCam(\'' + c.topic + '\')">' +
            name + (c.has_frame ? '' : ' (无信号)') + '</button>';
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
    if (!on) { await loadRecordings(); }   // 新片段出现，刷新列表
  } catch (e) {}
}

// ══════════ 回放页 ══════════
const P = { base: null, info: null, idx: 0, playing: false, timer: null };

function switchTab(name) {
  document.getElementById('tab-live').style.display = name === 'live' ? '' : 'none';
  document.getElementById('tab-replay').style.display = name === 'replay' ? '' : 'none';
  document.getElementById('tabbtn-live').classList.toggle('on', name === 'live');
  document.getElementById('tabbtn-replay').classList.toggle('on', name === 'replay');
  if (name === 'replay') { loadRecordings(); pause(); }
}

async function loadRecordings() {
  let list = [];
  try {
    const r = await fetch('/api/recordings');
    list = await r.json();
  } catch (e) {}
  const box = document.getElementById('reclist');
  if (!list.length) {
    box.innerHTML = '<div class="status">暂无录像片段（在"实时画面"页点击开始录像）</div>';
    document.getElementById('player').style.display = 'none';
    return;
  }
  let html = '';
  for (const s of list) {
    const sel = P.base === s.base ? ' sel' : '';
    html += '<button class="reccell' + sel + '" onclick="openSegment(\'' + s.base + '\')">' +
            s.base + '　' + s.frames + ' 帧 @ ' + s.fps.toFixed(1) + 'fps　' +
            s.width + 'x' + s.height + '</button>';
  }
  box.innerHTML = html;
}

async function openSegment(base) {
  try {
    const r = await fetch('/api/segment_info?file=' + base);
    const info = await r.json();
    if (!info.ok) { alert('无法打开片段 ' + base); return; }
    P.base = base; P.info = info; P.idx = 0; pause();
    document.getElementById('slider').max = info.frames - 1;
    document.getElementById('finfo').textContent =
      info.frames + ' 帧 / ' + info.fps.toFixed(1) + ' fps / ' + info.width + 'x' + info.height;
    document.getElementById('player').style.display = '';
    loadRecordings();   // 高亮当前
    showFrame();
  } catch (e) {}
}

function showFrame() {
  if (!P.info) return;
  const n = P.info.frames;
  if (n <= 0) return;
  if (P.idx < 0) P.idx = 0;
  if (P.idx > n - 1) P.idx = n - 1;
  const img = document.getElementById('pimg');
  img.src = '/frame?file=' + P.base + '&idx=' + P.idx + '&kind=mask&t=' + Date.now();
  document.getElementById('cur').textContent = P.idx;
  document.getElementById('total').textContent = n - 1;
  document.getElementById('slider').value = P.idx;
}

function togglePlay() {
  if (P.playing) { pause(); } else { play(); }
}

function play() {
  if (!P.info || P.info.frames < 2) return;
  P.playing = true;
  document.getElementById('playbtn').textContent = '暂停';
  const ms = Math.max(20, Math.round(1000 / P.info.fps));
  P.timer = setInterval(() => {
    if (P.idx >= P.info.frames - 1) { pause(); return; }
    P.idx++;
    showFrame();
  }, ms);
}

function pause() {
  P.playing = false;
  if (P.timer) { clearInterval(P.timer); P.timer = null; }
  document.getElementById('playbtn').textContent = '播放';
}

function step(d) {
  if (!P.info) return;
  if (P.playing) pause();           // 手动逐帧时暂停
  P.idx += d;
  showFrame();
}

function onSlider() {
  if (!P.info) return;
  P.idx = parseInt(document.getElementById('slider').value, 10);
  showFrame();
}

async function exportFrame(kind) {
  if (!P.base) return;
  if (P.playing) pause();
  try {
    const r = await fetch('/api/export?file=' + P.base + '&idx=' + P.idx + '&kind=' + kind,
                          { method: 'POST' });
    const j = await r.json();
    const toast = document.getElementById('toast');
    toast.textContent = j.ok ? ('已导出 → ' + j.path) : ('导出失败: ' + (j.reason || ''));
  } catch (e) {}
}

function sleep(ms) { return new Promise(res => setTimeout(res, ms)); }

refreshStatus();
setInterval(refreshStatus, 1000);
</script>
</body>
</html>
)HTML";
