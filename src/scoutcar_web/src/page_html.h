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
    .tab { padding:14px 16px; }
    h2 { margin:4px 0 10px; font-size:18px; }
    .controls { display:flex; flex-wrap:wrap; align-items:center; gap:36px; margin:8px 0; }
    .status { font-size:13px; color:#aaa; }
    .view-control { white-space:nowrap; }
    .recbtn { padding:10px 26px; font-size:16px; border:none; border-radius:8px;
              cursor:pointer; color:#fff; }
    .recbtn.idle { background:#2a7; }
    .recbtn.rec { background:#d33; }
    .dot { display:inline-block; width:12px; height:12px; border-radius:50%;
           background:#333; vertical-align:middle; margin-right:6px; }
    .dot.on { background:#f44; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity:.25; } }
    img.live { border:2px solid #444; max-width:95vw; }
    #view-select { background:#252525; color:#eee; border:1px solid #555;
                   border-radius:6px; padding:6px 10px; font-size:14px; cursor:pointer; }
    .panel { background:#181818; border:1px solid #333; border-radius:8px;
             padding:10px 14px; min-width:230px; font-size:13px; }
    .panel-sec { margin-bottom:12px; }
    .panel-sec:last-child { margin-bottom:0; }
    .panel-title { color:#2a7; font-weight:bold; margin-bottom:6px; font-size:13px; }
    .kv { line-height:1.8; color:#ccc; white-space:pre-line; }
    .kv b { color:#eee; }
  </style>
</head>
<body>
<!-- ══════════ 实时画面 ══════════ -->
<div id="tab-live" class="tab">
  <h2>实时画面</h2>
  <div class="controls">
    <div class="status">
      <span class="dot" id="rec-dot"></span>
      <span id="rec-text">录像未开始</span>
      <button class="recbtn idle" id="rec-btn" onclick="toggleRecord()">● 开始录像</button>
    </div>
    <div class="view-control">
      <label for="view-select" style="color:#aaa;font-size:13px">显示画面：</label>
      <select id="view-select" onchange="switchView(this.value)">
        <option value="perception">普通感知（自动相机）</option>
        <option value="ipm">逆透视</option>
        <option value="usb_raw">USB 原图</option>
        <option value="mipi_raw">MIPI 原图</option>
      </select>
    </div>
  </div>
  <div style="display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start">
    <img class="live" src="/video_feed" alt="实时画面" style="max-width:640px">
    <div id="infopanel" class="panel">
      <div class="panel-sec">
        <div class="panel-title">道路跟踪参数</div>
        <div id="mi-boundary" class="kv">等待数据…</div>
      </div>
    </div>
  </div>
</div>


<script>
// ══════════ 实时页 ══════════
let recState = { recording: false, bag_recording: false };

async function refreshStatus() {
  try {
    const r = await fetch('/api/status');
    recState = await r.json();
  } catch (e) { return; }
  const btn = document.getElementById('rec-btn');
  const dot = document.getElementById('rec-dot');
  const txt = document.getElementById('rec-text');
  btn.disabled = false;
  btn.className = recState.recording ? 'recbtn rec' : 'recbtn idle';
  btn.textContent = recState.recording ? '■ 停止录像' : '● 开始录像';
  dot.className = 'dot ' + (recState.recording ? 'on' : '');
  if (recState.recording) {
    txt.textContent = recState.bag_recording
      ? '录像中（双 AVI + rosbag）'
      : '录像中（rosbag 启动失败）';
  } else {
    txt.textContent = '录像未开始';
  }
  const viewSelect = document.getElementById('view-select');
  if (viewSelect && ['perception','ipm','usb_raw','mipi_raw'].includes(recState.view_mode)) {
    viewSelect.value = recState.view_mode;
  }
  renderInfoPanel();
}

async function switchView(mode) {
  try {
    const response = await fetch('/api/view/set?mode=' + encodeURIComponent(mode), { method: 'POST' });
    if (!response.ok) throw new Error('切换失败');
    await refreshStatus();
    const img = document.querySelector('img.live');
    if (img) { img.src = '/video_feed?' + Date.now(); }
  } catch (e) {}
}

// 侧边参数面板：显示当前跟踪算法使用的阈值和扫描行。
function renderInfoPanel() {
  const b = recState.boundary || {};
  const html = !b.online
    ? '<span style="color:#766">离线 / 无数据</span>'
    : '扫描行　<b>y = ' + (b.scan_y ?? '—') + ' px</b>' +
      '\n道路宽度阈值　<b>' + (b.min_width ?? '—') +
      ' ~ ' + (b.max_width ?? '—') + ' px</b>';
  document.getElementById('mi-boundary').innerHTML = html;
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
