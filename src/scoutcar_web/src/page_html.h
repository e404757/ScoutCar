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
    .obstacle-btn { padding:10px 20px; font-size:15px; border:1px solid #c55;
                    border-radius:8px; cursor:pointer; color:#fff; background:#922; }
    .obstacle-btn:disabled { opacity:.45; cursor:not-allowed; }
    .detect-btn { padding:10px 20px; font-size:15px; border:1px solid #3a8;
                  border-radius:8px; cursor:pointer; color:#fff; background:#176b4b; }
    .detect-btn.active { border-color:#c55; background:#922; }
    .detect-btn:disabled { opacity:.45; cursor:not-allowed; }
    .btp-btn { padding:9px 14px; font-size:14px; border:1px solid #587;
               border-radius:8px; cursor:pointer; color:#fff; background:#245; }
    .btp-btn.active { border-color:#4db; background:#176b5f; }
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
        <option value="front_raw">正前方原图</option>
        <option value="turn_raw">转向原图</option>
        <option value="front_detection">正前方 Detect</option>
        <option value="turn_detection">转向 Detect</option>
      </select>
    </div>
    <div class="status">
      <button class="obstacle-btn" id="obstacle-btn" onclick="triggerObstacle()">模拟遇到障碍</button>
      <span id="obstacle-text"></span>
    </div>
    <div class="status">
      <button class="detect-btn" id="detect-btn" onclick="triggerDetect()">开始侦察</button>
      <span id="detect-text"></span>
    </div>
    <div class="status">
      <span style="color:#aaa">BTP 调试：</span>
      <button class="btp-btn" id="btp-left" onclick="setBtpDebug('left')">左侧</button>
      <button class="btp-btn" id="btp-right" onclick="setBtpDebug('right')">右侧</button>
      <button class="btp-btn" id="btp-off" onclick="setBtpDebug('off')">退出并复位</button>
      <span id="btp-text"></span>
    </div>
  </div>
  <div style="display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start">
    <img class="live" src="/video_feed" alt="实时画面" style="max-width:640px">
    <div id="infopanel" class="panel">
      <div class="panel-sec">
        <div class="panel-title">任务状态</div>
        <div id="mi-mission" class="kv">等待数据…</div>
      </div>
      <div class="panel-sec">
        <div class="panel-title">道路跟踪参数</div>
        <div id="mi-boundary" class="kv">等待数据…</div>
      </div>
      <div class="panel-sec">
        <div class="panel-title">BTP 调试</div>
        <div id="mi-btp" class="kv">未开启</div>
      </div>
    </div>
  </div>
</div>


<script>
// ══════════ 实时页 ══════════
let recState = { recording: false, bag_recording: false, detecting: false };

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
  const detectBtn = document.getElementById('detect-btn');
  detectBtn.disabled = false;
  detectBtn.className = recState.detecting ? 'detect-btn active' : 'detect-btn';
  detectBtn.textContent = recState.detecting ? '停止侦察' : '开始侦察';
  const btp = recState.btp_debug || {};
  document.getElementById('btp-left').className =
    btp.active && btp.pose === 2 ? 'btp-btn active' : 'btp-btn';
  document.getElementById('btp-right').className =
    btp.active && btp.pose === 3 ? 'btp-btn active' : 'btp-btn';
  const viewSelect = document.getElementById('view-select');
  if (viewSelect && ['perception','ipm','front_raw','turn_raw',
                     'front_detection','turn_detection'].includes(recState.view_mode)) {
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
  const m = recState.mission || {};
  const missionHtml = !m.online
    ? '<span style="color:#766">Mission 离线 / 无数据</span>'
    : '阶段　<b>' + (m.mission_state_name || '—') + '</b>' +
      '\n状态版本　<b>' + (m.revision ?? '—') + '</b>' +
      '\n当前路段　<b>' + (m.segment_start ?? '—') + ' → ' +
        (m.segment_goal ?? '—') + '（#' + (m.segment_index ?? '—') + '）</b>' +
      '\n到达后动作　<b>' + (m.arrival_action_name || '—') + '</b>' +
      '\n前视相机　<b>' + (m.front_camera_pose_name || '—') + '</b>' +
      '\n转向相机　<b>' + (m.turn_camera_pose_name || '—') + '</b>' +
      '\n剩余任务　<b>固定 ' + (m.fixed_remaining ?? '—') +
        ' / 随机 ' + (m.random_remaining ?? '—') + '</b>';
  document.getElementById('mi-mission').innerHTML = missionHtml;

  const b = recState.boundary || {};
  const statusText = ({0:'正常', 1:'无分割结果', 2:'道路跟踪失败',
                       3:'推理错误'})[b.status] ?? '未知状态';
  const html = !b.online
    ? '<span style="color:#766">离线 / 无数据</span>'
    : '状态　<b>' + statusText + '</b>' +
      '\n偏差　<b>' + (b.deviation ?? '—') + ' px</b>' +
      '\n路面中心　<b>x = ' + (b.road_center ?? '—') + ' px</b>' +
      '\n左右边界　<b>' + (b.left ?? '—') + ' / ' + (b.right ?? '—') + ' px</b>' +
      '\n边界来源　<b>' + (b.boundary_source || '—') + '</b>' +
      '\n扫描行　<b>y = ' + (b.scan_y ?? '—') + ' px</b>' +
      '\n检测道路宽度　<b>' + (b.min_width ?? '—') +
      ' ~ ' + (b.max_width ?? '—') + ' px</b>' +
      '\n前视参考 x　<b>' + (b.front_reference_x ?? '—') + ' px</b>' +
      '\n转向参考 x　<b>' + (b.turn_reference_x ?? '—') + ' px</b>';
  document.getElementById('mi-boundary').innerHTML = html;

  const d = recState.btp_debug || {};
  const debugStatus = ({0:'正常', 1:'无分割结果', 2:'道路跟踪失败',
                        3:'推理错误'})[d.status] ?? '未知状态';
  const btpHtml = !d.active
    ? '未开启（车身转向始终禁用）'
    : '方向　<b>' + (d.pose === 2 ? '左侧' : '右侧') + '</b>' +
      '\n感知　<b>' + (d.online ? debugStatus : '等待转向相机数据') + '</b>' +
      '\n参考中心　<b>x = ' + (d.reference_x ?? '—') + ' px</b>' +
      '\n偏差　<b>' + (d.online ? d.deviation : '—') + ' px</b>' +
      '\n有效范围　<b>' + (d.min_deviation ?? '—') + ' ～ ' +
        (d.max_deviation ?? '—') + ' px</b>' +
      '\n连续满足　<b>' + (d.consecutive_frames ?? 0) + ' / ' +
        (d.required_frames ?? '—') + ' 帧</b>' +
      '\nBTP 条件　<b style="color:' +
        (d.condition_met ? '#4d8' : '#e88') + '">' +
        (d.condition_met ? '满足' : '未满足') + '</b>' +
      '\n安全状态　<b>仅调试，不触发车身</b>';
  document.getElementById('mi-btp').innerHTML = btpHtml;
}

async function toggleRecord() {
  const on = recState.recording;
  try {
    await fetch('/api/record/' + (on ? 'stop' : 'start'), { method: 'POST' });
    if (!on) { await sleep(200); }   // 等片段创建
    await refreshStatus();
  } catch (e) {}
}

async function triggerObstacle() {
  const btn = document.getElementById('obstacle-btn');
  const text = document.getElementById('obstacle-text');
  btn.disabled = true;
  text.textContent = '发送中…';
  try {
    const response = await fetch('/api/obstacle', { method: 'POST' });
    if (!response.ok) throw new Error('发送失败');
    text.textContent = '已发送';
  } catch (e) {
    text.textContent = '发送失败';
  }
  setTimeout(() => {
    btn.disabled = false;
    text.textContent = '';
  }, 1000);
}

async function triggerDetect() {
  const btn = document.getElementById('detect-btn');
  const text = document.getElementById('detect-text');
  btn.disabled = true;
  text.textContent = '发送中…';
  try {
    const response = await fetch('/api/detect/toggle', { method: 'POST' });
    if (!response.ok) throw new Error('发送失败');
    recState = await response.json();
    text.textContent = recState.detecting ? '侦察已开始' : '侦察已停止';
    await refreshStatus();
  } catch (e) {
    text.textContent = '发送失败';
  }
  setTimeout(() => {
    btn.disabled = false;
    text.textContent = '';
  }, 1000);
}

async function setBtpDebug(pose) {
  const text = document.getElementById('btp-text');
  text.textContent = '发送中…';
  try {
    const response = await fetch(
      '/api/btp-debug/set?pose=' + encodeURIComponent(pose),
      { method: 'POST' });
    if (!response.ok) throw new Error('发送失败');
    recState = await response.json();
    text.textContent = pose === 'off' ? '已退出并复位' : '调试已开启';
    if (pose !== 'off') await switchView('perception');
    await refreshStatus();
  } catch (e) {
    text.textContent = '发送失败';
  }
  setTimeout(() => { text.textContent = ''; }, 1200);
}

function sleep(ms) { return new Promise(res => setTimeout(res, ms)); }

refreshStatus();
setInterval(refreshStatus, 250);
</script>
</body>
</html>
)HTML";
