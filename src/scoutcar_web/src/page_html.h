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
    .route-map { width:280px; max-width:90vw; height:auto; display:block; }
    .map-edge { stroke:#444; stroke-width:3; }
    .map-route { stroke:#9a9a9a; stroke-width:6; stroke-linecap:round; }
    .map-route.current { stroke:#ff9d32; }
    .map-node { fill:#202020; stroke:#aaa; stroke-width:2; }
    .map-node.current { stroke:#ff9d32; stroke-width:4; }
    .map-label { fill:#eee; font-size:12px; text-anchor:middle;
                 dominant-baseline:middle; pointer-events:none; }
    .map-legend { color:#bbb; font-size:12px; line-height:1.8; }
    .map-dot { display:inline-block; width:10px; height:10px; border-radius:50%;
               margin:0 5px 0 10px; }
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
    </div>
    <div id="routepanel" class="panel">
      <div class="panel-title">路线预览（当前起最多5段）</div>
      <svg id="route-map" class="route-map" viewBox="0 0 260 330"
           role="img" aria-label="当前与后续路线段">
      </svg>
      <div class="map-legend">
        <span class="map-dot" style="background:#ff9d32"></span>当前路段
        <span class="map-dot" style="background:#9a9a9a"></span>后续计划
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
  renderRouteMap();
}

const routeMapPositions = {
  1:[130,22], 2:[100,78], 3:[130,78], 4:[160,78],
  5:[40,134], 6:[100,134], 7:[160,134], 8:[220,134],
  9:[40,190], 10:[100,190], 11:[160,190], 12:[220,190],
  13:[40,246], 14:[100,246], 15:[160,246], 16:[220,246],
  17:[40,302], 18:[100,302], 19:[160,302], 20:[220,302]
};
const routeMapEdges = [
  [1,3],[2,3],[2,6],[3,4],[4,7],
  [5,6],[5,9],[6,7],[6,10],[7,8],[7,11],[8,12],
  [9,10],[9,13],[10,11],[10,14],[11,12],[11,15],[12,16],
  [13,14],[13,17],[14,15],[14,18],[15,16],[15,19],[16,20],
  [17,18],[18,19],[19,20]
];

function renderRouteMap() {
  const svg = document.getElementById('route-map');
  if (!svg) return;
  const m = recState.mission || {};
  const route = Array.isArray(m.route_nodes) ? m.route_nodes : [];
  const segmentIndex = Number(m.segment_index ?? -1);
  const routeSegmentIndex =
    segmentIndex - Number(m.route_segment_offset ?? 0);
  const waitingToStart = Number(m.mission_state) === 0;
  const hasCurrentRouteSegment =
    routeSegmentIndex >= 0 && routeSegmentIndex < route.length - 1;
  const showPlannedRoute =
    hasCurrentRouteSegment || waitingToStart || segmentIndex >= 0;
  const first = hasCurrentRouteSegment
    ? routeSegmentIndex
    : (showPlannedRoute ? 0 : route.length);
  const end = Math.min(route.length - 1, first + 5);
  const parts = [];

  for (const [a, b] of routeMapEdges) {
    const p1 = routeMapPositions[a], p2 = routeMapPositions[b];
    parts.push('<line class="map-edge" x1="' + p1[0] + '" y1="' + p1[1] +
      '" x2="' + p2[0] + '" y2="' + p2[1] + '"/>');
  }
  for (let i = first; i < end; ++i) {
    const a = Number(route[i]), b = Number(route[i + 1]);
    const p1 = routeMapPositions[a], p2 = routeMapPositions[b];
    if (!p1 || !p2 || a === b) continue;
    if (i === routeSegmentIndex) continue;
    parts.push('<line class="map-route planned' +
      '" x1="' + p1[0] + '" y1="' + p1[1] + '" x2="' +
      p2[0] + '" y2="' + p2[1] + '"/>');
  }
  if (hasCurrentRouteSegment && first + 1 < route.length) {
    const a = Number(route[first]), b = Number(route[first + 1]);
    const p1 = routeMapPositions[a], p2 = routeMapPositions[b];
    if (p1 && p2 && a !== b) {
      parts.push('<line class="map-route current" x1="' + p1[0] +
        '" y1="' + p1[1] + '" x2="' + p2[0] + '" y2="' + p2[1] + '"/>');
    }
  }
  for (const [node, point] of Object.entries(routeMapPositions)) {
    const isCurrent = hasCurrentRouteSegment &&
      Number(node) === Number(route[first]);
    parts.push('<circle class="map-node ' + (isCurrent ? 'current' : '') +
      '" cx="' + point[0] + '" cy="' + point[1] + '" r="13"/>');
    parts.push('<text class="map-label" x="' + point[0] + '" y="' +
      point[1] + '">' + node + '</text>');
  }
  svg.innerHTML = parts.join('');
  const title = document.querySelector('#routepanel .panel-title');
  if (title) {
    title.textContent = hasCurrentRouteSegment
      ? '路线预览（当前起最多5段）'
      : (waitingToStart && route.length
        ? '路线预览（即将执行前5段）'
        : (showPlannedRoute && route.length
          ? '路线预览（重规划路径前5段）'
          : '路线预览（暂无未来路段）'));
  }
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
