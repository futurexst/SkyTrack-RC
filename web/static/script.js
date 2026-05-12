/* =============================================
 * MISSION_CTRL // dashboard front-end script
 * ESP32 ROS 토픽 기반 대시보드 연동 버전
 * ============================================= */

const state = {
  // ESP32 / ROS 기반 상태값
  encoder: 0,
  safety: '안전',
  rssi: -100,
  wifi: 0,
  wifiCh: '—',
  loopRate: 0,
  loopDt: 0,
  maxLoopDt: 0,
  heapKB: 0,
  minHeapKB: 0,
  espIp: '',

  // 기존 호환 필드. 실제 화면에서는 ESP32 지표로 대체됨.
  battery: 100,
  voltage: 0,
  cpu: 0,
  memory: 0,

  // 주행/제어 상태
  leftSpeed: 0.0,
  rightSpeed: 0.0,
  distance: 0.0,
  heading: 0.0,
  heading_error: 0.0,
  lateral_error: 0.0,
  lookahead_error: 0.0,
  pwm_l: 0,
  pwm_r: 0,
  gyro_x: 0.0,
  lastCommand: 'S',

  // MAP_005 좌표/행렬
  robotX: 0,
  robotY: 0,
  prevStatTime: null,
  trajectory: [],
  transform: {
    theta_deg: 0,
    cx: 0,
    cy: 0,
    s: 4.27,
    xc: 0,
    yc: 0
  }
};

/* ---------- Analytics graph (Chart.js) ---------- */
const ANA_MAX_PTS = 80;
const anaBuf = {
  lat:  [],   // /esp32/debug_status Lat (px)
  head: []    // /esp32/debug_status Head (°)
};
const _anaLabels = [];
let _anaChart = null;

function initAnalyticsChart() {
  const canvas = document.getElementById('ana-chartjs');
  if (!canvas || _anaChart || typeof Chart === 'undefined') return;

  const gridColor = 'rgba(255,255,255,0.05)';
  const tickColor = 'rgba(255,255,255,0.32)';
  const monoFont  = "'JetBrains Mono', 'Courier New', monospace";

  const crosshairPlugin = {
    id: 'centerValueGuide',
    afterDatasetsDraw(chart) {
      const { ctx, chartArea, scales } = chart;
      if (!chartArea || !_anaLabels.length) return;

      let idx = Math.round((_anaLabels.length - 1) / 2); // 기본: 그래프 중앙값 표시
      const active = chart.tooltip?.getActiveElements?.() || [];
      if (active.length) idx = active[0].index;          // 마우스 올리면 해당 지점 표시
      idx = Math.max(0, Math.min(_anaLabels.length - 1, idx));

      const x = scales.x.getPixelForValue(idx);
      const lat = anaBuf.lat[idx];
      const head = anaBuf.head[idx];
      if (!Number.isFinite(lat) || !Number.isFinite(head)) return;

      ctx.save();
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(255,255,255,0.22)';
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(x, chartArea.top);
      ctx.lineTo(x, chartArea.bottom);
      ctx.stroke();
      ctx.setLineDash([]);

      function valueTag(value, label, color, yOffset = 0) {
        const y = scales.y.getPixelForValue(value);
        const text = `${label} ${Number(value).toFixed(1)}`;
        ctx.font = `10px ${monoFont}`;
        const w = ctx.measureText(text).width + 10;
        const h = 16;
        let tx = x + 8;
        if (tx + w > chartArea.right) tx = x - w - 8;
        const ty = Math.max(chartArea.top + 2, Math.min(chartArea.bottom - h - 2, y - h / 2 + yOffset));

        ctx.fillStyle = 'rgba(14,14,20,0.92)';
        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(tx, ty, w, h, 3);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = color;
        ctx.textBaseline = 'middle';
        ctx.fillText(text, tx + 5, ty + h / 2 + 0.5);
      }

      valueTag(lat, 'LAT', '#6dd9b0', -8);
      valueTag(head, 'HEAD', '#e8a445', 10);

      ctx.fillStyle = 'rgba(255,255,255,0.38)';
      ctx.font = `9px ${monoFont}`;
      ctx.textAlign = 'center';
      ctx.fillText(_anaLabels[idx] ?? '', x, chartArea.bottom + 13);
      ctx.restore();
    }
  };

  _anaChart = new Chart(canvas, {
    type: 'line',
    data: {
      labels: _anaLabels,
      datasets: [
        {
          label: 'lateral_error',
          data: anaBuf.lat,
          borderColor: '#6dd9b0',
          backgroundColor: 'rgba(109,217,176,0.08)',
          borderWidth: 1.4,
          pointRadius: 0,
          tension: 0.35,
          fill: true,
          yAxisID: 'y'
        },
        {
          label: 'heading_error',
          data: anaBuf.head,
          borderColor: '#e8a445',
          borderWidth: 1.4,
          pointRadius: 0,
          tension: 0.35,
          fill: false,
          yAxisID: 'y'
        },
        // 임계선 (annotation plugin 없이 dataset으로 처리)
        { label: '+30°', data: [], borderColor: 'rgba(236,89,89,0.45)', borderWidth: 1, borderDash: [5,4], pointRadius: 0, tension: 0, fill: false, yAxisID: 'y' },
        { label: '-30°', data: [], borderColor: 'rgba(236,89,89,0.45)', borderWidth: 1, borderDash: [5,4], pointRadius: 0, tension: 0, fill: false, yAxisID: 'y' },
        { label: '+12°', data: [], borderColor: 'rgba(232,164,69,0.45)', borderWidth: 1, borderDash: [3,3], pointRadius: 0, tension: 0, fill: false, yAxisID: 'y' },
        { label: '-12°', data: [], borderColor: 'rgba(232,164,69,0.45)', borderWidth: 1, borderDash: [3,3], pointRadius: 0, tension: 0, fill: false, yAxisID: 'y' }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: 'index', intersect: false },
      plugins: {
        legend: { display: false },
        tooltip: {
          enabled: true,
          backgroundColor: 'rgba(14,14,20,0.95)',
          borderColor: 'rgba(180,185,200,0.18)',
          borderWidth: 1,
          titleFont: { family: monoFont, size: 10 },
          bodyFont:  { family: monoFont, size: 11 },
          titleColor: 'rgba(255,255,255,0.5)',
          bodyColor:  'rgba(255,255,255,0.85)',
          padding: 8,
          filter: (item) => item.datasetIndex < 2
        }
      },
      scales: {
        x: {
          ticks: { color: tickColor, font: { family: monoFont, size: 9 }, maxTicksLimit: 6, maxRotation: 0, autoSkip: true },
          grid: { color: gridColor, drawTicks: false },
          border: { display: false }
        },
        y: {
          min: -80, max: 80,
          ticks: { color: tickColor, font: { family: monoFont, size: 9 }, stepSize: 40 },
          grid: { color: gridColor, drawTicks: false },
          border: { display: false }
        }
      }
    },
    plugins: [crosshairPlugin]
  });

  const tag = document.getElementById('analytics-tag');
  if (tag) tag.textContent = 'LIVE · 2CH';
}

function pushAnalytics(lat, head) {
  // Chart 미초기화 시 lazy init
  if (!_anaChart) initAnalyticsChart();

  const now = new Date();
  const label = `${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;

  _anaLabels.push(label);
  anaBuf.lat.push(parseFloat(Number(lat).toFixed(1)));
  anaBuf.head.push(parseFloat(Number(head).toFixed(1)));

  if (_anaLabels.length > ANA_MAX_PTS) {
    _anaLabels.shift();
    anaBuf.lat.shift();
    anaBuf.head.shift();
  }

  // 임계선 동기화
  if (_anaChart) {
    const len = _anaLabels.length;
    _anaChart.data.datasets[2].data = Array(len).fill(30);
    _anaChart.data.datasets[3].data = Array(len).fill(-30);
    _anaChart.data.datasets[4].data = Array(len).fill(12);
    _anaChart.data.datasets[5].data = Array(len).fill(-12);
    _anaChart.update('none');
  }

  // 메트릭 수치 + 색상 경고
  const lvLat  = document.getElementById('lv-lat');
  const lvHead = document.getElementById('lv-head');
  if (lvLat)  lvLat.textContent  = Number(lat).toFixed(1);
  if (lvHead) lvHead.textContent = Number(head).toFixed(1);

  if (lvHead) {
    const ah = Math.abs(Number(head));
    lvHead.style.color = ah > 30 ? '#ec5959' : '#e8a445';
  }
}

function updateAnalyticsCmdBadge(cmd) {
  const badge = document.getElementById('ana-cmd-badge');
  if (!badge) return;
  const cmdMap = { '0': 'CMD: DUAL', '10': 'CMD: L_ONLY', '15': 'CMD: R_ONLY', '-1': 'CMD: LOST' };
  badge.dataset.cmd = String(cmd);
  badge.textContent = cmdMap[String(cmd)] ?? `CMD: ${cmd}`;
}

const missionStart = Date.now();

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function appendLog(message, cls = 'info') {
  const list = document.getElementById('perception-log-list');
  if (!list) return;
  const div = document.createElement('div');
  div.className = `log-line ${cls}`;
  div.textContent = message;
  list.prepend(div);
  while (list.children.length > 60) list.removeChild(list.lastChild);
  setText('log-count', String(list.children.length));
}

/* ---------- Mission Time ---------- */
function tickMissionTime() {
  const elapsed = Math.floor((Date.now() - missionStart) / 1000);
  const h = String(Math.floor(elapsed / 3600)).padStart(2, '0');
  const m = String(Math.floor((elapsed % 3600) / 60)).padStart(2, '0');
  const s = String(elapsed % 60).padStart(2, '0');
  setText('mission-time', `T+ ${h}:${m}:${s}`);
}

/* ---------- System Status 자동 판정 ---------- */
function evaluateSystemStatus() {
  const pip = document.getElementById('status-pip');
  const text = document.getElementById('system-status');
  if (!pip || !text) return;

  let level = 'NOMINAL';
  let cls = '';

  if (!state.wifi || state.rssi < -85 || state.maxLoopDt > 80) {
    level = 'CRITICAL'; cls = 'crit';
  } else if (state.rssi < -70 || state.maxLoopDt > 35 || state.loopRate < 30) {
    level = 'WARN'; cls = 'warn';
  }

  text.textContent = level;
  pip.className = 'pip ' + cls;
}

/* ---------- Metric bar 자동 갱신 ---------- */
function updateMetricBars() {
  // 새 HLTH_002는 배터리/전압 대신 ESP32 상태값을 표시한다.
  const rssiPct = Math.max(0, Math.min(100, ((state.rssi + 100) / 70) * 100)); // -100~-30 dBm
  const loopPct = Math.max(0, Math.min(100, (state.loopRate / 300) * 100));
  const heapPct = Math.max(0, Math.min(100, (state.heapKB / 220) * 100));

  const map = {
    'metric-rssi': rssiPct,
    'metric-wifi': state.wifi ? 100 : 0,
    'metric-loop': loopPct,
    'metric-heap': heapPct,

    // 구형 HTML 호환용
    'metric-battery': state.battery,
    'metric-cpu': state.cpu,
    'metric-memory': state.memory,
    'metric-voltage': state.voltage > 0 ? (state.voltage / 60) * 100 : 0
  };

  Object.keys(map).forEach((id) => {
    const fill = document.querySelector(`.met-bar-fill[data-bind="${id}"]`);
    if (!fill) return;
    const v = Math.max(0, Math.min(100, Number(map[id]) || 0));
    fill.style.width = `${v}%`;

    if (id === 'metric-rssi') {
      fill.style.background = state.rssi < -80
        ? 'linear-gradient(90deg,#ff5b5b,#ff8b8b)'
        : state.rssi < -65
          ? 'linear-gradient(90deg,#ffb547,#ffd47a)'
          : 'linear-gradient(90deg,#4ab48a,#7df9c0)';
    } else if (id === 'metric-wifi') {
      fill.style.background = state.wifi
        ? 'linear-gradient(90deg,#4ab48a,#7df9c0)'
        : 'linear-gradient(90deg,#ff5b5b,#ff8b8b)';
    } else if (id === 'metric-loop' || id === 'metric-heap') {
      fill.style.background = 'linear-gradient(90deg,#4ab48a,#7df9c0)';
    } else if (id === 'metric-cpu' || id === 'metric-memory') {
      if (v > 80) fill.style.background = 'linear-gradient(90deg,#ff5b5b,#ff8b8b)';
      else if (v > 60) fill.style.background = 'linear-gradient(90deg,#ffb547,#ffd47a)';
      else fill.style.background = 'linear-gradient(90deg,#4ab48a,#7df9c0)';
    } else if (id === 'metric-battery') {
      if (v < 20) fill.style.background = 'linear-gradient(90deg,#ff5b5b,#ff8b8b)';
      else if (v < 40) fill.style.background = 'linear-gradient(90deg,#ffb547,#ffd47a)';
      else fill.style.background = 'linear-gradient(90deg,#4ab48a,#7df9c0)';
    }
  });
}

function updateMetrics(data) {
  if (typeof data.encoder !== 'undefined') state.encoder = data.encoder;
  if (typeof data.safety !== 'undefined') state.safety = data.safety;
  if (typeof data.battery !== 'undefined') state.battery = data.battery;
  if (typeof data.voltage !== 'undefined') state.voltage = data.voltage;
  if (typeof data.cpu !== 'undefined') state.cpu = data.cpu;
  if (typeof data.memory !== 'undefined') state.memory = data.memory;
  if (typeof data.leftSpeed !== 'undefined') state.leftSpeed = data.leftSpeed;
  if (typeof data.rightSpeed !== 'undefined') state.rightSpeed = data.rightSpeed;
  if (typeof data.distance !== 'undefined') state.distance = data.distance;
  if (typeof data.heading !== 'undefined') state.heading = data.heading;
  if (typeof data.heading_error !== 'undefined') state.heading_error = data.heading_error;
  if (typeof data.lateral_error !== 'undefined') state.lateral_error = data.lateral_error;
  if (typeof data.lookahead_error !== 'undefined') state.lookahead_error = data.lookahead_error;
  if (typeof data.pwm_l !== 'undefined') state.pwm_l = data.pwm_l;
  if (typeof data.pwm_r !== 'undefined') state.pwm_r = data.pwm_r;
  if (typeof data.gyro_x !== 'undefined') state.gyro_x = data.gyro_x;
  if (typeof data.transform !== 'undefined') state.transform = { ...state.transform, ...data.transform };

  if (data.esp) {
    state.rssi = Number(data.esp.rssi ?? state.rssi);
    state.wifi = Number(data.esp.wifi ?? state.wifi);
    state.wifiCh = data.esp.ch ?? state.wifiCh;
    state.loopRate = Number(data.esp.loop_rate ?? state.loopRate);
    state.loopDt = Number(data.esp.loop_dt ?? state.loopDt);
    state.maxLoopDt = Number(data.esp.max_dt ?? state.maxLoopDt);
    state.heapKB = Number(data.esp.free_heap ?? 0) / 1024;
    state.minHeapKB = Number(data.esp.min_heap ?? 0) / 1024;
    state.espIp = data.esp.ip ?? state.espIp;
  }

  // 구형 ID 호환
  setText('metric-encoder', String(state.encoder));
  setText('metric-safety', state.safety);
  setText('metric-battery', `${Number(state.battery).toFixed(1)}`);
  setText('metric-voltage', `${Number(state.voltage).toFixed(1)}`);
  setText('metric-cpu', `${Number(state.cpu).toFixed(1)}`);
  setText('metric-memory', `${Number(state.memory).toFixed(1)}`);

  // 새 HLTH_002
  setText('metric-rssi', Number.isFinite(state.rssi) ? String(Math.round(state.rssi)) : '—');
  setText('metric-wifi', state.wifi ? 'ON' : 'OFF');
  setText('metric-wifi-ch', `CH ${state.wifiCh ?? '—'}`);
  setText('metric-loop', Number(state.loopRate).toFixed(1));
  setText('metric-heap', Number(state.heapKB).toFixed(1));

  setText('left-speed', `${Number(state.leftSpeed).toFixed(2)}`);
  setText('right-speed', `${Number(state.rightSpeed).toFixed(2)}`);

  if (typeof reflectDriveDirection === 'function') {
    reflectDriveDirection(Number(state.leftSpeed) || 0, Number(state.rightSpeed) || 0);
  }

  setText('distance-value', `${Number(state.distance).toFixed(2)}`);
  setText('robot-heading', `${Number(state.heading).toFixed(1)}`);

  const headingErrEl = document.getElementById('heading-error-val');
  if (headingErrEl) {
    headingErrEl.innerHTML = `${Number(state.heading_error).toFixed(1)}<span style="font-size:9px; color:var(--tx-3); margin-left:2px;">deg</span>`;
  }

  updateMatrixWidget();

  // 기존 호환: motor-bars (있으면 갱신, 없어도 무해)
  const leftBar = document.getElementById('bar-left');
  const rightBar = document.getElementById('bar-right');
  if (leftBar) leftBar.style.width = `${Math.min(Math.abs(state.leftSpeed) * 60, 100)}%`;
  if (rightBar) rightBar.style.width = `${Math.min(Math.abs(state.rightSpeed) * 60, 100)}%`;

  updateMetricBars();
  evaluateSystemStatus();
  integratePosition();
}

function updateMatrixWidget() {
  const t = state.transform || {};
  const theta = Number(t.theta_deg ?? state.heading ?? 0);
  const rad = theta * Math.PI / 180;
  const c = Math.cos(rad);
  const si = Math.sin(rad);

  setText('mat-s', Number(t.s ?? 4.27).toFixed(2));
  setText('mat-a', signed(c, 3));
  setText('mat-b', signed(si, 3));
  setText('mat-c', signed(-si, 3));
  setText('mat-d', signed(c, 3));
  setText('mat-cx', Number(t.cx ?? 0).toFixed(0));
  setText('mat-cy', Number(t.cy ?? 0).toFixed(0));
  setText('mat-theta', Number(theta).toFixed(2));
  setText('mat-xc', Number(t.xc ?? 0).toFixed(2));
  setText('mat-yc', Number(t.yc ?? 0).toFixed(2));
}

function signed(v, digits = 3) {
  const n = Number(v) || 0;
  return `${n >= 0 ? '+' : ''}${n.toFixed(digits)}`;
}

async function fetchRobotStats() {
  try {
    const res = await fetch('/api/robot_stats');
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();

    updateMetrics({
      encoder: data.pulse ?? 0,
      safety: data.obstacle ?? '안전',
      leftSpeed: data.L_speed ?? data.speed ?? 0,
      rightSpeed: data.R_speed ?? data.speed ?? 0,
      distance: data.distance ?? state.distance,
      heading: data.heading ?? state.heading,
      heading_error: data.heading_error ?? 0,
      lateral_error: data.lateral_error ?? 0,
      lookahead_error: data.lookahead_error ?? 0,
      pwm_l: data.pwm_l ?? 0,
      pwm_r: data.pwm_r ?? 0,
      gyro_x: data.gyro_x ?? 0,
      esp: data.esp,
      transform: data.transform
    });

    pushAnalytics(data.lateral_error ?? 0, data.heading_error ?? 0);

    if (data.search_cmd !== undefined) {
      updateAnalyticsCmdBadge(data.search_cmd);
    }
  } catch (err) {
    // Flask 미연결 시 mock 데이터로 대체 가능 — 콘솔 노이즈 줄임
  }
}

/* ---------- 위치 적분 (heading + speed → x, y) ---------- */
function integratePosition() {
  const now = Date.now();

  if (state.prevStatTime === null) {
    state.prevStatTime = now;
    return;
  }

  const dt = (now - state.prevStatTime) / 1000;
  state.prevStatTime = now;

  if (dt > 2 || dt <= 0) return;

  const speed = (Number(state.leftSpeed) + Number(state.rightSpeed)) / 2;

  // 노이즈 데드존 — 0.05 m/s 이하면 정지로 간주
  if (Math.abs(speed) < 0.05) return;

  const headingRad = state.heading * Math.PI / 180;

  state.robotX += speed * Math.cos(headingRad) * dt;
  state.robotY += speed * Math.sin(headingRad) * dt;

  const last = state.trajectory[state.trajectory.length - 1];
  if (!last || Math.hypot(state.robotX - last.x, state.robotY - last.y) > 0.005) {
    state.trajectory.push({ x: state.robotX, y: state.robotY });
    if (state.trajectory.length > 400) state.trajectory.shift();
  }
}

function drawTelemetry() {
  const canvas = document.getElementById('telemetry-canvas');
  if (!canvas) return;

  const rect = canvas.getBoundingClientRect();
  if (canvas.width !== rect.width || canvas.height !== rect.height) {
    canvas.width = rect.width;
    canvas.height = rect.height;
  }

  const ctx = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  const cx = W / 2;
  const cy = H / 2;
  const scale = 50;

  ctx.fillStyle = '#070b15';
  ctx.fillRect(0, 0, W, H);

  // follow 모드 — world origin 화면 위치
  const originPx = cx - state.robotX * scale;
  const originPy = cy + state.robotY * scale;

  // 미세 격자 (50cm)
  ctx.strokeStyle = 'rgba(125, 249, 192, 0.05)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  const minor = scale / 2;
  for (let x = ((originPx % minor) + minor) % minor; x < W; x += minor) { ctx.moveTo(x, 0); ctx.lineTo(x, H); }
  for (let y = ((originPy % minor) + minor) % minor; y < H; y += minor) { ctx.moveTo(0, y); ctx.lineTo(W, y); }
  ctx.stroke();

  // 굵은 격자 (1m)
  ctx.strokeStyle = 'rgba(125, 249, 192, 0.12)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = ((originPx % scale) + scale) % scale; x < W; x += scale) { ctx.moveTo(x, 0); ctx.lineTo(x, H); }
  for (let y = ((originPy % scale) + scale) % scale; y < H; y += scale) { ctx.moveTo(0, y); ctx.lineTo(W, y); }
  ctx.stroke();

  // 화살표 헬퍼
  function drawArrow(x1, y1, x2, y2, color, width = 2) {
    const angle = Math.atan2(y2 - y1, x2 - x1);
    ctx.strokeStyle = color; ctx.fillStyle = color;
    ctx.lineWidth = width;
    ctx.shadowColor = color; ctx.shadowBlur = 6;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2 - 7 * Math.cos(angle - 0.4), y2 - 7 * Math.sin(angle - 0.4));
    ctx.lineTo(x2 - 7 * Math.cos(angle + 0.4), y2 - 7 * Math.sin(angle + 0.4));
    ctx.closePath(); ctx.fill();
    ctx.shadowBlur = 0;
  }

  // ── 월드 좌표계 axes ──
  const axisLen = scale * 0.7;
  drawArrow(originPx, originPy, originPx + axisLen, originPy, '#ff5b5b');
  ctx.fillStyle = '#ff5b5b'; ctx.font = 'bold 9px "JetBrains Mono", monospace';
  ctx.fillText('X', originPx + axisLen + 4, originPy + 4);

  drawArrow(originPx, originPy, originPx, originPy - axisLen, '#7df9c0');
  ctx.fillStyle = '#7df9c0'; ctx.font = 'bold 9px "JetBrains Mono", monospace';
  ctx.fillText('Y', originPx + 3, originPy - axisLen - 3);

  // 월드 원점
  ctx.fillStyle = '#fff';
  ctx.shadowColor = 'rgba(255,255,255,0.6)'; ctx.shadowBlur = 6;
  ctx.beginPath(); ctx.arc(originPx, originPy, 3, 0, Math.PI * 2); ctx.fill();
  ctx.shadowBlur = 0;
  ctx.fillStyle = 'rgba(255,255,255,0.6)';
  ctx.font = '9px "JetBrains Mono", monospace';
  ctx.fillText('W', originPx + 5, originPy - 5);

  // ── 궤적 ──
  if (state.trajectory.length > 1) {
    ctx.strokeStyle = 'rgba(125, 249, 192, 0.7)';
    ctx.lineWidth = 1.5;
    ctx.shadowColor = 'rgba(125, 249, 192, 0.5)'; ctx.shadowBlur = 4;
    ctx.beginPath();
    for (let i = 0; i < state.trajectory.length; i++) {
      const p = state.trajectory[i];
      const px = cx + (p.x - state.robotX) * scale;
      const py = cy - (p.y - state.robotY) * scale;
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.stroke(); ctx.shadowBlur = 0;
  }

  // ── RC카 차량 형태 (항상 중앙) ──
  const hRad = state.heading * Math.PI / 180;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(hRad);

  // 차체 (직사각형) — 앞쪽이 오른쪽(+x)
  ctx.fillStyle = '#c87c10';
  ctx.shadowColor = 'rgba(255, 181, 71, 0.6)';
  ctx.shadowBlur = 10;
  ctx.beginPath();
  ctx.roundRect(-10, -6, 22, 12, 2);
  ctx.fill();
  ctx.shadowBlur = 0;

  // 차체 테두리
  ctx.strokeStyle = '#ffb547';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(-10, -6, 22, 12, 2);
  ctx.stroke();

  // 헤드라이트 (앞쪽 — 노란 흰색)
  ctx.fillStyle = '#fffde0';
  ctx.shadowColor = 'rgba(255,253,224,0.9)';
  ctx.shadowBlur = 6;
  ctx.beginPath(); ctx.arc(11, -4, 2, 0, Math.PI * 2); ctx.fill();
  ctx.beginPath(); ctx.arc(11,  4, 2, 0, Math.PI * 2); ctx.fill();
  ctx.shadowBlur = 0;

  // 테일라이트 (뒤쪽 — 빨강)
  ctx.fillStyle = '#ff3333';
  ctx.shadowColor = 'rgba(255,51,51,0.8)';
  ctx.shadowBlur = 5;
  ctx.beginPath(); ctx.arc(-9, -4, 1.5, 0, Math.PI * 2); ctx.fill();
  ctx.beginPath(); ctx.arc(-9,  4, 1.5, 0, Math.PI * 2); ctx.fill();
  ctx.shadowBlur = 0;

  // 앞방향 표시 화살표 (옅게)
  ctx.strokeStyle = 'rgba(255,181,71,0.5)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(12, 0); ctx.lineTo(20, 0);
  ctx.stroke();

  ctx.restore();

  // ── 로봇 좌표계 axes ──
  const robLen = 28;
  drawArrow(cx, cy, cx + Math.cos(hRad) * robLen, cy + Math.sin(hRad) * robLen, '#ff8888', 1.5);
  const ryRad = hRad + Math.PI / 2;
  drawArrow(cx, cy, cx + Math.cos(ryRad) * robLen, cy + Math.sin(ryRad) * robLen, '#88ffcc', 1.5);

  // F(앞) / B(뒤) 라벨
  ctx.font = 'bold 8px "JetBrains Mono", monospace';
  ctx.fillStyle = 'rgba(255, 181, 71, 0.85)';
  ctx.textAlign = 'center';
  const fLx = cx + Math.cos(hRad) * 34;
  const fLy = cy + Math.sin(hRad) * 34;
  ctx.fillText('F', fLx, fLy + 3);
  ctx.fillStyle = 'rgba(255, 100, 100, 0.7)';
  const bLx = cx - Math.cos(hRad) * 18;
  const bLy = cy - Math.sin(hRad) * 18;
  ctx.fillText('B', bLx, bLy + 3);
  ctx.textAlign = 'left';

  // ── 카메라 좌표계 (좌상단 고정) ──
  const camX = 42, camY = 36, camLen = 16;
  ctx.fillStyle = 'rgba(7, 11, 21, 0.8)';
  ctx.fillRect(camX - camLen - 8, camY - 20, (camLen + 8) * 2, camLen * 2 + 30);
  ctx.strokeStyle = 'rgba(92, 200, 255, 0.3)'; ctx.lineWidth = 1;
  ctx.strokeRect(camX - camLen - 8, camY - 20, (camLen + 8) * 2, camLen * 2 + 30);

  ctx.fillStyle = '#5cc8ff'; ctx.font = '8px "JetBrains Mono", monospace';
  ctx.textAlign = 'center'; ctx.fillText('CAM', camX, camY - 8); ctx.textAlign = 'left';

  // CAM x (빨강)
  ctx.strokeStyle = '#ff5b5b'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(camX, camY); ctx.lineTo(camX + camLen, camY); ctx.stroke();
  ctx.fillStyle = '#ff5b5b'; ctx.fillText('x', camX + camLen + 2, camY + 4);

  // CAM y (초록, 아래 — 이미지 좌표계)
  ctx.strokeStyle = '#7df9c0'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(camX, camY); ctx.lineTo(camX, camY + camLen); ctx.stroke();
  ctx.fillStyle = '#7df9c0'; ctx.fillText('y', camX + 3, camY + camLen + 10);

  // CAM z (파랑, 원 — 위→아래 방향)
  ctx.strokeStyle = '#5cc8ff'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(camX, camY, 5, 0, Math.PI * 2); ctx.stroke();
  ctx.fillStyle = '#5cc8ff';
  ctx.beginPath(); ctx.arc(camX, camY, 1.5, 0, Math.PI * 2); ctx.fill();
  ctx.fillText('z', camX + 7, camY - 3);

  // ── 스케일 바 ──
  const sbLen = scale, sbX = 8, sbY = H - 42;
  ctx.strokeStyle = 'rgba(125, 249, 192, 0.6)'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(sbX, sbY); ctx.lineTo(sbX + sbLen, sbY);
  ctx.moveTo(sbX, sbY - 3); ctx.lineTo(sbX, sbY + 3);
  ctx.moveTo(sbX + sbLen, sbY - 3); ctx.lineTo(sbX + sbLen, sbY + 3);
  ctx.stroke();
  ctx.fillStyle = 'rgba(125, 249, 192, 0.7)';
  ctx.font = '8px "JetBrains Mono", monospace';
  ctx.textAlign = 'center';
  ctx.fillText('1 m', sbX + sbLen / 2, sbY - 5);
  ctx.textAlign = 'left';

  // ── 좌표 오버레이 ──
  ctx.font = '10px "JetBrains Mono", monospace';
  ctx.textBaseline = 'bottom';
  ctx.fillStyle = '#7df9c0';
  ctx.fillText(`X ${state.robotX.toFixed(2)}m`, 8, H - 22);
  ctx.fillText(`Y ${state.robotY.toFixed(2)}m`, 8, H - 8);
  ctx.textAlign = 'right';
  ctx.fillStyle = '#5cc8ff';
  ctx.fillText(`H ${state.heading.toFixed(1)}°`, W - 8, H - 8);
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';

  requestAnimationFrame(drawTelemetry);
}

/* ---------- RESET 버튼 ---------- */
function bindMapReset() {
  const btn = document.getElementById('map-reset');
  if (!btn) return;
  btn.addEventListener('click', () => {
    state.robotX = 0;
    state.robotY = 0;
    state.prevStatTime = null;
    state.trajectory = [];
    appendLog('[MAP] trajectory reset', 'info');
  });
}

async function sendCommand(command) {
  state.lastCommand = command;
  setText('last-command', command);
  appendLog(`[TX] remote command = ${command}`, 'tx');

  try {
    await fetch('/update_position', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x: 0, y: 0, direction: command })
    });
  } catch (err) {
    appendLog(`[ERR] command send failed: ${err.message}`, 'error');
  }
}

// joystick fade controller — supports two layers:
//   1) USER input (lit-strong) — keyboard / mouse press
//   2) DRIVE direction (lit) — auto reflection of leftSpeed/rightSpeed
const _joyTimers = {};      // dir -> timeout id (user fade out)
const _joyDriveActive = {}; // dir -> bool (driven by state)
function _joyAllBtns() { return document.querySelectorAll('.joy-btn'); }
function _joyBtn(dir) { return document.querySelector(`.joy-btn[data-command="${dir}"]`); }

function joyPress(dir) {
  const btn = _joyBtn(dir);
  if (!btn) return;
  btn.classList.add('lit-strong');
}
function joyRelease(dir) {
  const btn = _joyBtn(dir);
  if (!btn) return;
  if (_joyTimers[dir]) clearTimeout(_joyTimers[dir]);
  _joyTimers[dir] = setTimeout(() => {
    btn.classList.remove('lit-strong');
  }, 320); // soft trailing glow
}

// reflect actual drive direction onto joystick (called from drive-state updater)
function reflectDriveDirection(left, right) {
  // dead zone
  const eps = 0.06;
  let dir = null;
  const aL = Math.abs(left), aR = Math.abs(right);
  if (aL < eps && aR < eps) {
    dir = 'S';
  } else if (left > 0 && right > 0) {
    // forward (or fwd-curve)
    if (right - left > 0.25) dir = 'FL';
    else if (left - right > 0.25) dir = 'FR';
    else dir = 'F';
  } else if (left < 0 && right < 0) {
    if (Math.abs(right) - Math.abs(left) > 0.25) dir = 'BL';
    else if (Math.abs(left) - Math.abs(right) > 0.25) dir = 'BR';
    else dir = 'B';
  } else if (left < 0 && right > 0) {
    dir = 'L'; // pivot left
  } else if (left > 0 && right < 0) {
    dir = 'R'; // pivot right
  }

  // clear all drive-active flags
  _joyAllBtns().forEach((b) => {
    const d = b.dataset.command;
    if (_joyDriveActive[d] && d !== dir) {
      _joyDriveActive[d] = false;
      // only remove .lit if user isn't currently strong-pressing
      if (!b.classList.contains('lit-strong')) b.classList.remove('lit');
    }
  });
  if (dir) {
    const btn = _joyBtn(dir);
    if (btn) {
      _joyDriveActive[dir] = true;
      btn.classList.add('lit');
    }
  }
}

function bindRemoteButtons() {
  document.querySelectorAll('.joy-btn').forEach((btn) => {
    btn.addEventListener('mousedown', () => joyPress(btn.dataset.command));
    btn.addEventListener('mouseup', () => joyRelease(btn.dataset.command));
    btn.addEventListener('mouseleave', () => joyRelease(btn.dataset.command));
    btn.addEventListener('click', () => sendCommand(btn.dataset.command));
  });
}

function bindKeyboardControl() {
  const keyMap = {
    ArrowUp: 'F',
    ArrowDown: 'B',
    ArrowLeft: 'L',
    ArrowRight: 'R',
    ' ': 'S'
  };

  window.addEventListener('keydown', (e) => {
    if (!(e.key in keyMap)) return;
    e.preventDefault();

    const toggle = document.getElementById('remote-toggle');
    if (toggle && !toggle.checked) return;

    const command = keyMap[e.key];
    joyPress(command);

    if (state.lastCommand !== command) sendCommand(command);
  });

  window.addEventListener('keyup', (e) => {
    if (!(e.key in keyMap)) return;
    const command = keyMap[e.key];
    joyRelease(command);
  });
}

async function fetchWeather() {
  try {
    const res = await fetch('/api/weather');
    if (!res.ok) throw new Error(`HTTP ${res.status}`);

    const data = await res.json();

    setText('weather-summary', data.summary ?? '날씨 정보');
    setText('weather-temp', data.temp ?? '--°C');
    setText('weather-humidity', `습도 ${data.humidity ?? '--%'}`);
    setText('weather-icon', getWeatherIcon(data.summary));
  } catch (err) {
    setText('weather-summary', '날씨 오류');
    setText('weather-temp', '--°C');
    setText('weather-humidity', '습도 --%');
    setText('weather-icon', '·');
  }
}

function watchVideoStream() {
  const img = document.getElementById('perception-stream');
  if (!img) return;

  img.addEventListener('load', () => {
    setText('overlay-status', 'OPENCV_DEBUG_OK');
  });

  img.addEventListener('error', () => {
    setText('overlay-status', 'STREAM_OFFLINE');
  });
}

function getWeatherIcon(summary) {
  if (!summary) return '·';

  if (summary.includes('맑음')) return '☀';
  if (summary.includes('대체로 맑음')) return '☼';
  if (summary.includes('구름')) return '☁';
  if (summary.includes('흐림')) return '☁';
  if (summary.includes('비')) return '☂';
  if (summary.includes('소나기')) return '☂';
  if (summary.includes('눈')) return '❄';
  if (summary.includes('안개')) return '≈';
  if (summary.includes('뇌우')) return '⚡';

  return '·';
}

async function fetchSystemLogs() {
  try {
    const res = await fetch('/api/system_logs');
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const logs = await res.json();

    const logListContainer = document.getElementById('perception-log-list');
    if (!logListContainer) return;

    if (logs.length === 0) {
      logListContainer.innerHTML = '<div class="log-line info">// 수신된 시스템 로그 없음</div>';
      setText('log-count', '0');
      return;
    }

    logListContainer.innerHTML = '';

    [...logs].reverse().forEach(log => {
      const div = document.createElement('div');
      div.className = `log-line ${log.type || 'info'}`;
      div.textContent = `[${log.time}] ${log.msg}`;
      logListContainer.appendChild(div);
    });

    setText('log-count', String(logs.length));
  } catch (err) {
    // Flask 미연결 시 정상 — 콘솔에 노이즈 안 띄움
  }
}

async function fetchNews() {
  try {
    const res = await fetch('/api/news');
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const items = await res.json();

    const track = document.getElementById('ticker-track');
    if (!track) return;

    if (!items || items.length === 0) {
      track.innerHTML = '<span class="ticker-item">// 뉴스 데이터 없음</span>';
      return;
    }

    // 무한 스크롤을 위해 두 번 복제
    const buildItems = () => items.map(it => `
      <span class="ticker-item">
        <span class="ticker-src">${it.source}</span>
        <a href="${it.link}" target="_blank" rel="noopener">${it.title}</a>
        <span class="ticker-sep">◇</span>
      </span>
    `).join('');

    track.innerHTML = buildItems() + buildItems();
  } catch (err) {
    // 정상 — Flask 없으면 무시
  }
}

document.addEventListener('DOMContentLoaded', () => {
  bindRemoteButtons();
  bindKeyboardControl();
  watchVideoStream();
  bindMapReset();
  initAnalyticsChart();
  updateMetrics(state);
  
  drawTelemetry();

  // Mission time 초기 + 1초 tick
  tickMissionTime();
  setInterval(tickMissionTime, 1000);

  // wall clock (sidebar quick-actions)
  function tickWallClock() {
    const d = new Date();
    const hours = d.getHours();
    const ampm = hours < 12 ? '오전' : '오후';
    const hh = String(hours).padStart(2, '0');
    const mm = String(d.getMinutes()).padStart(2, '0');
    const ss = String(d.getSeconds()).padStart(2, '0');
    setText('util-clock', `${hh}:${mm}:${ss}`);
    const ampmEl = document.getElementById('util-clock-ampm');
    if (ampmEl) ampmEl.textContent = ampm;
  }
  tickWallClock();
  setInterval(tickWallClock, 1000);

  fetchWeather();
  setInterval(fetchWeather, 600000);

  fetchRobotStats();
  setInterval(fetchRobotStats, 1500);

  fetchSystemLogs();
  setInterval(fetchSystemLogs, 2000);

  fetchNews();
  setInterval(fetchNews, 5 * 60 * 1000);  // 5분마다

  appendLog('[BOOT] mission_ctrl dashboard initialized', 'success');
  appendLog('[NET]  uplink established // ros2_bridge', 'info');

  // 추가 init
  initGauges();
  initMissionProgress();
  initTweaks();
  initQuickActions();
  applyGlowLevel(localStorage.getItem('mc_glow') || 'low');

  setInterval(updateGauges, 200);
  setInterval(tickMission, 3000);
});

/* ============================================
 * GAUGES — Speed + RPM (analog SVG)
 * ============================================ */
function initGauges() {
  const speedTicks = document.getElementById('gauge-speed-ticks');
  const rpmTicks = document.getElementById('gauge-rpm-ticks');
  if (!speedTicks || !rpmTicks) return;
  // 11 ticks across the half-arc (180° → 0°)
  for (let i = 0; i <= 10; i++) {
    const ang = Math.PI - (i / 10) * Math.PI;
    const cx = 50, cy = 60, r1 = 38, r2 = i % 5 === 0 ? 32 : 35;
    const x1 = cx + Math.cos(ang) * r1;
    const y1 = cy - Math.sin(ang) * r1;
    const x2 = cx + Math.cos(ang) * r2;
    const y2 = cy - Math.sin(ang) * r2;
    const mk = (parent, color, op) => {
      const ln = document.createElementNS('http://www.w3.org/2000/svg', 'line');
      ln.setAttribute('x1', x1); ln.setAttribute('y1', y1);
      ln.setAttribute('x2', x2); ln.setAttribute('y2', y2);
      ln.setAttribute('stroke', color);
      ln.setAttribute('stroke-width', i % 5 === 0 ? '1.2' : '0.6');
      ln.setAttribute('opacity', op);
      parent.appendChild(ln);
    };
    mk(speedTicks, '#6dd9b0', i % 5 === 0 ? 0.7 : 0.3);
    mk(rpmTicks, '#e8a445', i % 5 === 0 ? 0.7 : 0.3);
  }
}

function updateGauges() {
  const MAX_SPEED = 2.0;   // m/s 최대값 (필요시 조정)
  const MAX_DIST  = 50.0;  // m 기준 최대 거리 (바 100% 기준)

  // ── L_SPEED 게이지 ──
  const lSpeed = parseFloat(document.getElementById('left-speed')?.textContent) || 0;
  setGauge('gauge-speed', lSpeed, MAX_SPEED);

  // ── R_SPEED 게이지 ──
  const rSpeed = parseFloat(document.getElementById('right-speed')?.textContent) || 0;
  setGauge('gauge-rpm', rSpeed, MAX_SPEED);

  // ── 거리 진행 바 ──
  const dist = parseFloat(document.getElementById('distance-value')?.textContent) || 0;
  const distPct = Math.min((dist / MAX_DIST) * 100, 100).toFixed(1);
  const distBar = document.getElementById('dist-bar-fill');
  if (distBar) distBar.style.width = distPct + '%';

  // ── 헤딩 오차 컬러 변화 ──
  const errEl = document.getElementById('heading-error-val');
  if (errEl) {
    const errNum = state.heading_error || 0;
    const absErr = Math.abs(errNum);
    let color, shadow;
    if (absErr < 5) {
      color = 'var(--ph)';       // 초록: 오차 거의 없음
      shadow = '0 0 6px var(--ph-glow)';
    } else if (absErr < 20) {
      color = 'var(--am)';       // amber: 약간 오차
      shadow = '0 0 6px rgba(232,164,69,0.4)';
    } else {
      color = 'var(--rd)';       // red: 오차 큼
      shadow = '0 0 6px rgba(236,89,89,0.4)';
    }
    errEl.style.color = color;
    errEl.style.textShadow = shadow;
  }
}

function setGauge(prefix, value, max) {
  const arc = document.getElementById(prefix + '-arc');
  const needle = document.getElementById(prefix + '-needle');
  if (!arc || !needle) return;
  const pct = Math.max(0, Math.min(1, Math.abs(value) / max));
  // arc length ≈ π * 38 ≈ 119.4
  const arcLen = 119.4;
  arc.setAttribute('stroke-dasharray', arcLen);
  arc.setAttribute('stroke-dashoffset', arcLen * (1 - pct));
  // needle: 180° → 0°
  const ang = 180 - pct * 180;
  needle.setAttribute('transform', `rotate(${ang - 90} 50 60)`);
}

/* ============================================
 * MISSION PROGRESS
 * ============================================ */
let missionPct = 42;
function initMissionProgress() {
  // 클릭으로 단계 토글 가능
  document.querySelectorAll('.ms-step').forEach((el, idx) => {
    el.addEventListener('click', () => {
      el.classList.toggle('done');
      el.classList.remove('active');
      recalcMissionPct();
    });
  });
}
function tickMission() {
  // 살짝 자동 진행
  if (missionPct < 100) {
    missionPct = Math.min(100, missionPct + Math.random() * 1.4);
    setMissionPct(missionPct);
  }
}
function recalcMissionPct() {
  const all = document.querySelectorAll('.ms-step');
  const done = document.querySelectorAll('.ms-step.done').length;
  missionPct = (done / all.length) * 100;
  setMissionPct(missionPct);
}
function setMissionPct(v) {
  const fill = document.getElementById('mission-bar-fill');
  if (fill) fill.style.width = v + '%';
  setText('mission-pct', String(Math.round(v)));
}

/* ============================================
 * QUICK ACTIONS
 * ============================================ */
function initQuickActions() {
  const stop = document.getElementById('qa-estop');
  const home = document.getElementById('qa-home');
  if (stop) stop.addEventListener('click', () => {
    sendCommand('S');
    appendLog('[!!!] EMERGENCY STOP triggered // operator', 'error');
    flashStatus('crit', 'E-STOP');
  });
  if (home) home.addEventListener('click', () => {
    appendLog('[CMD] Return-to-home initiated', 'tx');
    flashStatus('warn', 'RTH');
  });
  window.addEventListener('keydown', (e) => {
    if (e.key === 'e' || e.key === 'E') stop && stop.click();
    if (e.key === 'h' || e.key === 'H') home && home.click();
  });
}
function flashStatus(cls, label) {
  const pip = document.getElementById('status-pip');
  const text = document.getElementById('system-status');
  if (!pip || !text) return;
  pip.className = 'pip ' + cls;
  text.textContent = label;
  setTimeout(() => evaluateSystemStatus(), 2500);
}

/* ============================================
 * TWEAKS PANEL
 * ============================================ */
function initTweaks() {
  const panel = document.getElementById('tw-panel');
  const open = document.getElementById('qa-tweaks');
  const close = document.getElementById('tw-close');
  if (!panel || !open || !close) return;

  // 초기 상태 복원
  const saved = JSON.parse(localStorage.getItem('mc_tweaks') || '{}');
  Object.entries(saved).forEach(([k, v]) => applyTweak(k, v));

  open.addEventListener('click', () => panel.classList.add('on'));
  close.addEventListener('click', () => panel.classList.remove('on'));

  panel.querySelectorAll('.tw-radio').forEach((grp) => {
    const key = grp.dataset.tw;
    grp.querySelectorAll('button').forEach((btn) => {
      btn.addEventListener('click', () => {
        grp.querySelectorAll('button').forEach(b => b.classList.remove('on'));
        btn.classList.add('on');
        applyTweak(key, btn.dataset.val);
        persistTweak(key, btn.dataset.val);
      });
    });
  });

  // mock toggle
  const mockBtn = document.getElementById('tw-mock');
  if (mockBtn) {
    mockBtn.addEventListener('click', () => {
      const on = !mockBtn.classList.contains('on');
      mockBtn.classList.toggle('on', on);
      mockBtn.textContent = on ? 'MOCK_DATA · ON' : 'MOCK_DATA · OFF';
      window.__mockMode = on;
      if (on) startMockMode(); else stopMockMode();
    });
  }
}
function persistTweak(k, v) {
  const saved = JSON.parse(localStorage.getItem('mc_tweaks') || '{}');
  saved[k] = v;
  localStorage.setItem('mc_tweaks', JSON.stringify(saved));
  // UI 동기화
  const grp = document.querySelector(`.tw-radio[data-tw="${k}"]`);
  if (grp) {
    grp.querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.val === v));
  }
}
function applyTweak(key, val) {
  if (key === 'analytics') applyAnalyticsVariant(val);
  else if (key === 'remote') applyRemoteVariant(val);
  else if (key === 'glow') applyGlowLevel(val);
}
function applyAnalyticsVariant(v) {
  const body = document.getElementById('analytics-body');
  const tag = document.getElementById('analytics-tag');
  if (!body) return;
  body.dataset.variant = v;

  // 현재 index.html은 Chart.js 캔버스를 사용한다. 구형 variant 요소가 없어도 에러가 나지 않게 처리.
  const graph = body.querySelector('.ph-graph');
  const note = body.querySelector('.ph-note');
  const legend = body.querySelector('.ph-legend');
  const table = document.getElementById('tel-table');
  const canvasWrap = body.querySelector('.ana-canvas-wrap');
  const line2 = document.getElementById('ana-line2');
  const line3 = document.getElementById('ana-line3');

  if (v === 'dense') {
    if (graph) graph.style.display = 'none';
    if (note) note.style.display = 'none';
    if (legend) legend.style.display = 'none';
    if (canvasWrap) canvasWrap.style.display = 'none';
    if (table) table.style.display = 'grid';
    if (tag) tag.textContent = 'LIVE_TABLE';
    return;
  }

  if (graph) graph.style.display = '';
  if (note) note.style.display = 'none';
  if (legend) legend.style.display = v === 'multiline' ? '' : 'none';
  if (table) table.style.display = 'none';
  if (canvasWrap) canvasWrap.style.display = '';
  if (line2) line2.style.display = v === 'multiline' ? '' : 'none';
  if (line3) line3.style.display = 'none';
  if (tag) tag.textContent = v === 'waveform' ? 'WAVEFORM · 2CH' : 'LIVE · 2CH';
}

function applyRemoteVariant(v) {
  const body = document.getElementById('remote-body');
  if (!body) return;
  body.dataset.variant = v;
  const slider = document.getElementById('remote-speed-slider');
  const motorBars = document.getElementById('remote-motor-bars');
  if (v === 'joystick') {
    if (slider) slider.style.display = 'none';
    if (motorBars) motorBars.style.display = 'none';
  } else if (v === 'dpad-pro') {
    if (slider) slider.style.display = '';
    if (motorBars) motorBars.style.display = 'none';
  } else if (v === 'gauge-pad') {
    if (slider) slider.style.display = 'none';
    if (motorBars) motorBars.style.display = 'grid';
  }
}
function applyGlowLevel(v) {
  const r = document.documentElement;
  if (v === 'off') {
    r.style.setProperty('--ph-glow', 'rgba(125, 249, 192, 0.0)');
  } else if (v === 'low') {
    r.style.setProperty('--ph-glow', 'rgba(125, 249, 192, 0.28)');
  } else if (v === 'high') {
    r.style.setProperty('--ph-glow', 'rgba(125, 249, 192, 0.55)');
  }
  localStorage.setItem('mc_glow', v);
  // sync radio UI
  const grp = document.querySelector('.tw-radio[data-tw="glow"]');
  if (grp) grp.querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.val === v));
}

// motor-bars 업데이트 hook (drive state 갱신 시)
const _origUpdateMetrics = updateMetrics;
updateMetrics = function (data) {
  _origUpdateMetrics(data);
  const lFill = document.getElementById('mb-l-fill');
  const rFill = document.getElementById('mb-r-fill');
  const lVal = document.getElementById('mb-l-val');
  const rVal = document.getElementById('mb-r-val');
  if (lFill) {
    const lp = Math.min(100, Math.abs(state.leftSpeed) * 60);
    lFill.style.width = lp + '%';
    lFill.style.left = state.leftSpeed >= 0 ? '50%' : (50 - lp / 2) + '%';
  }
  if (rFill) {
    const rp = Math.min(100, Math.abs(state.rightSpeed) * 60);
    rFill.style.width = rp + '%';
    rFill.style.left = state.rightSpeed >= 0 ? '50%' : (50 - rp / 2) + '%';
  }
  if (lVal) lVal.textContent = Number(state.leftSpeed).toFixed(2);
  if (rVal) rVal.textContent = Number(state.rightSpeed).toFixed(2);
};

// speed slider readout
document.addEventListener('DOMContentLoaded', () => {
  const inp = document.getElementById('rss-input');
  const out = document.getElementById('rss-val');
  if (inp && out) inp.addEventListener('input', () => { out.textContent = inp.value; });
});

/* ============================================
 * MOCK MODE — Flask 없이 미리보기용
 * fetch를 가로채서 mock 데이터 반환
 * ============================================ */
const _origFetch = window.fetch;
let _mockTick = 0;
let _mockTimer = null;

function startMockMode() {
  appendLog('[DEMO] mock mode enabled — frontend-only', 'system');
  window.fetch = mockFetch;
  // perception img placeholder
  const img = document.getElementById('perception-stream');
  if (img) img.src = mockPerceptionImg();
  // start light motion
  _mockTimer = setInterval(() => {
    _mockTick++;
    state.leftSpeed = 0.6 + Math.sin(_mockTick * 0.1) * 0.3;
    state.rightSpeed = 0.6 + Math.sin(_mockTick * 0.1 + 0.4) * 0.3;
    state.heading = (state.heading + 1.5) % 360;
    updateMetrics({});
    // analytics mock: lateral_error, heading_error
    const t = _mockTick * 0.12;
    const mockLat  = 14 * Math.sin(t * 0.35) + 7 * Math.sin(t * 0.9 + 1.2);
    const mockHead = 20 * Math.sin(t * 0.35 + 0.4) + 9 * Math.sin(t * 0.7);
    pushAnalytics(
      parseFloat(mockLat.toFixed(1)),
      parseFloat(mockHead.toFixed(1))
    );
    // search_cmd mock: 주기적으로 상태 변화
    const cmdCycle = _mockTick % 80;
    const mockCmd = cmdCycle < 6 ? -1 : cmdCycle < 14 ? 10 : cmdCycle < 20 ? 15 : 0;
    updateAnalyticsCmdBadge(mockCmd);
  }, 250);
}
function stopMockMode() {
  appendLog('[DEMO] mock mode disabled', 'system');
  window.fetch = _origFetch;
  if (_mockTimer) { clearInterval(_mockTimer); _mockTimer = null; }
}
async function mockFetch(url, opts) {
  const u = String(url);
  if (u.includes('/api/robot_stats')) {
    return jsonResp({
      pulse: 12000 + _mockTick,
      obstacle: '안전',
      L_speed: state.leftSpeed,
      R_speed: state.rightSpeed,
      speed: (state.leftSpeed + state.rightSpeed) / 2,
      distance: state.distance + 0.02,
      heading: state.heading,
      gyro_x: 1.5,
      lateral_error: 14 * Math.sin(_mockTick * 0.04),
      heading_error: 20 * Math.sin(_mockTick * 0.05),
      pwm_l: Math.round(state.leftSpeed * 255),
      pwm_r: Math.round(state.rightSpeed * 255),
      search_cmd: 0,
      esp: {
        loop_rate: 240 + Math.sin(_mockTick * 0.03) * 20,
        loop_dt: 4.1,
        max_dt: 18 + Math.sin(_mockTick * 0.07) * 4,
        wifi: 1,
        rssi: -48 + Math.sin(_mockTick * 0.04) * 5,
        ch: 2,
        ip: '192.168.0.15',
        free_heap: 182372,
        min_heap: 160000
      },
      transform: {
        theta_deg: state.heading,
        cx: 518,
        cy: 273,
        s: 4.27,
        xc: -121.31,
        yc: -63.93
      }
    });
  }
  if (u.includes('/api/weather')) {
    return jsonResp({ summary: '맑음', temp: '21°C', humidity: '54%' });
  }
  if (u.includes('/api/news')) {
    return jsonResp([
      { source: 'YONHAP', title: '국내 자율주행 시장 22% 성장', link: '#' },
      { source: 'ROS_NEWS', title: 'ROS2 Jazzy 보안 패치 릴리즈', link: '#' },
      { source: 'KCTV',   title: '도심 자율배송 실증 단계 진입', link: '#' }
    ]);
  }
  if (u.includes('/api/system_logs')) {
    return jsonResp([
      { time: '14:22:01', msg: 'IMU calibration ok', type: 'info' },
      { time: '14:22:14', msg: 'lidar scan rate 12Hz', type: 'ros' },
      { time: '14:22:33', msg: 'path planner: WP_03 reached', type: 'success' },
      { time: '14:22:48', msg: 'cpu spike — 78%', type: 'warn' }
    ]);
  }
  if (u.includes('/update_position')) return jsonResp({ ok: true });
  return _origFetch(url, opts);
}
function jsonResp(obj) {
  return Promise.resolve(new Response(JSON.stringify(obj), { status: 200, headers: { 'Content-Type': 'application/json' } }));
}
function mockPerceptionImg() {
  // 1x1 dark png + overlay handled by the existing video-stage
  return 'data:image/svg+xml;utf8,' + encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 400 300">
      <rect width="400" height="300" fill="#0a0f1a"/>
      <g stroke="rgba(125,217,176,0.3)" stroke-width="0.5" fill="none">
        ${Array.from({length:20}).map((_,i)=>`<line x1="0" y1="${i*15}" x2="400" y2="${i*15}"/>`).join('')}
        ${Array.from({length:27}).map((_,i)=>`<line x1="${i*15}" y1="0" x2="${i*15}" y2="300"/>`).join('')}
      </g>
      <rect x="160" y="120" width="80" height="60" fill="none" stroke="#6dd9b0" stroke-width="1.5"/>
      <text x="160" y="115" font-family="monospace" font-size="9" fill="#6dd9b0">car · 0.92</text>
      <text x="10" y="290" font-family="monospace" font-size="9" fill="#5cc8ff">/sim/perception · MOCK</text>
    </svg>`
  );
}