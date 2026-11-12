// Custom Trash Detection UI for ESP32-CAM OV3660
// Served directly as a plain HTML string (no gzip compression needed)

static const char TRASH_CAM_HTML[] = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <title>TrashCam</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      background: #0f0f0f;
      color: #f0f0f0;
      font-family: 'Segoe UI', system-ui, sans-serif;
      display: flex;
      flex-direction: column;
      height: 100dvh;
      overflow: hidden;
    }

    /* ── Header ── */
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 10px 16px;
      background: #1a1a1a;
      border-bottom: 1px solid #2a2a2a;
      flex-shrink: 0;
    }
    header h1 {
      font-size: 1.1rem;
      font-weight: 700;
      letter-spacing: 0.05em;
      color: #4ade80;
    }
    header h1 span { color: #f0f0f0; }

    #status-dot {
      width: 10px; height: 10px;
      border-radius: 50%;
      background: #ef4444;
      box-shadow: 0 0 6px #ef4444;
      transition: background 0.3s, box-shadow 0.3s;
    }
    #status-dot.live {
      background: #4ade80;
      box-shadow: 0 0 8px #4ade80;
    }

    /* ── Camera view ── */
    #camera-wrap {
      flex: 1;
      display: flex;
      align-items: center;
      justify-content: center;
      background: #000;
      position: relative;
      overflow: hidden;
    }

    #stream {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }

    /* ── Tracker canvas overlay ── */
    #overlay {
      position: absolute;
      inset: 0;
      pointer-events: none;
    }
    #tracker-canvas {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
    }

    /* ── Bottom controls ── */
    #controls {
      background: #1a1a1a;
      border-top: 1px solid #2a2a2a;
      padding: 12px 16px;
      flex-shrink: 0;
    }

    /* Resolution + Quality row */
    .ctrl-row {
      display: flex;
      gap: 10px;
      margin-bottom: 10px;
      align-items: center;
    }
    .ctrl-row label {
      font-size: 0.72rem;
      color: #888;
      white-space: nowrap;
    }
    select, input[type=range] {
      background: #262626;
      color: #f0f0f0;
      border: 1px solid #333;
      border-radius: 6px;
      padding: 5px 8px;
      font-size: 0.8rem;
      flex: 1;
    }
    select:focus { outline: none; border-color: #4ade80; }

    input[type=range] {
      padding: 0;
      height: 6px;
      accent-color: #4ade80;
      cursor: pointer;
    }

    /* Action buttons */
    .btn-row {
      display: flex;
      gap: 10px;
    }
    .btn {
      flex: 1;
      padding: 11px 8px;
      border: none;
      border-radius: 8px;
      font-size: 0.85rem;
      font-weight: 600;
      cursor: pointer;
      transition: opacity 0.15s, transform 0.1s;
    }
    .btn:active { transform: scale(0.96); opacity: 0.85; }

    #btn-stream  { background: #4ade80; color: #0f0f0f; }
    #btn-capture { background: #3b82f6; color: #fff; }
    #btn-flash   { background: #f59e0b; color: #0f0f0f; }

    #btn-stream.streaming  { background: #ef4444; color: #fff; }

    /* Flash toggle indicator */
    #btn-flash.on { background: #fde68a; }

    /* ── Collect mode button in header ── */
    #btn-collect-toggle {
      background: none;
      border: 1.5px solid #666;
      color: #aaa;
      border-radius: 6px;
      padding: 4px 10px;
      font-size: 0.72rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s;
      letter-spacing: 0.04em;
    }
    #btn-collect-toggle.active {
      border-color: #f59e0b;
      color: #f59e0b;
      box-shadow: 0 0 6px #f59e0b66;
    }

    /* ── Collect panel ── */
    #collect-panel {
      background: #141414;
      border-top: 1px solid #f59e0b44;
      padding: 10px 16px;
      flex-shrink: 0;
      display: none;
    }
    #collect-panel.show { display: block; }

    .collect-row {
      display: flex;
      gap: 8px;
      align-items: center;
      margin-bottom: 8px;
    }
    .collect-row label {
      font-size: 0.72rem;
      color: #888;
      white-space: nowrap;
    }

    /* Class pills */
    .class-pills {
      display: flex;
      gap: 6px;
      flex: 1;
    }
    .pill {
      flex: 1;
      padding: 6px 4px;
      border-radius: 6px;
      border: 2px solid transparent;
      font-size: 0.72rem;
      font-weight: 700;
      cursor: pointer;
      text-align: center;
      background: #222;
      color: #888;
      transition: all 0.15s;
      letter-spacing: 0.03em;
    }
    .pill.bio.selected      { background: #14532d; border-color: #4ade80; color: #4ade80; }
    .pill.non_bio.selected  { background: #450a0a; border-color: #ef4444; color: #ef4444; }
    .pill.recyclable.selected { background: #1e3a5f; border-color: #3b82f6; color: #3b82f6; }
    .pill:not(.selected):hover { background: #2a2a2a; color: #ccc; }

    /* Counters */
    .counters {
      display: flex;
      gap: 8px;
      margin-bottom: 8px;
    }
    .counter-box {
      flex: 1;
      background: #1a1a1a;
      border-radius: 6px;
      padding: 6px 8px;
      text-align: center;
      border: 1px solid #2a2a2a;
    }
    .counter-box .c-label {
      font-size: 0.62rem;
      color: #666;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .counter-box .c-num {
      font-size: 1.1rem;
      font-weight: 700;
      margin-top: 2px;
    }
    .counter-box.bio      .c-num { color: #4ade80; }
    .counter-box.non_bio  .c-num { color: #ef4444; }
    .counter-box.recyclable .c-num { color: #3b82f6; }

    /* Collect action row */
    .collect-actions {
      display: flex;
      gap: 8px;
      align-items: center;
    }
    #btn-snap-one {
      flex: 1;
      padding: 10px;
      background: #292929;
      border: 1.5px solid #444;
      color: #f0f0f0;
      border-radius: 8px;
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
    }
    #btn-auto-collect {
      flex: 1;
      padding: 10px;
      background: #f59e0b;
      border: none;
      color: #0f0f0f;
      border-radius: 8px;
      font-size: 0.82rem;
      font-weight: 700;
      cursor: pointer;
    }
    #btn-auto-collect.running {
      background: #ef4444;
      color: #fff;
    }
    #collect-interval {
      width: 52px;
      padding: 8px 4px;
      text-align: center;
      background: #222;
      border: 1px solid #333;
      border-radius: 6px;
      color: #f0f0f0;
      font-size: 0.8rem;
    }
    .interval-label {
      font-size: 0.68rem;
      color: #666;
      white-space: nowrap;
    }

    /* Flash-feedback on capture */
    #capture-flash {
      position: absolute;
      inset: 0;
      background: #fff;
      opacity: 0;
      pointer-events: none;
      transition: opacity 0.05s;
      z-index: 10;
    }
    #capture-flash.flash { opacity: 0.35; }

    /* Snapshot preview */
    #snap-wrap {
      display: none;
      position: fixed;
      inset: 0;
      background: #000000cc;
      align-items: center;
      justify-content: center;
      z-index: 99;
      flex-direction: column;
      gap: 14px;
    }
    #snap-wrap.show { display: flex; }
    #snap-img {
      max-width: 92vw;
      max-height: 70vh;
      border-radius: 10px;
      border: 2px solid #4ade80;
    }
    #snap-close {
      background: #ef4444;
      color: #fff;
      border: none;
      border-radius: 8px;
      padding: 10px 32px;
      font-size: 0.9rem;
      font-weight: 600;
      cursor: pointer;
    }
    #snap-save {
      background: #3b82f6;
      color: #fff;
      border: none;
      border-radius: 8px;
      padding: 10px 32px;
      font-size: 0.9rem;
      font-weight: 600;
      cursor: pointer;
    }
    .snap-btns { display: flex; gap: 12px; }
  </style>
</head>
<body>

  <header>
    <h1>Trash<span>Cam</span></h1>
    <div style="display:flex;align-items:center;gap:8px;">
      <button id="btn-collect-toggle" onclick="toggleCollectMode()">⚙ Collect</button>
      <span id="fps-label" style="font-size:0.75rem;color:#666;">-- fps</span>
      <div id="status-dot"></div>
    </div>
  </header>

  <!-- Camera stream -->
  <div id="camera-wrap">
    <img id="stream" alt="camera stream" />
    <div id="capture-flash"></div>
    <div id="overlay">
      <canvas id="tracker-canvas"></canvas>
    </div>
  </div>

  <!-- Controls -->
  <div id="controls">
    <div class="ctrl-row">
      <label>Size</label>
      <select id="sel-framesize">
        <option value="4" selected>QVGA 320×240</option>
        <option value="5">VGA 640×480</option>
        <option value="6">SVGA 800×600</option>
        <option value="8">XGA 1024×768</option>
        <option value="9">HD 1280×720</option>
        <option value="10">SXGA 1280×1024</option>
        <option value="11">UXGA 1600×1200 ★</option>
      </select>
      <label>Quality</label>
      <input type="range" id="sl-quality" min="4" max="30" value="10" />
    </div>
    <div class="btn-row">
      <button class="btn" id="btn-stream"  onclick="toggleStream()">▶ Start</button>
      <button class="btn" id="btn-capture" onclick="takeSnapshot()">📷 Snap</button>
      <button class="btn" id="btn-flash"   onclick="toggleFlash()">💡 Flash</button>
    </div>
  </div>

  <!-- Collect mode panel -->
  <div id="collect-panel">
    <!-- Class selector -->
    <div class="collect-row">
      <label>Class</label>
      <div class="class-pills">
        <div class="pill bio selected" onclick="selectClass('bio')">🌿 Bio</div>
        <div class="pill non_bio"      onclick="selectClass('non_bio')">🚫 Non-Bio</div>
        <div class="pill recyclable"   onclick="selectClass('recyclable')">♻ Recycle</div>
      </div>
    </div>
    <!-- Per-class counters -->
    <div class="counters">
      <div class="counter-box bio">
        <div class="c-label">Bio</div>
        <div class="c-num" id="cnt-bio">0</div>
      </div>
      <div class="counter-box non_bio">
        <div class="c-label">Non-Bio</div>
        <div class="c-num" id="cnt-non_bio">0</div>
      </div>
      <div class="counter-box recyclable">
        <div class="c-label">Recycle</div>
        <div class="c-num" id="cnt-recyclable">0</div>
      </div>
    </div>
    <!-- Capture controls -->
    <div class="collect-actions">
      <button id="btn-snap-one" onclick="collectOne()">📷 Capture</button>
      <input  id="collect-interval" type="number" value="2" min="1" max="10" />
      <span class="interval-label">sec</span>
      <button id="btn-auto-collect" onclick="toggleAutoCollect()">▶ Auto</button>
    </div>
  </div>

  <!-- Snapshot modal -->
  <div id="snap-wrap">
    <img id="snap-img" alt="snapshot" />
    <div class="snap-btns">
      <button id="snap-save"  onclick="saveSnap()">💾 Save</button>
      <button id="snap-close" onclick="closeSnap()">✕ Close</button>
    </div>
  </div>

<script>
  const streamImg   = document.getElementById('stream');
  const statusDot   = document.getElementById('status-dot');
  const fpsLabel    = document.getElementById('fps-label');
  const btnStream   = document.getElementById('btn-stream');
  const btnFlash    = document.getElementById('btn-flash');
  const selSize     = document.getElementById('sel-framesize');
  const slQuality   = document.getElementById('sl-quality');
  const snapWrap    = document.getElementById('snap-wrap');
  const snapImg     = document.getElementById('snap-img');
  const canvas      = document.getElementById('tracker-canvas');
  const ctx         = canvas.getContext('2d');

  let isStreaming = false;
  let flashOn     = false;
  let fpsTimer    = null;
  let frameCount  = 0;
  let rafId       = null;

  // ── Tracker state ─────────────────────────────────────────
  const LABEL_COLORS = {
    'bio':        '#4ade80',  // green
    'non_bio':    '#ef4444',  // red
    'recyclable': '#3b82f6',  // blue
    'uncertain':  '#888888',
    'waiting...': '#444444',
  };

  // Multiple animated boxes — one per detection
  let detBoxes = []; // array of {label, confidence, color, box:{x,y,w,h}, targetBox:{x,y,w,h}}
  let pulsePhase = 0;

  function lerp(a, b, t) { return a + (b - a) * t; }

  function setDetection(data) {
    if (!data.valid || !data.boxes || data.boxes.length === 0) {
      // Fade to idle — single loose gray box
      detBoxes = [{
        label: 'uncertain', confidence: 0, color: '#444444',
        box: { x: 0.10, y: 0.10, w: 0.80, h: 0.80 },
        targetBox: { x: 0.10, y: 0.10, w: 0.80, h: 0.80 }
      }];
      return;
    }
    // Match incoming boxes to existing animated boxes by proximity
    const newBoxes = [];
    for (const d of data.boxes) {
      // Find closest existing box with same label to reuse its animated position
      let match = null;
      let minDist = 999;
      for (const eb of detBoxes) {
        if (eb.label !== d.label) continue;
        const dist = Math.abs(eb.targetBox.x - d.bbox.x) + Math.abs(eb.targetBox.y - d.bbox.y);
        if (dist < minDist) { minDist = dist; match = eb; }
      }
      newBoxes.push({
        label:     d.label,
        confidence: d.confidence,
        color:     d.color || LABEL_COLORS[d.label] || '#888888',
        box:       match ? { ...match.box } : { x: d.bbox.x, y: d.bbox.y, w: d.bbox.w, h: d.bbox.h },
        targetBox: { x: d.bbox.x, y: d.bbox.y, w: d.bbox.w, h: d.bbox.h }
      });
    }
    detBoxes = newBoxes;
  }

  // ── Canvas draw loop ──────────────────────────────────────
  function resizeCanvas() {
    const wrap = document.getElementById('camera-wrap');
    canvas.width  = wrap.clientWidth;
    canvas.height = wrap.clientHeight;
  }

  function drawTracker() {
    // Only resize when needed
    const wrap = document.getElementById('camera-wrap');
    if (canvas.width !== wrap.clientWidth || canvas.height !== wrap.clientHeight) {
      canvas.width  = wrap.clientWidth;
      canvas.height = wrap.clientHeight;
    }
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const W = canvas.width;
    const H = canvas.height;
    pulsePhase += 0.04;

    // Draw each detected box
    for (const det of detBoxes) {
      const spd = 0.20;
      det.box.x = lerp(det.box.x, det.targetBox.x, spd);
      det.box.y = lerp(det.box.y, det.targetBox.y, spd);
      det.box.w = lerp(det.box.w, det.targetBox.w, spd);
      det.box.h = lerp(det.box.h, det.targetBox.h, spd);

      const valid = det.confidence > 0;
      const pulse = valid ? 0.5 + 0.5 * Math.sin(pulsePhase) : 0.4;
      const color = det.color;

      const bx = det.box.x * W;
      const by = det.box.y * H;
      const bw = det.box.w * W;
      const bh = det.box.h * H;

      // Glowing box border
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth   = valid ? 2 : 1.5;
      ctx.globalAlpha = valid ? 0.55 + 0.3 * pulse : 0.2;
      ctx.shadowColor = color;
      ctx.shadowBlur  = valid ? 12 : 4;
      ctx.strokeRect(bx, by, bw, bh);
      ctx.restore();

      // Corner L-brackets
      const cLen = 18;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth   = 3;
      ctx.lineCap     = 'square';
      ctx.globalAlpha = valid ? 0.85 + 0.15 * pulse : 0.4;
      ctx.shadowColor = color;
      ctx.shadowBlur  = valid ? 8 : 2;
      ctx.beginPath(); ctx.moveTo(bx, by + cLen); ctx.lineTo(bx, by); ctx.lineTo(bx + cLen, by); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(bx+bw-cLen, by); ctx.lineTo(bx+bw, by); ctx.lineTo(bx+bw, by+cLen); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(bx, by+bh-cLen); ctx.lineTo(bx, by+bh); ctx.lineTo(bx+cLen, by+bh); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(bx+bw-cLen, by+bh); ctx.lineTo(bx+bw, by+bh); ctx.lineTo(bx+bw, by+bh-cLen); ctx.stroke();
      ctx.restore();

      // Label tag
      if (det.label !== 'uncertain') {
        const tag  = det.label.replace('_', ' ').toUpperCase();
        const conf = valid ? ' ' + (det.confidence * 100).toFixed(0) + '%' : '';
        const text = tag + conf;
        const fSize = Math.max(11, Math.min(15, W * 0.032));

        ctx.save();
        ctx.font = `700 ${fSize}px 'Segoe UI', system-ui, sans-serif`;
        ctx.textBaseline = 'bottom';
        const tw  = ctx.measureText(text).width;
        const pad = 5;
        const tx  = bx;
        const ty  = by - 2;
        const rx  = tx - pad, ry = ty - fSize - pad;
        const rw  = tw + pad * 2, rh = fSize + pad * 2;
        const r   = 4;

        // Pill background
        ctx.globalAlpha = valid ? 0.85 : 0.5;
        ctx.fillStyle   = '#000';
        ctx.beginPath();
        ctx.moveTo(rx+r, ry); ctx.lineTo(rx+rw-r, ry);
        ctx.quadraticCurveTo(rx+rw, ry, rx+rw, ry+r);
        ctx.lineTo(rx+rw, ry+rh-r);
        ctx.quadraticCurveTo(rx+rw, ry+rh, rx+rw-r, ry+rh);
        ctx.lineTo(rx+r, ry+rh);
        ctx.quadraticCurveTo(rx, ry+rh, rx, ry+rh-r);
        ctx.lineTo(rx, ry+r);
        ctx.quadraticCurveTo(rx, ry, rx+r, ry);
        ctx.closePath(); ctx.fill();

        // Accent bar
        ctx.globalAlpha = valid ? 1 : 0.6;
        ctx.fillStyle   = color;
        ctx.fillRect(rx, ry, 3, rh);

        // Text
        ctx.globalAlpha = valid ? 1 : 0.7;
        ctx.fillStyle   = color;
        ctx.shadowColor = color;
        ctx.shadowBlur  = valid ? 5 : 0;
        ctx.fillText(text, tx + pad/2, ty);
        ctx.restore();

        // Confidence bar along bottom
        if (valid) {
          ctx.save();
          ctx.globalAlpha = 0.5;
          ctx.fillStyle   = '#222';
          ctx.fillRect(bx, by+bh+2, bw, 3);
          ctx.globalAlpha = 0.85;
          ctx.fillStyle   = color;
          ctx.shadowColor = color;
          ctx.shadowBlur  = 5;
          ctx.fillRect(bx, by+bh+2, bw * det.confidence, 3);
          ctx.restore();
        }
      }
    }

    rafId = requestAnimationFrame(drawTracker);
  }

  // ── Inference polling ─────────────────────────────────────
  let inferenceTimer = null;

  function startInference() {
    inferenceTimer = setInterval(fetchInference, 600); // poll every 600ms
  }
  function stopInference() {
    clearInterval(inferenceTimer);
    detBoxes = [];
  }

  async function fetchInference() {
    try {
      const host = window.location.hostname;
      const res  = await fetch('http://' + host + '/inference');
      const data = await res.json();
      console.log('EI:', JSON.stringify(data)); // debug — check browser console
      setDetection(data);
    } catch (e) {
      // silent fail
    }
  }

  // ── Stream control ────────────────────────────────────────
  function toggleStream() {
    if (isStreaming) stopStream(); else startStream();
  }

  function startStream() {
    const host = window.location.hostname;
    streamImg.src = 'http://' + host + ':81/stream';
    streamImg.onload = () => { statusDot.classList.add('live'); };
    isStreaming = true;
    btnStream.textContent = '⏹ Stop';
    btnStream.classList.add('streaming');
    detBoxes = []; // reset boxes on stream start
    startFpsCounter();
    startInference();
    if (!rafId) drawTracker();
  }

  function stopStream() {
    streamImg.src = '';
    statusDot.classList.remove('live');
    isStreaming = false;
    btnStream.textContent = '▶ Start';
    btnStream.classList.remove('streaming');
    fpsLabel.textContent = '-- fps';
    clearInterval(fpsTimer);
    cancelAnimationFrame(rafId);
    rafId = null;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    stopInference();
  }

  // ── FPS counter ───────────────────────────────────────────
  function startFpsCounter() {
    clearInterval(fpsTimer);
    frameCount = 0;
    fpsTimer = setInterval(() => {
      fpsLabel.textContent = frameCount > 0 ? frameCount + ' fps' : '-- fps';
      frameCount = 0;
    }, 1000);
    streamImg.addEventListener('load', countFrame);
  }
  function countFrame() { frameCount++; }

  // ── Snapshot ──────────────────────────────────────────────
  function takeSnapshot() {
    const host = window.location.hostname;
    snapImg.src = 'http://' + host + '/capture?' + Date.now();
    snapWrap.classList.add('show');
  }
  function closeSnap() { snapWrap.classList.remove('show'); }
  function saveSnap() {
    const a = document.createElement('a');
    a.href = snapImg.src;
    a.download = 'trash_' + Date.now() + '.jpg';
    a.click();
  }

  // ── Flash ─────────────────────────────────────────────────
  function toggleFlash() {
    flashOn = !flashOn;
    fetch('/control?var=led_intensity&val=' + (flashOn ? 150 : 0));
    btnFlash.classList.toggle('on', flashOn);
    btnFlash.textContent = flashOn ? '💡 Flash ON' : '💡 Flash';
  }

  // ── Settings ──────────────────────────────────────────────
  selSize.addEventListener('change', () => {
    fetch('/control?var=framesize&val=' + selSize.value)
      .then(() => {
        if (isStreaming) { stopStream(); setTimeout(startStream, 400); }
      });
  });

  slQuality.addEventListener('change', () => {
    fetch('/control?var=quality&val=' + slQuality.value);
  });

  // ── Collect mode ──────────────────────────────────────────
  const collectPanel    = document.getElementById('collect-panel');
  const btnCollectToggle = document.getElementById('btn-collect-toggle');
  const btnAutoCollect  = document.getElementById('btn-auto-collect');
  const captureFlash    = document.getElementById('capture-flash');

  let collectMode    = false;
  let autoCollecting = false;
  let autoTimer      = null;
  let activeClass    = 'bio';

  // counters persist per session
  const counts = { bio: 0, non_bio: 0, recyclable: 0 };

  function toggleCollectMode() {
    collectMode = !collectMode;
    collectPanel.classList.toggle('show', collectMode);
    btnCollectToggle.classList.toggle('active', collectMode);
    btnCollectToggle.textContent = collectMode ? '✕ Collect' : '⚙ Collect';
    // pause inference while collecting so ESP32 isn't overloaded
    if (collectMode) {
      stopInference();
    } else {
      stopAutoCollect();
      if (isStreaming) startInference();
    }
  }

  function selectClass(cls) {
    activeClass = cls;
    document.querySelectorAll('.pill').forEach(p => {
      p.classList.toggle('selected', p.classList.contains(cls));
    });
  }

  function flashScreen() {
    captureFlash.classList.add('flash');
    setTimeout(() => captureFlash.classList.remove('flash'), 150);
  }

  async function collectOne() {
    if (!isStreaming) { alert('Start the stream first.'); return; }
    const host = window.location.hostname;
    try {
      // fetch image as blob so we can name it properly
      const res  = await fetch('http://' + host + '/capture?' + Date.now());
      const blob = await res.blob();
      counts[activeClass]++;
      document.getElementById('cnt-' + activeClass).textContent = counts[activeClass];
      const num  = String(counts[activeClass]).padStart(3, '0');
      const name = activeClass + '_' + num + '.jpg';
      const url  = URL.createObjectURL(blob);
      const a    = document.createElement('a');
      a.href = url; a.download = name; a.click();
      URL.revokeObjectURL(url);
      flashScreen();
    } catch (e) {
      console.error('Capture failed', e);
    }
  }

  function toggleAutoCollect() {
    if (autoCollecting) { stopAutoCollect(); return; }
    if (!isStreaming) { alert('Start the stream first.'); return; }
    autoCollecting = true;
    btnAutoCollect.textContent = '⏹ Stop';
    btnAutoCollect.classList.add('running');
    const interval = Math.max(1, parseInt(document.getElementById('collect-interval').value) || 2) * 1000;
    collectOne(); // immediate first shot
    autoTimer = setInterval(collectOne, interval);
  }

  function stopAutoCollect() {
    autoCollecting = false;
    clearInterval(autoTimer);
    btnAutoCollect.textContent = '▶ Auto';
    btnAutoCollect.classList.remove('running');
  }

  window.addEventListener('resize', () => {
    const wrap = document.getElementById('camera-wrap');
    canvas.width  = wrap.clientWidth;
    canvas.height = wrap.clientHeight;
  });

  // ── Auto-start ────────────────────────────────────────────
  window.addEventListener('load', () => { startStream(); });
</script>
</body>
</html>
)rawhtml";
