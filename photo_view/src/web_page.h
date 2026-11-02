// Upload page served by the board. Cropping happens in the browser: the photo is
// drawn into a 368x448 frame (drag, zoom, rotate) and uploaded as a JPEG.
#pragma once

static const char WEB_PAGE[] = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Photo Frame</title>
<style>
  :root { --bg:#f5f6f8; --panel:#fff; --text:#1d2330; --muted:#667085; --border:#d9dde5; --accent:#2563eb; --danger:#c62828; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#0d1117; --panel:#161b22; --text:#e6edf3; --muted:#8b949e; --border:#30363d; --accent:#3b82f6; --danger:#ef5350; }
  }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--text); font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
  main { max-width:720px; margin:0 auto; padding:20px 16px; }
  h1 { font-size:20px; margin:0 0 4px; }
  .sub { color:var(--muted); margin:0 0 16px; }
  .panel { background:var(--panel); border:1px solid var(--border); border-radius:10px; padding:16px; margin-bottom:16px; }
  .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; }
  button, .file { font:inherit; padding:8px 14px; border-radius:8px; border:1px solid var(--border); background:var(--panel); color:var(--text); cursor:pointer; }
  button.primary { background:var(--accent); border-color:var(--accent); color:#fff; }
  button:disabled { opacity:.5; cursor:default; }
  .file input { display:none; }
  #stage { display:block; margin:12px auto; width:276px; height:336px; border-radius:10px; background:#000; touch-action:none; cursor:grab; }
  input[type=range] { flex:1; min-width:140px; }
  .status { color:var(--muted); min-height:1.4em; margin-top:8px; }
  .grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(110px,1fr)); gap:10px; }
  .item { text-align:center; }
  .item img { width:100%; aspect-ratio:368/448; object-fit:cover; border-radius:8px; background:#000; display:block; }
  .item button { margin-top:6px; width:100%; padding:4px; color:var(--danger); }
  .muted { color:var(--muted); }
  .sel { font:inherit; padding:6px 10px; border-radius:8px; border:1px solid var(--border); background:var(--panel); color:var(--text); }
</style>
</head>
<body>
<main>
  <h1>Photo Frame</h1>
  <p class="sub">Pick a photo, crop it to the screen, and upload.</p>

  <div class="panel">
    <div class="row">
      <label class="file">Choose photo<input type="file" id="file" accept="image/*"></label>
      <button id="rotL" disabled>&#8634; Rotate left</button>
      <button id="rotR" disabled>Rotate right &#8635;</button>
      <button id="fit" disabled>Fit whole photo</button>
    </div>
    <canvas id="stage" width="368" height="448"></canvas>
    <div class="row">
      <span class="muted">Zoom</span>
      <input type="range" id="zoom" min="1" max="4" step="0.01" value="1" disabled>
      <button id="upload" class="primary" disabled>Upload to board</button>
    </div>
    <div class="status" id="status">Drag to move, pinch or use the slider to zoom.</div>
  </div>

  <div class="panel">
    <div class="row" style="justify-content:space-between">
      <strong>Slideshow</strong>
      <button id="pause">Pause</button>
    </div>
    <div class="row" style="margin-top:12px">
      <span class="muted">Change every</span>
      <select id="interval" class="sel"></select>
    </div>
    <div class="row" style="margin-top:10px">
      <span class="muted">Transition</span>
      <select id="transition" class="sel"></select>
      <span class="muted">Speed</span>
      <select id="speed" class="sel"></select>
    </div>
    <div class="row" style="margin-top:10px">
      <label><input type="checkbox" id="shuffle"> Shuffle</label>
      <label><input type="checkbox" id="builtin"> Include built-in photos</label>
    </div>
    <div class="status muted">On the board: double-tap to pause or resume (red dot = paused); swipe up from the bottom for these settings.</div>
  </div>

  <div class="panel">
    <div class="row" style="justify-content:space-between">
      <strong>On the board</strong><span class="muted" id="count"></span>
    </div>
    <div class="grid" id="list" style="margin-top:12px"></div>
  </div>
</main>
<script>
const W = 368, H = 448;
const stage = document.getElementById('stage'), ctx = stage.getContext('2d');
const $ = id => document.getElementById(id);
let img = null, rot = 0, zoom = 1, ox = 0, oy = 0; // offset in frame pixels
let fit = false; // false: cover the frame (crop); true: whole photo visible (black bars)

// Scale that makes the (rotated) image just cover, or just fit inside, the frame
function baseScale() {
  const w = rot % 2 ? img.height : img.width, h = rot % 2 ? img.width : img.height;
  return fit ? Math.min(W / w, H / h) : Math.max(W / w, H / h);
}
// Keep the image within reach: covering the frame when larger, inside it when smaller
function clamp() {
  const s = baseScale() * zoom;
  const w = (rot % 2 ? img.height : img.width) * s, h = (rot % 2 ? img.width : img.height) * s;
  const mx = Math.abs(w - W) / 2, my = Math.abs(h - H) / 2;
  ox = Math.max(-mx, Math.min(mx, ox));
  oy = Math.max(-my, Math.min(my, oy));
}
function render(target) {
  const c = target || ctx;
  c.fillStyle = '#000'; c.fillRect(0, 0, W, H);
  if (!img) return;
  const s = baseScale() * zoom;
  c.save();
  c.translate(W / 2 + ox, H / 2 + oy);
  c.rotate(rot * Math.PI / 2);
  c.scale(s, s);
  c.drawImage(img, -img.width / 2, -img.height / 2);
  c.restore();
}

$('file').onchange = e => {
  const f = e.target.files[0];
  if (!f) return;
  const url = URL.createObjectURL(f);
  const im = new Image();
  im.onload = () => {
    img = im; rot = 0; zoom = 1; ox = oy = 0;
    $('zoom').value = 1;
    for (const id of ['rotL', 'rotR', 'fit', 'zoom', 'upload']) $(id).disabled = false;
    render();
    $('status').textContent = 'Drag to move, pinch or use the slider to zoom.';
  };
  im.src = url;
};
$('rotL').onclick = () => { rot = (rot + 3) % 4; ox = oy = 0; clamp(); render(); };
$('rotR').onclick = () => { rot = (rot + 1) % 4; ox = oy = 0; clamp(); render(); };
$('zoom').oninput = e => { zoom = +e.target.value; clamp(); render(); };
$('fit').onclick = () => {
  fit = !fit;
  $('fit').textContent = fit ? 'Fill screen' : 'Fit whole photo';
  zoom = 1; $('zoom').value = 1; ox = oy = 0;
  render();
};

// Drag with one pointer, pinch-zoom with two
const pointers = new Map();
let pinchStart = 0, zoomStart = 1;
const toFrame = () => W / stage.getBoundingClientRect().width;
stage.addEventListener('pointerdown', e => { stage.setPointerCapture(e.pointerId); pointers.set(e.pointerId, e); });
stage.addEventListener('pointermove', e => {
  if (!img || !pointers.has(e.pointerId)) return;
  const prev = pointers.get(e.pointerId);
  pointers.set(e.pointerId, e);
  if (pointers.size === 1) {
    ox += (e.clientX - prev.clientX) * toFrame();
    oy += (e.clientY - prev.clientY) * toFrame();
  } else if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    const d = Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
    if (!pinchStart) { pinchStart = d; zoomStart = zoom; }
    zoom = Math.max(1, Math.min(4, zoomStart * d / pinchStart));
    $('zoom').value = zoom;
  }
  clamp(); render();
});
const endPointer = e => { pointers.delete(e.pointerId); if (pointers.size < 2) pinchStart = 0; };
stage.addEventListener('pointerup', endPointer);
stage.addEventListener('pointercancel', endPointer);
stage.addEventListener('wheel', e => {
  if (!img) return;
  e.preventDefault();
  zoom = Math.max(1, Math.min(4, zoom * (e.deltaY < 0 ? 1.08 : 1 / 1.08)));
  $('zoom').value = zoom; clamp(); render();
}, { passive: false });

$('upload').onclick = () => {
  const out = document.createElement('canvas');
  out.width = W; out.height = H;
  render(out.getContext('2d'));
  $('upload').disabled = true;
  $('status').textContent = 'Uploading...';
  out.toBlob(async blob => {
    const form = new FormData();
    form.append('photo', blob, 'photo.jpg');
    try {
      const res = await fetch('/upload', { method: 'POST', body: form });
      const msg = await res.text();
      $('status').textContent = res.ok ? 'Uploaded - showing on the board now.' : 'Upload failed: ' + msg;
      loadList();
    } catch (err) {
      $('status').textContent = 'Upload failed: ' + err;
    }
    $('upload').disabled = false;
  }, 'image/jpeg', 0.9);
};

async function loadList() {
  const res = await fetch('/list');
  const data = await res.json();
  $('count').textContent = data.photos.length + ' uploaded, ' + Math.round(data.free / 1024) + ' KB free';
  const list = $('list');
  list.innerHTML = '';
  if (!data.photos.length) { list.innerHTML = '<span class="muted">No uploaded photos yet.</span>'; return; }
  for (const name of data.photos) {
    const item = document.createElement('div');
    item.className = 'item';
    const im = document.createElement('img');
    im.src = '/photo?name=' + encodeURIComponent(name);
    im.loading = 'lazy';
    const del = document.createElement('button');
    del.textContent = 'Delete';
    del.onclick = async () => {
      del.disabled = true;
      await fetch('/delete?name=' + encodeURIComponent(name), { method: 'POST' });
      loadList();
    };
    item.append(im, del);
    list.append(item);
  }
}
// Slideshow settings
let paused = false;
function showSettings(s) {
  const sel = $('interval');
  if (!sel.options.length)
    s.options.forEach((sec, i) => sel.add(new Option(sec ? (sec >= 60 ? sec / 60 + ' min' : sec + ' s') : 'Off', i)));
  sel.value = s.interval;
  if (!$('transition').options.length) s.transitions.forEach((n, i) => $('transition').add(new Option(n, i)));
  if (!$('speed').options.length) s.speeds.forEach((n, i) => $('speed').add(new Option(n, i)));
  $('transition').value = s.transition;
  $('speed').value = s.speed;
  $('shuffle').checked = s.shuffle;
  $('builtin').checked = s.builtin;
  paused = s.paused;
  $('pause').textContent = paused ? 'Resume' : 'Pause';
  $('pause').disabled = !s.options[s.interval];
}
async function saveSettings(params) {
  const res = await fetch('/settings?' + new URLSearchParams(params), { method: 'POST' });
  showSettings(await res.json());
}
$('interval').onchange = e => saveSettings({ interval: e.target.value });
$('transition').onchange = e => saveSettings({ transition: e.target.value });
$('speed').onchange = e => saveSettings({ speed: e.target.value });
$('shuffle').onchange = e => saveSettings({ shuffle: e.target.checked ? 1 : 0 });
$('builtin').onchange = e => saveSettings({ builtin: e.target.checked ? 1 : 0 });
$('pause').onclick = () => saveSettings({ paused: paused ? 0 : 1 });

render();
loadList();
fetch('/settings').then(r => r.json()).then(showSettings);
</script>
</body>
</html>
)HTML";
