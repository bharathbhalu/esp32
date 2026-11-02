#!/usr/bin/env python3
"""Simple local web UI to flash the PlatformIO projects in this esp32/ folder.

Usage: python3 flasher.py   (opens http://127.0.0.1:8765)
"""
import glob
import json
import os
import shutil
import subprocess
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST, PORT = "127.0.0.1", 8765
ESP32_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIO = shutil.which("pio") or "/opt/homebrew/bin/pio"

log = ""
running = False


def projects():
    return sorted(n for n in os.listdir(ESP32_DIR)
                  if os.path.isfile(os.path.join(ESP32_DIR, n, "platformio.ini")))


def ports():
    return sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))


def flash(project, port):
    global log, running
    cmd = [PIO, "run", "-d", os.path.join(ESP32_DIR, project), "-t", "upload", "--upload-port", port]
    log = f"Flashing {project} to {port}...\n"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
    for line in proc.stdout:
        log += line
    log += "\nDONE\n" if proc.wait() == 0 else "\nFAILED\n"
    running = False


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, body, ctype="application/json"):
        data = body.encode() if isinstance(body, str) else json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self.send(PAGE, "text/html")
        elif self.path == "/state":
            self.send({"projects": projects(), "ports": ports(), "log": log, "running": running})

    def do_POST(self):
        global running
        req = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        # Only accept names we listed ourselves
        if not running and req["project"] in projects() and req["port"] in ports():
            running = True
            threading.Thread(target=flash, args=(req["project"], req["port"]), daemon=True).start()
        self.send({})


PAGE = """<!doctype html>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Flasher</title>
<style>
  body { font: 15px sans-serif; max-width: 800px; margin: 24px auto; padding: 0 16px; }
  select, button { font: inherit; padding: 6px 10px; margin: 4px 4px 4px 0; }
  pre { background: #111; color: #ddd; padding: 10px; height: 400px; overflow: auto; white-space: pre-wrap; }
</style>
<h2>ESP32 Flasher</h2>
Firmware <select id="project"></select>
Port <select id="port"></select>
<button id="go">Flash</button>
<pre id="log"></pre>
<script>
const $ = id => document.getElementById(id);
function fill(sel, items) {
  if (sel.options.length === items.length) return;
  const keep = sel.value;
  sel.innerHTML = items.map(i => `<option>${i}</option>`).join('');
  if (items.includes(keep)) sel.value = keep;
}
async function refresh() {
  const s = await (await fetch('/state')).json();
  fill($('project'), s.projects);
  fill($('port'), s.ports);
  $('go').disabled = s.running || !s.ports.length;
  if ($('log').textContent !== s.log) { $('log').textContent = s.log; $('log').scrollTop = 1e9; }
}
$('go').onclick = () => fetch('/', { method: 'POST',
  body: JSON.stringify({ project: $('project').value, port: $('port').value }) });
refresh(); setInterval(refresh, 500);
</script>
"""

if __name__ == "__main__":
    print(f"ESP32 flasher at http://{HOST}:{PORT}  (Ctrl+C to stop)")
    threading.Timer(0.5, lambda: webbrowser.open(f"http://{HOST}:{PORT}")).start()
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
