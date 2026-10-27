"""Live health board for the teleop server.

Each data stream (Quest, Rokoko, arm, LEAP hand...) records packets on a
StreamStats; a small HTTP server exposes them as JSON and as a status page,
so a dead stream shows up as a red row instead of a silent hang.
Only uses the standard library so it runs inside the existing containers.
"""
import json
import logging
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class StreamStats:
    def __init__(self, label, warn_after=0.5, dead_after=3.0):
        self.label = label
        self.warn_after = warn_after
        self.dead_after = dead_after
        self.count = 0
        self.last_ts = None
        self.detail = {}
        self._times = deque(maxlen=2000)
        self._lock = threading.Lock()

    def mark(self, **detail):
        now = time.time()
        with self._lock:
            self.count += 1
            self.last_ts = now
            self._times.append(now)
            self.detail.update(detail)

    def set_detail(self, **detail):
        with self._lock:
            self.detail.update(detail)

    def age(self):
        return None if self.last_ts is None else time.time() - self.last_ts

    def snapshot(self):
        now = time.time()
        with self._lock:
            while self._times and now - self._times[0] > 2.0:
                self._times.popleft()
            rate = len(self._times) / 2.0
            age = None if self.last_ts is None else now - self.last_ts
            if age is None or age > self.dead_after:
                state = "dead"
            elif age > self.warn_after:
                state = "stale"
            else:
                state = "ok"
            return {
                "label": self.label,
                "state": state,
                "age": age,
                "rate": rate,
                "count": self.count,
                "detail": dict(self.detail),
            }


class StatusBoard:
    def __init__(self):
        self.streams = {}
        self.info = {}
        self.events = deque(maxlen=50)
        self.started = time.time()
        self._lock = threading.Lock()

    def stream(self, key, label, warn_after=0.5, dead_after=3.0):
        if key not in self.streams:
            self.streams[key] = StreamStats(label, warn_after, dead_after)
        return self.streams[key]

    def set_info(self, **info):
        with self._lock:
            self.info.update(info)

    def event(self, level, msg):
        with self._lock:
            self.events.append({"t": time.time(), "level": level, "msg": msg})

    def snapshot(self):
        with self._lock:
            info = dict(self.info)
            events = list(self.events)
        return {
            "now": time.time(),
            "uptime": time.time() - self.started,
            "streams": {k: s.snapshot() for k, s in self.streams.items()},
            "info": info,
            "events": events,
        }


class BoardLogHandler(logging.Handler):
    """Mirrors warnings/errors from a logger onto the board's event list."""

    def __init__(self, board, level=logging.WARNING):
        super().__init__(level)
        self.board = board

    def emit(self, record):
        self.board.event(record.levelname, record.getMessage())


class RateLimitedWarning:
    """Logs a repeated condition at most once per `period` seconds."""

    def __init__(self, logger, period=5.0):
        self.logger = logger
        self.period = period
        self._last = {}

    def __call__(self, key, msg):
        now = time.time()
        if now - self._last.get(key, 0) >= self.period:
            self._last[key] = now
            self.logger.warning(msg)


def start_status_server(board, port=8080, host="0.0.0.0"):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/status.json"):
                body = json.dumps(board.snapshot()).encode()
                ctype = "application/json"
            elif self.path == "/" or self.path.startswith("/index"):
                body = STATUS_PAGE.encode()
                ctype = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # the client gave up waiting (e.g. the launcher's 1 s timeout); nothing to report

        def log_message(self, *args):
            pass

    class QuietServer(ThreadingHTTPServer):
        def handle_error(self, request, client_address):
            # Default prints a traceback to the teleop's output for every dropped client
            if not isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
                super().handle_error(request, client_address)

    server = QuietServer((host, port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


STATUS_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleop Status</title>
<style>
:root {
  --bg: #f6f7f9; --card: #ffffff; --text: #1b1f24; --muted: #646d78; --border: #dde1e6;
  --ok: #1f8a4c; --stale: #b7791f; --dead: #c53030;
}
@media (prefers-color-scheme: dark) {
  :root { --bg: #121417; --card: #1c1f24; --text: #e6e9ed; --muted: #9aa3ad; --border: #2c3138;
          --ok: #48bb78; --stale: #ecc94b; --dead: #fc8181; }
}
* { box-sizing: border-box; }
body { margin: 0; padding: 16px; background: var(--bg); color: var(--text);
       font: 15px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
h1 { font-size: 20px; margin: 0 0 4px; }
.sub { color: var(--muted); font-size: 13px; margin-bottom: 16px; }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 12px; }
.card { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 12px 14px; }
.row { display: flex; align-items: center; gap: 8px; }
.dot { width: 12px; height: 12px; border-radius: 50%; flex: none; }
.ok .dot { background: var(--ok); } .stale .dot { background: var(--stale); } .dead .dot { background: var(--dead); }
.name { font-weight: 600; }
.state { margin-left: auto; font-size: 12px; text-transform: uppercase; letter-spacing: .04em; }
.ok .state { color: var(--ok); } .stale .state { color: var(--stale); } .dead .state { color: var(--dead); }
.nums { display: flex; gap: 16px; margin-top: 8px; font-variant-numeric: tabular-nums; }
.nums b { font-size: 18px; } .nums span { color: var(--muted); font-size: 12px; display: block; }
.detail { margin-top: 6px; color: var(--muted); font-size: 12px; word-break: break-word; }
h2 { font-size: 15px; margin: 20px 0 8px; }
table { width: 100%; border-collapse: collapse; font-size: 13px; }
td { padding: 4px 6px; border-top: 1px solid var(--border); vertical-align: top; }
td.t { color: var(--muted); white-space: nowrap; font-variant-numeric: tabular-nums; }
.WARNING { color: var(--stale); } .ERROR, .CRITICAL { color: var(--dead); }
#conn.down { color: var(--dead); font-weight: 600; }
</style>
</head>
<body>
<h1>Teleop status</h1>
<div class="sub"><span id="conn">connecting…</span> · <span id="info"></span></div>
<div class="grid" id="streams"></div>
<h2>Recent warnings</h2>
<table id="events"></table>
<script>
const fmtAge = a => a == null ? "never" : a < 1 ? (a * 1000).toFixed(0) + " ms" : a < 120 ? a.toFixed(1) + " s" : (a / 60).toFixed(0) + " min";
const esc = s => String(s).replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
async function tick() {
  const conn = document.getElementById("conn");
  try {
    const r = await fetch("status.json", {cache: "no-store"});
    const s = await r.json();
    conn.textContent = "teleop server up " + fmtAge(s.uptime);
    conn.className = "";
    document.getElementById("info").textContent =
      Object.entries(s.info).map(([k, v]) => k + ": " + v).join(" · ");
    document.getElementById("streams").innerHTML = Object.values(s.streams).map(x => `
      <div class="card ${x.state}">
        <div class="row"><span class="dot"></span><span class="name">${esc(x.label)}</span><span class="state">${x.state}</span></div>
        <div class="nums"><div><b>${x.rate.toFixed(1)}</b><span>Hz</span></div>
          <div><b>${fmtAge(x.age)}</b><span>since last</span></div>
          <div><b>${x.count}</b><span>total</span></div></div>
        <div class="detail">${Object.entries(x.detail).map(([k, v]) => esc(k) + ": " + esc(v)).join("<br>")}</div>
      </div>`).join("");
    document.getElementById("events").innerHTML = s.events.slice().reverse().map(e =>
      `<tr><td class="t">${new Date(e.t * 1000).toLocaleTimeString()}</td><td class="${e.level}">${esc(e.msg)}</td></tr>`).join("");
  } catch (e) {
    conn.textContent = "teleop server not responding";
    conn.className = "down";
  }
}
tick();
setInterval(tick, 500);
</script>
</body>
</html>
"""
