"""
Push-to-talk web interface for SAGE (default port 8005).

Serves a mobile-friendly hold-to-talk page.  Audio is recorded in the
browser via MediaRecorder, POSTed as WebM, converted to 16 kHz mono
WAV by ffmpeg, and transcribed locally with faster_whisper  the same
engine the on-robot STT uses.  The transcribed text is then fed into
the normal SAGE LLM pipeline so the robot responds via TTS.

Dependencies (all already present on the robot):
    pip install fastapi uvicorn faster-whisper soundfile numpy

Start-up is handled by SageApp  see ``main.py``.
"""

from __future__ import annotations

import asyncio
import io
import logging
import os
import ssl
import subprocess
import tempfile
import threading
import time
from functools import lru_cache
from pathlib import Path
from typing import Any, Optional, TYPE_CHECKING

import numpy as np
import soundfile as sf
import uvicorn
from fastapi import FastAPI, File, UploadFile
from fastapi.responses import HTMLResponse, JSONResponse

if TYPE_CHECKING:
    from config import AppConfig

_log = logging.getLogger("sage.web")

# ======================================================================
# Whisper model (loaded once, reused across requests)
# ======================================================================

_whisper_lock = threading.Lock()
_whisper_model: Optional[Any] = None


def _get_whisper_model(model_name: str, device: str) -> Any:
    """Lazy-load a faster_whisper model (thread-safe singleton)."""
    global _whisper_model
    if _whisper_model is not None:
        return _whisper_model
    with _whisper_lock:
        if _whisper_model is not None:
            return _whisper_model
        import faster_whisper
        _log.info("Loading faster_whisper model '%s' on %s for web PTT", model_name, device)
        compute = "float16" if device == "cuda" else "int8"
        _whisper_model = faster_whisper.WhisperModel(
            model_size_or_path=model_name,
            device=device,
            compute_type=compute,
        )
        _log.info("Web PTT whisper model ready")
        return _whisper_model


def _audio_bytes_to_float32(audio_bytes: bytes) -> Optional[np.ndarray]:
    """Convert browser audio (webm/opus/ogg/wav) � 16 kHz float32 mono.

    Uses ffmpeg for format conversion, which handles every codec the
    browser might produce.
    """
    try:
        result = subprocess.run(
            [
                "ffmpeg", "-y",
                "-i", "pipe:0",
                "-ar", "16000",
                "-ac", "1",
                "-f", "wav",
                "pipe:1",
            ],
            input=audio_bytes,
            capture_output=True,
            timeout=10,
        )
        if result.returncode != 0:
            _log.error("ffmpeg failed: %s", result.stderr.decode(errors="replace")[:300])
            return None

        audio, sr = sf.read(io.BytesIO(result.stdout), dtype="float32")
        if audio.size == 0:
            return None
        return audio

    except FileNotFoundError:
        _log.error("ffmpeg not found  install it: sudo apt install ffmpeg")
        return None
    except Exception:
        _log.exception("Audio conversion error")
        return None


def _transcribe(model: Any, audio: np.ndarray, language: str = "en") -> str:
    """Run faster_whisper transcription and return the joined text."""
    # Normalise to -0.95 dBFS (same as RealtimeSTT)
    peak = np.max(np.abs(audio))
    if peak > 0:
        audio = (audio / peak) * 0.95

    segments, _info = model.transcribe(
        audio, language=language, beam_size=5, vad_filter=True,
    )
    return " ".join(seg.text for seg in segments).strip()


# ======================================================================
# FastAPI application factory
# ======================================================================

def create_app(sage_app: Any, device: str = "cpu") -> FastAPI:
    """Build and return the FastAPI instance.

    Parameters
    ----------
    sage_app:
        The running :class:`main.SageApp` instance (used to inject
        transcribed text into ``_process_user_input``).
    device:
        ``"cuda"`` or ``"cpu"``  passed to faster_whisper.
    """
    api = FastAPI(title="SAGE Push-to-Talk", docs_url=None, redoc_url=None)
    whisper_model_name = sage_app.cfg.stt.whisper_model
    stt_language = sage_app.cfg.stt.language

    @api.get("/", response_class=HTMLResponse)
    async def index():
        return _PAGE_HTML

    @api.get("/health")
    async def health():
        return {"status": "ok", "phase": sage_app.ui.last_phase}

    @api.post("/api/talk")
    async def talk(audio: UploadFile = File(...)):
        """Accept recorded audio, transcribe, and feed to SAGE."""
        start = time.time()

        raw = await audio.read()
        if len(raw) < 500:
            return JSONResponse(
                {"error": "Audio too short"}, status_code=400,
            )

        # Convert to float32 (blocking I/O � run in thread)
        loop = asyncio.get_event_loop()
        samples = await loop.run_in_executor(None, _audio_bytes_to_float32, raw)
        if samples is None:
            return JSONResponse(
                {"error": "Could not decode audio"}, status_code=422,
            )

        # Transcribe
        model = _get_whisper_model(whisper_model_name, device)
        text = await loop.run_in_executor(None, _transcribe, model, samples, stt_language)
        t_transcribe = time.time() - start

        if not text:
            return JSONResponse(
                {"error": "No speech detected"}, status_code=422,
            )

        _log.info("Web PTT transcribed (%0.2fs): %s", t_transcribe, text)

        # Feed into SAGE pipeline (runs LLM + TTS on robot)
        reply = await loop.run_in_executor(
            None, sage_app._process_user_input, text,
        )

        return {
            "user_text": text,
            "reply": reply or "",
            "transcribe_ms": int(t_transcribe * 1000),
        }

    return api


# ======================================================================
# Self-signed certificate (auto-generated once)
# ======================================================================

_CERT_DIR = Path.home() / ".sage" / "certs"
_CERT_FILE = _CERT_DIR / "sage-ptt.pem"
_KEY_FILE  = _CERT_DIR / "sage-ptt-key.pem"


def _ensure_ssl_cert() -> tuple[str, str]:
    """Generate a self-signed cert + key if they don't already exist.

    Returns ``(cert_path, key_path)``.  The cert is valid for 365 days
    and covers any IP on the local network.
    """
    if _CERT_FILE.exists() and _KEY_FILE.exists():
        _log.debug("Reusing existing SSL cert: %s", _CERT_FILE)
        return str(_CERT_FILE), str(_KEY_FILE)

    _CERT_DIR.mkdir(parents=True, exist_ok=True)

    _log.info("Generating self-signed SSL certificate for web PTT&")
    subprocess.run(
        [
            "openssl", "req", "-x509",
            "-newkey", "rsa:2048",
            "-keyout", str(_KEY_FILE),
            "-out", str(_CERT_FILE),
            "-days", "365",
            "-nodes",                    # no passphrase
            "-subj", "/CN=SAGE-Robot",
        ],
        check=True,
        capture_output=True,
    )
    _log.info("SSL certificate written to %s", _CERT_DIR)
    return str(_CERT_FILE), str(_KEY_FILE)


# ======================================================================
# Background server runner
# ======================================================================

class PushToTalkServer:
    """Wraps uvicorn so the web server can be started in a daemon thread.

    Serves over HTTPS with an auto-generated self-signed certificate so
    that browsers allow microphone access from non-localhost origins.
    On first connect the phone will show a "not secure" warning  tap
    "Advanced � Proceed" once and it will be remembered.
    """

    def __init__(self, sage_app: Any, device: str = "cpu") -> None:
        self._sage_app = sage_app
        self._device = device
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        cfg = self._sage_app.cfg.web
        if not cfg.enabled:
            _log.info("Web PTT server disabled in config")
            return

        cert_path, key_path = _ensure_ssl_cert()

        app = create_app(self._sage_app, device=self._device)
        uvi_cfg = uvicorn.Config(
            app, host=cfg.host, port=cfg.port,
            log_level="warning",
            access_log=False,
            ssl_certfile=cert_path,
            ssl_keyfile=key_path,
        )
        server = uvicorn.Server(uvi_cfg)

        self._thread = threading.Thread(
            target=server.run, daemon=True, name="web-ptt",
        )
        self._thread.start()
        _log.info(
            "Web push-to-talk server on https://%s:%d  "
            "(accept the self-signed cert warning on first visit)",
            cfg.host, cfg.port,
        )


# ======================================================================
# HTML page (self-contained)
# ======================================================================

_PAGE_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<title>SAGE Remote</title>
<link rel="preconnect" href="https://fonts.googleapis.com"/>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&family=Outfit:wght@300;500;700&display=swap" rel="stylesheet"/>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#080a0f;--surface:#10131a;--border:#1a1f2e;
  --accent:#38bdf8;--accent-dim:#1e3a5f;--accent-glow:rgba(56,189,248,.25);
  --text:#e2e8f0;--text-dim:#64748b;
  --danger:#f43f5e;--success:#34d399;
  --radius:16px;
  --font-display:'Outfit',system-ui,sans-serif;
  --font-mono:'JetBrains Mono',monospace;
}
html,body{height:100%;overflow:hidden}
body{
  font-family:var(--font-display);background:var(--bg);color:var(--text);
  display:flex;flex-direction:column;align-items:center;justify-content:center;
  -webkit-user-select:none;user-select:none;
  -webkit-tap-highlight-color:transparent;
  touch-action:manipulation;
}

/*  header  */
.header{
  position:absolute;top:0;left:0;right:0;
  padding:20px 24px;display:flex;align-items:center;gap:12px;
}
.logo{
  font-family:var(--font-mono);font-weight:700;font-size:1.1rem;
  letter-spacing:.08em;color:var(--accent);
}
.dot{
  width:8px;height:8px;border-radius:50%;background:var(--success);
  box-shadow:0 0 8px var(--success);
  animation:pulse-dot 2s ease-in-out infinite;
}
@keyframes pulse-dot{0%,100%{opacity:1}50%{opacity:.4}}

/*  status  */
.status-bar{
  margin-bottom:48px;text-align:center;
}
.status-text{
  font-size:.85rem;font-weight:500;color:var(--text-dim);
  letter-spacing:.05em;text-transform:uppercase;
  min-height:1.4em;
  transition:color .3s;
}
.status-text.active{color:var(--accent)}
.status-text.error{color:var(--danger)}

/*  mic button  */
.mic-wrap{
  position:relative;width:160px;height:160px;
  display:flex;align-items:center;justify-content:center;
}
.mic-ring{
  position:absolute;inset:0;border-radius:50%;
  border:2px solid var(--border);
  transition:border-color .3s,box-shadow .3s,transform .3s;
}
.mic-ring.recording{
  border-color:var(--danger);
  box-shadow:0 0 0 6px rgba(244,63,94,.15),0 0 40px rgba(244,63,94,.2);
  animation:ring-pulse 1s ease-in-out infinite;
}
@keyframes ring-pulse{0%,100%{transform:scale(1)}50%{transform:scale(1.06)}}

.mic-btn{
  width:120px;height:120px;border-radius:50%;border:none;
  background:linear-gradient(145deg,#1a2030,#141824);
  box-shadow:4px 4px 16px rgba(0,0,0,.6),-2px -2px 12px rgba(255,255,255,.03);
  display:flex;align-items:center;justify-content:center;
  cursor:pointer;transition:transform .15s,background .3s;
  outline:none;position:relative;z-index:2;
}
.mic-btn:active,.mic-btn.recording{
  transform:scale(.95);
  background:linear-gradient(145deg,#1e1525,#1a1020);
}
.mic-btn svg{
  width:40px;height:40px;
  fill:none;stroke:var(--text-dim);stroke-width:1.8;
  stroke-linecap:round;stroke-linejoin:round;
  transition:stroke .3s;
}
.mic-btn.recording svg{stroke:var(--danger)}

/*  response card  */
.response{
  margin-top:48px;width:min(90vw,400px);
  max-height:35vh;overflow-y:auto;
}
.card{
  background:var(--surface);border:1px solid var(--border);
  border-radius:var(--radius);padding:16px 20px;
  margin-bottom:12px;
  animation:slide-up .35s ease-out;
}
@keyframes slide-up{from{opacity:0;transform:translateY(12px)}to{opacity:1;transform:none}}
.card-label{
  font-family:var(--font-mono);font-size:.65rem;font-weight:700;
  letter-spacing:.1em;text-transform:uppercase;
  color:var(--text-dim);margin-bottom:6px;
}
.card-text{
  font-size:.92rem;line-height:1.55;color:var(--text);font-weight:300;
}
.card.user{border-left:3px solid var(--accent)}
.card.sage{border-left:3px solid var(--success)}
.card.sage .card-label{color:var(--success)}
.card.user .card-label{color:var(--accent)}

/*  hint  */
.hint{
  position:absolute;bottom:32px;
  font-size:.75rem;color:var(--text-dim);letter-spacing:.04em;
}

/*  loading dots  */
.dots span{animation:blink 1.4s infinite both}
.dots span:nth-child(2){animation-delay:.2s}
.dots span:nth-child(3){animation-delay:.4s}
@keyframes blink{0%,80%,100%{opacity:.2}40%{opacity:1}}
</style>
</head>
<body>

<div class="header">
  <div class="dot" id="dot"></div>
  <div class="logo">SAGE</div>
</div>

<div class="status-bar">
  <div class="status-text" id="status">Hold to talk</div>
</div>

<div class="mic-wrap">
  <div class="mic-ring" id="ring"></div>
  <button class="mic-btn" id="mic" aria-label="Push to talk">
    <svg viewBox="0 0 24 24">
      <rect x="9" y="1" width="6" height="13" rx="3"/>
      <path d="M5 10a7 7 0 0 0 14 0"/>
      <line x1="12" y1="17" x2="12" y2="22"/>
      <line x1="8" y1="22" x2="16" y2="22"/>
    </svg>
  </button>
</div>

<div class="response" id="response"></div>
<div class="hint">hold the mic button &middot; release to send</div>

<script>
const mic   = document.getElementById('mic');
const ring  = document.getElementById('ring');
const status = document.getElementById('status');
const resp  = document.getElementById('response');

let mediaRec = null;
let chunks   = [];
let recording = false;
let stream   = null;

function setStatus(msg, cls='') {
  status.textContent = msg;
  status.className = 'status-text' + (cls ? ' '+cls : '');
}

function dots(msg) {
  return msg + '<span class="dots"><span>.</span><span>.</span><span>.</span></span>';
}

async function startRecording() {
  if (recording) return;
  try {
    // Request mic on first press
    if (!stream) {
      stream = await navigator.mediaDevices.getUserMedia({
        audio: { channelCount:1, sampleRate:16000, echoCancellation:true, noiseSuppression:true }
      });
    }

    chunks = [];
    // Prefer webm/opus; fall back to whatever the browser supports
    const mimeType = MediaRecorder.isTypeSupported('audio/webm;codecs=opus')
      ? 'audio/webm;codecs=opus'
      : (MediaRecorder.isTypeSupported('audio/webm') ? 'audio/webm' : '');

    mediaRec = new MediaRecorder(stream, mimeType ? { mimeType } : {});
    mediaRec.ondataavailable = e => { if (e.data.size > 0) chunks.push(e.data); };
    mediaRec.start(100); // collect in 100ms chunks
    recording = true;

    mic.classList.add('recording');
    ring.classList.add('recording');
    setStatus('Listening\u2026', 'active');
  } catch(e) {
    setStatus('Mic access denied', 'error');
    console.error(e);
  }
}

async function stopRecording() {
  if (!recording || !mediaRec) return;
  recording = false;
  mic.classList.remove('recording');
  ring.classList.remove('recording');

  return new Promise(resolve => {
    mediaRec.onstop = async () => {
      const blob = new Blob(chunks, { type: mediaRec.mimeType || 'audio/webm' });
      chunks = [];
      resolve(blob);
    };
    mediaRec.stop();
  });
}

async function send(blob) {
  status.innerHTML = dots('Transcribing');
  status.className = 'status-text active';

  const form = new FormData();
  form.append('audio', blob, 'recording.webm');

  try {
    const r = await fetch('/api/talk', { method:'POST', body: form });
    const data = await r.json();

    if (!r.ok) {
      setStatus(data.error || 'Error', 'error');
      return;
    }

    // Show result cards
    resp.innerHTML =
      `<div class="card user">
         <div class="card-label">You</div>
         <div class="card-text">${esc(data.user_text)}</div>
       </div>
       <div class="card sage">
         <div class="card-label">SAGE</div>
         <div class="card-text">${esc(data.reply || '(no reply)')}</div>
       </div>`;

    setStatus('Hold to talk');
  } catch(e) {
    setStatus('Network error', 'error');
    console.error(e);
  }
}

function esc(s) {
  const d = document.createElement('div');
  d.textContent = s;
  return d.innerHTML;
}

//  pointer events (work on both touch + mouse) 
mic.addEventListener('pointerdown', e => {
  e.preventDefault();
  mic.setPointerCapture(e.pointerId);
  startRecording();
});
mic.addEventListener('pointerup', async e => {
  e.preventDefault();
  const blob = await stopRecording();
  if (blob && blob.size > 500) send(blob);
  else setStatus('Too short  hold longer', 'error');
});
mic.addEventListener('pointercancel', async () => {
  await stopRecording();
  setStatus('Cancelled', 'error');
});

// Prevent context menu on long press (mobile)
mic.addEventListener('contextmenu', e => e.preventDefault());
</script>
</body>
</html>
"""