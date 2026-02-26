import os
import sys
import signal
import threading
import time
import queue
import requests
import openai
import traceback
import subprocess
import torch

from RealtimeSTT import AudioToTextRecorder
from piper_tts import PiperTTS
import threading, rclpy
from rclpy.executors import MultiThreadedExecutor
from nav2async import Nav2AsyncBridge
from waypoints import WAYPOINTS
from utils import read_openai_key, json 
from ui_state_client import UIStatePublisher


ui = UIStatePublisher()

# Spin ROS in a background thread once at startup
_nav_exec = None
_nav_node = None
def start_ros_background():
    global _nav_exec, _nav_node
    if _nav_node: 
        return
    if not rclpy.ok():
        rclpy.init()
    _nav_node = Nav2AsyncBridge(enqueue_arrival=enqueue_arrival)
    _nav_exec = MultiThreadedExecutor()
    _nav_exec.add_node(_nav_node)
    threading.Thread(target=_nav_exec.spin, daemon=True).start()

def nav_status():
    if not _nav_node:
        return {"status": "offline"}
    return {
        "status": _nav_node.status,
        "target_name": getattr(_nav_node, "current_target", None),  # <-- changed
        "feedback": _nav_node.last_feedback,
    }

# =========================
# Config / Flags
# =========================
VERBOSE = True                 # top-level logs
VERBOSE_TOKENS = False         # per-token logging
VERBOSE_TOOLS = True           # tool delta/args/results
VERBOSE_ROBOT = False
VERBOSE_TTS = True
VERBOSE_STT = True

MODEL_NAME = "gpt-4.1-nano" # gpt-4.1-nano for fastest responses.

# =========================
# UI & backend endpoints
# =========================
ROBOT_BACKEND_URL = "http://127.0.0.1:8002"
SEARCH_URL = "http://127.0.0.1:8004/api/kb/search"
ALSA_DEVICE = "pulse"  # IMPORTANT: go through Pulse/PipeWire
# =========================
# Globals
# =========================
messages = []                 # shared conversation history (system injected per call)
ai_reply = ""
done = False

event_q = queue.PriorityQueue()  # (priority, ts, event_dict)
messages_lock = threading.Lock()
llm_lock = threading.Lock()

is_recording_flag = False
nav_epoch = 0
current_target = None

client = None
tts = None
recorder = None

shutdown_flag = False

# =========================
# Logging helpers
# =========================
def log(msg, *, lvl="INFO"):
    if VERBOSE:
        print(f"[{time.strftime('%H:%M:%S')}] {lvl}: {msg}", flush=True)

def log_exc(where):
    etype, e, tb = sys.exc_info()
    log(f"{where} EXCEPTION: {e}\n{''.join(traceback.format_tb(tb))}", lvl="ERROR")

# =========================
# Tools schema
# =========================
tools = [
    {
        "type": "function",
        "function": {
            "name": "set_goal",
            "description": "Start or update Nav2 goal to a named waypoint. Returns immediately.",
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {"type": "string", "description": f"One of: {', '.join(WAYPOINTS.keys())}"}
                },
                "required": ["location"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_goal",
            "description": "Cancel the current Nav2 goal (if any).",
            "parameters": {"type": "object", "properties": {}}
        }
    },
    {
    "type": "function",
    "function": {
        "name": "valpo_search",
        "description": (
            "Search the local Valparaiso University knowledge base for factual information. "
            "Use this whenever the user asks about Valpo policies, programs, offices, "
            "deadlines, facilities, departments, campus resources, or other university-specific details. "
            "Returns relevant text chunks with citations."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": (
                        "The natural language search query describing what information is needed. "
                        "Should capture the user's question as clearly as possible."
                    )
                },
                "top_k": {
                    "type": "integer",
                    "description": (
                        "Maximum number of relevant chunks to retrieve from the knowledge base."
                    ),
                    "default": 3,
                    "minimum": 1,
                    "maximum": 20
                }
            },
            "required": ["query"]
        }
    }
}
]

def generate_system_prompt():
    """
    Generate system prompt with current robot status (real robot, Nav2 only).
    - Avoids x, y or low-level values
    - Shows only status, target name, distance remaining (if moving)
    - Lists available waypoint names
    - Describes tools: set_goal(location), cancel_goal()
    """
    # --- Runtime status ---
    status_lines = []
    try:
        ns = nav_status()
        if ns and isinstance(ns, dict):
            nav2_status = ns.get("status", "unknown").lower()
            tgt = ns.get("target_name") or ns.get("target_xy")
            fb = ns.get("feedback") or {}

            status_lines.append(f"- Navigation status: {nav2_status.capitalize()}")
            if tgt:
                if isinstance(tgt, str):
                    status_lines.append(f"- Current goal: {tgt}")
                elif isinstance(tgt, (list, tuple)) and len(tgt) >= 2:
                    status_lines.append(f"- Moving toward active goal")
            if fb.get("distance_remaining") is not None and nav2_status == "navigating":
                try:
                    dr = float(fb["distance_remaining"])
                    status_lines.append(f"- Distance remaining: {dr:.1f} meters")
                except Exception:
                    pass
    except Exception:
        pass

    status_section = ""
    if status_lines:
        status_section = "\n\nCURRENT ROBOT STATUS:\n" + "\n".join(status_lines)

    # --- Waypoint list ---
    waypoint_names = sorted(WAYPOINTS.keys())
    waypoints_bulleted = "\n".join(f"- {name}: {WAYPOINTS[name]['description']}" for name in waypoint_names)

    base_prompt = f"""
You are SAGE — a friendly, witty, and helpful tour guide robot at Valparaiso University's College of Engineering (Gellersen).

Your job is to guide visitors through the building by driving to specific named locations and engaging them with short, clear, and fun dialogue. 
If someone asks something off-topic or inappropriate, respond humorously but stay professional.

---

### HOW TO ACT
- When asked to go somewhere, **translate the request** to a waypoint name from the list below.
- **Set only one goal at a time.** Do not queue or chain destinations.
- After arrival, you'll receive an event prompt such as:
  `[EVENT PROMPT] Arrived at Senior_Design.`
  At that point, say welcome and follow any next step requested (e.g., return to Docking_Station).

Example:
    User: Go to Senior Design and come back to Docking Station?
    Tool Call: set_goal("Senior_Design")
    [robot drives to Senior Design...]
    System: [EVENT PROMPT] Arrived at Senior_Design.
    Tool Call: set_goal("Docking_Station")
    Assistant: Welcome to Senior Design! Now heading back to Docking Station as requested.
---

### RULES
- Use **only** the exact waypoint names from the list below when calling tools.
- Speak naturally — plain English only (no symbols like *, #, or _).
- While driving, you may chat, but keep it brief.
- If the user changes their mind, **cancel the current goal** before setting a new one.

---

### AVAILABLE WAYPOINTS
{waypoints_bulleted}

---

### TOOLS
- **set_goal(location)** → Start driving toward a waypoint (non-blocking; you can still talk).
- **cancel_goal()** → Stop the current route immediately.
- **valpo_search(query, top_k)** → Search Valpo KB for factual info (use for policies, programs, offices, deadlines, facilities, departments, campus resources, etc.). Returns text chunks with citations.
When answering Valparaiso University-specific questions, always call valpo_search first. Only answer using retrieved information. If the knowledge base does not contain sufficient information, say so clearly instead of guessing.
---

### ARRIVAL BEHAVIOR
When you arrive:
- Say: “Welcome to <waypoint name>.”
- Then wait for further instructions or proceed as previously told.

---

### PERSONALITY
- your speech transcription isnt perfect so you may hear, gellerson or galerson instead of gellersen but anything close to gellersen should be assumed to be gellersen.
- Be warm, concise, and a little witty.
- Sound like a friendly student helper, not a formal assistant.
- You were created by **Fayol**, a senior computer engineering student, as part of his **Fall 2025 Senior Design project**.
- You're a **prototype**, so it's okay to be playful about your limitations.

---
"""

    return base_prompt + status_section if status_section else base_prompt


# =========================
# Robot backend
# =========================
def set_goal(location: str):
    global nav_epoch, current_target
    start_ros_background()
    if location not in WAYPOINTS:
        return f"Unknown location '{location}'. Valid: {', '.join(WAYPOINTS.keys())}"
    wp = WAYPOINTS[location]
    nav_epoch += 1                 # <-- add
    current_target = location      # <-- add
    msg = _nav_node.set_goal(
        frame_id=wp.get("frame_id", "map"),
        x=wp["x"], y=wp["y"], ox=wp["ox"], oy=wp["oy"], oz=wp["oz"], ow=wp["ow"],
        location_name=location,
    )
    return f"{msg} Target={location}."


def cancel_goal():
    start_ros_background()
    return _nav_node.cancel_goal()

def valpo_search(query: str, top_k: int = 3):
    """
    Calls the local KB search endpoint and returns structured results.
    Expected response: {"results": [{"chunk_id":..., "text":..., "citation": {...}}, ...]}
    """
    top_k = max(1, min(int(top_k), 20))
    ui.searching()
    try:
        resp = requests.get(
            SEARCH_URL,
            params={"q": query, "k": top_k},
            timeout=(2, 8),  # connect, read
        )
        resp.raise_for_status()

        data = resp.json()
        results = data.get("results", [])

        # Minimal validation / normalization
        normalized = []
        for r in results:
            normalized.append({
                "text": r.get("text", ""),
                "chunk_id": r.get("chunk_id"),
                "citation": r.get("citation", {}),
            })

        return {"query": query, "top_k": top_k, "results": normalized}

    except requests.Timeout:
        return {"query": query, "top_k": top_k, "results": [], "error": "KB search timed out."}
    except requests.RequestException as e:
        return {"query": query, "top_k": top_k, "results": [], "error": f"KB request failed: {e}"}
    except ValueError as e:
        # JSON decode error
        return {"query": query, "top_k": top_k, "results": [], "error": f"Invalid JSON from KB: {e}"}

funcs = {
    "set_goal": set_goal,
    "cancel_goal": cancel_goal,
    "valpo_search": valpo_search,
}


# =========================
# Recorder callbacks (barge-in)
# =========================
def set_recording():
    global is_recording_flag
    try:
        if tts and tts.is_playing():
            if VERBOSE_TTS:
                log("⏹️  TTS stop (barge-in)")
            tts.stop()
        is_recording_flag = True
        if VERBOSE_STT:
            log("🎙️  Recording START")
        ui.listening()
    except Exception:
        log_exc("set_recording")

def unset_recording():
    global is_recording_flag
    is_recording_flag = False
    if VERBOSE_STT:
        log("🎙️  Recording STOP")

# =========================
# Eventing
# =========================
def enqueue_arrival(target):
    ev = {"kind": "arrival", "target": target, "epoch": nav_epoch}
    log(f"📨 Queue arrival event: {target} (epoch {nav_epoch})")
    event_q.put((10, time.time(), ev))

def event_dispatcher():
    while not shutdown_flag:
        try:
            _, _, ev = event_q.get(timeout=0.25)
        except queue.Empty:
            continue

        if ev.get("kind") != "arrival":
            continue

        target = ev.get("target")
        epoch = ev.get("epoch")
        log(f"🛬 Processing arrival event: {target} at epoch {epoch}")

        while (tts and tts.is_playing()) or is_recording_flag:
            time.sleep(0.05)
            if shutdown_flag:
                return

        if epoch != nav_epoch or target != current_target:
            log(f"🗑️  Drop stale arrival event for {target} (epoch mismatch)")
            continue

        # Preserve context: append an event message to shared history
        arrival_msg = (
            f"[EVENT PROMPT] Arrived at {target}."
        )
        with messages_lock:
            messages.append({"role": "system", "name":"event", "content": arrival_msg})

        with llm_lock:
            reply_text, _ = process_by_llm_streaming(messages=messages, tools=tools)

        log(f"🗣️  Spoke arrival ({target}), chars={len(reply_text)}")



# =========================
# OpenAI streaming
# =========================
def stream_llm_events(client, messages, tools=None, model=MODEL_NAME):
    """
    Yield:
      ("text", str)
      ("tools", list_of_calls)  once finish_reason == "tool_calls"
    """
    pending = {}
    finish_reason = None

    log(f"🧠 OpenAI stream start (model={model}) with {len(messages)} msgs")

    resp = client.chat.completions.create(
        model=model,
        messages=messages,
        tools=tools or [],
        tool_choice="auto",
        stream=True,
    )

    for chunk in resp:
        choice = chunk.choices[0]
        delta = getattr(choice, "delta", None)
        if delta is None:
            continue

        # text
        content = getattr(delta, "content", None)
        if content:
            if VERBOSE_TOKENS:
                log(f"🔹 token: {repr(content)}", lvl="TOK")
            yield ("text", content)

        # tool deltas
        deltas = getattr(delta, "tool_calls", None)
        if deltas:
            for tc in deltas:
                idx = tc.index
                entry = pending.setdefault(idx, {"id": None, "name": None, "args": ""})
                if tc.id:
                    entry["id"] = tc.id
                if tc.function and tc.function.name:
                    entry["name"] = tc.function.name
                if tc.function and tc.function.arguments:
                    entry["args"] += tc.function.arguments
                if VERBOSE_TOOLS:
                    log(f"🧩 tool-delta idx={idx} id={entry['id']} name={entry['name']} args+=({len(entry['args'])})")

        if choice.finish_reason:
            finish_reason = choice.finish_reason

    if finish_reason == "tool_calls" and pending:
        ordered = [pending[i] for i in sorted(pending)]
        if VERBOSE_TOOLS:
            log(f"🧩 tool-calls finish with {len(ordered)} call(s)")
        yield ("tools", ordered)

# =========================
# LLM pass (feed-first playback)
# =========================
def process_by_llm_streaming(messages, tools, max_depth=3):
    """
    Stream assistant output. If a pass requests tools, run ALL tool calls,
    append their results, then do another streamed pass. Returns (final_text, messages).
    """
    depth = 0
    try:
        ui.thinking()
        while True:
            if tts.is_playing():
                if VERBOSE_TTS:
                    log("⏹️  TTS stop (new LLM pass)")
                try:
                    tts.stop()
                except Exception:
                    pass

            # Build request with a fresh system message each pass
            system_prompt = generate_system_prompt()
            with messages_lock:
                use_messages = [{"role": "system", "content": system_prompt}] + list(messages)

            log(f"🚿 LLM streaming pass (depth={depth})")

            full_text_parts = []
            assistant_tool_calls = None

            try:
                for kind, payload in stream_llm_events(
                    client,
                    use_messages,
                    tools=tools,
                    model=MODEL_NAME,
                ):
                    if kind == "text":
                        full_text_parts.append(payload)

                    elif kind == "tools":
                        assistant_tool_calls = payload  # list of {"id","name","args"}
                        break

            except Exception:
                log_exc("process_by_llm_streaming: streaming loop")

            # If tools were requested, execute ALL of them, append results, and loop
            if assistant_tool_calls:
                # If we were buffering to avoid preamble, we simply drop it.
                # If you prefer to keep it, set SUPPRESS_PREAMBLE_IF_TOOLS=False.
                tc_payload = [
                    {
                        "id": c["id"],
                        "type": "function",
                        "function": {"name": c["name"], "arguments": c.get("args") or "{}"},
                    }
                    for c in assistant_tool_calls
                ]
                assistant_tc_msg = {"role": "assistant", "content": "", "tool_calls": tc_payload}

                with messages_lock:
                    messages.append(assistant_tc_msg)

                # Execute tools sequentially (simple + predictable). For latency,
                # you can parallelize with ThreadPoolExecutor if your tools are I/O bound.
                for c in assistant_tool_calls:
                    name = c["name"]
                    raw_args = c.get("args") or ""
                    if VERBOSE_TOOLS:
                        log(f"🛠️  running tool: {name} raw_args={raw_args!r}")
                    try:
                        args = json.loads(raw_args) if raw_args else {}
                    except Exception:
                        if VERBOSE_TOOLS:
                            log("⚠️  tool args JSON parse failed; using empty dict", lvl="WARN")
                        args = {}

                    func = funcs.get(name)
                    try:
                        result = func(**args) if func else f"Unknown tool: {name}"
                    except Exception as e:
                        result = f"Tool '{name}' raised: {e}"

                    if VERBOSE_TOOLS:
                        log(f"🛠️  tool result: {result}")

                    with messages_lock:
                        messages.append({"role": "tool", "tool_call_id": c["id"], "content": str(result)})

                depth += 1
                if depth > max_depth:
                    if VERBOSE_TOOLS:
                        log("🧯 Reached max tool depth; stopping.", lvl="WARN")
                    # Best-effort: emit whatever text we buffered this pass (usually preamble)
                    assistant_text = "".join(full_text_parts)
                    if assistant_text.strip():
                        with messages_lock:
                            messages.append({"role": "assistant", "content": assistant_text})
                    return assistant_text, messages

                # Continue to next pass (loop)
                continue

            # No tools requested on this pass: finalize text & speak it (if we buffered)
            assistant_text = "".join(full_text_parts)
            log(f"💬 assistant text len={len(assistant_text)} (depth={depth})")

            if assistant_text.strip():
                tts.say(assistant_text, block=False, interrupt=True)
                with messages_lock:
                    messages.append({"role": "assistant", "content": assistant_text})

            return assistant_text, messages

    finally:
        pass

# =========================
# Graceful shutdown
# =========================
def handle_sigint(sig, frame):
    global shutdown_flag
    shutdown_flag = True
    log("🔻 SIGINT received, shutting down…")
    try:
        if recorder:
            recorder.terminate()
    except Exception:
        pass
    try:
        if tts and tts.is_playing():
            tts.stop()
    except Exception:
        pass
    try:
        if _nav_exec and _nav_node:
            _nav_exec.remove_node(_nav_node)
        if _nav_exec:
            _nav_exec.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, handle_sigint)

def play_wav(path: str, device: str | None = ALSA_DEVICE):
    """Fire-and-forget WAV playback using ALSA 'aplay'."""
    if not os.path.isfile(path):
        return
    cmd = ["aplay", "-q"]
    if device:
        cmd += ["-D", device]
    cmd.append(path)
    # Non-blocking so STT can start immediately
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

ready_chime_path = "assets/audio/ui-wakesound.wav"
def on_wakeword():
    if tts.is_playing():
        tts.stop()  # stop ONLY on “sage”
    if ui._last_phase != "listening":
        ui.listening()
        threading.Thread(target=play_wav, args=(ready_chime_path,), daemon=True).start()
# =========================
# Main
# =========================
if __name__ == "__main__":
    # ---- TTS init (engine selection) ----
    try:
        tts = PiperTTS(
            model_path=["assets/models/piper/en_US-amy-medium.onnx",\
			"assets/models/piper/ljspeech.onnx"][0],
            aplay_device=ALSA_DEVICE,
            buffer_time_us=40000,
            period_size=256,
        )
        tts.on_start = lambda: ui.speaking()
        tts.on_end = lambda: ui.idle()
        if VERBOSE_TTS:
            log("✅ PiperTTS ready.")
    except Exception:
        log("⚠️ PiperTTS init failed.", lvl="ERROR")
        raise

    # ---- OpenAI client ----
    client = openai.OpenAI(api_key=read_openai_key("api_keys/api_keys.json"))

    # ---- Background workers ----
    threading.Thread(target=event_dispatcher, daemon=True).start()
    start_ros_background()
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    log(f"Using device: {device}")
    # ---- STT init ----
    USE_WAKEWORD = True

    recorder_kwargs = dict(
        language="en",
        spinner=True,
        model=["tiny.en", "small.en"][0],
        device=device,
        on_recording_start=set_recording,
        on_recording_stop=unset_recording,
        no_log_file=True,
    )

    if USE_WAKEWORD:
        recorder_kwargs.update(
            wakeword_backend=["pvporcupine", "openwakeword"][1],
            openwakeword_model_paths="assets/models/wakeword/sage_wakeword_2.onnx",
            openwakeword_inference_framework="onnx",
            wake_words_sensitivity=0.3,      # lower makes it more sensitive
            wake_word_buffer_duration=0.2,  # 0.75-1s to keep "sage" out of transcript
            on_wakeword_detected=on_wakeword,
            wake_words="sage"  # You need to leave this here for openwakeword to work (even though it's not used)
        )

    recorder = AudioToTextRecorder(**recorder_kwargs)

    log("Ready to guide tours with streaming TTS…")

    # ---- Interaction loop ----
    while not shutdown_flag:
        try:
            user_input = recorder.text()
        except KeyboardInterrupt:
            handle_sigint(None, None)
            break
        except Exception:
            log_exc("recorder.text()")
            continue

        if not user_input:
            continue

        if VERBOSE_STT:
            log(f"🗣️  User input: {user_input}")
        # Add user message & trim history
        with messages_lock:
            messages.append({"role": "user", "content": user_input})
            if len(messages) > 12:
                messages[:] = messages[-12:]
            log(f"🧾 messages now: {len(messages)}")

        # Stream model → TTS (tools allowed)
        with llm_lock:
            ai_reply, messages = process_by_llm_streaming(messages=messages, tools=tools)
            log(f"🤖 AI: {ai_reply}")
