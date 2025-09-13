# llm4.py — Streaming OpenAI -> RealtimeTTS with feed-first playback, working tool calls, shared-context events, and robust logging

import os
import sys
import signal
import threading
import time
import queue
import requests
import openai
import traceback
import re

# Use your utils if available
try:
    from utils import read_openai_key, json  # your helpers
except Exception:
    import json as json
    def read_openai_key(_):
        key = os.getenv("OPENAI_API_KEY")
        if not key:
            raise RuntimeError("OPENAI_API_KEY not set and utils.read_openai_key unavailable.")
        return key

from RealtimeSTT import AudioToTextRecorder
from RealtimeTTS import TextToAudioStream, SystemEngine, GTTSEngine, CoquiEngine

# =========================
# Config / Flags
# =========================
VERBOSE = True                 # top-level logs
VERBOSE_TOKENS = False         # per-token logging
VERBOSE_TOOLS = True           # tool delta/args/results
VERBOSE_ROBOT = False
VERBOSE_TTS = True
VERBOSE_STT = True

TTS_MODE = os.getenv("TTS_MODE", "balanced")
MODEL_NAME = "gpt-4o"
POLLING_INTERVAL = 1

SINGLE_STOP_MODE = True

# If a new LLM pass begins while audio is still playing, stop the old audio first
FLUSH_AUDIO_ON_NEW_PASS = True
SUPPRESS_PREAMBLE_IF_TOOLS = True
# =========================
# UI & backend endpoints
# =========================
ROBOT_BACKEND_URL = "http://127.0.0.1:8002"
llm_thinking = ROBOT_BACKEND_URL + "/llm_thinking/"
llm_recording = ROBOT_BACKEND_URL + "/llm_recording/"


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

_last_status = None
_last_arrived_at = None
_last_destinations = None

client = None
stream = None
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
            "name": "update_robot_route",
            "description": "Update the robot's route with one or more destinations.",
            "parameters": {
                "type": "object",
                "properties": {
                    "destinations": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "List of EXACT short building names (e.g., ['GEMC'])."
                    },
                    "clear_existing": {
                        "type": "boolean",
                        "description": "If true, replace any existing route."
                    }
                },
                "required": ["destinations"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_robot_route",
            "description": "Cancel the robot's route or a specific destination.",
            "parameters": {
                "type": "object",
                "properties": {
                    "destination": {
                        "type": "string",
                        "description": "Optional: cancel only this specific destination."
                    }
                }
            }
        }
    }
]

def generate_system_prompt():
    """Generate system prompt with current robot status"""
    robot_info = get_robot_status()

    base_prompt = """You are a tour guide AI robot called Sage at Valparaiso University's College of Engineering.

Your job is to guide visitors through the engineering facilities and campus based on their voice commands. You have detailed knowledge of the main engineering facilities and can provide informative tours.

IMPORTANT: When using tools to control yourself, you MUST use the EXACT building names listed below. You are responsible for understanding user requests and mapping them to these exact identifiers.

TOUR FLOW POLICY (Single-Stop Mode by default)
- Never schedule more than one destination at a time.
- Acknowledge full multi-stop requests, but only send the FIRST stop now.
- Call update_robot_route with exactly one destination and clear_existing=true.
- On arrival, enter Talk Phase:
  1) Deliver a concise intro (10-15 seconds).
  2) Ask at least one preference question (e.g., quick overview, deep dive, or hands-on?).
  3) Offer options: continue here, pick the next stop, or end the tour.
- Only after the user confirms, send the next single destination.
- If the user goes silent after arrival, politely prompt once, then wait.
- If user says “do all,” still proceed one stop at a time unless explicitly told to batch.

TOOL USAGE RULE
- update_robot_route: always 1 destination, clear_existing=true.
- cancel_robot_route: use to stop current movement before changing plans.

MAIN ENGINEERING FACILITIES (with detailed knowledge and exact building names):
- GEMC: The main engineering building housing the College of Engineering. This is the heart of engineering education at Valpo. (Aliases: "main engineering building", "Gellersen", "engineering building", "Gellersen Engineering", "GEMC")
- Fites: A 13,470 sq ft addition opened in 2011 that adds undergraduate labs for senior design, energy/materials research, and manufacturing (Aliases: "Senior Design Space", "engineering innovation center", "Donald V. Fites")
- SERF: Houses one of just five solar furnaces in the U.S. — the only one at an undergraduate institution. Features a 20x20 ft heliostat feeding a concentrator with 303 mirrors and a louver system. The reactor routinely exceeds 3,000°F, enabling solar-fuels research. Located just east of GEMC. (Aliases: "solar furnace", "solar energy facility", "Markiewicz facility", "solar research")
- Hesse: Drop-in academic support hub for engineering students in GEM 121, providing free tutoring and study help. (Aliases: "tutoring center", "learning center", "Hesse Learning Center", "academic support")
- iHub: Open, cross-disciplinary maker space with tools from 3D printers and laser cutters/etchers to low-tech craft supplies, used for prototyping and student projects. (Aliases: "makerspace", "innovation hub", "3D printing lab", "prototyping lab", "maker space")

OTHER CAMPUS LOCATIONS (can guide to but limited details - use exact building names):
- Library: Main library with late-night hours and 24/7 Community Room (Aliases: "Christopher Center", "main library", "university library")
- Chapel: Iconic campus landmark dedicated 1959, seats ~2,000, added to National Register of Historic Places in 2021 (Aliases: "Chapel of the Resurrection", "Brandt Campanile", "campus chapel", "main chapel")
- Science: Center for the Sciences (2017) with modern wet-labs for Chemistry, Biochemistry & Biology (Aliases: "science complex", "CFS", "NSC", "Center for the Sciences")
- Meteorology: Home base for meteorology program with weather observation deck and radiosonde launcher (Aliases: "Kallay-Christopher Hall", "weather station", "meteorology building")
- ARC: 5,000-seat arena for basketball/volleyball, includes gym, pool, indoor track (Aliases: "Athletics-Recreation Center", "gym", "basketball arena", "recreation center")
- Museum: University art museum with 5,000+ piece collection (Aliases: "Brauer Museum", "art museum", "university museum")
- Weseman: previously law school, now for psychology and sociology classes (Aliases: "Weseman Hall", "Weseman", "law school", "psychology", "sociology")
- Lebien: for Nursing classes (Aliases: "Lebien Hall", "Lebien", "Nursing")
- Lankenau: Residential hall for freshman. Closed since 2023.
- Alumni: Residential hall for freshman.
- Brandt: Residential hall for freshman.
- Founders: refectory where students eat.
- Kretzmann: administrative offices.
- Nielsen: institute for Sex Education.
- Ateufack: School of AI.

ROOM NAME MAPPING RULES:
1. ALWAYS use the exact building names (GEMC, Fites, SERF, Hesse, iHub, Library, Chapel, Science, Meteorology, ARC, Museum, Weseman, Lebien, Lankenau, Alumni, Brandt, Founders, Kretzmann, Nielsen, Ateufack)
2. When users say things like "take me to the main engineering building", you understand they mean GEMC
3. When users say "show me the solar furnace", you understand they mean SERF
4. When users say "let's go to the makerspace", you understand they mean iHub
5. Convert ALL natural language requests to exact building names before calling tools

TOUR FLOW EXAMPLES:
- User: "Give me a tour of the engineering facilities"
- AI Response: "I'll start by taking you to GEMC, the main engineering building. This is the heart of engineering education at Valpo. Let me navigate there first."
- AI Action: update_robot_route(["GEMC"])
- After Arrival: "Welcome to GEMC! This building houses the College of Engineering and is where students learn mechanical, electrical, and computer engineering. Would you like to see the solar furnace next, or do you have questions about GEMC?"

- User: "Take me on a campus tour"
- AI Response: "I'll start with the iconic Chapel of the Resurrection, then we can explore other highlights. Let me take you there first."
- AI Action: update_robot_route(["Chapel"])
- After Arrival: "Here we are at the Chapel of the Resurrection, dedicated in 1959 and added to the National Register of Historic Places in 2021. It seats about 2,000 people. Would you like to see the library next, or explore another area?"

You have tools available to control yourself:
- update_robot_route: Set new destinations or add to existing route
- cancel_robot_route: Cancel specific or all destinations

When giving tours, focus on the engineering facilities where you have detailed knowledge. For other locations, you can guide visitors there but provide basic information only. Always confirm what you're doing e.g. "Alright, let's go to Fites!". Be enthusiastic about Valparaiso University's engineering program and facilities."""

    if robot_info:
        robot_data = robot_info["data"]
        position = robot_data.get("position", {"x": 0, "y": 0, "z": 0})
        destinations = robot_data.get("destinations", [])
        mode = robot_data.get("mode", "unknown")
        status = robot_data.get("status", "unknown")

        status_section = f"""

CURRENT ROBOT STATUS:
- Position: ({position['x']:.1f}, {position['y']:.1f}, {position['z']:.1f})
- Current destinations: {destinations if destinations else 'None'}
- Mode: {mode}
- Status: {status}

Use this information to make informed decisions about routes and respond appropriately to the user's requests. DO NOT USE SPECIAL CHARACTERS LIKE *, #, OR _ IN YOUR RESPONSE. Just plain text. Keep your responses short and concise."""
        return base_prompt + status_section
    else:
        return base_prompt + "\n\nNote: Robot status unavailable - robot may not be connected."

# =========================
# Robot backend
# =========================
def get_robot_status():
    try:
        r = requests.get(f"{ROBOT_BACKEND_URL}/robots", timeout=5)
        if VERBOSE_ROBOT:
            log(f"📡 GET /robots -> {r.status_code}", lvl="NET")
        if r.status_code == 200:
            data = r.json()
            if VERBOSE_ROBOT:
                log(f"📡 payload: {data}", lvl="NET")
            if data.get("connected_robots", 0) > 0:
                robot_id = list(data.get("robot_states", {}).keys())[0]
                return {"robot_id": robot_id, "data": data["robot_states"][robot_id]}
        return None
    except Exception:
        log_exc("get_robot_status")
        return None

def update_robot_route(destinations, clear_existing=False):
    global nav_epoch, current_target
    if SINGLE_STOP_MODE and len(destinations) > 1:
        if VERBOSE_TOOLS:
            log(f"🔧 SINGLE_STOP_MODE trim {destinations} -> {destinations[:1]}")
        destinations = destinations[:1]
        clear_existing = True

    if VERBOSE_TOOLS:
        log(f"🔧 update_robot_route called with: destinations={destinations}, clear_existing={clear_existing}")

    try:
        robot_info = get_robot_status()
        if not robot_info:
            return "Error: No robot connected"

        robot_id = robot_info["robot_id"]
        if VERBOSE_ROBOT:
            log(f"🤖 Robot ID: {robot_id}", lvl="NET")

        current_target = destinations[0] if destinations else None
        nav_epoch += 1

        payload = {"destinations": destinations, "clear_existing": clear_existing}
        if VERBOSE_ROBOT:
            log(f"📤 POST /robots/{robot_id}/route {payload}", lvl="NET")
        r = requests.post(f"{ROBOT_BACKEND_URL}/robots/{robot_id}/route", json=payload, timeout=5)
        if VERBOSE_ROBOT:
            log(f"📥 Response status: {r.status_code}", lvl="NET")

        if r.status_code == 200:
            action = "Updated" if not clear_existing else "Set new"
            msg = f"{action} robot route to: {', '.join(destinations)}"
            if VERBOSE_TOOLS:
                log(f"✅ {msg}")
            return msg
        err = f"Failed to update route: {r.status_code}"
        if VERBOSE_TOOLS:
            log(f"❌ {err}")
        return err
    except Exception:
        log_exc("update_robot_route")
        return "Error updating route"

def cancel_robot_route(destination=None):
    if VERBOSE_TOOLS:
        log(f"🔧 cancel_robot_route called with: destination={destination}")
    try:
        robot_info = get_robot_status()
        if not robot_info:
            return "Error: No robot connected"

        robot_id = robot_info["robot_id"]
        if destination:
            r = requests.delete(f"{ROBOT_BACKEND_URL}/robots/{robot_id}/route/{destination}", timeout=5)
        else:
            r = requests.delete(f"{ROBOT_BACKEND_URL}/robots/{robot_id}/route", timeout=5)
        if VERBOSE_ROBOT:
            log(f"📥 Cancel response status: {r.status_code}", lvl="NET")

        if r.status_code == 200:
            return "Cancelled route" if not destination else f"Cancelled destination: {destination}"
        return f"Failed to cancel route: {r.status_code}"
    except Exception:
        log_exc("cancel_robot_route")
        return "Error cancelling route"

funcs = {
    "update_robot_route": update_robot_route,
    "cancel_robot_route": cancel_robot_route,
}

# =========================
# Recorder callbacks (barge-in)
# =========================
def set_recording():
    global is_recording_flag
    try:
        if stream and stream.is_playing():
            if VERBOSE_TTS:
                log("⏹️  TTS stop (barge-in)")
            stream.stop()
        is_recording_flag = True
        if VERBOSE_STT:
            log("🎙️  Recording START")
        try:
            requests.get(llm_thinking + "unset", timeout=0.25)
            requests.get(llm_recording + "set", timeout=0.25)
        except Exception:
            pass
    except Exception:
        log_exc("set_recording")

def unset_recording():
    global is_recording_flag
    is_recording_flag = False
    if VERBOSE_STT:
        log("🎙️  Recording STOP")
    try:
        requests.get(llm_recording + "unset", timeout=0.25)
    except Exception:
        pass

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

        while (stream and stream.is_playing()) or is_recording_flag:
            time.sleep(0.05)
            if shutdown_flag:
                return

        if epoch != nav_epoch or target != current_target:
            log(f"🗑️  Drop stale arrival event for {target} (epoch mismatch)")
            continue

        # Preserve context: append an event message to shared history
        arrival_msg = (
            f"[EVENT] Arrived at {target}. Give a 12 sentence intro and EXACTLY ONE short follow-up question."
        )
        with messages_lock:
            messages.append({"role": "system", "content": arrival_msg})

        with llm_lock:
            reply_text, _ = process_by_llm_streaming(messages=messages, tools=tools)

        log(f"🗣️  Spoke arrival ({target}), chars={len(reply_text)}")

def robot_watcher():
    global _last_status, _last_arrived_at, _last_destinations
    while not shutdown_flag:
        info = get_robot_status()
        if info:
            data = info["data"]
            status = data.get("status")
            arrived_at = data.get("arrived_at")
            destinations = data.get("destinations")

            if arrived_at and arrived_at != _last_arrived_at:
                _last_arrived_at = arrived_at
                target = data.get("current_target") or (destinations[0] if destinations else arrived_at)
                if target:
                    enqueue_arrival(target)

            _last_status = status
            _last_destinations = destinations

        time.sleep(POLLING_INTERVAL)

# =========================
# TTS helpers
# =========================
def fast_split(text: str):
    # Lightweight, no-NLTK tokenizer for low-latency sentence fragments
    return re.findall(r'[^.!?…]+[.!?…]?', text)



TTS_PRESETS = {
    # ~350–500 ms initial voice, ok for interjections
    "fast": dict(
        fast_sentence_fragment=True,
        fast_sentence_fragment_allsentences=False,
        fast_sentence_fragment_allsentences_multiple=False,
        minimum_first_fragment_length=20,
        force_first_fragment_after_words=7,
        buffer_threshold_seconds=0.35,
        sentence_fragment_delimiters=".?!…",
        context_size=18,
        context_size_look_overhead=24,
    ),
    # ~600–900 ms, natural for most conversations
    "balanced": dict(
        fast_sentence_fragment=True,               # keep fast startup only for first sentence
        fast_sentence_fragment_allsentences=False,
        fast_sentence_fragment_allsentences_multiple=False,
        minimum_first_fragment_length=28,
        minimum_sentence_length=30,
        force_first_fragment_after_words=10,
        buffer_threshold_seconds=0.65,
        sentence_fragment_delimiters=".?!…",
        context_size=24,
        context_size_look_overhead=36,
    ),
    # prioritize prosody over latency
    "natural": dict(
        fast_sentence_fragment=False,
        minimum_sentence_length=40,
        buffer_threshold_seconds=0.8,
        sentence_fragment_delimiters=".?!…",
        context_size=32,
        context_size_look_overhead=48,
    ),
}

def tts_feed(text_or_iter, *, low_latency=True):
    just_started = False
    if not stream.is_playing():
        if FLUSH_AUDIO_ON_NEW_PASS:
            try:
                stream.stop()
            except Exception:
                pass
        just_started = True

    stream.feed(text_or_iter)

    if just_started:
        if VERBOSE_TTS:
            log("▶️  TTS play_async start")
        kwargs = TTS_PRESETS[TTS_MODE]
        stream.play_async(
            # if you want your custom tokenizer, keep it;
            # otherwise let RealtimeTTS handle sentence splitting
            tokenize_sentences=fast_split if TTS_MODE != "natural" else None,
            **kwargs,
        )
    else:
        if VERBOSE_TTS:
            log("↪️  TTS already playing; appended to stream")
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
        # thinking UI on
        try:
            requests.get(llm_thinking + "set", timeout=0.25)
        except Exception:
            pass

        while True:
            # Stop old audio if desired
            if FLUSH_AUDIO_ON_NEW_PASS and stream.is_playing():
                if VERBOSE_TTS:
                    log("⏹️  TTS stop (new LLM pass)")
                try:
                    stream.stop()
                except Exception:
                    pass

            # Build request with a fresh system message each pass
            system_prompt = generate_system_prompt()
            with messages_lock:
                use_messages = [{"role": "system", "content": system_prompt}] + list(messages)

            log(f"🚿 LLM streaming pass (depth={depth})")

            full_text_parts = []
            assistant_tool_calls = None
            playback_started = False

            try:
                for kind, payload in stream_llm_events(
                    client,
                    use_messages,
                    tools=tools,
                    model=MODEL_NAME,
                ):
                    if kind == "text":
                        # Optionally buffer text until we know there are no tools
                        if SUPPRESS_PREAMBLE_IF_TOOLS:
                            full_text_parts.append(payload)
                        else:
                            if not playback_started:
                                tts_feed(payload, low_latency=True)
                                playback_started = True
                            else:
                                stream.feed(payload)
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
                # Start playback now if we held back earlier
                if SUPPRESS_PREAMBLE_IF_TOOLS and assistant_text:
                    tts_feed(assistant_text, low_latency=True)
                with messages_lock:
                    messages.append({"role": "assistant", "content": assistant_text})

            return assistant_text, messages

    finally:
        # thinking UI off
        try:
            requests.get(llm_thinking + "unset", timeout=0.25)
        except Exception:
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
        if stream and stream.is_playing():
            stream.stop()
    except Exception:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, handle_sigint)

def on_wakeword():
    if stream.is_playing():
        stream.stop()  # stop ONLY on “sage”
# =========================
# Main
# =========================
if __name__ == "__main__":
    # ---- TTS init (engine selection) ----
    try:
        engine = GTTSEngine()
        # engine = SystemEngine()
        # engine = CoquiEngine()
        #engine.set_voice("Claribel Dervla")
    except Exception:
        log("⚠️  Coqui/GTTSEngine init failed, falling back to SystemEngine", lvl="WARN")
        engine = SystemEngine()
    stream = TextToAudioStream(engine)

    # ---- OpenAI client ----
    client = openai.OpenAI(api_key=read_openai_key("api_keys.json"))

    # ---- Background workers ----
    threading.Thread(target=event_dispatcher, daemon=True).start()
    threading.Thread(target=robot_watcher, daemon=True).start()

    # ---- STT init ----
    recorder = AudioToTextRecorder(
        language="en",
        spinner=True,
        model=["large-v2", "small.en"][1],
        device="cuda",
        on_recording_start=set_recording,
        on_recording_stop=unset_recording,
        # wakeword_backend=["pvporcupine", "openwakeword"][1],
        # openwakeword_model_paths="sage.onnx",
        # openwakeword_inference_framework="onnx",
        # wake_words_sensitivity=0.5,      # try 0.7–0.85 in a school
        # wake_word_buffer_duration=0.75,   # 0.75–1.0s keeps “sage” out of transcript,
        # on_wakeword_detected=on_wakeword,
        # wake_words="sage" # You need to leave this here for openwakeword to work (even though it's not used)
    )

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

        # Reset if previous convo marked done
        if done:
            with messages_lock:
                messages.clear()
            done = False

        # Add user message & trim history
        with messages_lock:
            messages.append({"role": "user", "content": user_input})
            if len(messages) > 12:
                messages[:] = messages[-12:]
            log(f"🧾 messages now: {len(messages)}")

        # Stream model → TTS (tools allowed)
        with llm_lock:
            ai_reply, messages = process_by_llm_streaming(messages=messages, tools=tools)

        if "DONE" in ai_reply:
            ai_reply = ai_reply.replace("DONE", "")
            done = True

        print(f"assistant: {ai_reply if ai_reply.strip() else '[no content]'}", flush=True)
