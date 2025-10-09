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
import torch

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
from RealtimeTTS import TextToAudioStream, SystemEngine, GTTSEngine, CoquiEngine, PiperEngine, PiperVoice
# ---- Nav2 async bridge (non-blocking) ----
import threading, rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class Nav2AsyncBridge(Node):
    def __init__(self):
        super().__init__('nav2_llm_bridge_async')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.status = "idle"          # idle | navigating | arrived | failed | cancelling
        self.current_target = None
        self.last_feedback = {}

    def set_goal(self, *, frame_id, x, y, ox, oy, oz, ow, location_name):
        self.current_target = location_name
        self.status = "navigating"

        # Preempt old goal if any
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        if not self.client.wait_for_server(timeout_sec=0.5):
            self.status = "failed"
            return "Nav2 action server not available."

        pose = PoseStamped()
        pose.header.frame_id = frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = float(ox)
        pose.pose.orientation.y = float(oy)
        pose.pose.orientation.z = float(oz)
        pose.pose.orientation.w = float(ow)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        def _feedback_cb(fb):
            try:
                f = fb.feedback
                # available: current_pose, navigation_time, number_of_recoveries, distance_remaining
                self.last_feedback = {
                    "distance_remaining": getattr(f, "distance_remaining", 0.0),
                    "recoveries": getattr(f, "number_of_recoveries", 0),
                }
            except Exception:
                pass

        send_fut = self.client.send_goal_async(goal, feedback_callback=_feedback_cb)

        def _after_send(fut):
            try:
                gh = fut.result()
                if not gh or not gh.accepted:
                    self.status = "failed"
                    return
                self.goal_handle = gh
                res_fut = gh.get_result_async()

                def _after_result(rf):
                    try:
                        res = rf.result()                    # GetResult.Response
                        st = res.status                      # int (GoalStatus.*)
                        if st == GoalStatus.STATUS_SUCCEEDED:
                            self.status = "arrived"
                            try:
                                enqueue_arrival(self.current_target)
                            except Exception:
                                pass
                        elif st == GoalStatus.STATUS_CANCELED:
                            self.status = "idle"
                        else:
                            # ABORTED or others
                            self.status = "failed"
                    except Exception:
                        self.status = "failed"

                res_fut.add_done_callback(_after_result)
            except Exception:
                self.status = "failed"

        send_fut.add_done_callback(_after_send)
        return "Goal accepted (navigating)."

    def cancel_goal(self):
        if not self.goal_handle:
            return "No active goal."
        self.status = "cancelling"
        fut = self.goal_handle.cancel_goal_async()
        def _after_cancel(_):
            self.status = "idle"
            self.goal_handle = None
        fut.add_done_callback(_after_cancel)
        return "Cancel requested."

# Spin ROS in a background thread once at startup
_nav_exec = None
_nav_node = None
def start_ros_background():
    global _nav_exec, _nav_node
    if _nav_node: 
        return
    if not rclpy.ok():
        rclpy.init()
    _nav_node = Nav2AsyncBridge()
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
# -------------------------
# Waypoints (PoseStamped-like)
# -------------------------

WAYPOINTS = {
    "Fites": {
        "frame_id": "map",
        "x": 5.020315170288086, "y": 0.5106609463691711,
        "ox": 0.0, "oy": 0.0, "oz": 0.6982647334138825, "ow": 0.7158396203553137
    },
    "Gelly_Delly": {
        "frame_id": "map",
        "x": 22.26424217224121, "y": -3.0069425106048584,
        "ox": 0.0, "oy": 0.0, "oz": 0.058206536467100216, "ow": 0.9983045623017578
    },
    "GE_100": {
        "frame_id": "map",
        "x": 43.25941467285156, "y": -2.9542574882507324,
        "ox": 0.0, "oy": 0.0, "oz": 0.017809096604719063, "ow": 0.9998414054629483
    },
    "ECE_LAB_1": {
        "frame_id": "map",
        "x": 55.77067565917969, "y": 29.38323402404785,
        "ox": 0.0, "oy": 0.0, "oz": -0.7288416827551458, "ow": 0.6846822631547039
    },
    "ECE_LAB_2": {
        "frame_id": "map",
        "x": 42.942630767822266, "y": 31.10202980041504,
        "ox": 0.0, "oy": 0.0, "oz": -0.982344960779826, "ow": 0.1870785343925971
    },
    "HESSE_CENTER": {
        "frame_id": "map",
        "x": 83.51174926757812, "y": -6.202152252197266,
        "ox": 0.0, "oy": 0.0, "oz": 0.010512876726070833, "ow": 0.9999447381845371
    },
    "3D_PRINTING_LAB": {
        "frame_id": "map",
        "x": 50.47529983520508, "y": -14.694130897521973,
        "ox": 0.0, "oy": 0.0, "oz": 0.009561396317804793, "ow": 0.9999542888054703
    },
    "VALPO_ROBOTICS": {
        "frame_id": "map",
        "x": 46.69096374511719, "y": -32.61865234375,
        "ox": 0.0, "oy": 0.0, "oz": 0.9991932193368056, "ow": 0.040161056153322494
    },
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

TTS_MODE = os.getenv("TTS_MODE", ["balanced", "fast", "natural"][2]) # balanced | fast | natural
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
    waypoints_bulleted = "\n".join(f"- {name}" for name in waypoint_names)

    base_prompt = f"""You are a tour guide robot named Sage at Valparaiso University's College of Engineering.

Your job is to guide visitors by driving to named campus locations and engaging them with short, helpful dialogue.

IMPORTANT:
- Always use the exact waypoint names listed below when calling tools.
- Convert user requests to one exact location before calling tools.
- Keep responses in plain English with no special characters.

WAYPOINTS YOU CAN DRIVE TO:
{waypoints_bulleted}

TOOLS YOU CAN USE:
- set_goal(location): Starts driving to the given waypoint. Non-blocking — you can talk while moving.
- cancel_goal(): Cancels the current goal.

WHILE MOVING:
- You may speak to the user.
- Keep speech short while navigating.
- Do not schedule a new goal unless the user changes their mind (then cancel first).
- If the user asks “where are you?” — use the status section below to answer.

ON ARRIVAL:
- Deliver a short 10-15 second intro.
- Ask exactly one short follow-up question.
- Wait for a reply before driving anywhere else.

CONVERSATION STYLE:
- Be helpful, warm, and concise.
- Do not use *, #, or _ characters in your responses.
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

funcs = {
    "set_goal": set_goal,
    "cancel_goal": cancel_goal,
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

def on_wakeword():
    if stream.is_playing():
        stream.stop()  # stop ONLY on “sage”
# =========================
# Main
# =========================
if __name__ == "__main__":
    # ---- TTS init (engine selection) ----
    try:
        engine = PiperEngine(voice=PiperVoice("./en_US-amy-medium.onnx"), piper_path="/home/agi/Desktop/SAGE_ROBOT/.venv/bin/piper")
        # engine = GTTSEngine()
        # engine = SystemEngine()
        # engine = CoquiEngine()
        #engine.set_voice("Claribel Dervla")
    except Exception:
        log("⚠️  Coqui/GTTSEngine init failed, falling back to SystemEngine", lvl="WARN")
        engine = SystemEngine()
    stream = TextToAudioStream(engine, frames_per_buffer=1024)

    # ---- OpenAI client ----
    client = openai.OpenAI(api_key=read_openai_key("api_keys.json"))

    # ---- Background workers ----
    threading.Thread(target=event_dispatcher, daemon=True).start()
    start_ros_background()
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    log(f"Using device: {device}")
    # ---- STT init ----
    USE_WAKEWORD = False

    recorder_kwargs = dict(
        language="en",
        spinner=True,
        model=["tiny.en", "small.en"][0],
        device="cpu",
        on_recording_start=set_recording,
        on_recording_stop=unset_recording,
    )

    if USE_WAKEWORD:
        recorder_kwargs.update(
            wakeword_backend=["pvporcupine", "openwakeword"][1],
            openwakeword_model_paths="sage.onnx",
            openwakeword_inference_framework="onnx",
            wake_words_sensitivity=0.5,      # try 0.7–0.85 in a school
            wake_word_buffer_duration=0.75,  # 0.75–1.0s keeps “sage” out of transcript,
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

