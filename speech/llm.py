
import openai
import requests
import threading
import time
import queue

from RealtimeSTT import AudioToTextRecorder
from RealtimeTTS import TextToAudioStream, SystemEngine, GTTSEngine, CoquiEngine
from utils import read_openai_key, json

# Robot tour backend configuration
ROBOT_BACKEND_URL = 'http://127.0.0.1:8002'
# UI endpoints
llm_thinking = ROBOT_BACKEND_URL + '/llm_thinking/'
llm_recording = ROBOT_BACKEND_URL + '/llm_recording/'

MODEL_NAME = "gpt-4o"
MAX_TOOL_CALLS = 5
# -----------------------------
# Global state & concurrency
# -----------------------------
SINGLE_STOP_MODE = True

messages = []
ai_reply = ""
done = False

# Event system
event_q = queue.PriorityQueue()  # (priority, timestamp, event_dict)

# Locks / flags
messages_lock = threading.Lock()
llm_lock = threading.Lock()
is_recording_flag = False

# Navigation epoch & current target for staleness checks
nav_epoch = 0
current_target = None

# Last-seen robot status for edge detection
_last_status = None
_last_arrived_at = None
_last_destinations = None


# -----------------------------
# LLM Tools: schema
# -----------------------------
tools = [
    {
        "type": "function",
        "function": {
            "name": "update_route",
            "description": "Updates the route with new destinations. Can clear existing destinations or add to them. IMPORTANT: Use EXACT system names like 'GEMC', 'Fites', 'SERF', 'Hesse', 'iHub', 'Library', 'Chapel', 'Science', 'Meteorology', 'ARC', 'Museum', 'Weseman', 'Lebien', 'Lankenau', 'Alumni', 'Brandt', 'Founders', 'Kretzmann', 'Nielsen', 'Ateufack'.",
            "parameters": {
                "type": "object",
                "properties": {
                    "destinations": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "List of EXACT system names to visit (e.g., ['GEMC', 'Fites', 'SERF']) - use the short identifiers, not full names"
                    },
                    "clear_existing": {
                        "type": "boolean",
                        "description": "Whether to clear existing destinations before adding new ones"
                    }
                },
                "required": ["destinations"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_route",
            "description": "Cancels routes. Can cancel a specific destination or all destinations. Use EXACT system names if cancelling a specific destination.",
            "parameters": {
                "type": "object",
                "properties": {
                    "destination": {
                        "type": "string",
                        "description": "Specific destination to cancel (use EXACT system name like 'GEMC', 'Fites', 'SERF'), or leave empty to cancel all"
                    }
                },
                "required": []
            }
        }
    }
]


# -----------------------------
# Robot status helpers
# -----------------------------
def get_robot_status():
    """Get current robot status from FastAPI backend"""
    try:
        response = requests.get(f"{ROBOT_BACKEND_URL}/robots", timeout=5)
        if response.status_code == 200:
            data = response.json()
            if data.get("connected_robots") > 0:
                robot_id = list(data.get("robot_states", {}).keys())[0]
                return {"robot_id": robot_id, "data": data["robot_states"][robot_id]}
        return None
    except Exception as e:
        print(f"Error getting robot status: {e}")
        return None


# -----------------------------
# Robot control functions
# -----------------------------
def update_route(destinations, clear_existing=False):
    """Update robot route with new destinations (single-stop enforced)"""
    global nav_epoch, current_target
    print(f"🔧 update_route called with: destinations={destinations}, clear_existing={clear_existing}")
    try:
        # Hard guard: Single-Stop Mode
        if SINGLE_STOP_MODE and len(destinations) > 1:
            destinations = destinations[:1]
            clear_existing = True

        robot_info = get_robot_status()
        print(f"📡 Robot status response: {robot_info}")

        if not robot_info:
            print("❌ No robot connected")
            return "Error: No robot connected"

        robot_id = robot_info["robot_id"]
        print(f"🤖 Robot ID: {robot_id}")

        if not robot_id:
            print("❌ Could not identify robot")
            return "Error: Could not identify robot"

        # Track nav epoch + current target for staleness checks
        current_target = destinations[0] if destinations else None
        nav_epoch += 1

        route_data = {
            "destinations": destinations,
            "clear_existing": clear_existing
        }
        print(f"📤 Sending route update: {route_data}")

        response = requests.post(
            f"{ROBOT_BACKEND_URL}/robots/{robot_id}/route",
            json=route_data,
            timeout=5
        )

        print(f"📥 Response status: {response.status_code}")
        if response.status_code == 200:
            action = "Updated" if not clear_existing else "Set new"
            result = f"{action} robot route to: {', '.join(destinations)}"
            print(f"✅ Success: {result}")
            return result
        else:
            print(f"❌ Failed with status: {response.status_code}")
            return f"Failed to update route: {response.status_code}"

    except Exception as e:
        print(f"💥 Exception in update_route: {e}")
        return f"Error updating route: {str(e)}"


def cancel_route(destination=None):
    """Cancel robot route"""
    print(f"🔧 cancel_route called with: destination={destination}")
    try:
        robot_info = get_robot_status()
        print(f"📡 Robot status response: {robot_info}")

        if not robot_info:
            print("❌ No robot connected")
            return "Error: No robot connected"

        robot_id = robot_info["robot_id"]
        print(f"🤖 Robot ID: {robot_id}")

        if not robot_id:
            print("❌ Could not identify robot")
            return "Error: Could not identify robot"

        cancel_data = {"destination": destination}
        print(f"📤 Sending cancel request: {cancel_data}")

        response = requests.post(
            f"{ROBOT_BACKEND_URL}/robots/{robot_id}/cancel",
            json=cancel_data,
            timeout=5
        )

        print(f"📥 Response status: {response.status_code}")
        if response.status_code == 200:
            if destination:
                result = f"Cancelled destination: {destination}"
            else:
                result = "Cancelled all destinations"
            print(f"✅ Success: {result}")
            return result
        else:
            print(f"❌ Failed with status: {response.status_code}")
            return f"Failed to cancel route: {response.status_code}"

    except Exception as e:
        print(f"💥 Exception in cancel_route: {e}")
        return f"Error cancelling route: {str(e)}"


# -----------------------------
# Dynamic system prompt
# -----------------------------
def generate_system_prompt():
    """Generate system prompt with current robot status"""
    robot_info = get_robot_status()

    base_prompt = """You are a tour guide AI robot called Sage at Valparaiso University's College of Engineering.

Your job is to guide visitors through the engineering facilities and campus based on their voice commands. You have detailed knowledge of the main engineering facilities and can provide informative tours.

IMPORTANT: When using tools to control yourself, you MUST use the EXACT system names listed below. You are responsible for understanding user requests and mapping them to these exact identifiers.

TOUR FLOW POLICY (Single-Stop Mode by default)
- Never schedule more than one destination at a time.
- Acknowledge full multi-stop requests, but only send the FIRST stop now.
- Call update_route with exactly one destination and clear_existing=true.
- On arrival, enter Talk Phase:
  1) Deliver a concise intro (20-40 seconds).
  2) Ask at least one preference question (e.g., quick overview, deep dive, or hands-on?).
  3) Offer options: continue here, pick the next stop, or end the tour.
- Only after the user confirms, send the next single destination.
- If the user goes silent after arrival, politely prompt once, then wait.
- If user says “do all,” still proceed one stop at a time unless explicitly told to batch.

TOOL USAGE RULE
- update_route: always 1 destination, clear_existing=true.
- cancel_route: use to stop current movement before changing plans.

MAIN ENGINEERING FACILITIES (with detailed knowledge and exact system names):
- GEMC: The main engineering building housing the College of Engineering. This is the heart of engineering education at Valpo. (Aliases: "main engineering building", "Gellersen", "engineering building", "Gellersen Engineering")
- Fites: A 13,470 sq ft addition opened in 2011 that adds undergraduate labs for senior design, energy/materials research, and manufacturing. It's LEED® Platinum certified and also houses CIS teaching labs (GEM 125 Linux, GEM 232 Windows). (Aliases: "Fites building", "engineering innovation center", "Donald V. Fites", "innovation center")
- SERF: Houses one of just five solar furnaces in the U.S. — the only one at an undergraduate institution. Features a 20x20 ft heliostat feeding a concentrator with 303 mirrors and a louver system. The reactor routinely exceeds 3,000°F, enabling solar-fuels research. Located just east of GEMC. (Aliases: "solar furnace", "solar energy facility", "Markiewicz facility", "solar research")
- Hesse: Drop-in academic support hub for engineering students in GEM 121, providing free tutoring and study help. (Aliases: "tutoring center", "learning center", "Hesse Learning Center", "academic support")
- iHub: Open, cross-disciplinary maker space with tools from 3D printers and laser cutters/etchers to low-tech craft supplies, used for prototyping and student projects. (Aliases: "makerspace", "innovation hub", "3D printing lab", "prototyping lab", "maker space")

OTHER CAMPUS LOCATIONS (can guide to but limited details - use exact system names):
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
1. ALWAYS use the exact system names (GEMC, Fites, SERF, Hesse, iHub, Library, Chapel, Science, Meteorology, ARC, Museum, Weseman, Lebien, Lankenau, Alumni, Brandt, Founders, Kretzmann, Nielsen, Ateufack)
2. When users say things like "take me to the main engineering building", you understand they mean GEMC
3. When users say "show me the solar furnace", you understand they mean SERF
4. When users say "let's go to the makerspace", you understand they mean iHub
5. Convert ALL natural language requests to exact system names before calling tools

TOUR FLOW EXAMPLES:
- User: "Give me a tour of the engineering facilities"
- AI Response: "I'll start by taking you to GEMC, the main engineering building. This is the heart of engineering education at Valpo. Let me navigate there first."
- AI Action: update_route(["GEMC"])
- After Arrival: "Welcome to GEMC! This building houses the College of Engineering and is where students learn mechanical, electrical, and computer engineering. Would you like to see the solar furnace next, or do you have questions about GEMC?"

- User: "Take me on a campus tour"
- AI Response: "I'll start with the iconic Chapel of the Resurrection, then we can explore other highlights. Let me take you there first."
- AI Action: update_route(["Chapel"])
- After Arrival: "Here we are at the Chapel of the Resurrection, dedicated in 1959 and added to the National Register of Historic Places in 2021. It seats about 2,000 people. Would you like to see the library next, or explore another area?"

You have tools available to control yourself:
- update_route: Set new destinations or add to existing route
- cancel_route: Cancel specific or all destinations

When giving tours, focus on the engineering facilities where you have detailed knowledge. For other locations, you can guide visitors there but provide basic information only. Always confirm what you're doing e.g. "Alright, let's go to Fites!". Be enthusiastic about Valparaiso University's engineering program and facilities and don't make up information not explicitly provided."""

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


# -----------------------------
# LLM plumbing
# -----------------------------
def process_by_llm(messages, tools):
    """Calls the LLM. This function is tool-agnostic; pass tools=[] to forbid tool use."""
    try:
        requests.get(llm_thinking + "set", timeout=0.25)
    except Exception:
        pass

    try:
        # Initial ask (system generated on the fly)
        response = client.chat.completions.create(
            messages=[{"role": "system", "content": generate_system_prompt()}] + messages,
            model=MODEL_NAME,
            tools=tools,
        )

        for _ in range(MAX_TOOL_CALLS):
            choice = response.choices[0]

            # Stop if the model isn't asking for tools
            if choice.finish_reason != "tool_calls":
                break

            assistant_msg = choice.message
            tool_calls = assistant_msg.tool_calls
            # Append the assistant's tool_calls message once
            messages.append(assistant_msg)

            # Execute ALL requested tools from this turn
            for tc in tool_calls:
                name = tc.function.name
                args = json.loads(tc.function.arguments)
                tool_response = funcs[name](**args)
                messages.append({
                    "role": "tool",
                    "content": tool_response,
                    "tool_call_id": tc.id,
                })

            # Now that all tools for this turn are done, ask again
            response = client.chat.completions.create(
                messages=[{"role": "system", "content": generate_system_prompt()}] + messages,
                model=MODEL_NAME,
                tools=tools,
            )

        # Append final assistant message and return
        messages.append(response.choices[0].message)
        return response.choices[0].message.content, messages

    finally:
        try:
            requests.get(llm_thinking + "unset", timeout=0.25)
        except Exception:
            pass


# -----------------------------
# Recorder callbacks (mic state)
# -----------------------------
def set_recording():
    """Called when STT starts recording; stop TTS and mark mic active."""
    global is_recording_flag
    if stream.is_playing():
        stream.stop()
    is_recording_flag = True
    try:
        requests.get(llm_thinking + "unset", timeout=0.25)
        requests.get(llm_recording + "set", timeout=0.25)
    except Exception:
        pass


def unset_recording():
    """Called when STT stops; mark mic idle."""
    global is_recording_flag
    is_recording_flag = False
    try:
        requests.get(llm_recording + "unset", timeout=0.25)
    except Exception:
        pass

def on_wakeword():
    if stream.is_playing():
        stream.stop()  # stop ONLY on “sage”

# -----------------------------
# Event system
# -----------------------------
def enqueue_arrival(target):
    """Queue an arrival event with moderate priority (lower number = higher priority)."""
    event = {"kind": "arrival", "target": target, "epoch": nav_epoch}
    event_q.put((10, time.time(), event))


def event_dispatcher():
    """Process queued events only during 'quiet windows', and without allowing tool use."""
    while True:
        try:
            priority, ts, ev = event_q.get(timeout=0.25)
        except queue.Empty:
            time.sleep(0.1)
            continue

        if ev.get("kind") == "arrival":
            target = ev.get("target")
            epoch = ev.get("epoch")
            print(f"Arrival event: {target} at epoch {epoch}")
            # Quiet window: don't speak over TTS or mic
            while stream.is_playing() or is_recording_flag:
                time.sleep(0.1)

            # Drop stale events if route was rescheduled
            if epoch != nav_epoch or target != current_target:
                continue

            # Get conversation history for context
            with messages_lock:
                messages.append({"role": "system", "content": f"EVENT: You have arrived at {target}. Enter Talk Phase. Give a 10-20 second intro, ask ONE preference question."})
                if len(messages) > 12:
                    messages[:] = messages[-12:]


            # Isolated LLM turn: local messages, tools=[]
            with llm_lock:
                print(f"Processing arrival event: {target} at epoch {epoch}")
                reply, _ = process_by_llm(messages=messages, tools=[])

            # Double-check quiet window before speaking
            if not stream.is_playing() and not is_recording_flag:
                print(f"Speaking after arrival: {reply}")
                stream.feed(reply)
                stream.play_async()


def robot_watcher():
    """Poll robot status and enqueue arrival events on rising edge."""
    global _last_status, _last_arrived_at, _last_destinations
    while True:
        info = get_robot_status()
        if info:
            data = info["data"]
            status = data.get("status")
            arrived_at = data.get("arrived_at")  # backend may set this on arrival
            destinations = data.get("destinations", [])

            # Rising edge detection:
            # 1) status transitioned to 'arrived'
            just_arrived = (_last_status != "arrived" and status == "arrived")

            # 2) arrived_at changed (if backend provides)
            arrived_changed = (arrived_at is not None and arrived_at != _last_arrived_at)

            # Decide arrival target (prefer explicit arrived_at; fallback to current_target)
            target = arrived_at or current_target

            if target and (just_arrived or arrived_changed):
                enqueue_arrival(target)

            _last_status = status
            _last_arrived_at = arrived_at
            _last_destinations = destinations

        time.sleep(1.0)  # light polling cadence


# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    # TTS / LLM init
    # engine = SystemEngine()
    engine = GTTSEngine()
    # engine = CoquiEngine()
    #engine.set_voice("Claribel Dervla")
    stream = TextToAudioStream(engine, frames_per_buffer=1024)

    client = openai.OpenAI(api_key=read_openai_key("api_keys.json"))
    funcs = {"update_route": update_route, "cancel_route": cancel_route}

    # Start background threads
    threading.Thread(target=event_dispatcher, daemon=True).start()
    threading.Thread(target=robot_watcher, daemon=True).start()

    recorder = AudioToTextRecorder(
        language="en",
        spinner=True,
        model=["large-v2", "tiny.en"][1],
        device="cpu",
        on_recording_start=set_recording,
        on_recording_stop=unset_recording,
        # wakeword_backend=["pvporcupine", "openwakeword"][1],
        # openwakeword_model_paths="sage.onnx",
        # openwakeword_inference_framework="onnx",
        # wake_words_sensitivity=0.1,      # try 0.7–0.85 in a school
        # wake_word_buffer_duration=0.75,   # 0.75–1.0s keeps “sage” out of transcript,
        # on_wakeword_detected=on_wakeword,
        # wake_words="sage" # You need to leave this here for openwakeword to work (even though it's not used)
    )

    print("Ready to guide Valparaiso University engineering tours...")

    while True:
        user_input = recorder.text()
        if user_input != "":
            print("User input: ", user_input)
            with messages_lock:
                messages.append({"role": "user", "content": user_input})
                if len(messages) > 12:
                    messages[:] = messages[-12:]

            with llm_lock:
                ai_reply, messages = process_by_llm(messages=messages, tools=tools)

            if stream.is_playing():
                stream.stop()
            stream.feed(ai_reply)
            stream.play()
            print("ai reply: ", ai_reply)

