#!/usr/bin/env python3
import argparse
import os
import re
import signal
import sys
import tempfile
import time
import uuid
import shutil
import subprocess
import wave
from contextlib import contextmanager
from piper import PiperVoice

# ---------- CLI ----------
def parse_args():
    ap = argparse.ArgumentParser(description="Ultra-low-latency Piper runner (headless, ALSA).")
    ap.add_argument("--model", default="../en_US-amy-medium.onnx", help="Path to .onnx voice (e.g., en_US-amy-medium.onnx)")
    ap.add_argument("--device", default=None, help='ALSA device for aplay (e.g., "hw:1,0"). Default: system default')
    ap.add_argument("--rate", type=int, default=22050, help="Playback sample rate if needed (typically 22050 for many Piper voices)")
    ap.add_argument("--no-chunk", action="store_true", help="Disable clause chunking; synthesize whole text at once")
    ap.add_argument("--aplay-buffer-time", type=int, default=None, help="aplay --buffer-time (µs). Smaller can reduce latency (e.g., 40000)")
    ap.add_argument("--aplay-period-size", type=int, default=None, help="aplay --period-size (frames). Smaller can reduce latency (e.g., 256)")
    return ap.parse_args()

# ---------- Utils ----------
@contextmanager
def ram_wav_path():
    # Prefer RAM (Linux) to avoid disk I/O latency
    base = "/dev/shm" if os.path.isdir("/dev/shm") else tempfile.gettempdir()
    path = os.path.join(base, f"piper_{uuid.uuid4().hex}.wav")
    try:
        yield path
    finally:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass

def have_aplay():
    return shutil.which("aplay") is not None

def build_aplay_cmd(path, device=None, buffer_time=None, period_size=None):
    cmd = ["aplay", "-q", path]  # -q: quiet
    if device:
        cmd = ["aplay", "-q", "-D", device, path]
    if buffer_time is not None:
        # aplay expects microseconds
        # Place before filename
        base = ["aplay", "-q"]
        if device:
            base += ["-D", device]
        base += ["--buffer-time", str(buffer_time)]
        if period_size is not None:
            base += ["--period-size", str(period_size)]
        cmd = base + [path]
    elif period_size is not None:
        base = ["aplay", "-q"]
        if device:
            base += ["-D", device]
        base += ["--period-size", str(period_size)]
        cmd = base + [path]
    return cmd

def split_into_clauses(text):
    # Fast, simple chunking to speak early: split at punctuation and long pauses
    # Keeps punctuation attached for prosody.
    parts = re.split(r"(?<=[\.\!\?\:\;\,])\s+", text.strip())
    # Merge tiny pieces to avoid too-short blips
    merged = []
    buff = []
    for p in parts:
        if not p:
            continue
        buff.append(p)
        if sum(len(x) for x in buff) >= 40:  # ~tunable
            merged.append(" ".join(buff))
            buff = []
    if buff:
        merged.append(" ".join(buff))
    return merged

# ---------- Core ----------
class PiperRunner:
    def __init__(self, model_path, samplerate_hint=22050):
        self.voice = PiperVoice.load(model_path)
        self.aplay_proc = None
        self.samplerate_hint = samplerate_hint

    def warmup(self):
        # Very short, ensures the first real call has minimal overhead
        with ram_wav_path() as outp:
            with wave.open(outp, "wb") as wavf:
                self.voice.synthesize_wav(".", wavf)

    def stop_playback(self):
        if self.aplay_proc and self.aplay_proc.poll() is None:
            try:
                self.aplay_proc.terminate()
                try:
                    self.aplay_proc.wait(timeout=0.2)
                except subprocess.TimeoutExpired:
                    self.aplay_proc.kill()
            except Exception:
                pass
        self.aplay_proc = None

    def say_wav(self, text, device=None, buffer_time=None, period_size=None):
        # Synthesize to RAM, then play via aplay (blocking)
        with ram_wav_path() as outp:
            with wave.open(outp, "wb") as wavf:
                self.voice.synthesize_wav(text, wavf)
            cmd = build_aplay_cmd(outp, device=device, buffer_time=buffer_time, period_size=period_size)
            self.stop_playback()
            self.aplay_proc = subprocess.Popen(cmd)
            self.aplay_proc.wait()
            self.aplay_proc = None

    def say_chunked(self, text, device=None, buffer_time=None, period_size=None):
        # Speak clause by clause for "feed-first" feel
        for chunk in split_into_clauses(text):
            self.say_wav(chunk, device=device, buffer_time=buffer_time, period_size=period_size)

def interactive_loop(runner: PiperRunner, device=None, buffer_time=None, period_size=None, chunk=True):
    print("Interactive Piper TTS. Type 'exit' or 'quit' to stop.")
    try:
        while True:
            try:
                line = input("Say> ").strip()
            except EOFError:
                print()
                break
            if not line:
                continue
            if line.lower() in ("exit", "quit"):
                break
            if chunk:
                runner.say_chunked(line, device=device, buffer_time=buffer_time, period_size=period_size)
            else:
                runner.say_wav(line, device=device, buffer_time=buffer_time, period_size=period_size)
    except KeyboardInterrupt:
        print("\nInterrupted.")

def main():
    args = parse_args()

    if not have_aplay():
        print("ERROR: 'aplay' not found. Install ALSA utils: sudo apt-get install alsa-utils", file=sys.stderr)
        sys.exit(1)

    # Load & warm up
    runner = PiperRunner(args.model, samplerate_hint=args.rate)
    runner.warmup()

    # Headless-friendly: ensure graceful exit kills playback
    def _sigterm(_signo, _frame):
        runner.stop_playback()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, signal.default_int_handler)

    interactive_loop(
        runner,
        device=args.device,
        buffer_time=args.aplay_buffer_time,
        period_size=args.aplay_period_size,
        chunk=(not args.no_chunk),
    )

if __name__ == "__main__":
    main()
