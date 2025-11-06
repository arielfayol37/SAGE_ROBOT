# stream_oww_min_beep.py
import argparse
import queue
import sys
import threading
import time
import wave
import numpy as np
import sounddevice as sd

import openwakeword
from openwakeword.model import Model


def load_wav(path):
    # minimal WAV loader using stdlib wave (PCM int16 mono/stereo)
    with wave.open(path, "rb") as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        sr = wf.getframerate()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)

    if sampwidth != 2:
        raise ValueError(f"Only 16-bit PCM WAV supported (got {8*sampwidth}-bit)")

    data = np.frombuffer(raw, dtype=np.int16)
    if n_channels == 2:
        data = data.reshape(-1, 2).mean(axis=1).astype(np.int16)  # downmix to mono
    elif n_channels != 1:
        raise ValueError(f"Unsupported channel count: {n_channels}")

    # Convert to float32 in [-1, 1]
    audio = (data.astype(np.float32) / 32768.0).clip(-1.0, 1.0)
    return audio, sr


class BeepPlayer:
    def __init__(self, wav_path, out_device=None, delay_s=0.5, allow_overlap=False):
        self.audio, self.sr = load_wav(wav_path)
        self.out_device = out_device
        self.delay_s = delay_s
        self.allow_overlap = allow_overlap
        self._lock = threading.Lock()
        self._is_playing = False

    def is_playing(self):
        with self._lock:
            return self._is_playing

    def _play_thread(self):
        try:
            time.sleep(self.delay_s)
            # blocking=True here keeps the *playback thread* busy, not the main loop
            sd.play(self.audio, samplerate=self.sr, device=self.out_device, blocking=True)
        finally:
            with self._lock:
                self._is_playing = False

    def trigger(self):
        with self._lock:
            if self._is_playing and not self.allow_overlap:
                return
            self._is_playing = True
        t = threading.Thread(target=self._play_thread, daemon=True)
        t.start()


def main():
    p = argparse.ArgumentParser(description="OpenWakeWord realtime stream + delayed WAV playback")
    p.add_argument("--model", default="builtin",
                   help='Path to .tflite/.onnx model, or "builtin" to load included models')
    p.add_argument("--sr", type=int, default=16000, help="Mic sample rate (must be 16000)")
    p.add_argument("--frame_ms", type=int, default=80, help="Frame size in ms (80 recommended)")
    p.add_argument("--threshold", type=float, default=0.5, help="Trigger when score >= threshold")
    p.add_argument("--in_device", default=None, help="sounddevice input device index/name")
    p.add_argument("--out_device", default=None, help="sounddevice output device index/name")
    p.add_argument("--print_all", action="store_true", help="Print raw scores every frame")
    p.add_argument("--beep_wav", required=True, help="Path to a WAV file to play (~5s)")
    p.add_argument("--beep_delay", type=float, default=0.5, help="Seconds after trigger to start playback")
    p.add_argument("--allow_overlap", action="store_true", help="Allow overlapping beep playbacks")
    p.add_argument("--framework", choices=["tflite","onnx"], default="onnx",
                   help="Inference backend for OWW builtin models")
    args = p.parse_args()

    openwakeword.utils.download_models()

    if args.model == "builtin":
        model = Model(inference_framework=args.framework)  # use ONNX to avoid NumPy/TFLite issues
    else:
        model = Model(wakeword_models=[args.model])

    if args.sr != 16000:
        raise ValueError("OpenWakeWord expects 16 kHz mic audio.")

    spp = int(args.sr * (args.frame_ms / 1000.0))
    if spp <= 0:
        raise ValueError("frame_ms too small.")

    # Prepare non-blocking beep player
    player = BeepPlayer(
        wav_path=args.beep_wav,
        out_device=args.out_device,
        delay_s=args.beep_delay,
        allow_overlap=args.allow_overlap,
    )

    print("Loaded models:", ", ".join(model.models))
    print(f"Mic stream: {args.sr} Hz, {args.frame_ms} ms frames ({spp} samples), dtype=int16, mono")
    print(f"Beep file: {args.beep_wav} @ {player.sr} Hz, out_device={args.out_device}")
    print("Listening… (Ctrl+C to stop)\n")

    q: "queue.Queue[np.ndarray]" = queue.Queue()

    def audio_callback(indata, frames, time_info, status):
        if status:
            print(f"[audio] {status}", file=sys.stderr)
        mono = indata[:, 0].copy()
        q.put(mono)

    try:
        with sd.InputStream(
            samplerate=args.sr,
            channels=1,
            dtype="int16",
            blocksize=spp,
            device=args.in_device,
            callback=audio_callback,
        ):
            while True:
                frame = q.get()
                preds = model.predict(frame)

                if args.print_all:
                    line = " | ".join(f"{k}:{preds.get(k, 0.0):.3f}" for k in preds.keys())
                    print(line, flush=True)
                else:
                    # fire instantly on threshold crossing
                    for k, v in preds.items():
                        if v >= args.threshold:
                            print(f"[TRIGGER-FRAME] {k} score={v:.3f}", flush=True)
                            player.trigger()
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        try:
            sd.stop()
        except Exception:
            pass
        try:
            model.reset()
        except Exception:
            pass


if __name__ == "__main__":
    main()
