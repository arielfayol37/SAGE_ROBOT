# stream_oww_min.py
import argparse
import queue
import sys

import numpy as np
import sounddevice as sd

import openwakeword
from openwakeword.model import Model


def main():
    p = argparse.ArgumentParser(description="OpenWakeWord: raw, realtime stream (no smoothing/debounce)")
    p.add_argument("--model", default="builtin",
                   help='Path to .tflite model, or "builtin" to load included models')
    p.add_argument("--sr", type=int, default=16000, help="Sample rate (must be 16000)")
    p.add_argument("--frame_ms", type=int, default=80, help="Frame size in ms (80 recommended)")
    p.add_argument("--threshold", type=float, default=0.5, help="Print when any score >= threshold")
    p.add_argument("--device", default=None, help="sounddevice input device index or name")
    p.add_argument("--print_all", action="store_true",
                   help="Print predictions for every frame (very verbose)")
    args = p.parse_args()

    # download included models (no-op if already present)
    openwakeword.utils.download_models()

    # load model(s)
    if args.model == "builtin":
        model = Model(inference_framework="onnx")  # all included pre-trained models
    else:
        model = Model(wakeword_models=[args.model])

    if args.sr != 16000:
        raise ValueError("OpenWakeWord expects 16 kHz audio (use --sr 16000).")

    samples_per_frame = int(args.sr * (args.frame_ms / 1000.0))
    if samples_per_frame % 1 != 0:
        raise ValueError("frame_ms must yield an integer number of samples.")
    if samples_per_frame <= 0:
        raise ValueError("frame_ms too small.")

    print(f"Streaming at {args.sr} Hz, {args.frame_ms} ms per frame "
          f"({samples_per_frame} samples), dtype=int16, mono")
    print("Listening… (Ctrl+C to stop)\n")

    q: "queue.Queue[np.ndarray]" = queue.Queue()

    def audio_callback(indata, frames, time_info, status):
        if status:
            print(f"[audio] {status}", file=sys.stderr)
        # indata: (frames, channels), dtype=int16
        mono = indata[:, 0].copy()
        q.put(mono)

    try:
        with sd.InputStream(
            samplerate=args.sr,
            channels=1,
            dtype="int16",
            blocksize=samples_per_frame,
            device=args.device,
            callback=audio_callback,
        ):
            while True:
                frame = q.get()  # np.int16 mono, length = samples_per_frame
                preds = model.predict(frame)  # raw per-model scores for this frame

                if args.print_all:
                    # print all raw scores each frame (can be very chatty)
                    line = " | ".join(f"{k}:{preds.get(k, 0.0):.3f}" for k in model.models)
                    print(line, flush=True)
                else:
                    # print only when anything crosses threshold
                    hot = {k: v for k, v in preds.items() if v >= args.threshold}
                    if hot:
                        for k, v in hot.items():
                            print(f"[TRIGGER-FRAME] {k} score={v:.3f}", flush=True)

    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        try:
            model.reset()
        except Exception:
            pass


if __name__ == "__main__":
    main()
