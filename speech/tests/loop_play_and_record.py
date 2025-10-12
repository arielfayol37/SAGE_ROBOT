# loop_play_and_record.py
import argparse
import threading
import time
import wave
from pathlib import Path

import numpy as np
import sounddevice as sd


def load_wav_mono_float32(path):
    """Load a 16-bit PCM WAV, downmix to mono if needed, return (audio[-1..1], sr)."""
    with wave.open(str(path), "rb") as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        sr = wf.getframerate()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)

    if sampwidth != 2:
        raise ValueError(f"Only 16-bit PCM WAV supported (got {8*sampwidth}-bit).")

    x = np.frombuffer(raw, dtype=np.int16)
    if n_channels == 2:
        x = x.reshape(-1, 2).mean(axis=1).astype(np.int16)  # simple stereo→mono downmix
    elif n_channels != 1:
        raise ValueError(f"Unsupported channel count: {n_channels}")

    x = (x.astype(np.float32) / 32768.0).clip(-1.0, 1.0)  # float32 [-1,1]
    return x, sr


class Looper:
    """Continuously plays a mono float32 buffer on an OutputStream (non-blocking)."""
    def __init__(self, audio_f32_mono, sr, out_device=None):
        self.audio = audio_f32_mono
        self.sr = sr
        self.out_device = out_device
        self.pos = 0
        self.stop_flag = threading.Event()
        self._stream = None
        self._lock = threading.Lock()

    def _callback(self, outdata, frames, time_info, status):
        if status:
            # status may report under/overruns; still try to fill audio
            print(f"[playback] {status}")
        with self._lock:
            a = self.audio
            N = a.shape[0]
            # prepare mono chunk with wrap-around
            end = self.pos + frames
            if end <= N:
                chunk = a[self.pos:end]
            else:
                head = a[self.pos:]
                tail = a[: (end - N)]
                chunk = np.concatenate([head, tail], axis=0)
            self.pos = end % N

        # outdata may be stereo/etc; duplicate across channels if needed
        if outdata.shape[1] == 1:
            outdata[:, 0] = chunk
        else:
            outdata[:] = np.tile(chunk[:, None], (1, outdata.shape[1]))

    def start(self):
        self._stream = sd.OutputStream(
            samplerate=self.sr,
            channels=1,              # we generate mono and duplicate in callback if device has >1 chans
            dtype="float32",
            device=self.out_device,
            callback=self._callback,
        )
        self._stream.start()

    def stop(self):
        self.stop_flag.set()
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None


class Recorder:
    """Records from mic into RAM; later saves as 16-bit PCM WAV."""
    def __init__(self, sr=16000, channels=1, in_device=None, blocksize=0):
        self.sr = sr
        self.channels = channels
        self.in_device = in_device
        self.blocksize = blocksize
        self.stop_flag = threading.Event()
        self.buffers = []  # list of np.int16 blocks
        self._stream = None

    def _callback(self, indata, frames, time_info, status):
        if status:
            print(f"[record] {status}")
        # store copy (int16)
        self.buffers.append(indata.copy())

    def start(self):
        self._stream = sd.InputStream(
            samplerate=self.sr,
            channels=self.channels,
            dtype="int16",
            device=self.in_device,
            blocksize=self.blocksize,  # 0 lets backend choose
            callback=self._callback,
        )
        self._stream.start()

    def stop(self):
        self.stop_flag.set()
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    def save_wav(self, path):
        data = np.concatenate(self.buffers, axis=0) if self.buffers else np.zeros((0, self.channels), dtype=np.int16)
        with wave.open(str(path), "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(self.sr)
            wf.writeframes(data.tobytes())


def main():
    ap = argparse.ArgumentParser(description="Play WAV in a loop and record mic simultaneously for N seconds.")
    ap.add_argument("--beep_wav", required=True, help="Path to the WAV to loop (e.g., your 5s beep).")
    ap.add_argument("--duration", type=float, default=60.0, help="Seconds to run (default 60).")
    ap.add_argument("--outfile", default="mic_capture.wav", help="Output WAV filename for recorded mic.")
    ap.add_argument("--in_device", default=None, help="Input device index/name for mic.")
    ap.add_argument("--out_device", default=None, help="Output device index/name for speaker.")
    ap.add_argument("--rec_sr", type=int, default=16000, help="Recording sample rate (Hz).")
    ap.add_argument("--rec_channels", type=int, default=1, help="Recording channels (1=mono).")
    args = ap.parse_args()

    # Load beep and start looping playback
    wav_path = Path(args.beep_wav)
    beep, beep_sr = load_wav_mono_float32(wav_path)
    looper = Looper(beep, beep_sr, out_device=args.out_device)
    looper.start()

    # Start recorder
    rec = Recorder(sr=args.rec_sr, channels=args.rec_channels, in_device=args.in_device)
    rec.start()

    print(f"Playing '{wav_path.name}' on loop @ {beep_sr} Hz | Recording mic to '{args.outfile}' @ {args.rec_sr} Hz")
    print("Running...", f"{args.duration:.1f}s", "(Ctrl+C to stop early)")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Stop streams
        looper.stop()
        rec.stop()
        # Save capture
        out_path = Path(args.outfile).resolve()
        rec.save_wav(out_path)
        print(f"Saved mic capture → {out_path}")


if __name__ == "__main__":
    main()
