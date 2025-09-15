import numpy as np, time, pyaudio

RATE = 48000
CH = 1
FMT = pyaudio.paInt16
FRAMES_PER_BUFFER = 2048           # ~43 ms at 48k; safe on ARM
WRITE_MS = 20                      # write in ~20 ms chunks
SAMPLES_PER_WRITE = int(RATE * WRITE_MS / 1000)

# 1 second tone precomputed (we'll slice 20 ms chunks from it)
t = np.arange(RATE, dtype=np.float32)
tone = 0.1 * np.sin(2*np.pi*440*t/RATE)
tone_i16 = (tone * 32767).astype(np.int16)

pa = pyaudio.PyAudio()
s = pa.open(format=FMT, channels=CH, rate=RATE,
            output=True, frames_per_buffer=FRAMES_PER_BUFFER)

# --- PREBUFFER ~200 ms to mimic streaming safety ---
pre_samp = int(0.2 * RATE)
s.write(tone_i16[:pre_samp].tobytes(), exception_on_underflow=False)

# --- STREAM loop: write strictly ~20 ms at a time ---
idx = 0
duration_sec = 3
end = time.time() + duration_sec
chunk_bytes = (SAMPLES_PER_WRITE * 2)  # int16 mono

while time.time() < end:
    a = tone_i16[idx:idx+SAMPLES_PER_WRITE]
    if len(a) < SAMPLES_PER_WRITE:
        # wrap the buffer (continuous)
        need = SAMPLES_PER_WRITE - len(a)
        a = np.concatenate([a, tone_i16[:need]])
        idx = need
    else:
        idx += SAMPLES_PER_WRITE
    s.write(a.tobytes(), exception_on_underflow=False)

s.stop_stream(); s.close(); pa.terminate()
print("OK")
