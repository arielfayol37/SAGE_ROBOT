import numpy as np, time, random, pyaudio
RATE, CH, FMT = 48000, 1, pyaudio.paInt16
FRAMES_PER_BUFFER = 4096            # a bit bigger when we add jitter
WRITE_MS = 20
SAMPLES_PER_WRITE = int(RATE * WRITE_MS / 1000)

t = np.arange(RATE, dtype=np.float32)
tone = 0.1 * np.sin(2*np.pi*440*t/RATE)
tone_i16 = (tone * 32767).astype(np.int16)

pa = pyaudio.PyAudio()
s = pa.open(format=FMT, channels=CH, rate=RATE,
            output=True, frames_per_buffer=FRAMES_PER_BUFFER)

# Prebuffer ~300 ms
s.write(tone_i16[:int(0.3*RATE)].tobytes(), exception_on_underflow=False)

idx = 0
end = time.time() + 5
while time.time() < end:
    # random small stall (0–7 ms) like GC/decoder pauses
    time.sleep(random.random() * 0.007)

    a = tone_i16[idx:idx+SAMPLES_PER_WRITE]
    if len(a) < SAMPLES_PER_WRITE:
        need = SAMPLES_PER_WRITE - len(a)
        a = np.concatenate([a, tone_i16[:need]])
        idx = need
    else:
        idx += SAMPLES_PER_WRITE

    s.write(a.tobytes(), exception_on_underflow=False)

s.stop_stream(); s.close(); pa.terminate()
print("OK")
