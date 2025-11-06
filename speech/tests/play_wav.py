# play_wav.py
import sys
import sounddevice as sd
import soundfile as sf

if len(sys.argv) != 2:
    print("Usage: python play_wav.py <path_to_wav>")
    sys.exit(1)

path = sys.argv[1]
data, sr = sf.read(path, dtype="float32")
sd.play(data, sr, blocking=True)
