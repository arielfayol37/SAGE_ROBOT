import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav

# Select the Blue Snowball mic
sd.default.device = (2, None)  # (input_device_index, output_device_index)

# Record 5 seconds of audio
fs = 44100  # Sample rate
duration = 5
print("Recording...")
audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
sd.wait()
print("Done!")

# Save to a file
wav.write('matson_mic_test.wav', fs, audio)
