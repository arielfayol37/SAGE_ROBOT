import wave
import os
import sys
import subprocess
import shutil
from piper import PiperVoice

# Load the voice model once at startup
voice = PiperVoice.load("../en_US-amy-medium.onnx")


def _play_wav(path: str) -> None:
    """Play a WAV file using a platform-appropriate method."""
    if sys.platform == "linux":
        # Use aplay on Linux, which is part of alsa-utils
        if shutil.which("aplay"):
            try:
                subprocess.run(["aplay", "-q", path], check=True)
                return
            except (subprocess.CalledProcessError, FileNotFoundError):
                print("aplay failed, falling back to other methods.")
        else:
            print("aplay not found, install with 'sudo apt-get install alsa-utils'. Falling back.")

    if sys.platform == "win32":
        try:
            import winsound
            winsound.PlaySound(path, winsound.SND_FILENAME)
            return
        except ImportError:
            print("winsound module not found, falling back to simpleaudio.")

    try:
        import simpleaudio as sa
        wave_obj = sa.WaveObject.from_wave_file(path)
        play_obj = wave_obj.play()
        play_obj.wait_done()
        return
    except ImportError:
        print("simpleaudio is not installed. Please install it with 'pip install simpleaudio'.")
    except Exception as e:
        print(f"Playback with simpleaudio failed: {e}")

    print(f"Audio saved to: {os.path.abspath(path)}")



def synthesize_text(text: str, out_path: str = "test.wav") -> None:
    """Synthesize `text` to `out_path` and play it.

    The voice model is loaded once globally. The function ensures the
    WAV file is closed before attempting playback.
    """
    # Ensure directory exists for out_path
    out_dir = os.path.dirname(out_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    # Synthesize to WAV (voice.synthesize_wav writes WAV frames)
    with wave.open(out_path, "wb") as wav_file:
        voice.synthesize_wav(text, wav_file)

    # Play the generated WAV
    _play_wav(out_path)


def interactive_loop(prompt: str = "Say> ") -> None:
    """Infinite input loop: prompt the user, synthesize the response, play it.

    Type 'quit' or 'exit' to stop. Use Ctrl-C to interrupt.
    """
    print("Interactive TTS loop. Type 'quit' or 'exit' to stop.")
    try:
        while True:
            try:
                text = input(prompt)
            except EOFError:
                # Ctrl-D (or pipe close) — exit loop
                print()
                break

            if not text:
                continue

            if text.strip().lower() in ("quit", "exit"):
                break

            synthesize_text(text)
    except KeyboardInterrupt:
        print("\nInterrupted, exiting.")


if __name__ == "__main__":
    interactive_loop()