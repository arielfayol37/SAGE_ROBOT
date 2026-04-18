"""

The AudioToTextRecorder class in the provided code facilitates
fast speech-to-text transcription.

The class employs the faster_whisper library to transcribe the recorded audio
into text using machine learning models, which can be run either on a GPU or
CPU. Voice activity detection (VAD) is built in, meaning the software can
automatically start or stop recording based on the presence or absence of
speech. It integrates wake word detection through the pvporcupine library,
allowing the software to initiate recording when a specific word or phrase
is spoken. The system provides real-time feedback and can be further
customized.

Features:
- Voice Activity Detection: Automatically starts/stops recording when speech
  is detected or when speech ends.
- Wake Word Detection: Starts recording when a specified wake word (or words)
  is detected.
- Event Callbacks: Customizable callbacks for when recording starts
  or finishes.
- Fast Transcription: Returns the transcribed text from the audio as fast
  as possible.

Author: Kolja Beigel

"""

from faster_whisper import WhisperModel, BatchedInferencePipeline
from typing import Iterable, List, Optional, Union
from openwakeword.model import Model
import torch.multiprocessing as mp
from scipy.signal import resample
import signal as system_signal
from ctypes import c_bool
from scipy import signal
from .safepipe import SafePipe
import soundfile as sf
import faster_whisper
import openwakeword
import collections
import numpy as np
# import pvporcupine
import traceback
import threading
import webrtcvad
import datetime
import platform
import logging
import struct
import base64
import queue
import torch
import halo
import time
import copy
import os
import re
import gc

# Named logger for this module.
logger = logging.getLogger("realtimestt")
logger.propagate = False

# Set OpenMP runtime duplicate library handling to OK (Use only for development!)
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

INIT_MODEL_TRANSCRIPTION = "tiny"
INIT_MODEL_TRANSCRIPTION_REALTIME = "tiny"
INIT_REALTIME_PROCESSING_PAUSE = 0.2
INIT_REALTIME_INITIAL_PAUSE = 0.2
INIT_SILERO_SENSITIVITY = 0.4
INIT_WEBRTC_SENSITIVITY = 3
INIT_POST_SPEECH_SILENCE_DURATION = 0.6
INIT_MIN_LENGTH_OF_RECORDING = 0.5
INIT_MIN_GAP_BETWEEN_RECORDINGS = 0
INIT_WAKE_WORDS_SENSITIVITY = 0.6
INIT_PRE_RECORDING_BUFFER_DURATION = 1.0
INIT_WAKE_WORD_ACTIVATION_DELAY = 0.0
INIT_WAKE_WORD_TIMEOUT = 5.0
INIT_WAKE_WORD_BUFFER_DURATION = 0.1
ALLOWED_LATENCY_LIMIT = 100

TIME_SLEEP = 0.02
SAMPLE_RATE = 16000
BUFFER_SIZE = 512
INT16_MAX_ABS_VALUE = 32768.0

INIT_HANDLE_BUFFER_OVERFLOW = False
if platform.system() != 'Darwin':
    INIT_HANDLE_BUFFER_OVERFLOW = True


class TranscriptionWorker:
    def __init__(self, conn, stdout_pipe, model_path, download_root, compute_type, gpu_device_index, device,
                 ready_event, shutdown_event, interrupt_stop_event, beam_size, initial_prompt, suppress_tokens,
                 batch_size, faster_whisper_vad_filter, normalize_audio):
        self.conn = conn
        self.stdout_pipe = stdout_pipe
        self.model_path = model_path
        self.download_root = download_root
        self.compute_type = compute_type
        self.gpu_device_index = gpu_device_index
        self.device = device
        self.ready_event = ready_event
        self.shutdown_event = shutdown_event
        self.interrupt_stop_event = interrupt_stop_event
        self.beam_size = beam_size
        self.initial_prompt = initial_prompt
        self.suppress_tokens = suppress_tokens
        self.batch_size = batch_size
        self.faster_whisper_vad_filter = faster_whisper_vad_filter
        self.normalize_audio = normalize_audio
        self.queue = queue.Queue()

    def custom_print(self, *args, **kwargs):
        message = ' '.join(map(str, args))
        try:
            self.stdout_pipe.send(message)
        except (BrokenPipeError, EOFError, OSError):
            pass

    def poll_connection(self):
        while not self.shutdown_event.is_set():
            try:
                if self.conn.poll(0.01):
                    data = self.conn.recv()
                    self.queue.put(data)
                else:
                    time.sleep(TIME_SLEEP)
            except Exception as e:
                logging.error(f"Error receiving data from connection: {e}", exc_info=True)
                time.sleep(TIME_SLEEP)

    def run(self):
        if __name__ == "__main__":
             system_signal.signal(system_signal.SIGINT, system_signal.SIG_IGN)
             __builtins__['print'] = self.custom_print

        logging.info(f"Initializing faster_whisper main transcription model {self.model_path}")

        try:
            model = faster_whisper.WhisperModel(
                model_size_or_path=self.model_path,
                device=self.device,
                compute_type=self.compute_type,
                device_index=self.gpu_device_index,
                download_root=self.download_root,
            )
            if self.batch_size > 0:
                model = BatchedInferencePipeline(model=model)

            # Run a warm-up transcription
            current_dir = os.path.dirname(os.path.realpath(__file__))
            warmup_audio_path = os.path.join(
                current_dir, "warmup_audio.wav"
            )
            warmup_audio_data, _ = sf.read(warmup_audio_path, dtype="float32")
            segments, info = model.transcribe(warmup_audio_data, language="en", beam_size=1)
            model_warmup_transcription = " ".join(segment.text for segment in segments)
        except Exception as e:
            logging.exception(f"Error initializing main faster_whisper transcription model: {e}")
            raise

        self.ready_event.set()
        logging.debug("Faster_whisper main speech to text transcription model initialized successfully")

        # Start the polling thread
        polling_thread = threading.Thread(target=self.poll_connection)
        polling_thread.start()

        try:
            while not self.shutdown_event.is_set():
                try:
                    audio, language, use_prompt = self.queue.get(timeout=0.1)
                    try:
                        logging.debug(f"Transcribing audio with language {language}")
                        start_t = time.time()

                        # normalize audio to -0.95 dBFS
                        if audio is not None and audio.size > 0:
                            if self.normalize_audio:
                                peak = np.max(np.abs(audio))
                                if peak > 0:
                                    audio = (audio / peak) * 0.95
                        else:
                            logging.error("Received None audio for transcription")
                            self.conn.send(('error', "Received None audio for transcription"))
                            continue

                        prompt = None
                        if use_prompt:
                            prompt = self.initial_prompt if self.initial_prompt else None

                        if self.batch_size > 0:
                            segments, info = model.transcribe(
                                audio,
                                language=language if language else None,
                                beam_size=self.beam_size,
                                initial_prompt=prompt,
                                suppress_tokens=self.suppress_tokens,
                                batch_size=self.batch_size,
                                vad_filter=self.faster_whisper_vad_filter
                            )
                        else:
                            segments, info = model.transcribe(
                                audio,
                                language=language if language else None,
                                beam_size=self.beam_size,
                                initial_prompt=prompt,
                                suppress_tokens=self.suppress_tokens,
                                vad_filter=self.faster_whisper_vad_filter
                            )
                        elapsed = time.time() - start_t
                        transcription = " ".join(seg.text for seg in segments).strip()
                        logging.debug(f"Final text detected with main model: {transcription} in {elapsed:.4f}s")
                        self.conn.send(('success', (transcription, info)))
                    except Exception as e:
                        logging.error(f"General error in transcription: {e}", exc_info=True)
                        self.conn.send(('error', str(e)))
                except queue.Empty:
                    continue
                except KeyboardInterrupt:
                    self.interrupt_stop_event.set()
                    logging.debug("Transcription worker process finished due to KeyboardInterrupt")
                    break
                except Exception as e:
                    logging.error(f"General error in processing queue item: {e}", exc_info=True)
        finally:
            __builtins__['print'] = print
            self.conn.close()
            self.stdout_pipe.close()
            self.shutdown_event.set()
            polling_thread.join()


class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ENDC = '\033[0m'


class AudioToTextRecorder:
    """
    A class responsible for capturing audio from the microphone, detecting
    voice activity, and then transcribing the captured audio using the
    `faster_whisper` model.
    """

    def __init__(self,
                 model: str = INIT_MODEL_TRANSCRIPTION,
                 download_root: str = None,
                 language: str = "",
                 compute_type: str = "default",
                 input_device_index: int = None,
                 gpu_device_index: Union[int, List[int]] = 0,
                 device: str = "cuda",
                 on_recording_start=None,
                 on_recording_stop=None,
                 on_transcription_start=None,
                 ensure_sentence_starting_uppercase=True,
                 ensure_sentence_ends_with_period=True,
                 use_microphone=True,
                 spinner=True,
                 level=logging.WARNING,
                 batch_size: int = 16,

                 # Realtime transcription parameters
                 enable_realtime_transcription=False,
                 use_main_model_for_realtime=False,
                 realtime_model_type=INIT_MODEL_TRANSCRIPTION_REALTIME,
                 realtime_processing_pause=INIT_REALTIME_PROCESSING_PAUSE,
                 init_realtime_after_seconds=INIT_REALTIME_INITIAL_PAUSE,
                 on_realtime_transcription_update=None,
                 on_realtime_transcription_stabilized=None,
                 realtime_batch_size: int = 16,

                 # Voice activation parameters
                 silero_sensitivity: float = INIT_SILERO_SENSITIVITY,
                 silero_use_onnx: bool = False,
                 silero_deactivity_detection: bool = False,
                 webrtc_sensitivity: int = INIT_WEBRTC_SENSITIVITY,
                 post_speech_silence_duration: float = (
                     INIT_POST_SPEECH_SILENCE_DURATION
                 ),
                 min_length_of_recording: float = (
                     INIT_MIN_LENGTH_OF_RECORDING
                 ),
                 min_gap_between_recordings: float = (
                     INIT_MIN_GAP_BETWEEN_RECORDINGS
                 ),
                 pre_recording_buffer_duration: float = (
                     INIT_PRE_RECORDING_BUFFER_DURATION
                 ),
                 on_vad_start=None,
                 on_vad_stop=None,
                 on_vad_detect_start=None,
                 on_vad_detect_stop=None,
                 on_turn_detection_start=None,
                 on_turn_detection_stop=None,

                 # Wake word parameters
                 wakeword_backend: str = "",
                 openwakeword_model_paths: str = None,
                 openwakeword_inference_framework: str = "onnx",
                 wake_words: str = "",
                 wake_words_sensitivity: float = INIT_WAKE_WORDS_SENSITIVITY,
                 wake_word_activation_delay: float = (
                    INIT_WAKE_WORD_ACTIVATION_DELAY
                 ),
                 wake_word_timeout: float = INIT_WAKE_WORD_TIMEOUT,
                 wake_word_buffer_duration: float = INIT_WAKE_WORD_BUFFER_DURATION,
                 on_wakeword_detected=None,
                 on_wakeword_timeout=None,
                 on_wakeword_detection_start=None,
                 on_wakeword_detection_end=None,
                 on_recorded_chunk=None,
                 debug_mode=False,
                 handle_buffer_overflow: bool = INIT_HANDLE_BUFFER_OVERFLOW,
                 beam_size: int = 5,
                 beam_size_realtime: int = 3,
                 buffer_size: int = BUFFER_SIZE,
                 sample_rate: int = SAMPLE_RATE,
                 initial_prompt: Optional[Union[str, Iterable[int]]] = None,
                 initial_prompt_realtime: Optional[Union[str, Iterable[int]]] = None,
                 suppress_tokens: Optional[List[int]] = [-1],
                 print_transcription_time: bool = False,
                 early_transcription_on_silence: int = 0,
                 allowed_latency_limit: int = ALLOWED_LATENCY_LIMIT,
                 no_log_file: bool = False,
                 use_extended_logging: bool = False,
                 faster_whisper_vad_filter: bool = True,
                 normalize_audio: bool = False,
                 start_callback_in_new_thread: bool = False,
                 max_recording_duration: float = 0,
                 ):

        self.language = language
        self.compute_type = compute_type
        self.input_device_index = input_device_index
        self.gpu_device_index = gpu_device_index
        self.device = device
        self.wake_words = wake_words
        self.wake_word_activation_delay = wake_word_activation_delay
        self.wake_word_timeout = wake_word_timeout
        self.wake_word_buffer_duration = wake_word_buffer_duration
        self.ensure_sentence_starting_uppercase = (
            ensure_sentence_starting_uppercase
        )
        self.ensure_sentence_ends_with_period = (
            ensure_sentence_ends_with_period
        )
        self.use_microphone = mp.Value(c_bool, use_microphone)
        self.min_gap_between_recordings = min_gap_between_recordings
        self.min_length_of_recording = min_length_of_recording
        self.pre_recording_buffer_duration = pre_recording_buffer_duration
        self.post_speech_silence_duration = post_speech_silence_duration
        self.on_recording_start = on_recording_start
        self.on_recording_stop = on_recording_stop
        self.on_wakeword_detected = on_wakeword_detected
        self.on_wakeword_timeout = on_wakeword_timeout
        self.on_vad_start = on_vad_start
        self.on_vad_stop = on_vad_stop
        self.on_vad_detect_start = on_vad_detect_start
        self.on_vad_detect_stop = on_vad_detect_stop
        self.on_turn_detection_start = on_turn_detection_start
        self.on_turn_detection_stop = on_turn_detection_stop
        self.on_wakeword_detection_start = on_wakeword_detection_start
        self.on_wakeword_detection_end = on_wakeword_detection_end
        self.on_recorded_chunk = on_recorded_chunk
        self.on_transcription_start = on_transcription_start
        self.enable_realtime_transcription = enable_realtime_transcription
        self.use_main_model_for_realtime = use_main_model_for_realtime
        self.main_model_type = model
        if not download_root:
            download_root = None
        self.download_root = download_root
        self.realtime_model_type = realtime_model_type
        self.realtime_model = None  # FIX 13: separate attribute for model object
        self.realtime_processing_pause = realtime_processing_pause
        self.init_realtime_after_seconds = init_realtime_after_seconds
        self.on_realtime_transcription_update = (
            on_realtime_transcription_update
        )
        self.on_realtime_transcription_stabilized = (
            on_realtime_transcription_stabilized
        )
        self.debug_mode = debug_mode
        self.handle_buffer_overflow = handle_buffer_overflow
        self.beam_size = beam_size
        self.beam_size_realtime = beam_size_realtime
        self.allowed_latency_limit = allowed_latency_limit
        self.batch_size = batch_size
        self.realtime_batch_size = realtime_batch_size

        self.level = level
        self.audio_queue = mp.Queue()
        self.buffer_size = buffer_size
        self.sample_rate = sample_rate
        self.recording_start_time = 0
        self.recording_stop_time = 0
        self.last_recording_start_time = 0
        self.last_recording_stop_time = 0
        self.wake_word_detect_time = 0
        self.silero_check_time = 0
        self.silero_working = False
        self.speech_end_silence_start = 0
        self.silero_sensitivity = silero_sensitivity
        self.silero_deactivity_detection = silero_deactivity_detection
        self.listen_start = 0
        self.spinner = spinner
        self.halo = None
        self.state = "inactive"
        self.wakeword_detected = False
        self.text_storage = []
        self.realtime_stabilized_text = ""
        self.realtime_stabilized_safetext = ""
        self.is_webrtc_speech_active = False
        self.is_silero_speech_active = False
        self.recording_thread = None
        self.realtime_thread = None
        self.audio_interface = None
        self.audio = None
        self.stream = None
        self.start_recording_event = threading.Event()
        self.stop_recording_event = threading.Event()
        self.backdate_stop_seconds = 0.0
        self.backdate_resume_seconds = 0.0
        self.last_transcription_bytes = None
        self.last_transcription_bytes_b64 = None
        self.initial_prompt = initial_prompt
        self.initial_prompt_realtime = initial_prompt_realtime
        self.suppress_tokens = suppress_tokens
        self.use_wake_words = wake_words or wakeword_backend in {'oww', 'openwakeword', 'openwakewords'}
        self.detected_language = None
        self.detected_language_probability = 0
        self.detected_realtime_language = None
        self.detected_realtime_language_probability = 0
        self.transcription_lock = threading.Lock()
        self.shutdown_lock = threading.Lock()
        self.transcribe_count = 0
        self.print_transcription_time = print_transcription_time
        self.early_transcription_on_silence = early_transcription_on_silence
        self.use_extended_logging = use_extended_logging
        self.faster_whisper_vad_filter = faster_whisper_vad_filter
        self.normalize_audio = normalize_audio
        self.awaiting_speech_end = False
        self.start_callback_in_new_thread = start_callback_in_new_thread
        self.max_recording_duration = max_recording_duration

        # --------------------------------------------------------------------------
        # FIX 4: Prevent duplicate handlers on repeated instantiation
        logger.handlers.clear()

        logger.setLevel(logging.DEBUG)

        log_format = "RealTimeSTT: %(name)s - %(levelname)s - %(message)s"
        file_log_format = "%(asctime)s.%(msecs)03d - " + log_format

        console_handler = logging.StreamHandler()
        console_handler.setLevel(self.level)
        console_handler.setFormatter(logging.Formatter(log_format))

        logger.addHandler(console_handler)

        if not no_log_file:
            file_handler = logging.FileHandler('realtimesst.log')
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter(file_log_format, datefmt='%Y-%m-%d %H:%M:%S'))
            logger.addHandler(file_handler)
        # --------------------------------------------------------------------------

        self.is_shut_down = False
        self.shutdown_event = mp.Event()

        try:
            if mp.get_start_method(allow_none=True) is None:
                mp.set_start_method("spawn")
        except RuntimeError as e:
            logger.info(f"Start method has already been set. Details: {e}")

        logger.info("Starting RealTimeSTT")

        if use_extended_logging:
            logger.info("RealtimeSTT was called with these parameters:")
            for param, value in locals().items():
                logger.info(f"{param}: {value}")

        self.interrupt_stop_event = mp.Event()
        self.was_interrupted = mp.Event()
        self.main_transcription_ready_event = mp.Event()

        self.parent_transcription_pipe, child_transcription_pipe = SafePipe()
        self.parent_stdout_pipe, child_stdout_pipe = SafePipe()

        # Set device for model
        self.device = "cuda" if self.device == "cuda" and torch.cuda.is_available() else "cpu"

        self.transcript_process = self._start_thread(
            target=AudioToTextRecorder._transcription_worker,
            args=(
                child_transcription_pipe,
                child_stdout_pipe,
                self.main_model_type,
                self.download_root,
                self.compute_type,
                self.gpu_device_index,
                self.device,
                self.main_transcription_ready_event,
                self.shutdown_event,
                self.interrupt_stop_event,
                self.beam_size,
                self.initial_prompt,
                self.suppress_tokens,
                self.batch_size,
                self.faster_whisper_vad_filter,
                self.normalize_audio,
            )
        )

        # Start audio data reading process
        if self.use_microphone.value:
            logger.info("Initializing audio recording"
                         " (creating pyAudio input stream,"
                         f" sample rate: {self.sample_rate}"
                         f" buffer size: {self.buffer_size}"
                         )
            self.reader_process = self._start_thread(
                target=AudioToTextRecorder._audio_data_worker,
                args=(
                    self.audio_queue,
                    self.sample_rate,
                    self.buffer_size,
                    self.input_device_index,
                    self.shutdown_event,
                    self.interrupt_stop_event,
                    self.use_microphone
                )
            )

        # Initialize the realtime transcription model
        # FIX 13: Use self.realtime_model for the object, keep self.realtime_model_type as string
        if self.enable_realtime_transcription and not self.use_main_model_for_realtime:
            try:
                logger.info("Initializing faster_whisper realtime "
                             f"transcription model {self.realtime_model_type}, "
                             f"default device: {self.device}, "
                             f"compute type: {self.compute_type}, "
                             f"device index: {self.gpu_device_index}, "
                             f"download root: {self.download_root}"
                             )
                self.realtime_model = faster_whisper.WhisperModel(
                    model_size_or_path=self.realtime_model_type,
                    device=self.device,
                    compute_type=self.compute_type,
                    device_index=self.gpu_device_index,
                    download_root=self.download_root,
                )
                if self.realtime_batch_size > 0:
                    self.realtime_model = BatchedInferencePipeline(model=self.realtime_model)

                # Run a warm-up transcription
                current_dir = os.path.dirname(os.path.realpath(__file__))
                warmup_audio_path = os.path.join(
                    current_dir, "warmup_audio.wav"
                )
                warmup_audio_data, _ = sf.read(warmup_audio_path, dtype="float32")
                segments, info = self.realtime_model.transcribe(warmup_audio_data, language="en", beam_size=1)
                model_warmup_transcription = " ".join(segment.text for segment in segments)
            except Exception as e:
                logger.exception("Error initializing faster_whisper "
                                  f"realtime transcription model: {e}"
                                  )
                raise

            logger.debug("Faster_whisper realtime speech to text "
                          "transcription model initialized successfully")

        # Setup wake word detection
        if wake_words or wakeword_backend in {'oww', 'openwakeword', 'openwakewords', 'pvp', 'pvporcupine'}:
            self.wakeword_backend = wakeword_backend

            self.wake_words_list = [
                word.strip() for word in wake_words.lower().split(',')
            ]
            self.wake_words_sensitivity = wake_words_sensitivity
            self.wake_words_sensitivities = [
                float(wake_words_sensitivity)
                for _ in range(len(self.wake_words_list))
            ]

            if wake_words and self.wakeword_backend in {'pvp', 'pvporcupine'}:

                try:
                    self.porcupine = pvporcupine.create(
                        keywords=self.wake_words_list,
                        sensitivities=self.wake_words_sensitivities
                    )
                    self.buffer_size = self.porcupine.frame_length
                    self.sample_rate = self.porcupine.sample_rate

                except Exception as e:
                    logger.exception(
                        "Error initializing porcupine "
                        f"wake word detection engine: {e}. "
                        f"Wakewords: {self.wake_words_list}."
                    )
                    raise

                logger.debug(
                    "Porcupine wake word detection engine initialized successfully"
                )

            elif wake_words and self.wakeword_backend in {'oww', 'openwakeword', 'openwakewords'}:

                openwakeword.utils.download_models()

                try:
                    if openwakeword_model_paths:
                        model_paths = openwakeword_model_paths.split(',')
                        self.owwModel = Model(
                            wakeword_models=model_paths,
                            inference_framework=openwakeword_inference_framework
                        )
                        logger.info(
                            "Successfully loaded wakeword model(s): "
                            f"{openwakeword_model_paths}"
                        )
                    else:
                        self.owwModel = Model(
                            inference_framework=openwakeword_inference_framework)

                    self.oww_n_models = len(self.owwModel.models.keys())
                    if not self.oww_n_models:
                        logger.error(
                            "No wake word models loaded."
                        )

                    for model_key in self.owwModel.models.keys():
                        logger.info(
                            "Successfully loaded openwakeword model: "
                            f"{model_key}"
                        )

                except Exception as e:
                    logger.exception(
                        "Error initializing openwakeword "
                        f"wake word detection engine: {e}"
                    )
                    raise

                logger.debug(
                    "Open wake word detection engine initialized successfully"
                )

            else:
                logger.exception(f"Wakeword engine {self.wakeword_backend} unknown/unsupported or wake_words not specified. Please specify one of: pvporcupine, openwakeword.")


        # Setup voice activity detection model WebRTC
        try:
            logger.info("Initializing WebRTC voice with "
                         f"Sensitivity {webrtc_sensitivity}"
                         )
            self.webrtc_vad_model = webrtcvad.Vad()
            self.webrtc_vad_model.set_mode(webrtc_sensitivity)

        except Exception as e:
            logger.exception("Error initializing WebRTC voice "
                              f"activity detection engine: {e}"
                              )
            raise

        logger.debug("WebRTC VAD voice activity detection "
                      "engine initialized successfully"
                      )

        # Setup voice activity detection model Silero VAD
        try:
            self.silero_vad_model, _ = torch.hub.load(
                repo_or_dir="snakers4/silero-vad",
                model="silero_vad",
                verbose=False,
                onnx=silero_use_onnx
            )

        except Exception as e:
            logger.exception(f"Error initializing Silero VAD "
                              f"voice activity detection engine: {e}"
                              )
            raise

        logger.debug("Silero VAD voice activity detection "
                      "engine initialized successfully"
                      )

        self.audio_buffer = collections.deque(
            maxlen=int((self.sample_rate // self.buffer_size) *
                       self.pre_recording_buffer_duration)
        )
        self.last_words_buffer = collections.deque(
            maxlen=int((self.sample_rate // self.buffer_size) *
                       0.3)
        )
        self.frames = []
        self.last_frames = []

        # Recording control flags
        self.is_recording = False
        self.is_running = True
        self.start_recording_on_voice_activity = False
        self.stop_recording_on_voice_deactivity = False

        # Start the recording worker thread
        self.recording_thread = threading.Thread(target=self._recording_worker)
        self.recording_thread.daemon = True
        self.recording_thread.start()

        # Start the realtime transcription worker thread
        self.realtime_thread = threading.Thread(target=self._realtime_worker)
        self.realtime_thread.daemon = True
        self.realtime_thread.start()

        # Wait for transcription models to start
        logger.debug('Waiting for main transcription model to start')
        self.main_transcription_ready_event.wait()
        logger.debug('Main transcription model ready')

        self.stdout_thread = threading.Thread(target=self._read_stdout)
        self.stdout_thread.daemon = True
        self.stdout_thread.start()

        logger.debug('RealtimeSTT initialization completed successfully')

    def _start_thread(self, target=None, args=()):
        """
        Implement a consistent threading model across the library.
        """
        if (platform.system() == 'Linux'):
            thread = threading.Thread(target=target, args=args)
            thread.daemon = True  # FIX 2: was "deamon"
            thread.start()
            return thread
        else:
            thread = mp.Process(target=target, args=args)
            thread.start()
            return thread

    def _read_stdout(self):
        while not self.shutdown_event.is_set():
            try:
                if self.parent_stdout_pipe.poll(0.1):
                    logger.debug("Receive from stdout pipe")
                    message = self.parent_stdout_pipe.recv()
                    logger.info(message)
            except (BrokenPipeError, EOFError, OSError):
                pass
            except KeyboardInterrupt:
                logger.info("KeyboardInterrupt in read from stdout detected, exiting...")
                break
            except Exception as e:
                logger.error(f"Unexpected error in read from stdout: {e}", exc_info=True)
                logger.error(traceback.format_exc())
                break
            time.sleep(0.1)

    @staticmethod  # FIX 1: was missing @staticmethod
    def _transcription_worker(*args, **kwargs):
        worker = TranscriptionWorker(*args, **kwargs)
        worker.run()

    def _run_callback(self, cb, *args, **kwargs):
        if self.start_callback_in_new_thread:
            threading.Thread(target=cb, args=args, kwargs=kwargs, daemon=True).start()
        else:
            cb(*args, **kwargs)

    @staticmethod
    def _audio_data_worker(
        audio_queue,
        target_sample_rate,
        buffer_size,
        input_device_index,
        shutdown_event,
        interrupt_stop_event,
        use_microphone
    ):
        """
        Worker method that handles the audio recording process.
        """
        import pyaudio
        import numpy as np
        from scipy import signal

        if __name__ == '__main__':
            system_signal.signal(system_signal.SIGINT, system_signal.SIG_IGN)

        def get_highest_sample_rate(audio_interface, device_index):
            """Get the highest supported sample rate for the specified device."""
            try:
                device_info = audio_interface.get_device_info_by_index(device_index)
                logger.debug(f"Retrieving highest sample rate for device index {device_index}: {device_info}")
                max_rate = int(device_info['defaultSampleRate'])

                if 'supportedSampleRates' in device_info:
                    supported_rates = [int(rate) for rate in device_info['supportedSampleRates']]
                    if supported_rates:
                        max_rate = max(supported_rates)

                logger.debug(f"Highest supported sample rate for device index {device_index} is {max_rate}")
                return max_rate
            except Exception as e:
                logger.warning(f"Failed to get highest sample rate: {e}")
                return 48000

        def initialize_audio_stream(audio_interface, sample_rate, chunk_size):
            nonlocal input_device_index

            def validate_device(device_index):
                """Validate that the device exists and is actually available for input."""
                try:
                    device_info = audio_interface.get_device_info_by_index(device_index)
                    logger.debug(f"Validating device index {device_index} with info: {device_info}")
                    if not device_info.get('maxInputChannels', 0) > 0:
                        logger.debug("Device has no input channels, invalid for recording.")
                        return False

                    test_stream = audio_interface.open(
                        format=pyaudio.paInt16,
                        channels=1,
                        rate=target_sample_rate,
                        input=True,
                        frames_per_buffer=chunk_size,
                        input_device_index=device_index,
                        start=False
                    )

                    test_stream.start_stream()
                    test_data = test_stream.read(chunk_size, exception_on_overflow=False)
                    test_stream.stop_stream()
                    test_stream.close()

                    if len(test_data) == 0:
                        logger.debug("Device produced no data, invalid for recording.")
                        return False

                    logger.debug(f"Device index {device_index} successfully validated.")
                    return True

                except Exception as e:
                    logger.debug(f"Device validation failed for index {device_index}: {e}")
                    return False

            """Initialize the audio stream with error handling."""
            while not shutdown_event.is_set():
                try:
                    input_devices = []
                    device_count = audio_interface.get_device_count()
                    logger.debug(f"Found {device_count} total audio devices on the system.")
                    for i in range(device_count):
                        try:
                            device_info = audio_interface.get_device_info_by_index(i)
                            if device_info.get('maxInputChannels', 0) > 0:
                                input_devices.append(i)
                        except Exception as e:
                            logger.debug(f"Could not retrieve info for device index {i}: {e}")
                            continue

                    logger.debug(f"Available input devices with input channels: {input_devices}")
                    if not input_devices:
                        raise Exception("No input devices found")

                    if input_device_index is None or input_device_index not in input_devices:
                        try:
                            default_device = audio_interface.get_default_input_device_info()
                            logger.debug(f"Default device info: {default_device}")
                            if validate_device(default_device['index']):
                                input_device_index = default_device['index']
                                logger.debug(f"Default device {input_device_index} selected.")
                        except Exception:
                            logger.debug("Default device validation failed, checking other devices...")
                            for device_index in input_devices:
                                if validate_device(device_index):
                                    input_device_index = device_index
                                    logger.debug(f"Device {input_device_index} selected.")
                                    break
                            else:
                                raise Exception("No working input devices found")

                    if not validate_device(input_device_index):
                        raise Exception("Selected device validation failed")

                    logger.debug(f"Opening stream with device index {input_device_index}, "
                                f"sample_rate={sample_rate}, chunk_size={chunk_size}")
                    stream = audio_interface.open(
                        format=pyaudio.paInt16,
                        channels=1,
                        rate=sample_rate,
                        input=True,
                        frames_per_buffer=chunk_size,
                        input_device_index=input_device_index,
                    )

                    logger.info(f"Microphone connected and validated (device index: {input_device_index}, "
                                f"sample rate: {sample_rate}, chunk size: {chunk_size})")
                    return stream

                except Exception as e:
                    logger.error(f"Microphone connection failed: {e}. Retrying...", exc_info=True)
                    input_device_index = None
                    time.sleep(3)
                    continue

        def preprocess_audio(chunk, original_sample_rate, target_sample_rate):
            """Preprocess audio chunk similar to feed_audio method."""
            if isinstance(chunk, np.ndarray):
                if chunk.ndim == 2:
                    chunk = np.mean(chunk, axis=1)

                if original_sample_rate != target_sample_rate:
                    logger.debug(f"Resampling from {original_sample_rate} Hz to {target_sample_rate} Hz.")
                    num_samples = int(len(chunk) * target_sample_rate / original_sample_rate)
                    chunk = signal.resample(chunk, num_samples)

                chunk = chunk.astype(np.int16)
            else:
                chunk = np.frombuffer(chunk, dtype=np.int16)

                if original_sample_rate != target_sample_rate:
                    logger.debug(f"Resampling from {original_sample_rate} Hz to {target_sample_rate} Hz.")
                    num_samples = int(len(chunk) * target_sample_rate / original_sample_rate)
                    chunk = signal.resample(chunk, num_samples)
                    chunk = chunk.astype(np.int16)

            return chunk.tobytes()

        audio_interface = None
        stream = None
        device_sample_rate = None
        chunk_size = 1024

        def setup_audio():
            nonlocal audio_interface, stream, device_sample_rate, input_device_index
            try:
                if audio_interface is None:
                    logger.debug("Creating PyAudio interface...")
                    audio_interface = pyaudio.PyAudio()

                if input_device_index is None:
                    try:
                        default_device = audio_interface.get_default_input_device_info()
                        input_device_index = default_device['index']
                        logger.debug(f"No device index supplied; using default device {input_device_index}")
                    except OSError as e:
                        logger.debug(f"Default device retrieval failed: {e}")
                        input_device_index = None

                sample_rates_to_try = [16000]
                if input_device_index is not None:
                    highest_rate = get_highest_sample_rate(audio_interface, input_device_index)
                    if highest_rate != 16000:
                        sample_rates_to_try.append(highest_rate)
                else:
                    sample_rates_to_try.append(48000)

                logger.debug(f"Sample rates to try for device {input_device_index}: {sample_rates_to_try}")

                for rate in sample_rates_to_try:
                    try:
                        device_sample_rate = rate
                        logger.debug(f"Attempting to initialize audio stream at {device_sample_rate} Hz.")
                        stream = initialize_audio_stream(audio_interface, device_sample_rate, chunk_size)
                        if stream is not None:
                            logger.debug(
                                f"Audio recording initialized successfully at {device_sample_rate} Hz, "
                                f"reading {chunk_size} frames at a time"
                            )
                            return True
                    except Exception as e:
                        logger.warning(f"Failed to initialize audio stream at {device_sample_rate} Hz: {e}")
                        continue

                raise Exception("Failed to initialize audio stream with all sample rates.")

            except Exception as e:
                logger.exception(f"Error initializing pyaudio audio recording: {e}")
                if audio_interface:
                    audio_interface.terminate()
                return False

        logger.debug(f"Starting audio data worker with target_sample_rate={target_sample_rate}, "
                    f"buffer_size={buffer_size}, input_device_index={input_device_index}")

        if not setup_audio():
            raise Exception("Failed to set up audio recording.")

        buffer = bytearray()
        silero_buffer_size = 2 * buffer_size

        time_since_last_buffer_message = 0

        try:
            while not shutdown_event.is_set():
                try:
                    data = stream.read(chunk_size, exception_on_overflow=False)

                    if use_microphone.value:
                        processed_data = preprocess_audio(data, device_sample_rate, target_sample_rate)
                        buffer += processed_data

                        while len(buffer) >= silero_buffer_size:
                            to_process = buffer[:silero_buffer_size]
                            buffer = buffer[silero_buffer_size:]

                            if time_since_last_buffer_message:
                                time_passed = time.time() - time_since_last_buffer_message
                                if time_passed > 1:
                                    logger.debug("_audio_data_worker writing audio data into queue.")
                                    time_since_last_buffer_message = time.time()
                            else:
                                time_since_last_buffer_message = time.time()

                            audio_queue.put(to_process)

                except OSError as e:
                    if e.errno == pyaudio.paInputOverflowed:
                        logger.warning("Input overflowed. Frame dropped.")
                    else:
                        logger.error(f"OSError during recording: {e}", exc_info=True)
                        logger.error("Attempting to reinitialize the audio stream...")

                        try:
                            if stream:
                                stream.stop_stream()
                                stream.close()
                        except Exception:
                            pass

                        time.sleep(1)
                        if not setup_audio():
                            logger.error("Failed to reinitialize audio stream. Exiting.")
                            break
                        else:
                            logger.error("Audio stream reinitialized successfully.")
                    continue

                except Exception as e:
                    logger.error(f"Unknown error during recording: {e}")
                    tb_str = traceback.format_exc()
                    logger.error(f"Traceback: {tb_str}")
                    logger.error(f"Error: {e}")
                    logger.info("Attempting to reinitialize the audio stream...")
                    try:
                        if stream:
                            stream.stop_stream()
                            stream.close()
                    except Exception:
                        pass

                    time.sleep(1)
                    if not setup_audio():
                        logger.error("Failed to reinitialize audio stream. Exiting.")
                        break
                    else:
                        logger.info("Audio stream reinitialized successfully.")
                    continue

        except KeyboardInterrupt:
            interrupt_stop_event.set()
            logger.debug("Audio data worker process finished due to KeyboardInterrupt")
        finally:
            if buffer:
                audio_queue.put(bytes(buffer))

            try:
                if stream:
                    stream.stop_stream()
                    stream.close()
            except Exception:
                pass
            if audio_interface:
                audio_interface.terminate()

    def wakeup(self):
        """
        If in wake work modus, wake up as if a wake word was spoken.
        """
        self.listen_start = time.time()

    def abort(self):
        # FIX 9: Reordered to prevent deadlock; added timeout
        self.start_recording_on_voice_activity = False
        self.stop_recording_on_voice_deactivity = False
        self.interrupt_stop_event.set()
        if self.is_recording:
            self.stop()
        if self.state != "inactive":
            if not self.was_interrupted.wait(timeout=5.0):
                logger.warning("abort() timed out waiting for was_interrupted event")
            self._set_state("inactive")
        self.was_interrupted.clear()

    def wait_audio(self):
        """
        Waits for the start and completion of the audio recording process.
        """

        try:
            logger.info("Setting listen time")
            if self.listen_start == 0:
                self.listen_start = time.time()

            if not self.is_recording and not self.frames:
                self._set_state("listening")
                self.start_recording_on_voice_activity = True

                logger.debug('Waiting for recording start')
                while not self.interrupt_stop_event.is_set():
                    if self.start_recording_event.wait(timeout=0.02):
                        break

            if self.is_recording:
                self.stop_recording_on_voice_deactivity = True

                logger.debug('Waiting for recording stop')
                while not self.interrupt_stop_event.is_set():
                    if (self.stop_recording_event.wait(timeout=0.02)):
                        break

            frames = self.frames
            if len(frames) == 0:
                frames = self.last_frames

            samples_to_keep = int(self.sample_rate * self.backdate_resume_seconds)

            full_audio_array = np.frombuffer(b''.join(frames), dtype=np.int16)
            full_audio = full_audio_array.astype(np.float32) / INT16_MAX_ABS_VALUE

            if samples_to_keep > 0:
                samples_to_keep = min(samples_to_keep, len(full_audio))
                frames_to_read_audio = full_audio[-samples_to_keep:]

                frames_to_read_int16 = (frames_to_read_audio * INT16_MAX_ABS_VALUE).astype(np.int16)
                frame_bytes = frames_to_read_int16.tobytes()

                FRAME_SIZE = 2048
                frames_to_read = []
                for i in range(0, len(frame_bytes), FRAME_SIZE):
                    frame = frame_bytes[i:i + FRAME_SIZE]
                    if frame:
                        frames_to_read.append(frame)
            else:
                frames_to_read = []

            samples_to_remove = int(self.sample_rate * self.backdate_stop_seconds)

            if samples_to_remove > 0:
                if samples_to_remove < len(full_audio):
                    self.audio = full_audio[:-samples_to_remove]
                    logger.debug(f"Removed {samples_to_remove} samples "
                        f"({samples_to_remove/self.sample_rate:.3f}s) from end of audio")
                else:
                    self.audio = np.array([], dtype=np.float32)
                    logger.debug("Cleared audio (samples_to_remove >= audio length)")
            else:
                self.audio = full_audio
                logger.debug(f"No samples removed, final audio length: {len(self.audio)}")

            self.frames.clear()
            self.last_frames.clear()
            self.frames.extend(frames_to_read)

            self.backdate_stop_seconds = 0.0
            self.backdate_resume_seconds = 0.0

            self.listen_start = 0

            self._set_state("inactive")

        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt in wait_audio, shutting down")
            self.shutdown()
            raise


    def perform_final_transcription(self, audio_bytes=None, use_prompt=True):
        start_time = 0
        with self.transcription_lock:
            if audio_bytes is None:
                audio_bytes = copy.deepcopy(self.audio)

            if audio_bytes is None or len(audio_bytes) == 0:
                print("No audio data available for transcription")
                return ""

            try:
                if self.transcribe_count == 0:
                    logger.debug("Adding transcription request, no early transcription started")
                    start_time = time.time()
                    self.parent_transcription_pipe.send((audio_bytes, self.language, use_prompt))
                    self.transcribe_count += 1

                while self.transcribe_count > 0:
                    logger.debug(f"Receive from parent_transcription_pipe after sending transcription request, transcribe_count: {self.transcribe_count}")
                    if not self.parent_transcription_pipe.poll(0.1):
                        if self.interrupt_stop_event.is_set():
                            self.was_interrupted.set()
                            self._set_state("inactive")
                            return ""
                        continue
                    status, result = self.parent_transcription_pipe.recv()
                    self.transcribe_count -= 1

                self.allowed_to_early_transcribe = True
                self._set_state("inactive")
                if status == 'success':
                    transcription_text, info = result  # FIX 10: clearer variable name
                    self.detected_language = info.language if info.language_probability > 0 else None
                    self.detected_language_probability = info.language_probability
                    self.last_transcription_bytes = copy.deepcopy(audio_bytes)
                    self.last_transcription_bytes_b64 = base64.b64encode(self.last_transcription_bytes.tobytes()).decode('utf-8')
                    transcription = self._preprocess_output(transcription_text)
                    end_time = time.time()
                    transcription_time = end_time - start_time

                    if start_time:
                        if self.print_transcription_time:
                            print(f"Model {self.main_model_type} completed transcription in {transcription_time:.2f} seconds")
                        else:
                            logger.debug(f"Model {self.main_model_type} completed transcription in {transcription_time:.2f} seconds")
                    return "" if self.interrupt_stop_event.is_set() else transcription
                else:
                    logger.error(f"Transcription error: {result}")
                    raise Exception(result)
            except Exception as e:
                logger.error(f"Error during transcription: {str(e)}", exc_info=True)
                raise e


    def transcribe(self):
        """
        Transcribes audio captured by this class instance using the
        `faster_whisper` model.
        """
        audio_copy = copy.deepcopy(self.audio)
        self._set_state("transcribing")
        if self.on_transcription_start:
            abort_value = self.on_transcription_start(audio_copy)
            if not abort_value:
                return self.perform_final_transcription(audio_copy)
            return None
        else:
            return self.perform_final_transcription(audio_copy)


    def _process_wakeword(self, data):
        """
        Processes audio data to detect wake words.
        """
        if self.wakeword_backend in {'pvp', 'pvporcupine'}:
            pcm = struct.unpack_from(
                "h" * self.buffer_size,
                data
            )
            porcupine_index = self.porcupine.process(pcm)
            if self.debug_mode:
                logger.info(f"wake words porcupine_index: {porcupine_index}")
            return porcupine_index

        elif self.wakeword_backend in {'oww', 'openwakeword', 'openwakewords'}:
            pcm = np.frombuffer(data, dtype=np.int16)
            prediction = self.owwModel.predict(pcm)
            max_score = -1
            max_index = -1
            wake_words_in_prediction = len(self.owwModel.prediction_buffer.keys())
            # FIX 6C: Removed dead no-op line `self.wake_words_sensitivities`
            if wake_words_in_prediction:
                for idx, mdl in enumerate(self.owwModel.prediction_buffer.keys()):
                    scores = list(self.owwModel.prediction_buffer[mdl])
                    if scores[-1] >= self.wake_words_sensitivity and scores[-1] > max_score:
                        max_score = scores[-1]
                        max_index = idx
                if self.debug_mode:
                    logger.info(f"wake words oww max_index, max_score: {max_index} {max_score}")
                return max_index
            else:
                if self.debug_mode:
                    logger.info(f"wake words oww_index: -1")
                return -1

        if self.debug_mode:
            logger.info("wake words no match")

        return -1

    def text(self,
             on_transcription_finished=None,
             ):
        """
        Transcribes audio captured by this class instance
        using the `faster_whisper` model.
        """
        self.interrupt_stop_event.clear()
        self.was_interrupted.clear()
        try:
            self.wait_audio()
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt in text() method")
            self.shutdown()
            raise

        if self.is_shut_down or self.interrupt_stop_event.is_set():
            if self.interrupt_stop_event.is_set():
                self.was_interrupted.set()
            return ""

        if on_transcription_finished:
            threading.Thread(target=on_transcription_finished,
                            args=(self.transcribe(),)).start()
        else:
            return self.transcribe()


    def format_number(self, num):
        num_str = f"{num:.10f}"
        integer_part, decimal_part = num_str.split('.')
        result = f"{integer_part[-2:]}.{decimal_part[:2]}"
        return result

    def start(self, frames=None):
        """
        Starts recording audio directly without waiting for voice activity.
        """
        if (time.time() - self.recording_stop_time
                < self.min_gap_between_recordings):
            logger.info("Attempted to start recording "
                         "too soon after stopping."
                         )
            return self

        logger.info("recording started")
        self._set_state("recording")
        self.text_storage = []
        self.realtime_stabilized_text = ""
        self.realtime_stabilized_safetext = ""
        self.wakeword_detected = False
        self.wake_word_detect_time = 0
        self.frames = []
        if frames:
            self.frames = frames
        self.is_recording = True

        self.recording_start_time = time.time()
        self.is_silero_speech_active = False
        self.is_webrtc_speech_active = False
        self.stop_recording_event.clear()
        self.start_recording_event.set()

        if self.on_recording_start:
            self._run_callback(self.on_recording_start)

        return self

    def stop(self,
             backdate_stop_seconds: float = 0.0,
             backdate_resume_seconds: float = 0.0,
        ):
        """
        Stops recording audio.
        """
        if (time.time() - self.recording_start_time
                < self.min_length_of_recording):
            logger.info("Attempted to stop recording "
                         "too soon after starting."
                         )
            return self

        logger.info("recording stopped")
        self.last_frames = copy.deepcopy(self.frames)
        self.backdate_stop_seconds = backdate_stop_seconds
        self.backdate_resume_seconds = backdate_resume_seconds
        self.is_recording = False
        self.recording_stop_time = time.time()
        self.is_silero_speech_active = False
        self.is_webrtc_speech_active = False
        self.silero_check_time = 0
        self.start_recording_event.clear()
        self.stop_recording_event.set()

        self.last_recording_start_time = self.recording_start_time
        self.last_recording_stop_time = self.recording_stop_time

        if self.on_recording_stop:
            self._run_callback(self.on_recording_stop)

        return self

    def listen(self):
        """
        Puts recorder in immediate "listen" state.
        """
        self.listen_start = time.time()
        self._set_state("listening")
        self.start_recording_on_voice_activity = True

    def feed_audio(self, chunk, original_sample_rate=16000):
        """
        Feed an audio chunk into the processing pipeline.
        """
        # FIX 15: Warn if microphone is active to prevent interleaved corruption
        if self.use_microphone.value:
            logger.warning(
                "feed_audio() called while microphone is active. "
                "Disable microphone first with set_microphone(False) "
                "to avoid interleaved audio corruption."
            )

        if not hasattr(self, '_feed_buffer'):
            self._feed_buffer = bytearray()

        if isinstance(chunk, np.ndarray):
            if chunk.ndim == 2:
                chunk = np.mean(chunk, axis=1)

            if original_sample_rate != 16000:
                num_samples = int(len(chunk) * 16000 / original_sample_rate)
                chunk = resample(chunk, num_samples)

            chunk = chunk.astype(np.int16)
            chunk = chunk.tobytes()

        self._feed_buffer += chunk
        buf_size = 2 * self.buffer_size

        while len(self._feed_buffer) >= buf_size:
            to_process = self._feed_buffer[:buf_size]
            self._feed_buffer = self._feed_buffer[buf_size:]
            self.audio_queue.put(to_process)

    def set_microphone(self, microphone_on=True):
        """
        Set the microphone on or off.
        """
        logger.info("Setting microphone to: " + str(microphone_on))
        self.use_microphone.value = microphone_on

    def _safe_join_and_terminate(self, proc, name, timeout=10):
        """Safely join a thread or process, force-terminating if needed."""
        # FIX 3: Handle both Thread and Process safely
        if proc is None:
            return
        proc.join(timeout=timeout)
        if hasattr(proc, 'terminate') and callable(proc.terminate) and proc.is_alive():
            logger.warning(f"{name} did not terminate in time. Terminating forcefully.")
            proc.terminate()

    def shutdown(self):
        """
        Safely shuts down the audio recording by stopping the
        recording worker and closing the audio stream.
        """

        with self.shutdown_lock:
            if self.is_shut_down:
                return

            print("\033[91mRealtimeSTT shutting down\033[0m")

            self.is_shut_down = True
            self.start_recording_event.set()
            self.stop_recording_event.set()

            self.shutdown_event.set()
            self.is_recording = False
            self.is_running = False

            logger.debug('Finishing recording thread')
            if self.recording_thread:
                self.recording_thread.join()

            # FIX 3: Guard against missing reader_process
            logger.debug('Terminating reader process')
            if hasattr(self, 'reader_process'):
                self._safe_join_and_terminate(self.reader_process, "Reader process")

            logger.debug('Terminating transcription process')
            self._safe_join_and_terminate(self.transcript_process, "Transcript process")

            self.parent_transcription_pipe.close()
            self.parent_stdout_pipe.close()  # FIX 14: was never closed

            logger.debug('Finishing realtime thread')
            if self.realtime_thread:
                self.realtime_thread.join()

            if self.enable_realtime_transcription:
                # FIX 13: use self.realtime_model
                if self.realtime_model:
                    del self.realtime_model
                    self.realtime_model = None
            gc.collect()

    def _recording_worker(self):
        """
        The main worker method which constantly monitors the audio
        input for voice activity and accordingly starts/stops the recording.
        """

        if self.use_extended_logging:
            logger.debug('Debug: Entering try block')

        last_inner_try_time = 0
        try:
            if self.use_extended_logging:
                logger.debug('Debug: Initializing variables')
            time_since_last_buffer_message = 0
            was_recording = False
            delay_was_passed = False
            wakeword_detected_time = None
            wakeword_samples_to_remove = None
            self.allowed_to_early_transcribe = True

            if self.use_extended_logging:
                logger.debug('Debug: Starting main loop')

            while self.is_running:

                if last_inner_try_time:
                    last_processing_time = time.time() - last_inner_try_time
                    if last_processing_time > 0.1:
                        if self.use_extended_logging:
                            logger.warning('### WARNING: PROCESSING TOOK TOO LONG')
                last_inner_try_time = time.time()
                try:
                    try:
                        data = self.audio_queue.get(timeout=0.01)
                        self.last_words_buffer.append(data)
                    except queue.Empty:
                        if not self.is_running:
                            if self.use_extended_logging:
                                logger.debug('Debug: Not running, breaking loop')
                            break
                        continue

                    if self.use_extended_logging:
                        logger.debug('Debug: Checking for on_recorded_chunk callback')
                    if self.on_recorded_chunk:
                        if self.use_extended_logging:
                            logger.debug('Debug: Calling on_recorded_chunk')
                        self._run_callback(self.on_recorded_chunk, data)

                    if self.use_extended_logging:
                        logger.debug('Debug: Checking if handle_buffer_overflow is True')
                    if self.handle_buffer_overflow:
                        if self.use_extended_logging:
                            logger.debug('Debug: Handling buffer overflow')
                        if (self.audio_queue.qsize() >
                                self.allowed_latency_limit):
                            if self.use_extended_logging:
                                logger.debug('Debug: Queue size exceeds limit, logging warnings')
                            logger.warning("Audio queue size exceeds "
                                            "latency limit. Current size: "
                                            f"{self.audio_queue.qsize()}. "
                                            "Discarding old audio chunks."
                                            )

                        if self.use_extended_logging:
                            logger.debug('Debug: Discarding old chunks if necessary')
                        while (self.audio_queue.qsize() >
                                self.allowed_latency_limit):

                            data = self.audio_queue.get()

                except BrokenPipeError:
                    logger.error("BrokenPipeError _recording_worker", exc_info=True)
                    self.is_running = False
                    break

                if self.use_extended_logging:
                    logger.debug('Debug: Updating time_since_last_buffer_message')
                if time_since_last_buffer_message:
                    time_passed = time.time() - time_since_last_buffer_message
                    if time_passed > 1:
                        if self.use_extended_logging:
                            logger.debug("_recording_worker processing audio data")
                        time_since_last_buffer_message = time.time()
                else:
                    time_since_last_buffer_message = time.time()

                if self.use_extended_logging:
                    logger.debug('Debug: Initializing failed_stop_attempt')
                failed_stop_attempt = False

                if self.use_extended_logging:
                    logger.debug('Debug: Checking if not recording')
                if not self.is_recording:
                    if self.use_extended_logging:
                        logger.debug('Debug: Handling not recording state')
                    time_since_listen_start = (time.time() - self.listen_start
                                            if self.listen_start else 0)

                    wake_word_activation_delay_passed = (
                        time_since_listen_start >
                        self.wake_word_activation_delay
                    )

                    if self.use_extended_logging:
                        logger.debug('Debug: Handling wake-word timeout callback')
                    if wake_word_activation_delay_passed \
                            and not delay_was_passed:

                        if self.use_wake_words and self.wake_word_activation_delay:
                            if self.on_wakeword_timeout:
                                if self.use_extended_logging:
                                    logger.debug('Debug: Calling on_wakeword_timeout')
                                self._run_callback(self.on_wakeword_timeout)
                    delay_was_passed = wake_word_activation_delay_passed

                    if self.use_extended_logging:
                        logger.debug('Debug: Setting state and spinner text')
                    if not self.recording_stop_time:
                        if self.use_wake_words \
                                and wake_word_activation_delay_passed \
                                and not self.wakeword_detected:
                            if self.use_extended_logging:
                                logger.debug('Debug: Setting state to "wakeword"')
                            self._set_state("wakeword")
                        else:
                            if self.listen_start:
                                if self.use_extended_logging:
                                    logger.debug('Debug: Setting state to "listening"')
                                self._set_state("listening")
                            else:
                                if self.use_extended_logging:
                                    logger.debug('Debug: Setting state to "inactive"')
                                self._set_state("inactive")

                    if self.use_extended_logging:
                        logger.debug('Debug: Checking wake word conditions')
                    # FIX 6A: Added `not self.wakeword_detected` guard to prevent multiple detections
                    if self.use_wake_words \
                            and wake_word_activation_delay_passed \
                            and not self.wakeword_detected:
                        try:
                            if self.use_extended_logging:
                                logger.debug('Debug: Processing wakeword')
                            wakeword_index = self._process_wakeword(data)

                        except struct.error:
                            logger.error("Error unpacking audio data "
                                        "for wake word processing.", exc_info=True)
                            continue

                        except Exception as e:
                            logger.error(f"Wake word processing error: {e}", exc_info=True)
                            continue

                        if self.use_extended_logging:
                            logger.debug('Debug: Checking if wake word detected')
                        if wakeword_index >= 0:
                            if self.use_extended_logging:
                                logger.debug('Debug: Wake word detected, updating variables')
                            self.wake_word_detect_time = time.time()
                            wakeword_detected_time = time.time()
                            wakeword_samples_to_remove = int(self.sample_rate * self.wake_word_buffer_duration)
                            self.wakeword_detected = True

                            # FIX 6B: Reset openwakeword buffer to prevent re-triggering
                            if self.wakeword_backend in {'oww', 'openwakeword', 'openwakewords'}:
                                self.owwModel.reset()

                            if self.on_wakeword_detected:
                                if self.use_extended_logging:
                                    logger.debug('Debug: Calling on_wakeword_detected')
                                self._run_callback(self.on_wakeword_detected)

                    if self.use_extended_logging:
                        logger.debug('Debug: Checking voice activity conditions')

                    # FIX 12: Added `not self.is_recording` guard so this branch
                    # doesn't fire during active recording
                    if not self.is_recording and (
                        ((not self.use_wake_words
                          or not wake_word_activation_delay_passed)
                              and self.start_recording_on_voice_activity)
                        or self.wakeword_detected):

                        if self.use_extended_logging:
                            logger.debug('Debug: Checking if voice is active')

                        if self._is_voice_active():

                            if self.on_vad_start:
                               self._run_callback(self.on_vad_start)

                            if self.use_extended_logging:
                                logger.debug('Debug: Voice activity detected')
                            logger.info("voice activity detected")

                            if self.use_extended_logging:
                                logger.debug('Debug: Starting recording')
                            self.start()

                            self.start_recording_on_voice_activity = False

                            if self.use_extended_logging:
                                logger.debug('Debug: Adding buffered audio to frames')
                            self.frames.extend(list(self.audio_buffer))
                            self.audio_buffer.clear()

                            if self.use_extended_logging:
                                logger.debug('Debug: Resetting Silero VAD model states')
                            self.silero_vad_model.reset_states()
                        else:
                            if self.use_extended_logging:
                                logger.debug('Debug: Checking voice activity')
                            data_copy = data[:]
                            self._check_voice_activity(data_copy)

                    if self.use_extended_logging:
                        logger.debug('Debug: Resetting speech_end_silence_start')

                    if self.speech_end_silence_start != 0:
                        self.speech_end_silence_start = 0
                        if self.on_turn_detection_stop:
                            if self.use_extended_logging:
                                logger.debug('Debug: Calling on_turn_detection_stop')
                            self._run_callback(self.on_turn_detection_stop)

                else:
                    if self.use_extended_logging:
                        logger.debug('Debug: Handling recording state')
                    # If we are currently recording
                    if wakeword_samples_to_remove and wakeword_samples_to_remove > 0:
                        if self.use_extended_logging:
                            logger.debug('Debug: Removing wakeword samples')
                        samples_removed = 0
                        while wakeword_samples_to_remove > 0 and self.frames:
                            frame = self.frames[0]
                            frame_samples = len(frame) // 2
                            if wakeword_samples_to_remove >= frame_samples:
                                self.frames.pop(0)
                                samples_removed += frame_samples
                                wakeword_samples_to_remove -= frame_samples
                            else:
                                self.frames[0] = frame[wakeword_samples_to_remove * 2:]
                                samples_removed += wakeword_samples_to_remove
                                wakeword_samples_to_remove = 0  # FIX 7: was `samples_to_remove = 0`

                        wakeword_samples_to_remove = 0

                    if self.use_extended_logging:
                        logger.debug('Debug: Checking if stop_recording_on_voice_deactivity is True')
                    if self.stop_recording_on_voice_deactivity:
                        if self.use_extended_logging:
                            logger.debug('Debug: Determining if speech is detected')
                        is_speech = (
                            self._is_silero_speech(data) if self.silero_deactivity_detection
                            else self._is_webrtc_speech(data, True)
                        )

                        if self.use_extended_logging:
                            logger.debug('Debug: Formatting speech_end_silence_start')
                        if not self.speech_end_silence_start:
                            str_speech_end_silence_start = "0"
                        else:
                            str_speech_end_silence_start = datetime.datetime.fromtimestamp(self.speech_end_silence_start).strftime('%H:%M:%S.%f')[:-3]
                        if self.use_extended_logging:
                            logger.debug(f"is_speech: {is_speech}, str_speech_end_silence_start: {str_speech_end_silence_start}")

                        if self.use_extended_logging:
                            logger.debug('Debug: Checking if speech is not detected')
                        if not is_speech:
                            if self.use_extended_logging:
                                logger.debug('Debug: Handling voice deactivity')
                            if self.speech_end_silence_start == 0 and \
                                (time.time() - self.recording_start_time > self.min_length_of_recording):

                                self.speech_end_silence_start = time.time()
                                self.awaiting_speech_end = True
                                if self.on_turn_detection_start:
                                    if self.use_extended_logging:
                                        logger.debug('Debug: Calling on_turn_detection_start')

                                    self._run_callback(self.on_turn_detection_start)

                            if self.use_extended_logging:
                                logger.debug('Debug: Checking early transcription conditions')
                            # FIX 8: Wrap early transcription in transcription_lock
                            if self.speech_end_silence_start and self.early_transcription_on_silence and len(self.frames) > 0 and \
                                (time.time() - self.speech_end_silence_start > self.early_transcription_on_silence) and \
                                self.allowed_to_early_transcribe:
                                    if self.use_extended_logging:
                                        logger.debug("Debug: Adding early transcription request")
                                    with self.transcription_lock:
                                        self.transcribe_count += 1
                                        audio_array = np.frombuffer(b''.join(self.frames), dtype=np.int16)
                                        audio = audio_array.astype(np.float32) / INT16_MAX_ABS_VALUE

                                        if self.use_extended_logging:
                                            logger.debug("Debug: early transcription request pipe send")
                                        self.parent_transcription_pipe.send((audio, self.language, True))
                                        if self.use_extended_logging:
                                            logger.debug("Debug: early transcription request pipe send return")
                                    self.allowed_to_early_transcribe = False

                        else:
                            self.awaiting_speech_end = False
                            if self.use_extended_logging:
                                logger.debug('Debug: Handling speech detection')
                            if self.speech_end_silence_start:
                                if self.use_extended_logging:
                                    logger.info("Resetting self.speech_end_silence_start")

                                if self.speech_end_silence_start != 0:
                                    self.speech_end_silence_start = 0
                                    if self.on_turn_detection_stop:
                                        if self.use_extended_logging:
                                            logger.debug('Debug: Calling on_turn_detection_stop')
                                        self._run_callback(self.on_turn_detection_stop)

                                self.allowed_to_early_transcribe = True

                        if self.use_extended_logging:
                            logger.debug('Debug: Checking if silence duration exceeds threshold')
                        if self.speech_end_silence_start and time.time() - \
                                self.speech_end_silence_start >= \
                                self.post_speech_silence_duration:

                            if self.on_vad_stop:
                                self._run_callback(self.on_vad_stop)

                            if self.use_extended_logging:
                                logger.debug('Debug: Formatting silence start time')
                            silence_start_time = datetime.datetime.fromtimestamp(self.speech_end_silence_start).strftime('%H:%M:%S.%f')[:-3]

                            if self.use_extended_logging:
                                logger.debug('Debug: Calculating time difference')
                            time_diff = time.time() - self.speech_end_silence_start

                            if self.use_extended_logging:
                                logger.debug('Debug: Logging voice deactivity detection')
                                logger.info(f"voice deactivity detected at {silence_start_time}, "
                                        f"time since silence start: {time_diff:.3f} seconds")

                                logger.debug('Debug: Appending data to frames and stopping recording')
                            self.frames.append(data)
                            self.stop()
                            if not self.is_recording:
                                if self.speech_end_silence_start != 0:
                                    self.speech_end_silence_start = 0
                                    if self.on_turn_detection_stop:
                                        if self.use_extended_logging:
                                            logger.debug('Debug: Calling on_turn_detection_stop')
                                        self._run_callback(self.on_turn_detection_stop)

                                if self.use_extended_logging:
                                    logger.debug('Debug: Handling non-wake word scenario')
                            else:
                                if self.use_extended_logging:
                                    logger.debug('Debug: Setting failed_stop_attempt to True')
                                failed_stop_attempt = True

                            self.awaiting_speech_end = False

                # Enforce max recording duration
                if self.is_recording and self.max_recording_duration > 0:
                    if time.time() - self.recording_start_time >= self.max_recording_duration:
                        logger.info(f"Max recording duration ({self.max_recording_duration}s) reached, stopping recording")
                        if self.on_vad_stop:
                            self._run_callback(self.on_vad_stop)
                        self.stop()
                        self.awaiting_speech_end = False

                if self.use_extended_logging:
                    logger.debug('Debug: Checking if recording stopped')
                if not self.is_recording and was_recording:
                    if self.use_extended_logging:
                        logger.debug('Debug: Resetting after stopping recording')
                    self.stop_recording_on_voice_deactivity = False

                if self.use_extended_logging:
                    logger.debug('Debug: Checking Silero time')
                if time.time() - self.silero_check_time > 0.1:
                    self.silero_check_time = 0

                if self.use_extended_logging:
                    logger.debug('Debug: Handling wake word timeout')
                if self.wake_word_detect_time and time.time() - \
                        self.wake_word_detect_time > self.wake_word_timeout:

                    self.wake_word_detect_time = 0
                    if self.wakeword_detected and self.on_wakeword_timeout:
                        if self.use_extended_logging:
                            logger.debug('Debug: Calling on_wakeword_timeout')
                        self._run_callback(self.on_wakeword_timeout)
                    self.wakeword_detected = False

                if self.use_extended_logging:
                    logger.debug('Debug: Updating was_recording')
                was_recording = self.is_recording

                if self.use_extended_logging:
                    logger.debug('Debug: Checking if recording and not failed stop attempt')
                if self.is_recording and not failed_stop_attempt:
                    if self.use_extended_logging:
                        logger.debug('Debug: Appending data to frames')
                    self.frames.append(data)

                if self.use_extended_logging:
                    logger.debug('Debug: Checking if not recording or speech end silence start')
                if not self.is_recording or self.speech_end_silence_start:
                    if self.use_extended_logging:
                        logger.debug('Debug: Appending data to audio buffer')
                    self.audio_buffer.append(data)

        except Exception as e:
            logger.debug('Debug: Caught exception in main try block')
            if not self.interrupt_stop_event.is_set():
                logger.error(f"Unhandled exeption in _recording_worker: {e}", exc_info=True)
                raise

        if self.use_extended_logging:
            logger.debug('Debug: Exiting _recording_worker method')

    def _realtime_worker(self):
        """
        Performs real-time transcription if the feature is enabled.
        """

        try:

            logger.debug('Starting realtime worker')

            if not self.enable_realtime_transcription:
                return

            last_transcription_time = time.time()

            while self.is_running:

                if self.is_recording:

                    while (
                        time.time() - last_transcription_time
                    ) < self.realtime_processing_pause:
                        time.sleep(0.001)
                        if not self.is_running or not self.is_recording:
                            break

                    if self.awaiting_speech_end:
                        time.sleep(0.001)
                        continue

                    last_transcription_time = time.time()

                    audio_array = np.frombuffer(
                        b''.join(self.frames),
                        dtype=np.int16
                        )

                    logger.debug(f"Current realtime buffer size: {len(audio_array)}")

                    audio_array = audio_array.astype(np.float32) / \
                        INT16_MAX_ABS_VALUE

                    if self.use_main_model_for_realtime:
                        with self.transcription_lock:
                            try:
                                self.parent_transcription_pipe.send((audio_array, self.language, True))
                                if self.parent_transcription_pipe.poll(timeout=5):
                                    logger.debug("Receive from realtime worker after transcription request to main model")
                                    status, result = self.parent_transcription_pipe.recv()
                                    if status == 'success':
                                        # FIX 10: clearer variable name
                                        transcription_text, info = result
                                        self.detected_realtime_language = info.language if info.language_probability > 0 else None
                                        self.detected_realtime_language_probability = info.language_probability
                                        realtime_text = transcription_text
                                        logger.debug(f"Realtime text detected with main model: {realtime_text}")
                                    else:
                                        logger.error(f"Realtime transcription error: {result}")
                                        continue
                                else:
                                    logger.warning("Realtime transcription timed out")
                                    continue
                            except Exception as e:
                                logger.error(f"Error in realtime transcription: {str(e)}", exc_info=True)
                                continue
                    else:
                        if self.normalize_audio:
                            if audio_array is not None and audio_array.size > 0:
                                peak = np.max(np.abs(audio_array))
                                if peak > 0:
                                    audio_array = (audio_array / peak) * 0.95

                        # FIX 13: use self.realtime_model instead of self.realtime_model_type
                        if self.realtime_batch_size > 0:
                            segments, info = self.realtime_model.transcribe(
                                audio_array,
                                language=self.language if self.language else None,
                                beam_size=self.beam_size_realtime,
                                initial_prompt=self.initial_prompt_realtime,
                                suppress_tokens=self.suppress_tokens,
                                batch_size=self.realtime_batch_size,
                                vad_filter=self.faster_whisper_vad_filter
                            )
                        else:
                            segments, info = self.realtime_model.transcribe(
                                audio_array,
                                language=self.language if self.language else None,
                                beam_size=self.beam_size_realtime,
                                initial_prompt=self.initial_prompt_realtime,
                                suppress_tokens=self.suppress_tokens,
                                vad_filter=self.faster_whisper_vad_filter
                            )

                        self.detected_realtime_language = info.language if info.language_probability > 0 else None
                        self.detected_realtime_language_probability = info.language_probability
                        realtime_text = " ".join(
                            seg.text for seg in segments
                        )
                        logger.debug(f"Realtime text detected: {realtime_text}")

                    if self.is_recording and time.time() - \
                            self.recording_start_time > self.init_realtime_after_seconds:

                        self.realtime_transcription_text = realtime_text
                        self.realtime_transcription_text = \
                            self.realtime_transcription_text.strip()

                        self.text_storage.append(
                            self.realtime_transcription_text
                            )

                        if len(self.text_storage) >= 2:
                            last_two_texts = self.text_storage[-2:]

                            prefix = os.path.commonprefix(
                                [last_two_texts[0], last_two_texts[1]]
                                )

                            if len(prefix) >= \
                                    len(self.realtime_stabilized_safetext):

                                self.realtime_stabilized_safetext = prefix

                        matching_pos = self._find_tail_match_in_text(
                            self.realtime_stabilized_safetext,
                            self.realtime_transcription_text
                            )

                        if matching_pos < 0:
                            text_to_send = (
                                self.realtime_stabilized_safetext
                                if self.realtime_stabilized_safetext
                                else self.realtime_transcription_text
                            )
                            processed = self._preprocess_output(text_to_send, True)
                            self._run_callback(self._on_realtime_transcription_stabilized, processed)

                        else:
                            output_text = self.realtime_stabilized_safetext + \
                                self.realtime_transcription_text[matching_pos:]

                            self._run_callback(self._on_realtime_transcription_stabilized, self._preprocess_output(output_text, True))

                        self._run_callback(self._on_realtime_transcription_update, self._preprocess_output(self.realtime_transcription_text, True))

                else:
                    time.sleep(TIME_SLEEP)

        except Exception as e:
            logger.error(f"Unhandled exeption in _realtime_worker: {e}", exc_info=True)
            raise

    def _is_silero_speech(self, chunk):
        """
        Returns true if speech is detected in the provided audio data
        """
        if self.sample_rate != 16000:
            pcm_data = np.frombuffer(chunk, dtype=np.int16)
            data_16000 = signal.resample_poly(
                pcm_data, 16000, self.sample_rate)
            chunk = data_16000.astype(np.int16).tobytes()

        self.silero_working = True
        audio_chunk = np.frombuffer(chunk, dtype=np.int16)
        audio_chunk = audio_chunk.astype(np.float32) / INT16_MAX_ABS_VALUE
        vad_prob = self.silero_vad_model(
            torch.from_numpy(audio_chunk),
            SAMPLE_RATE).item()
        is_silero_speech_active = vad_prob > (1 - self.silero_sensitivity)
        if is_silero_speech_active:
            if not self.is_silero_speech_active and self.use_extended_logging:
                logger.info(f"{bcolors.OKGREEN}Silero VAD detected speech{bcolors.ENDC}")
        elif self.is_silero_speech_active and self.use_extended_logging:
            logger.info(f"{bcolors.WARNING}Silero VAD detected silence{bcolors.ENDC}")
        self.is_silero_speech_active = is_silero_speech_active
        self.silero_working = False
        return is_silero_speech_active

    def _is_webrtc_speech(self, chunk, all_frames_must_be_true=False):
        """
        Returns true if speech is detected in the provided audio data
        """
        speech_str = f"{bcolors.OKGREEN}WebRTC VAD detected speech{bcolors.ENDC}"
        silence_str = f"{bcolors.WARNING}WebRTC VAD detected silence{bcolors.ENDC}"
        if self.sample_rate != 16000:
            pcm_data = np.frombuffer(chunk, dtype=np.int16)
            data_16000 = signal.resample_poly(
                pcm_data, 16000, self.sample_rate)
            chunk = data_16000.astype(np.int16).tobytes()

        frame_length = int(16000 * 0.01)
        num_frames = int(len(chunk) / (2 * frame_length))
        speech_frames = 0

        for i in range(num_frames):
            start_byte = i * frame_length * 2
            end_byte = start_byte + frame_length * 2
            frame = chunk[start_byte:end_byte]
            if self.webrtc_vad_model.is_speech(frame, 16000):
                speech_frames += 1
                if not all_frames_must_be_true:
                    if self.debug_mode:
                        logger.info(f"Speech detected in frame {i + 1}"
                              f" of {num_frames}")
                    if not self.is_webrtc_speech_active and self.use_extended_logging:
                        logger.info(speech_str)
                    self.is_webrtc_speech_active = True
                    return True
        if all_frames_must_be_true:
            if self.debug_mode and speech_frames == num_frames:
                logger.info(f"Speech detected in {speech_frames} of "
                      f"{num_frames} frames")
            elif self.debug_mode:
                logger.info(f"Speech not detected in all {num_frames} frames")
            speech_detected = speech_frames == num_frames
            if speech_detected and not self.is_webrtc_speech_active and self.use_extended_logging:
                logger.info(speech_str)
            elif not speech_detected and self.is_webrtc_speech_active and self.use_extended_logging:
                logger.info(silence_str)
            self.is_webrtc_speech_active = speech_detected
            return speech_detected
        else:
            if self.debug_mode:
                logger.info(f"Speech not detected in any of {num_frames} frames")
            if self.is_webrtc_speech_active and self.use_extended_logging:
                logger.info(silence_str)
            self.is_webrtc_speech_active = False
            return False

    def _check_voice_activity(self, data):
        """
        Initiate check if voice is active based on the provided data.
        """
        self._is_webrtc_speech(data)

        if self.is_webrtc_speech_active:

            if not self.silero_working:
                self.silero_working = True

                threading.Thread(
                    target=self._is_silero_speech,
                    args=(data,)).start()

    def clear_audio_queue(self):
        """
        Safely empties the audio queue.
        """
        self.audio_buffer.clear()
        try:
            while True:
                self.audio_queue.get_nowait()
        except Exception:  # FIX 11: was bare `except:`
            pass

    def _is_voice_active(self):
        """
        Determine if voice is active.
        """
        return self.is_webrtc_speech_active and self.is_silero_speech_active

    def _set_state(self, new_state):
        """
        Update the current state of the recorder and execute
        corresponding state-change callbacks.
        """
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        logger.info(f"State changed from '{old_state}' to '{new_state}'")

        if old_state == "listening":
            if self.on_vad_detect_stop:
                self._run_callback(self.on_vad_detect_stop)
        elif old_state == "wakeword":
            if self.on_wakeword_detection_end:
                self._run_callback(self.on_wakeword_detection_end)

        if new_state == "listening":
            if self.on_vad_detect_start:
                self._run_callback(self.on_vad_detect_start)
            self._set_spinner("speak now")
            if self.spinner and self.halo:
                self.halo._interval = 250
        elif new_state == "wakeword":
            if self.on_wakeword_detection_start:
                self._run_callback(self.on_wakeword_detection_start)
            self._set_spinner(f"say {self.wake_words}")
            if self.spinner and self.halo:
                self.halo._interval = 500
        elif new_state == "transcribing":
            self._set_spinner("transcribing")
            if self.spinner and self.halo:
                self.halo._interval = 50
        elif new_state == "recording":
            self._set_spinner("recording")
            if self.spinner and self.halo:
                self.halo._interval = 100
        elif new_state == "inactive":
            if self.spinner and self.halo:
                self.halo.stop()
                self.halo = None

    def _set_spinner(self, text):
        """
        Update the spinner's text or create a new
        spinner with the provided text.
        """
        if self.spinner:
            if self.halo is None:
                self.halo = halo.Halo(text=text)
                self.halo.start()
            else:
                self.halo.text = text

    def _preprocess_output(self, text, preview=False):
        """
        Preprocesses the output text.
        """
        text = re.sub(r'\s+', ' ', text.strip())

        if self.ensure_sentence_starting_uppercase:
            if text:
                text = text[0].upper() + text[1:]

        if not preview:
            if self.ensure_sentence_ends_with_period:
                if text and text[-1].isalnum():
                    text += '.'

        return text

    def _find_tail_match_in_text(self, text1, text2, length_of_match=10):
        """
        Find the position where the last 'n' characters of text1
        match with a substring in text2.
        """
        if len(text1) < length_of_match or len(text2) < length_of_match:
            return -1

        target_substring = text1[-length_of_match:]

        for i in range(len(text2) - length_of_match + 1):
            current_substring = text2[len(text2) - i - length_of_match:
                                      len(text2) - i]

            if current_substring == target_substring:
                return len(text2) - i

        return -1

    def _on_realtime_transcription_stabilized(self, text):
        """
        Callback method invoked when the real-time transcription stabilizes.
        """
        if self.on_realtime_transcription_stabilized:
            if self.is_recording:
                self._run_callback(self.on_realtime_transcription_stabilized, text)

    def _on_realtime_transcription_update(self, text):
        """
        Callback method invoked when there's an update in the real-time
        transcription.
        """
        if self.on_realtime_transcription_update:
            if self.is_recording:
                self._run_callback(self.on_realtime_transcription_update, text)

    def __enter__(self):
        """
        Method to setup the context manager protocol.
        """
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Method to define behavior when the context manager protocol exits.
        """
        self.shutdown()