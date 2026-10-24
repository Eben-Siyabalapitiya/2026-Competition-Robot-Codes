"""ABR transcription and a deliberately small, duration-aware command grammar."""
import io
import re
import subprocess
import tempfile
import threading
import wave
from pathlib import Path
from flask import Blueprint, jsonify, request, send_file
from gemini_control import plan_text

voice = Blueprint('voice', __name__)
busy = threading.Lock()
tts_busy = threading.Lock()
planner_busy = threading.Lock()
WORDS = {'half': .5, 'a': 1, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5}
WSL = ['wsl', '-d', 'Ubuntu', '-u', 'root', '--']
LIBRARY = '/opt/minibot-abr/packages/niagara-38m-live.en-linux-x86_64/libniagara_38m_live.so'

def run_abr(args):
    return subprocess.run(WSL + args, capture_output=True, text=True, timeout=30,
                          creationflags=subprocess.CREATE_NO_WINDOW)

def recognize_audio(linux_path):
    args = ['/opt/minibot-abr/.venv/bin/python',
            '/mnt/c/Users/budhu/MiniBot/abr-laptop/probe_sdk.py',
            'asr', '--library', LIBRARY, '--audio', linux_path]
    result = run_abr(args)
    if result.returncode and 'ERR_LICENSE' in result.stderr:
        # WSL can invalidate the device-bound activation between sessions.
        # Use the user-approved private key file; never log its contents.
        activation = run_abr(['/opt/minibot-abr/.venv/bin/abr-sdk', 'activate', LIBRARY,
                              '--key-file', '/mnt/c/Users/budhu/MiniBot/abr-private/license.key'])
        if activation.returncode:
            raise ValueError('ABR license renewal failed. Check internet access and the evaluation license.')
        result = run_abr(args)
    return result

@voice.post('/api/voice/plan')
def gemini_plan():
    if not planner_busy.acquire(blocking=False):
        return jsonify(message='Gemini is already processing a request.'), 409
    try:
        text = (request.get_json(silent=True) or {}).get('text', '')
        return jsonify(plan_text(text))
    except ValueError as exc:
        return jsonify(message=str(exc)), 400
    finally:
        planner_busy.release()

@voice.post('/api/voice/speak')
def speak():
    text = (request.get_json(silent=True) or {}).get('text', '')
    if not isinstance(text, str) or not 1 <= len(text) <= 400:
        return jsonify(message='Spoken reply must be 1–400 characters.'), 400
    if not tts_busy.acquire(blocking=False):
        return jsonify(message='Speech output is busy.'), 409
    path = None
    try:
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            path = Path(f.name)
        linux_path = '/mnt/' + path.drive[0].lower() + path.as_posix()[2:]
        library = '/opt/minibot-abr/packages/nith-5m-live.en-f1-linux-x86_64/libnith_5m_live.so'
        args = ['/opt/minibot-abr/.venv/bin/python', '/mnt/c/Users/budhu/MiniBot/abr-laptop/probe_sdk.py',
                'tts', '--library', library, '--text', text, '--output', linux_path]
        result = run_abr(args)
        if result.returncode and 'ERR_LICENSE' in result.stderr:
            activation = run_abr(['/opt/minibot-abr/.venv/bin/abr-sdk', 'activate', library,
                                  '--key-file', '/mnt/c/Users/budhu/MiniBot/abr-private/license.key'])
            if not activation.returncode:
                result = run_abr(args)
        if result.returncode or path.stat().st_size < 44:
            return jsonify(message='ABR could not generate speech. Text replies still work.'), 503
        return send_file(io.BytesIO(path.read_bytes()), mimetype='audio/wav', max_age=0)
    except (OSError, subprocess.TimeoutExpired):
        return jsonify(message='ABR speech output timed out.'), 503
    finally:
        if path:
            path.unlink(missing_ok=True)
        tts_busy.release()

def parse_command(text):
    text = re.sub(r'(?<!\d)\.|\.(?!\d)|[,!?]', '', text.lower()).strip()
    text = re.sub(r'^(?:please |robot |minibot )+', '', text)
    text = re.sub(r' please$', '', text)
    if text in ('stop', 'stop moving', 'disable', 'emergency stop'):
        return {'direction': 'stop', 'seconds': 0}
    match = re.fullmatch(r'(?:(?:move|go|drive|turn|rotate) )?(forward|forwards|backward|backwards|back|left|right)(?: for (half a|half|a|one|two|three|four|five|\d+(?:\.\d+)?) seconds?)?', text)
    if not match:
        raise ValueError('Use: forward, backward, left or right for 1–5 seconds; or stop. One command at a time.')
    direction, duration = match.groups()
    direction = {'forwards': 'forward', 'backwards': 'backward', 'back': 'backward'}.get(direction, direction)
    duration = duration or 'one'
    seconds = .5 if duration == 'half a' else WORDS.get(duration)
    if seconds is None:
        seconds = float(duration)
    if not .25 <= seconds <= 5:
        raise ValueError('Duration must be between 0.25 and 5 seconds.')
    return {'direction': direction, 'seconds': seconds}

@voice.post('/api/voice/parse')
def parse():
    try:
        text = (request.get_json(silent=True) or {}).get('text', '')
        if not isinstance(text, str) or len(text) > 200:
            raise ValueError('Command is too long.')
        return jsonify(text=text, command=parse_command(text))
    except ValueError as exc:
        return jsonify(message=str(exc)), 400

@voice.post('/api/voice/transcribe')
def transcribe():
    if not busy.acquire(blocking=False):
        return jsonify(message='Speech recognition is busy.'), 409
    path = None
    try:
        data = request.stream.read(260001)
        if len(data) > 260000:
            raise ValueError('Recording is too long.')
        with wave.open(io.BytesIO(data)) as audio:
            if (audio.getnchannels(), audio.getsampwidth(), audio.getframerate()) != (1, 2, 16000) or not 0 < audio.getnframes() <= 128000:
                raise ValueError('Expected at most eight seconds of mono 16 kHz WAV.')
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            f.write(data)
            path = Path(f.name)
        linux_path = '/mnt/' + path.drive[0].lower() + path.as_posix()[2:]
        result = recognize_audio(linux_path)
        if result.returncode:
            return jsonify(message='ABR recognition failed. Check the WSL license and model installation.'), 503
        text = next((line.removeprefix('Transcript:').strip() for line in result.stdout.splitlines() if line.startswith('Transcript:')), '')
        return jsonify(text=text)
    except (ValueError, wave.Error, EOFError) as exc:
        return jsonify(message=str(exc)), 400
    except (OSError, subprocess.TimeoutExpired):
        return jsonify(message='ABR did not respond. Try again.'), 503
    finally:
        if path:
            path.unlink(missing_ok=True)
        busy.release()
