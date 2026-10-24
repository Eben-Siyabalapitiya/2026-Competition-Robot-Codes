"""Warm the installed speech runtime once and keep WSL alive for this server."""
import atexit
import subprocess
import threading

_status = {'abr': 'starting', 'gemini': 'checking'}
_guard = threading.Lock()
_keeper = None
_started = False

def status():
    with _guard:
        return dict(_status)

def _set(name, value):
    with _guard:
        _status[name] = value
    print(f'  {name.upper()}: {value}', flush=True)

def _close():
    global _keeper
    if _keeper is not None:
        try:
            _keeper.stdin.close()
            _keeper.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            _keeper.terminate()
        _keeper = None

def _warm():
    global _keeper
    from voice_control import run_abr
    from gemini_control import KEY_FILE
    _set('gemini', 'configured' if KEY_FILE.is_file() else 'missing private API key')
    try:
        _keeper = subprocess.Popen(['wsl', '-d', 'Ubuntu', '-u', 'root', '--',
            '/opt/minibot-abr/.venv/bin/python', '-c', 'import sys; sys.stdin.buffer.read()'],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW)
        for module, cls, library in [
            ('asr', 'Asr', '/opt/minibot-abr/packages/niagara-38m-live.en-linux-x86_64/libniagara_38m_live.so'),
            ('tts', 'Tts', '/opt/minibot-abr/packages/nith-5m-live.en-f1-linux-x86_64/libnith_5m_live.so')]:
            _set('abr', f'warming {module.upper()}')
            args=['/opt/minibot-abr/.venv/bin/python', '-c',
                  f'from abr_sdk.{module} import {cls}\nwith {cls}({library!r}): pass']
            result=run_abr(args)
            if result.returncode and 'ERR_LICENSE' in result.stderr:
                activation=run_abr(['/opt/minibot-abr/.venv/bin/abr-sdk', 'activate', library,
                    '--key-file', '/mnt/c/Users/budhu/MiniBot/abr-private/license.key'])
                if activation.returncode:
                    raise RuntimeError('license activation failed; check internet access and license')
                result=run_abr(args)
            if result.returncode:
                raise RuntimeError(f'{module.upper()} model could not load; check the installed WSL runtime')
        if _keeper.poll() is not None:
            raise RuntimeError('WSL keep-alive exited unexpectedly')
        _set('abr', 'ready')
    except (OSError, subprocess.TimeoutExpired, RuntimeError) as exc:
        _set('abr', str(exc) if isinstance(exc, RuntimeError) else 'WSL startup failed or timed out')

def start():
    global _started
    if _started:
        return
    _started=True
    atexit.register(_close)
    threading.Thread(target=_warm, daemon=True).start()
