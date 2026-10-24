"""One entry point for the already-installed MiniBot laptop environment."""
from pathlib import Path
import sys
import threading
import time
import urllib.request
import json
import webbrowser

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'vision'))

def panel_running(url):
    try:
        with urllib.request.urlopen(url + '/api/state', timeout=1) as response:
            data=json.load(response)
        return isinstance(data, dict) and 'bluetooth' in data and 'enabled' in data
    except (OSError, ValueError):
        return False

def main():
    args=sys.argv[1:]
    no_browser='--no-browser' in args
    args=[a for a in args if a != '--no-browser']
    port=8000
    if '--http-port' in args:
        try: port=int(args[args.index('--http-port')+1])
        except (ValueError, IndexError):
            raise SystemExit('--http-port needs a port number')
    url=f'http://127.0.0.1:{port}'
    if '--help' not in args and '-h' not in args and panel_running(url):
        print(f'MiniBot is already running: {url}', flush=True)
        if not no_browser: webbrowser.open(url)
        return
    if '--port' not in args:
        args += ['--port', 'COM7']
    sys.argv=[str(ROOT / 'vision' / 'web_ui.py'), *args]
    if not no_browser and '--help' not in args and '-h' not in args:
        def open_when_ready():
            for _ in range(120):
                if panel_running(url):
                    webbrowser.open(url)
                    return
                time.sleep(.25)
        threading.Thread(target=open_when_ready, daemon=True).start()
    print('Starting camera, Bluetooth, ABR and Gemini. No installation is needed.', flush=True)
    print('Robot starts disabled. Ctrl+C stops the panel.', flush=True)
    try:
        from web_ui import main as run_panel
        run_panel()
    except ImportError as exc:
        raise SystemExit(f'Use the configured MiniBot Python 3.12 interpreter. Missing dependency: {exc.name}') from None

if __name__ == '__main__':
    main()
