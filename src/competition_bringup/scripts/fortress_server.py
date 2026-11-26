#!/usr/bin/env python3
"""Run Fortress server and bound shutdown when its renderer stalls."""
import os
import signal
import subprocess
import sys
import time


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: fortress_server.py WORLD_SDF')
    environment = dict(os.environ)
    environment.pop('IGN_CONFIG_PATH', None)
    child = subprocess.Popen(['ign', 'gazebo', '-r', '-s', sys.argv[1]],
                             env=environment, start_new_session=True)
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    while not stopping:
        code = child.poll()
        if code is not None:
            return code
        time.sleep(0.1)
    # The separate GUI may still be detaching its rendering connection.
    time.sleep(0.8)
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2.0)
                print('Fortress server required forced shutdown', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
