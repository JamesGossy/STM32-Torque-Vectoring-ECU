"""Starts ecu_sim on a free port, then runs motor_interop against it."""
import socket
import subprocess
import sys
import time

ecu_sim, motor_interop = sys.argv[1], sys.argv[2]
with socket.socket() as probe:
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]

ecu = subprocess.Popen([ecu_sim, '--port', str(port), '--once'], stdout=subprocess.DEVNULL)
try:
    time.sleep(0.5)
    sys.exit(subprocess.run([motor_interop, str(port)], timeout=120).returncode)
finally:
    ecu.kill()
    ecu.wait()
