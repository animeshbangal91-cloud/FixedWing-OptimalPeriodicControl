"""Receive read-only Gazebo diagnostics, with PX4 and wall receive timestamps."""
import csv
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time


class EnergyCapture:
    def __init__(self, out):
        self.queue = queue.Queue()
        self.last_wall = 0.
        self.latest = None
        self.error_log = (out/'energy_bridge.log').open('w')
        self.process = subprocess.Popen(['/usr/bin/python3', '-u', str(Path(__file__).with_name('gz_energy_bridge.py'))],
                                        stdout=subprocess.PIPE, stderr=self.error_log, text=True,
                                        env=dict(os.environ, GZ_IP='127.0.0.1'))
        self.file = (out/'energy.csv').open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sim_s', 'phase', 'receive_monotonic_s', 'gazebo_s', 'airspeed_mps', 'alpha_rad',
                              'rotor0_rad_s', 'rotor1_rad_s', 'thrust0_N', 'thrust1_N',
                              'power_matlab_W', 'power_axis_W', 'world_x', 'world_y', 'world_z'])
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.process.stdout:
            try:
                values = json.loads(line)
                if len(values) == 12:
                    self.queue.put((time.monotonic(), values))
            except (ValueError, TypeError):
                pass

    def drain(self, sim, phase):
        while True:
            try:
                wall, values = self.queue.get_nowait()
            except queue.Empty:
                break
            self.last_wall, self.latest = wall, values
            self.writer.writerow([sim, phase, wall, *values])
        self.file.flush()

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.file.close()
        self.error_log.close()
