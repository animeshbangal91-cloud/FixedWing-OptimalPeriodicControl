"""Run a local, isolated SITL tuning trial, including explicit simulated arming.

Only used when a flight test has been requested. The interactive launcher still
requires manual arming. Each trial stops its own simulator after saving results.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument("tuning", type=Path)
parser.add_argument('--mission', type=Path)
args=parser.parse_args()
mission_args = ['--mission', str(args.mission.resolve())] if args.mission else []
timeout = 330
if args.mission:
    from mission_profile import Mission
    timeout = 450+2*Mission(args.mission).duration
if subprocess.run(["pgrep","-x","px4"],capture_output=True).returncode==0:
    raise SystemExit("Another PX4 is running; refusing to start a duplicate.")
stamp=datetime.now().strftime("%Y%m%d_%H%M%S")
out=HERE/"trials"/stamp
out.mkdir(parents=True)
px4=Path.home()/"PX4-Autopilot/build/px4_sitl_default/bin"
runtime=Path.home()/"stallion_sitl"
processes=[]
with (out/"px4.log").open("w") as pf, (out/"stream.log").open("w") as sf:
    try:
        sim=subprocess.Popen(["bash",str(runtime/"start_px4.sh")],stdin=subprocess.PIPE,stdout=pf,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(sim)
        time.sleep(12)
        if sim.poll() is not None: raise RuntimeError("PX4 exited during startup")
        stream=subprocess.Popen([str(Path.home()/"venvs/px4/bin/python"),"-u",str(HERE.parent/"fixedwing_demo/stream_mission.py"),
                                 "--model-config",str(runtime/"model_report.json"),"--tuning-config",str(args.tuning.resolve()),
                                 "--results",str(HERE/"results"), *mission_args],stdout=sf,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(stream)
        deadline=time.monotonic()+70
        while "READY." not in (out/"stream.log").read_text():
            if stream.poll() is not None: raise RuntimeError("Streamer exited before READY")
            if time.monotonic()>deadline: raise RuntimeError("READY timeout")
            time.sleep(.5)
        print(f"READY: {out}; selecting Offboard and arming this local simulator",flush=True)
        subprocess.run([str(px4/"px4-commander"),"mode","offboard"],check=True)
        time.sleep(1.5)
        subprocess.run([str(px4/"px4-commander"),"arm"],check=True)
        deadline=time.monotonic()+timeout
        previous=""
        while stream.poll() is None:
            contents=(out/"stream.log").read_text()
            if contents!=previous:
                print(contents[len(previous):].strip(),flush=True); previous=contents
            if time.monotonic()>deadline: raise RuntimeError("Flight test timed out")
            time.sleep(3)
        print((out/"stream.log").read_text()[len(previous):],flush=True)
        latest=Path((HERE/"results/latest.txt").read_text().strip())
        (out/"run_path.txt").write_text(str(latest)+"\n")
        if (latest/"energy_metrics.json").exists(): print((latest/"energy_metrics.json").read_text(),flush=True)
        elif (latest/"metrics.json").exists(): print((latest/"metrics.json").read_text(),flush=True)
        else: print((latest/"run.json").read_text(),flush=True)
        if stream.returncode: raise RuntimeError("Streamer reported failure")
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=8)
                except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM)
        print(f"Trial stopped. Logs: {out}",flush=True)
