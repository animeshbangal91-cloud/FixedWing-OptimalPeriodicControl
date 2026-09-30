"""Plot streamed and flown tracking for matched steady and periodic runs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load(run):
    run = Path(run)
    meta = json.loads((run / "run.json").read_text())
    start, end = meta["score_start_sim_s"], meta["score_end_sim_s"]
    heading = meta["experiment_entry_ne_heading"][2]
    origin_down = meta["origin_down_m"]

    cmd = pd.read_csv(run / "commands.csv")
    cmd = cmd[(cmd.phase == "measure") & (cmd.sim_s >= start) & (cmd.sim_s <= end)].copy()
    act = pd.read_csv(run / "actual.csv")
    act = act[(act.sim_s >= start) & (act.sim_s <= end)].copy()
    ctl = pd.read_csv(run / "direct_control.csv")
    ctl = ctl[(ctl.phase == "measure") & (ctl.sim_s >= start) & (ctl.sim_s <= end)].copy()
    energy = pd.read_csv(run / "scored_energy.csv")

    co, si = np.cos(heading), np.sin(heading)
    cmd_along = cmd.north.to_numpy()*co + cmd.east.to_numpy()*si
    act_along = act.north.to_numpy()*co + act.east.to_numpy()*si
    return {
        "t_cmd": cmd.sim_s.to_numpy()-start,
        "t_act": act.sim_s.to_numpy()-start,
        "t_energy": energy.score_elapsed_s.to_numpy(),
        "height_cmd": origin_down-cmd.down.to_numpy(),
        "height_act": origin_down-act.down.to_numpy(),
        "along_cmd": cmd_along-cmd_along[0],
        "along_act": act_along-act_along[0],
        "airspeed_ref": np.interp(energy.score_elapsed_s, ctl.sim_s-start,
                                  ctl.reference_airspeed_mps),
        "airspeed_act": energy.airspeed_mps.to_numpy(),
    }


parser = argparse.ArgumentParser()
parser.add_argument("steady", type=Path)
parser.add_argument("periodic", type=Path)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()

fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex="col", layout="constrained")
for col, (label, run) in enumerate((("Steady", args.steady), ("Periodic", args.periodic))):
    d = load(run)
    axes[0, col].plot(d["t_cmd"], d["height_cmd"], label="Streamed")
    axes[0, col].plot(d["t_act"], d["height_act"], label="Actual", alpha=.9)
    axes[1, col].plot(d["t_energy"], d["airspeed_ref"], label="Streamed")
    axes[1, col].plot(d["t_energy"], d["airspeed_act"], label="Actual", alpha=.9)
    axes[2, col].plot(d["t_cmd"], d["along_cmd"], label="Streamed")
    axes[2, col].plot(d["t_act"], d["along_act"], label="Actual", alpha=.9)
    axes[0, col].set_title(label)
    axes[0, col].set_ylabel("Height above origin [m]")
    axes[1, col].set_ylabel("Airspeed [m/s]")
    axes[2, col].set_ylabel("Along-track distance [m]")
    axes[2, col].set_xlabel("Scored time [s]")
    for ax in axes[:, col]:
        ax.grid(True)
        ax.legend()

fig.suptitle("Five-period streamed versus actual trajectory tracking")
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output.with_suffix(".png"), dpi=170)
fig.savefig(args.output.with_suffix(".pdf"))
print(args.output.with_suffix(".png"))
