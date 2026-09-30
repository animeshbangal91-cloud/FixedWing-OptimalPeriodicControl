"""Compare two recorded circles, preserving reference time and errors."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

parser=argparse.ArgumentParser()
parser.add_argument("before",type=Path)
parser.add_argument("after",type=Path)
parser.add_argument("--output",type=Path,default=Path(__file__).resolve().parent/"tuning_comparison")
args=parser.parse_args()
fig,axs=plt.subplots(2,2,figsize=(12,9),layout="constrained")
comparison={}
for folder,label,color in ((args.before,"Before","tab:red"),(args.after,"After","tab:blue")):
    meta=json.loads((folder/"run.json").read_text())
    metrics=json.loads((folder/"metrics.json").read_text())
    c=pd.read_csv(folder/"commands.csv"); c=c[c.phase=="circle"]
    a=pd.read_csv(folder/"actual.csv")
    a=a[(a.sim_s>=c.sim_s.iloc[0])&(a.sim_s<=c.sim_s.iloc[-1])]
    cn,ce=meta["circle_center_ne"]
    n=a.north.to_numpy()-cn; e=a.east.to_numpy()-ce
    theta=np.arctan2(c.east.iloc[0]-ce,c.north.iloc[0]-cn)
    # Rigidly align each run's intended circle center and entry bearing, not its flown path.
    north=n*np.cos(theta)+e*np.sin(theta)
    east=-n*np.sin(theta)+e*np.cos(theta)
    axs[0,0].plot(east,north,label=label,color=color)
    times=a.sim_s.to_numpy()-c.sim_s.iloc[0]
    radial=np.abs(np.hypot(n,e)-meta["radius_m"])
    ref=np.column_stack([np.interp(a.sim_s,c.sim_s,c[k]) for k in ("north","east","down")])
    err=np.linalg.norm(a[["north","east","down"]].to_numpy()-ref,axis=1)
    axs[0,1].plot(times,radial,label=label,color=color)
    axs[1,0].plot(times,meta["origin_down_m"]-a.down,label=label,color=color)
    axs[1,1].plot(times,err,label=label,color=color)
    comparison[label]={"run":str(folder),**metrics}
    comparison[label]["peak_radial_error_m"]=float(radial.max())
    comparison[label]["peak_time_aligned_error_m"]=float(err.max())
angle=np.linspace(0,2*np.pi,400)
axs[0,0].plot(120*np.sin(angle),120*np.cos(angle),"k--",label="120 m reference")
axs[0,0].set(xlabel="Aligned east [m]",ylabel="Aligned north [m]",aspect="equal",title="Same radius; centers/entry bearings aligned")
axs[0,1].set(xlabel="Circle time [s]",ylabel="Absolute radial error [m]")
axs[1,0].axhline(130,color="black",linestyle="--",label="130 m reference")
axs[1,0].set(xlabel="Circle time [s]",ylabel="Height above start [m]")
axs[1,1].set(xlabel="Circle time [s]",ylabel="Time-aligned 3D error [m]")
for ax in axs.flat: ax.grid(True); ax.legend()
fig.suptitle("Stallion model: original run vs entry/controller tuning")
fig.savefig(args.output.with_suffix(".png"),dpi=150)
fig.savefig(args.output.with_suffix(".pdf"))
args.output.with_suffix(".json").write_text(json.dumps(comparison,indent=2)+"\n")
print(json.dumps(comparison,indent=2))
