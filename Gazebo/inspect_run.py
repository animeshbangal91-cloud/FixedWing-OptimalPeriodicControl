import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from pyulog import ULog

parser=argparse.ArgumentParser()
parser.add_argument("run",type=Path)
parser.add_argument("--ulog",type=Path)
args=parser.parse_args()
a=pd.read_csv(args.run/"actual.csv"); c=pd.read_csv(args.run/"commands.csv")
for phase,g in a.groupby("phase",sort=False):
    print(phase, "time",g.sim_s.iloc[0],g.sim_s.iloc[-1],"height",-g.down.iloc[0],-g.down.iloc[-1],
          "speed first/median/last",np.hypot(g.vn,g.ve).iloc[0],np.hypot(g.vn,g.ve).median(),np.hypot(g.vn,g.ve).iloc[-1])
if not a[a.phase=="circle"].empty:
    print("first circle actual",a[a.phase=="circle"].iloc[0].to_dict())
if not c[c.phase=="circle"].empty:
    print("first circle command",c[c.phase=="circle"].iloc[0].to_dict())
if args.ulog:
    u=ULog(str(args.ulog))
    print("Topics",[(d.name,d.multi_id) for d in u.data_list if any(k in d.name for k in ("tecs","airspeed","setpoint","actuator","rates"))])
    t0=c[c.phase=="circle"].sim_s.iloc[0]; t1=c[c.phase=="circle"].sim_s.iloc[-1]
    for topic in ("tecs_status","vehicle_attitude_setpoint","vehicle_rates_setpoint","vehicle_angular_velocity","airspeed_validated","actuator_servos"):
        try: d=u.get_dataset(topic).data
        except KeyError: continue
        mask=(d["timestamp"]>=t0*1e6)&(d["timestamp"]<=t1*1e6)
        print(topic)
        for name,v in d.items():
            if name=="timestamp" or "timestamp" in name or "integral" in name: continue
            if name in ("roll_body","pitch_body","true_airspeed_m_s","calibrated_airspeed_m_s","true_airspeed_sp","height_rate_reference","altitude_reference","throttle_sp","pitch_sp") or name.startswith(("control[","xyz[")):
                x=v[mask]; print(name,np.percentile(x,[0,50,100]).tolist() if len(x) else [])
