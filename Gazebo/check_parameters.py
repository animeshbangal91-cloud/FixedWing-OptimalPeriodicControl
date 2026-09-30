"""Check user-supplied inputs; does not modify or run PX4/Gazebo."""
import json
import math
from pathlib import Path

p = json.loads(Path(__file__).with_name("parameters.json").read_text())
weight = p["m"] * p["g"]
k = 1 / (math.pi * p["e"] * p["AR"])


def speed_for_cl(cl):
    return math.sqrt(2 * weight / (p["rho"] * p["S"] * cl))


derived = {
    "AR_from_span_and_area": p["b"] ** 2 / p["S"],
    "k_using_supplied_AR": k,
    "eta_total": p["eta_prop"] * p["eta_motor"] * p["eta_esc"],
    "level_flight_stall_speed_mps": speed_for_cl(p["CL_max"]),
    "required_CL_at_V_min": 2 * weight / (p["rho"] * p["S"] * p["V_min"] ** 2),
    "linear_CL_at_alpha_max": p["CL0"] + p["CLa"] * math.radians(p["alpha_max_deg"]),
    "linear_alpha_at_CL_max_deg": math.degrees((p["CL_max"] - p["CL0"]) / p["CLa"]),
    "ideal_endurance_speed_mps": speed_for_cl(math.sqrt(3 * p["CD0"] / k)),
    "ideal_range_speed_mps": speed_for_cl(math.sqrt(p["CD0"] / k)),
}
print(json.dumps(derived, indent=2))
print("\nAnalytical checks only: steady, level flight; supplied density and AR;")
print("parabolic drag polar and constant propulsion efficiency. Not flight results.")
