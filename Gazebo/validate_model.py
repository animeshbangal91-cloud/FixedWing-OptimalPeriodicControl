"""Check generated physics parameters and ensure the stock source was preserved."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path.home()/"stallion_sitl"
report = json.loads((root/"model_report.json").read_text())
p = report["applied_inputs"]
model = ET.parse(root/"models/stallion/model.sdf").getroot().find("model")
mass = sum(float(e.text) for e in model.findall("link/inertial/mass"))
assert math.isclose(mass, p["m"], abs_tol=1e-10)
motors = [e for e in model.findall("plugin") if e.get("name").endswith("MulticopterMotorModel")]
assert len(motors) == p["n_propellers"]
assert math.isclose(sum(float(e.findtext("motorConstant"))*float(e.findtext("maxRotVelocity"))**2 for e in motors), p["T_max"])
assert {e.findtext("motorNumber") for e in motors} == {"0", "1"}
assert {e.findtext("turningDirection") for e in motors} == {"cw", "ccw"}
aero = model.find("plugin[@name='stallion::Aerodynamics']")
for key in ("CL0", "CLa", "CD0", "AR", "e", "CL_max", "rho"):
    assert math.isclose(float(aero.findtext(key)), p[key])
for tag,key in (("span","b"),("area","S"),("mac","cbar")):
    assert math.isclose(float(aero.findtext(tag)), p[key])
size = model.findtext("link[@name='base_link']/collision[@name='wings_collision']/geometry/box/size")
chord, span, _ = map(float, size.split())
assert math.isclose(chord*span, p["S"])
assert math.isclose(span, p["b"])
for inertia in model.findall("link/inertial/inertia"):
    x,y,z = [float(inertia.findtext(k)) for k in ("ixx","iyy","izz")]
    assert min(x,y,z)>0 and x+y>=z and x+z>=y and y+z>=x
gravity = ET.parse(root/"worlds/stallion_world.sdf").getroot().findtext("world/gravity")
assert math.isclose(float(gravity.split()[2]), -p["g"])
source = Path.home()/"PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/model.sdf"
assert hashlib.sha256(source.read_bytes()).hexdigest() == report["source_model_sha256"]
print("Model checks passed: total mass, inertia, wing dimensions, polar, gravity, twin thrust and unchanged stock source.")
