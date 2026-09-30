"""Generate an isolated Gazebo model/runtime from the supplied scalar parameters."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shlex
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent


def put(parent, name, value):
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def box_visual(link, name, size, pose, color="0.15 0.45 0.8 1"):
    visual = ET.SubElement(link, "visual", name=name)
    put(visual, "pose", pose)
    put(ET.SubElement(ET.SubElement(visual, "geometry"), "box"), "size", size)
    material = ET.SubElement(visual, "material")
    put(material, "ambient", color); put(material, "diffuse", color)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4", type=Path, default=Path.home()/"PX4-Autopilot")
    parser.add_argument("--runtime", type=Path, default=Path.home()/"stallion_sitl")
    parser.add_argument("--plugin-build", type=Path, default=Path.home()/"stallion_build")
    args = parser.parse_args()
    p = json.loads((HERE/"parameters.json").read_text())
    source = args.px4/"Tools/simulation/gz/models/rc_cessna/model.sdf"
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    runtime = args.runtime.resolve()
    model_dir = runtime/"models/stallion"
    model_dir.mkdir(parents=True, exist_ok=True)
    (runtime/"worlds").mkdir(exist_ok=True)
    tree = ET.parse(source)
    model = tree.getroot().find("model")
    model.set("name", "stallion")
    # Keep the stock sensor, joint, wheel and servo infrastructure, replace aero.
    motor_template = None
    for plugin in list(model.findall("plugin")):
        if "LiftDrag" in plugin.get("name", ""):
            model.remove(plugin)
        elif "MulticopterMotorModel" in plugin.get("name", ""):
            motor_template = copy.deepcopy(plugin)
            model.remove(plugin)
    base = model.find("link[@name='base_link']")
    for visual in list(base.findall("visual")):
        base.remove(visual)
    wing_chord = p["S"]/p["b"]
    put(base.find("collision[@name='wings_collision']/geometry/box"), "size", f"{wing_chord} {p['b']} 0.02")
    box_visual(base,"fuselage","0.65 0.08 0.10","-0.14 0 0 0 0 0")
    box_visual(base,"wing",f"{wing_chord} {p['b']} 0.02","-0.01 0 0.07 0 0 0")
    box_visual(base,"horizontal_tail","0.14 0.42 0.015","-0.5 0 0 0 0 0")
    box_visual(base,"vertical_tail","0.14 0.015 0.16","-0.5 0 0.06 0 0 0")
    # Replace old Cessna control meshes with schematic surfaces, not a claimed Stallion CAD model.
    control_visuals = {
        "left_elevon": (f"0.055 {p['b']*.22} 0.012", f"-0.08 {p['b']*.32} 0.07 0 0 0"),
        "right_elevon": (f"0.055 {p['b']*.22} 0.012", f"-0.08 {-p['b']*.32} 0.07 0 0 0"),
        "left_flap": ("0.055 0.15 0.012", "-0.08 0.14 0.07 0 0 0"),
        "right_flap": ("0.055 0.15 0.012", "-0.08 -0.14 0.07 0 0 0"),
        "elevator": ("0.055 0.4 0.012", "-0.56 0 0 0 0 0"),
        "rudder": ("0.055 0.012 0.14", "-0.56 0 0.07 0 0 0"),
    }
    for name,(size,pose) in control_visuals.items():
        link=model.find(f"link[@name='{name}']")
        for visual in list(link.findall("visual")): link.remove(visual)
        box_visual(link,name+"_visual",size,pose,"0.9 0.6 0.1 1")
    rotor=model.find("link[@name='rotor_puller']")
    joint=model.find("joint[@name='rotor_puller_joint']")
    model.remove(rotor); model.remove(joint)
    motor_y=p["b"]*.25
    for i,side in enumerate((1,-1)):
        r=copy.deepcopy(rotor); j=copy.deepcopy(joint); motor=copy.deepcopy(motor_template)
        r.set("name",f"rotor_{i}"); j.set("name",f"rotor_{i}_joint")
        put(r,"pose",f"0.12 {side*motor_y} 0 0 1.57079632679 0")
        put(j,"child",f"rotor_{i}")
        for uri in r.findall(".//uri"):
            uri.text=str(args.px4/"Tools/simulation/gz/models/rc_cessna/meshes/iris_prop_ccw.dae")
        put(motor,"jointName",f"rotor_{i}_joint"); put(motor,"linkName",f"rotor_{i}")
        put(motor,"motorNumber",i); put(motor,"turningDirection","cw" if i==0 else "ccw")
        put(motor,"maxRotVelocity",1000)
        put(motor,"motorConstant",p["T_max"]/p["n_propellers"]/1000**2)
        # No unidentified rotor drag: supplied CD0 covers airframe parasitic drag.
        put(motor,"rotorDragCoefficient",0); put(motor,"rollingMomentCoefficient",0)
        model.extend([r,j,motor])
    original_mass=sum(float(x.text) for x in model.findall("link/inertial/mass"))
    ratio=p["m"]/original_mass
    for link in model.findall("link"):
        inertia=link.find("inertial")
        put(inertia,"mass",float(inertia.findtext("mass"))*ratio)
        for entry in inertia.find("inertia"):
            entry.text=str(float(entry.text)*ratio)
    # Generic inertia estimate: stretch stock base-link mass distribution spanwise.
    inertia=base.find("inertial/inertia")
    ix,iy,iz=[float(inertia.findtext(k)) for k in ("ixx","iyy","izz")]
    x2,y2,z2=(iy+iz-ix)/2,(ix+iz-iy)/2,(ix+iy-iz)/2
    y2*=p["b"]**2  # stock collision span was 1 m
    for key,value in zip(("ixx","iyy","izz"),(y2+z2,x2+z2,x2+y2)): put(inertia,key,value)
    aero=ET.SubElement(model,"plugin",filename="libStallionAerodynamics.so",name="stallion::Aerodynamics")
    for key in ("CL0","CLa","CD0","AR","e","CL_max","rho"): put(aero,key,p[key])
    for key,value in dict(area=p["S"],span=p["b"],mac=p["cbar"],
                          alpha_min=math.radians(p["alpha_min_deg"]),alpha_max=math.radians(p["alpha_max_deg"])).items():
        put(aero,key,value)
    estimates=dict(CY_beta=-.5,Cl_beta=-.06,Cl_p=-.5,Cl_aileron=.18,
                   Cm0=.02,Cm_alpha=-.6,Cm_q=-12,Cm_elevator=1.2,
                   Cn_beta=.1,Cn_r=-.15,Cn_rudder=.12,CY_rudder=-.2)
    for key,value in estimates.items(): put(aero,key,value)
    for key,value in dict(energy_efficiency=p["eta_prop"]*p["eta_motor"]*p["eta_esc"],
                          energy_motor_constant=p["T_max"]/p["n_propellers"]/1000**2,
                          energy_rotor_slowdown=10).items(): put(aero,key,value)
    ET.indent(tree, space="  ")
    tree.write(model_dir/"model.sdf",encoding="utf-8",xml_declaration=True)
    (model_dir/"model.config").write_text('<model><name>Stallion generic</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>Local simulation</name></author><description>User polar; generic estimated rotational dynamics.</description></model>\n')
    world=ET.parse(args.px4/"Tools/simulation/gz/worlds/default.sdf")
    w=world.getroot().find("world"); w.set("name","stallion_world")
    put(w,"gravity",f"0 0 {-p['g']}")
    ET.indent(world,space="  ")
    world.write(runtime/"worlds/stallion_world.sdf",encoding="utf-8",xml_declaration=True)
    k=1/(math.pi*p["e"]*p["AR"])
    cl_opt=math.sqrt((3 if p["cruise_mode"]=="endurance" else 1)*p["CD0"]/k)
    cruise=math.sqrt(2*p["m"]*p["g"]/(p["rho"]*p["S"]*cl_opt))
    vs=math.sqrt(2*p["m"]*p["g"]/(p["rho"]*p["S"]*p["CL_max"]))
    settings={"NAV_DLL_ACT":0,"COM_RC_IN_MODE":4,"FW_AIRSPD_MIN":p["V_min"],
              "FW_AIRSPD_MAX":p["V_max"],"FW_AIRSPD_TRIM":cruise,
              "MIS_TAKEOFF_ALT":p["h_cruise"],"BAT1_N_CELLS":p["nominal_battery_series"],
              "CA_ROTOR_COUNT":2,"CA_ROTOR0_PX":.12,"CA_ROTOR1_PX":.12,
              "CA_ROTOR0_PY":-motor_y,"CA_ROTOR1_PY":motor_y,
              "CA_ROTOR0_AX":1,"CA_ROTOR0_AY":0,"CA_ROTOR0_AZ":0,
              "CA_ROTOR1_AX":1,"CA_ROTOR1_AY":0,"CA_ROTOR1_AZ":0,
              "CA_ROTOR0_KM":0,"CA_ROTOR1_KM":0,
              "SIM_GZ_EC_FUNC1":101,"SIM_GZ_EC_FUNC2":102,
              "SIM_GZ_EC_MIN1":0,"SIM_GZ_EC_MIN2":0,
              "SIM_GZ_EC_MAX1":1000,"SIM_GZ_EC_MAX2":1000,
              "FW_THR_MIN":p["T_min"]/p["T_max"]}
    # Runtime-local environment avoids touching PX4's model tree and default rootfs.
    env={"PX4_GZ_MODELS":runtime/"models","PX4_GZ_WORLDS":runtime/"worlds",
         "PX4_GZ_PLUGINS":args.plugin_build,
         "GZ_SIM_SERVER_CONFIG_PATH":args.px4/"src/modules/simulation/gz_bridge/server.config",
         "GZ_SIM_SYSTEM_PLUGIN_PATH":str(args.plugin_build)+":"+str(args.px4/"build/px4_sitl_default/src/modules/simulation/gz_plugins"),
         "GZ_SIM_RESOURCE_PATH":str(runtime/"models")+":"+str(runtime/"worlds")+":"+str(args.px4/"Tools/simulation/gz/models")}
    (runtime/"gz_env.sh").write_text("\n".join(f"export {key}={shlex.quote(str(value))}" for key,value in env.items())+"\n")
    launch_env={"PX4_SYS_AUTOSTART":4003,"PX4_SIM_MODEL":"gz_stallion",
                "PX4_GZ_WORLD":"stallion_world","HEADLESS":1,"GZ_IP":"127.0.0.1"}
    launch_env.update({"PX4_PARAM_"+key:value for key,value in settings.items()})
    script="#!/usr/bin/env bash\nset -euo pipefail\n"
    script+=f"cd {shlex.quote(str(runtime))}\n"
    script+="\n".join(f"export {key}={shlex.quote(str(value))}" for key,value in launch_env.items())+"\n"
    # PX4 links the supplied startup directory as runtime/etc; logs/parameters stay here.
    script+=f"exec {shlex.quote(str(args.px4/'build/px4_sitl_default/bin/px4'))} {shlex.quote(str(args.px4/'build/px4_sitl_default/etc'))} -w {shlex.quote(str(runtime))}\n"
    (runtime/"start_px4.sh").write_text(script)
    eta=p["eta_prop"]*p["eta_motor"]*p["eta_esc"]
    drag=.5*p["rho"]*cruise**2*p["S"]*(p["CD0"]+k*cl_opt**2)
    report=dict(model="stallion",source_model_sha256=source_hash,total_mass_kg=sum(float(x.text) for x in model.findall("link/inertial/mass")),
                applied_inputs=p,assumptions=dict(thrust="20 N combined, 10 N per propeller",altitude="above starting point",AR="5.6 retained for drag; geometry implies 6.77585",
                inertia="Stock inertia scaled with mass and base span; unvalidated estimate",CG="stock base origin and sensor/wheel offsets retained",
                moments=estimates,visuals="schematic geometry; not manufacturer CAD",post_stall="bounded generic extension beyond alpha limits",
                motor_response="stock time constants and reaction torque coefficient; no propeller map",
                battery="4S PX4 configuration only; no measured discharge/capacity model",
                efficiencies="used for steady-flight electrical power estimate, not applied again to already specified thrust"),
                derived=dict(k=k,eta_total=eta,stall_speed_mps=vs,selected_cruise_mps=cruise,
                             ideal_cruise_drag_N=drag,ideal_cruise_electric_power_W=drag*cruise/eta),
                px4_parameters=settings,runtime=str(runtime),
                warnings=["V_min=12.3 is below calculated level-flight stall speed 12.84 m/s.",
                          "Endurance cruise 13.67 m/s leaves little turning margin. Model implementation is not flight validation.",
                          "CL_max clips the linear law above 6.77 degrees, before alpha_max=10 degrees.",
                          "4S and efficiencies do not determine battery endurance without capacity and propulsion data."])
    (HERE/"model_report.json").write_text(json.dumps(report,indent=2)+"\n")
    (runtime/"model_report.json").write_text(json.dumps(report,indent=2)+"\n")
    assert hashlib.sha256(source.read_bytes()).hexdigest()==source_hash
    assert abs(report["total_mass_kg"]-p["m"])<1e-9
    print(json.dumps({"runtime":str(runtime),"total_mass_kg":report["total_mass_kg"],"derived":report["derived"]},indent=2))


if __name__=="__main__": main()
