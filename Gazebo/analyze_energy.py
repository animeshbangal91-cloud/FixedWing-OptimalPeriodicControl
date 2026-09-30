"""Integrate measured Gazebo energy proxy; never use the CSV reference thrust as flown thrust."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def integral(t, y, start, end):
    t, y = np.asarray(t), np.asarray(y)
    if len(t) < 2 or np.any(np.diff(t) <= 0) or start < t[0] or end > t[-1] or end <= start:
        raise ValueError('Invalid or incomplete integration window')
    inside = (t > start) & (t < end)
    times = np.r_[start, t[inside], end]
    values = np.r_[np.interp(start, t, y), y[inside], np.interp(end, t, y)]
    return float(np.trapezoid(values, times))


def analyze(out):
    out = Path(out)
    meta = json.loads((out/'run.json').read_text())
    if 'score_start_sim_s' not in meta:
        print('No scored window: no energy comparison produced.')
        return None
    energy = pd.read_csv(out/'energy.csv').drop_duplicates('gazebo_s').sort_values('gazebo_s')
    actual = pd.read_csv(out/'actual.csv').drop_duplicates('sim_s').sort_values('sim_s')
    commands = pd.read_csv(out/'commands.csv').drop_duplicates('sim_s').sort_values('sim_s')
    battery = pd.read_csv(out/'battery.csv').drop_duplicates('sim_s').sort_values('sim_s')
    if len(energy) < 3:
        raise ValueError('Missing energy telemetry')
    # Both simulation clocks advance at the same rate. Record arrival skew,
    # and integrate on the original Gazebo timestamps, not wall time.
    offsets = energy.sim_s-energy.gazebo_s
    offset = float(offsets.median())
    energy['aligned_s'] = energy.gazebo_s+offset
    start, end = meta['score_start_sim_s'], meta['score_end_sim_s']
    complete = (meta['status'] == 'completed_hold_confirmed' and
                energy.aligned_s.iloc[0] <= start and energy.aligned_s.iloc[-1] >= end and
                actual.sim_s.iloc[-1] >= end and commands.sim_s.iloc[-1] >= end)
    if not complete:
        summary = dict(status='incomplete', run_status=meta['status'], comparison_valid=False)
        (out/'energy_metrics.json').write_text(json.dumps(summary, indent=2)+'\n')
        print('Incomplete scored window; no savings result can be calculated.')
        return summary
    window = energy[(energy.aligned_s >= start) & (energy.aligned_s <= end)].copy()
    clock_error = float(np.quantile(np.abs(offsets-offset), .95))
    sample_gap = float(np.diff(window.gazebo_s).max())
    e = integral(energy.aligned_s, energy.power_matlab_W, start, end)
    eaxis = integral(energy.aligned_s, energy.power_axis_W, start, end)
    heading = meta['experiment_entry_ne_heading'][2]
    actual['along'] = actual.north*np.cos(heading)+actual.east*np.sin(heading)
    commands['along'] = commands.north*np.cos(heading)+commands.east*np.sin(heading)
    distance = float(np.interp(end, actual.sim_s, actual.along)-np.interp(start, actual.sim_s, actual.along))
    t = window.aligned_s.to_numpy()
    ref_n = np.interp(t, commands.sim_s, commands.north)
    ref_e = np.interp(t, commands.sim_s, commands.east)
    ref_d = np.interp(t, commands.sim_s, commands.down)
    act_n = np.interp(t, actual.sim_s, actual.north)
    act_e = np.interp(t, actual.sim_s, actual.east)
    act_d = np.interp(t, actual.sim_s, actual.down)
    speeds = pd.read_csv(out/'speed_control.csv')
    ref_v = np.interp(t, speeds.sim_s, speeds.requested_airspeed_mps)
    # Use PX4 local height for endpoint energy. Over a long horizontal flight,
    # Gazebo world Z and PX4's geodetic local-altitude convention diverge by
    # the Earth-curvature projection (about 30 m over this 16 km test).
    local_height = meta['origin_down_m']-actual['down']
    height_change = float(np.interp(end, actual.sim_s, local_height)-np.interp(start, actual.sim_s, local_height))
    v0, v1 = np.interp([start, end], energy.aligned_s, energy.airspeed_mps)
    p = meta['model_configuration']['applied_inputs']
    mechanical_change = p['m']*p['g']*height_change+.5*p['m']*(v1*v1-v0*v0)
    expected_distance = meta['experiment']['scored_distance_m']
    summary = dict(status='completed', kind=meta['experiment']['kind'], duration_s=end-start,
                   validation_only=meta['experiment'].get('validation_only', False),
                   energy_matlab_proxy_J=e, energy_matlab_proxy_Wh=e/3600,
                   energy_axis_proxy_J=eaxis, average_power_W=e/(end-start),
                   actual_distance_m=distance, target_distance_m=expected_distance,
                   actual_average_forward_speed_mps=distance/(end-start),
                   target_average_forward_speed_mps=meta['experiment']['matched_average_speed_mps'],
                   distance_error_pct=100*(distance-expected_distance)/expected_distance,
                   altitude_rmse_m=float(np.sqrt(np.mean((ref_d-act_d)**2))),
                   airspeed_rmse_mps=float(np.sqrt(np.mean((window.airspeed_mps-ref_v)**2))),
                   position_rmse_m=float(np.sqrt(np.mean((ref_n-act_n)**2+(ref_e-act_e)**2+(ref_d-act_d)**2))),
                   endpoint_height_change_m=height_change, endpoint_airspeed_change_mps=float(v1-v0),
                   endpoint_mechanical_energy_change_J=float(mechanical_change),
                   clock_offset_s=offset, clock_arrival_jitter_p95_s=clock_error, max_sample_gap_s=sample_gap,
                   measurement='constant-efficiency propulsion proxy from actual rotor speeds and airspeed, not battery SOC')
    battery_window = battery[(battery.sim_s >= start) & (battery.sim_s <= end)].dropna(subset=['remaining_pct'])
    if len(battery_window) >= 2:
        b0, b1 = battery_window.iloc[0], battery_window.iloc[-1]
        summary['px4_battery'] = dict(
            start_remaining_pct=float(b0.remaining_pct), end_remaining_pct=float(b1.remaining_pct),
            used_percentage_points=float(b0.remaining_pct-b1.remaining_pct),
            start_voltage_V=float(b0.voltage_V), end_voltage_V=float(b1.voltage_V),
            consumed_mAh_delta=float(b1.consumed_mAh-b0.consumed_mAh) if np.isfinite(b0.consumed_mAh+b1.consumed_mAh) else None,
            consumed_Wh_delta=float(b1.consumed_Wh-b0.consumed_Wh) if np.isfinite(b0.consumed_Wh+b1.consumed_Wh) else None,
            interpretation='PX4 SITL battery simulator telemetry; not accepted as load-dependent energy measurement')
    else:
        summary['px4_battery'] = None
    summary['measurement_valid'] = bool(clock_error < .2 and sample_gap < .25 and e > 0)
    summary['tracking_acceptable'] = bool(summary['altitude_rmse_m'] < 3 and summary['airspeed_rmse_mps'] < .5
        and abs(summary['distance_error_pct']) < 1 and abs(height_change) < 2 and abs(v1-v0) < .5)
    summary['comparison_valid'] = summary['measurement_valid'] and summary['tracking_acceptable']
    summary['acceptance_limits'] = dict(altitude_rmse_m=3, airspeed_rmse_mps=.5, distance_error_pct=1,
                                        endpoint_height_change_m=2, endpoint_airspeed_change_mps=.5)
    summary['periods'] = []
    for i in range(meta['experiment']['periods']):
        a = start+i*meta['experiment']['period_s']
        b = a+meta['experiment']['period_s']
        summary['periods'].append(dict(period=i+1, energy_J=integral(energy.aligned_s, energy.power_matlab_W, a, b)))
    (out/'energy_metrics.json').write_text(json.dumps(summary, indent=2)+'\n')
    window['score_elapsed_s'] = t-start
    window.to_csv(out/'scored_energy.csv', index=False)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    elapsed = t-start
    ax[0,0].plot(elapsed, meta['origin_down_m']-ref_d, label='Streamed')
    ax[0,0].plot(elapsed, meta['origin_down_m']-act_d, label='PX4 actual')
    ax[0,0].set_ylabel('Height (m)'); ax[0,0].legend()
    ax[0,1].plot(elapsed, ref_v, label='Requested airspeed')
    ax[0,1].plot(elapsed, window.airspeed_mps, label='Gazebo actual')
    ax[0,1].set_ylabel('Airspeed (m/s)'); ax[0,1].legend()
    ax[1,0].plot(elapsed, window.power_matlab_W, label='T V / eta')
    ax[1,0].plot(elapsed, window.power_axis_W, label='T V_body_forward / eta', alpha=.6)
    ax[1,0].set_ylabel('Modeled electrical power (W)'); ax[1,0].legend()
    ax[1,1].plot(elapsed, np.sqrt((ref_n-act_n)**2+(ref_e-act_e)**2+(ref_d-act_d)**2))
    ax[1,1].set_ylabel('Time-aligned position error (m)')
    for a in ax.flat:
        a.grid(True); a.set_xlabel('Scored simulation time (s)')
    fig.suptitle(f"{summary['kind']} | {summary['average_power_W']:.2f} W | tracking accepted: {summary['tracking_acceptable']}")
    fig.tight_layout(); fig.savefig(out/'energy_tracking.png', dpi=150); fig.savefig(out/'energy_tracking.pdf'); plt.close(fig)
    print(json.dumps(summary, indent=2), flush=True)
    return summary


def compare(steady, periodic, out):
    steady, periodic, out = Path(steady), Path(periodic), Path(out)
    a, b = [json.loads((p/'energy_metrics.json').read_text()) for p in (steady, periodic)]
    ma, mb = [json.loads((p/'run.json').read_text()) for p in (steady, periodic)]
    if a.get('kind') != 'steady' or b.get('kind') != 'periodic':
        raise ValueError('Expected completed steady then periodic runs')
    for key in ('source_sha256', 'periods', 'warmup_periods', 'matched_average_speed_mps'):
        if ma['experiment'][key] != mb['experiment'][key]:
            raise ValueError(f'Unmatched experiment configuration: {key}')
    if ma['model_configuration'] != mb['model_configuration'] or ma['tuning'] != mb['tuning'] or ma['px4_commit'] != mb['px4_commit']:
        raise ValueError('Model, controller tuning or PX4 revision differs')
    if ma.get('mission_controller') != mb.get('mission_controller'):
        raise ValueError('Mission tracking controller differs')
    valid = a['comparison_valid'] and b['comparison_valid']
    result = dict(comparison_valid=valid, steady_run=str(steady), periodic_run=str(periodic),
                  observed_proxy_energy_difference_pct=100*(1-b['energy_matlab_proxy_J']/a['energy_matlab_proxy_J']),
                  battery_savings_pct=None,
                  interpretation='Matched tracking accepted; modeled propulsion energy only' if valid else
                  'Tracking/measurement criteria failed: observed difference is not a validated mission saving',
                  steady=a, periodic=b)
    if a.get('px4_battery') and b.get('px4_battery'):
        steady_used = a['px4_battery']['used_percentage_points']
        periodic_used = b['px4_battery']['used_percentage_points']
        result['px4_battery_used_percentage_points'] = dict(steady=steady_used, periodic=periodic_used)
        result['px4_battery_usage_difference_points'] = steady_used-periodic_used
        result['px4_battery_relative_saving_pct'] = (100*(steady_used-periodic_used)/steady_used
                                                      if steady_used > 0 else None)
        result['px4_battery_result_valid'] = False
        result['px4_battery_caveat'] = ('Logged and compared as requested, but PX4 SITL battery drain is time-based; '
                                        'it is not a load-dependent battery-savings measurement.')
    out.mkdir(parents=True, exist_ok=True)
    (out/'comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(11, 10))
    for run, label in ((steady, 'Steady'), (periodic, 'Periodic')):
        d = pd.read_csv(run/'scored_energy.csv')
        t, power = d.score_elapsed_s.to_numpy(), d.power_matlab_W.to_numpy()
        cumulative = np.r_[0, np.cumsum(.5*(power[1:]+power[:-1])*np.diff(t))]/3600
        axes[0].plot(t, power, label=label)
        axes[1].plot(t, cumulative, label=label)
        battery = pd.read_csv(run/'battery.csv')
        run_meta = json.loads((run/'run.json').read_text())
        battery = battery[(battery.sim_s >= run_meta['score_start_sim_s']) &
                          (battery.sim_s <= run_meta['score_end_sim_s'])].dropna(subset=['remaining_pct'])
        axes[2].plot(battery.sim_s-run_meta['score_start_sim_s'], battery.remaining_pct, label=label)
    axes[0].set_ylabel('Modeled electrical power (W)'); axes[1].set_ylabel('Cumulative proxy energy (Wh)')
    axes[2].set_ylabel('PX4 battery remaining (%)')
    for ax in axes:
        ax.grid(True); ax.legend(); ax.set_xlabel('Scored simulation time (s)')
    fig.suptitle(f"Observed energy difference: {result['observed_proxy_energy_difference_pct']:.3f}% | comparison accepted: {valid}")
    fig.tight_layout(); fig.savefig(out/'comparison.png', dpi=150); fig.savefig(out/'comparison.pdf'); plt.close(fig)
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('runs', nargs='+', type=Path)
    p.add_argument('--out', type=Path, default=Path('energy_comparison'))
    a = p.parse_args()
    if len(a.runs) == 1: analyze(a.runs[0])
    elif len(a.runs) == 2: compare(*a.runs, a.out)
    else: p.error('Supply one run to analyze, or steady and periodic runs to compare')
