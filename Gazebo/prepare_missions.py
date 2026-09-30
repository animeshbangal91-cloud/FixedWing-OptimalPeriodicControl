"""Prepare a matched pair; preserves the supplied waveform and exact average speed."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import csv

from mission_profile import read_period


def prepare(source, out, periods=5, warmup=1, average_speed=13.67):
    source, out = Path(source), Path(out)
    data = read_period(source)
    period = float(data['time_s'][-1])
    speed = float(data['X_m'][-1]/period)
    if abs(speed-average_speed) > .01:
        raise ValueError(f'CSV average {speed:.6f} differs from requested {average_speed:.6f}; '
                         'regenerate the optimized trajectory or explicitly select its average. No time scaling applied.')
    if periods < 1 or warmup < 0:
        raise ValueError('Invalid period count')
    out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(source, out/'period.csv')
    # Materialize the five scored cycles for inspection and plotting. The
    # runtime still reads period.csv and repeats it from simulation time.
    names = list(data.dtype.names)
    energy_per_period = float(data['cumulative_energy_J'][-1]) if 'cumulative_energy_J' in names else 0.0
    with (out/f'periodic_{periods}_periods.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(names)
        for cycle in range(periods):
            stop = len(data) if cycle == periods-1 else len(data)-1
            for row in data[:stop]:
                values = [row[name].item() for name in names]
                values[names.index('time_s')] += cycle*period
                values[names.index('X_m')] += cycle*float(data['X_m'][-1])
                if 'cumulative_energy_J' in names:
                    values[names.index('cumulative_energy_J')] += cycle*energy_per_period
                writer.writerow(values)
    common = dict(source_csv='period.csv', source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  periods=periods, warmup_periods=warmup, period_s=period,
                  requested_average_speed_mps=average_speed, matched_average_speed_mps=speed,
                  scored_duration_s=periods*period, scored_distance_m=periods*float(data['X_m'][-1]),
                  energy_model='actual Gazebo rotor thrust * actual airspeed / 0.5712; not battery SOC')
    for kind in ('steady', 'periodic'):
        (out/f'{kind}.json').write_text(json.dumps(dict(common, kind=kind), indent=2)+'\n')
    print(json.dumps(common, indent=2))
    return common


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--periods', type=int, default=5)
    p.add_argument('--warmup-periods', type=int, default=1)
    p.add_argument('--average-speed', type=float, default=13.67)
    a = p.parse_args()
    prepare(a.source, a.out, a.periods, a.warmup_periods, a.average_speed)
