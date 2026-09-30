"""Create a small synthetic flight check, NOT an optimized savings experiment."""
import json
from pathlib import Path
from prepare_missions import prepare
from test_energy import write_test_wave

root = Path(__file__).resolve().parent/'validation_missions'
root.mkdir(exist_ok=True)
source = root/'synthetic_wave.csv'
write_test_wave(source)
out = root/'smoke_40s'
prepare(source, out, periods=1, warmup=1)
for kind in ('steady', 'periodic'):
    path = out/f'{kind}.json'
    data = json.loads(path.read_text())
    data['validation_only'] = True
    path.write_text(json.dumps(data, indent=2)+'\n')
