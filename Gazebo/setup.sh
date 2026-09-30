#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$HERE/aero" -B "$HOME/stallion_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/stallion_build" -j 4
ctest --test-dir "$HOME/stallion_build" --output-on-failure
python3 "$HERE/build_model.py"
python3 "$HERE/validate_model.py"
gz sdf -k "$HOME/stallion_sitl/models/stallion/model.sdf"
