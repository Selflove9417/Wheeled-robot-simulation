#!/usr/bin/env python3
"""Compatibility module for the shared offline flight-momentum analyzer."""
from pathlib import Path as _Path
from runpy import run_path as _run_path

_implementation = (_Path(__file__).resolve().parents[3] /
                   "experiments/jump/tools/analysis/analyze_flight_momentum.py")
globals().update({name: value for name, value in
                  _run_path(str(_implementation)).items()
                  if not name.startswith("__")})

if __name__ == "__main__":
    main()
