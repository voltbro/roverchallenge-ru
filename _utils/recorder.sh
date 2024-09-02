#!/usr/bin/env bash

filename=$(python3 << EOS
import os
from pathlib import Path
PROJECT_ROOT = Path(os.environ["PROJECT_ROOT"])
data_dir = PROJECT_ROOT / "data"
def get_id():
    dirs = [dir_ for dir_ in data_dir.iterdir() if dir_.is_dir()]
    int_suffixes = set()
    for dir_ in dirs:
        parts = dir_.name.split("_")
        if len(parts) < 2:
            continue
        suffix = parts[-1]
        int_suff = None
        try:
            int_suff = int(suffix)
        except ValueError:
            continue
        else:
            int_suffixes.add(int_suff)
    if not int_suffixes:
        return 0
    max_int = max(int_suffixes)
    if max_int is not None:
        return max_int + 1
    else:
        return 0
print(f"{data_dir.absolute()}/run_{get_id()}")
EOS
)

echo "Recording to ${filename}"
ros2 bag record -o ${filename} odom
