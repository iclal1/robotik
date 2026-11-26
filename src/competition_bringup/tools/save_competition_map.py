#!/usr/bin/env python3
"""Save a unique SLAM map and attach a read-only coverage audit."""
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from audit_saved_map import audit


def main():
    workspace = Path.cwd()
    if not (workspace/'src/competition_bringup').is_dir():
        raise SystemExit('Run from the robotics_competition_ws root')
    session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid4().hex[:8]
    destination = workspace/'maps/competition_mapping'/session
    destination.mkdir(parents=True, exist_ok=False)
    prefix = destination/'map'
    result = subprocess.run(
        ['ros2','run','nav2_map_server','map_saver_cli','-t','/map','-f',str(prefix)],
        capture_output=True, text=True, timeout=40, check=False)
    yaml_path = prefix.with_suffix('.yaml')
    pgm_path = prefix.with_suffix('.pgm')
    if result.returncode or not yaml_path.is_file() or not pgm_path.is_file():
        raise RuntimeError('Map save failed in '+str(destination)+': '+result.stderr)
    shutil.copy2(workspace/'src/competition_bringup/config/slam_mapping.yaml',
                 destination/'slam_mapping.yaml')
    summary = audit(yaml_path)
    summary['mapping_method'] = 'SLAM Toolbox sensor-derived occupancy grid'
    summary['saved_utc'] = datetime.now(timezone.utc).isoformat()
    summary['source_topic'] = '/map'
    summary['competition_world_used_as_map_input'] = False
    (destination/'coverage.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(destination)
    print('Known grid fraction:',round(summary['known_grid_fraction'],3))
    for name, region in summary['regions'].items():
        print(name,round(region['known_fraction'],3))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(error,file=sys.stderr)
        sys.exit(1)
