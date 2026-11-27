#!/usr/bin/env python3
"""YALNIZCA TEST/OLCUM: Gazebo'daki gercek kup konumlarini okuyup bolgede mi diye bakar.
Gorev dugumu (cube_pusher) bunu kullanmaz."""
import re
import subprocess

ZONES = {'red': (2.35, 1.2), 'green': (2.35, 0.0), 'blue': (2.35, -1.2)}
out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/competition/pose/info'],
                     capture_output=True, text=True, timeout=10).stdout
n = 0
for c, (zx, zy) in ZONES.items():
    m = re.search(r'name: "%s_cube".*?position \{(.*?)\}' % c, out, re.S)
    if not m:
        print('  %-5s okunamadi' % c)
        continue
    p = dict(re.findall(r'(\w+): ([-\d.e]+)', m.group(1)))
    x, y = float(p.get('x', 0)), float(p.get('y', 0))
    ok = abs(x - zx) <= 0.21 and abs(y - zy) <= 0.21
    n += ok
    print('  %-5s kup (%.3f, %.3f), bolge merkezinden (%.3f, %.3f) -> %s'
          % (c, x, y, x - zx, y - zy, 'TESLIM' if ok else 'DISARIDA'))
print('  Gercek sonuc: %d/3 kup bolgede' % n)
