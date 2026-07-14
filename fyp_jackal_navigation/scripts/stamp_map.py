#!/usr/bin/env python
"""Stamp a permanent obstacle (occupied rectangle) into a static map PGM.

For furniture the lidars see poorly (glossy curved kayak hull) the live
marks under-represent the true extent and the planner shaves too close.
Painting the object into the static map makes the global planner route
around it deterministically, every run.

Get the two opposite corners in RViz: select the "Publish Point" tool,
click a corner of the object (add ~10 cm margin), and read the coordinates
from a terminal running:
    rostopic echo /clicked_point

Then (paths relative to where you run it):
    python stamp_map.py <map.yaml> <x1> <y1> <x2> <y2>
e.g.
    python stamp_map.py ~/fyp_ws/src/fyp_jackal/fyp_jackal_navigation/maps/myroom.yaml \
        -1.8 1.2 -0.9 2.4

A one-time backup <image>.orig is kept next to the PGM. Restart
nav_real.launch afterwards (map_server reads the map at startup).
"""
import shutil
import sys
import os

import yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise ValueError('only binary P5 PGM supported')
    # parse header tokens (magic, width, height, maxval), skipping comments
    tokens = []
    i = 2
    while len(tokens) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(int(data[i:j]))
        i = j
    i += 1  # single whitespace after maxval
    width, height, _maxval = tokens
    pixels = bytearray(data[i:i + width * height])
    return data[:i], width, height, pixels


def main():
    if len(sys.argv) != 6:
        print(__doc__)
        sys.exit(1)
    yaml_path = os.path.expanduser(sys.argv[1])
    x1, y1, x2, y2 = [float(v) for v in sys.argv[2:6]]

    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    res = float(info['resolution'])
    ox, oy = float(info['origin'][0]), float(info['origin'][1])
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                            info['image'])

    backup = pgm_path + '.orig'
    if not os.path.isfile(backup):
        shutil.copyfile(pgm_path, backup)
        print('backup saved: %s' % backup)

    header, width, height, pixels = read_pgm(pgm_path)
    occupied = 0 if not info.get('negate', 0) else 254

    xmin, xmax = sorted((x1, x2))
    ymin, ymax = sorted((y1, y2))
    count = 0
    for row in range(height):
        y = oy + (height - row - 0.5) * res   # row 0 = top = max y
        if not (ymin <= y <= ymax):
            continue
        for col in range(width):
            x = ox + (col + 0.5) * res
            if xmin <= x <= xmax:
                pixels[row * width + col] = occupied
                count += 1
    with open(pgm_path, 'wb') as f:
        f.write(header)
        f.write(bytes(bytearray(pixels)))
    print('stamped %d cells occupied over x[%.2f, %.2f] y[%.2f, %.2f] in %s'
          % (count, xmin, xmax, ymin, ymax, pgm_path))
    print('restart nav_real.launch to load the updated map.')


if __name__ == '__main__':
    main()
