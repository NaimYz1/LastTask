#!/usr/bin/env python
"""Generate the static map (PGM + YAML) for the FYP arena.

The map must contain ONLY the permanent structure of
fyp_jackal_gazebo/worlds/fyp_arena.world:
    * outer walls
    * divider wall with its doorway
    * three landmark boxes
The bridge and the tripod are deliberately NOT in the map - they are the
"unexpected" obstacles the robot has to handle with its sensors.

Pure Python (2 or 3), no dependencies. Run from anywhere:
    python scripts/generate_map.py
Output goes next to this script's repository:
    fyp_jackal_navigation/maps/fyp_arena.{pgm,yaml}
"""
import os

RESOLUTION = 0.05          # metres / pixel
ORIGIN_X, ORIGIN_Y = -6.5, -5.5   # world coords of the map's lower-left corner
WIDTH_M, HEIGHT_M = 13.0, 11.0

FREE, OCCUPIED, UNKNOWN = 254, 0, 205

# Rectangles as (xmin, xmax, ymin, ymax) in world metres.
FREE_RECTS = [
    (-6.0, 6.0, -5.0, 5.0),            # arena interior
]
OCCUPIED_RECTS = [
    (-6.0, 6.0, 5.0, 5.15),            # north wall
    (-6.0, 6.0, -5.15, -5.0),          # south wall
    (6.0, 6.15, -5.15, 5.15),          # east wall
    (-6.15, -6.0, -5.15, 5.15),        # west wall
    (2.425, 2.575, 1.1, 5.0),          # divider, north segment
    (2.425, 2.575, -5.0, -1.1),        # divider, south segment
    (-2.25, -1.75, 2.75, 3.25),        # landmark 1
    (-4.75, -4.25, -3.75, -3.25),      # landmark 2
    (4.25, 4.75, 3.25, 3.75),          # landmark 3
]


def main():
    cols = int(round(WIDTH_M / RESOLUTION))
    rows = int(round(HEIGHT_M / RESOLUTION))
    grid = [[UNKNOWN] * cols for _ in range(rows)]

    def paint(rect, value):
        xmin, xmax, ymin, ymax = rect
        for r in range(rows):
            # row 0 is the TOP of the image = maximum y
            y = ORIGIN_Y + HEIGHT_M - (r + 0.5) * RESOLUTION
            if not (ymin <= y <= ymax):
                continue
            for c in range(cols):
                x = ORIGIN_X + (c + 0.5) * RESOLUTION
                if xmin <= x <= xmax:
                    grid[r][c] = value

    for rect in FREE_RECTS:
        paint(rect, FREE)
    for rect in OCCUPIED_RECTS:
        paint(rect, OCCUPIED)

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           os.pardir, 'fyp_jackal_navigation', 'maps')
    out_dir = os.path.normpath(out_dir)
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    pgm_path = os.path.join(out_dir, 'fyp_arena.pgm')
    with open(pgm_path, 'wb') as f:
        header = 'P5\n# FYP arena static map\n%d %d\n255\n' % (cols, rows)
        f.write(header.encode('ascii'))
        f.write(bytearray(v for row in grid for v in row))

    yaml_path = os.path.join(out_dir, 'fyp_arena.yaml')
    with open(yaml_path, 'w') as f:
        f.write('image: fyp_arena.pgm\n')
        f.write('resolution: %.3f\n' % RESOLUTION)
        f.write('origin: [%.2f, %.2f, 0.0]\n' % (ORIGIN_X, ORIGIN_Y))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')

    print('Wrote %s (%dx%d) and %s' % (pgm_path, cols, rows, yaml_path))


if __name__ == '__main__':
    main()
