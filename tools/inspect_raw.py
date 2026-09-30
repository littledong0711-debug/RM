"""Inspect RGB channel populations in saved NumPy buffers without dependencies."""
import ast
import json
import struct
import sys
from pathlib import Path

for path in sorted(Path(sys.argv[1]).glob('*_raw.npy')):
    with path.open('rb') as stream:
        assert stream.read(6) == b'\x93NUMPY'
        major, minor = stream.read(2)
        size = struct.unpack('<H' if major == 1 else '<I', stream.read(2 if major == 1 else 4))[0]
        header = ast.literal_eval(stream.read(size).decode('latin1'))
        assert header['descr'] == '|u1'
        raw = stream.read()
    metadata = json.loads(path.with_name(path.name.replace('_raw.npy', '.json')).read_text())
    width, height = metadata['width'], metadata['height']
    counts = [0, 0, 0, 0]
    for y in range(height):
        for x in range(width):
            i = y * metadata['step'] + 4 * x
            r, g, b = raw[i:i+3]
            index = 0 if y < height * 0.86 else 2
            counts[index] += int(r > 100 and r - b > 100 and r - g > 35)
            counts[index+1] += int(b > 100 and b - r > 100 and b - g > 35)
    print(path.stem, 'field red/blue, bottom red/blue:', counts)
