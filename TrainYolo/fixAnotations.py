import os
import numpy as np
import supervision as sv
from tqdm import tqdm

# Define image dimensions
img_width = 640  # replace with actual width
img_height = 640  # replace with actual height

def normalize_box(x1, y1, x2, y2):
    cx = (x1 + x2) / 2 / img_width
    cy = (y1 + y2) / 2 / img_height
    w = abs(x2 - x1) / img_width
    h = abs(y2 - y1) / img_height
    return cx, cy, w, h

label_files = os.listdir('dataset/test/labels')
for label_path in tqdm(label_files, desc="Processing labels"):
    full_path = os.path.join('dataset/test/labels', label_path)
    with open(full_path, 'r') as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        parts = line.strip().split()
        class_id = parts[0]
        coords = list(map(float, parts[1:]))

        if len(coords) > 4:  # Likely a polygon
            # Convert normalized to pixel coordinates
            pixel_points = np.array([
                [coords[i] * img_width, coords[i+1] * img_height]
                for i in range(0, len(coords), 2)
            ])
            # Convert to bounding box using supervision
            x1, y1, x2, y2 = sv.polygon_to_xyxy(pixel_points)
            cx, cy, w, h = normalize_box(x1, y1, x2, y2)
            new_lines.append(f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
            # print(f"Converted {line.strip()} \nto {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        else:
            new_lines.append(line + '\n')

    with open(full_path, 'w') as f:
        f.writelines(new_lines)
