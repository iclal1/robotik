"""Full-resolution color and silhouette measurements for competition cubes.

Scores are heuristic evidence scores, not calibrated probabilities.
"""
import math

import cv2
import numpy as np


RANGES = {
    'red': ((0, 90, 45), (12, 255, 255)),
    'green': ((35, 75, 40), (90, 255, 255)),
    'blue': ((95, 75, 35), (135, 255, 255)),
}


def detect_cube_candidates(image, fx, cx, min_area=240, min_side=16):
    """Measure each separate colored silhouette at native camera resolution."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    results = []
    for color, (low, high) in RANGES.items():
        mask = cv2.inRange(hsv, np.array(low, np.uint8),
                           np.array(high, np.uint8))
        if color == 'red':
            mask |= cv2.inRange(hsv, np.array((170, 90, 45), np.uint8),
                                np.array((179, 255, 255), np.uint8))
        # A small closing operation reconnects antialiased edge pixels while
        # retaining distinct cubes and the measured lower boundary.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area < min_area or w < min_side or h < min_side:
                continue
            clipped = (x <= 1 or y <= 1 or x + w >= image.shape[1] - 1
                       or y + h >= image.shape[0] - 1)
            fill = area / (w * h)
            solidity = area / max(cv2.contourArea(cv2.convexHull(contour)), 1)
            aspect = w / h
            regular = fill >= .53 and solidity >= .83 and .43 <= aspect <= 2.1
            irregular = (area >= 600 and fill >= .35 and solidity >= .65
                         and .30 <= aspect <= 3.0)
            if not clipped and not (regular or irregular):
                continue
            if clipped and area < 1000:
                continue
            # A high percentile of central bottom-edge pixels retains the
            # ground contact while rejecting isolated noisy pixels.
            central_mask = mask[y:y+h, x + w//4:x + 3*w//4]
            columns = np.flatnonzero(np.any(central_mask, axis=0))
            bottoms = [np.flatnonzero(central_mask[:, col])[-1] + y
                       for col in columns]
            bottom = float(np.percentile(bottoms, 90)) if bottoms else float(y+h-1)
            results.append({
                'color': color, 'bbox': [x, y, w, h],
                'bearing_rad': math.atan2(x + .5*w - cx, fx),
                'area_px': float(area), 'partial_view': clipped or not regular,
                'requires_track': not clipped and not regular,
                'contact_pixel': [x + .5*w, bottom],
                'fill_ratio': float(fill), 'solidity': float(solidity),
            })
    return sorted(results, key=lambda item: (-item['area_px'], item['color']))
