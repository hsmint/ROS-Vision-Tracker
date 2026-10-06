"""Find a colored target with valid aligned metric depth."""

import cv2
import numpy as np


def detect_target(rgb, depth, lower, upper, min_area, max_area_ratio,
                  min_depth, max_depth, min_valid_ratio, *, return_bbox=False):
    """Return normalized x/y error and area ratio, or zeros when absent.

    Choose the largest valid color contour, breaking area ties by distance
    to image center. Depth is used only for validation, never as point.z.
    With return_bbox=True, return (target, (left, top, width, height)); the
    bounding box is None when no target passes the color and depth checks.
    """
    height, width = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area <= area <= max_area_ratio * width * height:
            continue
        moments = cv2.moments(contour)
        if moments['m00'] <= 0:
            continue
        region = np.zeros((height, width), np.uint8)
        cv2.drawContours(region, [contour], -1, 255, -1)
        inner = cv2.erode(region, kernel)
        values = depth[(inner if inner.any() else region) != 0]
        valid = values[np.isfinite(values) & (values > 0)]
        if valid.size == 0 or valid.size < min_valid_ratio * values.size:
            continue
        if not min_depth <= float(np.median(valid)) <= max_depth:
            continue
        x = (moments['m10'] / moments['m00'] - width / 2) / (width / 2)
        y = (moments['m01'] / moments['m00'] - height / 2) / (height / 2)
        candidates.append((area, -(x * x + y * y), x, y, cv2.boundingRect(contour)))
    if not candidates:
        target = (0.0, 0.0, 0.0)
        return (target, None) if return_bbox else target
    area, _, x, y, bbox = max(candidates, key=lambda item: item[:4])
    target = (float(x), float(y), float(area / (width * height)))
    return (target, bbox) if return_bbox else target

