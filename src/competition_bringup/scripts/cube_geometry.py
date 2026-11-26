"""Calibrated monocular geometry for grounded 0.18 m competition cubes."""
import math

import cv2
import numpy as np


CUBE_SIZE_M = 0.18
CUBE_RADIUS_M = math.sqrt(2.0) * CUBE_SIZE_M / 2.0
ROBOT_RADIUS_M = math.hypot(0.2375, 0.12)  # Pusher front corner.
SAFETY_MARGIN_M = 0.10


def perception_state(camera_age, calibration_age, error, detections):
    """Classify observation health; no observation can certify unseen space."""
    if camera_age is None or camera_age > 0.5:
        return 'invalid_perception', 'camera_frames_stale'
    if calibration_age is None or calibration_age > 1.0:
        return 'invalid_perception', 'camera_info_stale'
    if error:
        return 'invalid_perception', error
    if any(item['valid'] and item['confirmed'] for item in detections):
        return 'confirmed_visible_obstacle', 'visible_cube_candidate'
    if any(item['valid'] for item in detections):
        return 'estimated_visible_obstacle', 'awaiting_temporal_confirmation'
    if detections:
        return ('visible_obstacle_distance_uncertain',
                'visible_candidate_without_valid_3d_position')
    return 'unknown_or_unobserved', 'no_valid_visible_cube'


def calibration_from_info(info):
    """Validate a CameraInfo message without substituting guessed intrinsics."""
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    if (info.width <= 0 or info.height <= 0 or
            not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or
            k[0, 0] <= 0 or k[1, 1] <= 0 or
            not 0 <= k[0, 2] < info.width or
            not 0 <= k[1, 2] < info.height or
            info.distortion_model not in ('plumb_bob', '')):
        return None
    return {'k': k, 'd': d, 'width': info.width, 'height': info.height}


def rotation_from_quaternion(quaternion):
    q = np.array([quaternion.x, quaternion.y, quaternion.z,
                  quaternion.w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        return None
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def plane_intersection(u, v, plane_z, calibration, rotation, translation):
    """Intersect an optical pixel ray with a horizontal robot-frame plane."""
    k, d = calibration['k'], calibration['d']
    if d.size and np.any(np.abs(d) > 1e-12):
        ray_xy = cv2.undistortPoints(
            np.array([[[u, v]]], dtype=np.float64), k, d)[0, 0]
        ray = np.array([ray_xy[0], ray_xy[1], 1.0])
    else:
        ray = np.array([(u - k[0, 2]) / k[0, 0],
                        (v - k[1, 2]) / k[1, 1], 1.0])
    direction = rotation @ ray
    if not np.all(np.isfinite(direction)) or direction[2] >= -1e-5:
        return None
    scale = (plane_z - translation[2]) / direction[2]
    if scale <= 0 or not math.isfinite(scale):
        return None
    return translation + scale * direction


def _pitch_rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def project_point(point, calibration, rotation, translation):
    optical = rotation.T @ (point - translation)
    if optical[2] <= 0:
        return None
    pixels, _ = cv2.projectPoints(
        optical.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
        calibration['k'], calibration['d'])
    return pixels[0, 0]


def estimate_cube(candidate, calibration, rotation, translation):
    """Estimate front contact and cube center; return a reason if invalid.

    The lower silhouette is treated as the visible front-ground edge. A cube
    center is offset 0.09 m inward along the viewing ray; its orientation is
    unknown, so this center carries extra uncertainty. Surface clearance uses
    the nearer contact point and a conservative lower range bound.
    """
    x, y, width, height = candidate['bbox']
    side_clipped = x <= 1 or x + width >= calibration['width'] - 1
    vertical_clipped = y <= 1 or y + height >= calibration['height'] - 1
    # A side crop can retain both the top and ground-contact edges. Accept
    # only a substantial, solid silhouette; its lateral center is less certain.
    usable_side_crop = (side_clipped and not vertical_clipped and
                        width >= 20 and height >= 24 and
                        candidate.get('area_px', 0) >= 900 and
                        candidate.get('fill_ratio', 0) >= .53 and
                        candidate.get('solidity', 0) >= .83)
    if (vertical_clipped or (candidate.get('partial_view') and
                             not usable_side_crop) or
            (side_clipped and not usable_side_crop)):
        # Color and visible silhouette can still support a 2D observation;
        # the missing outline cannot support a grounded 3D contact point.
        appearance = min(.75, .45 * candidate.get('fill_ratio', .6) +
                         .30 * candidate.get('solidity', .8))
        result = {'valid': False, 'reason': 'contact_or_outline_clipped',
                  'appearance_confidence': appearance}
        if (side_clipped and not vertical_clipped and height >= 24 and
                candidate.get('area_px',0) >= 200 and
                candidate.get('fill_ratio',0) >= .5 and
                candidate.get('solidity',0) >= .8):
            # The side outline is incomplete, so no 3D center is asserted.
            # A visible bottom edge still gives a conservative *near-field*
            # lower bound. Missing or suspect bottom edges retain no bound.
            possible_ranges = []
            for u in (x, x+width):
                for v in (y+height-4,y+height-1,y+height+2):
                    point = plane_intersection(u,v,0.,calibration,
                                               rotation,translation)
                    if point is not None:
                        possible_ranges.append(float(np.linalg.norm(point[:2])))
            if len(possible_ranges) == 6:
                result['tentative_center_range_bounds_m'] = [
                    max(0.,min(possible_ranges)-.14),
                    max(possible_ranges)+CUBE_SIZE_M+.14]
        return result
    u, bottom_v = candidate.get('contact_pixel',
                                [x + width / 2.0, y + height - 1.0])
    front = plane_intersection(u, bottom_v, 0.0, calibration,
                               rotation, translation)
    top = plane_intersection(u, y + 1.0, CUBE_SIZE_M, calibration,
                             rotation, translation)
    if front is None or top is None:
        return {'valid': False, 'reason': 'ground_ray_unavailable'}
    predicted_top = project_point(
        front + np.array([0.0, 0.0, CUBE_SIZE_M]),
        calibration, rotation, translation)
    if predicted_top is None:
        return {'valid': False, 'reason': 'top_reprojection_unavailable'}
    top_pixel_residual = abs(float(predicted_top[1]) - y)
    if top_pixel_residual > max(12.0, 0.15 * height):
        return {'valid': False, 'reason': 'flat_target_or_bad_geometry',
                'top_pixel_residual': top_pixel_residual}
    vertical_residual = float(np.linalg.norm(front[:2] - top[:2]))
    from_camera = front[:2] - translation[:2]
    from_camera_norm = np.linalg.norm(from_camera)
    if from_camera_norm < 1e-6:
        return {'valid': False, 'reason': 'contact_too_close'}
    center = front[:2] + 0.5 * CUBE_SIZE_M * from_camera / from_camera_norm

    # Local sensitivity envelope for segmentation, mounting height and pitch.
    # The TF calibration is used directly; this is a local sensitivity scale
    # plus a 3 cm shape/contact model term, not calibrated covariance or a
    # safety bound.
    ranges = []
    centers = []
    for dv in (-1.5, 1.5):
        for dz in (-0.005, 0.005):
            for pitch in (-0.005, 0.005):
                perturbed = translation.copy()
                perturbed[2] += dz
                point = plane_intersection(
                    u, bottom_v + dv, 0.0, calibration,
                    _pitch_rotation(pitch) @ rotation, perturbed)
                if point is not None:
                    ranges.append(float(np.linalg.norm(point[:2])))
                    centers.append(point[:2])
    if len(ranges) < 8:
        return {'valid': False, 'reason': 'uncertainty_unbounded'}
    surface_range = float(np.linalg.norm(front[:2]))
    center_range = float(np.linalg.norm(center))
    crop_uncertainty = (.12 if width < 24 else .08) if side_clipped else 0.
    position_uncertainty = max(
        float(np.linalg.norm(p - front[:2])) for p in centers) + 0.03 + crop_uncertainty
    # Clearance retains a wider, conservative interval than the reported
    # localization standard uncertainty.
    range_lower = max(0.0, min(ranges) - 0.06 - crop_uncertainty)
    range_upper = max(ranges) + 0.06 + crop_uncertainty
    if position_uncertainty > 0.30 or range_upper > 5.0:
        shape_score = min(1.0, candidate['area_px'] / (width * height))
        geometry_score = max(0.0, 1.0 - top_pixel_residual / max(12.0, 0.15*height))
        return {'valid': False, 'reason': 'range_uncertainty_too_large',
                'vertical_residual_m': vertical_residual,
                'tentative_center_range_m': center_range,
                'tentative_center_range_bounds_m':
                    [max(0.0, range_lower), range_upper + CUBE_SIZE_M],
                'bearing_base_rad': math.atan2(center[1], center[0]),
                'position_uncertainty_m': position_uncertainty,
                'appearance_confidence': min(1.0,
                    0.4 * shape_score + 0.2 * candidate.get('solidity', 1.0)
                    + 0.4 * geometry_score)}
    shape_score = min(1.0, candidate['area_px'] / (width * height))
    geometry_score = max(0.0, 1.0 - top_pixel_residual / max(12.0, 0.15*height))
    confidence = min(1.0, 0.4 * shape_score + 0.2 *
                     candidate.get('solidity', 1.0) + 0.4 * geometry_score)
    clearance_lower = max(
        0.0, range_lower - CUBE_RADIUS_M - ROBOT_RADIUS_M - SAFETY_MARGIN_M)
    return {
        'valid': True,
        'surface_position_base_m': [float(front[0]), float(front[1])],
        'center_position_base_m': [float(center[0]), float(center[1])],
        'surface_range_m': surface_range,
        'surface_range_bounds_m': [range_lower, range_upper],
        'center_range_m': center_range,
        'bearing_base_rad': math.atan2(center[1], center[0]),
        'position_uncertainty_m': position_uncertainty,
        'clearance_lower_bound_m': clearance_lower,
        'vertical_residual_m': vertical_residual,
        'top_pixel_residual': top_pixel_residual,
        'geometry_confidence': confidence,
    }
