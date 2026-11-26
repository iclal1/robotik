#!/usr/bin/env python3
"""Sensor-only colored-cube perception. Never publishes velocity or TF."""
import json
import math
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (Detection2D, Detection2DArray, Detection3D,
                             Detection3DArray, ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray

from cube_geometry import (CUBE_SIZE_M, calibration_from_info,
                           estimate_cube, perception_state,
                           rotation_from_quaternion)
from cube_vision import detect_cube_candidates
from cube_visualization import (HOLD_SECONDS, MARKER_LIFETIME_SECONDS, marker_ids,
                                smooth_display_range, sudden_area_loss,
                                visual_state)


COLORS = {
    'red': ((0, 0, 255), (1.0, 0.1, 0.1)),
    'green': ((0, 255, 0), (0.1, 0.9, 0.1)),
    'blue': ((255, 0, 0), (0.1, 0.3, 1.0)),
}


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos)
        self.pub_2d = self.create_publisher(Detection2DArray, '/cube_detector/detections_2d', 2)
        self.pub_3d = self.create_publisher(Detection3DArray, '/cube_detector/detections_3d', 2)
        self.pub_image = self.create_publisher(Image, '/cube_detector/image_annotated', 2)
        self.pub_markers = self.create_publisher(MarkerArray, '/cube_detector/markers', 2)
        self.pub_status = self.create_publisher(String, '/cube_detector/status', 2)
        self.pub_clearance = self.create_publisher(String, '/cube_detector/clearance', 2)
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, self)
        self.info = None
        self.info_received = None
        self.last_image_received = None
        self.last_processed = 0.0
        self.last_stamp = None
        self.tracks = {}
        self.next_track_id = 0
        self.assigned_track_ids = set()
        self.latest = []
        self.received_frames = 0
        self.processed_frames = 0
        self.latency_ms = None
        self.last_error = None
        self.create_timer(0.2, self.publish_status)

    def on_info(self, message):
        self.info = message
        self.info_received = time.monotonic()

    def on_image(self, message):
        start = time.monotonic()
        self.last_image_received = start
        self.received_frames += 1
        if start - self.last_processed < 0.064:
            return
        self.last_processed = start
        self.processed_frames += 1
        self.last_stamp = message.header.stamp
        self.latest = []
        self.assigned_track_ids.clear()
        error = None
        calibration = calibration_from_info(self.info) if self.info is not None else None
        if calibration is None or self.info_received is None or start - self.info_received > 1.0:
            error = 'invalid_or_stale_camera_info'
        elif (message.width != calibration['width'] or
              message.height != calibration['height'] or
              message.header.frame_id != self.info.header.frame_id):
            error = 'camera_info_image_mismatch'
        elif message.encoding not in ('rgb8', 'bgr8') or message.step < message.width * 3:
            error = 'unsupported_image_format'
        elif len(message.data) < message.step * message.height:
            error = 'truncated_image'
        transform = None
        rotation = None
        translation = None
        if error is None:
            try:
                transform = self.buffer.lookup_transform(
                    'base_footprint', message.header.frame_id,
                    Time.from_msg(message.header.stamp))
                rotation = rotation_from_quaternion(transform.transform.rotation)
                translation = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z], dtype=float)
                if rotation is None or not np.all(np.isfinite(translation)):
                    error = 'invalid_camera_transform'
            except Exception:
                error = 'camera_transform_unavailable_at_stamp'
        image = None
        preview = None
        if error is None:
            pixels = np.frombuffer(message.data, dtype=np.uint8)
            pixels = pixels.reshape((message.height, message.step))[:, :message.width * 3]
            image = pixels.reshape((message.height, message.width, 3))
            if message.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            preview = cv2.resize(image, (message.width // 2, message.height // 2),
                                 interpolation=cv2.INTER_AREA)
            candidates = detect_cube_candidates(
                image, calibration['k'][0, 0], calibration['k'][0, 2])
            odom_rotation = odom_translation = None
            try:
                odom_tf = self.buffer.lookup_transform(
                    'odom', 'base_footprint', Time.from_msg(message.header.stamp))
            except Exception:
                # Camera frames can arrive a few milliseconds ahead of odom.
                # A recent odom transform is sufficient for short track
                # association, but never use an old pose while moving.
                try:
                    odom_tf = self.buffer.lookup_transform(
                        'odom', 'base_footprint', Time())
                    image_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                    odom_ns = odom_tf.header.stamp.sec * 10**9 + odom_tf.header.stamp.nanosec
                    if abs(image_ns - odom_ns) > 50_000_000:
                        odom_tf = None
                except Exception:
                    odom_tf = None
            if odom_tf is not None:
                odom_rotation = rotation_from_quaternion(odom_tf.transform.rotation)
                odom_translation = np.array([
                    odom_tf.transform.translation.x,
                    odom_tf.transform.translation.y,
                    odom_tf.transform.translation.z])
            for candidate in candidates:
                estimate = estimate_cube(candidate, calibration, rotation, translation)
                if estimate.get('reason') == 'flat_target_or_bad_geometry':
                    continue
                estimate['distance_valid'] = estimate['valid']
                item = self.update_track(
                    candidate, estimate, start, odom_rotation, odom_translation)
                if candidate.get('requires_track') and 'track_id' not in item:
                    continue
                self.latest.append(item)
        self.last_error = error
        self.publish_detections(message, preview, error)
        self.latency_ms = round((time.monotonic() - start) * 1000, 2)
        self.publish_status(error)

    def update_track(self, candidate, estimate, now,
                     odom_rotation=None, odom_translation=None):
        color = candidate['color']
        if estimate['valid']:
            position = np.array(estimate['center_position_base_m'])
            odom_position = (odom_rotation @ np.array([*position, 0.0]) +
                             odom_translation)[:2] if odom_rotation is not None else None
            matches = [(key, track) for key, track in self.tracks.items()
                       if track['color'] == color and now - track['seen'] < HOLD_SECONDS and
                       key not in self.assigned_track_ids and
                       odom_position is not None and track['position'] is not None and
                       np.linalg.norm(odom_position - track['position']) <
                       max(0.20, 2 * estimate['position_uncertainty_m'])]
            if matches:
                key, track = min(matches, key=lambda pair:
                                 np.linalg.norm(odom_position - pair[1]['position']))
                if sudden_area_loss(track.get('area_px'), candidate['area_px'],
                                    now - track['seen']):
                    self.assigned_track_ids.add(key)
                    return {'color': color, 'bbox': candidate['bbox'],
                            'area_px': candidate['area_px'],
                            'partial_view': True, 'valid': False,
                            'distance_valid': False,
                            'reason': 'possible_partial_occlusion',
                            'confirmed': False, 'tracking_state': 'uncertain',
                            'confidence': round(min(.75,
                                estimate['geometry_confidence']), 3),
                            'track_id': key}
                hits = min(20, track['hits'] + 1)
                display_range = smooth_display_range(
                    track.get('display_range'), float(np.linalg.norm(position)),
                    track['seen'], now)
                odom_position = .7 * odom_position + .3 * track['position']
                position = (odom_rotation.T @
                            (np.array([*odom_position, 0.0]) - odom_translation))[:2]
            else:
                key = self.next_track_id
                self.next_track_id += 1
                hits = 1
                display_range = float(np.linalg.norm(position))
                track = {}
            self.assigned_track_ids.add(key)
            self.tracks[key] = {
                'id': key, 'color': color, 'position': odom_position,
                'hits': hits, 'seen': now, 'stamp': self.last_stamp,
                'uncertainty': estimate['position_uncertainty_m'],
                'display_range': display_range,
                'bbox': candidate['bbox'],
                'area_px': candidate['area_px'],
                'marker_visible': track.get('marker_visible', False) or
                                  (hits >= 3 and odom_position is not None),
                'confidence': estimate['geometry_confidence'],
            }
            estimate['center_position_base_m'] = position.tolist()
            estimate['center_range_m'] = float(np.linalg.norm(position))
            estimate['bearing_base_rad'] = math.atan2(position[1], position[0])
            estimate['confirmed'] = hits >= 3
            estimate['confidence'] = round(
                estimate['geometry_confidence'] * (0.75 if hits == 1 else
                                                  0.9 if hits == 2 else 1.0), 3)
            estimate['track_id'] = key
            estimate['display_range_m'] = display_range
        else:
            estimate['confirmed'] = False
            estimate['confidence'] = round(
                estimate.get('appearance_confidence', 0.0), 3)
            # An incomplete silhouette can explain a missed 3D measurement.
            # Link it to a recent visual track without refreshing its pose.
            x, y, w, h = candidate['bbox']
            possible = []
            for key, track in self.tracks.items():
                if (key in self.assigned_track_ids or track['color'] != color or
                        now - track['seen'] > HOLD_SECONDS or
                        not track.get('marker_visible')):
                    continue
                tx, ty, tw, th = track['bbox']
                separation = math.hypot(x + w/2 - tx - tw/2,
                                        y + h/2 - ty - th/2)
                if separation < max(80, .8*max(w, h, tw, th)):
                    possible.append((separation, key))
            if possible:
                key = min(possible)[1]
                self.assigned_track_ids.add(key)
                estimate['track_id'] = key
                estimate['tracking_state'] = 'uncertain'
        return {'color': color, 'bbox': candidate['bbox'],
                'area_px': candidate['area_px'],
                'partial_view': candidate['partial_view'], **estimate}

    def publish_detections(self, source, preview, error):
        detections_2d = Detection2DArray()
        detections_2d.header = source.header
        detections_3d = Detection3DArray()
        detections_3d.header.stamp = source.header.stamp
        detections_3d.header.frame_id = 'base_footprint'
        annotated = preview.copy() if preview is not None else None
        for index, item in enumerate(self.latest):
            x, y, width, height = item['bbox']
            detection = Detection2D()
            detection.header = source.header
            detection.id = (f"{item['color']}_{item['track_id']}"
                            if 'track_id' in item else item['color'])
            detection.bbox.center.position.x = x + width / 2
            detection.bbox.center.position.y = y + height / 2
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = item['color'] + '_cube'
            hypothesis.hypothesis.score = item['confidence']
            detection.results.append(hypothesis)
            detections_2d.detections.append(detection)
            if item['valid'] and item['confirmed']:
                detection_3d = Detection3D()
                detection_3d.header = detections_3d.header
                detection_3d.id = detection.id
                pose = detection_3d.bbox.center
                pose.position.x, pose.position.y = item['center_position_base_m']
                pose.position.z = CUBE_SIZE_M / 2
                pose.orientation.w = 1.0
                detection_3d.bbox.size.x = CUBE_SIZE_M
                detection_3d.bbox.size.y = CUBE_SIZE_M
                detection_3d.bbox.size.z = CUBE_SIZE_M
                hypothesis_3d = ObjectHypothesisWithPose()
                hypothesis_3d.hypothesis.class_id = hypothesis.hypothesis.class_id
                hypothesis_3d.hypothesis.score = item['confidence']
                hypothesis_3d.pose.pose = pose
                variance = item['position_uncertainty_m'] ** 2
                hypothesis_3d.pose.covariance[0] = variance
                hypothesis_3d.pose.covariance[7] = variance
                hypothesis_3d.pose.covariance[14] = 0.01 ** 2
                detection_3d.results.append(hypothesis_3d)
                detections_3d.detections.append(detection_3d)
            if annotated is not None:
                bgr, _ = COLORS[item['color']]
                px, py, pw, ph = (int(round(v / 2)) for v in (x, y, width, height))
                cv2.rectangle(annotated, (px, py), (px + pw, py + ph), bgr, 2)
                label = item['color'] + ' ' + (
                    f"{item['center_range_m']:.2f}±{item['position_uncertainty_m']:.2f}m "
                    f"score {item['confidence']:.2f}"
                    if item['valid'] else
                    (f"~{item['tentative_center_range_m']:.1f}m uncertain"
                     if 'tentative_center_range_m' in item else item['reason']))
                line_y = 17 + 17 * index
                cv2.rectangle(annotated, (2, line_y - 13), (260, line_y + 3),
                              (32, 32, 32), -1)
                cv2.rectangle(annotated, (5, line_y - 10), (12, line_y - 3),
                              bgr, -1)
                cv2.putText(annotated, label, (18, line_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
        self.pub_2d.publish(detections_2d)
        self.pub_3d.publish(detections_3d)
        self.pub_markers.publish(self.make_markers(source))
        if annotated is not None:
            bottom = annotated.shape[0]
            cv2.rectangle(annotated, (0, bottom - 18),
                          (annotated.shape[1], bottom), (32, 32, 32), -1)
            cv2.putText(annotated, 'CLEARANCE UNKNOWN - MOTION BLOCKED',
                        (5, bottom - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.34, (80, 180, 255), 1)
            output = Image()
            output.header = source.header
            output.height, output.width = annotated.shape[:2]
            output.encoding = 'bgr8'
            output.is_bigendian = False
            output.step = output.width * 3
            output.data = annotated.tobytes()
            self.pub_image.publish(output)

    def make_markers(self, source):
        """Update stable odom-frame markers; expire them explicitly once."""
        result = MarkerArray()
        now = time.monotonic()
        fresh = {item['track_id'] for item in self.latest
                 if item['valid'] and item['confirmed'] and 'track_id' in item}
        uncertain = {item['track_id'] for item in self.latest
                     if not item['valid'] and 'track_id' in item}
        for key, track in self.tracks.items():
            state = visual_state(track, now, fresh, uncertain)
            if state == 'hidden':
                continue
            cube_id, label_id = marker_ids(key)
            if state == 'expired':
                for marker_id in (cube_id, label_id):
                    marker = Marker()
                    marker.header.frame_id = 'odom'
                    marker.header.stamp = source.header.stamp
                    marker.ns = 'estimated_cubes'
                    marker.id = marker_id
                    marker.action = Marker.DELETE
                    result.markers.append(marker)
                track['marker_visible'] = False
                continue
            for marker_id, marker_type in ((cube_id, Marker.CUBE),
                                           (label_id, Marker.TEXT_VIEW_FACING)):
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.header.stamp = (source.header.stamp if state == 'confirmed'
                                       else track['stamp'])
                marker.ns = 'estimated_cubes'
                marker.id = marker_id
                marker.type = marker_type
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y = track['position']
                if marker_type == Marker.TEXT_VIEW_FACING:
                    marker.pose.position.y += .20
                marker.pose.position.z = (CUBE_SIZE_M / 2 if marker_type == Marker.CUBE
                                          else .34)
                marker.pose.orientation.w = 1.0
                marker.scale.x = CUBE_SIZE_M if marker_type == Marker.CUBE else 0.0
                marker.scale.y = CUBE_SIZE_M if marker_type == Marker.CUBE else 0.0
                marker.scale.z = CUBE_SIZE_M if marker_type == Marker.CUBE else .17
                _, rgb = COLORS[track['color']]
                if marker_type == Marker.CUBE:
                    marker.color.r, marker.color.g, marker.color.b = rgb
                else:
                    label_colors = {'red': (.95, .45, .42),
                                    'green': (.28, .9, .42),
                                    'blue': (.45, .65, 1.0)}
                    marker.color.r, marker.color.g, marker.color.b = label_colors[track['color']]
                alpha = (0.65 if marker_type == Marker.CUBE else 1.0)
                marker.color.a = alpha * (1.0 if state == 'confirmed' else
                                          .55 if state == 'uncertain' else .35)
                label = (f"{track['color'][0].upper()}:{track['display_range']:.2f}"
                         f"±{track['uncertainty']:.2f}m|s{track['confidence']:.2f}")
                if state == 'uncertain':
                    label += '|PARTIAL'
                elif state == 'stale':
                    label += f"|LAST{now - track['seen']:.1f}s"
                else:
                    label += '|LIVE'
                marker.text = label if marker_type == Marker.TEXT_VIEW_FACING else ''
                marker.lifetime = Duration(seconds=MARKER_LIFETIME_SECONDS).to_msg()
                result.markers.append(marker)
        return result

    def publish_status(self, error=None):
        if error is None:
            error = self.last_error
        now = time.monotonic()
        stale = []
        for key, track in list(self.tracks.items()):
            age = now - track['seen']
            if age > 3.0:
                del self.tracks[key]
            elif age > 0.3:
                stale.append({'color': track['color'], 'age_s': round(age, 2),
                              'position_uncertainty_m': round(
                                  track['uncertainty'] + 0.10 * age, 3)})
        camera_age = None if self.last_image_received is None else now - self.last_image_received
        calibration_age = None if self.info_received is None else now - self.info_received
        state, reason = perception_state(camera_age, calibration_age,
                                         error, self.latest)
        valid = [item for item in self.latest if item['valid']] if state in (
            'confirmed_visible_obstacle', 'estimated_visible_obstacle') else []
        clearance = {
            'state': state, 'reason': reason, 'safe_to_move': False,
            'unknown_elsewhere': True,
            'visible_obstacle_clearance_lower_bound_m':
                min((item['clearance_lower_bound_m'] for item in valid), default=None),
            'stale_obstacles': stale,
            'camera_age_s': None if camera_age is None else round(camera_age, 3),
        }
        self.pub_clearance.publish(String(data=json.dumps(clearance)))
        status = {
            'stamp': None if self.last_stamp is None else
                {'sec': self.last_stamp.sec, 'nanosec': self.last_stamp.nanosec},
            'frame_id': 'base_footprint',
            'camera_frame': None if self.info is None else self.info.header.frame_id,
            'perception_state': state, 'reason': reason,
            'detections': self.latest, 'stale_tracks': stale,
            'received_frames': self.received_frames,
            'processed_frames': self.processed_frames,
            'processing_latency_ms': self.latency_ms,
        }
        self.pub_status.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CubeDetector()
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
