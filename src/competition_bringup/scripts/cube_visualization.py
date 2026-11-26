"""Small, testable policies for cube marker lifetime and displayed range.

These policies are visual only. Safety and 3D detections use fresh camera
measurements from the detector, never predicted markers.
"""

HOLD_SECONDS = 1.2
MARKER_LIFETIME_SECONDS = 1.8


def marker_ids(track_id):
    """Keep both marker IDs fixed throughout a track's life."""
    return 2 * track_id, 2 * track_id + 1


def visual_state(track, now, fresh_ids, uncertain_ids):
    """Return fresh, uncertain, stale, or expired for an existing marker."""
    if not track.get('marker_visible') or track.get('position') is None:
        return 'hidden'
    age = now - track['seen']
    if age < 0 or age > HOLD_SECONDS:
        return 'expired'
    if track['id'] in fresh_ids:
        return 'confirmed'
    if track['id'] in uncertain_ids:
        return 'uncertain'
    return 'stale'


def smooth_display_range(previous, measured, previous_time, now):
    """A short adaptive EMA; large real changes pass through promptly."""
    if previous is None or previous_time is None or now - previous_time > .5:
        return measured
    delta = abs(measured - previous)
    weight = .75 if delta > .15 else .4
    return weight * measured + (1 - weight) * previous


def sudden_area_loss(previous_area, current_area, elapsed):
    """Flag an implausibly large silhouette loss between adjacent frames."""
    return (previous_area is not None and previous_area > 0 and
            0 <= elapsed < .2 and current_area < .55 * previous_area)
