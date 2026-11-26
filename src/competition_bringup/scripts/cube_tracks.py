"""Odom-frame navigation tracks; observations never certify unseen free space."""
import math


COLORS = ('red', 'green', 'blue')
GROWTH_MPS = .02
MODEL_MARGIN_M = .06
FRESH_S = 1.2
SIDE_CROP_CENTER_ALLOWANCE_M = .18  # Entire physical cube width.
LOST_S = 30.0


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def track_state(age, confirmed, partial, hidden, inconsistent):
    if inconsistent or age >= LOST_S:
        return 'lost'
    if age > FRESH_S:
        return 'stale'
    if partial or not confirmed:
        return 'uncertain'
    return 'recently_observed' if hidden else 'visible'


class CubeTracks:
    def __init__(self, growth_mps=GROWTH_MPS):
        if not math.isfinite(growth_mps) or growth_mps <= 0:
            raise ValueError('Cube uncertainty must grow while unobserved')
        self.growth_mps = growth_mps
        self.tracks = {}
        self.reacquisitions = 0
        self.rejections = 0
        self.interruptions = 0
        self._hidden = set()

    def observe(self, color, xy, uncertainty, stamp, now, track_id=None,
                confirmed=True, partial=False):
        if (color not in COLORS or not math.isfinite(uncertainty) or
                uncertainty < 0 or not all(map(math.isfinite, xy))):
            self.rejections += 1
            return False
        source_time = stamp_seconds(stamp)
        if source_time <= 0 or not math.isfinite(source_time):
            self.rejections += 1
            return False
        old = self.tracks.get(color)
        if old is not None:
            if old['inconsistent']:
                self.rejections += 1
                return False
            dt = source_time - old['source_time']
            if dt < -1e-3:
                self.rejections += 1
                return False
            if abs(dt) <= 1e-3:
                if confirmed and not old['confirmed']:
                    old['confirmed'] = True
                return True
            gate = max(.25, old['uncertainty'] + uncertainty + self.growth_mps*dt + .05)
            if math.dist(old['odom_xy'], xy) > gate:
                old['inconsistent'] = True
                self.rejections += 1
                return False
            if color in self._hidden:
                self.reacquisitions += 1
                self._hidden.remove(color)
            residual = math.dist(old['odom_xy'], xy)
            measured_xy = xy
            weight = .10 if partial else .75
            xy = (weight*xy[0] + (1-weight)*old['odom_xy'][0],
                  weight*xy[1] + (1-weight)*old['odom_xy'][1])
            if partial:
                # A valid side crop measures the ground contact and top edge,
                # but its lateral center can be biased. Intersect the prior
                # propagated disk with a conservative measurement disk. The
                # full cube width covers the missing side; the measured local
                # sensitivity alone is not a certified safety bound.
                propagated = old['uncertainty'] + self.growth_mps*dt
                measured = (math.dist(xy, measured_xy) + uncertainty +
                            SIDE_CROP_CENTER_ALLOWANCE_M + MODEL_MARGIN_M)
                uncertainty = min(propagated, measured) - MODEL_MARGIN_M
        self.tracks[color] = {
            'odom_xy': tuple(xy), 'uncertainty': uncertainty + MODEL_MARGIN_M,
            'source_time': source_time, 'seen': now, 'track_id': track_id,
            'confirmed': confirmed, 'inconsistent': False,
            'partial': partial,
        }
        return True

    def note_visibility(self, visible_colors):
        for color in self.tracks:
            if color not in visible_colors and color not in self._hidden:
                self._hidden.add(color)
                self.interruptions += 1

    def age(self, color, now):
        track = self.tracks.get(color)
        return math.inf if track is None else max(0., now-track['seen'])

    def fresh(self, color, now):
        track = self.tracks.get(color)
        return (track is not None and not track['inconsistent'] and
                track['confirmed'] and not track['partial'] and
                self.age(color, now) <= FRESH_S)

    def statuses(self, now):
        """Keep age and observation quality distinct from obstacle persistence."""
        result = {}
        for color, track in self.tracks.items():
            age = self.age(color, now)
            state = track_state(age, track['confirmed'], track['partial'],
                                color in self._hidden, track['inconsistent'])
            result[color] = {'state': state, 'age_s': age,
                             'uncertainty_m': track['uncertainty'] +
                                              self.growth_mps*age,
                             'source_time': track['source_time']}
        return result

    def obstacles(self, now):
        for track in self.tracks.values():
            age = max(0., now-track['seen'])
            # The exclusion grows while hidden. Old tracks are obstacles,
            # never evidence that the rest of the arena is free.
            # make_grid also has an age term; growth is handled here once.
            yield track['odom_xy'], track['uncertainty'] + self.growth_mps*age, 0.
