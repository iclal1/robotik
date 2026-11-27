#!/usr/bin/env python3
"""Task node: deliver the three cubes to their goal zones (Phase 6/11).

Each cube is moved by a fixed sequence of straight pushes (segments). Per
segment the node
  1) reaches an approach pose behind the cube: Nav2 for long legs, its own
     short straight moves where Nav2 would have to squeeze between cube and
     wall (all such moves were clearance-checked offline with the real
     footprint: >= 7.5 cm),
  2) measures the cube with the camera when it is visible (first segment),
  3) pushes along the line with cross-track control, then backs off.

Command arbitration: while a Nav2 goal is active the node forwards
/nav2_cmd_vel to /cmd_vel; during its own moves it publishes /cmd_vel
itself. cmd_vel_guard stays the last stage (limits, timeout, /emergency_stop).
/emergency_stop is also honoured here: the run is aborted and goals cancelled.

Localization: TF map->base_footprint (AMCL). The saved map frame starts at the
robot spawn, so arena = map + START. No Gazebo ground truth is used.
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

START = (-2.40, -1.40)
HALF = 0.09                  # cube half size
FRONT = 0.2375               # base centre -> pusher face
CONTACT = HALF + FRONT       # robot centre - cube centre at contact
MAX_V, MAX_W = 0.30, 1.0     # node-side command limits

# colour: (camera/class name, zone centre,
#          [(cube start, cube end, approach distance, nav2 waypoints, mode), ...])
# mode: False -> Nav2 to the approach pose
#       'ara' -> Nav2 through the waypoints only (last = open staging pose),
#                then a short own move to the approach pose
#       True  -> own move to the approach pose
#       list  -> own moves through these points, then to the approach pose
ROUTES = {
    'green': ('green', (2.35, 0.0), [
        ((-1.60, 0.00), (2.35, -0.10), 0.70, [], False)]),
    'red': ('red', (2.35, 1.2), [
        ((-1.60, 1.20), (-0.40, -0.05), 0.55, [(-2.30, 0.55, 90), (-2.35, 1.40, 90)], 'ara'),
        ((-0.40, -0.05), (1.60, -0.10), 0.60, [], True),
        ((1.60, -0.10), (2.35, 1.20), 0.55, [], [(1.10, -0.10)])]),
    'blue': ('blue', (2.35, -1.2), [
        ((-1.60, -1.20), (-0.45, -0.05), 0.55, [(-2.30, -0.55, -90), (-2.35, -1.40, -90)], 'ara'),
        ((-0.45, -0.05), (1.60, -0.40), 0.60, [], True),
        ((1.60, -0.40), (2.35, -1.20), 0.50, [], False)]),
}
ALIASES = {'yesil': 'green', 'kirmizi': 'red', 'mavi': 'blue'}
TR = {'green': 'yesil', 'red': 'kirmizi', 'blue': 'mavi'}


class Aborted(Exception):
    pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CubePusher(Node):
    def __init__(self):
        super().__init__('cube_pusher')
        self.declare_parameter('order', 'green,red,blue')
        self.declare_parameter('start_delay_sec', 5.0)
        self.order = [ALIASES.get(c.strip(), c.strip())
                      for c in str(self.get_parameter('order').value).split(',') if c.strip()]
        bad = [c for c in self.order if c not in ROUTES]
        if bad:
            raise ValueError('order icinde bilinmeyen renk: %s' % bad)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navp = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/cube_pusher/status', 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(Detection3DArray, '/cube_detector/detections_3d', self.on_det, 5)
        self.create_subscription(Twist, '/nav2_cmd_vel', self.on_nav_cmd, 10)
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.color = None
        self.det = None
        self.det_time = 0.0
        self.nav_active = False
        self.goal_handle = None
        self.estop = False
        self.sim = 0.0
        self.times = []          # (colour, segment, step, wall seconds)
        self.label = ''

    # ---------- callbacks ----------
    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_det(self, msg):
        for d in msg.detections:
            if self.color and d.results and d.results[0].hypothesis.class_id == self.color + '_cube':
                p = d.bbox.center.position
                self.det = (p.x, p.y)
                self.det_time = time.time()

    def on_nav_cmd(self, msg):
        if self.nav_active and not self.estop:
            self.send(msg.linear.x, msg.angular.z)

    def on_estop(self, msg):
        if msg.data and not self.estop:
            self.estop = True
            self.log('ACIL DURDURMA geldi, gorev durduruluyor')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.cmd.publish(Twist())

    # ---------- helpers ----------
    def log(self, text):
        self.get_logger().info(text)

    def status(self, text):
        self.status_pub.publish(String(data=text))

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        if self.estop:
            raise Aborted()

    def send(self, v, w):
        t = Twist()
        t.linear.x = float(max(-MAX_V, min(MAX_V, v)))
        t.angular.z = float(max(-MAX_W, min(MAX_W, w)))
        self.cmd.publish(t)

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x + START[0], t.transform.translation.y + START[1],
                2 * math.atan2(q.z, q.w))

    def wait_pose(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.tick()
            p = self.pose()
            if p:
                return p
        raise RuntimeError('TF map->base_footprint yok (AMCL calisiyor mu?)')

    def step(self, seg, name, fn, *a, **k):
        t0 = time.time()
        r = fn(*a, **k)
        dt = time.time() - t0
        self.times.append((self.label, seg, name, dt))
        self.log('      [%5.1f sn] %s' % (dt, name))
        return r

    # ---------- Nav2 ----------
    def pose_msg(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x - START[0]
        ps.pose.position.y = y - START[1]
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        return ps

    def run_goal(self, client, goal, timeout):
        f = client.send_goal_async(goal)
        t0 = time.time()
        while not f.done() and time.time() - t0 < 30:
            self.tick()
        h = f.result() if f.done() else None
        if not h or not h.accepted:
            return False
        self.goal_handle = h
        self.nav_active = True
        try:
            r = h.get_result_async()
            t0 = time.time()
            while not r.done() and time.time() - t0 < timeout:
                self.tick()
            if not r.done():
                h.cancel_goal_async()
                return False
            return r.result().status == GoalStatus.STATUS_SUCCEEDED
        finally:
            self.nav_active = False
            self.goal_handle = None
            self.stop()

    def nav2_go(self, points, timeout=240):
        """One Nav2 goal through all points (no stop at intermediate ones)."""
        if len(points) > 1 and self.navp.wait_for_server(timeout_sec=2):
            g = NavigateThroughPoses.Goal()
            g.poses = [self.pose_msg(*p) for p in points]
            if self.run_goal(self.navp, g, timeout):
                return True
            self.log('      navigate_through_poses basarisiz, noktalara tek tek gidiliyor')
        ok = True
        for p in points:
            g = NavigateToPose.Goal()
            g.pose = self.pose_msg(*p)
            ok = self.run_goal(self.nav, g, timeout)
        return ok

    # ---------- own motions ----------
    def turn(self, heading, tol=0.025):
        t0 = time.time()
        while time.time() - t0 < 20:
            self.tick()
            p = self.pose()
            if not p:
                continue
            e = wrap(heading - p[2])
            if abs(e) < tol:
                break
            w = max(-0.6, min(0.6, 1.5 * e))
            if abs(w) < 0.12:
                w = math.copysign(0.12, w)
            self.send(0, w)
        self.stop()

    def go_to(self, hx, hy, tol=0.02):
        """Short move: turn towards the point (or away to reverse), drive straight."""
        p = self.wait_pose()
        if math.hypot(hx - p[0], hy - p[1]) < tol:
            return
        rev = False
        yaw = math.atan2(hy - p[1], hx - p[0])
        if abs(wrap(yaw - p[2])) > math.pi / 2:
            rev, yaw = True, wrap(yaw + math.pi)
        self.turn(yaw, tol=0.04)
        t0 = time.time()
        while time.time() - t0 < 20:
            self.tick()
            p = self.pose()
            if not p:
                continue
            d = math.hypot(hx - p[0], hy - p[1])
            ahead = (hx - p[0]) * math.cos(p[2]) + (hy - p[1]) * math.sin(p[2])
            if d < tol or (ahead < 0) != rev:
                break
            v = min(0.15, 0.6 * d + 0.03)
            self.send(-v if rev else v, 0)
        self.stop()

    def measure_cube(self, expected):
        """Average camera detections (base_footprint frame) into arena coordinates."""
        samples = []
        t0 = time.time()
        while time.time() - t0 < 1.0 and len(samples) < 8:
            self.tick()
            p = self.pose()
            if p and self.det and time.time() - self.det_time < 0.5:
                fx, fy = self.det
                c, s = math.cos(p[2]), math.sin(p[2])
                samples.append((p[0] + c * fx - s * fy, p[1] + s * fx + c * fy))
                self.det = None
        if not samples:
            self.log('      kamera: kup gorunmuyor, beklenen konum kullaniliyor')
            return expected
        cx = sum(o[0] for o in samples) / len(samples)
        cy = sum(o[1] for o in samples) / len(samples)
        off = math.hypot(cx - expected[0], cy - expected[1])
        if off > 0.35:
            self.log('      kamera: (%.2f, %.2f) beklenenden cok uzak, kullanilmadi' % (cx, cy))
            return expected
        self.log('      kamera: kup (%.3f, %.3f), beklenenden %.1f cm' % (cx, cy, 100 * off))
        return (cx, cy)

    def push_line(self, C, h, robot_end, speed=0.18):
        """Push along the line through C with heading h until the robot reaches robot_end."""
        ux, uy = math.cos(h), math.sin(h)
        max_cross = 0.0
        t0 = time.time()
        while time.time() - t0 < 90:
            self.tick()
            p = self.pose()
            if not p:
                continue
            x, y, yaw = p
            cross = -(x - C[0]) * uy + (y - C[1]) * ux
            left = (robot_end[0] - x) * ux + (robot_end[1] - y) * uy
            max_cross = max(max_cross, abs(cross))
            if left <= 0:
                break
            v = min(speed, 0.4 * left + 0.04)
            w = max(-0.5, min(0.5, 2.0 * wrap(h - yaw) - 3.0 * cross))
            self.send(v, w)
        self.stop()
        return max_cross

    def back_off(self, dist=0.25):
        p = self.wait_pose()
        t0 = time.time()
        while time.time() - t0 < 10:
            self.tick()
            q = self.pose()
            if q and math.hypot(q[0] - p[0], q[1] - p[1]) >= dist:
                break
            self.send(-0.15, 0)
        self.stop()

    # ---------- task ----------
    def segment(self, i, P, Q, dapp, waypoints, mode):
        h = math.atan2(Q[1] - P[1], Q[0] - P[0])
        ux, uy = math.cos(h), math.sin(h)
        A = (P[0] - dapp * ux, P[1] - dapp * uy)
        self.log('  parca %d: (%.2f, %.2f) -> (%.2f, %.2f), yon %.0f derece'
                 % (i, P[0], P[1], Q[0], Q[1], math.degrees(h)))
        self.status('%s parca %d' % (self.label, i))
        pts = [(wx, wy, math.radians(wd)) for (wx, wy, wd) in waypoints]
        if mode == 'ara':
            ok = self.step(i, 'Nav2 bekleme noktasina', self.nav2_go, pts)
            self.step(i, 'dogrudan yaklasma', self.go_to, *A)
        elif mode:
            for M in (mode if isinstance(mode, list) else []):
                self.step(i, 'dogrudan yaklasma (ara)', self.go_to, *M)
            self.step(i, 'dogrudan yaklasma', self.go_to, *A)
            ok = True
        else:
            ok = self.step(i, 'Nav2 yaklasma', self.nav2_go, pts + [(A[0], A[1], h)])
        p = self.wait_pose()
        self.log('      yaklasma %s: (%.2f, %.2f)' % ('tamam' if ok else 'BASARISIZ', p[0], p[1]))
        self.step(i, 'donme', self.turn, h)
        C = self.step(i, 'kamera', self.measure_cube, P) if i == 1 else P
        h2 = math.atan2(Q[1] - C[1], Q[0] - C[0])
        u2 = (math.cos(h2), math.sin(h2))
        lead = min(0.50, dapp)
        S = (C[0] - lead * u2[0], C[1] - lead * u2[1])
        p = self.wait_pose()
        side = -(p[0] - C[0]) * u2[1] + (p[1] - C[1]) * u2[0]
        behind = -((p[0] - C[0]) * u2[0] + (p[1] - C[1]) * u2[1])
        if abs(side) < 0.025 and abs(wrap(h2 - p[2])) < 0.15 and behind > CONTACT + 0.05:
            self.log('      hatta (yan sapma %.1f cm), hizalanma atlandi' % (100 * side))
        else:
            self.step(i, 'hizalanma', self.go_to, *S)
        self.step(i, 'donme', self.turn, h2)
        robot_end = (Q[0] - CONTACT * u2[0], Q[1] - CONTACT * u2[1])
        dev = self.step(i, 'itme', self.push_line, C, h2, robot_end)
        p = self.wait_pose()
        est = (p[0] + CONTACT * u2[0], p[1] + CONTACT * u2[1])
        self.step(i, 'geri cekilme', self.back_off)
        self.log('      kup tahmini (odometri/AMCL): (%.2f, %.2f), hattan sapma %.1f cm'
                 % (est[0], est[1], 100 * dev))
        return est

    def deliver(self, colour):
        name, zone, route = ROUTES[colour]
        self.color, self.label = name, colour
        self.log('### %s KUP ###' % TR[colour].upper())
        est = None
        for i, (P, Q, dapp, wps, mode) in enumerate(route, 1):
            est = self.segment(i, P, Q, dapp, wps, mode)
        inside = abs(est[0] - zone[0]) <= 0.30 - HALF and abs(est[1] - zone[1]) <= 0.30 - HALF
        self.log('  SONUC %s: tahmini (%.2f, %.2f), bolge merkezinden (%.2f, %.2f) -> %s'
                 % (TR[colour], est[0], est[1], est[0] - zone[0], est[1] - zone[1],
                    'bolgede (tahmin)' if inside else 'KONTROL ET'))
        self.status('%s bitti' % colour)

    def wait_ready(self):
        self.status('Nav2 bekleniyor')
        self.log('Nav2 ve konumlandirma bekleniyor...')
        t0 = time.time()
        while not self.nav.wait_for_server(timeout_sec=1.0):
            self.tick()
            if time.time() - t0 > 180:
                raise RuntimeError('navigate_to_pose bulunamadi')
        self.wait_pose(timeout=120)
        while self.sim == 0.0:
            self.tick()
        s0 = self.sim
        while self.sim - s0 < float(self.get_parameter('start_delay_sec').value):
            self.tick()

    def run(self):
        self.wait_ready()
        self.status('calisiyor')
        wall0, sim0 = time.time(), self.sim
        done = []
        for c in self.order:
            self.deliver(c)
            done.append((c, self.sim - sim0))
        wall, sim = time.time() - wall0, self.sim - sim0
        self.log('===== OZET =====')
        for c, t in done:
            self.log('  %-8s teslim ani: %.0f sn (simulasyon)' % (TR[c], t))
        self.log('  Toplam: %.0f sn simulasyon saati (sinir 480 sn), %.0f sn duvar saati [hiz orani %%%.0f]'
                 % (sim, wall, 100 * sim / max(wall, 1e-6)))
        kinds = {}
        for _, _, n, dt in self.times:
            k = 'Nav2' if n.startswith('Nav2') else ('dogrudan yaklasma' if n.startswith('dogrudan') else n)
            kinds[k] = kinds.get(k, 0) + dt
        for k, v in sorted(kinds.items(), key=lambda x: -x[1]):
            self.log('  %-18s %5.0f sn duvar saati (%%%.0f)' % (k, v, 100 * v / max(wall, 1e-6)))
        self.status('bitti %.0f' % sim)
        self.log('GOREV BITTI; robot duruyor.')


def main():
    rclpy.init()
    node = CubePusher()
    try:
        node.run()
    except Aborted:
        node.status('durduruldu')
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
