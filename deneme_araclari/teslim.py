#!/usr/bin/env python3
"""6A-6F: kupleri Nav2 + kamera + kontrollu itme ile hedeflerine teslim eder.

Kullanim:
  python3 ~/teslim.py kirmizi        # tek kup
  python3 ~/teslim.py yesil kirmizi mavi   # sirayla uc kup (sure olculur)

Her kup birkac duz itme parcasindan (segment) olusur. Her parcada:
  1) Nav2 ile parcanin gerisindeki yaklasma pozuna git          (Nav2 modu)
  2) Kamerayla kupun gercek konumunu olc                          (6A)
  3) Kupten parca sonuna giden hattin gerisine hizalan           (6B)
  4) Hat boyunca yavas ve kontrollu it, geri cekil               (6C, 6D)
Sonunda Gazebo'dan kupun bolgede olup olmadigi olculur           (6E)
Rotalarin duvar/kup paylari onceden geometrik olarak kontrol edildi
(en dar yer: yesilde B gecidi 9 cm, digerlerinde >= 11 cm).

v2 (sure kisaltma): her adimin suresi yazilir; ara Gazebo olcumleri kapali
(--olc ile acilir); kamera yalnizca ilk parcada; ara nokta + yaklasma tek
Nav2 hedefinde (navigate_through_poses); kirmizi/mavi 2. parcaya kisa
mesafe oldugu icin Nav2 yerine dogrudan gidilir (ayak izi payi 14 cm).

v3: kirmizi/mavi ilk yaklasma noktasi kup ile kose duvari arasinda sikisik
(ayak izi payi 1.5 cm) oldugu icin Nav2 oraya 240 sn ugrasip basaramiyordu.
Artik Nav2 yalnizca acik bir bekleme noktasina (-2.35, +-1.40) gider (donme
payi 21 cm); kalan 0.4 m'yi robot dogrudan gider (ayak izi payi 9 cm).
Sure hem duvar saati hem simulasyon saati ile yazilir.
"""
import math
import re
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

START = (-2.40, -1.40)
HALF = 0.09
FRONT = 0.2375
CONTACT = HALF + FRONT

# renk: (gazebo/kamera adi, bolge merkezi, [(baslangic, bitis, yaklasma_mesafesi), ...])
# Her parca: (kup baslangic, kup bitis, yaklasma mesafesi, Nav2 ara noktalari)
# Ara noktalar robotun diger kuplere carpmadan yaklasma noktasina gelmesi icin:
# kirmizi/mavi'nin ilk parcasinda robot kupun bati tarafindan gelir.
# Son alan 'kendi': 'ara' ise Nav2 yalnizca ara noktalara gider (sonuncusu
# acik bir bekleme noktasi), yaklasma noktasina robot dogrudan gider.
# True ise yaklasma noktasina Nav2 yerine dogrudan gidilir
# (yalnizca kisa ve payi genis atlamalar: kirmizi/mavi 1->2, pay ~14 cm).
# 2->3 atlamasinda pay ~4 cm (B duvari yakin), orada Nav2 kullanilir.
ROTALAR = {
    'yesil':   ('green', (2.35, 0.0),  [((-1.60, 0.00), (2.35, -0.10), 0.70, [], False)]),
    'kirmizi': ('red',   (2.35, 1.2),  [((-1.60, 1.20), (-0.40, -0.05), 0.55, [(-2.30, 0.55, 90), (-2.35, 1.40, 90)], 'ara'),
                                        ((-0.40, -0.05), (1.60, -0.10), 0.60, [], True),
                                        ((1.60, -0.10), (2.35, 1.20), 0.50, [], False)]),
    'mavi':    ('blue',  (2.35, -1.2), [((-1.60, -1.20), (-0.45, -0.05), 0.55, [(-2.30, -0.55, -90), (-2.35, -1.40, -90)], 'ara'),
                                        ((-0.45, -0.05), (1.60, -0.40), 0.60, [], True),
                                        ((1.60, -0.40), (2.35, -1.20), 0.50, [], False)]),
}
ARA_OLCUM = '--olc' in sys.argv   # parca sonlarinda Gazebo olcumu (yavaslatir)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gazebo_pose(name):
    try:
        out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/competition/pose/info'],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    m = re.search(r'name: "%s".*?position \{(.*?)\}' % name, out, re.S)
    if not m:
        return None
    p = dict(re.findall(r'(\w+): ([-\d.e]+)', m.group(1)))
    return float(p.get('x', 0)), float(p.get('y', 0))


class Teslim(Node):
    def __init__(self):
        super().__init__('teslim', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navp = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.sureler = []   # (renk, parca, adim, sn)
        self.renk = ''
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.color = None
        self.det = None
        self.det_time = 0.0
        self.create_subscription(Detection3DArray, '/cube_detector/detections_3d', self.on_det, 5)
        self.sim = 0.0
        self.create_subscription(Clock, '/clock', self.on_clock, 10)

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_det(self, msg):
        for d in msg.detections:
            if self.color and d.results and d.results[0].hypothesis.class_id == self.color + '_cube':
                p = d.bbox.center.position
                self.det = (p.x, p.y)
                self.det_time = time.time()

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x + START[0], t.transform.translation.y + START[1],
                2 * math.atan2(q.z, q.w))

    def wait_pose(self):
        for _ in range(100):
            self.tick()
            p = self.pose()
            if p:
                return p
        raise RuntimeError('TF map->base_footprint yok (Nav2/AMCL calisiyor mu?)')

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def stop(self):
        for _ in range(5):
            self.send(0, 0)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- Nav2 ----
    def nav2_git(self, x, y, yaw, timeout=240):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x - START[0]
        g.pose.pose.position.y = y - START[1]
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        if not self.nav.wait_for_server(timeout_sec=60):
            return False
        f = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30)
        h = f.result()
        if not h or not h.accepted:
            return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=timeout)
        if not r.done():
            h.cancel_goal_async()
            return False
        return r.result().status == GoalStatus.STATUS_SUCCEEDED

    def nav2_noktalar(self, noktalar, timeout=240):
        """Ara noktalardan durmadan gecip son noktaya git (tek Nav2 hedefi).
        navigate_through_poses yoksa noktalara tek tek gider."""
        if len(noktalar) == 1 or not self.navp.wait_for_server(timeout_sec=2):
            ok = True
            for (x, y, yaw) in noktalar:
                ok = self.nav2_git(x, y, yaw, timeout)
            return ok
        g = NavigateThroughPoses.Goal()
        for (x, y, yaw) in noktalar:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = x - START[0]
            ps.pose.position.y = y - START[1]
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            g.poses.append(ps)
        f = self.navp.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30)
        h = f.result()
        if not h or not h.accepted:
            print('     navigate_through_poses reddetti, tek tek gidiliyor')
            ok = True
            for (x, y, yaw) in noktalar:
                ok = self.nav2_git(x, y, yaw, timeout)
            return ok
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=timeout)
        if not r.done():
            h.cancel_goal_async()
            return False
        return r.result().status == GoalStatus.STATUS_SUCCEEDED

    def adim(self, parca, ad, fn, *a, **k):
        """fn'i calistir, suresini kaydet ve yazdir."""
        t0 = time.time()
        r = fn(*a, **k)
        dt = time.time() - t0
        self.sureler.append((self.renk, parca, ad, dt))
        print('       [%5.1f sn] %s' % (dt, ad))
        return r

    # ---- kendi hareketlerimiz ----
    def don(self, heading, tol=0.025):
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

    def noktaya_git(self, hx, hy, tol=0.02):
        """Kisa mesafede bir noktaya once donup sonra duz git."""
        p = self.wait_pose()
        if math.hypot(hx - p[0], hy - p[1]) < tol:
            return
        geri = False
        yon = math.atan2(hy - p[1], hx - p[0])
        if abs(wrap(yon - p[2])) > math.pi / 2:  # arkadaysa geri geri git
            geri, yon = True, wrap(yon + math.pi)
        self.don(yon, tol=0.04)
        t0 = time.time()
        while time.time() - t0 < 20:
            self.tick()
            p = self.pose()
            if not p:
                continue
            d = math.hypot(hx - p[0], hy - p[1])
            ileri = (hx - p[0]) * math.cos(p[2]) + (hy - p[1]) * math.sin(p[2])
            if d < tol or (ileri < 0) != geri:
                break
            v = min(0.15, 0.6 * d + 0.03)
            self.send(-v if geri else v, 0)
        self.stop()

    def kup_olc(self, beklenen):
        """Kamerayla kupun saha konumunu olc (birkac olcumun ortalamasi)."""
        olcumler = []
        t0 = time.time()
        while time.time() - t0 < 1.0 and len(olcumler) < 8:
            self.tick()
            p = self.pose()
            if p and self.det and time.time() - self.det_time < 0.5:
                fx, fy = self.det
                c, s = math.cos(p[2]), math.sin(p[2])
                olcumler.append((p[0] + c * fx - s * fy, p[1] + s * fx + c * fy))
                self.det = None
        if not olcumler:
            print('     kamera kupu goremedi, beklenen konum kullaniliyor')
            return beklenen
        cx = sum(o[0] for o in olcumler) / len(olcumler)
        cy = sum(o[1] for o in olcumler) / len(olcumler)
        if math.hypot(cx - beklenen[0], cy - beklenen[1]) > 0.35:
            print('     kamera olcumu (%.2f, %.2f) beklenenden cok uzak, beklenen kullaniliyor' % (cx, cy))
            return beklenen
        print('     kamera: kup (%.3f, %.3f), beklenenden %.1f cm' %
              (cx, cy, 100 * math.hypot(cx - beklenen[0], cy - beklenen[1])))
        return (cx, cy)

    def hat_boyunca_it(self, C, h, robot_son, hiz=0.18):
        """C'den gecen h yonlu hat uzerinde, robot merkezi robot_son'a gelene kadar it."""
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
            kalan = (robot_son[0] - x) * ux + (robot_son[1] - y) * uy
            max_cross = max(max_cross, abs(cross))
            if kalan <= 0:
                break
            v = min(hiz, 0.4 * kalan + 0.04)
            w = max(-0.5, min(0.5, 2.0 * wrap(h - yaw) - 3.0 * cross))
            self.send(v, w)
        self.stop()
        return max_cross

    def geri_cekil(self, mesafe=0.25):
        p = self.wait_pose()
        t0 = time.time()
        while time.time() - t0 < 10:
            self.tick()
            q = self.pose()
            if q and math.hypot(q[0] - p[0], q[1] - p[1]) >= mesafe:
                break
            self.send(-0.15, 0)
        self.stop()

    # ---- bir parca (segment) ----
    def parca(self, i, P, Q, dapp, ara, kendi, cube_name):
        h = math.atan2(Q[1] - P[1], Q[0] - P[0])
        ux, uy = math.cos(h), math.sin(h)
        A = (P[0] - dapp * ux, P[1] - dapp * uy)
        print('  parca %d: (%.2f, %.2f) -> (%.2f, %.2f), yon %.0f derece' % (i, P[0], P[1], Q[0], Q[1], math.degrees(h)))
        if kendi == 'ara':
            noktalar = [(wx, wy, math.radians(wd)) for (wx, wy, wd) in ara]
            ok = self.adim(i, 'Nav2 yaklasma (bekleme noktasina)', self.nav2_noktalar, noktalar)
            self.adim(i, 'dogrudan yaklasma', self.noktaya_git, *A)
        elif kendi:
            self.adim(i, 'dogrudan yaklasma', self.noktaya_git, *A)
            ok = True
        else:
            noktalar = [(wx, wy, math.radians(wd)) for (wx, wy, wd) in ara] + [(A[0], A[1], h)]
            ad = 'Nav2 yaklasma' + (' (+%d ara nokta)' % len(ara) if ara else '')
            ok = self.adim(i, ad, self.nav2_noktalar, noktalar)
        p = self.wait_pose()
        print('     yaklasma %s: (%.2f, %.2f)' % ('tamam' if ok else 'BASARISIZ', p[0], p[1]))
        self.adim(i, 'donme', self.don, h)
        C = self.adim(i, 'kamera', self.kup_olc, P) if i == 1 else P
        h2 = math.atan2(Q[1] - C[1], Q[0] - C[0])
        u2 = (math.cos(h2), math.sin(h2))
        on = min(0.50, dapp)
        S = (C[0] - on * u2[0], C[1] - on * u2[1])
        self.adim(i, 'hizalanma', self.noktaya_git, *S)
        self.adim(i, 'donme', self.don, h2)
        robot_son = (Q[0] - CONTACT * u2[0], Q[1] - CONTACT * u2[1])
        sapma = self.adim(i, 'itme', self.hat_boyunca_it, C, h2, robot_son)
        self.adim(i, 'geri cekilme', self.geri_cekil)
        if ARA_OLCUM:
            g = gazebo_pose(cube_name)
            if g:
                print('     parca sonu: kup gercekte (%.3f, %.3f), hedeften %.1f cm, hat sapmasi %.1f cm' %
                      (g[0], g[1], 100 * math.hypot(g[0] - Q[0], g[1] - Q[1]), 100 * sapma))

    def kup(self, renk):
        name, zone, rota = ROTALAR[renk]
        self.color = name
        self.renk = renk
        t0 = time.time()
        print('\n### %s kup ###' % renk.upper())
        for i, (P, Q, dapp, ara, kendi) in enumerate(rota, 1):
            self.parca(i, P, Q, dapp, ara, kendi, name + '_cube')
        g = gazebo_pose(name + '_cube')
        sure = time.time() - t0
        if g:
            icinde = abs(g[0] - zone[0]) <= 0.30 - HALF and abs(g[1] - zone[1]) <= 0.30 - HALF
            print('  SONUC %s: kup (%.3f, %.3f), bolge merkezinden (%.3f, %.3f) -> TESLIM %s  [%.0f sn]' %
                  (renk, g[0], g[1], g[0] - zone[0], g[1] - zone[1], 'EVET' if icinde else 'HAYIR', sure))
            return icinde, sure
        print('  SONUC %s: Gazebo konumu okunamadi [%.0f sn]' % (renk, sure))
        return None, sure

    def sim_sn(self):
        return self.sim

    def run(self, renkler):
        t = time.time()
        while self.sim == 0.0 and time.time() - t < 5:   # /clock gelsin
            self.tick()
        s0 = self.sim_sn()
        t0 = time.time()
        sonuc = []
        for r in renkler:
            ok, s = self.kup(r)
            sonuc.append((r, ok, time.time() - t0))
        print('\n===== OZET =====')
        for r, ok, t in sonuc:
            print('  %-8s %-6s teslim ani: %.0f sn' % (r, {True: 'EVET', False: 'HAYIR', None: '?'}[ok], t))
        toplam = time.time() - t0
        print('  Toplam: %.0f sn duvar saati' % toplam)
        if s0 > 0:
            sim = self.sim_sn() - s0
            print('  Toplam: %.0f sn SIMULASYON saati (sinir 480 sn)  [hiz orani %%%.0f]' % (sim, 100 * sim / toplam))
        else:
            print('  (simulasyon saati okunamadi)')
        print('  Gorev bitti; robot duruyor, baska komut gonderilmiyor.')
        print('\n===== SURE DAGILIMI =====')
        tur = {}
        for _, _, ad, dt in self.sureler:
            k = 'Nav2 yaklasma' if ad.startswith('Nav2') else ad
            tur[k] = tur.get(k, 0) + dt
        for k, v in sorted(tur.items(), key=lambda x: -x[1]):
            print('  %-18s %6.0f sn  (%%%.0f)' % (k, v, 100 * v / toplam))
        print('  %-18s %6.0f sn' % ('diger/olcum', toplam - sum(tur.values())))
        print('\n  En uzun 5 adim:')
        for r, i, ad, dt in sorted(self.sureler, key=lambda x: -x[3])[:5]:
            print('    %-8s parca %d  %-28s %5.1f sn' % (r, i, ad, dt))


def main():
    renkler = [a for a in sys.argv[1:] if a in ROTALAR] or ['yesil']
    rclpy.init()
    node = Teslim()
    try:
        node.run(renkler)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
