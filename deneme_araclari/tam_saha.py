#!/usr/bin/env python3
"""5K: tam saha navigasyon testi (kup yaklasma pozlari + hedef bolge onleri).

Hedefler saha koordinatinda; harita cercevesi robotun baslangicinda (-2.40, -1.40)
sifirlandigi icin harita = saha - baslangic. Gazebo gercek konumu yalnizca olcum icindir.
"""
import math, re, subprocess, time
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

START_X, START_Y = -2.40, -1.40
HEDEFLER = [
    ('Mavi kup yaklasma',    -2.30, -1.20,   0),
    ('Yesil kup yaklasma',   -2.30,  0.00,   0),
    ('Kirmizi kup yaklasma', -2.30,  1.20,   0),
    ('A gecidi onu',         -0.90, -0.10,   0),
    ('B gecidi arkasi',       1.50, -0.40,   0),
    ('Kirmizi hedef onu',     1.70,  1.20,   0),
    ('Yesil hedef onu',       1.70,  0.00,   0),
    ('Mavi hedef onu',        1.70, -1.20,   0),
    ('B gecidi arkasi donus', 1.50, -0.40, 180),
    ('A gecidi onu donus',   -0.90, -0.10, 180),
]

def gazebo_robot():
    try:
        out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/competition/pose/info'],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    m = re.search(r'name: "pusher_robot".*?position \{(.*?)\}.*?orientation \{(.*?)\}', out, re.S)
    if not m:
        return None
    p = dict(re.findall(r'(\w+): ([-\d.e]+)', m.group(1)))
    q = dict(re.findall(r'(\w+): ([-\d.e]+)', m.group(2)))
    return float(p.get('x', 0)), float(p.get('y', 0)), 2 * math.atan2(float(q.get('z', 0)), float(q.get('w', 1)))

class Test(Node):
    def __init__(self):
        super().__init__('tam_saha')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def git(self, wx, wy, yd):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = wx - START_X
        g.pose.pose.position.y = wy - START_Y
        g.pose.pose.orientation.z = math.sin(math.radians(yd) / 2)
        g.pose.pose.orientation.w = math.cos(math.radians(yd) / 2)
        t0 = time.time()
        f = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if not h.accepted:
            return 'REDDEDILDI', 0
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=300)
        if not r.done():
            h.cancel_goal_async()
            return 'ZAMAN ASIMI', time.time() - t0
        s = r.result().status
        return ('BASARILI' if s == GoalStatus.STATUS_SUCCEEDED else 'BASARISIZ(%d)' % s), time.time() - t0

    def run(self):
        print('Nav2 bekleniyor...')
        if not self.client.wait_for_server(timeout_sec=60):
            print('Nav2 bulunamadi.')
            return
        for ad, wx, wy, yd in HEDEFLER:
            sonuc, sure = self.git(wx, wy, yd)
            p = gazebo_robot()
            ek = '' if not p else 'gercek (%.2f, %.2f), hata %.3f m' % (p[0], p[1], math.hypot(p[0] - wx, p[1] - wy))
            print('%-16s %-12s %5.1f sn  %s' % (ad, sonuc, sure, ek))
            if sonuc != 'BASARILI':
                print('DURDU.')
                return
        print('TAMAM: 5K tam saha navigasyonu basarili.')

rclpy.init()
Test().run()
rclpy.shutdown()
