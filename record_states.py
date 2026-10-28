import csv, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM = ["crane_plus_joint1","crane_plus_joint2","crane_plus_joint3","crane_plus_joint4"]
HAND = "crane_plus_joint_hand"

class Rec(Node):
    def __init__(self, path, hz):
        super().__init__("record_states")
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t"] + ARM + [HAND])
        self.t0 = None
        self.last = -1.0
        self.dt = 1.0 / hz
        self.create_subscription(JointState, "/joint_states", self.cb, 50)
    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        rel = t - self.t0
        if rel - self.last < self.dt:
            return
        self.last = rel
        d = dict(zip(msg.name, msg.position))
        if not all(j in d for j in ARM + [HAND]):
            return
        self.w.writerow([round(rel,4)] + [round(d[j],6) for j in ARM] + [round(d[HAND],6)])
        self.f.flush()

rclpy.init()
n = Rec(sys.argv[1], float(sys.argv[2]) if len(sys.argv)>2 else 20.0)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
