#!/usr/bin/env python3
"""関節の可動域を一定時間測定する（閉ループ検証用）

学習済み方策を推論ノードで走らせている最中に実行し、/joint_states を
購読して各関節の最小値・最大値を記録する。

目視では「動いているように見えるが、実は基部が旋回していない」といった
状態を見分けられないため、数値で切り分けるために用意した。

使い方:
    source /opt/ros/humble/setup.bash
    python3 mon.py            # 60秒間測定して結果を表示

出力される関節の並びは /joint_states のものに従う。
CRANE+ V2 では以下の順になる（joint3 と joint4 が入れ替わっている点に注意）:
    crane_plus_joint1, crane_plus_joint2, crane_plus_joint4,
    crane_plus_joint3, crane_plus_joint_hand
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DURATION_SEC = 60.0
NUM_JOINTS = 5


class RangeMonitor(Node):
    def __init__(self):
        super().__init__("range_monitor")
        self.lo = [float("inf")] * NUM_JOINTS
        self.hi = [float("-inf")] * NUM_JOINTS
        self.names = None
        self.count = 0
        self.create_subscription(JointState, "/joint_states", self.callback, 10)

    def callback(self, msg):
        self.names = list(msg.name)
        self.count += 1
        for i, pos in enumerate(list(msg.position)[:NUM_JOINTS]):
            self.lo[i] = min(self.lo[i], pos)
            self.hi[i] = max(self.hi[i], pos)


def main():
    rclpy.init()
    node = RangeMonitor()
    start = time.time()
    while time.time() - start < DURATION_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"受信メッセージ数: {node.count}")
    if node.names is None:
        print("/joint_states を受信できませんでした。")
    else:
        print(f"{'関節':<28}{'最小':>10}{'最大':>10}{'可動幅':>10}")
        for i, name in enumerate(node.names[:NUM_JOINTS]):
            width = node.hi[i] - node.lo[i]
            print(f"{name:<28}{node.lo[i]:>10.3f}{node.hi[i]:>10.3f}{width:>10.3f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
