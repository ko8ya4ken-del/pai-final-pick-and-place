#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MoveIt  pick and place (CSV)rosbag """
import argparse, csv, os, signal, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_TOPIC = "/crane_plus_arm_controller/joint_trajectory"
HAND_TOPIC = "/crane_plus_gripper_controller/joint_trajectory"
# /joint_states joint1, joint2, joint4, joint3, hand
ARM_JOINTS = ["crane_plus_joint1", "crane_plus_joint2",
              "crane_plus_joint4", "crane_plus_joint3"]
HAND_JOINTS = ["crane_plus_joint_hand"]
HAND_COL = "crane_plus_joint_hand"

CUBE_REQ = ('name: "wood_cube_5cm", position: {x: %f, y: %f, z: 1.04}, '
            'orientation: {x: -0.7071068, y: 0.0, z: 0.0, w: 0.7071068}')


def reset_cube(x=0.0, y=-0.1):
    subprocess.run(["ign", "service", "-s", "/world/default/set_pose",
                    "--reqtype", "ignition.msgs.Pose",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "3000", "--req", CUBE_REQ % (x, y)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def load_csv(path):
    rows = []
    with open(path) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[j]) for j in ARM_JOINTS] + [float(d[HAND_COL])])
    return rows


class Replay(Node):
    def __init__(self, rows, hz=20.0, lookahead=3):
        super().__init__("replay_demo", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.hand_pub = self.create_publisher(JointTrajectory, HAND_TOPIC, 10)
        self.rows = rows
        self.la = lookahead
        self.dt = 1.0 / hz
        self.i = 0
        self.finished = True
        self.create_timer(self.dt, self.tick)

    def _dur(self, sec):
        return Duration(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))

    def hold(self, idx, n=20):
        """ n """
        for _ in range(n):
            self.publish_row(idx, 0.5)
            time.sleep(0.05)

    def publish_row(self, idx, sec):
        r = self.rows[idx]
        a = JointTrajectory(); a.joint_names = ARM_JOINTS
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in r[0:4]]
        p.time_from_start = self._dur(sec); a.points = [p]
        self.arm_pub.publish(a)
        h = JointTrajectory(); h.joint_names = HAND_JOINTS
        q = JointTrajectoryPoint(); q.positions = [float(r[4])]
        q.time_from_start = self._dur(sec); h.points = [q]
        self.hand_pub.publish(h)

    def start(self):
        self.i = 0
        self.finished = False

    def tick(self):
        if self.finished:
            return
        if self.i >= len(self.rows):
            self.finished = True
            return
        arm_pts, hand_pts = [], []
        for k in range(self.la):
            r = self.rows[min(self.i + k, len(self.rows) - 1)]
            d = self._dur(self.dt * (k + 1))
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in r[0:4]]; p.time_from_start = d
            arm_pts.append(p)
            q = JointTrajectoryPoint()
            q.positions = [float(r[4])]; q.time_from_start = d
            hand_pts.append(q)
        a = JointTrajectory(); a.joint_names = ARM_JOINTS; a.points = arm_pts
        self.arm_pub.publish(a)
        h = JointTrajectory(); h.joint_names = HAND_JOINTS; h.points = hand_pts
        self.hand_pub.publish(h)
        self.i += 1


def bag_start(outdir, name, sim):
    cmd = ["ros2", "bag", "record"]
    if sim:
        cmd.append("--use-sim-time")
    cmd += ["/joint_states", ARM_TOPIC, HAND_TOPIC, "-o", os.path.join(outdir, name)]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, preexec_fn=os.setsid)


def bag_stop(proc):
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    proc.wait(timeout=60)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/demo.csv")
    ap.add_argument("--episodes", type=int, default=15)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--outdir", default=os.path.expanduser("~/dataset/rosbags"))
    ap.add_argument("--no-sim-time", action="store_true")
    args = ap.parse_args()

    rows = load_csv(args.csv)
    os.makedirs(args.outdir, exist_ok=True)
    print("loaded %d rows" % len(rows), flush=True)

    rclpy.init()
    node = Replay(rows, args.hz)
    time.sleep(2.0)
    try:
        for i in range(args.start, args.start + args.episodes):
            name = "ep%d" % i
            reset_cube()
            node.hold(0, 20)
            proc = bag_start(args.outdir, name, not args.no_sim_time)
            time.sleep(3.0)
            node.start()
            while not node.finished:
                rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(1.0)
            bag_stop(proc)
            print("%s recorded" % name, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("ALL DONE", flush=True)


if __name__ == "__main__":
    main()
