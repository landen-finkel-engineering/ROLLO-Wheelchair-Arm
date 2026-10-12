"""ROS 2 (Humble) node for WRA-1.  Thin wrapper around WraClient (all logic lives there).

Topics
  pub  /joint_states            sensor_msgs/JointState   URDF joints (50 Hz)
  pub  /wra/status              std_msgs/String          JSON: supervisor state, faults, inputs, servo temps
  pub  /wra/tcp_pose            geometry_msgs/PoseStamped  TCP in base_link
  sub  /wra/joint_goal          sensor_msgs/JointState   URDF joint names + positions (rad / m)
  sub  /wra/tcp_goal            geometry_msgs/PoseStamped  TCP pose in base_link (orientation = tool flange)
  sub  /wra/claw                std_msgs/Float32         pad-to-pad opening in mm (0..65)
  sub  /wra/knob_pose           geometry_msgs/PoseStamped  knob frame for /wra/open_door (vision or taught)
  sub  /wra/stop                std_msgs/Empty           cancel motion / task, hold position
Services (std_srvs)
  /wra/arm SetBool (true: arm/resume - press START on the pendant when asked; false: torque off)
  /wra/power_down Trigger   /wra/clear_faults Trigger   /wra/home Trigger   /wra/open_door Trigger
Parameters: port (/dev/ttyUSB0), calibration (path), side (right|left), max_joint_speed_dps (30)
"""
from __future__ import annotations

import json
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32, String
from std_srvs.srv import SetBool, Trigger

from .client import WraClient, load_calibration
from .tasks import DoorKnobTask, KnobParams


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()      # x, y, z, w


class WraDriver(Node):
    def __init__(self):
        super().__init__("wra_driver")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("calibration", "")
        self.declare_parameter("side", "right")
        self.declare_parameter("max_joint_speed_dps", 30.0)
        cal = load_calibration(self.get_parameter("calibration").value or None)
        self.client = WraClient(self.get_parameter("port").value, calibration=cal,
                                max_joint_speed_dps=self.get_parameter("max_joint_speed_dps").value,
                                side=self.get_parameter("side").value)      # user keep-out on (docs/09)
        self.m = self.client.model
        self.names = self.m.urdf_names()
        self.cancel = threading.Event()
        self.worker = None
        self.knob = None
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_st = self.create_publisher(String, "/wra/status", 10)
        self.pub_tcp = self.create_publisher(PoseStamped, "/wra/tcp_pose", 10)
        self.create_subscription(JointState, "/wra/joint_goal", self.on_joint_goal, 10)
        self.create_subscription(PoseStamped, "/wra/tcp_goal", self.on_tcp_goal, 10)
        self.create_subscription(Float32, "/wra/claw", self.on_claw, 10)
        self.create_subscription(PoseStamped, "/wra/knob_pose", self.on_knob, 10)
        self.create_subscription(Empty, "/wra/stop", self.on_stop, 10)
        self.create_service(SetBool, "/wra/arm", self.srv_arm)
        self.create_service(Trigger, "/wra/power_down", lambda rq, rs: self._trig(rs, self.client.power_down()))
        self.create_service(Trigger, "/wra/clear_faults", lambda rq, rs: self._trig(rs, self.client.clear_faults()))
        self.create_service(Trigger, "/wra/home", self.srv_home)
        self.create_service(Trigger, "/wra/open_door", self.srv_open_door)
        self.create_timer(0.02, self.publish)
        self.get_logger().info(f"WRA-1 driver ({self.m.variant}) on {self.get_parameter('port').value}; "
                               f"arm_geometry.json params_sha {self.m.g.get('params_sha', 'missing')} "
                               f"(must match output/reports/verification.json)")

    # ------------------------------------------------------------------ helpers
    def _trig(self, rs, code):
        rs.success = code == "OK"
        rs.message = code
        return rs

    def _run(self, fn, what):
        if self.worker and self.worker.is_alive():
            self.get_logger().warn(f"busy - {what} ignored (send /wra/stop first)")
            return False
        self.cancel.clear()

        def job():
            try:
                fn()
            except Exception as e:                   # noqa: BLE001 - report every failure to the user
                self.get_logger().error(f"{what} failed: {e}")
        self.worker = threading.Thread(target=job, daemon=True)
        self.worker.start()
        return True

    # ------------------------------------------------------------------ callbacks
    def publish(self):
        t = self.client.tel
        if t is None:
            return
        st = {"state": t.state, "faults": t.faults, "estop_ok": t.estop_ok, "docked": t.docked, "bus_V": t.bus_V,
              "knob_mode": t.knob_mode, "calibrated": t.calibrated,
              "servo_temp_C": {s.id: s.temp for s in t.servos.values()},
              "busy": bool(self.worker and self.worker.is_alive())}
        self.pub_st.publish(String(data=json.dumps(st)))
        q = self.client.joints()
        if q is None:
            return
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        js.name = self.names + ["claw_b"]
        v = self.m.to_urdf(*q)
        js.position = v + [v[-1]]
        self.pub_js.publish(js)
        p, R = self.m.tcp(*q[:4])
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (p[:3] / 1000.0).tolist()
        x, y, z, w = R_to_quat(R)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = x, y, z, w
        self.pub_tcp.publish(ps)

    def on_joint_goal(self, msg: JointState):
        cur = self.client.joints()
        if cur is None:
            return
        v = self.m.to_urdf(*cur)
        for n, x in zip(msg.name, msg.position):
            if n in self.names:
                v[self.names.index(n)] = x
        tgt = self.m.from_urdf(v)
        self._run(lambda: self.client.move_joints(tgt, should_stop=self.cancel.is_set), "joint goal")

    def on_tcp_goal(self, msg: PoseStamped):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) * 1000.0
        R = quat_to_R(msg.pose.orientation)
        self._run(lambda: self.client.move_tcp(p, R, should_stop=self.cancel.is_set), "tcp goal")

    def on_claw(self, msg: Float32):
        cur = self.client.joints()
        if cur is None:
            return
        tgt = (*cur[:4], max(0.0, min(65.0, msg.data)) / 2.0)
        self._run(lambda: self.client.move_joints(tgt, should_stop=self.cancel.is_set), "claw")

    def on_knob(self, msg: PoseStamped):
        self.knob = (np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) * 1000.0,
                     quat_to_R(msg.pose.orientation))

    def on_stop(self, _msg):
        self.cancel.set()
        self.get_logger().warn("STOP: motion cancelled, holding position")

    def srv_arm(self, rq, rs):
        code = self.client.arm() if rq.data else self.client.torque_off()
        t = self.client.tel
        rs.success = code == "OK"
        rs.message = f"{code}; state {t.state if t else '?'}" + ("  -> press START on the pendant" if t and t.state == "ARMING" else "")
        return rs

    def srv_home(self, rq, rs):
        h = self.m.g["home"]
        tgt = (h["j1"], h["j2"], h["j3"], tuple(h["wt"]) if self.m.abenics else (h["j4"], h["j5"]), 8.0)
        rs.success = self._run(lambda: self.client.move_joints(tgt, should_stop=self.cancel.is_set), "home")
        rs.message = "moving to park" if rs.success else "busy"
        return rs

    def srv_open_door(self, rq, rs):
        if self.knob is None:
            rs.success, rs.message = False, "no /wra/knob_pose received"
            return rs
        p, R = self.knob
        task = DoorKnobTask(self.client, log=self.get_logger().info, should_stop=self.cancel.is_set)
        rs.success = self._run(lambda: task.run(p, R, KnobParams()), "open_door")
        rs.message = "door task started" if rs.success else "busy"
        return rs


def main():
    rclpy.init()
    n = WraDriver()
    try:
        rclpy.spin(n)
    finally:
        n.client.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
