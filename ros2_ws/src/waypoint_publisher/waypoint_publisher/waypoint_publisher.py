# offboard_waypoints.py
import math, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)

sensor_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)

NAN = float('nan')

WAYPOINTS = [
    (0.0, 0.0, -3.0, 0.0),  # deeper first climb to guarantee motion
    (6.0, 0.0, -2.0, 0.0),
    (6.0, 6.0, -2.0, 1.57),
    (0.0, 6.0, -2.0, 3.14),
    (0.0, 0.0, -2.0, 0.0),
]
POS_TOL = 0.4  # m

def now_us(node: Node) -> int:
    # PX4 expects microseconds
    return int(node.get_clock().now().nanoseconds / 1000)

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard_waypoints')
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd  = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.setpoint_count = 0
        self.offboard_sent = False

        self.wp_idx = 0
        self.pose = None
        self.offboard_req_sent = False

        # 50 Hz streams (>=2 Hz required by PX4)
        self.create_timer(0.02, self.tick_mode)
        self.create_timer(0.02, self.tick_traj)
        # Arm + OFFBOARD request at 1 Hz (after streams are already flowing)
        self.create_timer(1.0,  self.try_arm_offboard)

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_pose,
            sensor_qos
        )

    def on_pose(self, msg: VehicleLocalPosition):
        self.pose = (msg.x, msg.y, msg.z)

    def tick_mode(self):
        m = OffboardControlMode()
        m.timestamp = now_us(self)
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.thrust_and_torque = False
        m.direct_actuator = False
        self.pub_mode.publish(m)


    def tick_traj(self):
        x, y, z, yaw = WAYPOINTS[self.wp_idx]
        t = TrajectorySetpoint()
        t.timestamp = now_us(self)

        # self.pub_traj.publish(t)
        self.setpoint_count += 1

        t.position = [float(x), float(y), float(z)]
        t.velocity = [float('nan')] * 3
        t.acceleration = [float('nan')] * 3
        t.jerk = [float('nan')] * 3  # present per your interface; logging only
        t.yaw = float(yaw)
        t.yawspeed = float('nan')

        self.pub_traj.publish(t)

        if self.pose:
            dx = x - self.pose[0]; dy = y - self.pose[1]; dz = z - self.pose[2]
            if (dx*dx + dy*dy + dz*dz) ** 0.5 < POS_TOL:
                self.wp_idx = min(self.wp_idx + 1, len(WAYPOINTS)-1)

    def cmd(self, command: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        c = VehicleCommand()
        c.timestamp = now_us(self)

        # enums/ids must be ints
        c.command = int(command)
        c.target_system = int(1)
        c.target_component = int(1)
        c.source_system = int(1)
        c.source_component = int(1)
        c.confirmation = int(0)

        # params are floats (per the msg definition)
        c.param1 = float(p1)
        c.param2 = float(p2)
        c.param3 = float(p3)
        c.param4 = float(p4)
        c.param5 = float(p5)
        c.param6 = float(p6)
        c.param7 = float(p7)

        c.from_external = True
        self.pub_cmd.publish(c)


    def try_arm_offboard(self):
        # Keep streaming first; then arm and request OFFBOARD
        if self.pose is None:
            self.get_logger().warn("No local pose yet; waiting before arming/OFFBOARD...")
            return
        if self.setpoint_count < 50:  # ~1s at 50 Hz
            return
        if self.offboard_sent:
            return
        # 1) Arm
        self.cmd(400, 1.0)  # MAV_CMD_COMPONENT_ARM_DISARM (arm)
        # # 2) Request OFFBOARD: base_mode=1 (custom mode enabled), custom_main_mode=6 (OFFBOARD)
        # #    MAV_CMD_DO_SET_MODE == 176
        self.cmd(176, 1.0, 6.0, 0.0)
        # Arm
        # self.cmd(OffboardNode.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # # Switch to OFFBOARD
        # self.cmd(OffboardNode.MAV_CMD_DO_SET_MODE, 1.0, 6.0, 0.0)


def main():
    rclpy.init()
    n = OffboardNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
