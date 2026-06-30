import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleAttitude, VehicleLocalPosition
import socket
import re
import numpy as np

CONTROL_RATE = 20  # Hz


class AutonomousControl(Node):
    def __init__(self):
        super().__init__('autonomous_control')

        namespace = self.declare_parameter('namespace', '').value
        if namespace == '':
            namespace = socket.gethostname()
            namespace = re.sub(r'[^a-zA-Z0-9_~{}]', '_', namespace)

        self.get_logger().info(f'Namespace: {namespace}')

        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.actuator_pub = self.create_publisher(
            ActuatorMotors,
            f'/{namespace}/fmu/in/actuator_motors',
            qos_pub
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'/{namespace}/fmu/in/offboard_control_mode',
            qos_pub
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            f'/{namespace}/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_sub
        )
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/{namespace}/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_sub
        )

        # Current state
        self.attitude = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        self.position = np.array([0.0, 0.0, 0.0])         # NED [m]
        self.velocity = np.array([0.0, 0.0, 0.0])         # NED [m/s]

        # Target state — modify these to set your goal
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.target_yaw = 0.0  # radians

        self.create_timer(1.0 / CONTROL_RATE, self.control_loop)

    def attitude_callback(self, msg: VehicleAttitude):
        self.attitude = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

    def position_callback(self, msg: VehicleLocalPosition):
        self.position = np.array([msg.x, msg.y, msg.z])
        self.velocity = np.array([msg.vx, msg.vy, msg.vz])

    def compute_actuator_commands(self):
        """
        Implement your control law here.
        Returns a list of 8 motor commands (0.0 to 1.0).

        self.position  — current position in NED [m]
        self.velocity  — current velocity in NED [m/s]
        self.attitude  — current quaternion [w, x, y, z]
        self.target_position — goal position in NED [m]
        """
        motors = [0.0] * 8

        # Example: simple position error in X axis fires T1/T2
        pos_error = self.target_position - self.position
        threshold = 0.05  # meters

        # X axis
        if pos_error[0] > threshold:
            motors[0] = 1.0   # T1 +X
            motors[2] = 1.0   # T3 +X
        elif pos_error[0] < -threshold:
            motors[1] = 1.0   # T2 -X
            motors[3] = 1.0   # T4 -X

        # Y axis
        if pos_error[1] > threshold:
            motors[4] = 1.0   # T5 +Y
            motors[6] = 1.0   # T7 +Y
        elif pos_error[1] < -threshold:
            motors[5] = 1.0   # T6 -Y
            motors[7] = 1.0   # T8 -Y

        return motors

    def control_loop(self):
        now = int(self.get_clock().now().nanoseconds / 1000)

        # Must publish offboard mode continuously
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = now
        offboard_msg.direct_actuator = True
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_pub.publish(offboard_msg)

        motors = self.compute_actuator_commands()

        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = now
        actuator_msg.timestamp_sample = now
        for i, v in enumerate(motors):
            actuator_msg.control[i] = float(v)
        self.actuator_pub.publish(actuator_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
