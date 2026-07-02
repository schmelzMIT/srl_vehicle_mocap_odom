import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleCommand
import socket
import re

CONTROL_RATE = 20  # Hz

# Calibrate by timing how long a yaw burst at THRUST takes to turn 90 degrees.
YAW_RATE_DEG_PER_SEC = 45.0
THRUST = 1.0

# Motor index map (see rc_direct_control.py / autonomous_control.py)
# NOTE: T1-T4 are tied together on the actuation PCB — command all four
# simultaneously to get full voltage to the solenoid drivers.
T1, T2, T3, T4, T5, T6, T7, T8 = range(8)


def forward(duration, thrust=THRUST):
    return [({T1: thrust, T2: thrust, T3: thrust, T4: thrust}, duration)]


def backward(duration, thrust=THRUST):
    return [({T1: thrust, T2: thrust, T3: thrust, T4: thrust}, duration)]


def strafe_right(duration, thrust=THRUST):
    return [({T5: thrust, T7: thrust}, duration)]


def strafe_left(duration, thrust=THRUST):
    return [({T6: thrust, T8: thrust}, duration)]


def yaw_cw(duration, thrust=THRUST):
    return [({T1: thrust, T2: thrust, T3: thrust, T4: thrust, T5: thrust, T8: thrust}, duration)]


def yaw_ccw(duration, thrust=THRUST):
    return [({T1: thrust, T2: thrust, T3: thrust, T4: thrust, T6: thrust, T7: thrust}, duration)]


def turn(degrees, thrust=THRUST):
    """Positive degrees turns CW, negative turns CCW."""
    duration = abs(degrees) / YAW_RATE_DEG_PER_SEC
    return yaw_cw(duration, thrust) if degrees > 0 else yaw_ccw(duration, thrust)


def turn_90(thrust=THRUST):
    return turn(90, thrust)


def stop(duration):
    return [({}, duration)]


# Sequence of ({motor_index: value, ...}, duration_seconds)
# Runs in order, one step at a time, then stops (all motors off) and disarms.
SEQUENCE = (
    ({T1: 1.0, T2: 1.0, T3: 1.0, T4: 1.0}, 5),  # solenoids 1-4 (must fire together)
    ({T5: 1.0, T6: 1.0, T7: 1.0, T8: 1.0}, 5),  # solenoids 5-8
)


class OpenLoopSequence(Node):
    def __init__(self):
        super().__init__('open_loop_sequence')

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
        self.command_pub = self.create_publisher(
            VehicleCommand,
            f'/{namespace}/fmu/in/vehicle_command',
            qos_pub
        )

        self.step_index = 0
        self.step_elapsed = 0.0
        self.dt = 1.0 / CONTROL_RATE
        self.finished = False
        self.armed = False
        self.elapsed = 0.0

        self.create_timer(self.dt, self.control_loop)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        now = int(self.get_clock().now().nanoseconds / 1000)
        msg = VehicleCommand()
        msg.timestamp = now
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent.')
        self.armed = True

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent.')

    def engage_offboard(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent.')

    def current_motor_command(self):
        motors = [0.0] * 8

        if self.step_index >= len(SEQUENCE):
            if not self.finished:
                self.get_logger().info('Sequence complete, all motors off.')
                self.disarm()
                self.finished = True
            return motors

        motor_values, duration = SEQUENCE[self.step_index]
        for motor_idx, value in motor_values.items():
            motors[motor_idx] = value

        self.step_elapsed += self.dt
        if self.step_elapsed >= duration:
            fired = ', '.join(f'T{i + 1}' for i in motor_values) or 'stop'
            self.get_logger().info(f'Step {self.step_index} done ({fired})')
            self.step_index += 1
            self.step_elapsed = 0.0

        return motors

    def control_loop(self):
        now = int(self.get_clock().now().nanoseconds / 1000)
        self.elapsed += self.dt

        # Publish offboard signal for 1s before switching mode, arm at 1.5s
        if not self.armed:
            if self.elapsed >= 1.0:
                self.engage_offboard()
            if self.elapsed >= 1.5:
                self.arm()

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = now
        offboard_msg.direct_actuator = True
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_pub.publish(offboard_msg)

        motors = self.current_motor_command() if self.armed else [0.0] * 8

        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = now
        actuator_msg.timestamp_sample = now
        for i, v in enumerate(motors):
            actuator_msg.control[i] = float(v)
        self.actuator_pub.publish(actuator_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopSequence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
