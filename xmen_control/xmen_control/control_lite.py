"""OpenCR: home tilt/pan at startup, then bridge Twist and JointState."""
import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

PERIOD, TIMEOUT = 0.01, 0.2
MAX_RAD_S, HOME_SPEED, HOME_TIMEOUT = 1.0, 0.3, 30.0
RAD_PER_TICK = 2 * math.pi / 4096
HOME_TOLERANCE, HOME_SETTLE_TIME = 8 * RAD_PER_TICK, 0.2


def read_position(line):
    fields = line.split()
    if len(fields) != 5 or fields[0] != 'P1':
        raise ValueError('Unexpected firmware reply')
    sample, pan, tilt, fault = map(int, fields[1:])
    if fault or not 0 <= sample <= 0xffffffff:
        raise ValueError('OpenCR fault or invalid sample')
    if abs(pan) > 2056 or abs(tilt) > 1544:
        raise ValueError('Position outside travel limits')
    return sample, [pan * RAD_PER_TICK, tilt * RAD_PER_TICK]


class OpenCRBridge(Node):
    def __init__(self):
        super().__init__('control_new')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.board = serial.Serial(
            port, 115200, timeout=0, write_timeout=0.02, exclusive=True)
        self.board.reset_input_buffer()
        self.start_at = self.sample_at = time.monotonic() + 1.5
        self.sample = self.position = self.pending = None
        self.command_at = 0.0
        self.home_axis = 1  # tilt, then pan; -1 means complete
        self.home_deadline = self.settled_at = None
        self.buffer = b''
        self.positions = self.create_publisher(JointState, '/joint_states', 1)
        self.commands = self.create_subscription(
            Twist, '/cmd_vel', self.write_speed, 1)
        self.timer = self.create_timer(PERIOD, self.tick)

    def send(self, text):
        data = (text + '\n').encode('ascii')
        if self.board.write(data) != len(data):
            raise OSError('Incomplete serial write')

    def write_speed(self, msg):
        if self.sample is None or self.home_axis >= 0:
            return
        speed = (msg.angular.z, msg.angular.y)  # pan, tilt; rad/s
        valid = all(math.isfinite(v) and abs(v) <= MAX_RAD_S for v in speed)
        self.pending = speed if valid else (0.0, 0.0)
        self.command_at = time.monotonic()

    def publish_position(self, line, now):
        sample, position = read_position(line)
        if sample == self.sample:
            return
        self.sample, self.position, self.sample_at = sample, position, now
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = ['pan', 'tilt'], position
        self.positions.publish(msg)

    def home(self, previous, now):
        if self.home_deadline is None:
            self.send('x')
            self.home_deadline = now + HOME_TIMEOUT
        if now >= self.home_deadline:
            raise ValueError('Startup homing timed out')
        position = self.position[self.home_axis]
        if abs(position) > HOME_TOLERANCE:
            self.settled_at = None
            speed = min(HOME_SPEED, max(0.03, abs(position)))
            command = [0.0, 0.0]  # 0.03 exceeds one motor speed unit
            command[self.home_axis] = math.copysign(speed, -position)
            self.send(f'v {command[0]:.6f} {command[1]:.6f}')
            return
        self.send('v 0 0')
        moved = previous is None or abs(
            position - previous[self.home_axis]) > RAD_PER_TICK
        if moved or self.settled_at is None:
            self.settled_at = now
        if now - self.settled_at < HOME_SETTLE_TIME:
            return
        if self.home_axis == 0 and any(
                abs(p) > HOME_TOLERANCE for p in self.position):
            raise ValueError('An axis moved away from home')
        self.send('x')
        self.home_axis -= 1
        self.home_deadline = self.settled_at = None
        if self.home_axis < 0:
            self.get_logger().info('Home reached; Twist enabled')

    def tick(self):
        if time.monotonic() < self.start_at:
            return
        try:
            old_sample, previous = self.sample, self.position
            self.buffer += self.board.read(min(self.board.in_waiting, 4096))
            *lines, self.buffer = self.buffer.split(b'\n')
            now = time.monotonic()
            for line in lines:
                self.publish_position(line.decode('ascii'), now)
            if len(self.buffer) > 256 or now - self.sample_at >= TIMEOUT:
                raise ValueError('Position feedback missing or invalid')
            if self.home_axis >= 0 and self.sample != old_sample:
                self.home(previous, now)  # advance only on fresh feedback
            elif self.home_axis < 0 and self.pending is not None:
                pan, tilt = self.pending
                if now - self.command_at >= TIMEOUT:
                    pan = tilt = 0.0
                self.send(f'v {pan:.6f} {tilt:.6f}')
                self.pending = None  # never replay a Twist
            self.send('p')
        except (ValueError, OSError) as error:
            self.close()
            self.timer.cancel()
            self.get_logger().error(f'{error}; restart the bridge')

    def close(self):
        if self.board.is_open:
            try:
                if self.sample is not None:
                    self.send('x')
            except OSError:
                pass
            self.board.close()
        self.sample = self.pending = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenCRBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
