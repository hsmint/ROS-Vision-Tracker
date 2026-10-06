"""Home once at startup, then bridge Twist and JointState over USB."""
import math
import time
from contextlib import suppress
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

PERIOD, TIMEOUT = 0.01, 0.2
MAX_SPEED, HOME_SPEED, HOME_TIMEOUT = 1.0, 0.3, 30.0
TICK_RAD = 2 * math.pi / 4096
HOME_TOLERANCE, SETTLE_TIME = 8 * TICK_RAD, 0.2
POSITION_LIMITS = ((-1024, 1024), (-796, 170))  # pan/tilt encoder ticks
POSITION_TOLERANCE = 8  # match the firmware feedback tolerance


class OpenCRBridge(Node):
    def __init__(self):
        super().__init__('control_lite')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.board = serial.Serial(
            port, 115200, timeout=0, write_timeout=0.02, exclusive=True)
        self.board.reset_input_buffer()
        self.start_at = self.sample_at = time.monotonic() + 1.5
        self.sample = self.position = self.pending = None
        self.command_at, self.buffer = 0.0, b''
        self.startup = self.home()  # runs once, on fresh feedback
        self.positions = self.create_publisher(JointState, '/joint_states', 1)
        self.commands = self.create_subscription(
            Twist, '/cmd_vel', self.write_speed, 1)
        self.timer = self.create_timer(PERIOD, self.tick)

    def send(self, text):
        data = (text + '\n').encode('ascii')
        if self.board.write(data) != len(data):
            raise OSError('Incomplete serial write')

    def write_speed(self, msg):
        if self.sample is None or self.startup is not None: return
        speed = (msg.angular.z, msg.angular.y)  # pan, tilt; rad/s
        valid = all(math.isfinite(v) and abs(v) <= MAX_SPEED for v in speed)
        self.pending = speed if valid else (0.0, 0.0)
        self.command_at = time.monotonic()

    def read_position(self, line, now):
        fields = line.split()
        if len(fields) != 5 or fields[0] != b'P1':
            raise ValueError('Unexpected firmware reply')
        sample, pan, tilt, fault = map(int, fields[1:])
        in_range = all(low - POSITION_TOLERANCE <= p <=
                       high + POSITION_TOLERANCE
                       for p, (low, high) in zip((pan, tilt), POSITION_LIMITS))
        if fault or not 0 <= sample <= 0xffffffff or not in_range:
            raise ValueError('OpenCR fault or invalid position')
        if sample == self.sample: return
        self.sample, self.sample_at = sample, now
        self.position = [pan * TICK_RAD, tilt * TICK_RAD]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = ['pan', 'tilt'], self.position
        self.positions.publish(msg)

    def home(self):
        for axis in (1, 0):  # tilt, then pan
            self.send('x')
            deadline = time.monotonic() + HOME_TIMEOUT
            settled = previous = None
            while True:
                now, position = time.monotonic(), self.position[axis]
                if now >= deadline: raise ValueError('Homing timed out')
                speed = [0.0, 0.0]
                if abs(position) > HOME_TOLERANCE:
                    settled = None
                    rate = min(HOME_SPEED, max(0.03, abs(position)))
                    speed[axis] = math.copysign(rate, -position)
                else:
                    if (settled is None or previous is None or
                            abs(position - previous) > TICK_RAD):
                        settled = now
                    if now - settled >= SETTLE_TIME: break
                self.send(f'v {speed[0]:.6f} {speed[1]:.6f}')
                previous = position
                yield True  # wait for another measured position
            self.send('x')
        if any(abs(p) > HOME_TOLERANCE for p in self.position):
            raise ValueError('An axis moved away from home')
        self.get_logger().info('Home reached; Twist enabled')

    def tick(self):
        if time.monotonic() < self.start_at: return
        try:
            old_sample = self.sample
            self.buffer += self.board.read(min(self.board.in_waiting, 4096))
            *lines, self.buffer = self.buffer.split(b'\n')
            now = time.monotonic()
            for line in lines: self.read_position(line, now)
            if len(self.buffer) > 256 or now - self.sample_at >= TIMEOUT:
                raise ValueError('Position feedback missing or invalid')
            if self.startup is not None and self.sample != old_sample:
                if not next(self.startup, False): self.startup = None
            elif self.startup is None and self.pending is not None:
                pan, tilt = self.pending
                if now - self.command_at >= TIMEOUT: pan = tilt = 0.0
                self.send(f'v {pan:.6f} {tilt:.6f}')
                self.pending = None  # never replay a Twist
            self.send('p')
        except (ValueError, OSError) as error:
            self.close()
            self.timer.cancel()
            self.get_logger().error(f'{error}; restart the bridge')

    def close(self):
        if self.board.is_open:
            with suppress(OSError):
                if self.sample is not None: self.send('x')
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
        if rclpy.ok(): rclpy.shutdown()
