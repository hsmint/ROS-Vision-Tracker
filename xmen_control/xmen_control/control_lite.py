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
WRITE_TIMEOUT = 0.1  # allow USB scheduling jitter; firmware watchdog stays 0.2s
MAX_SPEED, HOME_SPEED, HOME_TIMEOUT = 1.0, 0.3, 30.0
TICK_RAD = 2 * math.pi / 4096
HOME_TOLERANCE, SETTLE_TIME = 8 * TICK_RAD, 0.2
HOME_APPROACH_TOLERANCE = 4 * TICK_RAD
HOME_ATTEMPTS = 3
POSITION_LIMITS = ((-2048, 2048), (-1365, 1365))  # pan +/-180, tilt +/-120 deg
POSITION_TOLERANCE = 8  # match the firmware feedback tolerance


class OpenCRBridge(Node):
    def __init__(self):
        super().__init__('control_lite')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.board = serial.Serial(
            port, 115200, timeout=0, write_timeout=WRITE_TIMEOUT, exclusive=True)
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
        try:
            written = self.board.write(data)
        except serial.SerialTimeoutException as error:
            raise OSError(
                f'USB write timed out after {WRITE_TIMEOUT:.2f}s '
                f'(command={text!r})') from error
        if written != len(data):
            raise OSError('Incomplete serial write')

    def write_speed(self, msg):
        if self.sample is None or self.startup is not None: return
        speed = (msg.angular.z, msg.angular.y)  # pan, tilt; rad/s
        valid = all(math.isfinite(v) and abs(v) <= MAX_SPEED for v in speed)
        self.pending = speed if valid else (0.0, 0.0)
        self.command_at = time.monotonic()

    def read_position(self, line, now):
        fields = line.split()
        if len(fields) == 7 and fields[0] == b'F1':
            reason = fields[1].decode('ascii')
            motor, register, value, library, status = map(int, fields[2:])
            raise ValueError(
                f'OpenCR fault: {reason}; motor_id={motor}, '
                f'register={register}, value={value}, '
                f'library_error={library}, status_error={status}')
        if len(fields) != 5 or fields[0] != b'P1':
            raise ValueError(
                f'Unexpected firmware reply: {line[:160]!r}; '
                'expected P1 SAMPLE PAN_TICKS TILT_TICKS FAULT from '
                'opencr_lite.ino. Full control firmware is incompatible')
        sample, pan, tilt, fault = map(int, fields[1:])
        details = f'sample={sample}, pan={pan}, tilt={tilt}, fault={fault}'
        if fault:
            raise ValueError(
                f'OpenCR firmware fault latched ({details}); '
                'check motor power/communication, startup position and motor '
                'status, then restart the OpenCR board before restarting '
                'the bridge')
        if not 0 <= sample <= 0xffffffff:
            raise ValueError(f'Invalid OpenCR sample counter ({details})')
        for name, position, (low, high) in zip(
                ('pan', 'tilt'), (pan, tilt), POSITION_LIMITS):
            if not low - POSITION_TOLERANCE <= position <= high + POSITION_TOLERANCE:
                raise ValueError(
                    f'OpenCR {name} position out of range ({details}); '
                    f'expected {low}..{high} ticks with '
                    f'{POSITION_TOLERANCE}-tick tolerance')
        if sample == self.sample: return
        self.sample, self.sample_at = sample, now
        self.position = [pan * TICK_RAD, tilt * TICK_RAD]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = ['pan', 'tilt'], self.position
        self.positions.publish(msg)

    def home(self):
        for attempt in range(1, HOME_ATTEMPTS + 1):
            yield from self.home_axes()
            if all(abs(p) <= HOME_TOLERANCE for p in self.position):
                self.get_logger().info('Home reached; Twist enabled')
                return
            details = ', '.join(
                f'{name}={position / TICK_RAD:.0f} ticks'
                for name, position in zip(('pan', 'tilt'), self.position))
            if attempt == HOME_ATTEMPTS:
                raise ValueError(
                    f'An axis moved away from home after {attempt} attempts '
                    f'({details}; tolerance=8 ticks)')
            self.get_logger().warning(
                f'Home drift detected ({details}); '
                f'retrying homing ({attempt + 1}/{HOME_ATTEMPTS})')

    def home_axes(self):
        for axis in (1, 0):  # tilt, then pan
            self.send('x')
            name = ('pan', 'tilt')[axis]
            started = next_log = time.monotonic()
            deadline = started + HOME_TIMEOUT
            initial = closest = self.position[axis]
            last_speed = 0.0
            settled = previous = None
            while True:
                now, position = time.monotonic(), self.position[axis]
                if abs(position) < abs(closest): closest = position
                if now >= deadline:
                    raise ValueError(
                        f'Homing timed out: axis={name}, '
                        f'elapsed={now - started:.1f}s, '
                        f'start={initial / TICK_RAD:.0f} ticks, '
                        f'position={position / TICK_RAD:.0f} ticks, '
                        f'closest={closest / TICK_RAD:.0f} ticks, '
                        f'last_command={last_speed:.6f} rad/s, '
                        f'pan={self.position[0] / TICK_RAD:.0f} ticks, '
                        f'tilt={self.position[1] / TICK_RAD:.0f} ticks; '
                        'required: approach within 4 ticks, then stay '
                        'within 8 ticks and settle for 0.2s')
                speed = [0.0, 0.0]
                # Stop inside the acceptance band, leaving room for feedback
                # jitter. Once stopped, use the wider band to avoid chatter.
                tolerance = (HOME_APPROACH_TOLERANCE if settled is None
                             else HOME_TOLERANCE)
                if abs(position) > tolerance:
                    settled = None
                    rate = min(HOME_SPEED, max(0.03, abs(position)))
                    speed[axis] = math.copysign(rate, -position)
                else:
                    if (settled is None or previous is None or
                            abs(position - previous) > TICK_RAD):
                        settled = now
                    if now - settled >= SETTLE_TIME: break
                self.send(f'v {speed[0]:.6f} {speed[1]:.6f}')
                last_speed = speed[axis]
                if now >= next_log:
                    self.get_logger().info(
                        f'Homing {name}: elapsed={now - started:.1f}s, '
                        f'position={position / TICK_RAD:.0f} ticks, '
                        f'command={last_speed:.6f} rad/s')
                    next_log = now + 2.0
                previous = position
                yield True  # wait for another measured position
            self.send('x')

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
            self.get_logger().error(f'{error}; bridge stopped')

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
