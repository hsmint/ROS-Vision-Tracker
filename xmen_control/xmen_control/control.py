#!/usr/bin/env python3
"""ROS 2 serial bridge for opencr_control.ino; run after sourcing ROS.

Requires rclpy, geometry_msgs, sensor_msgs, std_msgs and pyserial.
Run: ros2 run xmen_control control --ros-args -p port:=/dev/ttyACM0
Auto-home is enabled by default. Disable at node startup with:
  ros2 run xmen_control control --ros-args -p auto_home:=false
With auto_home enabled, the node waits for verified firmware and an idle board,
then sends h once and blocks /cmd_vel until DONE HOME. Existing boot motion is
allowed to finish first. /opencr/stop cancels pending or running startup homing.
The parameter also saves the firmware's auto-home setting for future board boots.

Command angular velocity in radians/second:
  angular.z -> pan, motor ID11; angular.y -> tilt, motor ID12.
  linear.* and angular.x are ignored. Publish continuously (e.g. 10 Hz):
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    '{angular: {z: 0.3, y: -0.2}}'
  A zero Twist stops both axes; commands expire after 0.5 seconds.
  Firmware keeps the existing pan/tilt travel limits and holds at each boundary.
  Speeds below one motor unit (~0.02398 rad/s) are treated as zero.
Read measured position (radians) and velocity (radians/s):
  ros2 topic echo /joint_states
Read firmware responses, including rejection/fault messages:
  ros2 topic echo /status
Home immediately (stops any current motion first):
  ros2 topic pub --once /opencr/home std_msgs/msg/Empty '{}'
Stop and hold:
  ros2 topic pub --once /opencr/stop std_msgs/msg/Empty '{}'

Requires OPENCR_CONTROL_V3_RAD_VELOCITY firmware.
The firmware rejects nonzero velocity commands during home/restore moves.
Only this process should open the serial port. Opening it may reset the board
and trigger the firmware's configured startup home/restore behavior.
Normal node shutdown requests a stop; firmware also has a command timeout.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String


def parse_axis(line):
    """Return motor ID, position radians, velocity radians/s or None."""
    parts = line.split()
    if len(parts) < 3 or parts[0] not in ('pan', 'tilt'):
        return None
    if parts[1] not in ('ID11', 'ID12'):
        return None
    fields = dict(part.split('=', 1) for part in parts[2:] if '=' in part)
    try:
        position = float(fields['home_rad'])
        velocity = float(fields['velocity_rad_s'])
    except (KeyError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (position, velocity)):
        return None
    return int(parts[1][2:]), position, velocity


class OpenCRBridge(Node):
    def __init__(self):
        super().__init__('control')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.auto_home = self.declare_parameter(
            'auto_home', True,
            ParameterDescriptor(
                read_only=True,
                description='Home on node startup and save automatic homing for board startup.'),
        ).value
        self.auto_home_sent = False
        self.startup_home = 'waiting' if self.auto_home else 'done'
        self.firmware_state = None
        self.firmware_ready = False
        self.pending_velocity = None
        self.last_velocity_received = None
        self.velocity_command_active = False
        self.next_velocity_write = 0.0
        self.positions = self.create_publisher(JointState, '/joint_states', 10)
        self.status = self.create_publisher(String, '/status', 10)
        self.move_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.move, 1)
        self.stop_subscription = self.create_subscription(
            Empty, '/opencr/stop', self.stop, 1)
        self.home_subscription = self.create_subscription(
            Empty, '/opencr/home', self.home, 1)
        self.board = serial.Serial(
            port, 115200, timeout=0, write_timeout=0.2, exclusive=True)
        self.buffer = bytearray()
        self.axes = {}
        self.collecting = False
        # Firmware setup waits two seconds; leave additional startup margin.
        self.ready_at = time.monotonic() + 3.0
        self.next_poll = self.ready_at
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f'Connected to {port}; waiting 3 s before commands.')

    def send(self, command):
        # A failed write disconnects; do not replay old commands after reconnect.
        payload = (command + '\n').encode('ascii')
        try:
            if self.board.write(payload) != len(payload):
                raise serial.SerialException('Incomplete serial write')
        except (serial.SerialException, OSError) as error:
            self.disconnect(error)

    def disconnect(self, error):
        self.get_logger().error(f'Serial disconnected: {error}; restart this node.')
        self.timer.cancel()
        self.board.close()

    def move(self, message):
        if not self.board.is_open or time.monotonic() < self.ready_at:
            self.get_logger().warning('Serial is not ready; command discarded.')
            return
        if not self.firmware_ready:
            self.get_logger().warning('Waiting for V3 rad/s firmware status; command discarded.')
            return
        if self.startup_home not in ('done', 'cancelled'):
            # Do not queue joystick commands (including zero) during startup home.
            return
        pan, tilt = message.angular.z, message.angular.y
        max_rad_s = math.radians(1023 * 1.374)
        if not all(math.isfinite(v) and abs(v) <= max_rad_s for v in (pan, tilt)):
            self.get_logger().warning('Invalid angular velocity; stopping both axes.')
            self.stop(None)
            return
        # Keep only the latest ROS command; cap serial command traffic at 20 Hz.
        self.pending_velocity = (pan, tilt)
        self.last_velocity_received = time.monotonic()

    def service_velocity(self, now):
        if self.last_velocity_received is None:
            return
        if now - self.last_velocity_received >= 0.5:
            if self.velocity_command_active or self.pending_velocity is not None:
                self.send('v 0 0')
            self.pending_velocity = None
            self.velocity_command_active = False
            self.last_velocity_received = None
        elif self.pending_velocity is not None and now >= self.next_velocity_write:
            pan, tilt = self.pending_velocity
            # The firmware parser requires fixed decimal notation, not exponents.
            self.send(f'v {pan:.6f} {tilt:.6f}')
            self.pending_velocity = None
            self.velocity_command_active = pan != 0.0 or tilt != 0.0
            self.next_velocity_write = now + 0.05

    def stop(self, _message):
        self.startup_home = 'cancelled'
        self.pending_velocity = None
        self.last_velocity_received = None
        self.velocity_command_active = False
        if self.board.is_open:
            self.send('x')

    def home(self, _message):
        if (not self.board.is_open or time.monotonic() < self.ready_at
                or not self.firmware_ready):
            self.get_logger().warning('Home discarded: wait for the firmware connection, then retry.')
            return
        # x cancels any existing move; h starts firmware's Y-then-X home sequence.
        self.stop(None)
        if not self.board.is_open:
            return
        self.startup_home = 'homing'
        self.send('h')
        self.get_logger().info('Home requested: ID12 then ID11; /cmd_vel paused until DONE HOME.')

    def service_startup(self, now):
        if now < self.ready_at or not self.firmware_ready:
            return
        if not self.auto_home_sent:
            self.send('auto on' if self.auto_home else 'auto off')
            if not self.board.is_open:
                return
            self.auto_home_sent = True
        if self.startup_home == 'waiting' and self.firmware_state == 'IDLE':
            self.pending_velocity = None
            self.last_velocity_received = None
            self.velocity_command_active = False
            self.startup_home = 'homing'
            self.send('h')
            self.get_logger().info('Startup homing requested: ID12 then ID11; /cmd_vel paused.')

    def receive_line(self, line):
        response = String()
        response.data = line
        self.status.publish(response)
        if line.startswith('STATE='):
            self.firmware_state = line.split()[0].split('=', 1)[1]
        if self.startup_home in ('waiting', 'homing'):
            if line.startswith('FAULT:') or line.startswith('STATE=FAULT'):
                self.startup_home = 'failed'
                self.get_logger().error('Homing blocked by firmware fault; inspect /status.')
            elif self.startup_home == 'homing':
                if line.startswith('DONE HOME:'):
                    self.startup_home = 'done'
                    self.get_logger().info('Homing complete; /cmd_vel enabled.')
                elif line.startswith(('REJECTED:', 'BUSY:')):
                    self.startup_home = 'failed'
                    self.get_logger().error('Homing was rejected; inspect /status.')
        if line.startswith(('FAULT:', 'REJECTED:', 'BUSY:', 'WARNING:')):
            self.get_logger().warning(line)
        # Start a new p response; never combine axes from different samples.
        if line.startswith('FIRMWARE='):
            self.firmware_ready = line == 'FIRMWARE=OPENCR_CONTROL_V3_RAD_VELOCITY'
            if not self.firmware_ready:
                self.pending_velocity = None
                self.last_velocity_received = None
                self.velocity_command_active = False
                self.get_logger().error('Upload the V3 rad/s sketch before sending /cmd_vel.')
            self.axes.clear()
            self.collecting = True
        axis = parse_axis(line)
        if self.collecting and axis is not None:
            self.axes[axis[0]] = axis[1:]
        # Firmware always prints tilt ID12 followed by pan ID11, even on errors.
        if self.collecting and line.startswith('pan ID11 '):
            self.collecting = False
            if 11 in self.axes and 12 in self.axes:
                state = JointState()
                state.header.stamp = self.get_clock().now().to_msg()
                state.name = ['pan', 'tilt']
                state.position = [self.axes[i][0] for i in (11, 12)]
                state.velocity = [self.axes[i][1] for i in (11, 12)]
                self.positions.publish(state)

    def tick(self):
        try:
            self.buffer.extend(self.board.read(min(self.board.in_waiting, 4096)))
        except (serial.SerialException, OSError) as error:
            self.disconnect(error)
            return
        while b'\n' in self.buffer:
            raw, _, remaining = self.buffer.partition(b'\n')
            self.buffer = bytearray(remaining)
            self.receive_line(raw.decode('ascii', errors='replace').strip())
        if len(self.buffer) > 8192:
            self.buffer.clear()
            self.collecting = False
            self.get_logger().warning('Discarded oversized serial line.')
        now = time.monotonic()
        self.service_startup(now)
        if not self.board.is_open:
            return
        self.service_velocity(now)
        if not self.board.is_open:
            return
        if now >= self.next_poll:
            self.send('p')
            self.next_poll = now + 0.5  # 2 Hz position feedback


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
            if node.board.is_open:
                node.send('v 0 0')
            node.board.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
