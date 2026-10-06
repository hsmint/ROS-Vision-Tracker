"""target_detector — 영상 1장마다 검출해 /target을 1회 발행한다.

  [D435] ══USB══▶ target_detector ──▶ /target (PointStamped)
                                   └─▶ /perception_status (String)

source
  realsense (기본)  pyrealsense2로 카메라를 직접 연다. 영상을 ROS 토픽으로 내보내지 않아 복사·직렬화 비용이 없다.
                    전용 스레드가 wait_for_frames로 새 프레임을 기다린다(폴링 없음) → 뎁스를 컬러에 정렬 → 검출 → 발행.
  ros               /camera/camera/rgbd(realsense2_camera 또는 bag 재생)를 구독한다 — bag 재처리용.
검출: detector.py (HSV → 컨투어 → 후보 필터 → 뎁스 중앙값 검증 → 선택 → 모멘트 중심), 설정은 config/detector.yaml.
show:=true면 검출 화면(오버레이·mask)을 띄운다(PC 확인용, RPi는 false).

지키는 규칙
- 매 프레임 독립 판단. 미검출이면 z=0(x·y=0)으로 발행 — 이전 좌표를 쓰지 않는다.
- header.stamp = 촬영 시각(RealSense global/system 시간 도메인, 아니면 수신 시각 — 로그에 기준 표시).
- 같은 영상은 발행하지 않는다(realsense: 프레임 번호, ros: stamp).
- 새 영상이 stall_timeout 동안 없으면 CAMERA_STALL — 발행하지 않는다(미검출 z=0과 통신 중단을 구분).
"""
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from target_perception import detector
from tracking_common import interface as I


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        p = self.declare_parameter
        default_cfg = Path(get_package_share_directory('target_perception')) / 'config' / 'detector.yaml'
        cfg_path = p('config', str(default_cfg)).value
        self.source = p('source', 'realsense').value          # realsense | ros
        self.stall = p('stall_timeout', 0.3).value            # [s] 새 영상 없음 → CAMERA_STALL
        self.initial_reset = p('initial_reset', True).value   # 시작 시 카메라 하드웨어 리셋(뎁스 모듈 멈춤 방지)
        self.show = p('show', False).value                    # 검출 화면(PC 확인용)
        self.frame_id = p('frame_id', 'camera_color_optical_frame').value
        # 출력 토픽 — bag 재처리는 /target_replay·/perception_status_replay로 내서 저장된 결과와 섞지 않는다
        target_topic = p('target_topic', I.TOPICS['target']['name']).value
        status_topic = p('status_topic', I.TOPICS['perception_status']['name']).value
        self.cfg = detector.load_config(cfg_path)
        self.use_depth = detector.depth_cfg(self.cfg) is not None

        self.pub = self.create_publisher(PointStamped, target_topic, I.TARGET_QOS)
        self.status_pub = self.create_publisher(String, status_topic, I.STATE_QOS)
        self.status, self.stamp_kind = None, None
        self.last_stamp_ns = None      # ros: 마지막으로 처리한 영상 stamp
        self.last_frame_no = None      # realsense: 마지막 프레임 번호
        self.last_rx = None            # 마지막으로 새 영상을 받은 시각(노드 시계)
        self.count = {'published': 0, 'detected': 0, 'duplicate': 0}
        self.focal_set = False         # ros: 카메라 정보로 초점거리를 반영했는지
        self.running = True

        if self.source == 'realsense':
            self.open_realsense()
            self.thread = threading.Thread(target=self.realsense_loop, daemon=True)
            self.thread.start()
        elif self.source == 'ros':
            self.open_ros()
        else:
            raise ValueError(f'source={self.source} — realsense | ros')
        self.create_timer(0.05, self.check_stall)
        self.create_timer(1.0, self.heartbeat)
        self.get_logger().info(f'source={self.source} → {target_topic}, {status_topic} | config={cfg_path} '
                               f'HSV={self.cfg["target"]["hsv_ranges"]} depth={self.use_depth} show={self.show}')

    # ---------- 입력: RealSense 직접 ----------
    def open_realsense(self):
        import pyrealsense2 as rs
        self.rs = rs
        if self.initial_reset:
            devs = rs.context().query_devices()
            if len(devs) == 0:
                raise RuntimeError('RealSense 카메라가 없다 — USB3 연결 확인')
            self.get_logger().info('카메라 하드웨어 리셋…')
            devs[0].hardware_reset()
            time.sleep(2.0)                               # 끊겼다가
            for _ in range(80):                           # 다시 잡힐 때까지 최대 8초
                if len(rs.context().query_devices()) > 0:
                    break
                time.sleep(0.1)
            time.sleep(1.0)
        c = self.cfg['camera']
        W, H = self.cfg['processing']['width'], self.cfg['processing']['height']
        conf = rs.config()
        conf.enable_stream(rs.stream.color, W, H, rs.format.bgr8, c.get('fps', 30))
        if self.use_depth:
            conf.enable_stream(rs.stream.depth, W, H, rs.format.z16, c.get('fps', 30))
        self.pipe = rs.pipeline()
        try:
            profile = self.pipe.start(conf)
        except RuntimeError as e:
            raise RuntimeError(f'RealSense 시작 실패: {e} — USB3·다른 프로그램(realsense2_camera·run.py) 점유 확인') from e
        i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy = i.fx, i.fy
        if self.use_depth:
            self.align = rs.align(rs.stream.color)       # 뎁스를 컬러 픽셀 좌표로
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.apply_camera_options(profile.get_device().first_color_sensor(), c)
        self.update_focal(self.fx, i.width)
        self.get_logger().info(f'카메라 {W}×{H}@{c.get("fps", 30)} fx={self.fx:.1f} fy={self.fy:.1f}')

    def apply_camera_options(self, sensor, c):
        """exposure·white_balance에 숫자가 있으면 해당 자동 기능을 끄고 고정한다.
        camera.options = 그 밖의 컬러 센서 옵션 {이름: 값}(예: auto_exposure_priority). 지원하지 않으면 경고 후 무시."""
        rs = self.rs
        for auto, key, opt in [(rs.option.enable_auto_exposure, 'exposure', rs.option.exposure),
                               (rs.option.enable_auto_white_balance, 'white_balance', rs.option.white_balance)]:
            if c.get(key) is not None:
                sensor.set_option(auto, 0)
                sensor.set_option(opt, float(c[key]))
        for name, value in (c.get('options') or {}).items():
            opt = getattr(rs.option, name, None)
            if opt is None or not sensor.supports(opt):
                self.get_logger().warning(f'컬러 센서가 {name} 옵션을 지원하지 않는다 — 무시')
                continue
            r = sensor.get_option_range(opt)
            v = min(max(float(value), r.min), r.max)
            sensor.set_option(opt, v)
            self.get_logger().info(f'camera option {name} = {v}')

    def update_focal(self, fx, width):
        """실제 컬러 초점거리로 크기 기반 면적 기준(selection.min_area: auto 등)을 다시 계산한다.
        config의 fx_px는 근사값(640×360 ≈ 460)이다."""
        c = self.cfg['camera']
        c['fx_px'], c['fx_width'] = float(fx), int(width)
        detector.resolve_size_limits(self.cfg)
        s = self.cfg['selection']
        self.get_logger().info(f'fx={fx:.1f}px@{width} → min_area={s["min_area"]} max_area_ratio={s["max_area_ratio"]}')

    def realsense_loop(self):
        """전용 스레드: 새 프레임이 올 때까지 기다린다(폴링 없음)."""
        while self.running and rclpy.ok():
            try:
                frames = self.pipe.wait_for_frames(1000)
            except RuntimeError:
                continue                                  # 1초 동안 없음 → check_stall이 CAMERA_STALL 처리
            color = frames.get_color_frame()
            if not color:
                continue
            no = color.get_frame_number()
            if self.last_frame_no is not None and no <= self.last_frame_no:
                self.count['duplicate'] += 1
                continue
            now = self.get_clock().now()
            self.last_frame_no, self.last_rx = no, now
            stamp, kind = self.capture_stamp(color, now)
            depth = None
            if self.use_depth:
                aligned = self.align.process(frames)
                color, d = aligned.get_color_frame(), aligned.get_depth_frame()
                if d:
                    depth = detector.DepthFrame(np.asanyarray(d.get_data()), self.depth_scale, self.fx, self.fy)
            try:
                self.process(np.asanyarray(color.get_data()), stamp, kind, depth)
            except Exception as e:                        # 종료 중 발행 실패 등
                if self.running and rclpy.ok():
                    self.get_logger().error(f'처리 실패: {e}')

    def capture_stamp(self, color, now):
        """RealSense global/system time 도메인이면 촬영 시각(호스트 시계 기준)을 쓴다."""
        rs = self.rs
        dom = color.get_frame_timestamp_domain()
        if dom in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time):
            t = Time(nanoseconds=int(color.get_timestamp() * 1e6), clock_type=now.clock_type)
            age = (now - t).nanoseconds / 1e9
            if -0.05 < age < 1.0:                         # 시계 기준이 맞는지 확인
                return t, f'capture({dom.name})'
            self.get_logger().warning(f'촬영 시각이 호스트 시계와 {age:.3f}s 차이 → 수신 시각 사용',
                                      throttle_duration_sec=5.0)
        return now, f'receive(domain={dom.name})'

    # ---------- 입력: ROS 토픽 (bag 재처리) ----------
    def open_ros(self):
        from cv_bridge import CvBridge
        from realsense2_camera_msgs.msg import RGBD
        self.bridge = CvBridge()
        topic = self.declare_parameter('rgbd_topic', I.CAMERA_TOPICS['rgbd']).value
        self.create_subscription(RGBD, topic, self.on_rgbd, I.IMAGE_QOS)
        self.get_logger().info(f'구독 {topic}')

    def on_rgbd(self, msg):
        stamp = Time.from_msg(msg.rgb.header.stamp)
        if self.last_stamp_ns is not None and stamp.nanoseconds <= self.last_stamp_ns:
            self.count['duplicate'] += 1
            return
        self.last_stamp_ns, self.last_rx = stamp.nanoseconds, self.get_clock().now()
        bgr = self.bridge.imgmsg_to_cv2(msg.rgb, 'bgr8')
        k = msg.rgb_camera_info.k
        if not self.focal_set and k[0] > 0:            # 첫 영상에서 실제 초점거리로 면적 기준 재계산
            self.update_focal(k[0], msg.rgb_camera_info.width or bgr.shape[1])
            self.focal_set = True
        depth = None
        if self.use_depth:
            depth = detector.DepthFrame(self.bridge.imgmsg_to_cv2(msg.depth, 'passthrough'), 0.001, k[0], k[4])
        self.process(bgr, stamp, 'topic(header.stamp)', depth)

    # ---------- 검출·발행 ----------
    def process(self, bgr, stamp, kind, depth):
        frame, scale = detector.preprocess(bgr, self.cfg)
        d, mask = detector.detect(frame, self.cfg, scale, detector.preprocess_depth(depth, scale))
        out = PointStamped()
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.frame_id
        if d.detected:
            out.point.x, out.point.y = map(float, d.error)
            out.point.z = float(d.area_ratio)
            self.count['detected'] += 1
        # 미검출: x = y = z = 0 (기본값). z=0이면 받는 쪽은 x·y를 쓰지 않는다
        self.pub.publish(out)
        self.count['published'] += 1
        if kind != self.stamp_kind:
            self.stamp_kind = kind
            self.get_logger().info(f'header.stamp 기준: {kind}')
        self.set_status('OK' if d.detected else 'NO_TARGET')
        if self.show:
            cv2.imshow('target_detector', np.hstack([detector.draw(frame, d), cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
            cv2.waitKey(1)

    # ---------- 상태 ----------
    def check_stall(self):
        if self.last_rx is None:
            return
        gap = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
        if gap > self.stall:
            self.set_status('CAMERA_STALL', f'{gap:.2f}s 동안 새 영상 없음')

    def set_status(self, s, detail=''):
        if s != self.status:
            self.get_logger().info(f'perception {self.status} → {s} {detail}')
            self.status = s
            self.status_pub.publish(String(data=s))

    def heartbeat(self):
        if self.status:
            self.status_pub.publish(String(data=self.status))
        self.get_logger().debug(str(self.count))

    def close(self):
        self.running = False
        if self.source == 'realsense':
            self.thread.join(timeout=2.0)
            self.pipe.stop()
        if self.show:
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise                  # 종료 신호로 컨텍스트가 닫힌 직후 발행 실패만 무시
    finally:
        try:
            node.get_logger().info(f'종료 {node.count}')
            node.close()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
