"""camera_viewer — RealSense 영상 토픽을 구독해 수신 상태를 확인한다(디버깅용).

구독 (use_rgbd)
  true(기본)  /camera/camera/rgbd — realsense2_camera가 컬러·정렬 뎁스·camera_info를 한 메시지로 묶어 보낸다
              (enable_rgbd:=true). 컬러와 뎁스가 항상 같은 프레임이다.
  false       아래 세 토픽을 같은 시각(header.stamp)끼리 묶어 받는다(bag 재생 등 RGBD 토픽이 없을 때)
                /camera/camera/color/image_raw                     컬러 (rgb8 또는 bgr8)
                /camera/camera/aligned_depth_to_color/image_raw    정렬 뎁스 (16UC1, mm, 0 = 측정 실패)
                /camera/camera/color/camera_info                   K = [fx 0 cx; 0 fy cy; 0 0 1]
  RGBD를 기본으로 하는 이유: realsense2_camera 영상 토픽은 reliable·KEEP_LAST(1)이라 받는 쪽이 조금만 밀려도
  프레임이 덮어써진다. 한 Python 노드에서 컬러·뎁스를 따로 받으면 컬러 29.8 Hz, 뎁스 21.8 Hz로 짝의 21%가
  사라졌고(PC 실측), RGBD 한 토픽은 29.8 Hz로 빠짐없이 들어왔다.

출력
  1초마다 로그: 수신 FPS, 지연(현재 − 촬영 시각), 해상도·형식, 화면 중앙 뎁스, fx·fy
  show:=true면 창: 왼쪽 컬러(중앙 십자·뎁스 값), 오른쪽 뎁스 컬러맵. 마우스를 올린 픽셀의 뎁스도 표시
  RPi처럼 화면이 없으면 show:=false (로그만)
"""

import cv2
import numpy as np
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from realsense2_camera_msgs.msg import RGBD

# 영상 구독 QoS. 큰 영상(≈1 MB/장)을 best-effort로 받으면 조각 유실로 대부분 버려진다(ws_test 실측 163장 중 7장).
# realsense2_camera와 bag play는 reliable로 발행하므로 reliable로 구독한다.
RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        p = self.declare_parameter
        ns = p('camera_ns', '/camera/camera').value
        self.color_topic = p('color_topic', f'{ns}/color/image_raw').value
        self.depth_topic = p('depth_topic', f'{ns}/aligned_depth_to_color/image_raw').value
        self.info_topic = p('info_topic', f'{ns}/color/camera_info').value
        self.rgbd_topic = p('rgbd_topic', f'{ns}/rgbd').value
        self.use_rgbd = p('use_rgbd', True).value          # false = 세 토픽을 따로 받아 시각으로 묶는다
        self.use_depth = p('use_depth', True).value       # false = 컬러·camera_info만
        self.show = p('show', True).value                 # false = 창 없이 로그만(RPi)
        qos = RELIABLE if p('input_qos', 'reliable').value == 'reliable' else qos_profile_sensor_data

        self.bridge = CvBridge()
        if self.use_rgbd:
            self.create_subscription(RGBD, self.rgbd_topic, self.on_rgbd, qos)
            topics = self.rgbd_topic
        else:
            subs = [message_filters.Subscriber(self, Image, self.color_topic, qos_profile=qos),
                    message_filters.Subscriber(self, CameraInfo, self.info_topic, qos_profile=qos)]
            if self.use_depth:
                subs.append(message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=qos))
            # 컬러와 정렬 뎁스는 보통 stamp가 같다. 30 fps 간격(33 ms)보다 충분히 작은 여유만 준다
            self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.01)
            self.sync.registerCallback(self.on_frame)
            topics = f'{self.color_topic}, {self.info_topic}' + (f', {self.depth_topic}' if self.use_depth else '')

        self.frames, self.latency_ms, self.last = 0, [], None   # 1초 통계
        self.mouse = None
        self.create_timer(1.0, self.report)
        if self.show:
            cv2.namedWindow('camera_viewer')
            cv2.setMouseCallback('camera_viewer', self.on_mouse)
        self.get_logger().info(f'구독: {topics} | 창={self.show}')

    # ---------- 수신 ----------
    def on_rgbd(self, msg):
        self.on_frame(msg.rgb, msg.rgb_camera_info, msg.depth if self.use_depth else None)

    def on_frame(self, color_msg, info_msg, depth_msg=None):
        stamp = Time.from_msg(color_msg.header.stamp)
        self.latency_ms.append((self.get_clock().now() - stamp).nanoseconds / 1e6)
        self.frames += 1
        # cv_bridge가 rgb8(realsense2_camera 기본) → bgr8(OpenCV)로 바꿔 준다. 안 바꾸면 빨강·파랑이 뒤바뀐다
        bgr = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough') if depth_msg is not None else None
        self.last = dict(bgr=bgr, depth=depth, encoding=color_msg.encoding,
                         depth_encoding=depth_msg.encoding if depth_msg is not None else None,
                         fx=info_msg.k[0], fy=info_msg.k[4], frame_id=color_msg.header.frame_id)
        if self.show:
            self.draw()

    def depth_m(self, depth, u, v, encoding):
        """(u, v) 주변 5×5의 유효 뎁스 중앙값 [m]. 0(측정 실패)뿐이면 None."""
        h, w = depth.shape
        patch = depth[max(0, v - 2):min(h, v + 3), max(0, u - 2):min(w, u + 3)].astype(np.float64)
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))
        return z if encoding == '32FC1' else z / 1000.0     # 16UC1 = mm

    # ---------- 출력 ----------
    def report(self):
        if self.frames == 0:
            self.get_logger().warning('1초 동안 수신 없음 — 토픽 이름·카메라 노드 실행·QoS·ROS_DOMAIN_ID 확인',
                                      throttle_duration_sec=5.0)
            return
        L, s = self.last, ''
        h, w = L['bgr'].shape[:2]
        if L['depth'] is not None:
            z = self.depth_m(L['depth'], w // 2, h // 2, L['depth_encoding'])
            s = f" | 뎁스 {L['depth_encoding']} 중앙 Z={'측정 실패' if z is None else f'{z:.3f} m'}"
        lat = np.array(self.latency_ms)
        self.get_logger().info(
            f"{self.frames} fps | 지연 평균 {lat.mean():.1f} ms (최대 {lat.max():.1f}) | {w}×{h} {L['encoding']}"
            f" | fx={L['fx']:.1f} fy={L['fy']:.1f} | frame_id={L['frame_id']}{s}")
        self.frames, self.latency_ms = 0, []

    def on_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse = (x, y)

    def draw(self):
        L = self.last
        img = L['bgr'].copy()
        h, w = img.shape[:2]
        cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
        view = img
        if L['depth'] is not None:
            d = L['depth'].astype(np.float32)
            vis = cv2.applyColorMap(cv2.convertScaleAbs(d, alpha=255 / 3000.0), cv2.COLORMAP_JET)  # 0~3 m
            vis[L['depth'] == 0] = 0                                                              # 측정 실패 = 검정
            z = self.depth_m(L['depth'], w // 2, h // 2, L['depth_encoding'])
            cv2.putText(img, f"center Z={'-' if z is None else f'{z:.3f}m'}", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            if self.mouse:
                u, v = self.mouse[0] % w, min(self.mouse[1], h - 1)       # 오른쪽(뎁스) 위도 같은 픽셀
                zm = self.depth_m(L['depth'], u, v, L['depth_encoding'])
                cv2.circle(img, (u, v), 4, (0, 255, 255), -1)
                cv2.putText(img, f"({u},{v}) Z={'-' if zm is None else f'{zm:.3f}m'}", (8, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            view = np.hstack([img, vis])
        cv2.imshow('camera_viewer', view)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:                       # Ctrl+C가 정리 중에 한 번 더 들어와도 조용히 끝낸다
            node.destroy_node()
            cv2.destroyAllWindows()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
