"""인터페이스 단일 정의. 노드·문서·검사 스크립트가 모두 이 값을 쓴다(구현과 문서 일치)."""
import os
from pathlib import Path

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


def results_dir():
    """시험 결과 저장 폴더. TRACKING_RESULTS_DIR 환경변수 > <ws>/results(--symlink-install 소스 위치) > ./results"""
    if os.environ.get('TRACKING_RESULTS_DIR'):
        return Path(os.environ['TRACKING_RESULTS_DIR'])
    # --symlink-install이면 <ws>/build/... 또는 소스 위치. 위로 올라가며 src/와 install/이 있는 ws를 찾는다.
    # ws/src/Xmen이 다른 곳을 가리키는 링크면 resolve()가 ws 밖으로 나가므로 링크 그대로의 경로를 먼저 본다
    here = Path(__file__)
    for d in [*here.absolute().parents, *here.resolve().parents]:
        if (d / 'src').is_dir() and (d / 'install').is_dir():
            return d / 'results'
    return Path.cwd() / 'results'


# 목표: 최신 값만 의미가 있으므로 재전송하지 않고(best-effort) 1개만 보관
TARGET_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                        durability=DurabilityPolicy.VOLATILE)
# 상태·명령: 유실되면 안 되는 값은 reliable, 최신 1개
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.VOLATILE)
CMD_QOS = STATE_QOS
# 영상 구독: realsense2_camera 영상은 reliable·KEEP_LAST(1)로 발행된다. 같은 reliable로 받는다.
# 구독도 reliable — 큰 영상을 best-effort로 받으면 조각 유실로 대부분 버려진다(bag 재처리 실측 163장 중 7장).
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=5,
                       durability=DurabilityPolicy.VOLATILE)

# 카메라 영상 토픽 — realsense2_camera(rs_launch.py) 기본 이름. 인지는 'rgbd' 하나만 구독한다.
# 컬러·뎁스를 따로 구독하면 한 노드에서 뎁스가 밀려 덮어써진다(컬러 29.8 Hz, 뎁스 21.8 Hz 실측).
CAMERA_TOPICS = {
    'rgbd': '/camera/camera/rgbd',          # realsense2_camera enable_rgbd:=true — 컬러·정렬 뎁스·camera_info 한 묶음
    'color': '/camera/camera/color/image_raw',                       # rgb8
    'color_info': '/camera/camera/color/camera_info',                # K = [fx 0 cx; 0 fy cy; 0 0 1]
    'aligned_depth': '/camera/camera/aligned_depth_to_color/image_raw',   # 16UC1 [mm], 0 = 측정 실패
}
# bag 재처리 출력 — 저장된 /target·/perception_status와 섞지 않는다
REPLAY_TOPICS = {'target': '/target_replay', 'perception_status': '/perception_status_replay'}

TOPICS = {
    'target': dict(
        name='/target', type='geometry_msgs/msg/PointStamped', qos=TARGET_QOS,
        publisher='target_detector', subscriber='tracking_controller',
        fields={
            'header.stamp': '원본 영상 촬영 시각 — realsense2_camera 영상 header.stamp를 그대로 복사',
            'header.frame_id': 'camera_color_optical_frame',
            'point.x': 'ex = (cx-W/2)/(W/2), 오른쪽 + [-1, 1]',
            'point.y': 'ey = (cy-H/2)/(H/2), 아래쪽 + [-1, 1]',
            'point.z': '면적비 contour_area/(W×H) (0, 1]. 0 = 미검출(x·y 무시)',
        },
        rate='영상 처리마다 1회(카메라 30 fps). 정상 영상의 미검출도 z=0으로 발행. '
             '새 영상이 없으면 발행하지 않는다(이전 영상 재발행 금지)'),
    'perception_status': dict(
        name='/perception_status', type='std_msgs/msg/String', qos=STATE_QOS,
        publisher='target_detector', subscriber='(모니터링)',
        fields={'data': 'OK | NO_TARGET | CAMERA_STALL'},
        rate='상태가 바뀔 때 + 1 Hz'),
    'gimbal_cmd': dict(
        name='/cmd_vel', type='geometry_msgs/msg/Twist', qos=CMD_QOS,
        publisher='tracking_controller', subscriber='motor_driver',
        fields={
            'angular.z': '팬(좌우) 축 각속도 [rad/s], REP-103: + = 왼쪽(반시계). '
                         'ex>0(목표가 오른쪽) → 음수(오른쪽으로 회전)',
            'angular.y': '틸트(상하) 축 각속도 [rad/s], REP-103: + = 아래(피치 다운). '
                         'ey>0(목표가 아래) → 양수(아래로 회전). 실제 모터로 부호 확인 필요',
            '나머지': '0 (사용하지 않음)',
        },
        rate='20 Hz 고정 주기. 정지 명령 = angular.z·angular.y 0.0을 계속 발행'),
    'tracking_status': dict(
        name='/tracking_status', type='std_msgs/msg/String', qos=STATE_QOS,
        publisher='tracking_controller', subscriber='(모니터링·기록)',
        fields={'data': 'WAITING | TRACKING | CENTERED | LOST | TIMEOUT | SEARCHING'},
        rate='명령과 같은 20 Hz'),
}

ACTIONS = {
    'search': dict(name='/search', type='tracking_interfaces/action/Search',
                   server='tracking_controller', client='search_client(시험) / 상위 임무 노드'),
}

# 상태 값
WAITING, TRACKING, CENTERED, LOST, TIMEOUT, SEARCHING = (
    'WAITING', 'TRACKING', 'CENTERED', 'LOST', 'TIMEOUT', 'SEARCHING')
STATES = {
    WAITING: '시작 후 신선한 입력을 아직 받지 못함 → 정지',
    TRACKING: '검출(z>0), |ex| ≥ deadband 또는 |ey| ≥ deadband_tilt → 오차를 줄이는 회전(데드밴드 안의 축은 0)',
    CENTERED: '검출(z>0), 두 축 모두 데드밴드 안 → 정지(중심에서 불필요한 회전 없음)',
    LOST: '신선한 입력의 z=0(미검출) → 정지, 이전 좌표 사용 안 함',
    TIMEOUT: '마지막 신선한 입력 수신 후 timeout 초과(토픽 침묵·카메라 정지) → 정지',
    SEARCHING: '/search 액션 실행 중 → 팬만 지정 방향 일정 속도 회전, 틸트 0',
}
