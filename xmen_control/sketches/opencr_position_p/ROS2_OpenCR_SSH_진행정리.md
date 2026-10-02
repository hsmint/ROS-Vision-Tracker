# ROS 2 · Raspberry Pi · OpenCR 진행 정리

## 1. 목표와 작업 구조

수평 1축 객체 추적 시스템에서 제어를 담당한다. 목표는 인지 담당이 보내는 목표 오차를 받아 OpenCR과 Dynamixel을 통해 카메라를 회전시키는 것이다.

```text
카메라 → 인지 → /target → 제어 프로그램 → OpenCR → Dynamixel → 카메라 회전

PC → SSH → Raspberry Pi → USB / Serial → OpenCR → Dynamixel
```

열어둔 Notion 메모에서는 OpenCR 빌드·업로드·시리얼 수신을 Raspberry Pi에서 수행하고, PC는 SSH 접속과 결과 확인에 사용하도록 안내했다. 이 정리는 대화와 화면에서 읽은 메모를 기준으로 하며, 메모에서 언급한 원본 PDF는 직접 확인하지 않았다.

## 2. 현재까지 확인한 내용

| 항목 | 확인 결과 | 의미 |
| --- | --- | --- |
| SSH | 사용자가 연결 완료했다고 확인 | 현재 SSH 터미널에서 작업 중 |
| 호스트 이름 | `pa06` | 명령 출력으로 확인 |
| 운영체제 | Ubuntu 22.04.5 LTS / Jammy | 명령 출력으로 확인 |
| CPU 아키텍처 | `aarch64` | ARM 64비트 환경 |
| ROS 2를 받은 위치 | PC | 사용자가 답변함. 다운로드와 설치 완료 여부는 구분 필요 |
| SSH 대상의 ROS 기본 경로 | `/opt/ros` 없음 | 일반적인 설치 경로가 없음. 다른 경로의 설치 여부는 미확인 |
| ROS 환경 변수 | `ROS_DISTRO` 출력 없음 | 현재 셸에 ROS 배포판 환경이 확인되지 않음 |
| 시리얼 포트 | `/dev/ttyACM0` | OpenCR 포트로 확인 |
| USB 장치 제조사 | `ROBOTIS` | 장치 속성으로 확인 |
| USB 장치 모델 | `OpenCR_Virtual_ComPort_in_FS_Mode` | OpenCR이며, Dynamixel 모델명은 아님 |
| 사용자 그룹 | `dialout` 포함 | 포트의 그룹 접근 권한을 보유 |
| 모터 연결·외부 전원·펌웨어 업로드 | 사용자가 이전에 모두 했다고 확인 | 현재 동작 상태와 업로드한 코드 내용은 아직 미확인 |

`aarch64`와 호스트 이름만으로 Raspberry Pi 모델을 특정할 수는 없다. 대화에서는 SSH 대상을 Raspberry Pi로 두고 진행했다.

## 3. 실행한 명령과 결과

### 기본 환경 확인

```bash
hostname
cat /etc/os-release
printenv ROS_DISTRO
ls -l /dev/ttyACM* /dev/ttyUSB*
```

주요 출력:

```text
pa06
PRETTY_NAME="Ubuntu 22.04.5 LTS"
VERSION_ID="22.04"
VERSION_CODENAME=jammy

ls: cannot access '/dev/ttyUSB*': No such file or directory
crw-rw---- 1 root dialout 166, 0 Jul 29 22:41 /dev/ttyACM0
```

`printenv ROS_DISTRO`는 출력이 없었다. `/dev/ttyUSB*`가 없어도 OpenCR이 `/dev/ttyACM0`로 인식되었으므로, 이 메시지만으로 연결 실패를 뜻하지는 않는다.

### ROS 경로·아키텍처·권한·장치 확인

```bash
ls /opt/ros
uname -m
id -nG
udevadm info --query=property --name=/dev/ttyACM0 | grep -E 'ID_VENDOR=|ID_MODEL='
```

출력:

```text
ls: cannot access '/opt/ros': No such file or directory
aarch64
pa06 adm dialout cdrom sudo audio video plugdev games users input render netdev gpio spi i2c
ID_VENDOR=ROBOTIS
ID_MODEL=OpenCR_Virtual_ComPort_in_FS_Mode
```

## 4. 지금 알 수 있는 것과 아직 모르는 것

### 확인 완료

- SSH 대상에서 OpenCR USB 장치가 인식된다.
- `/dev/ttyACM0`의 소유 그룹은 `dialout`이고, 현재 사용자도 해당 그룹에 속한다.
- ROS 2는 PC에 받았다고 사용자가 확인했다. PC의 설치 완료 여부와 배포판은 아직 확인하지 않았다.
- 사용자는 모터 연결, 외부 전원 연결, OpenCR 프로그램 업로드를 이전에 완료했다고 답했다.

### 아직 확인하지 않은 것

- OpenCR에 실제로 올라간 프로그램의 이름과 코드
- Dynamixel 모델명, ID, baud rate, protocol
- 현재 모터 전원 및 통신의 실제 정상 동작 여부
- 모터 회전 방향과 허용 회전 범위
- Raspberry Pi의 다른 경로에 ROS 2가 설치되어 있는지 여부

OpenCR의 USB 장치 정보만으로는 연결된 Dynamixel 모델을 알 수 없다. SSH를 통해 모터 정보를 조회하려면, OpenCR의 기존 펌웨어가 어떤 방식으로 명령을 받아 모터와 통신하는지 먼저 확인해야 한다. 모터 모델 번호를 조회할 수 있는지는 그 펌웨어와 통신 설정에 달려 있다.

## 5. 바로 다음에 할 일

1. 이전에 OpenCR에 올린 프로그램 이름이나 소스 코드를 찾는다.
   - 예: `usb_to_dxl`, 실습 예제, 직접 만든 코드
   - 이름을 모르면 업로드할 때 사용한 명령어 또는 참고 페이지를 확인한다.
2. 기존 프로그램의 통신 방식과 설정을 확인한다.
3. 그 방식에 맞춰 모터를 움직이지 않는 조회로 ID와 모델 번호를 확인한다.
4. 모델에 맞는 전원·protocol·baud rate·동작 모드를 확인한 뒤 작은 움직임을 시험한다.
5. 실제 +/− 회전 방향과 허용 범위를 기록한다.

현재 대화에서는 기존 펌웨어 확인을 요청한 단계에서 멈췄다. 새 펌웨어 업로드, 모터 스캔, 모터 구동, ROS 2 설치 변경은 수행하지 않았다. 기존 업로드를 처음부터 다시 해야 한다고 결정한 것도 아니다.

## 6. 이후 제어 구현 순서 — Notion 메모 기준

아래는 앞으로 할 계획이며, 완료한 작업이 아니다.

1. OpenCR + Dynamixel 단독 동작 및 회전 방향 확인
2. 인지 담당과 `/target` 규약 합의
   - 메시지: `geometry_msgs/msg/PointStamped`
   - `point.x`: 수평 정규화 오차
   - `point.z = 0`: 목표 없음 → 정지
   - 정상 입력이 끊기면 기본 0.5초 timeout 후 정지
3. 모터 구동 없이 가짜 `/target` 입력으로 시험
   - `x=0, z>0`: 정지
   - `x=+0.4, z>0`: 한 방향 명령
   - `x=-0.4, z>0`: 반대 방향 명령
   - `z=0`: 정지
   - 발행 중단: 0.5초 후 정지
4. P 제어 구현
   ```text
   command = clamp(direction × Kp × ex, -speed_limit, +speed_limit)
   ```
   `direction`은 실제 동작을 확인해 +1 또는 -1로 정한다.
5. 속도 제한, 각도 제한, 중심 deadband 적용
6. 실제 `/target → 제어 → OpenCR → Dynamixel` 연결
7. `IDLE / TRACKING / LOST` 상태 처리 및 재검출 조건 구현
8. Raspberry Pi와 OpenCR 사이의 통신이 끊겨도 정지하도록 보드 측 timeout 구현
9. Kp별 반복 실험 및 `time / ex / command / state` CSV 로그 비교

## 7. 참고 페이지

- [현재 작업 중인 Notion 제어 메모](https://app.notion.com/p/3eb7969693958019b682d3b64fcfd8a4?v=2be796969395826db80b88aca747a180&p=3ec7969693958090b542ce68187e9ced&pm=s)
