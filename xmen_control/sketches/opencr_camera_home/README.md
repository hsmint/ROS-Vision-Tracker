# 카메라 X·Y축 제어

XM430-W350, Protocol 2.0, 1 Mbps. Pan ID11, tilt ID12.
Saved home: pan 4067 ticks, tilt 2116 ticks, homing offsets 0.
Angles are relative to the saved home, not relative to each command's starting position.

## 두 축 동시 입력

X축은 pan ID11, Y축은 tilt ID12입니다. 각도는 저장된 홈 기준이며, 속도 단위는 °/s입니다.

```text
move X각도 Y각도 X속도 Y속도
```

```text
move 30 -10 5 3
```

X축을 홈 +30°로 최대 5°/s, Y축을 홈 -10°로 최대 3°/s로 이동합니다.
두 모터에 하나의 Sync Write 패킷으로 목표와 속도를 전달해 함께 출발합니다.
이동 거리와 축별 속도에 따라 도착 시간은 달라질 수 있습니다.

| 항목 | 입력 범위 |
|---|---|
| X각도 | -90..90° |
| Y각도 | -135..135° |
| X속도·Y속도 | 1.374..30°/s, 각 모터의 속도 한계 이내 |

속도는 모터 단위로 내림 처리되며, 적용된 프로파일 속도는 `MOVING XY`에 표시됩니다.
잘못된 입력은 전체를 거부합니다. 두 축 모두 도착해야 `DONE XY`가 출력되고 토크는 유지됩니다.
`speed` 명령은 기존 단일 축 이동과 순차 홈 복귀에 적용됩니다. `move`는 입력한 축별 속도를 사용합니다.

## 시리얼 연결

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200 --eol LF -e
```

| Command | Meaning |
|---|---|
| `move 30 -10 5 3` | X·Y축의 각도와 속도를 한 번에 입력해 동시 이동 |
| `pan 30` | ID11 to home +30 degrees; positive was observed to turn left |
| `tilt -10` | ID12 to home -10 degrees; physical sign must be observed |
| `speed 5` | Future moves use up to 5 degrees/s (motor resolution rounds down) |
| `h` | Tilt home first, then pan home |
| `x` | Immediate input handling; stop at current position and retain torque |
| `off` | Release both motors; support the camera before entering |
| `p` | State, position, torque and motor hardware error |
| `auto on` | Save automatic homing on board startup in OpenCR EEPROM |
| `auto off` | Save waiting for commands on startup |


## Completion
Move completion does not disable torque. 

## Commands
- Commands other than x are line-based.
- Backspace is supported. 
- Send x before changing commands during movement.

## Default
- Default speed is 5 deg/s. 
- Speed setting range is 1.374..30 deg/s and the motor's configured velocity limit is checked.
- Pan targets: -90..90 deg. Tilt targets: -135..135 deg. - These are software test ranges, not measured mechanical clearances.

## How code works
- On first activation, current position must lie within the same home range.
- After off, the next movement rereads and establishes the home branch.
- Return follows the calibrated home branch across the 0/4096 encoder boundary.

## Warning
- Do not rotate the camera multiple full turns manually: the stored single-turn home cannot identify cable winding.

## Apparatus set up
- First use: keep camera supported, ensure cable slack throughout the route, enter p, then h.
- Only enable auto on after observing a successful complete return. Automatic homing happens on OpenCR firmware startup; restoring only motor power while the board remains running does not itself restart homing.
- Communication/hardware faults latch; p reports the fault. Stop and inspect before restarting the board.
If holding fails, firmware requests torque off; camera support may be necessary.

## Model
The controller uses extended position mode and motor internal position PID, unlike the original OpenCR P-control experiment. It is a camera positioning tool, not evidence for the external-P-control exercise.

## 검증 범위

동시 이동 기능은 OpenCR용 빌드와 입력 파서 검사를 수행했습니다.
보드 업로드와 실제 두 모터의 동시 이동은 아직 확인하지 않았습니다.
