"""라즈베리파이에서 OpenCR 이동·홈·정지·조회 명령을 입력한다."""
import select
import sys
import time

import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
HELP = """X변화량 Y변화량 속도  예: 5 -5 3 (현재 위치 기준, X +5° Y -5°)
h: 홈 복귀(30°/s) | x: 정지·위치 저장 | off: 토크 해제 | p: 상태 | scan: 모터 조회
auto on/off: 보드 시작 시 홈 복귀 설정 (off면 저장 위치로 복원) | q: 정지 후 종료
명령 입력 후 Enter. Ctrl+C도 정지 후 종료."""


def send_command(board, command):
    board.write((command.strip() + "\n").encode("ascii"))
    board.flush()


def main():
    with serial.Serial(PORT, BAUDRATE, timeout=0.1, write_timeout=2) as board:
        time.sleep(2)
        # 이전에 남아 있던 입력을 끝내고 새 명령을 받는다.
        send_command(board, "")
        print(HELP, flush=True)
        pending = bytearray()
        try:
            while True:
                readable, _, _ = select.select([sys.stdin, board], [], [], 0.2)
                if board in readable:
                    pending.extend(board.read(board.in_waiting or 1))
                    while b"\n" in pending:
                        line, _, pending = pending.partition(b"\n")
                        message = line.decode("utf-8", errors="replace").strip()
                        if message:
                            print(message, flush=True)
                if sys.stdin in readable:
                    line = sys.stdin.readline()
                    if not line or line.strip() == "q":
                        break
                    command = line.strip()
                    if command == "help":
                        print(HELP, flush=True)
                    elif command:
                        try:
                            send_command(board, command)
                        except UnicodeEncodeError:
                            print("숫자 또는 안내된 영문 명령을 입력해줘.", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            # 종료 시 이동을 취소하고 현재 위치를 유지하도록 요청한다.
            send_command(board, "x")
            print("정지 명령 전송, 종료.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, OSError) as error:
        print(f"시리얼 연결 오류: {error}", file=sys.stderr)
        sys.exit(1)
