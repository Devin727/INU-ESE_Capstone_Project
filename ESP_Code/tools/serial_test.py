"""USB 시리얼 명령 전송과 상태 CSV 기록 도구."""

import argparse
import csv
import queue
import threading
import time
from pathlib import Path


# 명령줄 옵션을 읽는 함수
def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="예: COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", type=Path, help="STATE 행을 저장할 CSV 파일")
    parser.add_argument("--heartbeat-ms", type=int, default=0,
                        help="PING 전송 간격. 기본값 0은 자동 heartbeat 없음")
    return parser.parse_args()


# 콘솔에서 받은 명령을 전송 대기 큐에 넣는 함수
def read_console(commands):
    while True:
        try:
            command = input()
        except EOFError:
            commands.put("QUIT")
            return
        commands.put(command)
        if command.strip().upper() == "QUIT":
            return


# 상태 행을 CSV에 저장할 필드로 변환하는 함수
def parse_state(line):
    if not line.startswith("STATE "):
        return None
    fields = {}
    for token in line.split()[1:]:
        if "=" not in token:
            return None
        key, value = token.split("=", 1)
        fields[key] = value
    return fields


# 명령 송신과 상태 수신 및 선택한 CSV 기록을 실행하는 함수
def main():
    args = parse_args()
    if args.baud <= 0 or args.heartbeat_ms < 0:
        raise SystemExit("baud는 양수, heartbeat-ms는 0 이상이어야 합니다.")
    try:
        import serial
    except ImportError:
        raise SystemExit("py -m pip install -r tools/requirements.txt 를 먼저 실행하세요.")

    columns = ["pc_monotonic_s", "id", "ms", "mode", "target_pwm", "target_rpm",
               "count", "rpm", "out", "in1", "in2", "fault", "rpm_valid"]
    commands = queue.Queue()
    # 기존 로그 파일을 덮어쓰지 않도록 새 파일만 생성한다.
    csv_file = args.csv.open("x", newline="", encoding="utf-8") if args.csv else None
    try:
        writer = csv.DictWriter(csv_file, fieldnames=columns, extrasaction="ignore") if csv_file else None
        if writer:
            writer.writeheader()
        with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=0.2) as connection:
            print("PWM 0 10 / RPM 0 60 / STOP / STATUS / PING / QUIT")
            print("자동 heartbeat:", args.heartbeat_ms, "ms (0이면 비활성)")
            threading.Thread(target=read_console, args=(commands,), daemon=True).start()
            pending = bytearray()
            last_heartbeat = time.monotonic()
            try:
                while True:
                    try:
                        command = commands.get_nowait().strip()
                    except queue.Empty:
                        command = None
                    if command == "QUIT":
                        break
                    if command:
                        try:
                            payload = (command + "\n").encode("ascii")
                        except UnicodeEncodeError:
                            print("명령은 ASCII 문자로 입력하세요.")
                        else:
                            if len(payload) > 96:
                                print("명령이 너무 깁니다.")
                            else:
                                connection.write(payload)
                    now = time.monotonic()
                    if args.heartbeat_ms and (now - last_heartbeat) * 1000 >= args.heartbeat_ms:
                        connection.write(b"PING\n")
                        last_heartbeat = now
                    pending.extend(connection.read(256))
                    while b"\n" in pending:
                        raw, _, rest = pending.partition(b"\n")
                        pending = bytearray(rest)
                        line = raw.decode("ascii", errors="replace").rstrip("\r")
                        print(line)
                        fields = parse_state(line)
                        if fields is not None and writer:
                            fields["pc_monotonic_s"] = f"{now:.6f}"
                            writer.writerow(fields)
                            csv_file.flush()
                    if len(pending) > 4096:
                        pending.clear()
                        print("수신 행이 너무 길어 버퍼를 비웠습니다.")
            except KeyboardInterrupt:
                pass
            finally:
                connection.write(b"STOP\n")
                connection.flush()
    finally:
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
