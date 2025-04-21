import can

def send_motor_speed(left_speed, right_speed):
    # int16_t => 2바이트 little-endian, signed
    left_bytes = left_speed.to_bytes(2, byteorder='little', signed=True)
    right_bytes = right_speed.to_bytes(2, byteorder='little', signed=True)

    data = left_bytes + right_bytes  # 총 4바이트
    data += bytes([0] * (8 - len(data)))  # 총 8바이트로 패딩

    msg = can.Message(arbitration_id=0x123, data=data, is_extended_id=False)

    try:
        bus.send(msg)
        print(f"[SEND] L={left_speed}, R={right_speed}")
    except can.CanError as e:
        print(f"[ERROR] {e}")

# CAN 인터페이스 설정
bus = can.interface.Bus(channel='can0', bustype='socketcan')

print("Enter motor speeds. Format: <left> <right> (e.g. 100 150)")

while True:
    try:
        line = input(">> ").strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 2:
            print("Please enter exactly two numbers.")
            continue

        left = int(parts[0])
        right = int(parts[1])

        # int16 범위 제한
        if not (-32768 <= left <= 32767) or not (-32768 <= right <= 32767):
            print("Values must be in int16_t range (-32768 ~ 32767)")
            continue

        send_motor_speed(left, right)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        break
    except ValueError:
        print( Invalid input. Use: <int> <int>")
