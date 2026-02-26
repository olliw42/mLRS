import time
from datetime import datetime
import serial

# ------------- CONFIG -------------
PORT = "COM17"       # e.g. "COM3"
BAUD = 115200
SEND_PERIOD_S = 0.020  # 20 ms
# ----------------------------------

DEFAULT_MSG = bytes(range(64))  # 0..63

def main() -> None:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=1,
    )

    frame_id = 0
    total_sent = 0
    sent_this_second = 0
    last_rate_t = time.monotonic()

    next_send_t = time.monotonic()

    print(f"Sending raw 64-byte frames on {PORT} @ {BAUD} baud every {int(SEND_PERIOD_S*1000)} ms... (Ctrl+C to stop)")

    try:
        while True:
            now = time.monotonic()

            if now >= next_send_t:
                # schedule next tick (keeps average rate stable)
                next_send_t += SEND_PERIOD_S

                payload = bytearray(DEFAULT_MSG)
                payload[0] = frame_id & 0xFF
                frame_id = (frame_id + 1) & 0xFF

                # Ensure we always write all 64 bytes
                ser.write(payload)
                ser.flush()  # wait until bytes handed off to driver

                total_sent += 1
                sent_this_second += 1

            if (now - last_rate_t) >= 1.0:
                ts = datetime.now().isoformat(timespec="milliseconds")
                print(f"{ts} | sent_last_1s={sent_this_second} | total={total_sent}")
                sent_this_second = 0
                last_rate_t += 1.0

            # tiny sleep to reduce CPU usage while maintaining timing
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()