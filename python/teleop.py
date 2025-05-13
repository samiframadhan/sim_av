import sys
import termios
import tty
import select  # Needed for select.select
import struct
import websocket
import time

# WebSocket server URL (adjust IP and port as needed)
WS_URL = "ws://192.168.57.19:81/"  # replace with your ESP32 IP

# Control variables (0.0 - 100.0)
steer = 50.0      # center at 50
throttle = 0.0    # start stopped
STEP = 5.0        # increment step

# Key mappings
# 'a' and 'd' for left/right steering
# 'w' and 's' for increase/decrease throttle
# 'q' to quit

def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def clamp(value, min_v=-100.0, max_v=100.0):
    return max(min_v, min(max_v, value))


def main():
    global steer, throttle
    print(f"Connecting to {WS_URL}...")
    ws = websocket.create_connection(WS_URL)
    print("Connected. Use 'w/s' to throttle, 'a/d' to steer, 'q' to quit.")

    try:
        while True:
            key = get_key()
            if key == 'a':
                steer = clamp(steer - STEP)
            elif key == 'd':
                steer = clamp(steer + STEP)
            elif key == 'w':
                throttle = clamp(throttle + STEP)
            elif key == 's':
                throttle = clamp(throttle - STEP)
            elif key == 'q':
                break
            else:
                # no change
                continue

            # Pack two floats in little-endian
            payload = struct.pack('<ff', steer, throttle)
            ws.send(payload, opcode=websocket.ABNF.OPCODE_BINARY)
            print(f"Sent -> Steer: {steer:.1f}, Throttle: {throttle:.1f}")
            time.sleep(0.05)
    finally:
        ws.close()
        print("Connection closed.")

if __name__ == '__main__':
    main()
