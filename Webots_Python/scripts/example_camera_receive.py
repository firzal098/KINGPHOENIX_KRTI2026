#!/usr/bin/env python3

#
# An example script that receives MJPEG images from a WebotsArduVehicle 
# on UDP port 5599 and displays using OpenCV.
# Requires opencv-python (`pip3 install opencv-python`)
#

# flake8: noqa

import argparse
import cv2
import socket
import time
import numpy as np

def get_default_host():
    """Auto-detect Windows host IP if running inside WSL2, otherwise default to 127.0.0.1"""
    try:
        with open('/proc/net/route', 'r') as rf:
            for line in rf:
                fields = line.strip().split()
                if len(fields) > 2 and fields[1] == '00000000':
                    import struct
                    return socket.inet_ntoa(struct.pack('<L', int(fields[2], 16)))
    except Exception:
        pass
    return "127.0.0.1"

def get_args():
    parser = argparse.ArgumentParser()
    default_host = get_default_host()
    parser.add_argument("--host", type=str, default=default_host,
                        help=f"Host address of Webots vehicle controller (default: {default_host})")
    parser.add_argument("--port", type=int, default=5599,
                        help="Port of Webots camera stream (default: 5599)")
    return parser.parse_args()

args = get_args()

# connect to WebotsArduVehicle via UDP
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
s.settimeout(0.5)

server_address = (args.host, args.port)
print(f"Connecting to MJPEG camera stream at {server_address}...")

# Send initial handshake
s.sendto(b"ping", server_address)
last_ping_time = time.time()

while True:
    # Send periodic heartbeat so server keeps streaming
    if time.time() - last_ping_time > 1.0:
        s.sendto(b"ping", server_address)
        last_ping_time = time.time()

    try:
        data, _ = s.recvfrom(65535)
    except socket.timeout:
        continue

    if not data:
        continue

    # Decode JPEG directly
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        continue

    # display image
    
    print('showing image')
    cv2.imshow("MJPEG Stream", img)
    if cv2.waitKey(1) == ord("q"):
        break

s.close()