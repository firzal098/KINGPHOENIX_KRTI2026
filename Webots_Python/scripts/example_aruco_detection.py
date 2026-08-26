#!/usr/bin/env python3

#
# An example script that receives MJPEG images from a WebotsArduVehicle on UDP port 5599 
# and displays them overlaid with any ArUco markers using OpenCV.
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

# ArUco setup
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

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

    # detect ArUco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(img, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

    # display image
    cv2.imshow("image", img)
    if cv2.waitKey(1) == ord("q"):
        break

s.close()