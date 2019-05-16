#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import math
import time
import socket
import struct
import numpy as np
import transforms3d
import pyrealsense2 as rs
import cv2 as cv

from utils import get_new_gps_coords
from utils import MavLinkHandler
mavlink = MavLinkHandler()

_lat = 35.6535
_lon = 139.837

amap = np.ones((600, 600, 3), np.uint8) * 255

# Draw vertical lines:
for i in np.arange(100, 501, 100):
    cv.line(amap, (i, 0), (i, 599), (0,0,0), 1)  # vertical line
    cv.line(amap, (0, i), (599, i), (0,0,0), 1)  # horizontal line

cv.imshow('Velocities (Camera coordinates)', amap)
cv.waitKey(1)

pipeline = rs.pipeline()
config = rs.config()

if len(sys.argv) > 1:
    config.enable_device(sys.argv[1])

config.enable_stream(rs.stream.pose)
#config.enable_stream(rs.stream.accel)
#config.enable_stream(rs.stream.gyro)
#config.enable_stream(rs.stream.fisheye, 1)
#config.enable_stream(rs.stream.fisheye, 2)
profile = pipeline.start(config)


SEND_UDP_PACKETS = True
data_sock = None
if SEND_UDP_PACKETS:
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind(("127.0.0.1", 0))
def sendOrientationPacket(rotMatrix):
    '''Send a UDP packet to ParaView to visualize the orientation'''
    angX = np.dot(rotMatrix, np.array([1,0,0]))
    angY = np.dot(rotMatrix, np.array([0,1,0]))
    angZ = np.dot(rotMatrix, np.array([0,0,1]))
    udpData = struct.pack('!fffffffff', angX[0], angX[1], angX[2],
                                        angY[0], angY[1], angY[2],
                                        angZ[0], angZ[1], angZ[2])
    data_sock.sendto(udpData, ("127.0.0.1", 12311))


sensor = profile.get_device().as_tm2()

t0 = None
vels = np.zeros(3)
c_rot_cam = np.array([[1, 0],
                      [0, 1]])

while True:

    frames = pipeline.wait_for_frames()

    # Fetch pose frame
    pose = frames.get_pose_frame()
    if pose:
        data = pose.get_pose_data()

        T265body = transforms3d.quaternions.quat2mat([
            data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z
        ])

        if SEND_UDP_PACKETS:
            sendOrientationPacket(T265body)

        bodyInv = T265body.T

        vels_world_coordinates = np.array([data.velocity.x, data.velocity.y, data.velocity.z])
        vels_camera_coordinates = np.dot(bodyInv, vels_world_coordinates)
        # Smoothing:
        vels = 0.6 * vels + 0.4 * vels_camera_coordinates
        #print(vels)


        ################ 
        ## Get the Euler angles
        ##    - Camera is pointed forward, looking at the horizon
        #aa = transforms3d.euler.mat2euler(T265body, axes='szxy') # looking at horizon
        #rpy_rad = np.array(aa)
        #eAngles = rpy_rad * 180/math.pi
        #print("Roll: %.01f,  Pitch: %.01f,  Yaw:  %.01f" % (-eAngles[0], eAngles[1], -eAngles[2]))
        #mavlink.send_attitude(-eAngles[0], eAngles[1], -eAngles[2])

        ################
        ## Get the Euler angles
        ##    - Camera pointed to the ground, top of camera is "forward"
        aa = transforms3d.euler.mat2euler(T265body, axes='syxy')
        rpy_rad = np.array(aa)
        eAngles = rpy_rad * 180/math.pi
        mavlink.send_attitude(eAngles[0], eAngles[1]+90, -eAngles[2])

        if t0 is None:
            t0 = time.time()
        else:
            t1 = time.time()
            tDelta = t1 - t0
            t0 = t1
            cosYaw = np.cos(-rpy_rad[2])
            sinYaw = np.sin(-rpy_rad[2])
            c_rot_cam = np.array([[ cosYaw, sinYaw],
                                  [-sinYaw, cosYaw]])
            movement = np.dot(c_rot_cam, np.array([vels[0], vels[1]])) * tDelta
            _lat, _lon = get_new_gps_coords(_lat, _lon, movement[1], movement[0])

            mavlink.send_gps(_lat, _lon)

        print("gps: %.07f, %.07f" % (_lat, _lon),)
        print("Roll: %.01f, Pitch: %.01f, Yaw: %.01f" % (eAngles[0], eAngles[1]+90, -eAngles[2]))

        img = np.copy(amap)
        xyStart = (200, 300)
        xyStop = (xyStart[0] + int(round(100*vels[0])), xyStart[1] + int(round(-100*vels[1])))
        cv.arrowedLine(img, xyStart, xyStop, (0,0,255), 2)

        xyStart = (400, 300)
        xyStop = (xyStart[0], xyStart[1] + int(round(100*vels[2])))
        cv.arrowedLine(img, xyStart, xyStop, (255,0,0), 2)

        cv.imshow('Velocities (Camera coordinates)', img)
        cv.waitKey(1)

    else:
        print(" -- pose is empty.  wtf?")


