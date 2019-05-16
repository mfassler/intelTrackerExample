
import math
import time
import socket
import struct
import numpy as np

from builtins import object
from pymavlink.dialects.v10 import ardupilotmega as mavlink1




def get_new_gps_coords(lat, lon, dlat_meters, dlon_meters):
    earthRadiusMeters = 6371.0 * 1000.0
    lat1 = lat + np.degrees(dlat_meters / earthRadiusMeters)
    lon1 = lon + np.degrees(dlon_meters / earthRadiusMeters) / np.cos(np.radians(lat))
    return (lat1, lon1)


class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


class MavLinkHandler:
    def __init__(self):
        self._BOOTUP_TIME = time.time()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        f = fifo()
        self._mav = mavlink1.MAVLink(f)

        self._last_heartbeat_time = time.time()
        self._last_attitude_time = time.time()
        self._last_gps_time = time.time()
        self._heading = 0

    def heartbeat(self):
        if (time.time() - self._last_heartbeat_time) > 1.0:
            self._last_heartbeat_time = time.time()
            ## _type, autopilot, base_mode, custom_mode, system_status, mavlink_version
            _type = 10
            autopilot = 0
            base_mode = 128 + 64
            custom_mode = 65
            system_status = 4
            mavlink_version = 1
            msg = mavlink1.MAVLink_heartbeat_message(
                _type, autopilot, base_mode, custom_mode, system_status, mavlink_version)

            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, ("127.0.0.1", 14550))

    def send_attitude(self, roll, pitch, yaw):
        self.heartbeat()
        self._heading = int(round(yaw*100))
        while self._heading < 0:
            self._heading += 36000
        if (time.time() - self._last_attitude_time) > 0.1:
            self._last_attitude_time = time.time()
            time_boot_ms = int( (time.time() - self._BOOTUP_TIME) * 1000)
            rollspeed, pitchspeed, yawspeed = 0,0,0

            msg = mavlink1.MAVLink_attitude_message(
                time_boot_ms,
                math.radians(roll), math.radians(pitch), math.radians(yaw),
                rollspeed, pitchspeed, yawspeed
            )
            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, ("127.0.0.1", 14550))

    def send_gps(self, _lat, _lon):
        self.heartbeat()
        if (time.time() - self._last_gps_time) > 0.1:
            self._last_gps_time = time.time()
            time_boot_ms = int( (time.time() - self._BOOTUP_TIME) * 1000)

            lat = int(round(_lat * 1e7))
            lon = int(round(_lon * 1e7))
            alt, relative_alt = 2750, 2757
            vx, vy, vz, hdg = 0,0,0, self._heading
            msg = mavlink1.MAVLink_global_position_int_message(
                time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg
            )
            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, ("127.0.0.1", 14550))

            time_us = int(time.time() * 1000000)
            eph, epv, vel, cog = 65535, 65535, 65535, 65535
            msg = mavlink1.MAVLink_gps_raw_int_message(
                time_us, 3, lat, lon, alt, eph, epv, vel, cog, 10
            )
            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, ("127.0.0.1", 14550))



