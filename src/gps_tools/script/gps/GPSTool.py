#!/usr/bin/env python3

import math

class LLA:
    def __init__(self, lat=0.0, lon=0.0, alt=0.0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class ECEF:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class ENU:
    def __init__(self, e=0.0, n=0.0, u=0.0):
        self.e = e
        self.n = n
        self.u = u

def lla2ecef(lla):
    ecef = ECEF(0, 0, 0)
    A_EARTH = 6378137.0
    flattening = 1.0/298.257223563
    NAV_E2 = (2.0-flattening)*flattening
    slat = math.sin(lla.lat)
    clat = math.cos(lla.lat)
    r_n = A_EARTH/math.sqrt(1.0 - NAV_E2*slat*slat)
    ecef.x = (r_n + lla.alt)*clat*math.cos(lla.lon)
    ecef.y = (r_n + lla.alt)*clat*math.sin(lla.lon)
    ecef.z = (r_n*(1.0 - NAV_E2) + lla.alt)*slat
    return ecef

def ecef2enu(ecef, ref_ecef, ref_lla):
    enu = ENU(0, 0, 0)
    diff = [0.0] * 3
    diff[0] = ecef.x - ref_ecef.x
    diff[1] = ecef.y - ref_ecef.y
    diff[2] = ecef.z - ref_ecef.z
    lon_rad = math.pi/2.0 + ref_lla.lon
    lat_rad = math.pi/2.0 - ref_lla.lat
    cos_lon = math.cos(lon_rad)
    sin_lon = math.sin(lon_rad)
    cos_lat = math.cos(lat_rad)
    sin_lat = math.sin(lat_rad)
    enu.e = cos_lon*diff[0] + sin_lon*diff[1]
    enu.n = -sin_lon*cos_lat*diff[0] + cos_lon*cos_lat*diff[1] + sin_lat*diff[2]
    enu.u = sin_lon*sin_lat*diff[0] - sin_lat*cos_lon*diff[1] + cos_lat*diff[2]
    return enu


def data_extraction(data: str):
    theta = 0
    velocity = 0
    result_lla = LLA(0, 0, 0)
    data_split = data.split('\n')

    # GPGGA
    gpgga = next((x for x in data_split if x.startswith('$GPGGA')), None)
    if gpgga:
        pos = [i for i, char in enumerate(gpgga) if char == ',']
        B = float(gpgga[pos[1] + 1:pos[2]])
        L = float(gpgga[pos[3] + 1:pos[4]])
        altitude = float(gpgga[pos[8] + 1:pos[9]])
        latitude = int(B / 100) + ((B % 100) / 60.0)
        longitude = int(L / 100) + ((L % 100) / 60.0)
        result_lla.alt = altitude
        result_lla.lat = latitude * math.pi / 180
        result_lla.lon = longitude * math.pi / 180

    # GPVTG
    gpvtg = next((x for x in data_split if x.startswith('$GPVTG')), None)
    if gpvtg:
        pos = [i for i, char in enumerate(gpvtg) if char == ',']
        theta = float(gpvtg[pos[0] + 1:pos[1]])
        velocity = float(gpvtg[pos[6] + 1:pos[7]])

    # HEADINGA
    heading = next((x for x in data_split if x.startswith('#HEADINGA')), None)
    if heading:
        pos = [i for i, char in enumerate(heading) if char == ',']
        baseline = float(heading[pos[1] + 1:pos[2]])
        theta_2 = float(heading[pos[2] + 1:pos[3]])
        pitch_angle = float(heading[pos[3] + 1:pos[4]])
        deviation_sailing = float(heading[pos[5] + 1:pos[6]])
        deviation_pitch = float(heading[pos[6] + 1:pos[7]])

        # 航向转为从东，逆时针 0 ~ 360
        theta_2 = -theta_2 + 90 if 0 <= theta_2 <= 90 else - theta_2 + 450
        is_have_shuangtianxian = baseline != 0.0 or theta_2 != 0.0 or pitch_angle != 0.0

    return result_lla, theta, velocity
