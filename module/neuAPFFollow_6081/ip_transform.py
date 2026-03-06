# -*- coding: utf-8 -*-
"""
Transform parameters involved in ICONS_Python functions.
"""

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import numpy as np
from math import radians, cos, sin, asin, sqrt
import math

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2020/3/1"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2020 Jialun Liu. All rights reserved."

# --------------------------------------------------------------------------- #
# GPS transformer
# --------------------------------------------------------------------------- #
"""
Transform GPS longitudes and latitudes to earth-fixed coords.
Longitude and latitude for the center path_point of FuZiMiao is [118.7650, 32.0150].
"""


def gps2coord(gps_type, cent_gps, curr_gps):
    if gps_type == "wgs84":
        gps_transformer = Gps2CoordWgs84(cent_gps, curr_gps)

    else:
        gps_transformer = None
        print("ERROR: " + "gps_type = " + str(gps_type) + " is not defined")

    return [gps_transformer.curr_x, gps_transformer.curr_y]

def xy_to_latlon(Lon1, Lat1, x1, y1, x2, y2):
    """
    根据已知的A点的经纬度和两点的平面坐标，推导B点的经纬度。

    :param Lon1: A点的经度
    :param Lat1: A点的纬度
    :param x1: A点的x平面坐标（米）
    :param y1: A点的y平面坐标（米）
    :param x2: B点的x平面坐标（米）
    :param y2: B点的y平面坐标（米）
    :return: B点的经纬度 (Lon2, Lat2)
    """
    # 地球半径，单位为米
    R = 6375100

    # 计算 A 点和 B 点之间的平面坐标差值
    dx = x2 - x1
    dy = y2 - y1

    # 将 A 点的经纬度从度转换为弧度
    Lon1_rad = math.radians(Lon1)
    Lat1_rad = math.radians(Lat1)

    # 计算 B 点的纬度增量
    dLat = dy / R  # 纬度差值，以弧度为单位
    Lat2_rad = Lat1_rad + dLat  # B 点的纬度（弧度）

    # 计算 B 点的经度增量
    # 考虑到纬度不同，经度的差异会不同，因此需要乘以 cos(Lat1)
    dLon = dx / (R * math.cos(Lat1_rad))  # 经度差值，以弧度为单位
    Lon2_rad = Lon1_rad + dLon  # B 点的经度（弧度）

    # 将 B 点的经纬度从弧度转换为度
    Lon2 = math.degrees(Lon2_rad)
    Lat2 = math.degrees(Lat2_rad)

    return Lon2, Lat2

class Gps2CoordWgs84(object):

    def __init__(self, ctr_gps, curr_gps):
        self.ctr_lon_deg = ctr_gps[0]
        self.ctr_lat_deg = ctr_gps[1]
        self.curr_lon_deg = curr_gps[0]
        self.curr_lat_deg = curr_gps[1]

        self.earth_radius_m = 6371393.0  # (m)

        self.curr_x = self.get_distance_x() * np.sign(self.curr_lat_deg - self.ctr_lat_deg)
        self.curr_y = self.get_distance_y() * np.sign(self.curr_lon_deg - self.ctr_lon_deg)

    def get_distance_x(self):
        delta_lon_x = abs(np.deg2rad(self.curr_lon_deg) - np.deg2rad(self.curr_lon_deg))
        delta_lat_x = abs(np.deg2rad(self.ctr_lat_deg) - np.deg2rad(self.curr_lat_deg))

        h_x = (np.sin(
            delta_lat_x / 2)) ** 2 + np.cos(np.deg2rad(self.ctr_lat_deg)) * np.cos(np.deg2rad(self.curr_lat_deg)) * (
                      np.sin(delta_lon_x / 2) ** 2)
        coord_x = 2.0 * self.earth_radius_m * np.arcsin(np.sqrt(h_x))

        return coord_x

    def get_distance_y(self):
        delta_lon_y = abs(np.deg2rad(self.ctr_lon_deg) - np.deg2rad(self.curr_lon_deg))
        delta_lat_y = abs(np.deg2rad(self.ctr_lat_deg) - np.deg2rad(self.ctr_lat_deg))

        h_y = (np.sin(delta_lat_y / 2)) ** 2 + np.cos(np.deg2rad(self.ctr_lat_deg)) * np.cos(
            np.deg2rad(self.ctr_lat_deg)) * (np.sin(delta_lon_y / 2)) ** 2

        coord_y = 2.0 * self.earth_radius_m * np.arcsin(np.sqrt(h_y))

        return coord_y


def coord2gps(gps_type, ctr_gps, ctr_coord, curr_coord):
    if gps_type == "wgs84":
        coord_transformer = Coord2GpsWgs84(ctr_gps, ctr_coord, curr_coord)

    else:
        coord_transformer = None
        print("ERROR: " + "gps_type = " + str(gps_type) + " is not defined")

    return [coord_transformer.curr_lon_deg, coord_transformer.curr_lat_deg]


class Coord2Gps(object):

    def __init__(self):
        pass


class Coord2GpsWgs84(Coord2Gps):

    def __init__(self, cent_gps, cent_coord, curr_coord):
        super(Coord2Gps, self).__init__()
        self.ctr_lon_deg = cent_gps[0]
        self.ctr_lat_deg = cent_gps[1]
        self.ctr_coord_x = cent_coord[0]
        self.ctr_coord_y = cent_coord[1]
        self.curr_coord_x = curr_coord[0]
        self.curr_coord_y = curr_coord[1]

        self.earth_radius_m = 6371393.0  # (m)

        self.curr_lat_deg = self.ctr_lat_deg + ((self.curr_coord_x - self.ctr_coord_x) /
                                                (self.earth_radius_m * np.pi)) * 180

        self.curr_lon_deg = self.ctr_lon_deg + (self.curr_coord_y - self.ctr_coord_y) / (
                self.earth_radius_m * np.pi * np.cos(np.deg2rad(self.curr_lat_deg))) * 180


# --------------------------------------------------------------------------- #
# Unit transformer
# --------------------------------------------------------------------------- #
def ms2kmh(speed):
    # Transform speed from m/s to km/h.
    return speed * 3600 / 1000


def kmh2ms(speed):
    # Transform speed from km/h to m/s.
    return speed * 1000 / 3600


def ms2knot(speed):
    # Transform speed from m/s to knot.
    return speed * 3600 / 1852


def knot2ms(speed):
    # Transform speed from knot to m/s.
    return speed * 1852 / 3600


def km2knot(speed):
    # Transform speed from km/h to knot.
    return speed * 1.0 / 1.852


def knot2km(speed):
    # Transform speed from knot to km.
    return speed * 1.852 / 1.0


# --------------------------------------------------------------------------- #
# Angle transformer
# --------------------------------------------------------------------------- #
def limit_angle_360(angle_in_deg):
    angle_out_deg = None
    while angle_in_deg > 360:
        angle_out_deg = angle_in_deg - 360

    while angle_in_deg <= -360:
        angle_out_deg = angle_in_deg + 360

    return angle_out_deg


def angle_within_180(angle_in_deg):
    angle_out_deg = None
    while angle_in_deg > 180:
        angle_out_deg = angle_in_deg - 360

    while angle_in_deg <= -180:
        angle_out_deg = angle_in_deg + 360

    return angle_out_deg


# --------------------------------------------------------------------------- #
# Scale transformer
# --------------------------------------------------------------------------- #
def m2f_time(vessel, time_ms):
    """
    Transform model-scale time to full-scale time.

    Args:
        vessel (class): An instance of class Vessel
        time_ms (float): Model-scale time.

    Returns:
        float: Full-scale time.
    """
    time_fs = time_ms * vessel.SC ** 0.5
    return time_fs


def m2f_frequency(vessel, frequency_ms):
    """
    Transform model-scale frequency to full-scale frequency.

    Args:
        vessel (class): An instance of class Vessel
        frequency_ms (float): Model-scale frequency.

    Returns:
        float: Full-scale frequency.
    """
    frequency_fs = frequency_ms / vessel.SC ** 0.5
    return frequency_fs


def m2f_dimension(vessel, dimension_ms):
    """
    Transform model-scale dimension to full-scale dimension.

    Args:
        vessel (class): An instance of class Vessel
        dimension_ms (float): Model-scale dimension.

    Returns:
        float: Full-scale dimension.
    """
    dimension_fs = dimension_ms * vessel.SC
    return dimension_fs


def m2f_angle(vessel, angle_ms):
    """
    Transform model-scale angle to full-scale angle.

    Args:
        vessel (class): An instance of class Vessel
        angle_ms (float): Model-scale angle.

    Returns:
        float: Full-scale angle.
    """
    angle_fs = angle_ms
    return angle_fs


def m2f_linear_speed(vessel, linear_speed_ms):
    """
    Transform model-scale linear speed to full-scale linear speed.

    Args:
        vessel (class): An instance of class Vessel
        linear_speed_ms (float): Model-scale linear speed

    Returns:
        float: Full-scale linear speed.
    """
    linear_speed_fs = linear_speed_ms * vessel.SC ** 0.5
    return linear_speed_fs


def m2f_angular_speed(vessel, angular_speed_ms):
    """
    Transform model-scale angular speed to full-scale angular speed.

    Args:
        vessel (class): An instance of class Vessel
        angular_speed_ms (float): Model-scale angular speed

    Returns:
        float: Full-scale angular speed.
    """
    angular_speed_fs = angular_speed_ms / vessel.SC ** 0.5
    return angular_speed_fs


def m2f_force(vessel, force_ms):
    """
    Transform model-scale force to full-scale force

    Args:
        vessel (class): An instance of class Vessel
        force_ms (float): Model-scale force.

    Returns:
        float: Full-scale force.
    """
    force_fs = force_ms * vessel.SC ** 3
    return force_fs


def m2f_moment(vessel, moment_ms):
    """
    Transform model-scale moment to full-scale moment

    Args:
        vessel (class): An instance of class Vessel
        moment_ms (float): Model-scale moment.

    Returns:
        float: Full-scale moment.
    """
    moment_fs = moment_ms * vessel.SC ** 4
    return moment_fs


def f2m_time(vessel, time_fs):
    """
    Transform full-scale time to model-scale time.

    Args:
        vessel (class): An instance of class Vessel
        time_fs (float): Full-scale time.

    Returns:
        float: model-scale time.
    """
    time_ms = time_fs / vessel.SC ** 0.5
    return time_ms


def f2m_frequency(vessel, frequency_fs):
    """
    Transform full-scale frequency to model-scale frequency.

    Args:
        vessel (class): An instance of class Vessel
        frequency_fs (float): Full-scale frequency.

    Returns:
        float: Full-scale frequency.
    """
    frequency_ms = frequency_fs * vessel.SC ** 0.5
    return frequency_ms


def f2m_dimension(vessel, dimension_fs):
    """
    Transform full-scale dimension to model-scale dimension.

    Args:
        vessel (class): An instance of class Vessel
        dimension_fs (float): Full-scale dimension.

    Returns:
        float: model-scale dimension.
    """
    dimension_ms = dimension_fs / vessel.SC
    return dimension_ms


def f2m_angle(angle_fs):
    """
    Transform full-scale angle to model-scale angle.

    Args:
        angle_fs (float): Full-scale angle.

    Returns:
        float: model-scale angle.
    """
    angle_ms = angle_fs
    return angle_ms


def f2m_linear_speed(vessel, linear_speed_fs):
    """
    Transform full-scale linear speed to model-scale linear speed.

    Args:
        vessel (class): An instance of class Vessel
        linear_speed_fs (float): Full-scale linear speed

    Returns:
        float: model-scale linear speed.
    """
    linear_speed_ms = linear_speed_fs / vessel.SC ** 0.5
    return linear_speed_ms


def f2m_angular_speed(vessel, angular_speed_fs):
    """
    Transform full-scale angular speed to model-scale angular speed.

    Args:
        vessel (class): An instance of class Vessel
        angular_speed_fs (float): Full-scale angular speed

    Returns:
        float: model-scale angular speed.
    """
    angular_speed_ms = angular_speed_fs * vessel.SC ** 0.5
    return angular_speed_ms


def f2m_force(vessel, force_fs):
    """
    Transform full-scale force to model-scale force

    Args:
        vessel (class): An instance of class Vessel
        force_fs (float): Full-scale force.

    Returns:
        float: model-scale force.
    """
    force_ms = force_fs / vessel.SC ** 3
    return force_ms


def f2m_moment(vessel, moment_fs):
    """
    Transform full-scale moment to model-scale moment

    Args:
        vessel (class): An instance of class Vessel
        moment_fs (float): Full-scale moment.

    Returns:
        float: model-scale moment.
    """
    moment_ms = moment_fs / vessel.SC ** 4
    return moment_ms


def hist_m2f(vessel, sim_hist_fs):
    """
    Transform full-scale history to model scale.

    Args:
        vessel (class): Scale factor of the ship
        sim_hist_fs (ndarray): Full-scale history of simulations

    Returns:
        np.array: Model-scale history of simulations
    """
    sim_hist_ms = np.array(sim_hist_fs)
    sim_hist_ms[:, vessel.hist_idx_t] = np.array(m2f_time(vessel, sim_hist_fs[:, vessel.hist_idx_t]), ndmin=2)  # t (s)
    sim_hist_ms[:, vessel.hist_idx_n] = np.array(m2f_dimension(vessel, sim_hist_fs[:, vessel.hist_idx_n]),
                                                 ndmin=2)  # n (m)
    sim_hist_ms[:, vessel.hist_idx_u] = np.array(m2f_linear_speed(vessel, sim_hist_fs[:, vessel.hist_idx_u]),
                                                 ndmin=2)  # u (m/s)
    sim_hist_ms[:, vessel.hist_idx_e] = np.array(m2f_dimension(vessel, sim_hist_fs[:, vessel.hist_idx_e]),
                                                 ndmin=2)  # e (m)
    sim_hist_ms[:, vessel.hist_idx_v] = np.array(m2f_linear_speed(vessel, sim_hist_fs[:, vessel.hist_idx_v]),
                                                 ndmin=2)  # v (m/s)
    sim_hist_ms[:, vessel.hist_idx_psi_deg] = np.array(m2f_angle(vessel, sim_hist_fs[:, vessel.hist_idx_psi_deg]),
                                                       ndmin=2)  # psi_deg (deg)
    sim_hist_ms[:, vessel.hist_idx_r_deg] = np.array(m2f_angular_speed(vessel, sim_hist_fs[:, vessel.hist_idx_r_deg]),
                                                     ndmin=2)  # r_deg (deg/s)
    sim_hist_ms[:, vessel.hist_idx_rps] = np.array(m2f_frequency(vessel, sim_hist_fs[:, vessel.hist_idx_rps]),
                                                   ndmin=2)  # rps (Hz)
    sim_hist_ms[:, vessel.hist_idx_delta_deg] = np.array(m2f_angle(vessel, sim_hist_fs[:, vessel.hist_idx_delta_deg]),
                                                         ndmin=2)  # delta_deg (deg)

    return sim_hist_ms


def hist_f2m(vessel, sim_hist_ms):
    """
    Transform model-scale history to full scale.

    Args:
        vessel (class): An instance of class Vessel
        sim_hist_ms (ndarray): Model-scale history of simulations

    Returns:
        np.array: Full-scale history of simulations
    """
    sim_hist_fs = np.array(sim_hist_ms)
    sim_hist_fs[:, vessel.hist_idx_t] = f2m_time(vessel, sim_hist_ms[:, vessel.hist_idx_t])  # t (s)
    sim_hist_fs[:, vessel.hist_idx_n] = f2m_dimension(vessel, sim_hist_ms[:, vessel.hist_idx_n])  # n (m)
    sim_hist_fs[:, vessel.hist_idx_u] = f2m_linear_speed(vessel, sim_hist_ms[:, vessel.hist_idx_u])  # u (m/s)
    sim_hist_fs[:, vessel.hist_idx_e] = f2m_dimension(vessel, sim_hist_ms[:, vessel.hist_idx_e])  # e (m)
    sim_hist_fs[:, vessel.hist_idx_v] = f2m_linear_speed(vessel, sim_hist_ms[:, vessel.hist_idx_v])  # v (m/s)
    sim_hist_fs[:, vessel.hist_idx_psi_deg] = f2m_angle(sim_hist_ms[:, vessel.hist_idx_psi_deg])  # psi_deg (deg)
    sim_hist_fs[:, vessel.hist_idx_r_deg] = f2m_angular_speed(vessel,
                                                              sim_hist_ms[:, vessel.hist_idx_r_deg])  # r_deg (deg/s)
    sim_hist_fs[:, vessel.hist_idx_rps] = f2m_frequency(vessel, sim_hist_ms[:, vessel.hist_idx_rps])  # rps (Hz)
    sim_hist_fs[:, vessel.hist_idx_delta_deg] = f2m_angle(sim_hist_ms[:, vessel.hist_idx_delta_deg])  # delta_deg (deg)

    return sim_hist_fs


def hist_rad2deg(vessel, sim_hist_rad):
    """
    Transform history from np.deg2rad to degrees.

    Args:
        vessel (class): An instance of class Vessel
        sim_hist_rad (ndarray): History in np.deg2rad.

    Returns:
        ndarray: History in degrees.
    """
    sim_hist_deg = np.array(sim_hist_rad)
    sim_hist_deg[:, vessel.hist_idx_psi_deg] = np.rad2deg(sim_hist_rad[:, vessel.hist_idx_psi_deg])  # (deg)
    sim_hist_deg[:, vessel.hist_idx_r_deg] = np.rad2deg(sim_hist_rad[:, vessel.hist_idx_r_deg])  # (deg/s)
    if vessel.n_rudder == 1 and vessel.hist_idx_delta_deg is not None:
        sim_hist_deg[:, vessel.hist_idx_delta_deg] = np.rad2deg(sim_hist_rad[:, vessel.hist_idx_delta_deg])  # (deg)
    elif vessel.n_rudder == 2 and \
            vessel.hist_idx_delta_port_deg is not None and vessel.hist_idx_delta_port_deg is not None:
        sim_hist_deg[:, vessel.hist_idx_delta_port_deg] = np.rad2deg(sim_hist_rad[:, vessel.hist_idx_delta_port_deg])
        sim_hist_deg[:, vessel.hist_idx_delta_star_deg] = np.rad2deg(sim_hist_rad[:, vessel.hist_idx_delta_star_deg])
    else:
        ValueError("vessel.n_rudder = ", str(vessel.n_rudder), "is not properly defined!")

    return sim_hist_deg


def hist_deg2rad(vessel, sim_hist_deg):
    sim_hist_rad = np.array(sim_hist_deg)
    sim_hist_rad[:, vessel.hist_idx_psi_deg] = np.deg2rad(sim_hist_deg[:, vessel.hist_idx_psi_deg])  # (rad)
    sim_hist_rad[:, vessel.hist_idx_r_deg] = np.deg2rad(sim_hist_deg[:, vessel.hist_idx_r_deg])  # (rad/s)

    if vessel.n_rudder == 1 and vessel.hist_idx_delta_deg is not None:
        sim_hist_rad[:, vessel.hist_idx_delta_deg] = np.deg2rad(sim_hist_deg[:, vessel.hist_idx_delta_deg])  # (rad)
    elif vessel.n_rudder == 2 and \
            vessel.hist_idx_delta_port_deg is not None and vessel.hist_idx_delta_port_deg is not None:
        sim_hist_rad[:, vessel.hist_idx_delta_port_deg] = np.deg2rad(sim_hist_deg[:, vessel.hist_idx_delta_port_deg])
        sim_hist_rad[:, vessel.hist_idx_delta_star_deg] = np.deg2rad(sim_hist_deg[:, vessel.hist_idx_delta_star_deg])
    else:
        ValueError("vessel.n_rudder = ", str(vessel.n_rudder), "is not properly defined!")

    return sim_hist_rad


# --------------------------------------------------------------------------- #
# Communication with hardware
# --------------------------------------------------------------------------- #
def data_16bit_8bit(value_16bit):
    str_value_16bit = "{:016b}".format(value_16bit)  # 十进制数转二进制16位数的字符串, 高位补0
    value_8bit_high = int(str_value_16bit[0:8], 2)  # 取高八位数转十进制数
    value_8bit_low = int(str_value_16bit[9:16], 2)  # 取低八位数转十进制数

    return value_8bit_high, value_8bit_low


def geodistance(lng1,lat1,lng2,lat2):
    # lng1,lat1,lng2,lat2 = (120.12802999999997,30.28708,115.86572000000001,28.7427)
    lng1, lat1, lng2, lat2 = map(radians, [float(lng1), float(lat1), float(lng2), float(lat2)]) # 经纬度转换成弧度
    dlon=lng2-lng1
    dlat=lat2-lat1

    a=sin(0/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2   # x
    b=sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(0/2)**2  # y

    distance_x=2*asin(sqrt(a))*6375.1*1000 # 地球平均半径，6371km    6365.375
    # distance_x=round(distance_x/1000,3)

    distance_y=2*asin(sqrt(b))*6375.1*1000 # 地球平均半径，6371km
    # distance_y=round(distance_y/1000,3)

    if lat2 >= 0 and lat1 >= 0:
        if abs(lat2) >= abs(lat1):
            distance_y = abs(distance_y)
        else:
            distance_y = - abs(distance_y)
    if lat2 <= 0 and lat1 <= 0:
        if abs(lat2) >= abs(lat2):
            distance_y = - abs(distance_y)
        else:
            distance_y = abs(distance_y)

    if lng2 >= 0 and lng1 >= 0:
        if abs(lng2) >= abs(lng1):
            distance_x = abs(distance_x)
        else:
            distance_x = - abs(distance_x)
    if lng2 < 0 and lng1 < 0:
        if abs(lng2) >= abs(lng1):
            distance_x = - abs(distance_x)
        else:
            distance_x = abs(distance_x)

    return distance_x, distance_y



if __name__ == '__main__':
    pass