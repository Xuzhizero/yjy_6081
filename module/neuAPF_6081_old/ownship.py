import numpy as np
from para import unit_to_meter, ship_turn_angle, ship_safe_distance, ship_tolerance
from algo_utility import cal_distance, norm, convert_lonlat_to_abs_xy, convert_abs_xy_to_lonlat
from algo_utility import collision_detection_circle, course_check, cal_collision_angle, find_current_segment_index

# create a ship class init with ship information including lat and lon, turn angle, speed
class Ship:
    def __init__(self, own_pos_lonlat, abs_speed, heading, u2m=unit_to_meter):
        self.u2m = u2m
        self.own_pos_lonlat = own_pos_lonlat
        self.own_abs_pos_xy = convert_lonlat_to_abs_xy(own_pos_lonlat, u2m) 
        self.own_rel_pos_xy = (0, 0)
        self.start_lonlat = own_pos_lonlat
        self.start_abs_pos_xy = self.own_abs_pos_xy
        self.abs_speed = abs_speed
        self.heading = heading
        self.own_speed_uv = (abs_speed * np.sin(np.radians(heading)), abs_speed * np.cos(np.radians(heading)))
        if abs_speed > 0:
            self.moving = True
        else:
            self.moving = False

    def step_update(self, heading_delta=0, simu_time=10):     
        self.heading += heading_delta
        self.own_speed_uv = (self.abs_speed * np.sin(np.radians(self.heading)), self.abs_speed * np.cos(np.radians(self.heading)))
        self.own_abs_pos_xy = (self.own_abs_pos_xy[0] + self.own_speed_uv[0] * simu_time, self.own_abs_pos_xy[1] + self.own_speed_uv[1] * simu_time)
        self.own_rel_pos_xy = (self.own_abs_pos_xy[0] - self.start_abs_pos_xy[0], self.own_abs_pos_xy[1] - self.start_abs_pos_xy[1])
        self.own_pos_lonlat = convert_abs_xy_to_lonlat(self.own_abs_pos_xy, self.u2m)

    def set_plan_speed(self, speed):
        self.abs_speed = speed
        self.own_speed_uv = (speed * np.sin(np.radians(self.heading)), speed * np.cos(np.radians(self.heading)))

    def __str__(self):
        return ("--------------------OWN SHIP INFO--------------------\n"
                f"Ship Position (Lon, Lat): {self.own_pos_lonlat}\n"
                f"Absolute Position (X, Y): {self.own_abs_pos_xy}\n"
                f"Speed: {self.abs_speed}\n"
                f"Heading: {self.heading}\n"
                f"Moving: {self.moving}\n"
                "====================OWN SHIP INFO===================\n")
                