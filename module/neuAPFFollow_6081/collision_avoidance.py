"""
Alarm class containing an alarm status which can be "on", "off", "standby", and "slienced", 
and a pandas dataframe of alarm information ('t_idx', 'alarm_level', 'last_alarm_level', 'alarm_time') for each ship
"""
import numpy as np
import datetime
import pandas as pd

from para import unit_to_meter, ship_safe_distance, ship_tolerance
from algo_utility import closest_point_on_line, convert_lonlat_to_abs_xy, convert_abs_xy_to_lonlat, convert_lonlat_to_rel_xy, convert_dxy_to_lonlat, get_next_path_point, revert_speed_change, turnfun
from apf import NeuAPF
from new_alarm import Alarm

class PathPlanning:
    def __init__(self, safe_distance=ship_safe_distance, tolerance=ship_tolerance, u2m = unit_to_meter, update_rate = 10, auto_ca = 2, global_goal_lonlat_list=None, locked_t_idx = None, enable_heading_filter=True, filter_config=None, enable_anti_zigzag=True):
        self.safe_distance = safe_distance
        self.tolerance = tolerance                    
        self.global_goal_lonlat_list = global_goal_lonlat_list
        self.locked_t_idx = locked_t_idx
        self.auto_ca = auto_ca
        
        # Default filter configuration
        default_filter_config = {
            'process_noise': 1.0,
            'measurement_noise': 25.0, 
            'jump_threshold': 45.0,
            'responsiveness': 'balanced'
        }
        
        # Use provided config or defaults
        self.filter_config = filter_config if filter_config else default_filter_config
        
        self.apf_planner = NeuAPF(enable_heading_filter=enable_heading_filter, filter_config=self.filter_config)
        #########
        self.gp_index = 0
        self.update_rate = update_rate
        self.rc_state = 0
        if self.auto_ca != 2:
            self.update_rate = 1
            self.tolerance = 30/u2m
        self.inner_clock = 0
        self.u2m = u2m
        self.path_str = "$LP,0,0"
        self.default_speed = 8
        self.turn_speed = 4
        self.max_speed = 8
        self.min_speed = 1
        self.crit_target_idx = set()
        self.revert_speed_time = datetime.datetime.now()
        self.targeting_speed = self.default_speed
        self.alarm_instance = Alarm()
        self.temp_goal = None
        
        # Path evaluation variables to prevent zigzagging
        self.enable_anti_zigzag = enable_anti_zigzag
        self.last_path_rel_xy = None  # Store last planned path in relative coordinates
        self.last_path_timestamp = None
        self.path_evaluation_enabled = enable_anti_zigzag

        
    def lonlat_to_xy_rel2_start(self, abs_pos_lonlat):
        abs_pos_xy = convert_lonlat_to_abs_xy(abs_pos_lonlat, self.u2m)
        return (abs_pos_xy[0] - self.start_abs_pos_xy[0], abs_pos_xy[1] - self.start_abs_pos_xy[1])

    def set_auto_ca(self, auto_ca):
        self.auto_ca = auto_ca

    def set_heading_filter_enabled(self, enabled):
        """
        Enable or disable the Kalman filter for target heading smoothing
        
        Args:
            enabled (bool): True to enable filter, False to disable
        """
        self.apf_planner.set_heading_filter_enabled(enabled)

    def set_path_evaluation_enabled(self, enabled):
        """
        Enable or disable path evaluation to prevent zigzagging
        
        Args:
            enabled (bool): True to enable path evaluation, False to disable
        """
        self.path_evaluation_enabled = enabled
        self.enable_anti_zigzag = enabled
        if not enabled:
            self.last_path_rel_xy = None
            self.last_path_timestamp = None
            print("Path evaluation disabled - will always use new path")
        else:
            print("Path evaluation enabled - anti-zigzag behavior active")

    def set_anti_zigzag_enabled(self, enabled):
        """
        Enable or disable anti-zigzag behavior entirely
        
        Args:
            enabled (bool): True to enable anti-zigzag, False to always use new path
        """
        self.enable_anti_zigzag = enabled
        self.path_evaluation_enabled = enabled
        if not enabled:
            self.last_path_rel_xy = None
            self.last_path_timestamp = None
            print("Anti-zigzag disabled - will always use new path (original behavior)")
        else:
            print("Anti-zigzag enabled - will evaluate path safety to prevent zigzagging")

    def evaluate_path_safety(self, path_rel_xy, ship, target_df, simulation_time=60):
        """
        Simulate following a path and check if it's safe from collisions.
        
        Args:
            path_rel_xy: Path points in relative coordinates [(x1,y1), (x2,y2), ...]
            ship: Current ship state
            target_df: DataFrame of targets
            simulation_time: How long to simulate in seconds
            
        Returns:
            tuple: (is_safe, min_distance, has_opposite_direction)
        """
        if not path_rel_xy or len(path_rel_xy) < 2 or target_df.empty:
            return True, float('inf'), False
        
        # Make a copy of targets for simulation
        sim_targets = target_df.copy()
        
        # Ship simulation parameters
        current_pos = list(ship.own_rel_pos_xy)
        ship_speed = ship.abs_speed
        min_distance_to_targets = float('inf')
        
        # Calculate path segments and directions
        has_opposite_direction = False
        global_path_direction = None
        
        # Estimate global path direction from src to dst point in do_path_plan
        if hasattr(self, '_current_src_point') and hasattr(self, '_current_dst_point'):
            global_dx = self._current_dst_point[0] - self._current_src_point[0]
            global_dy = self._current_dst_point[1] - self._current_src_point[1]
            global_path_length = np.sqrt(global_dx**2 + global_dy**2)
            
            if global_path_length > 1e-6:  # Avoid division by zero
                global_path_direction = np.degrees(np.arctan2(global_dx, global_dy))
                global_path_direction = (global_path_direction + 360) % 360
        
        # Simulate movement along path
        time_step = 2.0  # seconds per step
        steps = int(simulation_time / time_step)
        
        for step in range(steps):
            current_time = step * time_step
            
            # Update target positions
            for idx, target in sim_targets.iterrows():
                new_x = target['x'] + target['u'] * current_time
                new_y = target['y'] + target['v'] * current_time
                sim_targets.at[idx, 'x'] = new_x
                sim_targets.at[idx, 'y'] = new_y
                
                # Calculate distance to ownship
                distance = np.sqrt((new_x - current_pos[0])**2 + (new_y - current_pos[1])**2)
                min_distance_to_targets = min(min_distance_to_targets, distance)
            
            # Move ownship along path
            travel_distance = ship_speed * time_step
            remaining_distance = travel_distance
            
            # Find next position along path
            for i in range(len(path_rel_xy) - 1):
                if remaining_distance <= 0:
                    break
                    
                segment_start = path_rel_xy[i]
                segment_end = path_rel_xy[i + 1]
                
                # Check if current position is before this segment
                dist_to_start = np.sqrt((current_pos[0] - segment_start[0])**2 + 
                                      (current_pos[1] - segment_start[1])**2)
                
                if dist_to_start <= remaining_distance:
                    # Can reach or pass this segment
                    segment_length = np.sqrt((segment_end[0] - segment_start[0])**2 + 
                                           (segment_end[1] - segment_start[1])**2)
                    
                    if segment_length > 1e-6:  # Avoid zero-length segments
                        # Check direction relative to global path
                        if global_path_direction is not None:
                            segment_dx = segment_end[0] - segment_start[0]
                            segment_dy = segment_end[1] - segment_start[1]
                            segment_direction = np.degrees(np.arctan2(segment_dx, segment_dy))
                            segment_direction = (segment_direction + 360) % 360
                            
                            # Calculate angle difference
                            angle_diff = abs(segment_direction - global_path_direction)
                            if angle_diff > 180:
                                angle_diff = 360 - angle_diff
                            
                            # If segment goes roughly opposite direction (>90 degrees difference)
                            if angle_diff > 90:
                                has_opposite_direction = True
                        
                        if dist_to_start + segment_length <= remaining_distance:
                            # Pass entire segment
                            current_pos = list(segment_end)
                            remaining_distance -= (dist_to_start + segment_length)
                        else:
                            # Move partway along segment
                            progress = (remaining_distance - dist_to_start) / segment_length
                            progress = max(0, min(1, progress))  # Clamp to [0,1]
                            current_pos = [
                                segment_start[0] + progress * (segment_end[0] - segment_start[0]),
                                segment_start[1] + progress * (segment_end[1] - segment_start[1])
                            ]
                            remaining_distance = 0
                            break
                    else:
                        # Zero-length segment, just move to end point
                        current_pos = list(segment_end)
                        remaining_distance -= dist_to_start
                else:
                    break
            
            # Early exit if too close to any target
            if min_distance_to_targets < 50 / self.u2m:  # 50 meters
                break
        
        # Path is safe if minimum distance > 50m and no opposite direction segments
        is_safe = min_distance_to_targets >= (50 / self.u2m)
        
        return is_safe, min_distance_to_targets * self.u2m, has_opposite_direction

    def determine_path_side_deviation(self, path_rel_xy, src_point, dst_point):
        """
        Determine which side of the global path the planned path deviates to.
        
        Returns:
            str: 'left', 'right', or 'none'
        """
        if not path_rel_xy or len(path_rel_xy) < 2:
            return 'none'
        
        # Calculate global path vector
        global_vec = np.array([dst_point[0] - src_point[0], dst_point[1] - src_point[1]])
        global_vec_norm = np.linalg.norm(global_vec)
        
        if global_vec_norm < 1e-6:  # Avoid division by zero
            return 'none'
        
        global_vec_unit = global_vec / global_vec_norm
        
        # Check deviation of path points
        total_deviation = 0
        deviation_count = 0
        
        for point in path_rel_xy[1:-1]:  # Skip first and last points
            # Vector from src to current point
            point_vec = np.array([point[0] - src_point[0], point[1] - src_point[1]])
            
            # Project onto global path direction
            proj_length = np.dot(point_vec, global_vec_unit)
            proj_point = np.array(src_point) + proj_length * global_vec_unit
            
            # Calculate perpendicular deviation
            deviation_vec = np.array([point[0] - proj_point[0], point[1] - proj_point[1]])
            
            # Cross product to determine side (positive = left, negative = right)
            cross_product = global_vec[0] * deviation_vec[1] - global_vec[1] * deviation_vec[0]
            
            if abs(cross_product) > 1e-6:  # Significant deviation
                total_deviation += cross_product
                deviation_count += 1
        
        if deviation_count == 0:
            return 'none'
        
        avg_deviation = total_deviation / deviation_count
        
        if avg_deviation > 0:
            return 'left'
        elif avg_deviation < 0:
            return 'right'
        else:
            return 'none'

    def should_use_new_path(self, new_path_rel_xy, src_point, dst_point, ship, target_df):
        """
        Decide whether to use new path or stick with the last path to prevent zigzagging.
        
        Args:
            new_path_rel_xy: Newly planned path
            src_point: Source point of current segment
            dst_point: Destination point of current segment
            ship: Current ship state
            target_df: DataFrame of targets
            
        Returns:
            bool: True if should use new path, False if should use last path
        """
        # If anti-zigzag is disabled, always use new path (original behavior)
        if not self.enable_anti_zigzag: # 如果全局开关被关闭
            return True
            
        # Always use new path if no previous path exists
        if self.last_path_rel_xy is None: # 如果没有旧路径
            return True
        
        # Check if there are active threats
        has_threats = self.apf_planner.has_active_threats(target_df) # 检测在避碰过程中什么时候回到原航线
        
        # If no threats, always use new path (return to global path)
        if not has_threats:
            print("No threats detected - using new path to return to global path")
            return True
        
        # Evaluate safety of last path
        last_path_safe, last_min_dist, last_opposite = self.evaluate_path_safety( # 评估当前路径是否安全，返回旧路径是否安全(布尔值)/最小安全距离/路径是否反向
            self.last_path_rel_xy, ship, target_df
        )
        
        print(f"Last path evaluation - Safe: {last_path_safe}, Min distance: {last_min_dist:.1f}m, Opposite direction: {last_opposite}")
        
        # If last path is unsafe, must use new path
        if not last_path_safe or last_opposite:
            print("Last path is unsafe or has opposite direction - using new path")
            return True
        
        # Both paths appear safe, check for side switching to prevent zigzag
        last_side = self.determine_path_side_deviation(self.last_path_rel_xy, src_point, dst_point)
        new_side = self.determine_path_side_deviation(new_path_rel_xy, src_point, dst_point)
        
        print(f"Path side analysis - Last: {last_side}, New: {new_side}")
        
        # If new path switches to opposite side, stick with last path
        if (last_side == 'left' and new_side == 'right') or (last_side == 'right' and new_side == 'left'):# 如果「新路径的偏航侧」 和 「旧路径的偏航侧」 是「左右相反」的 → 判定为「即将出现之字形抖动」，返回 False，强制沿用旧路径。
            print("Detected side switching - keeping last path to prevent zigzagging")
            return False
        
        # Otherwise use new path
        print("No zigzag detected - using new path")
        return True

    def do_path_plan(self, ship=None, target_df=None):
        if not ship or not self.global_goal_lonlat_list:
            return True
        
        gp_abs_xy = self.path_lonlat_to_absxy(self.global_goal_lonlat_list)
        # # the starting value of gp_index is -1 therefore when used it should be added by 1, but during value update it should be substrated by 1
        if self.gp_index != len(gp_abs_xy):
            self.gp_index = get_next_path_point(self.gp_index - 1, gp_abs_xy, ship.start_abs_pos_xy, self.tolerance, 30) + 1       
        if len(gp_abs_xy)> self.gp_index >=1:
            src_point = convert_lonlat_to_rel_xy(self.global_goal_lonlat_list[self.gp_index-1], ship.start_lonlat, self.u2m)
            dst_point = convert_lonlat_to_rel_xy(self.global_goal_lonlat_list[self.gp_index], ship.start_lonlat, self.u2m)
        elif self.gp_index == 0:
            src_point = (0, 0)
            dst_point = convert_lonlat_to_rel_xy(self.global_goal_lonlat_list[0], ship.start_lonlat, self.u2m)
        elif self.gp_index == len(gp_abs_xy):
            print("Arrived at the destination")          
            return True

        if self.auto_ca == 2 and self.inner_clock % self.update_rate == 0:   
            print(f"gp_index-=-=-=-=-: {self.gp_index}") 
            # print(f"src_point: {src_point}, dst_point: {dst_point}")   
            self.targeting_speed = self.default_speed 
            # self.change_speed_to_avoid_all(ship, target_df)   
            # self.change_course_to_avoid_one(ship, target_df)
            if self.temp_goal is not None:
                dst_point = self.temp_goal 
            
            # Store current segment points for path evaluation
            self._current_src_point = src_point
            self._current_dst_point = dst_point
            
            # self.apf_planner.set_plan_speed(self.targeting_speed)       
            self.apf_planner.set_plan_speed(ship.abs_speed)       
            
            # Plan new path
            new_lpath = self.apf_planner.do_path_plan(ship, target_df, src_point, dst_point, self.update_rate * 5)
            
            # Decide whether to use new path or keep last path
            if self.path_evaluation_enabled and self.enable_anti_zigzag and self.should_use_new_path(new_lpath, src_point, dst_point, ship, target_df):
                # Use new path
                lpath = new_lpath
                self.last_path_rel_xy = lpath.copy()
                self.last_path_timestamp = datetime.datetime.now()
                print("Using newly planned path")
            elif self.last_path_rel_xy is not None and self.path_evaluation_enabled and self.enable_anti_zigzag:
                # Keep using last path
                lpath = self.last_path_rel_xy
                print("Keeping last path to prevent zigzagging")
            else:
                # Fallback to new path (anti-zigzag disabled or no previous path)
                lpath = new_lpath
                if self.enable_anti_zigzag:
                    self.last_path_rel_xy = lpath.copy()
                    self.last_path_timestamp = datetime.datetime.now()
                print("Using new path" + (" (anti-zigzag disabled)" if not self.enable_anti_zigzag else " (fallback)"))
            
            path_lonlat = [convert_dxy_to_lonlat(p, ship.start_lonlat, self.u2m) for p in lpath] 
            self.path_str = self.get_LP_string(path_lonlat, self.targeting_speed)
                 
        # elif not self.auto_ca and self.inner_clock % self.update_rate == 0:
        elif self.auto_ca == 1:
            lineproj_xy = closest_point_on_line((0, 0), src_point, dst_point)
            vec_src_to_dst = (dst_point[0] - src_point[0], dst_point[1] - src_point[1])
            vec_src_to_lineproj = (lineproj_xy[0] - src_point[0], lineproj_xy[1] - src_point[1])
            if np.dot(vec_src_to_dst, vec_src_to_lineproj) < 0:
                lineproj_xy = src_point
            lineproj_abs_xy = (lineproj_xy[0] + ship.start_abs_pos_xy[0], lineproj_xy[1] + ship.start_abs_pos_xy[1])
            self.targeting_speed = self.slow_to_avoid_all(ship, target_df)
            # if np.sqrt(dst_point[0]**2 + dst_point[1]**2) < (self.tolerance + 75) and self.targeting_speed > self.turn_speed:
            #     self.targeting_speed = self.turn_speed
            # 将路径转换为经度和纬度
            path_lonlat = [convert_abs_xy_to_lonlat(p, self.u2m) for p in [lineproj_abs_xy]+gp_abs_xy[self.gp_index:]]
            self.path_str = self.get_LP_string(path_lonlat, self.targeting_speed)
        
        elif not self.auto_ca:
            lineproj_xy = closest_point_on_line((0, 0), src_point, dst_point)
            vec_src_to_dst = (dst_point[0] - src_point[0], dst_point[1] - src_point[1])
            vec_src_to_lineproj = (lineproj_xy[0] - src_point[0], lineproj_xy[1] - src_point[1])
            if np.dot(vec_src_to_dst, vec_src_to_lineproj) < 0:
                lineproj_xy = src_point
            lineproj_abs_xy = (lineproj_xy[0] + ship.start_abs_pos_xy[0], lineproj_xy[1] + ship.start_abs_pos_xy[1])
            self.targeting_speed = self.default_speed
            # if np.sqrt(dst_point[0]**2 + dst_point[1]**2) < (self.tolerance + 75) and self.targeting_speed > self.turn_speed:
            #     self.targeting_speed = self.turn_speed
            # 将路径转换为经度和纬度
            path_lonlat = [convert_abs_xy_to_lonlat(p, self.u2m) for p in [lineproj_abs_xy]+gp_abs_xy[self.gp_index:]]
            self.path_str = self.get_LP_string(path_lonlat, self.targeting_speed)
        
        return False
    
    def update(self, global_goal_lonlat_list, default_speed=4, rc_state = 0):
        self.default_speed = default_speed
        self.rc_state = rc_state
        if global_goal_lonlat_list!=self.global_goal_lonlat_list:
            self.global_goal_lonlat_list = global_goal_lonlat_list.copy()
            self.gp_index = 0
            self.inner_clock = 0
        else:
            self.inner_clock = (self.inner_clock + 1) % self.update_rate
    

    def detect_targets_in_sector(self, ship_position, targets_df, radius, angle, ship_orientation):
        def is_within_sector(target_x, target_y):
            dx = target_x - ship_position[0]
            dy = target_y - ship_position[1]
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance > radius:
                return False           
            target_angle = np.degrees(np.arctan2(dy, dx))
            relative_angle = (target_angle - ship_orientation) % 360
            if relative_angle > 180:
                relative_angle -= 360
            
            return abs(relative_angle) <= angle / 2
        
        for _, row in targets_df.iterrows():
            if is_within_sector(row['x'], row['y']):
                return True
        return False  
    
    
    def path_lonlat_to_absxy(self, path_lonlat):
        pathxy = [convert_lonlat_to_abs_xy(lonlat, self.u2m) for lonlat in path_lonlat]
        return pathxy

    def speed_to_rpm(self, speed):#实船
        # if speed == 0:
        #     return 0
        # elif speed == -1:
        #     return 50
        # elif speed <= 2:
        #     return 60
        # elif speed <= 4:
        #     return 70
        # elif speed <= 6:
        #     return 80
        # elif speed <= 8:
        #     return 90
        # elif speed > 8:
        #     return 90
        # else:
        #     return 0 
        ###########################################
        # if speed == 0:
        #     return 0
        # elif speed == -1:
        #     return 450
        # elif speed <= 2:
        #     return 500
        # elif speed <= 4:
        #     return 600
        # elif speed <= 6:
        #     return 700
        # elif speed <= 8:
        #     return 800
        # elif speed > 8:
        #     return 850
        # else:
        #     return 0 
       
        ###########################################
        # 6081实船
        if speed == 0:
            return 0
        elif speed == 2:
            return 80
        elif speed == 4:  
            return 180
        elif speed == 6:
            return 280
        elif speed == 8:
            return 310
        elif speed == 10:
            return 360
        elif speed == 12:
            return 450
        else:
            return 0 
        
    def get_LP_string(self, path_lonlat, targeting_speed=None, u2m=unit_to_meter, rpm_output=True):
        if not path_lonlat:
            if self.rc_state == 1:
                return f"$LP,{str(targeting_speed)},0"
            else:
                return "$LP,0,0"

        # filter all duplicated points that are next to each other
        path_lonlat = [path_lonlat[i] for i in range(len(path_lonlat)) if i == 0 or path_lonlat[i] != path_lonlat[i-1]]
        # 将路径转换为字符串
        path_str = ','.join([str(p[0]) + ',' + str(p[1]) for p in path_lonlat])
        if rpm_output:
            targeting_speed = self.speed_to_rpm(targeting_speed)
        else:
            targeting_speed = 2 if targeting_speed == -1 else targeting_speed
        if targeting_speed is not None:
            path_str = f"$LP,{str(targeting_speed)},{str(len(path_lonlat))},{path_str}"
        else:
            path_str = f"$LP,{str(len(path_lonlat))},{path_str}"
        # print("targeting_speed: ", targeting_speed)
        # path_str = f"$LP,{str(len(path_lonlat))},{path_str}"
        return path_str

    def crit_target_filter(self, own_xy, own_uv, target_speed, target_distance, target_xy, target_uv, tcpa, alarm):
        if (0<tcpa<4 or (target_speed <=0.51444 and target_distance <= 720)) and alarm>=2 and \
            (self.alarm_instance.calculate_collision_angle(own_xy, own_uv, target_xy, target_uv, 200) == "front" or \
             self.alarm_instance.calculate_collision_angle(own_xy, own_uv, target_xy, target_uv, 100) == "front"):
            return True
        # if 0<tcpa<4 and alarm>=2:
        #     return True
        return False

    def change_course_to_avoid_one(self, ship, target_df):
        current_time = datetime.datetime.now()
        
        # Step 1: Find current crossing-back targets
        target_df_b = target_df[target_df.apply(lambda row: (self.apf_planner.colreg_check(ship.own_rel_pos_xy, (row['x'], row['y']), ship.heading, row['heading'], self.default_speed, row['speed'], row['azimuth'], row['tcpa'], row['dcpa']) in ["CROSS_BACK_A", "CROSS_BACK", "OVERTAKING_R", "OVERTAKING_L"]), axis=1)].dropna().reset_index(drop=True)
        
        # Step 2: Clean up critical target index - remove targets that no longer exist
        current_target_ids = set(target_df['t_idx'].values)
        self.crit_target_idx = self.crit_target_idx.intersection(current_target_ids)
        
        # Step 3: Filter critical targets using revert_speed_change (remove targets that have passed)
        self.crit_target_idx = set(filter(lambda t_idx: not revert_speed_change(ship, target_df, t_idx), self.crit_target_idx))
        
        # Step 4: Add new crossing-back targets to critical set
        if not target_df_b.empty:
            new_critical_targets = set(target_df_b['t_idx'])
            self.crit_target_idx.update(new_critical_targets)
            self.apf_planner.update_s_domain_tidx(new_critical_targets)
            
            # Calculate temp goals for new threats only
            temp_goals = target_df_b.apply(lambda row: turnfun(ship.own_rel_pos_xy, ship.own_speed_uv, (row['x'], row['y']), (row['u'], row['v']), 200), axis=1).tolist()
            temp_goals = [goal for goal in temp_goals if goal is not None]
            if temp_goals:
                # Choose closest temp goal to current position
                self.temp_goal = min(temp_goals, key=lambda tg: np.linalg.norm(np.array(tg) - np.array(ship.own_rel_pos_xy)))
                # Reset timer when new temp goal is set
                self.revert_speed_time = current_time + datetime.timedelta(seconds=10)
        
        print(f"crit_target_idx: {self.crit_target_idx}")
        print(f"temp_goal: {self.temp_goal}")
        
        # Step 5: Handle active avoidance behavior
        if self.crit_target_idx and self.temp_goal is not None:
            # Check if we're still within avoidance time window
            if current_time < self.revert_speed_time:
                # Scale temp goal gradually but with bounds to prevent runaway scaling
                scale_factor = 1.1  # Reduced from 1.25 to prevent excessive scaling
                max_distance = 1000 / self.u2m  # Maximum 1000m from ownship
                
                scaled_goal = (self.temp_goal[0] * scale_factor, self.temp_goal[1] * scale_factor)
                goal_distance = np.linalg.norm(np.array(scaled_goal) - np.array(ship.own_rel_pos_xy))
                
                if goal_distance <= max_distance:
                    self.temp_goal = scaled_goal
                # If goal would be too far, keep current temp_goal without scaling
                return
        
        # Step 6: Check if all critical targets have been resolved
        active_critical_targets = [t_idx for t_idx in self.crit_target_idx 
                                 if t_idx in current_target_ids and 
                                 not revert_speed_change(ship, target_df, t_idx)]
        
        # Step 7: Reset if no active critical targets or time expired
        if not active_critical_targets or current_time >= self.revert_speed_time:
            print("Resetting temp goal - no active critical targets or time expired")
            self.revert_speed_time = current_time
            self.temp_goal = None
            self.crit_target_idx.clear()  # Clear all critical targets when resetting
            self.apf_planner.reset_s_domain_tidx()

    def change_speed_to_avoid_all(self, ship, target_df):
        target_df_ba = target_df[target_df.apply(lambda row: (self.apf_planner.colreg_check(ship.heading, row['heading'], ship.abs_speed, row['speed'], row['azimuth'], row['tcpa'], row['dcpa']) == "CROSS_BACK_A"), axis=1)].dropna().reset_index(drop=True)
        target_df_b = target_df[target_df.apply(lambda row: (self.apf_planner.colreg_check(ship.heading, row['heading'], ship.abs_speed, row['speed'], row['azimuth'], row['tcpa'], row['dcpa']) == "CROSS_BACK"), axis=1)].dropna().reset_index(drop=True)
        current_time = datetime.datetime.now()

        if not target_df_ba.empty:
            self.crit_target_idx.update(set(target_df_ba['t_idx']))
            self.targeting_speed = self.max_speed

        if not target_df_b.empty:
            self.crit_target_idx.update(set(target_df_b['t_idx']))
            self.apf_planner.update_s_domain_tidx(set(target_df_b['t_idx']))
            self.targeting_speed = self.min_speed 
            
        self.crit_target_idx = set(filter(lambda t_idx: not revert_speed_change(ship, target_df, t_idx), self.crit_target_idx))
        print(f"crit_target_idx: {self.crit_target_idx}")
        if self.crit_target_idx:
            self.revert_speed_time = current_time + datetime.timedelta(minutes=0.5)

        if current_time < self.revert_speed_time:
            return

        self.revert_speed_time = current_time
        self.targeting_speed = self.default_speed
        self.apf_planner.reset_s_domain_tidx()

    
    def slow_to_avoid_all(self, ship, target_df):
        own_xy = ship.own_rel_pos_xy
        own_uv = ship.own_speed_uv
        # filter the target_df using the crit_target_filter
        # target_df = target_df[target_df.apply(lambda row: self.crit_target_filter(own_xy, own_uv, (row['x'], row['y']), (row['u'], row['v']), row['tcpa'], row['dcpa']), axis=1)]
        target_df = target_df[target_df.apply(lambda row: self.crit_target_filter(own_xy, own_uv, row['speed'], row['distance'], (row['x'], row['y']), (row['u'], row['v']), row['tcpa'], row['alarm']), axis=1)]
        # extend the self.crit_target_idx set with the "t_idx" of the target_df
        print(target_df)
        self.crit_target_idx.update(target_df['t_idx'])
        print(f"crit_target_idx: {self.crit_target_idx}")
        if not set(filter(lambda t_idx: target_df.loc[target_df['t_idx'] == t_idx, 'alarm'].empty , self.crit_target_idx)):
            self.crit_target_idx = set(filter(lambda t_idx: target_df.loc[target_df['t_idx'] == t_idx, 'alarm'].values[0] > 0, self.crit_target_idx))
        else:
            self.crit_target_idx = set()
        print(f"crit_target_idx===: {self.crit_target_idx}")
        if self.crit_target_idx:
            self.revert_speed_time = datetime.datetime.now() + datetime.timedelta(minutes=0.5)
        if datetime.datetime.now() < self.revert_speed_time:
            # return self.apf_planner.min_speed
            return -1
        self.revert_speed_time = datetime.datetime.now()
        return self.default_speed