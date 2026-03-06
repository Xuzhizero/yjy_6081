import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from scipy.special import expit
from algo_utility import cal_distance, cal_DCPA_TCPA, closest_point_on_line, check_point_on_seg
from algo_utility import stop_time_estimation, revert_speed_change, smooth_last_segment, calculate_collision_angle
from para import unit_to_meter, ship_safe_distance

SimTime = 10 # 单步仿真时间
SafatyMargin = 1.0 # 领域的安全裕度
SafatyMargin2 = 1.0 # 领域的安全裕度2
SafatyMargin3 = 2.0 # 领域的安全裕度3，计算虚拟目标点
SafatyMargin4 = 5*5 # 领域的安全裕度4，大于5倍领域就没有势场
AttrGain = 1.0 # 引力增益，增加系数可以增大目标点引力
RepuGain = 1.0 # 斥力增益，增加系数可以增大障碍物斥力
ObstacleDetect = 2000/unit_to_meter # 障碍物检测距离，单位米
# MapBound = 10 # 地图边界，单位海里
MapBound = 3 # 地图边界，单位海里
m_scale = MapBound*1852/unit_to_meter # 目标点引力范围
MaxTurn = 0.8*SimTime # 单步规划的最大转向范围
apf_num = 7 # 人工势场搜索的点数，为大于等于3的奇数
ReachRadius = 4*SimTime # 单步的长度=速度*仿真时间
DomainLength = 250/unit_to_meter # 船舶领域设定的船长(在海上以100米为标准)
SafeTCPA = 300/60 # 判定安全的TCPA值，大于此值看作无避碰风险
SafeDCPA = 400/1852 # 判定安全的DCPA值，大于此值看作无避碰风险
MyDomain = {'fore': 9.9*DomainLength,"aft": 9.9*DomainLength, "starb": 1.5*DomainLength, "port": 1.05*DomainLength} # 船舶领域，前、后、右、左，实际避让距离是领域长度*3左右
MyStaticDomain = {'fore': 1.5*DomainLength,"aft": 1.5*DomainLength, "starb": 1.5*DomainLength, "port": 1.5*DomainLength} # static船舶领域，前、后、右、左，实际避让距离是领域长度*3左右

class NeuAPF:
    def __init__(self, enable_heading_filter=True, filter_config=None):
        self.colreg_domain_mem = {}
        self.plan_speed = 4
        # self.prioritized_t_idx = None
        self.end_acc_time = None
        self.s_domain_tidx = set()  # Changed from {} to set() for consistency
        # Kalman filter for target heading smoothing
        self.enable_heading_filter = enable_heading_filter
        
        # Default filter configuration
        default_config = {
            'process_noise': 1.0,
            'measurement_noise': 25.0,
            'jump_threshold': 45.0,
            'responsiveness': 'balanced'
        }
        self.filter_config = filter_config if filter_config else default_config
        
        self.target_heading_filters = {}
        self.target_last_update = {}
        
        # Ownship heading tracking for compensation
        self.last_ownship_heading = None
        self.ownship_heading_history = []
        
        if enable_heading_filter:
            print(f"NeuAPF initialized with heading filter config: {self.filter_config['responsiveness']}")

    def set_plan_speed(self, speed):
        self.plan_speed = speed

    def update_s_domain_tidx(self, s_domain_tidx):
        self.s_domain_tidx.update(s_domain_tidx)

    def reset_s_domain_tidx(self):
        self.s_domain_tidx = set()

    def set_heading_filter_enabled(self, enabled):
        """
        Enable or disable the Kalman filter for target heading smoothing
        
        Args:
            enabled (bool): True to enable filter, False to disable
        """
        self.enable_heading_filter = enabled
        if not enabled:
            # Clear existing filters when disabled
            self.target_heading_filters.clear()
            print("Target heading filter disabled")
        else:
            print("Target heading filter enabled")

    def normalize_heading(self, heading):
        """Normalize heading to [0, 360) degrees"""
        return (heading + 360) % 360

    def heading_difference(self, h1, h2):
        """Calculate the smallest angular difference between two headings"""
        diff = h1 - h2
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def update_target_heading_filter(self, t_idx, raw_heading, dt=1.0):
        """
        Update Kalman filter for target heading smoothing with configurable intensity
        
        Args:
            t_idx: Target index
            raw_heading: Raw heading from radar (degrees)
            dt: Time step (seconds)
        
        Returns:
            filtered_heading: Smoothed heading (degrees)
        """
        raw_heading = self.normalize_heading(raw_heading)
        
        # Get configurable parameters
        process_noise_base = self.filter_config['process_noise']
        measurement_noise = self.filter_config['measurement_noise']
        jump_threshold = self.filter_config['jump_threshold']
        
        if t_idx not in self.target_heading_filters:
            # Initialize new filter for this target
            self.target_heading_filters[t_idx] = {
                'heading': raw_heading,  # State: heading (degrees)
                'rate': 0.0,             # State: turning rate (degrees/s)
                'P': np.array([[100.0, 0.0],    # State covariance matrix
                              [0.0, 10.0]]),
                'Q': np.array([[process_noise_base, 0.0],      # Process noise (configurable)
                              [0.0, process_noise_base * 0.5]]),
                'R': measurement_noise,  # Measurement noise (configurable)
                'last_raw': raw_heading
            }
            return raw_heading
        
        filter_state = self.target_heading_filters[t_idx]
        
        # Handle heading wrap-around and large jumps using configurable threshold
        heading_diff = self.heading_difference(raw_heading, filter_state['last_raw'])
        if abs(heading_diff) > jump_threshold:
            # Limit the change to prevent sudden jumps (configurable threshold)
            innovation = self.heading_difference(raw_heading, filter_state['heading'])
            if abs(innovation) > jump_threshold:
                innovation = np.sign(innovation) * jump_threshold  # Limit large changes
        else:
            innovation = self.heading_difference(raw_heading, filter_state['heading'])
        
        # Update process noise dynamically (use current config values)
        filter_state['Q'] = np.array([[process_noise_base, 0.0],
                                     [0.0, process_noise_base * 0.5]])
        filter_state['R'] = measurement_noise
        
        # Kalman filter prediction step
        F = np.array([[1.0, dt],   # State transition matrix
                      [0.0, 1.0]])
        
        # Predict state
        x_pred = np.array([filter_state['heading'], filter_state['rate']])
        x_pred = F @ x_pred
        x_pred[0] = self.normalize_heading(x_pred[0])  # Keep heading in [0,360)
        
        # Predict covariance
        P_pred = F @ filter_state['P'] @ F.T + filter_state['Q']
        
        # Kalman filter update step
        H = np.array([1.0, 0.0])  # Measurement matrix (we observe heading only)
        
        # Innovation
        z = filter_state['heading'] + innovation  # Expected measurement
        y = self.heading_difference(z, x_pred[0])  # Innovation
        
        # Innovation covariance
        S = H @ P_pred @ H.T + filter_state['R']
        
        # Kalman gain
        K = P_pred @ H.T / S
        
        # Update state
        x_updated = x_pred + K * y
        x_updated[0] = self.normalize_heading(x_updated[0])
        
        # Update covariance
        P_updated = (np.eye(2) - np.outer(K, H)) @ P_pred
        
        # Store updated filter state
        filter_state['heading'] = x_updated[0]
        filter_state['rate'] = x_updated[1]
        filter_state['P'] = P_updated
        filter_state['last_raw'] = raw_heading
        
        return x_updated[0]

    def apply_heading_filter(self, target_df, ownship=None):
        """
        Apply Kalman filtering to all target headings with ownship course change compensation
        
        Args:
            target_df: DataFrame with target data
            ownship: Ownship object (optional, for heading compensation)
            
        Returns:
            target_df: DataFrame with filtered headings (if enabled) or original headings
        """
        if target_df.empty:
            return target_df
        
        # Check if heading filter is enabled
        if not self.enable_heading_filter:
            print("Heading filter disabled - using raw radar headings")
            return target_df

        # Track ownship heading changes for compensation
        ownship_heading_delta = 0.0
        if ownship is not None:
            current_heading = ownship.heading
            if self.last_ownship_heading is not None:
                # Calculate heading change since last update
                ownship_heading_delta = self.heading_difference(current_heading, self.last_ownship_heading)
                
                # Keep history for debugging (limit to last 10 entries)
                self.ownship_heading_history.append({
                    'time': len(self.ownship_heading_history),
                    'heading': current_heading,
                    'delta': ownship_heading_delta
                })
                if len(self.ownship_heading_history) > 10:
                    self.ownship_heading_history.pop(0)
                    
            self.last_ownship_heading = current_heading

        # Clean up filters for targets that no longer exist
        current_targets = set(target_df['t_idx'])
        self.target_heading_filters = {k: v for k, v in self.target_heading_filters.items() 
                                     if k in current_targets}
        
        # Apply filtering to each target with ownship heading compensation
        def filter_heading(row):
            raw_heading = row['heading']
            
            # Compensate for ownship course changes
            # When ownship turns right (+), targets appear to turn left (-) in relative coordinates
            # So we add the ownship heading change to compensate
            compensated_heading = raw_heading
            if abs(ownship_heading_delta) > 0.1:  # Only compensate for significant changes
                compensated_heading = self.normalize_heading(raw_heading + ownship_heading_delta)
                # Debug output for significant compensation
                if abs(ownship_heading_delta) > 1.0:
                    print(f"  Target {row['t_idx']}: Raw={raw_heading:.1f}° → Compensated={compensated_heading:.1f}° (Δ={ownship_heading_delta:.1f}°)")
            
            filtered_heading = self.update_target_heading_filter(
                row['t_idx'], 
                compensated_heading, 
                dt=SimTime  # Use simulation time step
            )
            return filtered_heading
        
        # Create a copy to avoid modifying the original
        target_df = target_df.copy()
        target_df['heading_raw'] = target_df['heading']  # Store original
        target_df['heading'] = target_df.apply(filter_heading, axis=1)
        
        # Recalculate velocity components with filtered headings
        target_df['u'] = target_df['speed'] * np.sin(np.radians(target_df['heading']))
        target_df['v'] = target_df['speed'] * np.cos(np.radians(target_df['heading']))
        
        # Debug output for significant ownship heading changes
        if abs(ownship_heading_delta) > 1.0:
            print(f"Applied heading filter to {len(target_df)} targets (ownship Δheading: {ownship_heading_delta:.1f}°)")
        else:
            print(f"Applied heading filter to {len(target_df)} targets")
        
        return target_df

    def has_active_threats(self, target_df):
        """
        Check if there are any active threats based on DCPA, TCPA, and alarm levels
        """
        if target_df.empty:
            return False
        
        # Filter for targets that pose a threat 
        # 四个条件全部满足（dcpa/tcpa/报警等级）
        threat_targets = target_df[
            (target_df['dcpa'] < SafeDCPA) & 
            (target_df['tcpa'] > 0) & 
            (target_df['tcpa'] < SafeTCPA) & 
            (target_df['alarm'] >= 2)
        ]
        
        return not threat_targets.empty

    def calculate_path_segment_heading(self, src_point, dst_point):
        """
        Calculate the heading (in degrees) from src_point to dst_point
        """
        dx = dst_point[0] - src_point[0]
        dy = dst_point[1] - src_point[1]
        
        # Calculate heading in degrees (0 degrees = North, 90 degrees = East)
        heading = np.degrees(np.arctan2(dx, dy))
        
        # Normalize to 0-360 degrees
        heading = (heading + 360) % 360
        
        return heading

    def is_significantly_off_path(self, ownship_heading, path_heading, threshold=10):
        """
        Check if the ownship heading is significantly different from the path heading
        """
        # Calculate the smallest angle difference between two headings
        diff = abs(ownship_heading - path_heading)
        if diff > 180:
            diff = 360 - diff
        
        return diff > threshold

    def get_dist_point_to_segm(self, cur_pos, start_pos, end_pos):
        px, py = cur_pos[0], cur_pos[1]
        ax, ay = start_pos[0], start_pos[1]
        bx, by = end_pos[0], end_pos[1]
        ABx, ABy = bx - ax, by - ay
        APx, APy = px - ax, py - ay
        AB_AP = ABx * APx + ABy * APy
        distAB2 = ABx * ABx + ABy * ABy
        Dx, Dy = ax, ay
        if distAB2 != 0:
            t = AB_AP / distAB2
            if t >= 1:
                Dx, Dy = bx, by
            elif t > 0:
                Dx, Dy = ax + ABx * t, ay + ABy * t
        PDx, PDy = Dx - px, Dy - py
        return np.sqrt(PDx * PDx + PDy * PDy)

    def cal_ts_potential(self, cur_position, ts_pos, ts_heading, ts_fore, ts_aft, ts_port, ts_starb):
        d_x = cur_position[0] - ts_pos[0]
        d_y = cur_position[1] - ts_pos[1]
        angle = np.degrees(np.arctan2(d_x, d_y)) - ts_heading
        angle = (angle + 360) % 360

        tmp_d_x = d_x
        d_x = d_y * np.sin(np.radians(-ts_heading)) + d_x * np.cos(np.radians(-ts_heading))
        d_y = -tmp_d_x * np.sin(np.radians(-ts_heading)) + d_y * np.cos(np.radians(-ts_heading))

        domianpotential = 1000.0
        if 0 <= angle <= 90:
            domianpotential = (d_y / ts_fore)**2 + (d_x / ts_starb)**2
        elif 90 < angle <= 180:
            domianpotential = (d_y / ts_aft)**2 + (d_x / ts_starb)**2
        elif 180 < angle <= 270:
            domianpotential = (d_y / ts_aft)**2 + (d_x / ts_port)**2
        elif 270 < angle < 360:
            domianpotential = (d_y / ts_fore)**2 + (d_x / ts_port)**2

        potential = 1 / (1 + expit(SafatyMargin * domianpotential))
        if domianpotential > SafatyMargin4:
            potential = 0

        return potential * RepuGain

    def cal_all_potential(self, cur_position, goal, target_df, m_srcpoint, m_dstpoint):
        temp_df = target_df.copy()
        repu_force = 0.0
        attr_force = AttrGain * (np.power(goal[0] - cur_position[0], 2) + np.power(goal[1] - cur_position[1], 2))
        # print("attr_force_1", attr_force)
        attr_force = -AttrGain * np.exp(-np.power(cal_distance(cur_position, goal) / m_scale, 2))  # Goal point attraction
        # print("attr_force_2", attr_force)
        dist2globalroute = self.get_dist_point_to_segm(cur_position, m_srcpoint, m_dstpoint)
        attr_force += -AttrGain * np.exp(-np.power(dist2globalroute / (m_scale / 3), 2))  # Global path attraction
        # print("attr_force_3", attr_force)

        # get sub dataframe of target_df if the distance column is less than obstacle_detect
        temp_df = temp_df[temp_df['distance'] <= ObstacleDetect]
        if not temp_df.empty:
            repu_force += temp_df.apply(lambda ts: self.cal_ts_potential(cur_position, (ts['x'], ts['y']), ts['heading'], ts['fore'], ts['aft'], ts['port'], ts['starb']), axis=1).sum()
        # print("repu_force", repu_force)
        # Uncomment and implement if needed
        # for obstacle in m_obstaclepoly:
        #     repu_force += cal_static_potential(cur_position, obstacle)
        return repu_force + attr_force


    def cap_goal(self, cur_point, dst_point, src_point):
        cur_proj_online = closest_point_on_line(cur_point, src_point, dst_point)
        if cal_distance(dst_point, cur_proj_online) > MapBound*1852:
            dx = dst_point[0] - cur_proj_online[0]
            dy = dst_point[1] - cur_proj_online[1]
            dist = cal_distance(cur_proj_online, dst_point)
            return (cur_proj_online[0] + dx/dist*MapBound*1852, cur_proj_online[1] + dy/dist*MapBound*1852)
        return dst_point

    def init_ship_domain(self, target_df):
        # 船舶领域，前、后、右、左，实际避让距离是领域长度*3左右
        target_df['fore'] = MyDomain['fore']
        target_df['aft'] = MyDomain['aft']
        target_df['starb'] = MyDomain['starb']
        target_df['port'] = MyDomain['port']
        return target_df

    def init_static_domain(self, target_df):
        target_df.loc[(target_df['dcpa'] > SafeDCPA) | (target_df['tcpa'] > SafeTCPA) | (target_df['tcpa'] < 0) | (target_df['speed'] < 0.51444/unit_to_meter), ['fore', 'aft', 'starb', 'port']] = MyStaticDomain['fore']
        target_df.loc[target_df['speed'] < 0.51444/unit_to_meter, 'speed'] = 0
        return target_df
 
        
    # def colreg_check(self, own_heading, target_heading, own_speed, target_speed, azimuth, tcpa, dcpa):
    #     heading_diff = (target_heading - own_heading + 360) % 360
    #     azimuth = (azimuth - own_heading + 360) % 360
    #     if dcpa < SafeDCPA and 0 < tcpa < 10:
    #         if 165 <= heading_diff <= 195:
    #             return "HEADON"
    #         elif 0 <= heading_diff < 15 or 345 < heading_diff <= 360:
    #             if 270 <= azimuth <= 360  or 0 < azimuth <= 90:
    #                 if 345 < heading_diff < 360 and own_speed < target_speed:
    #                     return "OVERTAKING_R"
    #                 else:
    #                     return "OVERTAKING_L"
    #             else:
    #                 if 0 < heading_diff < 15 and own_speed > target_speed:
    #                     return "OVERTAKEN_L"
    #                 else:
    #                     return "OVERTAKEN_R"
    #         elif (15 <= heading_diff < 45 or 285 < heading_diff <= 315) and own_speed >= target_speed + 2:
    #             return "CROSS_BACK_A"      
            
    #         elif 15 <= heading_diff < 100 or 260 < heading_diff <= 345:
    #             return "CROSS_BACK"
            
    #         elif 100 <= heading_diff < 165 or 195 < heading_diff <= 260:
    #             return "CROSS_FRONT"
    #     else:
    #         return "SAFE"
    
    def colreg_check(self, own_xy, target_xy, own_heading, target_heading, own_speed, target_speed, azimuth, tcpa, dcpa):
        heading_diff = (target_heading - own_heading + 360) % 360
        azimuth = (azimuth - own_heading + 360) % 360
        own_uv = (own_speed * np.sin(np.radians(own_heading)), own_speed * np.cos(np.radians(own_heading)))
        target_uv = (target_speed * np.sin(np.radians(target_heading)), target_speed * np.cos(np.radians(target_heading)))
        coli_situ = calculate_collision_angle(own_xy, target_xy, own_uv, target_uv, 200/unit_to_meter)
        coli_situ2 = calculate_collision_angle(own_xy, target_xy, own_uv, target_uv, 100/unit_to_meter)
        if dcpa < SafeDCPA and 0 < tcpa < 10:
            if 165 <= heading_diff <= 195:
                return "HEADON"
            elif 0 <= heading_diff < 15 or 345 < heading_diff <= 360:
                if 270 <= azimuth <= 360  or 0 < azimuth <= 90:
                    if 345 < heading_diff < 360 and own_speed < target_speed:
                        return "OVERTAKING_R"
                    else:
                        return "OVERTAKING_L"
                else:
                    if 0 < heading_diff < 15 and own_speed > target_speed:
                        return "OVERTAKEN_L"
                    else:
                        return "OVERTAKEN_R"
            elif (15 <= heading_diff < 45 or 285 < heading_diff <= 315) and own_speed >= target_speed + 2 and (coli_situ == "front" or coli_situ2 == "front"):
                return "CROSS_BACK_A"      
            
            elif 15 <= heading_diff < 100 or 260 < heading_diff <= 345 and (coli_situ == "front" or coli_situ2 == "front"):
                return "CROSS_BACK"
            
            elif 100 <= heading_diff < 165 or 195 < heading_diff <= 260:
                return "CROSS_FRONT"
        else:
            return "SAFE"

    def colreg_domain(self, ownship, target_row):
        if target_row['t_idx'] not in self.colreg_domain_mem or self.colreg_domain_mem[target_row['t_idx']] == "SAFE":
            # coli_situ = self.colreg_check(ownship.heading, target_row['heading'], ownship.abs_speed, target_row['speed'], target_row['azimuth'], target_row['tcpa'], target_row['dcpa'])
            coli_situ = self.colreg_check((0, 0), (target_row['x'], target_row['y']), ownship.heading, target_row['heading'], ownship.abs_speed, target_row['speed'], target_row['azimuth'], target_row['tcpa'], target_row['dcpa'])
            self.colreg_domain_mem[target_row['t_idx']] = coli_situ
        else:
            coli_situ = self.colreg_domain_mem[target_row['t_idx']]

        # print("coli_situ", coli_situ)
        target_row['fore'] = MyDomain['fore']
        target_row['aft'] = MyDomain['aft']

        if coli_situ == "HEADON":   
            target_row['starb'] = MyDomain['starb']
            target_row['port'] = MyDomain['port']
        elif coli_situ in ["OVERTAKING_L", "OVERTAKEN_L"]:
            target_row['starb'] = MyDomain['port']
            target_row['port'] = MyDomain['starb']
        elif coli_situ in ["OVERTAKING_R", "OVERTAKEN_R"]:
            target_row['starb'] = MyDomain['starb']
            target_row['port'] = MyDomain['port']
        else:
            target_row['starb'] = MyDomain['starb']
            target_row['port'] = MyDomain['port']
            
        return target_row


    def init_colreg_domain(self, ownship, target_df):     
        target_idx_set = set(target_df['t_idx'])
        self.colreg_domain_mem = {k: v for k, v in self.colreg_domain_mem.items() if k in target_idx_set}
        target_df = target_df.apply(lambda ts: self.colreg_domain(ownship, ts), axis=1)
        return target_df


    def dynamic_domain(self, OS, TS, MaxTurn, SimTime, MyStaticDomain, MyDomain):
        COG_A = OS.heading
        SOG_A = OS.abs_speed
        COG_B = TS['heading']
        SOG_B = TS['speed']
        
        v_x0 = SOG_A * np.sin(np.radians(COG_A))  # 本船速度在xy的分量
        v_y0 = SOG_A * np.cos(np.radians(COG_A))
        
        v_xT = SOG_B * np.sin(np.radians(COG_B))  # 来船速度在xy的分量
        v_yT = SOG_B * np.cos(np.radians(COG_B))
        
        v_xR = (v_xT - v_x0) * np.cos(np.radians(COG_B)) - (v_yT - v_y0) * np.sin(np.radians(COG_B))  # 相对速度在xy的分量
        v_yR = (v_xT - v_x0) * np.sin(np.radians(COG_B)) + (v_yT - v_y0) * np.cos(np.radians(COG_B))
        
        domain_y = (v_yR) / MaxTurn * 90 * SimTime / 3
        domain_x = (v_xR) / MaxTurn * 90 * SimTime / 3  # 动态船舶领域变化太大了md，改小一点免得瞎跑
        
        if v_yR <= 0:
            pass
            # TS['aft'] = TS['aft'] - domain_y if TS['aft'] - domain_y > 150 else 150
            # TS['fore'] = TS['fore'] + domain_y if TS['fore'] + domain_y > 150 else 150
        else:
            TS['fore'] = TS['fore'] + domain_y if TS['fore'] + domain_y > 150 else 150
            # TS['aft'] = TS['aft'] - domain_y if TS['aft'] - domain_y > 150 else 150
        
        if v_xR <= 0:
            TS['port'] = TS['port'] - domain_x if TS['port'] - domain_x > MyStaticDomain['port'] else MyStaticDomain['port']
        else:
            TS['starb'] = TS['starb'] + domain_x if TS['starb'] + domain_x > MyStaticDomain['starb'] else MyStaticDomain['starb']
        
        if TS['fore'] > MyDomain['fore'] * 1.5:
            TS['fore'] = MyDomain['fore'] * 1.5
        if TS['aft'] > MyDomain['aft'] * 1.5:
            TS['aft'] = MyDomain['aft'] * 1.5
        
        return TS

                
    def do_path_plan(self, ownship, target_df, src_point, dst_point, maxInter):
        # print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        # print("src_point", src_point)
        # print("dst_point", dst_point)
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        path = []
        diffCourse = list(np.linspace(-MaxTurn,MaxTurn,apf_num))
        dst_point = self.cap_goal((0, 0), dst_point, src_point)
        own_p_on_seg = closest_point_on_line((0, 0), src_point, dst_point)
        
        target_df = target_df[target_df['distance'] <= ObstacleDetect * 2]
        
        # Apply Kalman filter to smooth target headings (fixes radar lag issues)
        # Pass ownship object for heading change compensation
        target_df = self.apply_heading_filter(target_df, ownship)
        
        # target_df = self.init_ship_domain(target_df)
        target_df = self.init_colreg_domain(ownship, target_df)
        target_df = target_df.apply(lambda ts: self.dynamic_domain(ownship, ts, MaxTurn, SimTime, MyStaticDomain, MyDomain), axis=1)
        target_df = self.init_static_domain(target_df)

        # for the rows withe the same values in self.s_domain_tidx in  t_idx column, set the values of theiir fore, aft, starb, port columns to MyStaticDomain['fore']
        target_df.loc[target_df['t_idx'].isin(self.s_domain_tidx), ['fore', 'aft', 'starb', 'port']] = MyStaticDomain['fore']

        print("plan_speed", self.plan_speed)
        # print("prioritized_t_idx", self.prioritized_t_idx)s
        print(ownship)
        ownship.set_plan_speed(self.plan_speed)
        # print(target_df)

        # Check if there are active threats 检测是否有碰撞风险
        has_threats = self.has_active_threats(target_df)
        print(f"Active threats detected: {has_threats}")
        
        # If no threats, modify the ownship heading to align with global path for faster return 若无威胁，调整本船航向与全局路径一致以加快返回速度
        original_heading = ownship.heading
        heading_modified = False
        if not has_threats:
            path_heading = self.calculate_path_segment_heading(src_point, dst_point)
            if self.is_significantly_off_path(original_heading, path_heading):
                print(f"No threats detected and significantly off path. Aligning heading from {original_heading:.1f}° to path direction {path_heading:.1f}°")
                ownship.heading = path_heading
                heading_modified = True
            else:
                print(f"No threats detected but heading {original_heading:.1f}° is close to path direction {path_heading:.1f}°")

        for i in range(maxInter):
            if not target_df.empty:
                target_df['x'] += target_df['u'] * SimTime
                target_df['y'] += target_df['v'] * SimTime
                target_df['tcpa'], target_df['dcpa'] = zip(*target_df.apply(lambda ts: cal_DCPA_TCPA((ts['u'], ts['v']), ownship.own_speed_uv, (ts['x'], ts['y']), ownship.own_rel_pos_xy), axis=1))

            PotentialValue = []
            for i in range(apf_num):
                SearchCourse = diffCourse[i] + ownship.heading
                temp_os_x = ownship.own_rel_pos_xy[0] + ownship.abs_speed * SimTime * np.sin(np.radians(SearchCourse))
                temp_os_y = ownship.own_rel_pos_xy[1] + ownship.abs_speed * SimTime * np.cos(np.radians(SearchCourse))
                tmp_poptential = self.cal_all_potential((temp_os_x, temp_os_y), dst_point, target_df, src_point, dst_point)
                if i-(apf_num+1)/2-1>=0:
                    tmp_poptential += 0
                PotentialValue.append(tmp_poptential)
            # get index of the minimum value
            min_po_index = PotentialValue.index(min(PotentialValue))
            ownship.step_update(diffCourse[min_po_index], SimTime)
            path.append(ownship.own_rel_pos_xy)

        capped_path = []
        for p in path:          
            if check_point_on_seg(p, own_p_on_seg, dst_point):
                # check if p's distance to the last point in capped_path is less than 1.5* ReachRadius
                capped_path.append(p)    
            else:
                break  
    
        capped_path.append(dst_point)
        capped_path = smooth_last_segment(capped_path)
        
        # Restore original heading if it was modified
        if heading_modified:
            ownship.heading = original_heading
            print(f"Restored original heading: {original_heading:.1f}°")
        
        return capped_path
    
