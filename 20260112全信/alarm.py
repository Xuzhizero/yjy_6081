import numpy as np
from algo_utility import convert_lonlat_to_abs_xy

# TODO: 1. priodity of alarm based on colregs: headon > crossing > overtaking > standon. 2. number adjustment based on colregs


OVERTAKING_SIT = "OVERTAKING_SIT"
STANDON_SIT = "STANDON_SIT"
HEADON_SIT = "HEADON_SIT"
GIVEWAY_SIT = "GIVEWAY_SIT"
SAFE_SIT = "SAFE_SIT"

class ShipDomain:
    def __init__(self, domainlength=100, scale=3):
        self.fore = 9.9 * domainlength * scale
        self.aft = 9.9 * domainlength * scale
        self.port = 1.5 * domainlength * scale
        self.starb = 1.05 * domainlength * scale

    def in_domain(self, own_xy, target_xy, target_course):
        d_x = own_xy[0] - target_xy[0]
        d_y = own_xy[1] - target_xy[1]
        angle = np.arctan2(d_x, d_y) * 180 / np.pi
        angle = (angle + 360) % 360
        target_course = (target_course + 360) % 360
        angle -= target_course
        angle = (angle + 360.0) % 360.0
        tcourse = target_course
        tmp_d_x = d_x
        d_x = d_y * np.sin(-tcourse * np.pi / 180) + d_x * np.cos(-tcourse * np.pi / 180)
        d_y = -tmp_d_x * np.sin(-tcourse * np.pi / 180) + d_y * np.cos(-tcourse * np.pi / 180)
        if 0 <= angle <= 90:
            return (d_y / self.fore) ** 2 + (d_x / self.starb) ** 2 <= 1
        elif 90 < angle <= 180:
            return (d_y / self.aft) ** 2 + (d_x / self.starb) ** 2 <= 1
        elif 180 < angle <= 270:
            return (d_y / self.aft) ** 2 + (d_x / self.port) ** 2 <= 1
        elif 270 < angle < 360:
            return (d_y / self.fore) ** 2 + (d_x / self.port) ** 2 <= 1
        return False
          

class Alarm:
    def __init__(self, domain_scale=3):
        self.SafeDCPA = 0.107991 # 200M to nautical miles
        self.CritDCPA1 = 0.053996 # 100M to nautical miles
        self.CritDCPA = 0.0269978 # 50M to nautical miles
        self.SafeTCPA = 10 # minutes
        self.colregs_rel_theta = [22.5, 90, 112.5, 247.5, 270, 337.5]
        self.colregs_ts_theta = [67.5, 90, 157.5, 202.5, 270, 292.5, 360]
        # self.colregs_rule = [
        #                         [OVERTAKING_SIT, STANDON_SIT, STANDON_SIT, HEADON_SIT, GIVEWAY_SIT, GIVEWAY_SIT, OVERTAKING_SIT],
        #                         [OVERTAKING_SIT, STANDON_SIT, STANDON_SIT, HEADON_SIT, GIVEWAY_SIT, GIVEWAY_SIT, OVERTAKING_SIT],
        #                         [SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT],
        #                         [STANDON_SIT, HEADON_SIT, HEADON_SIT, HEADON_SIT, HEADON_SIT, STANDON_SIT, STANDON_SIT],
        #                         [SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT],
        #                         [OVERTAKING_SIT, STANDON_SIT, STANDON_SIT, HEADON_SIT, SAFE_SIT, SAFE_SIT, OVERTAKING_SIT]
        #                     ]
        self.colregs_rule = [
                                [OVERTAKING_SIT, STANDON_SIT, STANDON_SIT, HEADON_SIT, GIVEWAY_SIT, GIVEWAY_SIT, OVERTAKING_SIT],
                                [OVERTAKING_SIT, STANDON_SIT, STANDON_SIT, HEADON_SIT, GIVEWAY_SIT, GIVEWAY_SIT, OVERTAKING_SIT],
                                [SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT],
                                [STANDON_SIT, HEADON_SIT, HEADON_SIT, HEADON_SIT, HEADON_SIT, STANDON_SIT, STANDON_SIT],
                                [SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT, SAFE_SIT],
                                [OVERTAKING_SIT, STANDON_SIT, STANDON_SIT, HEADON_SIT, SAFE_SIT, SAFE_SIT, OVERTAKING_SIT]
                            ]
        self.domain_scale = domain_scale

    def cpa_filter(self, DCPA, TCPA):
        if DCPA >= self.SafeDCPA or TCPA >= self.SafeTCPA or TCPA < 0:
            return True
        else:
            return False
        
    def cal_psi1(self, dX, dY):
        if dX > 0 and dY > 0:  # 1st quadrant
            return np.arctan(dX / dY)
        if dX > 0 and dY < 0:  # 2nd quadrant
            return (np.pi / 2 - np.arctan(dY / dX))
        if dX < 0 and dY < 0:  # 3rd quadrant
            return (np.pi + np.arctan(dX / dY))
        if dX < 0 and dY > 0:  # 4th quadrant
            return (1.5 * np.pi - np.arctan(dY / dX))
        if dX == 0 and dY > 0:  # Y+
            return 0
        if dX == 0 and dY < 0:  # Y-
            return np.pi
        if dX > 0 and dY == 0:  # X+
            return np.pi / 2
        if dX < 0 and dY == 0:  # X-
            return 3 * np.pi / 2
        if dX == 0 and dY == 0:  # Origin
            return 0
        return 0

    def FindCollisionRegion(self, alpha_T):
        alpha_T = int(alpha_T + 360) % 360
        if alpha_T > self.colregs_rel_theta[0] and alpha_T <= self.colregs_rel_theta[1]:
            return 1
        if alpha_T > self.colregs_rel_theta[1] and alpha_T <= self.colregs_rel_theta[2]:
            return 2
        if alpha_T > self.colregs_rel_theta[2] and alpha_T <= self.colregs_rel_theta[3]:
            return 3
        if alpha_T > self.colregs_rel_theta[3] and alpha_T <= self.colregs_rel_theta[4]:
            return 4
        if alpha_T > self.colregs_rel_theta[4] and alpha_T < self.colregs_rel_theta[5]:
            return 5
        if alpha_T >= self.colregs_rel_theta[5] or alpha_T <= self.colregs_rel_theta[0]:
            return 0
        return 0

    def FindTSCourseRegion(self, course):
        course = int(course + 360) % 360
        if course > self.colregs_ts_theta[0] and course <= self.colregs_ts_theta[1]:
            return 1
        if course > self.colregs_ts_theta[1] and course <= self.colregs_ts_theta[2]:
            return 2
        if course > self.colregs_ts_theta[2] and course <= self.colregs_ts_theta[3]:
            return 3
        if course > self.colregs_ts_theta[3] and course <= self.colregs_ts_theta[4]:
            return 4
        if course > self.colregs_ts_theta[4] and course < self.colregs_ts_theta[5]:
            return 5
        if course > self.colregs_ts_theta[5] and course < self.colregs_ts_theta[6]:
            return 6
        if course <= self.colregs_ts_theta[0]:
            return 0
        return 0

    def colregs_filter(self, own_xy, target_xy, own_course, target_course):
        alpha_T = self.cal_psi1(own_xy[0]- target_xy[0], own_xy[1] - target_xy[1])
        alpha_T -= np.radians(own_course)
        collisionRegion = self.FindCollisionRegion(np.degrees(alpha_T))
        tscourseRegion = self.FindTSCourseRegion(target_course - own_course)
        print("collisionRegion: ", collisionRegion)
        print("tscourseRegion: ", tscourseRegion)
        if tscourseRegion > 6:
            # return 'SAFE_SIT'
            return False
        if collisionRegion > 5:
            # return 'SAFE_SIT'
            return False
        return SAFE_SIT!=self.colregs_rule[collisionRegion][tscourseRegion]
    

    def determine_crossing(self, own_xy, own_uv, target_xy, target_uv):
        x0, y0 = own_xy
        u0, v0 = own_uv 
        x1, y1 = target_xy
        u1, v1 = target_uv
        # if ownship is not moving
        if u0 == 0 and v0 == 0:
            dx = x1 - x0
            dy = y1 - y0
            du = u1
            dv = v1
            
            if dx == 0 and dy == 0:
                return "parallel_f"  # When ships are at the same position, consider front
            
            # Determine front/back based on target's movement relative to position difference
            dot_product = dx * du + dy * dv
            if dx * dv - du * dy == 0:
                if du != 0:
                    t = -dx / du
                elif dv != 0:
                    t = -dy / dv
                else:
                    return "parallel_f" if dot_product >= 0 else "parallel_b"
                
                if t >= 0:
                    return "parallel_f" if dot_product >= 0 else "parallel_b"
            
            return "parallel_f" if dot_product >= 0 else "parallel_b"
        else:
            dx = x1 - x0
            dy = y1 - y0
            du = u1 - u0
            dv = v1 - v0
            
            # Project relative position onto own ship's velocity direction
            dot_product = dx * u0 + dy * v0
            
            cross_S_Vrel = u0 * dv - v0 * du
            cross_D_S = dx * v0 - dy * u0
            
            if cross_S_Vrel == 0:
                if cross_D_S == 0:
                    dot_DS = dx * u0 + dy * v0
                    dot_VS = du * u0 + dv * v0
                    S_squared = u0**2 + v0**2
                    
                    t0 = dot_DS / S_squared
                    k = dot_VS / S_squared
                    
                    if k == 0:
                        return "parallel_f" if dot_product >= 0 else "parallel_b"
                    
                    if (t0 < 0 and k > 0) or (t0 > 0 and k < 0):
                        if k > 0:
                            return "front"
                        else:
                            return "back"
                    else:
                        return "parallel_f" if dot_product >= 0 else "parallel_b"
                else:
                    return "parallel_f" if dot_product >= 0 else "parallel_b"
            else:
                t = cross_D_S / cross_S_Vrel
                if t >= 0:
                    if u0 != 0:
                        s = (dx + du * t) / u0
                    else:
                        s = (dy + dv * t) / v0
                    
                    if s > 0:
                        return "front"
                    elif s < 0:
                        return "back"
                    else:
                        return "front"
                else:
                    return "parallel_f" if dot_product >= 0 else "parallel_b"

    def hitpoint_filter(self, crossing_type, relative_course, coli_type, dcpa):
        # if 90 <= np.abs(relative_course) <= 125 and crossing_type == "back" and dcpa > self.CritDCPA and coli_situ == "front_p":
        if 45 <= np.abs(relative_course) <= 135 and crossing_type == "back" and coli_type != "front" and dcpa > self.CritDCPA:
            return True
        return False

    def pred_paralle_filter(self, dcpa, target_rel_course, distance):
        if self.CritDCPA < dcpa < self.SafeDCPA and (0<target_rel_course<5 or 355<target_rel_course<360 or 175<target_rel_course<185) and distance < 300:
            return False
        return True

    def domain_filter(self, own_xy, target_xy, target_course, target_size=100):
        shipDomain = ShipDomain(target_size, self.domain_scale)
        return shipDomain.in_domain(own_xy, target_xy, target_course)
    
    def distance_classifer(self, distance, tcpa, target_speed):
        # if distance is greater that 1 nauctical mile smaller than 3 nauctical mile
        print("distance: ", distance)
        # if ((1852 < distance or 5 < tcpa < 10 ) and distance < 5556 and target_speed > 0.51444) or (target_speed <= 0.51444 and 1296 < distance < 1852):
        if ((1852 < distance or 5 < tcpa < 10 ) and distance < 5556 and target_speed > 0.51444) or (target_speed <= 0.51444 and 900 < distance < 1200):
            return 1
        elif (distance <= 1852 and 0 < tcpa <= 5 and target_speed > 0.51444) or (target_speed <= 0.51444 and distance <= 900):
            return 2
        else:
            return 0

    def crossed_filter(self, own_xy, target_xy, own_uv, target_uv, relative_course, dcpa):
        return ((self.pass_course(own_xy, target_xy, own_uv, target_uv) and dcpa>self.CritDCPA) \
                or (self.pass_course(target_xy, own_xy, target_uv, own_uv) and dcpa>self.CritDCPA1)) \
                      and (((np.abs(relative_course) > 15 and np.abs(relative_course) < 165) or dcpa>self.CritDCPA1)) \
                        and np.abs(relative_course) != 0 and np.abs(relative_course) != 180

    def pass_course(self, pos1, pos2, vec1, vec2):
        x0, y0 = pos1
        x1, y1 = pos2
        u0, v0 = vec1
        u1, v1 = vec2
        cross_pos = (x1 - x0) * v0 - (y1 - y0) * u0
        cross_target = u1 * v0 - v1 * u0
        if cross_target == 0:
            return False
        return cross_pos * cross_target > 0


    def calculate_collision_angle(self, own_xy, own_uv, target_xy, target_uv, R):
        """
        Calculate collision parameters between a ship and a target.
        
        Parameters:
        -----------
        x0, y0 : float
            Ship position coordinates
        u0, v0 : float
            Ship velocity components
        x1, y1 : float
            Target position coordinates
        u1, v1 : float
            Target velocity components
        R : float
            Ship radius or collision distance threshold
        
        Returns:
        --------
        dict
            Contains collision status, angle, and time to collision
            The collision angle phic indicates where the target hits the ship:
            - 0°≤phic<90° or 270°<phic<360°: front half of ship
            - 90°<phic<270°: back half of ship
            - phic=90° or 270°: middle of ship
        """
        x0, y0 = own_xy
        u0, v0 = own_uv 
        x1, y1 = target_xy
        u1, v1 = target_uv
        # Important parameters
        w0 = np.sqrt(u0**2 + v0**2)
        thet0 = np.arctan2(v0, u0)
        
        w1 = np.sqrt(u1**2 + v1**2)
        thet1 = np.arctan2(v1, u1)
        
        p = x0 - x1
        q = y0 - y1
        
        psi = np.arctan2(v0 - v1, u0 - u1)
        sinpsi = np.sin(psi)
        cospsi = np.cos(psi)
        
        # Judgment of collision
        hbar = (-p * sinpsi + q * cospsi) / R
        d = p * cospsi + q * sinpsi
        
        collision = (abs(hbar) <= 1) and (d < 0)
        
        if not collision:
            return False
        
        # Analysis of collision
        a = w1**2 + w0**2 - 2*w0*w1*np.cos(thet1 - thet0)
        b = p*(u0 - u1) + q*(v0 - v1)
        c = p**2 + q**2 - R**2
        
        tc = (-b - np.sqrt(b**2 - a*c)) / a
        
        # Calculate collision point in ship's coordinate system
        xcstr = -p*np.cos(thet0) - q*np.sin(thet0) + \
                (w1*np.cos(thet1 - thet0) - w0)*tc
        ycstr = p*np.sin(thet0) - q*np.cos(thet0) + w1*np.sin(thet1 - thet0)*tc
        phic = np.arctan2(ycstr, xcstr)
        
        # Convert phic to degrees along clockwise direction
        if phic >= 0:
            phic_deg = -phic * 180/np.pi + 360
        else:
            phic_deg = -phic * 180/np.pi

        if 0 <= phic_deg <=90 or 270 <= phic_deg < 360:
            return "front"
        else:
            return "back"

    def coli_situation(self, own_xy, target_xy, own_uv):
        # determine if the target is in front of the own ship or behind the ownship 
        # if the target is in front of the own ship, return "front"
        # if the target is behind the own ship, return "back"
        
        # Unpack coordinates and velocity vectors
        x0, y0 = own_xy
        x1, y1 = target_xy
        u0, v0 = own_uv
        
        # If own ship isn't moving, we can't determine front/back based on heading
        if u0 == 0 and v0 == 0:
            return "front_p"  # Default to front when not moving
        
        # Calculate vector from own ship to target ship
        dx = x1 - x0
        dy = y1 - y0
        
        # Take dot product between own ship's velocity vector and 
        # the vector pointing to the target ship
        dot_product = u0 * dx + v0 * dy
        
        if dot_product >= 0:
            return "front_p"  # Target is in front of own ship's heading
        else:
            return "back_p"   # Target is behind own ship's heading

    def cal_alarm(self, dcpa, tcpa, target_lonlat, azimuth, own_course, distance, target_course, own_speed, target_speed, now_os_abs_xy, target_size=100):
        temp_alarm = self.distance_classifer(distance, tcpa, target_speed)
        if temp_alarm == 0:
            print("distance_classifer")
            return 0   
        
        if self.cpa_filter(dcpa, tcpa):
            print("cpa_filter")
            return 0
        
        # own_course = target_course - relative_course
        own_course = (own_course + 360) %360
        relative_course = target_course - own_course
        if relative_course > 180:
            relative_course -= 360
        print("relative_course: ", relative_course)
        # print("target_course: ", target_course)
        # print("own_course: ", own_course)
        target_xy = convert_lonlat_to_abs_xy(target_lonlat, 1)
        own_xy = now_os_abs_xy

        # if not self.colregs_filter(own_xy, target_xy, own_course, target_course):
        #     print("colregs_filter")
        #     return 0

        # if not self.pred_paralle_filter(dcpa, target_rel_course, distance):
        #     print("paralle_filter")
        #     return 0
    
        own_uv = own_speed * np.sin(np.radians(own_course)), own_speed * np.cos(np.radians(own_course)) 
        target_uv = target_speed * np.sin(np.radians(target_course)), target_speed * np.cos(np.radians(target_course))
        

        if self.crossed_filter(own_xy, target_xy, own_uv, target_uv, relative_course, dcpa) and dcpa>self.CritDCPA and target_speed > 0.514444:
            print("crossed_filter")
            return 0

        # crossing_type = self.determine_crossing(own_xy, own_uv, target_xy, target_uv)
        crossing_type = self.calculate_collision_angle(own_xy, own_uv, target_xy, target_uv, 200)
        print("crossing_type: ", crossing_type)
        
        # coli_situ = self.coli_situation(own_xy, target_xy, own_uv)
        # print("coli_situ:", coli_situ)
        
        coli_type = self.calculate_collision_angle(own_xy, own_uv, target_xy, target_uv, 100)
        print("coli_type", coli_type)

        if self.hitpoint_filter(crossing_type, relative_course, coli_type, dcpa):
            print("hitpoint_filter")
            return 0

        
        # if (0<tcpa<2 or (target_speed <= 0.51444 and distance <= 633)) \
        #     and (
        #             (dcpa<self.CritDCPA1 and \
        #                 (coli_type=="front" or coli_situ=="front_p") \
        #             ) \
        #          or dcpa<self.CritDCPA
        #         ):
        #     return 3        
        
        # if (0<tcpa<2 or (target_speed <= 0.51444 and distance <= 633)) \
        if (0<tcpa<2 or (target_speed <= 0.51444 and distance <= 500)) \
            and (
                    (dcpa<self.CritDCPA1 and coli_type=="front") \
                 or dcpa<self.CritDCPA
                ):
            return 3
        
        # if not self.domain_filter(own_xy, target_xy, target_course, target_size):
        #     print("domain_filter")
        #     return 0 

        return temp_alarm
        