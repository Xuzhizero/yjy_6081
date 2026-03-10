#!/usr/bin/env python3
# filepath: c:\Users\ashen\projects\neu_apf\neu_apf\publisher_script.py
import sys
import os
import time
import collision_avoidance
import redis_interface   


class PathPlanningScript:
    def __init__(self):
        self.sim_speed = 1
        self.redis_conn = redis_interface.init_redis()
        self.Auto_CA_sw = 1
        self.status = "running"
        self.has_deleted_lpath = False

        
        # Heading filter configuration
        self.enable_heading_filter = False  # Change this to False to disable the filter
        
        # Anti-zigzag configuration
        self.enable_anti_zigzag = False # Change this to False to always use new path (original behavior)
        
        # === FILTER INTENSITY PRESETS ===
        # Choose one of these presets or customize your own:
        
        # PRESET 1: Very Smooth (heavy filtering, slow response)
        # filter_config = {
        #     'process_noise': 0.2, 'measurement_noise': 60.0, 'jump_threshold': 15.0, 'responsiveness': 'very_smooth'
        # }
        
        # PRESET 2: Smooth (moderate filtering)  
        # filter_config = {
        #     'process_noise': 0.5, 'measurement_noise': 40.0, 'jump_threshold': 25.0, 'responsiveness': 'smooth'
        # }
        
        # PRESET 3: Balanced (default - good compromise)
        filter_config = {
            'process_noise': 1.0, 'measurement_noise': 25.0, 'jump_threshold': 45.0, 'responsiveness': 'balanced'
        }
        
        # PRESET 4: Responsive (light filtering, quick response)
        # filter_config = {
        #     'process_noise': 2.0, 'measurement_noise': 15.0, 'jump_threshold': 60.0, 'responsiveness': 'responsive'
        # }
        
        # PRESET 5: Very Responsive (minimal filtering, almost raw data)
        # filter_config = {
        #     'process_noise': 4.0, 'measurement_noise': 8.0, 'jump_threshold': 90.0, 'responsiveness': 'very_responsive'
        # }
        
        self.pathplanner = collision_avoidance.PathPlanning(
            enable_heading_filter=self.enable_heading_filter,
            filter_config=filter_config,
            enable_anti_zigzag=self.enable_anti_zigzag
        )
        
        print(f"PathPlanning initialized with:")
        print(f"  - Heading filter: {'enabled' if self.enable_heading_filter else 'disabled'}")
        if self.enable_heading_filter:
            print(f"  - Filter preset: {filter_config['responsiveness']}")
        print(f"  - Anti-zigzag: {'enabled' if self.enable_anti_zigzag else 'disabled (always use new path)'}")
        
    def clean_path(self):
        self.redis_conn.hdel("Navi", "LPath")

    def toggle_anti_zigzag(self):
        """Toggle anti-zigzag behavior on/off"""
        self.enable_anti_zigzag = not self.enable_anti_zigzag
        self.pathplanner.set_anti_zigzag_enabled(self.enable_anti_zigzag)
        print(f"Anti-zigzag behavior {'enabled' if self.enable_anti_zigzag else 'disabled'}")

    def process_path(self):
        new_auto_ca = redis_interface.read_Auto_CA_sw(self.redis_conn)
        new_status = redis_interface.read_status_bytes(self.redis_conn)
        if new_auto_ca != self.Auto_CA_sw:
            print(f"🔄 Auto_CA_sw changed: {self.Auto_CA_sw} → {new_auto_ca}")
        self.Auto_CA_sw = new_auto_ca
        if new_status != self.status:
            print(f"🔄 status changed: {self.status} → {new_status}")
        self.status = new_status

        can_run = (self.Auto_CA_sw == 1 and self.status == "running")
        if not can_run:
            if not self.has_deleted_lpath:
                self.clean_path()
                self.has_deleted_lpath = True
                print(f"⛔ 跳过规划：status={self.status}, Auto_CA_sw={self.Auto_CA_sw}")
                # redis_interface.write_speed_control_mode(self.redis_conn, 0) # 新加0305,如果不及时归0,下一次开启时redis就只会保留上一次遗留的值(2)，导致如果这时开启循迹会误判进入航速控制模式
            return   # ⬅️ 硬门控，后面不再执行
        self.has_deleted_lpath = False

        # redis_interface.reset_stop(self.redis_conn)
        # navi_state = redis_interface.get_navi_state(self.redis_conn)
        rc_state = redis_interface.get_rc_state(self.redis_conn)
        # self.pathplanner.set_auto_ca(navi_state)
        # self.sim_speed = redis_interface.get_sim_speed(self.redis_conn) 
        ownship = redis_interface.read_ownship(self.redis_conn) # 本船
        target_df = redis_interface.read_target_data(self.redis_conn, ownship) # 他船
        gp = redis_interface.read_global_path(self.redis_conn) # 全局
        default_speed = redis_interface.get_default_speed(self.redis_conn)
        self.pathplanner.update(gp, default_speed, rc_state)
        reached = self.pathplanner.do_path_plan(ownship, target_df)
        if reached:
            self.clean_path()
        redis_interface.write_speed_control_mode(self.redis_conn, self.pathplanner.speed_control_str) # 变速避碰开关，启动变速避碰时开启
        # redis_interface.write_speed_control_mode(self.redis_conn, self.pathplanner.course_control_str)
        redis_interface.write_local_path(
            self.redis_conn,
            self.pathplanner.path_str
        )
        print(f'Path updated: {self.pathplanner.path_str[:30]}...')



        # #     redis_interface.set_stop(self.redis_conn)
        # # else:
        # redis_interface.write_local_path(self.redis_conn, self.pathplanner.path_str)
        # print(f'Path updated: {self.pathplanner.path_str[:30]}...')  # Print abbreviated path for logging

    def run(self):
        try:
            print("Path planning script started. Press Ctrl+C to stop.")
            while True:
                # try:
                start_time = time.time()
                self.process_path()
                # Calculate sleep time to maintain desired frequency
                sleep_time = max(0, (1.0/self.sim_speed) - (time.time() - start_time))
                time.sleep(sleep_time)
                # except Exception as e:
                #     print(f"Error during path processing: {e}")
        except KeyboardInterrupt:
            print("Stopping path planning script...")
            # redis_interface.write_speed_control_mode(self.redis_conn, 0) # 新加0305
            self.clean_path()
        finally:
            print("Path planning script terminated.")
            # redis_interface.write_speed_control_mode(self.redis_conn, 0) # 新加0305
            

def main():
    script = PathPlanningScript()
    script.run()

if __name__ == '__main__':
    main()