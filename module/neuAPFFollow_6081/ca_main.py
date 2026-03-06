#!/usr/bin/env python3
# filepath: c:\Users\ashen\projects\neu_apf\neu_apf\publisher_script.py
import sys
import os
import time
# Add the parent directory to the Python path
import collision_avoidance
import redis_interface

class PathPlanningScript:
    def __init__(self):
        self.sim_speed = 1
        self.redis_conn = redis_interface.init_redis()
        
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
        
    # def clean_path(self):
    #     self.redis_conn.hdel("Navi", "LPath")

    def toggle_anti_zigzag(self):
        """Toggle anti-zigzag behavior on/off"""
        self.enable_anti_zigzag = not self.enable_anti_zigzag
        self.pathplanner.set_anti_zigzag_enabled(self.enable_anti_zigzag)
        print(f"Anti-zigzag behavior {'enabled' if self.enable_anti_zigzag else 'disabled'}")

    def process_path(self):
        # redis_interface.reset_stop(self.redis_conn)
        # navi_state = redis_interface.get_navi_state(self.redis_conn)
        rc_state = redis_interface.get_rc_state(self.redis_conn)
        # self.pathplanner.set_auto_ca(navi_state)
        # self.sim_speed = redis_interface.get_sim_speed(self.redis_conn) 
        ownship = redis_interface.read_ownship(self.redis_conn) # 本船
        target_df = redis_interface.read_target_data(self.redis_conn, ownship) # 如何区分目标船和障碍船
        # gp = redis_interface.read_global_path(self.redis_conn) # 全局(两个经纬度坐标点)
        gp = redis_interface.read_follow_path(self.redis_conn) # 读取跟随路径

        default_speed = redis_interface.get_default_speed(self.redis_conn)
        self.pathplanner.update(gp, default_speed, rc_state)
        reached = self.pathplanner.do_path_plan(ownship, target_df)
        if reached:
            self.clean_path()
        #     redis_interface.set_stop(self.redis_conn)
        # else:
        redis_interface.write_local_path(self.redis_conn, self.pathplanner.path_str)
        print(f'Path updated: {self.pathplanner.path_str[:30]}...')  # Print abbreviated path for logging

    def stop(self):
        print("Stopping path planning script...")
        self.running = False


    def run(self):
        print("Path planning script started.")
        self.running = True   # ✅ 运行标志

        try:
            while self.running:
                start_time = time.time()

                self.process_path()

                sleep_time = max(
                    0,
                    (1.0 / self.sim_speed) - (time.time() - start_time)
                )
                time.sleep(sleep_time)

        except Exception as e:
            print(f"Path planning script error: {e}")

        finally:
            # self.clean_path()
            pass
            print("Path planning script terminated.")
    
    # # 死循环
    # def run(self):
    #     try:
    #         print("Path planning script started. Press Ctrl+C to stop.")
    #         while True:
    #             # try:
    #             start_time = time.time()
    #             self.process_path()
    #             # Calculate sleep time to maintain desired frequency
    #             sleep_time = max(0, (1.0/self.sim_speed) - (time.time() - start_time))
    #             time.sleep(sleep_time)
    #             # except Exception as e:
    #             #     print(f"Error during path processing: {e}")
    #     except KeyboardInterrupt:
    #         print("Stopping path planning script...")
    #         self.clean_path()
    #     finally:
    #         print("Path planning script terminated.")

 


    

def main():
   
    script = PathPlanningScript()
    script.run()

if __name__ == '__main__':
    main()