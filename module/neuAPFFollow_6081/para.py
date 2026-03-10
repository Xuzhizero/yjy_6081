# # ship profile
# unit_to_meter=1000
# unit_to_meter=1852
unit_to_meter=1
ship_turn_angle=30
ship_safe_distance=400/unit_to_meter
ship_tolerance=250/unit_to_meter # goal reach criterum
# # sim
init_lonlat = (114.43066862436631, 21.134829191418977)
speed_scaler = 1
# init_speed = 6.5 * 0.5144444 / unit_to_meter
init_speed = 0.005
init_heading = 0
# # plotting
axis_lim=4
gap=1
chess_grid=(300, 300)
safe_distance=0.2
# # ip 
redis_host = '127.0.0.1'
udp_incoming_host = '127.0.0.1'
udp_targeting_host = '127.0.0.1'