import pyproj
import math
import numpy as np
from para import unit_to_meter

def khachiyan_algorithm(points, tolerance=0.01):
    """
    Find the minimum volume ellipse.
    input: points - (d x N) array where d is the number of dimensions and N is the number of points
    output: A - (d x d) matrix, the semi-axes of the ellipse
    """
    N, d = points.shape
    Q = np.column_stack((points, np.ones(N))).T
    err = 1.0 + tolerance
    u = np.ones(N) / N
    while err > tolerance:
        X = np.dot(np.dot(Q, np.diag(u)), Q.T)
        epsilon = 1e-5  # or some small number of your choice
        X += np.eye(X.shape[0]) * epsilon
        M = np.diag(np.dot(np.dot(Q.T, np.linalg.inv(X)), Q))
        jdx = np.argmax(M)
        step_size = (M[jdx] - d - 1) / ((d + 1) * (M[jdx] - 1))
        new_u = (1 - step_size) * u
        new_u[jdx] += step_size
        err = np.linalg.norm(new_u - u)
        u = new_u
    c = np.dot(points.T, u)
    matrix = np.dot(np.dot(points.T, np.diag(u)), points) - np.outer(c, c)
    matrix += np.eye(matrix.shape[0]) * epsilon
    A = np.linalg.inv(matrix) / d
    return A, c

def fit_ellipse(points, tolerance=0.01):
    """
    Fit an ellipse to a set of points.
    """
    A, c = khachiyan_algorithm(points)
    # Get the eigenvalues and eigenvectors of the matrix A
    vals, vecs = np.linalg.eig(A)
    # Compute the angle of rotation and the lengths of the semi-axes
    angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
    width, height = 1 / np.sqrt(vals)
    return c, width, height, angle
   

# calulate the norm of a vector
def norm(t):
    return np.sqrt(sum(x**2 for x in t))

# calulate the distance between own ship and target ship
def cal_distance(obj_pos, self_pos):
    rel_obj_pos = (obj_pos[0] - self_pos[0], obj_pos[1] - self_pos[1])
    return np.sqrt(rel_obj_pos[0]**2 + rel_obj_pos[1]**2)

def convert_lonlat_to_rel_xy(pos, center, u2m=unit_to_meter):
    lon, lat = pos
    center_lon, center_lat = center
    Proj = pyproj.Proj(proj='utm', zone=50, ellps='WGS84', preserve_units=True)

    x, y = Proj(lon, lat)
    center_x, center_y = Proj(center_lon, center_lat)

    # Calculate the relative coordinates with respect to the center
    x = x - center_x
    y = y - center_y

    # Convert the coordinates to the desired unit
    x = x / u2m
    y = y / u2m

    return x, y

def convert_dxy_to_lonlat(dxy, center, u2m=unit_to_meter):
    dx, dy = dxy
    dx = dx * u2m
    dy = dy * u2m
    center_lon, center_lat = center
    
    Proj = pyproj.Proj(proj='utm', zone=50, ellps='WGS84', preserve_units=True)
    x, y = Proj(center_lon, center_lat)
    x = x + dx
    y = y + dy
    lon, lat = Proj(x,  y, inverse=True)

    return lon, lat

def convert_lonlat_to_abs_xy(pos, u2m=unit_to_meter):
    lon, lat = pos
    Proj = pyproj.Proj(proj='utm', zone=50, ellps='WGS84', preserve_units=True)

    x, y = Proj(lon, lat)

    # Convert the coordinates to the desired unit
    x = x / u2m
    y = y / u2m

    return x, y

def convert_abs_xy_to_lonlat(pos, u2m=unit_to_meter):
    x, y = pos

    # Convert the coordinates to the desired unit
    x = x * u2m
    y = y * u2m

    Proj = pyproj.Proj(proj='utm', zone=50, ellps='WGS84', preserve_units=True)

    lon, lat = Proj(x,  y, inverse=True)

    return lon, lat

def get_lonlat_distance(pos1, pos2, u2m=unit_to_meter):
    lon1, lat1 = pos1
    lon2, lat2 = pos2
    geod = pyproj.Geod(ellps='WGS84')

    distance = geod.inv(lon1, lat1, lon2, lat2)[2]
    distance = distance / u2m
    return distance

def get_azimuth_angle(own_lonlat, target_lonlat):
    lon1, lat1 = own_lonlat
    lon2, lat2 = target_lonlat
    geod = pyproj.Geod(ellps='WGS84')

    azimuth = geod.inv(lon1, lat1, lon2, lat2)[0]
    return azimuth

def check_binary_value_at_position(number, position):
    # input prefix 0b 
    # Right shift the number 'position' times and isolate the least significant bit
    bit_value = (number >> position) & 1
    return bit_value

def cal_collision_angle(obj_speed, self_speed):
    obj_norm = np.linalg.norm(obj_speed)
    self_norm = np.linalg.norm(self_speed)

    if obj_norm == 0 or self_norm == 0:
        angle = np.nan  # or handle the zero vector case appropriately
    else:
        cos_angle = np.dot(obj_speed, self_speed) / (obj_norm * self_norm)
        # Ensure the value is within the valid range for arccos
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.degrees(np.arccos(cos_angle))
        
        # Determine the direction using the cross product
        cross_product = np.cross(obj_speed, self_speed)
        if cross_product < 0:
            angle = -angle  # Left side is negative

    return angle

def course_check(obj_speed, self_speed):
    # calulate the angle between two vectors
    angle = cal_collision_angle(obj_speed, self_speed)
    if angle == 180:
        return "HEADON"
    # if (-15 <= angle <= 15):
    #     return "OVERTAKING"
    # elif (angle >= 135 or angle <= -135):
    #     return "HEADON"
    # elif (-135 <= angle < -90):
    #     return "CROSSING_FRONT_RIGHT"
    # elif (90 < angle <= 135):
    #     return "CROSSING_FRONT_LEFT"
    else:
        return "CROSSING"
    

def collision_detection_circle(own_pos, target_pos, own_speed, target_speed, r):
    """
    check if a moving circle obstacle will collide with ownship (0, 0) in relative axis, 
    if yes, return the time of entrance and exit, if no collision, return 0, 0.
    """
    x1, y1 = target_pos[0] - own_pos[0], target_pos[1] - own_pos[1]
    u1, v1 = target_speed[0] - own_speed[0], target_speed[1] - own_speed[1]
    a = u1**2 + v1**2
    b = 2 * (x1 * u1 + y1 * v1)
    c = x1**2 + y1**2 - r**2
    delta = b**2 - 4 * a * c
    if delta < 0:
        return None, None
    t1 = (-b + np.sqrt(delta)) / (2 * a + 1e-10)
    t2 = (-b - np.sqrt(delta)) / (2 * a + 1e-10)

    if t1 < 0 or t2 < 0:
        return None, None
    return min(t1, t2), max(t1, t2)

def collision_detection_ellipse(own_pos, target_pos, own_speed, target_speed, angle, width, height):
    """
    check if a moving ellipse obstacle will collide with ownship (0, 0) in relative axis, 
    if yes, return the time of entrance and exit, if no collision, return 0, 0.
    """
    x1, y1 = target_pos[0] - own_pos[0], target_pos[1] - own_pos[1]
    u1, v1 = target_speed[0] - own_speed[0], target_speed[1] - own_speed[1]
    a = (u1 * np.cos(np.radians(angle)) + v1 * np.sin(np.radians(angle)))**2 / width**2 + (u1 * np.sin(np.radians(angle)) - v1 * np.cos(np.radians(angle)))**2 / height**2
    b = 2 * ((x1 * u1 + y1 * v1) * np.cos(np.radians(angle)) / width**2 + (x1 * u1 + y1 * v1) * np.sin(np.radians(angle)) / height**2)
    c = (x1 * np.cos(np.radians(angle)) / width)**2 + (y1 * np.sin(np.radians(angle)) / height)**2 - 1
    delta = b**2 - 4 * a * c
    if delta < 0:
        return 0, 0
    t1 = (-b + np.sqrt(delta)) / (2 * a + 1e-10) 
    t2 = (-b - np.sqrt(delta)) / (2 * a + 1e-10)
    return min(t1, t2), max(t1, t2)


# def cal_DCPA_TCPA(obj_speed, self_speed, obj_pos, self_pos):
#     # Calculate the relative speed and position
#     rel_speed = (obj_speed[0] - self_speed[0], obj_speed[1] - self_speed[1])
#     rel_pos = (obj_pos[0] - self_pos[0], obj_pos[1] - self_pos[1])
  
#     # Calculate the magnitude of the relative speed
#     rel_speed_mag = np.sqrt(rel_speed[0]**2 + rel_speed[1]**2)
        
#     # Calculate DCPA using the perpendicular distance formula
#     dcpa = np.abs(rel_pos[0] * rel_speed[1] - rel_pos[1] * rel_speed[0]) / (rel_speed_mag + 1e-10)
    
#     # Calculate the dot product of relative speed and relative position
#     dot_product = rel_speed[0] * rel_pos[0] + rel_speed[1] * rel_pos[1]
#     # Calculate TCPA
#     tcpa = -dot_product / (rel_speed_mag**2 + 1e-10) / 60
    
#     return dcpa, tcpa


def cal_DCPA_TCPA(obj_speed, self_speed, obj_pos, self_pos):
    rel_speed = (obj_speed[0] - self_speed[0], obj_speed[1] - self_speed[1])
    rel_pos = (obj_pos[0] - self_pos[0], obj_pos[1] - self_pos[1])
    
    rel_speed_mag = np.sqrt(rel_speed[0]**2 + rel_speed[1]**2)
    rel_pos_mag = np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)  # Current distance
    
    # Handle zero relative speed case
    if rel_speed_mag < 1e-10:
        dcpa = rel_pos_mag
        tcpa = float('inf')  # Undefined, use infinity or another indicator
    else:
        cross = rel_pos[0] * rel_speed[1] - rel_pos[1] * rel_speed[0]
        dcpa = np.abs(cross) / rel_speed_mag
        dot_product = rel_speed[0] * rel_pos[0] + rel_speed[1] * rel_pos[1]
        tcpa = -dot_product / (rel_speed_mag**2) / 60  # Adjusted unit handling
    
    return dcpa, tcpa


def perpendicular_distance(point, start, end):
    """
    Calculate the perpendicular distance from a point to a line segment.
    """
    point = np.array(point)
    start = np.array(start)
    end = np.array(end)
    
    if np.array_equal(start, end):
        return np.linalg.norm(point - start)
    
    line_vec = end - start
    point_vec = point - start
    line_len = np.linalg.norm(line_vec)
    line_unitvec = line_vec / line_len
    point_vec_scaled = point_vec / line_len
    t = np.dot(line_unitvec, point_vec_scaled)
    t = np.clip(t, 0, 1)
    nearest = start + t * line_vec
    distance = np.linalg.norm(point - nearest)
    return distance

# def find_current_segment_index(current_pos, path):
#     """
#     Determine the index of the "end" coordinate of the segment of the path the current position is on.
#     Return -1 if the current position is in front of the first node of the path.
#     """
#     if not path:
#         return -1
    
#     min_distance = float('inf')
#     current_segment_index = -1
    
#     for i in range(len(path) - 1):
#         start = np.array(path[i])
#         end = np.array(path[i + 1])
#         distance = perpendicular_distance(np.array(current_pos), start, end)
        
#         if distance < min_distance:
#             min_distance = distance
#             current_segment_index = i + 1
    
#     # Check if the current position is in front of the first node
#     first_node = np.array(path[0])
#     if np.linalg.norm(np.array(current_pos) - first_node) < min_distance:
#         return 0
    
#     return current_segment_index


def find_current_segment_index(pos, path, tol):

    # if path is empty, return -1
    if not path:
        print("Path is empty")
        return -1
    
    # if path has only one node, check if the current position is within the tolerance of the node, if yes return 1, else return 0
    if len(path) == 1:
        if np.linalg.norm(np.array(pos) - np.array(path[0])) <= tol:
            return 1
        return 0
    
    path = np.array(path)
    # basic parameters
    nm = path.shape
    Npoint = nm[0]
    nmax = Npoint - 1

    xs = path[:, 0]
    ys = path[:, 1]

    xp = pos[0]
    yp = pos[1]

    length = Npoint

    # nearest point
    ds = np.sqrt((xp - xs)**2 + (yp - ys)**2)
    nmin = np.argmin(ds)
    na = nmin

    xa = xs[na]
    ya = ys[na]

    # second nearest point
    if na == 0:
        nb = na + 1
    elif na == nmax:
        nb = na - 1
    else:
        d1 = np.sqrt((xp - xs[na - 1])**2 + (yp - ys[na - 1])**2)
        d2 = np.sqrt((xp - xs[na + 1])**2 + (yp - ys[na + 1])**2)

        if d2 <= d1:
            nb = na + 1
        else:
            nb = na - 1

    # back number
    n1 = min(na, nb)
    n2 = n1 + 1

    r1p = np.array([xp - xs[n1], yp - ys[n1]])
    r2p = np.array([xp - xs[n2], yp - ys[n2]])
    r12 = np.array([xs[n2] - xs[n1], ys[n2] - ys[n1]])
    r21 = -1 * r12

    absr1p = np.linalg.norm(r1p)
    absr2p = np.linalg.norm(r2p)
    absr12 = np.linalg.norm(r12)

    costhet1 = np.dot(r1p, r12) / (absr1p * absr12)
    costhet2 = np.dot(r2p, r21) / (absr2p * absr12)

    thet1 = np.arccos(costhet1)
    thet2 = np.arccos(costhet2)

    nback = n2

    if n1 == 0:
        if thet1 >= np.pi / 2:
            nback = n1  # before the path

    if n2 == nmax:
        if thet2 >= np.pi / 2:
            nback = nmax + 1  # behind the path

    # modification by tolerance
    nstart = nback

    for n in range(nstart, nmax + 1):
        xn = xs[n]
        yn = ys[n]
        
        sn = np.sqrt((xp - xn)**2 + (yp - yn)**2)
        if sn <= tol:
            nback = n + 1  # revised expression

    if nback > nmax:
        nback = length

    return nback


def find_current_segment_index_cir(pos, path, n0, tol):
    path_arr = np.array(path)
    xs = path_arr[:, 0]
    ys = path_arr[:, 1]
    npoint = path_arr.shape[0]
    
    xp, yp = pos
    
    # Find nearest point starting from n0
    nmin = n0
    dmin = np.sqrt((xp - xs[n0])**2 + (yp - ys[n0])**2)
    for n in range(n0 + 1, npoint):
        x = xs[n]
        y = ys[n]
        dn = np.sqrt((xp - x)**2 + (yp - y)**2)
        if dn <= dmin:
            dmin = dn
            nmin = n
        else:
            break
    na = nmin
    
    # Determine second nearest point (nb)
    if na == 0:
        nb = na + 1
    elif na == npoint - 1:
        nb = na - 1
    else:
        x1 = xs[na - 1]
        y1 = ys[na - 1]
        d1 = np.sqrt((xp - x1)**2 + (yp - y1)**2)
        x2 = xs[na + 1]
        y2 = ys[na + 1]
        d2 = np.sqrt((xp - x2)**2 + (yp - y2)**2)
        if d2 <= d1:
            nb = na + 1
        else:
            nb = na - 1
    
    n1 = min(na, nb)
    n2 = n1 + 1
    
    # Calculate vectors and angles
    r1p = np.array([xp - xs[n1], yp - ys[n1]])
    r2p = np.array([xp - xs[n2], yp - ys[n2]])
    r12 = np.array([xs[n2] - xs[n1], ys[n2] - ys[n1]])
    r21 = -r12
    
    # Compute cos(theta1) and handle division by zero
    dot_r1p_r12 = np.dot(r1p, r12)
    norm_r1p = np.linalg.norm(r1p)
    norm_r12 = np.linalg.norm(r12)
    if norm_r1p == 0 or norm_r12 == 0:
        costhet1 = 0.0
    else:
        costhet1 = dot_r1p_r12 / (norm_r1p * norm_r12)
    
    # Compute cos(theta2) and handle division by zero
    dot_r2p_r21 = np.dot(r2p, r21)
    norm_r2p = np.linalg.norm(r2p)
    norm_r21 = np.linalg.norm(r21)
    if norm_r2p == 0 or norm_r21 == 0:
        costhet2 = 0.0
    else:
        costhet2 = dot_r2p_r21 / (norm_r2p * norm_r21)
    
    thet1 = np.arccos(np.clip(costhet1, -1.0, 1.0))
    thet2 = np.arccos(np.clip(costhet2, -1.0, 1.0))
    
    # Determine nback based on angles
    nback = n2
    if n1 == 0:
        if thet1 >= np.pi / 2:
            nback = n1
    if n2 == npoint - 1:
        if thet2 >= np.pi / 2:
            nback = npoint  
    
    # Modify nback based on tolerance
    nstart = nback
    if nstart <= npoint - 1:
        for n in range(nstart, npoint):
            xn = xs[n]
            yn = ys[n]
            sn = np.sqrt((xp - xn)**2 + (yp - yn)**2)
            if sn <= tol:
                nback = n + 1
            else:
                break
    
    return nback


def get_next_point(path, pos, R1=10.0, r_final=5.0):
    pathnum = len(path)
    boat_x, boat_y = pos
    
    # 检查是否在接纳圆内
    current_num = 0
    flag = False
    for i in range(pathnum):
        x, y = path[i]
        distance = np.hypot(x - boat_x, y - boat_y)
        if distance < R1:
            if i == pathnum - 1 and distance > r_final:
                continue
            current_num = i
            flag = True
    if flag:
        return current_num
    
    # 处理路径段
    min_distance = None
    selected_num = 0
    X_N_selected = Y_N_selected = 0  # 初始化以避免未定义错误
    
    for i in range(pathnum - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        denominator = A**2 + B**2
        
        if denominator == 0:
            continue  # 避免除以零
        
        # 计算垂足和距离
        d = abs(A * boat_x + B * boat_y + C) / np.sqrt(denominator)
        X_N = (B**2 * boat_x - A * B * boat_y - A * C) / denominator
        Y_N = (-A * B * boat_x + A**2 * boat_y - B * C) / denominator
        
        # 判断垂足是否在线段内
        dx1 = x1 - X_N
        dx2 = x2 - X_N
        dy1 = y1 - Y_N
        dy2 = y2 - Y_N
        dot_product = dx1 * dx2 + dy1 * dy2
        
        if i ==0:
            min_distance = np.hypot(x1 - boat_x, y1 - boat_y)
            selected_num = -1
        if dot_product > 0:
            segment_distance = np.hypot(x1 - boat_x, y1 - boat_y)
        else:
            segment_distance = d
        
        # 更新最小距离和对应路径段
        if min_distance is None or segment_distance < min_distance:
            min_distance = segment_distance
            selected_num = i
            X_N_selected, Y_N_selected = X_N, Y_N
    
    # 检查是否到达终点区域
    if selected_num == pathnum - 2:
        x_final, y_final = path[selected_num + 1]
        final_distance = np.hypot(x_final - boat_x, y_final - boat_y)
        
        # 重新计算垂足点是否在路径段外
        x1, y1 = path[selected_num]
        x2, y2 = path[selected_num + 1]
        dx1 = x1 - X_N_selected
        dx2 = x2 - X_N_selected
        dy1 = y1 - Y_N_selected
        dy2 = y2 - Y_N_selected
        dot_product = dx1 * dx2 + dy1 * dy2
        
        if dot_product > 0 and final_distance < min_distance:
            selected_num += 1
    
    return selected_num


def get_intersec_point(p1, p2, p3, p4):
    """
    Calculate the intersection point of two lines.
    """
    xdiff = (p1[0] - p2[0], p3[0] - p4[0])
    ydiff = (p1[1] - p2[1], p3[1] - p4[1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d1 = (p1[0], p1[1])
    d2 = (p2[0], p2[1])
    d3 = (p3[0], p3[1])
    d4 = (p4[0], p4[1])
    
    d = (det(d1, d2), det(d3, d4))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def line_intersection(p1, p2, p3, p4):
    """
    Find the intersection point of two lines defined by two pairs of points.
    Args:
    p1, p2 : tuples of the form (x, y) representing two points on the first line.
    p3, p4 : tuples of the form (x, y) representing two points on the second line.
    
    Returns:
    A tuple (x, y) representing the intersection point if it exists, else None if the lines are parallel.
    """
    
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    # Compute the determinant of the system
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    
    if denominator == 0:
        # Lines are parallel or coincident
        return None

    # Calculate the x and y coordinates of the intersection point
    intersect_x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denominator
    intersect_y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denominator

    return (intersect_x, intersect_y)     
    
   
def closest_point_on_line(point, line_start, line_end):
    # Convert points to numpy arrays
    point = np.array(point)
    line_start = np.array(line_start)
    line_end = np.array(line_end)
    
    # Calculate the direction vector of the line
    line_vec = line_end - line_start
    
    # Calculate the vector from line_start to the point
    point_vec = point - line_start
    
    # Project point_vec onto line_vec
    line_len = np.dot(line_vec, line_vec)
    if line_len == 0:
        raise ValueError("The start and end points of the line cannot be the same")
    
    projection = np.dot(point_vec, line_vec) / line_len
    
    # Calculate the closest point on the line
    closest_point = line_start + projection * line_vec
    
    return closest_point
    
    
def check_point_on_seg(point, line_start, line_end):
    x, y = point
    x0, y0 = line_start
    x1, y1 = line_end
    # Check if the line segment is a single point
    if x0 == x1 and y0 == y1:
        return (x == x0) and (y == y0)
    
    dx = x1 - x0
    dy = y1 - y0
    
    # Vector from (x0, y0) to (x, y)
    vx = x - x0
    vy = y - y0
    
    # Compute the dot product
    dot_product = vx * dx + vy * dy
    
    # Compute the squared length of the line segment
    len_sq = dx * dx + dy * dy
    
    # Compute the parameter t
    t = dot_product / len_sq
    
    # Check if t is within [0, 1]
    # return 0.0 <= t <= 1.0
    return t <= 1.0


def parse_polygon_str(poly_item):
    """Accept polygon as CSV string "lon,lat,..." or flat list/tuple [lon1,lat1,lon2,lat2,...]."""
    try:
        coords = []
        if isinstance(poly_item, str):
            parts = [p.strip() for p in poly_item.split(',') if p.strip() != '']
            if len(parts) < 6 or (len(parts) % 2) != 0:
                return None
            it = iter(parts)
            for lon_s, lat_s in zip(it, it):
                lon = float(lon_s)
                lat = float(lat_s)
                coords.append((lon, lat))
        elif isinstance(poly_item, (list, tuple)):
            flat = list(poly_item)
            if len(flat) < 6 or (len(flat) % 2) != 0:
                return None
            for i in range(0, len(flat), 2):
                lon = float(flat[i])
                lat = float(flat[i + 1])
                coords.append((lon, lat))
        else:
            return None
        if len(coords) < 3:
            return None
        minx = min(p[0] for p in coords)
        maxx = max(p[0] for p in coords)
        miny = min(p[1] for p in coords)
        maxy = max(p[1] for p in coords)
        return {'bbox': (minx, miny, maxx, maxy), 'coords': coords}
    except Exception:
        return None


def _point_on_segment(px, py, x1, y1, x2, y2, eps=1e-12):
    """Return True if point P lies on segment (x1,y1)-(x2,y2)."""
    # Bounding box check
    if (min(x1, x2) - eps) <= px <= (max(x1, x2) + eps) and (min(y1, y2) - eps) <= py <= (max(y1, y2) + eps):
        # Cross product close to 0 -> collinear
        dx1, dy1 = x2 - x1, y2 - y1
        dxp, dyp = px - x1, py - y1
        cross = dx1 * dyp - dy1 * dxp
        if abs(cross) <= eps:
            return True
    return False



def _point_in_polygon(lon, lat, polygon):
    """Ray casting algorithm for point-in-polygon.
    - polygon: list of (lon, lat)
    - returns True if inside or on edge
    """
    if not polygon or len(polygon) < 3:
        return True
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        # On-edge check
        if _point_on_segment(lon, lat, x1, y1, x2, y2):
            return True
        # Ray casting: check if edge crosses the horizontal ray to the right of point
        if ((y1 > lat) != (y2 > lat)):
            xinters = (x2 - x1) * (lat - y1) / (y2 - y1 + 0.0) + x1
            if lon < xinters:
                inside = not inside
    return inside


def is_point_on_land(lon, lat, fence_polygons):
    # Fast reject if no polygons
    polygons_snapshot = fence_polygons[:] if fence_polygons else []
    if not polygons_snapshot:
        return False
    for poly in polygons_snapshot:
        minx, miny, maxx, maxy = poly['bbox']
        if lon < minx or lon > maxx or lat < miny or lat > maxy:
            continue
        if _point_in_polygon(lon, lat, poly['coords']):
            return True
    return False
