import pyproj
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


# def cal_DCPA_TCPA(obj_speed, self_speed, obj_pos, self_pos):
#     rel_speed = (obj_speed[0] - self_speed[0], obj_speed[1] - self_speed[1])
#     rel_pos = (obj_pos[0] - self_pos[0], obj_pos[1] - self_pos[1])
    
#     rel_speed_mag = np.sqrt(rel_speed[0]**2 + rel_speed[1]**2)
#     rel_pos_mag = np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)  # Current distance
    
#     # Handle zero relative speed case
#     if rel_speed_mag < 1e-10:
#         dcpa = rel_pos_mag
#         tcpa = float('inf')  # Undefined, use infinity or another indicator
#     else:
#         cross = rel_pos[0] * rel_speed[1] - rel_pos[1] * rel_speed[0]
#         dcpa = np.abs(cross) / rel_speed_mag
#         dot_product = rel_speed[0] * rel_pos[0] + rel_speed[1] * rel_pos[1]
#         tcpa = -dot_product / (rel_speed_mag**2) / 60  # Adjusted unit handling
    
#     return dcpa, tcpa


def cal_DCPA_TCPA(obj_speed, self_speed, obj_pos, self_pos):
    x0, y0 = self_pos
    u0, v0 = self_speed
    x1, y1 = obj_pos
    u1, v1 = obj_speed
    if u0==u1 and v0==v1:
        return np.sqrt((x0-x1)**2+(y0-y1)**2), 0
    p = x0 - x1
    q = y0 - y1
    du = u0 - u1
    dv = v0 - v1
    tcpa = -((p*du+q*dv)/(du**2+dv**2))
    dcpa = np.sqrt((p+du*tcpa)**2+(q+dv*tcpa)**2)
    tcpa = tcpa / 60
    
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
    num = 0
    flag = False
    for i in range(num, pathnum):
        x, y = path[i]
        distance = np.hypot(x - boat_x, y - boat_y)
        if distance < R1:
            if i == pathnum - 1 and distance > r_final:
                continue
            num = i
            flag = True
    if flag:
        return num
    
    # 处理路径段
    min_distance = None
    X_N_selected = Y_N_selected = 0  # 初始化以避免未定义错误
    
    for i in range(num, pathnum - 1):
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
        
        if dot_product > 0:
            segment_distance = np.hypot(x1 - boat_x, y1 - boat_y)
        else:
            segment_distance = d
        if i ==0:
            min_distance = np.hypot(x1 - boat_x, y1 - boat_y)
            num = -1
        elif i == num:
            min_distance = segment_distance

        
        # 更新最小距离和对应路径段
        if min_distance is None or segment_distance < min_distance:
            min_distance = segment_distance
            num = i
            X_N_selected, Y_N_selected = X_N, Y_N
    
    # 检查是否到达终点区域
    if num == pathnum - 2:
        x_final, y_final = path[num + 1]
        final_distance = np.hypot(x_final - boat_x, y_final - boat_y)
        
        # 重新计算垂足点是否在路径段外
        x1, y1 = path[num]
        x2, y2 = path[num + 1]
        dx1 = x1 - X_N_selected
        dx2 = x2 - X_N_selected
        dy1 = y1 - Y_N_selected
        dy2 = y2 - Y_N_selected
        dot_product = dx1 * dx2 + dy1 * dy2
        
        if dot_product > 0 and final_distance < min_distance:
            num += 1
    
    return num


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

#neu_apf
def stop_time_estimation(ownship, target_pos, target_speed, safe_distance):
    u1, v1 = target_speed
    x1, y1 = target_pos
    x0, y0 = ownship.own_rel_pos_xy
    u0, v0 = ownship.own_speed_uv
    if np.abs((v0-v1)*(x1-x0) - (u0-u1)*(y1-y0)) /(np.sqrt(((u0-u1)**2 + (v0-v1)**2))) >= safe_distance*1.2:
        return None, None
    t_stop = ((y1-y0)*u1 - (x1-x0)*v1) / (v0*u1 - u0*v1 + 1e-10) - safe_distance*1.2 / (ownship.abs_speed + 1e-10) # norm((u0, v0))
    t_restart = ((y1-y0)*u0 - (x1-x0)*v0) / (v0*u1 - u0*v1 + 1e-10)
    if t_stop < 0 or t_restart < 0:
        return -1, -1
    return t_stop, t_restart

        
def revert_speed_change(ownship, target_df, prioritized_t_idx):
    if prioritized_t_idx not in target_df['t_idx'].values:
        return True
    target_row = target_df[target_df['t_idx'] == prioritized_t_idx].iloc[0]
    x0, y0 = ownship.own_rel_pos_xy
    u0, v0 = ownship.own_speed_uv
    x1, y1 = target_row['x'], target_row['y']
    u1, v1 = target_row['u'], target_row['v']
    return pass_course((x0, y0), (x1, y1), (u0, v0), (u1, v1)) or pass_course((x1, y1), (x0, y0), (u1, v1), (u0, v0))

def pass_course(pos1, pos2, vec1, vec2):
    x0, y0 = pos1
    x1, y1 = pos2
    u0, v0 = vec1
    u1, v1 = vec2
    cross_pos = (x1 - x0) * v0 - (y1 - y0) * u0
    cross_target = u1 * v0 - v1 * u0
    if cross_target == 0:
        return False
    return cross_pos * cross_target > 0



# def is_in_circle(num, line, linenum, boat_x, boat_y, R1, r_final):
#     flag = 0
#     if(num == -1):
#         num1 = 0
#     else:
#         num1 = num
#     for i in range(num1, linenum):
#         dis_point = np.sqrt((line[i][0] - boat_x)**2 + (line[i][1] - boat_y)**2)
#         if dis_point < R1:
#             if i == linenum - 1:
#                 if dis_point > r_final:
#                     break
#             num = i
#             flag = 1
#     return flag, num

# def path_segment(num, line, linenum, boat_x, boat_y, r_final):
#     if linenum < 2:
#         return num

#     start_dis = 0.0
#     new_dis = 0.0
#     X_N, Y_N = 0.0, 0.0

#     if(num == -1):
#         num1 = 0
#     else:
#         num1 = num

#     for i in range(num1, linenum - 1):
#         x_i, y_i = line[i][0], line[i][1]
#         x_i1, y_i1 = line[i+1][0], line[i+1][1]

#         A = y_i1 - y_i
#         B = x_i - x_i1
#         C = x_i1 * y_i - x_i * y_i1
#         denominator = A**2 + B**2

#         if denominator == 0:
#             continue

#         d = np.abs(A * boat_x + B * boat_y + C) / np.sqrt(denominator)
#         X_N = (-A * C - A * B * boat_y + B**2 * boat_x) / denominator
#         Y_N = (-B * C + A**2 * boat_y - A * B * boat_x) / denominator

#         dot_product = (x_i - X_N) * (x_i1 - X_N) + (y_i - Y_N) * (y_i1 - Y_N)
#         if dot_product > 0:
#             new_dis = np.sqrt((x_i - boat_x)**2 + (y_i - boat_y)**2)
#         else:
#             new_dis = d

#         if i == 0:
#             start_dis = np.sqrt((x_i - boat_x)**2 + (y_i - boat_y)**2)
#             num = -1
#         elif i == num:
#             start_dis = new_dis

#         if start_dis > new_dis:
#             start_dis = new_dis
#             num = i

#     if num == linenum - 2:
#         x_end, y_end = line[-1][0], line[-1][1]
#         dis_final = np.sqrt((x_end - boat_x)**2 + (y_end - boat_y)**2)
#         dot_product = (x_i - X_N) * (x_i1 - X_N) + (y_i - Y_N) * (y_i1 - Y_N)
#         if dot_product > 0 and dis_final < new_dis:
#             num += 1

#     if num == -1:
#         x_end, y_end = line[-1][0], line[-1][1]
#         dis_final = np.sqrt((x_end - boat_x)**2 + (y_end - boat_y)**2)
#         dot_product = (x_i - X_N) * (x_i1 - X_N) + (y_i - Y_N) * (y_i1 - Y_N)
#         if dot_product > 0 and dis_final < new_dis:
#             num = linenum-1
#     return num

# def get_next_path_point(num, line, pos, R1=10.0, r_final=5.0):
#     linenum = len(line)
#     boat_x, boat_y = pos
#     flag, new_num = is_in_circle(num, line, linenum, boat_x, boat_y, R1, r_final)
#     if not flag:
#         new_num = path_segment(new_num, line, linenum, boat_x, boat_y, r_final)

#     if linenum == 1:
#         return -1
#     if(np.sqrt(pow(line[linenum-1][0]-line[0][0],2)+pow(line[linenum-1][1]-line[0][1],2))<200):
#         new_num%=linenum-1

#     return new_num

def is_in_circle(num, line, linenum, boat_x, boat_y, R1, r_final, k=1):
    flag = 0

    if(num == -1):
        num1 = 0
    else:
        num1 = num
    j = 0
    
    for i in range(num1, linenum):
        if j > k:
            break
        j += 1
        dis_point = np.sqrt((line[i][0] - boat_x)**2 + (line[i][1] - boat_y)**2)
        if dis_point < R1:
            if i == linenum - 1:
                if dis_point > r_final:
                    break
            num = i
            flag = 1

    return flag, num

def path_segment(num, line, linenum, boat_x, boat_y, r_final, k=1):
    if linenum < 2:
        return num

    start_dis = 0.0
    new_dis = 0.0
    X_N, Y_N = 0.0, 0.0

    if(num == -1):
        num1 = 0
    else:
        num1 = num

    j = 0
    for i in range(num1, linenum - 1):
        if j > k:
            break
        j += 1
        x_i, y_i = line[i][0], line[i][1]
        x_i1, y_i1 = line[i+1][0], line[i+1][1]

        A = y_i1 - y_i
        B = x_i - x_i1
        C = x_i1 * y_i - x_i * y_i1
        denominator = A**2 + B**2

        if denominator == 0:
            continue

        d = np.abs(A * boat_x + B * boat_y + C) / np.sqrt(denominator)
        X_N = (-A * C - A * B * boat_y + B**2 * boat_x) / denominator
        Y_N = (-B * C + A**2 * boat_y - A * B * boat_x) / denominator

        dot_product = (x_i - X_N) * (x_i1 - X_N) + (y_i - Y_N) * (y_i1 - Y_N)
        if dot_product > 0:
            new_dis = np.sqrt((x_i - boat_x)**2 + (y_i - boat_y)**2)
        else:
            new_dis = d

        if i == 0:
            start_dis = np.sqrt((x_i - boat_x)**2 + (y_i - boat_y)**2)
            num = -1
        elif i == num:
            start_dis = new_dis

        if start_dis > new_dis:
            start_dis = new_dis
            num = i
        

    if num == linenum - 2:
        x_end, y_end = line[-1][0], line[-1][1]
        dis_final = np.sqrt((x_end - boat_x)**2 + (y_end - boat_y)**2)
        dot_product = (x_i - X_N) * (x_i1 - X_N) + (y_i - Y_N) * (y_i1 - Y_N)
        if dot_product > 0 and dis_final < new_dis:
            num += 1

    if num == -1:
        x_end, y_end = line[-1][0], line[-1][1]
        dis_final = np.sqrt((x_end - boat_x)**2 + (y_end - boat_y)**2)
        dot_product = (x_i - X_N) * (x_i1 - X_N) + (y_i - Y_N) * (y_i1 - Y_N)
        if dot_product > 0 and dis_final < new_dis:
            num = linenum-1
    return num


def get_next_path_point(num, line, pos, R1=10.0, r_final=5.0):
    linenum = len(line)
    boat_x, boat_y = pos
    flag, new_num = is_in_circle(num, line, linenum, boat_x, boat_y, R1, r_final)
    if not flag:
        new_num = path_segment(new_num, line, linenum, boat_x, boat_y, r_final)

    if linenum == 1:
        return -1
    if(np.sqrt(pow(line[linenum-1][0]-line[0][0],2)+pow(line[linenum-1][1]-line[0][1],2))<200):
        new_num%=linenum-1

    return new_num


def quadratic_bezier(p0, p1, p2, t):
    return ((1-t)**2 * np.array(p0) +
            2 * (1-t) * t * np.array(p1) +
            t**2 * np.array(p2))

def smooth_last_segment(path, num_interp_points=10):
    """
    Replace the straight line between the second last point and the goal with
    a quadratic Bézier curve and insert intermediate points.
    """
    if len(path) < 2:
        return path

    p0 = path[-2]
    p2 = path[-1]
    
    # Determine control point p1 using previous point for a smooth tangent
    if len(path) >= 3:
        p_prev = path[-3]
        # direction of the previous segment
        tangent = np.array(p0) - np.array(p_prev)
        # control point is p0 shifted a fraction (e.g., 0.5) along the tangent
        p1 = np.array(p0) + 0.5 * tangent  
    else:
        # If no previous point exists, use an offset of the straight line
        p1 = (np.array(p0) + np.array(p2)) / 2 + np.array([0, 0.1])
    
    new_points = []
    # Generate interpolated points along the Bézier curve (excluding endpoints)
    for i in range(1, num_interp_points):
        t = i / num_interp_points
        new_pt = quadratic_bezier(p0, p1, p2, t)
        new_points.append(tuple(new_pt))
    
    # Construct new path: all but the last segment, then the smooth segment
    return path[:-2] + [p0] + new_points + [p2]

def determine_crossing(own_xy, own_uv, target_xy, target_uv):
    x0, y0 = own_xy
    u0, v0 = own_uv 
    x1, y1 = target_xy
    u1, v1 = target_uv
    # 处理本船静止的情况
    if u0 == 0 and v0 == 0:
        dx = x1 - x0
        dy = y1 - y0
        du = u1
        dv = v1
        
        if dx == 0 and dy == 0:
            return "parallel"
        
        if dx * dv - du * dy == 0:
            if du != 0:
                t = -dx / du
            elif dv != 0:
                t = -dy / dv
            else:
                return "parallel"
            
            if t >= 0:
                return "parallel"
        
        return "parallel"
    else:
        dx = x1 - x0
        dy = y1 - y0
        du = u1 - u0
        dv = v1 - v0
        
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
                    return "parallel"
                
                if (t0 < 0 and k > 0) or (t0 > 0 and k < 0):
                    if k > 0:
                        return "front"
                    else:
                        return "back"
                else:
                    return "parallel"
            else:
                return "parallel"
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
                return "parallel"
            
def single_rulebased_planning_simple(own_pos, own_speed, target_pos, target_speed, radius, absphi):
    # The code is used to calculate the path of turning points.
    # Note that absphi is the absolute value of turning angle in the unit of radian.

    # key parameters for ship and target
    absphi = np.radians(absphi)
    x0, y0 = own_pos
    u0, v0 = own_speed
    thet0 = np.arctan2(v0, u0)
    x1, y1 = target_pos
    u1, v1 = target_speed
    thet1 = np.arctan2(v1, u1)
    w0 = np.sqrt(u0**2 + v0**2)
    w1 = np.sqrt(u1**2 + v1**2)

    thet = thet1 - thet0
    if thet > np.pi:
        thet = thet - 2 * np.pi
    if thet <= -np.pi:
        thet = thet + 2 * np.pi

    # collision information
    du = u0 - u1
    dv = v0 - v1
    psi = np.arctan2(dv, du)

    cospsi = np.cos(psi)
    sinpsi = np.sin(psi)

    p = x0 - x1
    q = y0 - y1
    h = -sinpsi * p + cospsi * q
    d = cospsi * p + sinpsi * q

    hbar = h / radius

    # choice of actual turning angle 
    if -np.pi+np.radians(5)< thet < -np.radians(5):
        phi = absphi

    if np.pi-np.radians(5)> thet > np.radians(5):
        phi = -absphi

    if -np.radians(5) <= thet <= np.radians(5):
        if hbar >= 0:
            phi = absphi
        else:
            phi = -absphi

    # if thet == np.pi:
    if np.pi-np.radians(5)<=thet<=np.pi or -np.pi<=thet<=-np.pi+np.radians(5) :
        if hbar <= 0:
            phi = -absphi
        else:
            # phi = absphi
            phi = -absphi


    # # choice of actual turning angle 
    # if thet < 0:
    #     phi = absphi

    # if thet > 0 and thet < np.pi:
    #     phi = -absphi

    # if thet == 0:
    #     if hbar >= 0:
    #         phi = absphi
    #     else:
    #         phi = -absphi

    # if thet == np.pi:
    #     if hbar <= 0:
    #         phi = -absphi
    #     else:
    #         phi = -absphi

    # path design parameters
    costhet = np.cos(thet)
    sinthet = np.sin(thet)

    cosphi = np.cos(phi)
    sinphi = np.sin(phi)

    if phi > 0:
        lamd = 1
        sgm = 1
    else:
        lamd = 1
        sgm = -1

    # characteristic times
    alf = -w0 * np.sin(thet0 + phi) + w1 * np.sin(thet1)
    bta = -w0 * np.cos(thet0 + phi) + w1 * np.cos(thet1)
    gma = -w0 * (sgm * cosphi + lamd * sinphi) + w1 * (sgm * costhet + lamd * sinthet)

    alfprm = w0 * np.sin(thet0) - w1 * np.sin(thet1)
    btaprm = w0 * np.cos(thet0) - w1 * np.cos(thet1)
    gmaprm = sgm * w0 - w1 * (sgm * costhet + lamd * sinthet)

    denom = w0**2 * sinphi - 2 * w0 * w1 * np.sin(phi / 2) * np.cos(thet - phi / 2)

    T = (alf * p - bta * q + gma * radius) / denom
    tao = (alfprm * p - btaprm * q + gmaprm * radius) / denom

    # optimization of tmn
    jugy = abs(w0 * sinphi) - abs(w1 * sinthet)

    if phi > 0:
        if jugy > 0:
            wtr = np.sqrt(w0**2 + w1**2 - 2 * w0 * w1 * np.cos(thet + phi))
            abar = (w0 * sinphi + w1 * sinthet) / (w0 * cosphi - w1 * costhet + wtr)
            tmn = radius * (1 + abar) * (w0 * sinphi + w1 * sinthet) / (w0**2 * sinphi - 2 * w0 * w1 * np.sin(phi / 2) * np.cos(thet + phi / 2))
        else:
            tmn = radius / 5 / w0

    if phi < 0:
        if jugy > 0:
            wtr = np.sqrt(w0**2 + w1**2 - 2 * w0 * w1 * np.cos(thet + phi))
            abar = -(w0 * sinphi + w1 * sinthet) / (w0 * cosphi - w1 * costhet + wtr)
            tmn = radius * (1 + abar) * (w0 * sinphi + w1 * sinthet) / (w0**2 * sinphi - 2 * w0 * w1 * np.sin(phi / 2) * np.cos(thet + phi / 2))
        else:
            tmn = radius / 5 / w0

    tmn = tmn * 1.5  # adjustment for safety

    path = []
    # turning points
    xt = x0 + w0 * np.cos(thet0) * T
    yt = y0 + w0 * np.sin(thet0) * T
    path.append((xt, yt))

    xm = xt + w0 * np.cos(thet0 + phi) * tao
    ym = yt + w0 * np.sin(thet0 + phi) * tao
    path.append((xm, ym))

    xn = xm + w0 * np.cos(thet0) * tmn
    yn = ym + w0 * np.sin(thet0) * tmn
    path.append((xn, yn))

    xb = xn + w0 * np.cos(thet0 - phi) * tao
    yb = yn + w0 * np.sin(thet0 - phi) * tao
    path.append((xb, yb))

    if T < 0 or tao < 0 or tmn < 0:
        return None, None
    return T, path

def calculate_collision_angle(own_xy, own_uv, target_xy, target_uv, R):
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

def turnfun(own_xy, own_uv, target_xy, target_uv, R):
    # The code is used to calculate the coordinates of
    # the turn goal, i.e., Tgoal[0] and Tgoal[1] (using 0-based indexing for Python).
    x0, y0 = own_xy
    u0, v0 = own_uv
    x1, y1 = target_xy
    u1, v1 = target_uv

    # key parameters for ship and disc
    w0 = np.sqrt(u0**2 + v0**2)
    w1 = np.sqrt(u1**2 + v1**2)
 
    thet0 = np.arctan2(v0, u0)
    thet1 = np.arctan2(v1, u1)
    thet = thet1 - thet0
 
    p = x0 - x1
    q = y0 - y1
 
    # classification parameter
    # Normalize thet to (-pi, pi]
    if thet > np.pi:
        thet = thet - 2 * np.pi
    if thet <= -np.pi: # If thet == -pi, it becomes pi.
        thet = thet + 2 * np.pi
 
    if thet >= 0:
        sgma = 1
    else:
        sgma = -1
                           
    # calculation of turn angle and sail time
    a = np.cos(thet0) * p + np.sin(thet0) * q + R
    b = -np.sin(thet0) * p + np.cos(thet0) * q + sgma * R
    c = np.sin(thet1) * p - np.cos(thet1) * q + R * (np.sin(thet) - sgma * np.cos(thet))
 
    phi0 = np.arctan2(b, a)
    
    # Calculate fphi. NumPy's arithmetic with division by zero or invalid operations
    # (like sqrt of negative, though not expected for a^2+b^2)
    # will result in np.inf or np.nan, similar to MATLAB's behavior.
    fphi_numerator = (w1 / w0) * c if w0 != 0 else np.nan if c == 0 else np.inf * np.sign(c) * np.sign(w1) # Handle w0=0 explicitly for clarity
    fphi_denominator = np.sqrt(a**2 + b**2)

    if fphi_denominator == 0:
        if fphi_numerator == 0:
            fphi = np.nan # 0/0
        elif np.isinf(fphi_numerator):
             fphi = fphi_numerator # inf/0 or nan/0
        else: # non-zero / 0
            fphi = np.inf * np.sign(fphi_numerator) if fphi_numerator != 0 else np.nan
    elif w0 == 0 : # if w0 was zero, fphi_numerator is already inf/nan
        fphi = fphi_numerator / fphi_denominator # inf/denom or nan/denom
    else: # Default case, w0 !=0 and fphi_denominator !=0
        fphi = (w1 / w0) * c / fphi_denominator

    if fphi < -1 or fphi > 1 or fphi == np.nan or fphi == np.inf:
        return None
    # np.arcsin will produce nan if fphi is nan or abs(fphi) > 1 (with a RuntimeWarning).
    # This is consistent with MATLAB's asin.
    arcsin_fphi = np.arcsin(fphi)
 
    phi_1 = arcsin_fphi + phi0
    
    # If fphi is NaN, (fphi >= 0) is False.
    # If arcsin_fphi is NaN, phi_1 and phi_2 will become NaN.
    if fphi >= 0: # This handles fphi being NaN correctly (condition becomes False)
        phi_2 = np.pi - arcsin_fphi + phi0
    else:
        phi_2 = -np.pi - arcsin_fphi + phi0
 
    # Normalize phi_1 to [-pi, pi]
    # If phi_1 is NaN, comparisons are False, phi_1 remains NaN.
    if phi_1 > np.pi:
        phi_1 = phi_1 - 2 * np.pi
    if phi_1 < -np.pi: # MATLAB original is '< -pi' for phi values
        phi_1 = phi_1 + 2 * np.pi
 
    # Normalize phi_2 to [-pi, pi]
    # If phi_2 is NaN, comparisons are False, phi_2 remains NaN.
    if phi_2 > np.pi:
        phi_2 = phi_2 - 2 * np.pi
    if phi_2 < -np.pi:
        phi_2 = phi_2 + 2 * np.pi
 
    # In MATLAB, min/max on a vector with NaNs ignore NaNs (unless all are NaN).
    # np.nanmin and np.nanmax replicate this behavior.
    # phis_values = [phi_1, phi_2] # For clarity if needed
    
    if sgma == 1:
        phi = np.nanmin([phi_1, phi_2]) # Returns min of non-NaNs, or NaN if both are NaN / all-NaN slice
        # If phi is NaN, (phi < -np.pi / 2) is False.
        if not np.isnan(phi) and phi < -np.pi / 2:
            phi = np.nanmax([phi_1, phi_2])
    else: # sgma == -1
        phi = np.nanmax([phi_1, phi_2])
        # If phi is NaN, (phi > np.pi / 2) is False.
        if not np.isnan(phi) and phi > np.pi / 2:
            phi = np.nanmin([phi_1, phi_2])
    phi_max = np.arccos(w1*np.cos(thet) / w0) if w0 != 0 else np.nan # Handle division by zero
    if np.isnan(phi) or np.isnan(phi_max) or np.abs(phi) > phi_max:
        return None  
    # Calculate T. Division by zero will result in np.inf or np.nan.
    # If phi or thet is NaN, cos operations result in NaN, T_denominator is NaN, T is NaN.
    T_denominator = (w0 * np.cos(phi) - w1 * np.cos(thet))
    
    if T_denominator == 0:
        T = np.nan if a == 0 else np.inf * np.sign(-a)
    else:
        T = -a / T_denominator
    
    # The coordinates of the turn goal
    # If T, phi, or thet0 is NaN, results will be NaN.
    Tgoal_x = x0 + w0 * np.cos(thet0 + phi) * T
    Tgoal_y = y0 + w0 * np.sin(thet0 + phi) * T
    if Tgoal_x is np.nan or Tgoal_y is np.nan or T < 0:
        return None
    return (Tgoal_x, Tgoal_y)  # Return the coordinates of the turn goal, the turn angle, and the sail time