def is_intersecting_line_segment(x1, y1, x2, y2, x3, y3, x4, y4):
    # 计算叉积
    def cross_product(xa, ya, xb, yb):
        return xa * yb - ya * xb

    # 向量 AB 和 AC 的叉积
    def vector_cross(xa, ya, xb, yb, xc, yc):
        return cross_product(xb - xa, yb - ya, xc - xa, yc - ya)

    # 判断交点是否在线段 AB 上
    def is_on_segment(px, py, ax, ay, bx, by):
        return min(ax, bx) <= px <= max(ax, bx) and min(ay, by) <= py <= max(ay, by)
    # 特殊逻辑：如果 B 点在 CD 上，直接掉头
    # if is_on_segment(x2, y2, x3, y3, x4, y4):
    #     print("B 在 CD 上，强制判定为掉头模式")
        return False
    # 判断 AB 和 CD 是否相交
    def check_intersection():
        # 计算 C, D 与线段 AB 的叉积
        cross1 = vector_cross(x1, y1, x2, y2, x3, y3)
        cross2 = vector_cross(x1, y1, x2, y2, x4, y4)

        # 如果叉积异号，说明 C 和 D 在 AB 的两侧
        if cross1 * cross2 <= 0:
            # 计算直线 CD 的方向向量
            dx, dy = x4 - x3, y4 - y3
            # 计算 AB 与 CD 的交点
            denominator = cross_product(dx, dy, x1 - x2, y1 - y2)
            if denominator == 0:
                print("本船与跟随位置在目标同一侧，进入掉头模式1")  # 平行或共线
                return False

            # 计算交点参数
            t = cross_product(x3 - x1, y3 - y1, dx, dy) / denominator
            # 计算交点坐标
            inter_x = x1 + t * (x2 - x1)
            inter_y = y1 + t * (y2 - y1)

            # 检查交点是否在线段 AB 上
            if is_on_segment(inter_x, inter_y, x1, y1, x2, y2):
                 if abs(inter_x - x2) < 1e-6 and abs(inter_y - y2) < 1e-6:
                    print("交点即为 B 点，进入掉头模式（新加逻辑）")
                    return False
                 else:
                    print("本船与跟随位置在目标两侧，进入绕尾模式2")
                    return True
            else:
                print("本船与跟随位置在目标同一侧，进入掉头模式3")
                return False

        print("本船与跟随位置在目标同一侧，进入掉头模式4")
        return False

    return check_intersection()

# 线段AB
x1, y1 = 1, 10  # 拖轮位置
x2, y2 = 1, 3  # 伴航点位置
# 直线CD
x3, y3 = 0, 5  # 目标船位置
x4, y4 = 5, 0  # 目标船顶点(航向上的顶点)

result = is_intersecting_line_segment(0, 0, 2012.1321380070947, 120.21719196633215 ,1977.9343272158935, 214.1879839023864, 1883.9635352798393, 179.99017311118516)
# print("线段 AB 和直线 CD 是否相交:", result)
