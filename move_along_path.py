import A_star_v2
import uwb_kalman
import numpy as np


# 串口通信，告知stm32小车左轮和右轮的速度
def set_vel(vl, vr):
    return




def turn(angle):
    # angle_hex = angle.to_bytes(1, byteorder='big')
    # if direction == "left":
    #     ser.write(b'\x10' + angle_hex)
    # elif direction == "right":
    #     ser.write(b'\x20' + angle_hex)
    return


# 计算向量夹角，单位为度
# 算出正角度逆时针，负角度顺时针
def angle_between_vectors(vector_a, vector_b):
    # 计算点积
    dot_product = np.dot(vector_a, vector_b)

    # 计算向量的模
    norm_a = np.linalg.norm(vector_a)
    norm_b = np.linalg.norm(vector_b)

    # 计算余弦值
    cos_theta = dot_product / (norm_a * norm_b)

    # 反余弦得到夹角（弧度）
    angle_radians = np.arccos(cos_theta)

    # 将弧度转换为角度
    angle_degrees = np.degrees(angle_radians)

    # 计算二维向量的叉积（结果是标量）
    cross_product = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]

    # 确定角度的正负
    if cross_product < 0:  # 如果叉积为负，则角度为负
        angle_degrees = -angle_degrees

    return angle_degrees


# ros Subscriber：使用uwb获得当前坐标，频率为50hz，使用kalman滤波以获得更精确的位置
def get_current_pos():
    return (0, 0)


# 从一个节点走到另一个节点的运动控制，带有偏移矫正
# 通过分别下达左右轮的速度指令来实现
def move_to(p1, p2):

    return


if __name__ == '__main__':

    # ser = serial.Serial('/dev/ttyAMA2', 115200)  # 打开串口设备
    # if not ser.isOpen:
    #     ser.open()
    # print('serial opened')

    # 建立地图，找到path
    points = [(170, 80), (340, 80), (30, 90),
              (30, 230), (170, 225), (200, 235), (340, 230),
              (30, 375), (200, 375), (340, 375), (420, 375)]
    obstacles = [((180, 50), (310, 200)),
                 ((60, 110), (150, 200)),
                 ((370, 50), (450, 200)),
                 ((60, 260), (190, 350)),
                 ((230, 260), (310, 350)),
                 ((370, 260), (450, 350))]
    start = (135, 25)
    goal = (100, 352)
    points.append(start)
    points.append(goal)
    graph = A_star_v2.create_graph(points, obstacles)
    A_star_v2.visualize_graph(graph, obstacles)
    path = A_star_v2.a_star(graph, start, goal)
    print("Path:", path)

    # 从path中的起点开始，走过每一个坐标点，最终到达终点
    # 假设一开始小车朝向y轴放置

    for i in range(0, len(path) - 1):
        if i == 0:
            v0 = (0, 1)
        else:
            v0 = np.subtract(path[i], path[i - 1])
        v1 = np.subtract(path[i+1], path[i])
        angle = angle_between_vectors(v0, v1)

        # 原地转向
        turn(angle)

        # 走到下一个节点
        move_to(path[i], path[i+1])


    print('process finished')

    # ser.close()
