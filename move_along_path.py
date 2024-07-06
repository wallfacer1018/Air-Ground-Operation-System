import A_star_v2
import uwb_kalman
import numpy as np
import time


# 串口通信，告知stm32小车左轮和右轮的速度
# 每个命令为两个字节，16进制
# 1xyz，左转，速度模式为x，角度为yz，十进制最大值为255
# 2xyz，右转，速度模式为x，角度为yz，十进制最大值为255
# 3xyz，设置左轮速度，为xyz，十进制最大值为4095
# 4xyz，设置右轮速度，为xyz，十进制最大值为4095
# 5000，按照最新设置的左右轮速度前行
def set_vel(vl, vr):
    vl_hex = f'3{vl:03X}'.encode('utf-8')  # 将速度设置为16进制编码
    vr_hex = f'4{vr:03X}'.encode('utf-8')  # 将速度设置为16进制编码
    ser.write(b'aaaa')  # 起始校验位
    ser.write(vl_hex)
    ser.write(vr_hex)
    ser.write(b'5000')  # 命令小车前行
    ser.write(b'ffff')  # 结束校验位
    print(f"Setting velocities: Left = {vl}, Right = {vr}")
    return

def turn(angle):
    angle_abs = int(abs(angle))
    speed_mode = 56
    # 假设速度模式为1

    if angle > 0:  # 左转
        command = f'1{speed_mode}{angle_abs:02X}'.encode('utf-8')
    elif angle < 0:  # 右转
        command = f'2{speed_mode}{angle_abs:02X}'.encode('utf-8')
    ser.write(b'aaaa')  # 起始校验位
    ser.write(command)
    ser.write(b'ffff') # 结束校验位
    print(f"Turning by {angle} degrees")
    return


# 计算向量夹角，单位为度
# 算出正角度逆时针，负角度顺时针
def angle_between_vectors(vector_a, vector_b):
    dot_product = np.dot(vector_a, vector_b)
    norm_a = np.linalg.norm(vector_a)
    norm_b = np.linalg.norm(vector_b)
    cos_theta = dot_product / (norm_a * norm_b)
    angle_radians = np.arccos(cos_theta)
    angle_degrees = np.degrees(angle_radians)
    cross_product = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]
    if cross_product < 0:
        angle_degrees = -angle_degrees
    return angle_degrees


# ros Subscriber：使用uwb获得当前坐标，频率为50hz，使用kalman滤波以获得更精确的位置
def get_current_pos():
    # Placeholder for obtaining the current position using UWB and Kalman filtering
    return (0, 0)


# 从一个节点走到另一个节点的运动控制，带有偏移矫正
# 通过分别下达左右轮的速度指令来实现
def move_to(p1, p2):
    target_vector = np.subtract(p2, p1)
    while True:
        current_pos = get_current_pos()
        current_vector = np.subtract(p2, current_pos)
        distance_to_target = np.linalg.norm(current_vector)
        if distance_to_target < 5:  # Adjust the threshold as needed
            break
        angle = angle_between_vectors(target_vector, current_vector)
        if abs(angle) > 5:  # Adjust the threshold as needed
            if angle > 0:
                set_vel(50, 60)  # Turn left
            else:
                set_vel(60, 50)  # Turn right
        else:
            set_vel(50, 50)  # Move forward
        time.sleep(0.1)  # Adjust the delay as needed
    set_vel(0, 0)  # Stop the vehicle


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
