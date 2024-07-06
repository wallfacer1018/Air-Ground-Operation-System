import sys
import serial
import time
import math


# 串口通信
def forward(length):
    bytes_value = length.to_bytes(2, byteorder='big')
    ser.write(bytes_value)


def turn(direction, angle):
    angle_hex = angle.to_bytes(1, byteorder='big')
    if direction == "left":
        ser.write(b'\x10' + angle_hex)
    elif direction == "right":
        ser.write(b'\x20' + angle_hex)


# 定义Node类
class Node(object):
    def __init__(self, point, parent, d, f, direction=None, action=None):
        self.point = point
        self.parent = parent
        self.d = d
        self.f = f
        self.direction = direction
        self.action = action


# 曼哈顿距离
def Manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


# 初始化障碍物
def init_obstacles():
    global rectObs
    rectObs = []
    rectObs.append(((60, 110), (140, 200)))
    rectObs.append(((200, 110), (310, 200)))
    rectObs.append(((370, 50), (450, 200)))
    rectObs.append(((60, 260), (170, 350)))
    rectObs.append(((230, 260), (310, 350)))
    rectObs.append(((370, 260), (450, 350)))


# 节点与障碍物碰撞检测
def collides(p):
    buffer = 10
    for rect in rectObs:
        if (rect[0][0] - buffer <= p[0] <= rect[1][0] + buffer and
                rect[0][1] - buffer <= p[1] <= rect[1][1] + buffer):
            return True
    return False


# A*估算函数
def evaluation_Astar(ynode, goalPoint, openlist):
    if collides(ynode.point):
        return
    if ynode.point[0] < 0 or ynode.point[1] < 0:
        return
    ynode.f = ynode.d + 0.1 * Manhattan(ynode.point, goalPoint.point)
    for p in openlist:
        if p.point == ynode.point:
            if p.d > ynode.d:
                openlist.remove(p)
                openlist.append(ynode)
            return
    openlist.append(ynode)


# 实时路径规划和执行
def run_game(start, end):
    init_obstacles()
    goalPoint = Node(end, None, 0, 0)
    if collides(goalPoint.point):
        nearest_point = find_nearest_accessible_point(goalPoint.point)
        goalPoint = Node(nearest_point, None, 0, 0)

    while True:
        current_position = get_current_position()
        if Manhattan(current_position, goalPoint.point) < 10:
            print("Reached the goal!")
            break

        path, actions = a_star_path_planning(current_position, goalPoint.point)

        if not path:
            print("Failed to find the path")
            break

        for action in actions:
            if action.startswith("move"):
                forward(int(action.split()[2]))
            if action.startswith("turn"):
                direction = action.split()[1]
                angle = int(action.split()[2])
                turn(direction, angle)
            time.sleep(0.5)


def a_star_path_planning(start, end):
    openlist = []
    closelist = []

    initialPoint = Node(start, None, 0, 0)
    goalPoint = Node(end, None, 0, 0)
    initialPoint.f = initialPoint.d + 0.1 * Manhattan(goalPoint.point, initialPoint.point)
    openlist.append(initialPoint)

    while openlist:
        xnode = openlist.pop(0)
        if xnode.point == goalPoint.point:
            goalNode = xnode
            break
        closelist.append(xnode)
        for i in range(4):
            if i == 0:
                direction = "up"
                ynode = Node((xnode.point[0], xnode.point[1] + 10), xnode, xnode.d + 1, 0, direction,
                             "move up 10 units")
            elif i == 1:
                direction = "down"
                ynode = Node((xnode.point[0], xnode.point[1] - 10), xnode, xnode.d + 1, 0, direction,
                             "move down 10 units")
            elif i == 2:
                direction = "left"
                ynode = Node((xnode.point[0] - 10, xnode.point[1]), xnode, xnode.d + 1, 0, direction,
                             "move left 10 units")
            elif i == 3:
                direction = "right"
                ynode = Node((xnode.point[0] + 10, xnode.point[1]), xnode, xnode.d + 1, 0, direction,
                             "move right 10 units")

            if xnode.direction is not None and xnode.direction != direction:
                dir = ""
                if (xnode.direction, direction) in [("up", "right"), ("right", "down"), ("down", "left"),
                                                    ("left", "up")]:
                    dir = "turn right 90"
                elif (xnode.direction, direction) in [("right", "up"), ("down", "right"), ("left", "down"),
                                                      ("up", "left")]:
                    dir = "turn left 90"
                znode = Node((xnode.point[0], xnode.point[1]), xnode, xnode.d + 1, 0, direction, dir)
                ynode = Node((ynode.point[0], ynode.point[1]), znode, znode.d + 1, 0, direction, ynode.action)

            evaluation_Astar(ynode, goalPoint, openlist)

        openlist.sort(key=lambda node: node.f)

    if xnode.point == goalPoint.point:
        currNode = goalNode
        path = []
        actions = []
        while currNode.parent is not None:
            path.append(currNode.point)
            actions.append(currNode.action)
            currNode = currNode.parent
        path.reverse()
        actions.reverse()

        # 合并连续的直行动作
        merged_actions = []
        current_action = None
        current_distance = 0
        for action in actions:
            if action.startswith("move"):
                direction, distance = action.split()[1], int(action.split()[2])
                if current_action and current_action.startswith("move") and current_action.split()[1] == direction:
                    current_distance += distance
                else:
                    if current_action:
                        merged_actions.append(f"move {current_action.split()[1]} {current_distance} units")
                    current_action = action
                    current_distance = distance
            else:
                if current_action:
                    merged_actions.append(f"move {current_action.split()[1]} {current_distance} units")
                merged_actions.append(action)
                current_action = None
                current_distance = 0

        if current_action:
            merged_actions.append(f"move {current_action.split()[1]} {current_distance} units")

        return path, merged_actions
    else:
        return None, None


def find_nearest_accessible_point(point):
    min_dist = float('inf')
    nearest_point = None

    for rect in rectObs:
        for px in range(rect[0][0], rect[1][0] + 1, 10):
            for py in [rect[0][1] - 20, rect[1][1] + 20]:
                if not collides((px, py)):
                    dist = Manhattan(point, (px, py))
                    if dist < min_dist:
                        min_dist = dist
                        nearest_point = (px, py)
        for px in [rect[0][0] - 20, rect[1][0] + 20]:
            for py in range(rect[0][1], rect[1][1] + 1, 10):
                if not collides((px, py)):
                    dist = Manhattan(point, (px, py))
                    if dist < min_dist:
                        min_dist = dist
                        nearest_point = (px, py)
    return nearest_point


def get_current_position():
    # 获取当前坐标信息，这里假设通过串口通信获取
    # 需要根据实际情况实现
    ser.write(b'\x01')  # 发送请求当前位置的命令
    response = ser.read(4)  # 假设返回4字节数据，两个字节X坐标，两个字节Y坐标
    x = int.from_bytes(response[0:2], byteorder='big')
    y = int.from_bytes(response[2:4], byteorder='big')
    return (x, y)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python script.py startx,starty,endx,endy")
        sys.exit(1)

    coords = sys.argv[1].split(',')
    if len(coords) != 4:
        print("Invalid input format. Please use startx,starty,endx,endy")
        sys.exit(1)

    ser = serial.Serial('/dev/ttyAMA2', 115200)  # 打开串口设备
    if not ser.isOpen:
        ser.open()
    print('serial opened')

    ser.write(b'\xaa')
    ser.write(b'\xaa')
    startx, starty, endx, endy = map(int, coords)
    run_game((startx, starty), (endx, endy))
    ser.write(b'\xff')
    ser.write(b'\xff')

    print('process finished')

    ser.close()
