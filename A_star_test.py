# import sys
# import serial
# import time
# import math
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
#
#
# # 串口通信
# def forward(length):
#     pass  # 此处可以模拟前进或直接更新小车位置
#
#
# def turn(direction, angle):
#     pass  # 此处可以模拟转弯或直接更新小车方向
#
#
# # 定义Node类
# class Node(object):
#     def __init__(self, point, parent, d, f, direction=None, action=None):
#         self.point = point
#         self.parent = parent
#         self.d = d
#         self.f = f
#         self.direction = direction
#         self.action = action
#
#
# # 曼哈顿距离
# def Manhattan(p1, p2):
#     return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
#
#
# # 初始化障碍物
# def init_obstacles():
#     global rectObs
#     rectObs = []
#     rectObs.append(((60, 110), (140, 200)))
#     rectObs.append(((200, 110), (310, 200)))
#     rectObs.append(((370, 50), (450, 200)))
#     rectObs.append(((60, 260), (170, 350)))
#     rectObs.append(((230, 260), (310, 350)))
#     rectObs.append(((370, 260), (450, 350)))
#
#
# # 节点与障碍物碰撞检测
# def collides(p):
#     buffer = 10
#     for rect in rectObs:
#         if (rect[0][0] - buffer <= p[0] <= rect[1][0] + buffer and
#                 rect[0][1] - buffer <= p[1] <= rect[1][1] + buffer):
#             return True
#     return False
#
#
# # A*估算函数
# def evaluation_Astar(ynode, goalPoint, openlist):
#     if collides(ynode.point):
#         return
#     if ynode.point[0] < 0 or ynode.point[1] < 0:
#         return
#     ynode.f = ynode.d + 0.1 * Manhattan(ynode.point, goalPoint.point)
#     for p in openlist:
#         if p.point == ynode.point:
#             if p.d > ynode.d:
#                 openlist.remove(p)
#                 openlist.append(ynode)
#             return
#     openlist.append(ynode)
#
#
# # 实时路径规划和执行
# def run_game(start, end):
#     init_obstacles()
#     goalPoint = Node(end, None, 0, 0)
#     if collides(goalPoint.point):
#         nearest_point = find_nearest_accessible_point(goalPoint.point)
#         goalPoint = Node(nearest_point, None, 0, 0)
#
#     plt.ion()
#     fig, ax = plt.subplots()
#     draw_environment(ax, start, end)
#
#     while True:
#         current_position = get_current_position()
#         if Manhattan(current_position, goalPoint.point) < 10:
#             print("Reached the goal!")
#             break
#
#         path, actions = a_star_path_planning(current_position, goalPoint.point)
#
#         if not path:
#             print("Failed to find the path")
#             break
#
#         for action in actions:
#             if action.startswith("move"):
#                 forward(int(action.split()[2]))
#             if action.startswith("turn"):
#                 direction = action.split()[1]
#                 angle = int(action.split()[2])
#                 turn(direction, angle)
#             time.sleep(0.5)
#             current_position = get_current_position()
#             draw_environment(ax, current_position, goalPoint.point, path)
#             plt.pause(0.1)
#
#     plt.ioff()
#     plt.show()
#
#
# def a_star_path_planning(start, end):
#     openlist = []
#     closelist = []
#
#     initialPoint = Node(start, None, 0, 0)
#     goalPoint = Node(end, None, 0, 0)
#     initialPoint.f = initialPoint.d + 0.1 * Manhattan(goalPoint.point, initialPoint.point)
#     openlist.append(initialPoint)
#
#     while openlist:
#         xnode = openlist.pop(0)
#         if xnode.point == goalPoint.point:
#             goalNode = xnode
#             break
#         closelist.append(xnode)
#         for i in range(4):
#             if i == 0:
#                 direction = "up"
#                 ynode = Node((xnode.point[0], xnode.point[1] + 10), xnode, xnode.d + 1, 0, direction,
#                              "move up 10 units")
#             elif i == 1:
#                 direction = "down"
#                 ynode = Node((xnode.point[0], xnode.point[1] - 10), xnode, xnode.d + 1, 0, direction,
#                              "move down 10 units")
#             elif i == 2:
#                 direction = "left"
#                 ynode = Node((xnode.point[0] - 10, xnode.point[1]), xnode, xnode.d + 1, 0, direction,
#                              "move left 10 units")
#             elif i == 3:
#                 direction = "right"
#                 ynode = Node((xnode.point[0] + 10, xnode.point[1]), xnode, xnode.d + 1, 0, direction,
#                              "move right 10 units")
#
#             if xnode.direction is not None and xnode.direction != direction:
#                 dir = ""
#                 if (xnode.direction, direction) in [("up", "right"), ("right", "down"), ("down", "left"),
#                                                     ("left", "up")]:
#                     dir = "turn right 90"
#                 elif (xnode.direction, direction) in [("right", "up"), ("down", "right"), ("left", "down"),
#                                                       ("up", "left")]:
#                     dir = "turn left 90"
#                 znode = Node((xnode.point[0], xnode.point[1]), xnode, xnode.d + 1, 0, direction, dir)
#                 ynode = Node((ynode.point[0], ynode.point[1]), znode, znode.d + 1, 0, direction, ynode.action)
#
#             evaluation_Astar(ynode, goalPoint, openlist)
#
#         openlist.sort(key=lambda node: node.f)
#
#     if xnode.point == goalPoint.point:
#         currNode = goalNode
#         path = []
#         actions = []
#         while currNode.parent is not None:
#             path.append(currNode.point)
#             actions.append(currNode.action)
#             currNode = currNode.parent
#         path.reverse()
#         actions.reverse()
#
#         # 合并连续的直行动作
#         merged_actions = []
#         current_action = None
#         current_distance = 0
#         for action in actions:
#             if action.startswith("move"):
#                 direction, distance = action.split()[1], int(action.split()[2])
#                 if current_action and current_action.startswith("move") and current_action.split()[1] == direction:
#                     current_distance += distance
#                 else:
#                     if current_action:
#                         merged_actions.append(f"move {current_action.split()[1]} {current_distance} units")
#                     current_action = action
#                     current_distance = distance
#             else:
#                 if current_action:
#                     merged_actions.append(f"move {current_action.split()[1]} {current_distance} units")
#                 merged_actions.append(action)
#                 current_action = None
#                 current_distance = 0
#
#         if current_action:
#             merged_actions.append(f"move {current_action.split()[1]} {current_distance} units")
#
#         return path, merged_actions
#     else:
#         return None, None
#
#
# def find_nearest_accessible_point(point):
#     min_dist = float('inf')
#     nearest_point = None
#
#     for rect in rectObs:
#         for px in range(rect[0][0], rect[1][0] + 1, 10):
#             for py in [rect[0][1] - 20, rect[1][1] + 20]:
#                 if not collides((px, py)):
#                     dist = Manhattan(point, (px, py))
#                     if dist < min_dist:
#                         min_dist = dist
#                         nearest_point = (px, py)
#         for px in [rect[0][0] - 20, rect[1][0] + 20]:
#             for py in range(rect[0][1], rect[1][1] + 1, 10):
#                 if not collides((px, py)):
#                     dist = Manhattan(point, (px, py))
#                     if dist < min_dist:
#                         min_dist = dist
#                         nearest_point = (px, py)
#     return nearest_point
#
#
# def get_current_position():
#     # 模拟获取当前坐标信息
#     # 实际使用时需要替换为实际的UWB定位系统接口
#     return (140, 20)  # 示例返回值
#
#
# def draw_environment(ax, current_position, goal_position, path=None):
#     ax.clear()
#     for rect in rectObs:
#         ax.add_patch(
#             patches.Rectangle(rect[0], rect[1][0] - rect[0][0], rect[1][1] - rect[0][1], edgecolor='r', facecolor='r'))
#     ax.plot(current_position[0], current_position[1], 'bo')  # 小车当前位置
#     ax.plot(goal_position[0], goal_position[1], 'go')  # 目标位置
#     if path:
#         path_x = [p[0] for p in path]
#         path_y = [p[1] for p in path]
#         ax.plot(path_x, path_y, 'b-')
#     ax.set_xlim(0, 480)
#     ax.set_ylim(0, 400)
#     ax.set_aspect('equal')
#
#
# if __name__ == '__main__':
#     start = (50, 50)
#     end = (280, 300)
#
#     run_game(start, end)

import heapq

import matplotlib.pyplot as plt


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star(graph, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, goal)

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor, cost in graph[current].items():
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None


def line_intersects_rect(p1, p2, rect):
    """判断两点连线是否与矩形相交"""
    (x1, y1), (x2, y2) = p1, p2
    (rx1, ry1), (rx2, ry2) = rect

    # 如果线段在矩形区域外
    if max(x1, x2) < rx1 or min(x1, x2) > rx2 or max(y1, y2) < ry1 or min(y1, y2) > ry2:
        return False

    # 判断线段是否穿过矩形的任何边
    def on_segment(p, q, r):
        if min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
            return True
        return False

    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        return 1 if val > 0 else 2

    def do_intersect(p1, q1, p2, q2):
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and on_segment(p1, p2, q1):
            return True

        if o2 == 0 and on_segment(p1, q2, q1):
            return True

        if o3 == 0 and on_segment(p2, p1, q2):
            return True

        if o4 == 0 and on_segment(p2, q1, q2):
            return True

        return False

    rect_edges = [
        ((rx1, ry1), (rx2, ry1)),
        ((rx2, ry1), (rx2, ry2)),
        ((rx2, ry2), (rx1, ry2)),
        ((rx1, ry2), (rx1, ry1))
    ]

    for edge in rect_edges:
        if do_intersect(p1, p2, edge[0], edge[1]):
            return True

    return False


def create_graph(points, obstacles):
    graph = {point: {} for point in points}

    for i, point1 in enumerate(points):
        for point2 in points[i + 1:]:
            intersects = any(line_intersects_rect(point1, point2, obstacle) for obstacle in obstacles)
            if not intersects:
                dist = ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
                graph[point1][point2] = dist
                graph[point2][point1] = dist

    return graph


def visualize_graph(graph, obstacles):
    fig, ax = plt.subplots()

    # 绘制节点
    for node in graph:
        ax.plot(node[0], node[1], 'bo')  # 蓝色圆点表示节点
        for neighbor, _ in graph[node].items():
            ax.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'y')  # 黄色线段表示连线

    # 绘制障碍物
    for obstacle in obstacles:
        (x1, y1), (x2, y2) = obstacle
        rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, color='r', alpha=0.5)  # 红色矩形表示障碍物
        ax.add_patch(rect)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Graph Visualization with Obstacles')
    ax.set_xlim(0, 480)
    ax.set_ylim(0, 400)
    plt.grid(True)
    plt.show()


# 示例使用
points = [(170, 80), (340, 80), (170, 230), (340, 230), (30, 230), (200, 375), (340, 375), (420, 375), (30, 375), (200, 230), (30,100)]
obstacles = [((190, 110), (310, 200)),
             ((60, 110), (150, 200)),
             ((370, 50), (450, 200)),
             ((60, 260), (190, 350)),
             ((230, 260), (310, 350)),
             ((370, 260), (450, 350))]

graph = create_graph(points, obstacles)
# print("Graph:", graph)
visualize_graph(graph, obstacles)

# # 定义图结构
# graph = {
#     (0, 0): {(1, 0): 1, (0, 1): 1},
#     (1, 0): {(0, 0): 1, (1, 1): 1, (2, 0): 1},
#     (0, 1): {(0, 0): 1, (1, 1): 1},
#     (1, 1): {(1, 0): 1, (0, 1): 1, (2, 1): 1},
#     (2, 0): {(1, 0): 1, (2, 1): 1},
#     (2, 1): {(2, 0): 1, (1, 1): 1, (2, 2): 1},
#     (2, 2): {(2, 1): 1}
# }
#
start = (170, 80)
goal = (420, 375)
path = a_star(graph, start, goal)
print("Path:", path)
