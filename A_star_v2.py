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


# # 示例使用
# points = [(170, 80), (340, 80), (30, 90),
#           (30, 230), (170, 225), (200, 235), (340, 230),
#           (30, 375), (200, 375), (340, 375), (420, 375)]
# obstacles = [((180, 50), (310, 200)),
#              ((60, 110), (150, 200)),
#              ((370, 50), (450, 200)),
#              ((60, 260), (190, 350)),
#              ((230, 260), (310, 350)),
#              ((370, 260), (450, 350))]


# print("Graph:", graph)


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
# start = (135, 25)
# goal = (100, 352)
# points.append(start)
# points.append(goal)
# graph = create_graph(points, obstacles)
# path = a_star(graph, start, goal)
# print("Path:", path)
# visualize_graph(graph, obstacles)