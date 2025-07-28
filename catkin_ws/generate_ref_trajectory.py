import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from math import sqrt, atan2, pi, cos, sin
import numpy as np
from collections import defaultdict
import heapq

# 环境参数
CORRIDOR_X_MIN = 0.0
CORRIDOR_X_MAX = 40.0
CORRIDOR_Y_MIN = -2.5
CORRIDOR_Y_MAX = 2.5
SAFETY_DISTANCE = 0.4 # 0.4米会堵死路径，0.3米是安全和可通过之间的良好折衷
FLIGHT_HEIGHT = 1.5
DESIRED_SPEED = 1.5  # m/s, 新增期望速度参数

# 精确障碍物定义（位置+安全距离）
OBSTACLES = [
    # 墙壁（带安全距离）
    {"type": "wall", "pos": (20.0, 2.5), "size": (41.15, 0.15 + SAFETY_DISTANCE * 2), "color": "darkblue"},
    {"type": "wall", "pos": (20.0, -2.5), "size": (41.15, 0.15 + SAFETY_DISTANCE * 2), "color": "darkblue"},
    {"type": "wall", "pos": (-0.5, 0.0), "size": (0.15, 5.15 + SAFETY_DISTANCE * 2), "color": "darkblue"},
    {"type": "wall", "pos": (40.5, 0.0), "size": (0.15, 5.15 + SAFETY_DISTANCE * 2), "color": "darkblue"},

    # 箱体障碍物（带安全距离）
    {"type": "box", "pos": (5.192, 1.032), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (9.597, -0.991), "size": (1.8 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (22.581, 1.104), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (22.528, -1.287), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (30.283, 0.060), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (36.087, 0.922), "size": (1.8 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (3.322, -1.210), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (26.500, 0.751), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"},
    {"type": "box", "pos": (16.652, 0.040), "size": (1.0 + SAFETY_DISTANCE * 2, 1.0 + SAFETY_DISTANCE * 2),
     "color": "red"}
]


def get_box_corners(obstacle):
    """获取矩形障碍物的四个角点"""
    center_x, center_y = obstacle["pos"]
    width, height = obstacle["size"]
    
    corners = [
        (center_x - width/2, center_y - height/2),  # 左下
        (center_x + width/2, center_y - height/2),  # 右下
        (center_x + width/2, center_y + height/2),  # 右上
        (center_x - width/2, center_y + height/2)   # 左上
    ]
    return corners


def get_waypoints_for_obstacle(obstacle, offset=0.15):
    """
    为单个障碍物生成“卫星”航点，而不是使用角点本身。
    这能产生更平滑、更安全的路径。
    """
    waypoints = []
    corners = get_box_corners(obstacle)
    center = np.array(obstacle["pos"])
    
    for corner in corners:
        corner_vec = np.array(corner)
        # 从中心指向角点的向量，用于确定向外的方向
        direction_outward = (corner_vec - center)
        if np.linalg.norm(direction_outward) > 1e-6:
            direction_outward /= np.linalg.norm(direction_outward)
        
        # 将航点向外偏移
        waypoint = tuple(corner_vec + direction_outward * offset)
        
        # 确保生成的航点本身不在任何障碍物的安全区内
        if not is_point_in_obstacle(waypoint[0], waypoint[1]):
            waypoints.append(waypoint)
            
    return waypoints


def is_line_clear(p1, p2, sampling_dist=0.1):
    """
    通过沿线采样点来检查两点之间的线段是否无障碍。
    这比几何相交测试更健壮。
    """
    p1 = np.array(p1)
    p2 = np.array(p2)
    length = np.linalg.norm(p2 - p1)
    if length == 0:
        return True
    
    direction = (p2 - p1) / length
    
    # 沿着线段以固定的步长检查点
    # 从一个小的偏移量开始，以避免检查p1本身
    d = sampling_dist
    while d < length:
        point_to_check = p1 + d * direction
        if is_point_in_obstacle(point_to_check[0], point_to_check[1]):
            return False
        d += sampling_dist
        
    return True


def build_visibility_graph():
    """构建可见性图"""
    print("正在构建可见性图...")
    
    # 收集所有关键点（障碍物角点）
    vertices = []
    
    # 添加起点和终点
    start_point = (1.0, 0.0) # 向内移动以确保安全
    end_point = (38.0, 0.0) # 向内移动以确保安全
    vertices.extend([start_point, end_point])
    
    # 为每个障碍物生成“卫星”航点
    for obstacle in OBSTACLES:
        if obstacle["type"] == "box":
            waypoints = get_waypoints_for_obstacle(obstacle)
            vertices.extend(waypoints)
    
    print(f"  总共找到 {len(vertices)} 个关键点 (包括卫星点)")
    
    # 构建邻接表
    graph = defaultdict(list)
    
    # 检查每对顶点之间的可见性
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            p1, p2 = vertices[i], vertices[j]
            
            # 检查两点之间是否可见（无障碍物阻挡）
            if is_line_clear(p1, p2):
                distance = sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                graph[p1].append((p2, distance))
                graph[p2].append((p1, distance))
    
    print(f"  可见性图构建完成，共有 {sum(len(neighbors) for neighbors in graph.values()) // 2} 条边")
    return graph, vertices


def dijkstra_shortest_path(graph, vertices, start, end):
    """使用Dijkstra算法找到最短路径"""
    print(f"正在搜索从 {start} 到 {end} 的最短路径...")
    
    # 初始化距离和前驱节点
    distances = {vertex: float('inf') for vertex in vertices} # 使用所有顶点进行初始化
    distances[start] = 0
    previous = {}
    
    # 优先队列
    pq = [(0, start)]
    visited = set()
    
    while pq:
        current_distance, current_vertex = heapq.heappop(pq)
        
        if current_vertex in visited:
            continue
            
        visited.add(current_vertex)
        
        # 如果到达终点，重构路径
        if current_vertex == end:
            path = []
            while current_vertex in previous:
                path.append(current_vertex)
                current_vertex = previous[current_vertex]
            path.append(start)
            path.reverse()
            print(f"  找到路径，包含 {len(path)} 个节点")
            return path
        
        # 检查邻居
        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_vertex
                heapq.heappush(pq, (distance, neighbor))
    
    print("  未找到路径！")
    return None


def smooth_path_final(path):
    """
    通过移除不必要的中间节点来平滑路径（也称为“拉绳”或“捷径”）。
    """
    if not path or len(path) < 3:
        return path
    
    print("  正在进行最终路径平滑（拉绳算法）...")
    
    smoothed_path = [path[0]]
    current_index = 0
    while current_index < len(path) - 1:
        # 从当前点找到能“看到”的最远节点
        best_next_index = current_index + 1
        for next_index in range(current_index + 2, len(path)):
            if is_line_clear(path[current_index], path[next_index]):
                best_next_index = next_index  # 这是一个有效的捷径
            else:
                break  # 被障碍物挡住了，无法看得更远
        
        smoothed_path.append(path[best_next_index])
        current_index = best_next_index
        
    print(f"  路径平滑完成，从 {len(path)} 个点优化为 {len(smoothed_path)} 个点")
    return smoothed_path


def resample_path(path, sample_time_interval=0.1):
    """
    对最终路径进行重采样，以生成密集的轨迹点。
    """
    if not path:
        return []
        
    print(f"  正在以 {sample_time_interval} 秒为间隔重采样路径...")
    
    resampled_path = [path[0]]
    
    for i in range(len(path) - 1):
        p1 = np.array(path[i])
        p2 = np.array(path[i+1])
        
        segment_dist = np.linalg.norm(p2 - p1)
        # 根据期望速度和采样时间，计算出每个小段的长度
        sample_dist = DESIRED_SPEED * sample_time_interval
        
        num_samples = int(segment_dist / sample_dist)
        
        if num_samples > 0:
            direction = (p2 - p1) / segment_dist
            for j in range(1, num_samples + 1):
                new_point = p1 + j * sample_dist * direction
                resampled_path.append(tuple(new_point))
                
    # 确保最后一个点被包含
    if np.linalg.norm(np.array(resampled_path[-1]) - np.array(path[-1])) > 1e-3:
        resampled_path.append(path[-1])
        
    print(f"  路径重采样完成，点数从 {len(path)} 增加到 {len(resampled_path)}")
    return resampled_path


def generate_loop_trajectory():
    """使用可见性图生成高质量闭环轨迹"""
    
    # 1. 构建可见性图
    graph, vertices = build_visibility_graph()
    
    start_point = (1.0, 0.0) # 确保与build_visibility_graph中的定义一致
    end_point = (38.0, 0.0) # 确保与build_visibility_graph中的定义一致
    
    # 2. 生成去程路径
    print("\n=== 生成去程轨迹 ===")
    forward_path = dijkstra_shortest_path(graph, vertices, start_point, end_point)
    if not forward_path:
        print("错误：无法生成去程路径")
        return []
    
    # 3. 生成返程路径
    print("\n=== 生成返程轨迹 ===")
    backward_path = dijkstra_shortest_path(graph, vertices, end_point, start_point)
    if not backward_path:
        print("错误：无法生成返程路径")
        return [(x, y, FLIGHT_HEIGHT) for x, y in forward_path]
    
    # 4. 路径平滑
    print("\n=== 路径优化 ===")
    forward_smooth = smooth_path_final(forward_path)
    backward_smooth = smooth_path_final(backward_path)
    
    # 5. 合并路径
    full_path_2d = forward_smooth[:-1] + backward_smooth
    full_path_3d = [(x, y, FLIGHT_HEIGHT) for x, y in full_path_2d]
    
    print(f"\n=== 路径重采样 ===")
    resampled_path = resample_path(full_path_3d)
    
    print(f"\n=== 轨迹生成完成 ===")
    print(f"总路径点数 (平滑后): {len(full_path_3d)}")
    print(f"总路径点数 (重采样后): {len(resampled_path)}")
    
    return resampled_path


def is_point_in_obstacle(x, y):
    """精确检查点是否在障碍物安全区内"""
    for obs in OBSTACLES:
        if obs["type"] == "wall":
            # 墙壁是特殊矩形（长条形）
            center_x, center_y = obs["pos"]
            width, height = obs["size"]
            
            if (abs(x - center_x) <= width / 2 and abs(y - center_y) <= height / 2):
                return True
        elif obs["type"] == "box":
            # 普通矩形障碍物
            center_x, center_y = obs["pos"]
            width, height = obs["size"]
            
            if (abs(x - center_x) <= width / 2 and abs(y - center_y) <= height / 2):
                return True
    return False


def calculate_repulsion(x, y):
    """计算来自障碍物的斥力"""
    repulsion = np.array([0.0, 0.0])

    for obs in OBSTACLES:
        if obs["type"] == "wall":
            # 处理上下墙
            if obs["pos"][1] > 0:  # 上墙
                if y > obs["pos"][1] - obs["size"][1] / 2:
                    repulsion[1] -= 1.0 / max(0.1, abs(y - obs["pos"][1]))
            else:  # 下墙
                if y < obs["pos"][1] + obs["size"][1] / 2:
                    repulsion[1] += 1.0 / max(0.1, abs(y - obs["pos"][1]))
                    
            # 处理左右墙
            if obs["pos"][0] < 1:  # 左墙
                if x < obs["pos"][0] + obs["size"][0] / 2:
                    repulsion[0] += 1.0 / max(0.1, abs(x - obs["pos"][0]))
            elif obs["pos"][0] > 39:  # 右墙
                if x > obs["pos"][0] - obs["size"][0] / 2:
                    repulsion[0] -= 1.0 / max(0.1, abs(x - obs["pos"][0]))
        elif obs["type"] == "box":
            # 处理箱体障碍物
            center_x, center_y = obs["pos"]
            dx = x - center_x
            dy = y - center_y
            distance = sqrt(dx**2 + dy**2)
            
            if distance < 3.0:  # 影响范围
                repulsion[0] += dx / max(0.1, distance**2)
                repulsion[1] += dy / max(0.1, distance**2)
    
    return repulsion


def write_trajectory_file(points, filename="ref_trajectory_visibility.txt"):
    """
    根据期望速度写入轨迹文件，确保时间戳真实反映了飞行距离
    """
    if not points:
        print("错误：没有轨迹点可写入")
        return
    
    print(f"正在写入轨迹文件: {filename} (期望速度: {DESIRED_SPEED} m/s)")
    
    with open(filename, 'w') as f:
        cumulative_time = 0.0
        # 写入第一个点
        f.write(f"{cumulative_time:.3f} {points[0][0]:.3f} {points[0][1]:.3f} {points[0][2]:.3f}\n")
        
        for i in range(1, len(points)):
            p1 = np.array(points[i-1][:2])  # 仅使用x,y计算距离
            p2 = np.array(points[i][:2])
            
            distance = np.linalg.norm(p2 - p1)
            time_for_segment = distance / DESIRED_SPEED
            cumulative_time += time_for_segment
            
            f.write(f"{cumulative_time:.3f} {points[i][0]:.3f} {points[i][1]:.3f} {points[i][2]:.3f}\n")
    
    print(f"轨迹文件已保存，包含 {len(points)} 个点。")
    print(f"预计总时长: {cumulative_time:.2f} 秒。")


def visualize_trajectory(points, filename="trajectory_visualization_visibility.png"):
    """可视化轨迹"""
    if not points:
        print("没有轨迹点可可视化")
        return
    
    print("正在生成轨迹可视化...")
    
    fig, ax = plt.subplots(figsize=(20, 6))
    
    # 绘制障碍物
    for obs in OBSTACLES:
        if obs["type"] == "wall":
            center_x, center_y = obs["pos"]
            width, height = obs["size"]
            rect = Rectangle((center_x - width/2, center_y - height/2), 
                           width, height, 
                           facecolor=obs["color"], alpha=0.7)
            ax.add_patch(rect)
            ax.text(center_x, center_y, f"({center_x},{center_y})", 
                   ha='center', va='center', fontsize=8, color='white')
        elif obs["type"] == "box":
            center_x, center_y = obs["pos"]
            width, height = obs["size"]
            rect = Rectangle((center_x - width/2, center_y - height/2), 
                           width, height, 
                           facecolor=obs["color"], alpha=0.8)
            ax.add_patch(rect)
            ax.text(center_x, center_y, f"({center_x:.1f},{center_y:.1f})", 
                   ha='center', va='center', fontsize=8, color='white')
    
    # 绘制轨迹
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    
    ax.plot(x_coords, y_coords, 'g-', linewidth=3, label='无人机轨迹')
    ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='起点')
    ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='终点')
    
    # 标记关键点
    num_points_to_mark = 20
    step = max(1, len(points) // num_points_to_mark)
    for i, (x, y, z) in enumerate(points[::step]):
        ax.plot(x, y, 'bo', markersize=3, alpha=0.6)
        ax.text(x, y + 0.2, str(i * step), 
               ha='center', va='bottom', fontsize=6)
    
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(CORRIDOR_X_MIN - 1, CORRIDOR_X_MAX + 1)
    ax.set_ylim(CORRIDOR_Y_MIN - 1, CORRIDOR_Y_MAX + 1)

    # 设置图形属性
    plt.title('无人机轨迹 (可见性图路径规划)')
    plt.xlabel('X坐标 (米)')
    plt.ylabel('Y坐标 (米)')

    # 保存图像
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"轨迹可视化图已保存: {filename}")
    plt.show()


if __name__ == "__main__":
    print("=== 智能无人机轨迹生成器 ===")
    print("使用可见性图 + Dijkstra 算法")
    print("=" * 50)
    
    # 生成轨迹
    trajectory = generate_loop_trajectory()
    
    if trajectory:
        # 写入文件
        write_trajectory_file(trajectory)
        
        # 可视化
        visualize_trajectory(trajectory)
        
        print("\n=== 任务完成 ===")
        print("✓ 轨迹生成成功")
        print("✓ 文件已保存: ref_trajectory_visibility.txt")
        print("✓ 可视化图已保存: trajectory_visualization_visibility.png")
    else:
        print("\n=== 任务失败 ===")
        print("✗ 轨迹生成失败")