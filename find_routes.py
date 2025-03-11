import xml.etree.ElementTree as ET
import math
import heapq
import folium
import sys
import requests
import webbrowser
from IPython.display import display

# --- 1. 출발지/도착지 좌표 기반 bounding box 계산 ---
def compute_bbox(start_lat, start_lon, dest_lat, dest_lon, margin=0.02):
    """출발지와 도착지 좌표를 기준으로 여유를 둔 bounding box 계산"""
    south = min(start_lat, dest_lat) - margin
    north = max(start_lat, dest_lat) + margin
    west  = min(start_lon, dest_lon) - margin
    east  = max(start_lon, dest_lon) + margin
    return (south, west, north, east)

# --- 2. OSM 데이터 다운로드 (Overpass API 사용) ---
def download_osm_data(bbox, output_file):
    """Overpass API를 통해 OSM 데이터를 다운로드"""
    overpass_url = "http://overpass-api.de/api/interpreter"
    query = f"""
    <osm-script output="xml" timeout="25">
      <union>
        <query type="way">
          <has-kv k="highway"/>
          <bbox-query s="{bbox[0]}" w="{bbox[1]}" n="{bbox[2]}" e="{bbox[3]}"/>
        </query>
      </union>
      <union>
        <item/>
        <recurse type="way-node"/>
      </union>
      <print mode="body"/>
    </osm-script>
    """
    print("OSM 데이터를 다운로드 중입니다...")
    response = requests.get(overpass_url, params={'data': query})
    if response.status_code != 200:
        print("OSM 데이터 다운로드 실패")
        sys.exit(1)
    with open(output_file, 'wb') as f:
        f.write(response.content)
    print(f"OSM 데이터가 '{output_file}'에 저장되었습니다.")

# --- 3. 해버사인 공식 (두 좌표 간 거리 계산) ---
def haversine(lat1, lon1, lat2, lon2):
    """두 지점 간 거리(km)를 계산"""
    R = 6371.0  # 지구 반경 (km)
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    return R * c

# --- 4. OSM 데이터 파싱 및 그래프 구성 ---
def parse_osm(osm_file):
    """OSM 파일을 파싱하여 노드와 그래프 생성"""
    tree = ET.parse(osm_file)
    root = tree.getroot()
    nodes = {}  # {node_id: (lat, lon)}
    graph = {}  # {node_id: [(neighbor_id, distance), ...]}
    
    for node in root.findall('node'):
        node_id = node.attrib['id']
        lat = float(node.attrib['lat'])
        lon = float(node.attrib['lon'])
        nodes[node_id] = (lat, lon)
        graph[node_id] = []
    
    for way in root.findall('way'):
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in way.findall('tag')}
        if 'highway' in tags:
            nd_refs = [nd.attrib['ref'] for nd in way.findall('nd')]
            for i in range(len(nd_refs) - 1):
                n1, n2 = nd_refs[i], nd_refs[i + 1]
                if n1 in nodes and n2 in nodes:
                    lat1, lon1 = nodes[n1]
                    lat2, lon2 = nodes[n2]
                    distance = haversine(lat1, lon1, lat2, lon2)
                    graph[n1].append((n2, distance))
                    graph[n2].append((n1, distance))
    return nodes, graph

# --- 5. 입력 좌표와 가장 가까운 노드 찾기 ---
def find_nearest_node(lat, lon, nodes):
    """주어진 좌표와 가장 가까운 노드 ID 반환"""
    min_dist = float('inf')
    nearest = None
    for node_id, (nlat, nlon) in nodes.items():
        d = haversine(lat, lon, nlat, nlon)
        if d < min_dist:
            min_dist = d
            nearest = node_id
    return nearest

# --- 6. 휴리스틱 함수 ---
def heuristic_haversine(current, goal, nodes):
    """A* 및 Greedy 알고리즘용 Haversine 휴리스틱"""
    lat1, lon1 = nodes[current]
    lat2, lon2 = nodes[goal]
    return haversine(lat1, lon1, lat2, lon2)

def heuristic_zero(current, goal, nodes):
    """Dijkstra용 휴리스틱 (0)"""
    return 0

# --- 7. 경로 탐색 알고리즘 ---
def a_star_search(graph, start, goal, nodes, heuristic_func):
    """A* 알고리즘"""
    open_set = []
    heapq.heappush(open_set, (heuristic_func(start, goal, nodes), start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic_func(start, goal, nodes)
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, g_score[goal]
        
        for neighbor, distance in graph[current]:
            tentative_g = g_score[current] + distance
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic_func(neighbor, goal, nodes)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None, float('inf')

def greedy_best_first_search(graph, start, goal, nodes, heuristic_func):
    """Greedy Best-First Search"""
    open_set = []
    heapq.heappush(open_set, (heuristic_func(start, goal, nodes), start))
    came_from = {}
    visited = set()
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, None
        if current in visited:
            continue
        visited.add(current)
        for neighbor, _ in graph[current]:
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(open_set, (heuristic_func(neighbor, goal, nodes), neighbor))
    return None, None

def bfs_search(graph, start, goal):
    """Breadth-First Search"""
    queue = [(start, [start])]
    visited = set()
    
    while queue:
        node, path = queue.pop(0)
        if node == goal:
            return path, None
        if node not in visited:
            visited.add(node)
            for neighbor, _ in graph[node]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
    return None, None

def dfs_search(graph, start, goal):
    """Depth-First Search"""
    stack = [(start, [start])]
    visited = set()
    
    while stack:
        node, path = stack.pop()
        if node == goal:
            return path, None
        if node not in visited:
            visited.add(node)
            for neighbor, _ in graph[node]:
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))
    return None, None

# --- 8. 경로의 총 이동 거리 계산 ---
def compute_path_cost(path, graph):
    """경로의 총 거리 계산 (Greedy, BFS, DFS용)"""
    total = 0
    for i in range(len(path) - 1):
        node1, node2 = path[i], path[i + 1]
        for neighbor, dist in graph[node1]:
            if neighbor == node2:
                total += dist
                break
    return total

# --- 9. 지도에 경로 추가 ---
def add_route_to_map(m, nodes, path, color, tooltip_text):
    """지도에 경로를 PolyLine으로 추가"""
    route_coords = [nodes[node] for node in path]
    folium.PolyLine(route_coords, color=color, weight=5, opacity=0.8, tooltip=tooltip_text).add_to(m)

# --- 10. 기본 지도 생성 ---
def create_base_map(start_coords, goal_coords, zoom_start=13):
    """출발지와 도착지 중심의 지도 생성"""
    mid_lat = (start_coords[0] + goal_coords[0]) / 2
    mid_lon = (start_coords[1] + goal_coords[1]) / 2
    m = folium.Map(location=[mid_lat, mid_lon], zoom_start=zoom_start)
    folium.Marker(location=start_coords, popup="Start", icon=folium.Icon(color='green')).add_to(m)
    folium.Marker(location=goal_coords, popup="Goal", icon=folium.Icon(color='red')).add_to(m)
    return m

# --- 11. 메인 실행 블록 ---
if __name__ == '__main__':
    # 출발지 및 도착지 좌표
    start_lat, start_lon = 37.55, 126.95
    dest_lat, dest_lon = 37.58, 126.98

    # Bounding box 계산
    bbox = compute_bbox(start_lat, start_lon, dest_lat, dest_lon)
    print("계산된 bounding box:", bbox)
    
    osm_file = 'downloaded_map.osm'
    download_osm_data(bbox, osm_file)
    
    print("OSM 데이터 파싱 중...")
    nodes, graph = parse_osm(osm_file)
    print(f"파싱 완료. 노드 수: {len(nodes)}")
    
    # 가장 가까운 노드 찾기
    start_node = find_nearest_node(start_lat, start_lon, nodes)
    goal_node = find_nearest_node(dest_lat, dest_lon, nodes)
    print(f"출발 노드: {start_node}, 도착 노드: {goal_node}")
    
    # 알고리즘 선택
    print("\n탐색 알고리즘 선택:")
    print("1. A* (Haversine)")
    print("2. Dijkstra")
    print("3. Greedy Best-First Search")
    print("4. BFS")
    print("5. DFS")
    print("6. 모두 실행")
    choice = input("선택 (1-6): ")
    
    # 알고리즘 설정
    algo_settings = {
        "A*": {"func": lambda: a_star_search(graph, start_node, goal_node, nodes, heuristic_haversine), "color": "blue"},
        "Dijkstra": {"func": lambda: a_star_search(graph, start_node, goal_node, nodes, heuristic_zero), "color": "green"},
        "Greedy": {"func": lambda: greedy_best_first_search(graph, start_node, goal_node, nodes, heuristic_haversine), "color": "red"},
        "BFS": {"func": lambda: bfs_search(graph, start_node, goal_node), "color": "purple"},
        "DFS": {"func": lambda: dfs_search(graph, start_node, goal_node), "color": "orange"}
    }
    
    selected_algos = {}
    if choice in ['1', '2', '3', '4', '5']:
        algo_names = ["A*", "Dijkstra", "Greedy", "BFS", "DFS"]
        selected_algos[algo_names[int(choice) - 1]] = algo_settings[algo_names[int(choice) - 1]]
    elif choice == '6':
        selected_algos = algo_settings
    else:
        print("잘못된 선택입니다.")
        sys.exit(1)
    
    # 알고리즘 실행
    results = {}
    for algo_name, settings in selected_algos.items():
        print(f"\n{algo_name} 실행 중...")
        path, cost = settings["func"]()
        if path:
            cost = cost if cost is not None else compute_path_cost(path, graph)
            results[algo_name] = {"path": path, "cost": cost, "color": settings["color"]}
            print(f"{algo_name}: {cost:.2f} km")
        else:
            print(f"{algo_name} 경로 찾기 실패")
    
    if not results:
        print("경로를 찾지 못했습니다.")
        sys.exit(1)
    
    # 개별 지도 생성 및 출력
    start_coords, goal_coords = nodes[start_node], nodes[goal_node]
    for algo_name, data in results.items():
        m_single = create_base_map(start_coords, goal_coords)
        tooltip = f"{algo_name}: {data['cost']:.2f} km"
        add_route_to_map(m_single, nodes, data["path"], data["color"], tooltip)
        filename = f"result_{algo_name.replace(' ', '_')}.html"
        m_single.save(filename)
        print(f"{algo_name} 지도 저장: {filename}")
        display(m_single)  # Jupyter Notebook 출력
    
    # 통합 지도 생성 및 출력
    m_all = create_base_map(start_coords, goal_coords)
    for algo_name, data in results.items():
        tooltip = f"{algo_name}: {data['cost']:.2f} km"
        add_route_to_map(m_all, nodes, data["path"], data["color"], tooltip)
    combined_filename = "result_all.html"
    m_all.save(combined_filename)
    print(f"통합 지도 저장: {combined_filename}")
    display(m_all)  # Jupyter Notebook 출력
    webbrowser.open(combined_filename)