import heapq

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Manchattan


def a_star_search(grid, start, end):
    open_list = []
    closed_list = set()
    
    start_node = (0, start[0], start[1], heuristic(start, end))  # (f, x, y, h)
    heapq.heappush(open_list, start_node)
    
    g_costs = {start: 0}
    parents = {start: None}
    
    while open_list:
        current_f, x, y, current_h = heapq.heappop(open_list)
        
        if (x, y) == end:
            # Path reconstruction
            path = []
            while (x, y) != start:
                path.append((x, y))
                x, y = parents[(x, y)]
            path.append(start)
            return path[::-1]
        
        closed_list.add((x, y))
        
        # Checking neighbors (up, down, left, right)
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        
        for nx, ny in neighbors:
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
                if (nx, ny) in closed_list:
                    continue
                
                g_cost = g_costs[(x, y)] + 1
                h_cost = heuristic((nx, ny), end)
                f_cost = g_cost + h_cost
                
                if (nx, ny) not in g_costs or g_cost < g_costs[(nx, ny)]:
                    g_costs[(nx, ny)] = g_cost
                    parents[(nx, ny)] = (x, y)
                    heapq.heappush(open_list, (f_cost, nx, ny, h_cost))
    
    return None


def dynamic_path_correction(grid, start, end):
    path = a_star_search(grid, start, end)
    
    if path is None:
        return None
    
    for i in range(1, len(path)):
        x, y = path[i]
        

        if grid[x][y] == 1:
            print(f"Obstacle encountered on the path at {x, y}")
            return a_star_search(grid, path[i-1], end)
    
    return path
