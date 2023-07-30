import heapq

def heuristic(p1, p2):
    # Manhattan distance heuristic
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def get_neighbors(node, grid):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    rows, cols = len(grid), len(grid[0])
    
    for dx, dy in directions:
        x, y = node[0] + dx, node[1] + dy
        if 0 <= x < rows and 0 <= y < cols and grid[x][y] == 0:
            neighbors.append((x, y))
            
    return neighbors

def a_star(start, goal, grid):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {start: None}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        
        for neighbor in get_neighbors(current, grid):
            tentative_g_score = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
                
    return None

def print_path(path, grid):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if (i, j) in path:
                print("X", end=" ")
            else:
                print(grid[i][j], end=" ")
        print()

def print_path_to_file(path, grid, file_path):
    with open(file_path, "w") as f:
        f.write("{")
        for i in range(len(grid)):
            f.write("{")
            for j in range(len(grid[0])):
                if (i, j) in path:
                    f.write("2")
                else:
                    f.write(str(grid[i][j]))
                if j < len(grid[0]) - 1:
                    f.write(", ")
            f.write("}")
            if i < len(grid) - 1:
                f.write(",\n")
        f.write("};\n")

if __name__ == "__main__":
    # Example map matrix
    grid = [
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 0, 1],
        [1, 1, 1, 0, 0, 1, 0, 1],
        [1, 1, 0, 0, 0, 1, 0, 1],
        [1, 1, 0, 1, 1, 1, 0, 1],
        [1, 1, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1]
    ]
    
    start = (1, 1)
    goal = (6, 6)
    
    path = a_star(start, goal, grid)
    if path:
        print("Optimal Path:")
        print_path(path, grid)
        print_path_to_file(path, grid, "data.txt")
    else:
        print("No path found!")
