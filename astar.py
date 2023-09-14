class SimplePriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        self.elements.append((priority, item))
        self.elements.sort(reverse=True)  # Sort by priority

    def get(self):
        return self.elements.pop()[1]  # Return the item with highest priority

def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star_search(grid, start, goal):
    open_list = SimplePriorityQueue()
    open_list.put(start, 0)
    
    came_from = {start: None}
    g_score = {start: 0}

    while not open_list.empty():
        current = open_list.get()

        if current == goal:  # Directly check if the current node is the goal
            return reconstruct_path(came_from, start, goal)

        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
            x, y = current[0] + dx, current[1] + dy

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != -2:
                neighbor = (x, y)
                tentative_g_score = g_score.get(neighbor, float('inf'))
                new_g_score = g_score[current] + 1

                if new_g_score < tentative_g_score:
                    came_from[neighbor] = current
                    g_score[neighbor] = new_g_score
                    priority = new_g_score + heuristic(neighbor, goal)
                    open_list.put(neighbor, priority)

    return None

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []

    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path