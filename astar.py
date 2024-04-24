import heapq

# A class to represent each node in the grid
class Node:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

# A function to calculate the Manhattan distance between two nodes
def heuristic(node, goal):
    return abs(node.x - goal.x) + abs(node.y - goal.y)

# A function to perform the A* search algorithm
def a_star_search(start, goal, grid):
    # Create the start and goal nodes
    start_node = Node(start[0], start[1], grid[start[0]][start[1]])
    goal_node = Node(goal[0], goal[1], grid[goal[0]][goal[1]])

    # Create the open and closed lists
    open_list = []
    closed_list = set()

    # Add the start node to the open list
    heapq.heappush(open_list, start_node)

    # Loop until the open list is empty
    while open_list:
        # Get the node with the lowest f score from the open list
        current_node = heapq.heappop(open_list)

        # Check if we have reached the goal
        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            path = []
            while current_node is not None:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        # Add the current node to the closed list
        closed_list.add((current_node.x, current_node.y))

        # Generate the neighbors of the current node
        for x, y in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor_x = current_node.x + x
            neighbor_y = current_node.y + y

            # Check if the neighbor is in the grid
            if neighbor_x < 0 or neighbor_x >= len(grid) or neighbor_y < 0 or neighbor_y >= len(grid[0]):
                continue

            # Check if the neighbor is an obstacle or has already been visited
            if grid[neighbor_x][neighbor_y] == -1 or (neighbor_x, neighbor_y) in closed_list:
                continue

            # Create the neighbor node
            neighbor_node = Node(neighbor_x, neighbor_y, grid[neighbor_x][neighbor_y], current_node)

            # Calculate the g score of the neighbor node
            neighbor_node.g = current_node.g + neighbor_node.cost

            # Calculate the h score of the neighbor node
            neighbor_node.h = heuristic(neighbor_node, goal_node)

            # Calculate the f score of the neighbor node
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # Add the neighbor node to the open list
            heapq.heappush(open_list, neighbor_node)

    # If we reach this point, there is no path to the goal
    return None

# Example usage
start = (0, 0)
goal = (3, 3)
grid = [[0, 1, 2, 3],
        [1, 2, -1, 4],
        [2, -1, 4, 5],
        [3, 4, 5, 6]]

path = a_star_search(start, goal, grid)
print(path)
