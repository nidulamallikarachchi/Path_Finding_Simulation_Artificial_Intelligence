import time
import queue
from tkinter import *
import heapq
from Read_file import Rows, Cols, Walls, start_id, GOALS, CELL_SIZE

RECTANGLES = {}
RECTANGLE_COORDINATES = {}
RECTANGLE_DIRECTION_MAP = {}
NEIGHBOURS = {}

maze_canvas = None


def update_rectangle_color(rect_id, color):
    if rect_id in RECTANGLES:
        maze_canvas.itemconfig(RECTANGLES[rect_id], fill=color)


def find_neighbours(rect_id):
    neighbors = []

    if rect_id in RECTANGLE_DIRECTION_MAP:
        directions = RECTANGLE_DIRECTION_MAP[rect_id]

        # Check North neighbor
        if directions["N"] and (rect_id - Cols) not in Walls:
            neighbors.append(rect_id - Cols)

            # Check West neighbor
        if directions["W"] and (rect_id - 1) not in Walls and rect_id % Cols != 1:
            neighbors.append(rect_id - 1)

            # Check South neighbor
        if directions["S"] and (rect_id + Cols) not in Walls:
            neighbors.append(rect_id + Cols)

        # Check East neighbor
        if directions["E"] and (rect_id + 1) not in Walls and (rect_id + 1) % Cols != 1:
            neighbors.append(rect_id + 1)



    return neighbors


def initialize_maze(canvas):
    global RECTANGLE_COORDINATES, RECTANGLE_DIRECTION_MAP, RECTANGLES, maze_canvas

    maze_canvas = canvas

    for rect_id in range(1, Rows * Cols + 1):
        x_coordinate = (rect_id - 1) // Cols
        y_coordinate = (rect_id - 1) % Cols
        coordinates = [x_coordinate, y_coordinate]
        RECTANGLE_COORDINATES[rect_id] = coordinates

    for rect_id in RECTANGLE_COORDINATES:
        x_coordinate, y_coordinate = RECTANGLE_COORDINATES[rect_id]
        RECTANGLE_DIRECTION_MAP[rect_id] = {
            "N": x_coordinate > 0,
            "E": y_coordinate < Cols - 1,
            "W": y_coordinate > 0,
            "S": x_coordinate < Rows - 1,
        }

    for rect_id in range(1, Rows * Cols + 1):
        NEIGHBOURS[rect_id] = find_neighbours(rect_id)

    count = 1

    for i in range(Rows):
        for j in range(Cols):
            if count > Rows * Cols:
                break

            x1 = j * CELL_SIZE
            y1 = i * CELL_SIZE
            x2 = (j + 1) * CELL_SIZE
            y2 = (i + 1) * CELL_SIZE

            rect = maze_canvas.create_rectangle(
                x1, y1, x2, y2,
                outline="black",
                fill="white"
            )
            RECTANGLES[count] = rect

            text_x = (x1 + x2) // 2
            text_y = (y1 + y2) // 2

            maze_canvas.create_text(
                text_x, text_y,
                text=f"{count}"
            )

            count += 1

    update_rectangle_color(start_id, "red")

    for wall in Walls:
        update_rectangle_color(wall, "grey")

    for goal in GOALS:
        update_rectangle_color(goal, "lawn green")


def reset_rectangle_colors():
    for rect in range(1, Rows * Cols + 1):
        update_rectangle_color(rect, "white")

    update_rectangle_color(start_id, "red")

    for wall in Walls:
        update_rectangle_color(wall, "grey")

    for goal in GOALS:
        update_rectangle_color(goal, "lawn green")


def animate_visited_nodes(visited_nodes):
    for rect_id in visited_nodes:
        update_rectangle_color(rect_id, "light blue")
        maze_canvas.update()
        time.sleep(0.5)
        update_rectangle_color(rect_id, "white")


def BFS():
    # Starting node
    print("Method = BFS")
    start_node = start_id

    # Queue for BFS traversal
    bfs_queue = queue.Queue()
    bfs_queue.put(start_node)

    # Visited set to keep track of visited nodes
    visited = set([start_node])
    visited_list = [start_node]

    # Parent dictionary to keep track of the path
    parent = {start_node: None}

    while not bfs_queue.empty():
        current_node = bfs_queue.get()

        # Check if current node is a goal node
        if current_node in GOALS:
            # Reconstruct the path from the goal node to the start node
            update_rectangle_color(current_node,"lawn green")
            print(f"Goal: {current_node}")
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parent[current_node]
            path.reverse()
            print(f"No of Nodes: {len(visited_list)}")
            return path  # Return the path from start to goal

        # Explore neighbors
        for neighbor in NEIGHBOURS[current_node]:
            if neighbor not in visited:
                visited.add(neighbor)
                visited_list.append(neighbor)
                parent[neighbor] = current_node
                bfs_queue.put(neighbor)
                update_rectangle_color(neighbor, "aqua")
                maze_canvas.update()
                time.sleep(0.1)

    print("BFS Path Not Found!")
    print()
    return None  # If no path is found


def DFS():
    start_node = start_id
    visited = set()
    visited_list = [start_node]
    parent = {start_node: None}
    path_found = []
    print("Method = DFS")

    def dfs_util(node):
        update_rectangle_color(start_id,"red")
        nonlocal path_found
        if node in GOALS:
            update_rectangle_color(node, "lawn green")
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            path_found = path
            return True
        visited.add(node)
        visited_list.append(node)
        update_rectangle_color(node, "aqua")
        maze_canvas.update()
        time.sleep(0.1)
        for neighbor in NEIGHBOURS[node]:
            if neighbor not in visited:
                parent[neighbor] = node
                if dfs_util(neighbor):
                    return True
        # update_rectangle_color(node, "white")
        maze_canvas.update()
        return False

    dfs_util(start_node)
    print(f"Goal: {path_found[-1]}")
    print(f"No of Nodes: {len(visited_list)}")
    return path_found


def heuristic(node, goals):
    node_x, node_y = RECTANGLE_COORDINATES[node]
    min_dist = float('inf')

    for goal in goals:
        goal_x, goal_y = RECTANGLE_COORDINATES[goal]
        # Calculate Manhattan distance
        dist = abs(node_x - goal_x) + abs(node_y - goal_y)
        if dist < min_dist:
            min_dist = dist

    return min_dist


def GBFS():
    print("Method: GBFS")
    start_node = start_id
    visited_list = []
    goals = GOALS
    visited = set()
    parent = {start_node: None}
    frontier = []
    heapq.heappush(frontier, (heuristic(start_node, goals), start_node))

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node in visited:
            continue

        visited.add(current_node)
        visited_list.append(current_node)
        update_rectangle_color(current_node, "aqua")
        update_rectangle_color(start_node, "red")
        maze_canvas.update()
        time.sleep(0.1)

        if current_node in goals:
            # Reconstruct the path from the goal node to the start node
            update_rectangle_color(current_node, "lawn green")
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parent[current_node]
            path.reverse()
            print(f"Goal: {current_node}")
            print(f"No of Nodes: {len(visited_list)}")
            return path

        for neighbor in NEIGHBOURS[current_node]:
            if neighbor not in visited and (neighbor not in [n for _, n in frontier]):
                parent[neighbor] = current_node
                heapq.heappush(frontier, (heuristic(neighbor, goals), neighbor))

    print("GBFS Path Not Found!")
    return None


def a_star():
    print("Method = A*")
    start_node = start_id
    goals = GOALS
    temp_Goal = None
    visited_list = []
    visited = set()
    parent = {start_node: None}
    g_score = {start_node: 0}
    frontier = []
    heapq.heappush(frontier, (heuristic(start_node, goals), start_node))

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node in visited:
            continue

        visited.add(current_node)
        visited_list.append(current_node)
        update_rectangle_color(current_node, "aqua")
        update_rectangle_color(start_node, "red")
        maze_canvas.update()
        time.sleep(0.1)

        if current_node in goals:
            temp_Goal = current_node
            update_rectangle_color(current_node, "lawn green")
            # Reconstruct the path from the goal node to the start node
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parent[current_node]
            path.reverse()
            print(f"Goal: {temp_Goal}")
            print(f"No of Nodes: {len(visited_list)}")
            return path

        for neighbor in NEIGHBOURS[current_node]:
            if neighbor not in visited:
                tentative_g_score = g_score[current_node] + 1  # Assuming uniform cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    parent[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goals)
                    heapq.heappush(frontier, (f_score, neighbor))

    return None


def uniform_cost_search():
    print("Method = UCS")
    start_node = start_id
    visited_list = []
    temp_goal = None
    goals = GOALS
    visited = set()
    parent = {start_node: None}
    cost = {start_node: 0}
    frontier = []
    heapq.heappush(frontier, (0, start_node))

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node in visited:
            continue

        visited.add(current_node)
        visited_list.append(current_node)
        update_rectangle_color(current_node, "aqua")
        update_rectangle_color(start_node, "red")
        maze_canvas.update()
        time.sleep(0.1)

        if current_node in goals:
            temp_goal = current_node
            update_rectangle_color(current_node, "lawn green")
            # Reconstruct the path from the goal node to the start node
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parent[current_node]
            path.reverse()
            print(f"Goal: {temp_goal}")
            print(f"No of Nodes: {len(visited_list)}")
            return path

        for neighbor in NEIGHBOURS[current_node]:
            if neighbor not in visited:
                new_cost = cost[current_node] + 1  # Assuming uniform cost
                if neighbor not in cost or new_cost < cost[neighbor]:
                    parent[neighbor] = current_node
                    cost[neighbor] = new_cost
                    heapq.heappush(frontier, (new_cost, neighbor))

    return None


def ida_star():
    print("Method = IDA*")
    start_node = start_id
    goals = GOALS
    temp_goal = None
    visited_list = []

    def search(path, g, bound):
        current_node = path[-1]
        f = g + heuristic(current_node, goals)
        if f > bound:
            return f
        if current_node in goals:
            temp_goal = current_node
            print(f"Goal: {temp_goal}")
            print(f"Visited Nodes {len(visited_list)}")
            update_rectangle_color(current_node, "lawn green")
            return "FOUND"
        min_cost = float('inf')
        for neighbor in NEIGHBOURS[current_node]:
            if neighbor not in path:
                path.append(neighbor)

                if neighbor not in visited_list:
                    visited_list.append(neighbor)

                update_rectangle_color(neighbor, "aqua")  # Show exploration
                maze_canvas.update()
                time.sleep(0.1)  # Short delay to see the animation
                result = search(path, g + 1, bound)
                if result == "FOUND":
                    return "FOUND"
                if result < min_cost:
                    min_cost = result
                path.pop()
                update_rectangle_color(neighbor, "white")  # Reset color on backtrack
                maze_canvas.update()
                time.sleep(0.1)  # Short delay to see the animation
        return min_cost

    bound = heuristic(start_node, goals)
    path = [start_node]
    # print(f"Visited Nodes {visited_list}")

    while True:
        reset_rectangle_colors()  # Ensure all nodes are reset before each depth attempt
        maze_canvas.update()
        result = search(path, 0, bound)
        if result == "FOUND":
            # Highlight the IDA* path found
            for node in path:
                if node not in [start_id] + GOALS:
                    update_rectangle_color(node, "pink")
            maze_canvas.update()
            return path
        if result == float('inf'):
            print("No path found from start to any goal using IDA*.")
            return None
        bound = result


def iterative_deepening_dfs():
    start_node = start_id
    goals = GOALS
    visited_list = []
    temp_goal = None


    def dls(node, depth):

        if depth == 0 and node in goals:
            temp_goal = node
            return [node]  # Return the path as a list of nodes if the goal is found
        elif depth > 0:
            for neighbor in NEIGHBOURS[node]:
                if neighbor not in visited:
                    visited.add(neighbor)

                    if neighbor not in visited_list:
                        visited_list.append(neighbor)

                    update_rectangle_color(neighbor, "aqua")  # Show exploration
                    maze_canvas.update()
                    time.sleep(0.01)  # Short delay to see the animation
                    path = dls(neighbor, depth - 1)
                    if path:
                        return [node] + path
                    visited.remove(neighbor)
                    update_rectangle_color(neighbor, "white")  # Reset color on backtrack
                    maze_canvas.update()
                    time.sleep(0.01)  # Short delay to see the animation
        return None

    for depth in range(len(NEIGHBOURS)):  # Assuming the maximum depth could be the number of nodes
        visited = set([start_node])
        reset_rectangle_colors()  # Clear all node colors before search
        maze_canvas.update()
        path = dls(start_node, depth)
        if path:
            # Highlight the IDDFS path found
            for node in path:
                if node not in [start_id] + GOALS:
                    update_rectangle_color(node, "maroon")
                if node in GOALS:
                    update_rectangle_color(node, "lawn green")
            maze_canvas.update()
            print("Method = IDDFS")
            print("Path found at depth", depth)
            print(f"Goal: {temp_goal}")
            print(f"No of Nodes: {len(visited_list)}")
            return path

        # else:
        #     print("No path found at depth", depth)

    print("No path found from start to any goal using IDDFS.")
    return None


def run_BFS():
    reset_rectangle_colors()
    bfs_path = BFS()

    if bfs_path:
        print("BFS Path found:", bfs_path)
        print()
        # Highlight the BFS path
        for node in bfs_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()


def run_DFS():
    reset_rectangle_colors()
    dfs_path = DFS()

    if dfs_path:
        print("DFS Path found:", dfs_path)
        print()
        # Highlight the DFS path
        for node in dfs_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()


def run_UCS():
    reset_rectangle_colors()
    ucs_path = uniform_cost_search()

    if ucs_path:
        print("UCS Path found:", ucs_path)
        print()
        # Highlight the UCS path
        for node in ucs_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()

    else:
        print("No path found from start to any goal.")


def run_GBFS():
    reset_rectangle_colors()
    gbfs_path = GBFS()

    if gbfs_path:
        print("GBFS Path found:", gbfs_path)
        print()
        # Highlight the GBFS path
        for node in gbfs_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()


def run_Astar():
    reset_rectangle_colors()
    a_star_path = a_star()

    if a_star_path:
        print("A* Path found:", a_star_path)
        print()
        # Highlight the A* path
        for node in a_star_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()


def run_IDAstar():
    reset_rectangle_colors()
    ida_star_path = ida_star()

    if ida_star_path:
        print("IDA* Path found:", ida_star_path)
        print()
        # Highlight the IDA* path
        for node in ida_star_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()
    else:
        print("No path found from start to any goal using IDA*.")


def run_IDDFS():
    reset_rectangle_colors()  # Clear all node colors before starting
    iddfs_path = iterative_deepening_dfs()
    if iddfs_path:
        print("IDDFS Path found:", iddfs_path)
        print()
        # Highlight the IDDFS path
        for node in iddfs_path:
            if node not in [start_id] + GOALS:
                update_rectangle_color(node, "maroon")
        maze_canvas.update()
    else:
        print("No path found from start to any goal using IDDFS.")


def main():
    global maze_canvas

    window = Tk()
    window.title("Maze Solver using BFS, DFS, GBFS, A*, and UCS")
    window.geometry(f"{Cols * CELL_SIZE + 20}x{Rows * CELL_SIZE + 20 + 100}")

    maze_canvas = Canvas(window, width=Cols * CELL_SIZE, height=Rows * CELL_SIZE, bg="white")
    maze_canvas.pack(padx=10, pady=10)

    initialize_maze(maze_canvas)

    button_frame = Frame(window)
    button_frame.pack(side=BOTTOM, pady=10)

    bfs_button = Button(button_frame, text="BFS", command=run_BFS)
    bfs_button.pack(side=LEFT, padx=10)

    dfs_button = Button(button_frame, text="DFS", command=run_DFS)
    dfs_button.pack(side=LEFT, padx=10)

    ucs_button = Button(button_frame, text="UCS", command=run_UCS)
    ucs_button.pack(side=LEFT, padx=10)

    gbfs_button = Button(button_frame, text="GBFS", command=run_GBFS)
    gbfs_button.pack(side=LEFT, padx=10)

    a_star_button = Button(button_frame, text="A*", command=run_Astar)
    a_star_button.pack(side=LEFT, padx=10)

    ida_star_button = Button(button_frame, text="IDA*", command=run_IDAstar)
    ida_star_button.pack(side=LEFT, padx=10)

    iddfs_button = Button(button_frame, text="IDDFS", command=run_IDDFS)
    iddfs_button.pack(side=LEFT, padx=10)

    window.mainloop()


if __name__ == '__main__':
    main()
