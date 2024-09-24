import run

Rows = None
Cols = None

RECTANGLE_COORDINATES = {}
Walls = []
start_id = 0
GOALS = [1]


def get_rectangle_ids_using_coordinates(x, y):
    for rect_id, coordinates in RECTANGLE_COORDINATES.items():
        if coordinates == [x, y]:
            return rect_id
    return None


def initialize_maze():
    global RECTANGLE_COORDINATES
    RECTANGLE_COORDINATES = {}
    for rect_id in range(1, Rows * Cols + 1):
        x_coordinate = (rect_id - 1) // Cols
        y_coordinate = (rect_id - 1) % Cols
        coordinates = [x_coordinate, y_coordinate]
        RECTANGLE_COORDINATES[rect_id] = coordinates


def get_rect_ids(start_x, start_y, width, height):
    rectangle_ids = []
    for rect_id, coords in RECTANGLE_COORDINATES.items():
        x_coord, y_coord = coords
        if (start_y <= x_coord < start_y + height) and (start_x <= y_coord < start_x + width):
            rectangle_ids.append(rect_id)
    return rectangle_ids


def build_wall(notation):
    global Walls
    start_x, start_y, width, height = notation
    identified_rectangles = get_rect_ids(start_x, start_y, width, height)
    if identified_rectangles:
        Walls.extend(identified_rectangles)


def process_other_coordinates():
    global Rows, Cols, start_id, GOALS

    with open(file_path, 'r') as file:
        lines = file.readlines()[:3]
        count = 0
        for line in lines:
            count += 1
            if count == 1:
                if line.strip():
                    width_height = tuple(map(int, line.strip().strip('[]').split(',')))
                    Rows = width_height[0]
                    Cols = width_height[1]
            elif count == 2:
                if line.strip():
                    initial_state = tuple(map(int, line.strip().strip('()').split(',')))
                    start_x = initial_state[1]
                    start_y = initial_state[0]
                    start_id = get_rectangle_ids_using_coordinates(start_x, start_y)
            elif count == 3:
                tuple_strings = line.split('|')
                goals_array = []

                for tuple_str in tuple_strings:
                    numbers = tuple(map(int, tuple_str.strip().strip('()').split(',')))
                    goals_array.append(numbers)
                #
                count = 0
                GOALS.clear()
                for goal in goals_array:
                    goal_x = goal[1]
                    goal_y = goal[0]
                    goal_id = get_rectangle_ids_using_coordinates(goal_x, goal_y)
                    GOALS.append(goal_id)



def process_wall_notations_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()[3:]
        for line in lines:
            if line.strip():
                notation = tuple(map(int, line.strip().strip('()').split(',')))
                build_wall(notation)


def calculate_cell_size(rows, cols, max_window_size=(1080, 720)):
    """
    Calculate the maximum possible cell size based on the maximum window dimensions.
    """
    max_width, max_height = max_window_size
    cell_width = max_width // cols
    cell_height = max_height // rows
    return min(cell_width, cell_height)


# Initialize maze coordinates
file_path = run.file_path
# process_wall_notations_from_file(file_path)

process_other_coordinates()
initialize_maze()
process_other_coordinates()
process_wall_notations_from_file(file_path)
CELL_SIZE = calculate_cell_size(Rows, Cols)

