import os
import time
import random
import math
from collections import deque

def generate_empty_room(width, height):
    """
    Creates a 2D list (height x width) with boundary walls:
      - top/bottom: '-'
      - left/right: '|'
    No internal obstacles. Each cell = 1 meter.
    """
    grid = [[' ' for _ in range(width)] for _ in range(height)]

    # Top/Bottom walls
    for x in range(width):
        grid[0][x] = '-'
        grid[height - 1][x] = '-'
    # Left/Right walls
    for y in range(height):
        grid[y][0] = '|'
        grid[y][width - 1] = '|'

    return grid

def print_room(grid):
    """(Not used in final printing) Just for reference."""
    for row in grid:
        print("".join(row))

def build_map_lines(grid):
    """
    Convert the 2D 'grid' into a list of strings, each representing one row.
    We'll later zip these rows with the altitude bar rows to display side-by-side.
    """
    lines = []
    for row in grid:
        lines.append("".join(row))
    return lines

def build_alt_bar_lines(alt, max_alt=3.0, total_lines=10):
    """
    Create 'total_lines' strings that form a simple vertical bar chart
    for the current altitude 'alt' in range [0..max_alt].

    The top line corresponds to ~3.0m, the bottom line to ~0.0m.
    We fill lines with '###' if alt is above the threshold, else '   '.
    We'll also label top and bottom lines for clarity.
    """
    lines = []
    if total_lines < 2:
        return ["(alt chart)"]

    # alt step for each row
    # row=0 is top => threshold ~3.0
    # row=total_lines-1 is bottom => threshold ~0.0
    step = max_alt / (total_lines - 1)

    for i in range(total_lines):
        # i=0 => top
        # i=total_lines-1 => bottom
        # threshold altitude for this row:
        threshold = max_alt - (i * step)

        # Fill or not:
        if alt >= threshold:
            bar = "###"
        else:
            bar = "   "

        # We'll label top and bottom lines
        if i == 0:
            label = f"{max_alt:.1f}"  # top label
        elif i == total_lines - 1:
            label = "0.0"            # bottom label
        else:
            label = "   "

        # Format e.g. "### 3.0"
        # Enough spacing to keep columns aligned
        line_str = f"{bar} {label:3s}"
        lines.append(line_str)

    return lines

def get_random_interior_cell(width, height, grid):
    """Return a random cell (x,y) strictly inside the walls."""
    while True:
        x = random.randint(1, width - 2)
        y = random.randint(1, height - 2)
        if grid[y][x] == ' ':
            return x, y

def bfs_interior(width, height, start_x, start_y):
    """
    BFS from (start_x, start_y) for all interior cells.
    Returns a list of (x,y) in BFS order.
    """
    visited = {(start_x, start_y)}
    order = []
    queue = deque([(start_x, start_y)])

    while queue:
        x, y = queue.popleft()
        order.append((x, y))
        for nx, ny in [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]:
            if 1 <= nx <= width - 2 and 1 <= ny <= height - 2:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

    return order

def ascend(grid, x, y, state, target_alt=3.0, alt_step=0.2, delay=0.05):
    """Ascend in place from current altitude to target_alt."""
    while state['drone_alt'] < target_alt:
        state['drone_alt'] = min(state['drone_alt'] + alt_step, target_alt)
        update_view(grid, x, y, dx=0, dy=0, state=state, delay=delay)

def descend(grid, x, y, state, target_alt=0.0, alt_step=0.2, delay=0.05):
    """Descend in place from current altitude down to target_alt."""
    while state['drone_alt'] > target_alt:
        state['drone_alt'] = max(state['drone_alt'] - alt_step, target_alt)
        update_view(grid, x, y, dx=0, dy=0, state=state, delay=delay)

def move_2d(grid, start_x, start_y, end_x, end_y, state, delay=0.05):
    """
    Move in X/Y at the current altitude. Simple Manhattan path.
    """
    drone_char = 'D'
    if grid[start_y][start_x] == drone_char:
        grid[start_y][start_x] = ' '

    drone_x, drone_y = start_x, start_y

    # Horizontal
    while drone_x < end_x:
        drone_x += 1
        update_view(grid, drone_x, drone_y, dx=1, dy=0, state=state, delay=delay)
    while drone_x > end_x:
        drone_x -= 1
        update_view(grid, drone_x, drone_y, dx=-1, dy=0, state=state, delay=delay)

    # Vertical
    while drone_y < end_y:
        drone_y += 1
        update_view(grid, drone_x, drone_y, dx=0, dy=1, state=state, delay=delay)
    while drone_y > end_y:
        drone_y -= 1
        update_view(grid, drone_x, drone_y, dx=0, dy=-1, state=state, delay=delay)

    return drone_x, drone_y

def update_view(grid, x, y, dx, dy, state, delay=0.05):
    """
    1. Place drone at (x,y).
    2. Compute IMU => roll=0, pitch=0, yaw from (dx,dy), alt from state.
    3. Build 'map_lines' and 'alt_lines' for side-by-side printing.
    4. Print them, then print IMU line.
    5. Restore old char, sleep.
    """
    drone_char = 'D'

    old_char = grid[y][x]
    grid[y][x] = drone_char

    # --- Compute IMU ---
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    if dx != 0 or dy != 0:
        yaw = math.degrees(math.atan2(dy, dx))
        if yaw < 0:
            yaw += 360
    alt = state['drone_alt']

    # --- Build side-by-side output ---
    # 1) Convert the grid to lines
    map_lines = build_map_lines(grid)
    # 2) Build altitude bar lines
    alt_lines = build_alt_bar_lines(alt, max_alt=3.0, total_lines=len(map_lines))

    # Clear screen
    os.system('clear' if os.name == 'posix' else 'cls')

    # Print them side by side
    for ml, al in zip(map_lines, alt_lines):
        print(f"{ml}   {al}")

    # Print IMU line at bottom
    print(f"\nIMU => roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°, alt={alt:.1f}m")

    # Restore old char
    grid[y][x] = old_char

    time.sleep(delay)

def main():
    width = 15
    height = 10

    # We'll tile from corner (1,1)
    corner_x, corner_y = 1, 1

    # Create room
    grid = generate_empty_room(width, height)

    # Drone start at random interior
    start_x, start_y = get_random_interior_cell(width, height, grid)

    # BFS order from (1,1)
    bfs_cells = bfs_interior(width, height, corner_x, corner_y)

    # State keeps track of altitude
    state = {
        'drone_alt': 0.0  # start on the floor
    }

    step_delay = 0.1
    alt_step = 0.2
    flight_alt = 2.0  # "ceiling"

    for i, (tx, ty) in enumerate(bfs_cells, start=1):
        # 1) Ascend at START from 0 -> flight_alt
        ascend(grid, start_x, start_y, state, target_alt=flight_alt, alt_step=alt_step, delay=step_delay)

        # 2) Move to BFS cell
        drone_x, drone_y = move_2d(grid, start_x, start_y, tx, ty, state, delay=step_delay)

        # 3) Descend to floor
        descend(grid, drone_x, drone_y, state, target_alt=0.0, alt_step=alt_step, delay=step_delay)

        # 4) Place tile
        grid[ty][tx] = '#'
        # Show after placing tile (just clear & re-print once)
        os.system('clear' if os.name == 'posix' else 'cls')
        map_lines = build_map_lines(grid)
        alt_lines = build_alt_bar_lines(state['drone_alt'], max_alt=flight_alt, total_lines=len(map_lines))
        for ml, al in zip(map_lines, alt_lines):
            print(f"{ml}   {al}")
        print(f"\nPlaced tile #{i} at ({tx},{ty}).")
        time.sleep(step_delay)

        # 5) Ascend again from 0 -> flight_alt
        ascend(grid, tx, ty, state, target_alt=flight_alt, alt_step=alt_step, delay=step_delay)

        # 6) Move back to START
        drone_x, drone_y = move_2d(grid, tx, ty, start_x, start_y, state, delay=step_delay)

        # 7) Descend to 0 at START
        descend(grid, drone_x, drone_y, state, target_alt=0.0, alt_step=alt_step, delay=step_delay)

    print("\nAll interior cells have been tiled in BFS order from corner (1,1)!")
    print("Drone is now back at the start on the floor. Simulation complete.")

if __name__ == "__main__":
    main()
