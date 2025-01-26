import random

def generate_random_room(width, height):
    """
    Generate a 2D ASCII room of size `width` x `height`.

    - The outer boundary is made of walls:
        * Top/Bottom with '-'
        * Left/Right with '|'
    - Inside the room, we randomly place obstacles using certain ASCII characters.
    - We'll also return a random (x, y) position for the drone if it's an empty spot.
    """
    # Define possible obstacles (including some empty space for variety)
    # Feel free to modify or add more characters.
    obstacles = [' ', ' ', ' ', '_', '\\', '/', '-', ' ']

    # Create a 2D list filled with spaces initially
    grid = [[' ' for _ in range(width)] for _ in range(height)]
 
    # Fill the boundary with walls
    for x in range(width):
        grid[0][x] = '-'          # Top edge
        grid[height - 1][x] = '-' # Bottom edge
    for y in range(height):
        grid[y][0] = '|'          # Left edge
        grid[y][width - 1] = '|'  # Right edge
    """
    # Randomly place obstacles in the interior
    # Skip the boundary (that's the walls)
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            # 30% chance to place an obstacle
            if random.random() < 0.3:
                grid[y][x] = random.choice(obstacles)
    """
    # Now find a random free spot for the drone
    # We'll pick a random interior cell that is not a wall or obstacle
    drone_x, drone_y = None, None

    # We attempt up to 100 times to find a free spot
    for _ in range(100):
        rand_x = random.randint(1, width - 2)
        rand_y = random.randint(1, height - 2)
        if grid[rand_y][rand_x] == ' ':  # empty space
            drone_x, drone_y = rand_x, rand_y
            break

    if drone_x is not None and drone_y is not None:
        grid[drone_y][drone_x] = 'D'  # Place the drone

    return grid

def print_room(grid):
    """Print the 2D ASCII grid to the console."""
    for row in grid:
        print("".join(row))

def main():
    # Define room size
    width = 24
    height = 12

    room_grid = generate_random_room(width, height)
    print_room(room_grid)

if __name__ == "__main__":
    main()