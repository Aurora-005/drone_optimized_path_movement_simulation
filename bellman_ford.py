import pygame
import sys
import random
import heapq

pygame.init()

# Grid and display settings
grid_size = 4
cell_size = 100
margin = 5
window_size = (grid_size * cell_size + (grid_size + 1) * margin,
               grid_size * cell_size + (grid_size + 1) * margin)

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
cyan = (0, 255, 255)
green = (0, 255, 0)
red = (255, 0, 0)
blue = (0, 0, 255)
yellow = (255, 255, 0)  # Color for new obstacles

# Set up display
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("A* Algorithm with Drone Movement")

# Initialize font
font = pygame.font.Font(None, 36)

# Generate obstacles with weights
obstacles = {}
num_obstacles = 2
start = (0, 0)
end = (3, 3)

while len(obstacles) < num_obstacles:
    row = random.randint(0, grid_size - 1)
    col = random.randint(0, grid_size - 1)
    if (row, col) != start and (row, col) != end:
        weight = random.randint(1, 3)
        obstacles[(row, col)] = weight

# A* algorithm functions
def heuristic(a, b):
    distance = abs(a[0] - b[0]) + abs(a[1] - b[1])
    if a in obstacles:
        distance += obstacles[a]
    return distance

def astar(start, end):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in neighbors(current):
            tentative_g_score = g_score[current] + obstacles.get(neighbor, 1)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
    return []

def neighbors(node):
    x, y = node
    potential_neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    return [n for n in potential_neighbors if 0 <= n[0] < grid_size and 0 <= n[1] < grid_size]

# Bellman-Ford update for new obstacles
def bellman_ford_update(new_obstacle):
    for y, x in neighbors(new_obstacle):
        if (y, x) in obstacles:
            obstacles[(y, x)] += 2  # Increase cost if already an obstacle
        else:
            obstacles[(y, x)] = 2  # Set initial cost if not in obstacles

# Function to draw the grid and drone
def draw(drone_position, path=[], initial_path=[], new_obstacle=None):
    for row in range(grid_size):
        for col in range(grid_size):
            y = row * cell_size + (row + 1) * margin
            x = col * cell_size + (col + 1) * margin
            if (row, col) == new_obstacle:
                color = yellow  # Draw new obstacle in yellow
            elif (row, col) in obstacles:
                color = cyan
            else:
                color = white
            pygame.draw.rect(screen, color, (x, y, cell_size, cell_size))

            if (row, col) in obstacles:
                weight = obstacles[(row, col)]
                text = font.render(str(weight), True, black)
                text_rect = text.get_rect(center=(x + cell_size // 2, y + cell_size // 2))
                screen.blit(text, text_rect)
    
    # Draw the initial A* path in red
    for i in range(len(initial_path) - 1):
        start = initial_path[i]
        end = initial_path[i + 1]
        pygame.draw.line(screen, red, 
                         (start[1] * cell_size + (start[1] + 1) * margin + cell_size // 2, 
                          start[0] * cell_size + (start[0] + 1) * margin + cell_size // 2),
                         (end[1] * cell_size + (end[1] + 1) * margin + cell_size // 2, 
                          end[0] * cell_size + (end[0] + 1) * margin + cell_size // 2), 4)

    # Draw the updated path in green
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        pygame.draw.line(screen, green, 
                         (start[1] * cell_size + (start[1] + 1) * margin + cell_size // 2, 
                          start[0] * cell_size + (start[0] + 1) * margin + cell_size // 2),
                         (end[1] * cell_size + (end[1] + 1) * margin + cell_size // 2, 
                          end[0] * cell_size + (end[0] + 1) * margin + cell_size // 2), 4)

    # Draw the drone as a cross shape with diagonal wings, centrally aligned
    center_x = drone_position[1] * cell_size + (drone_position[1] + 1) * margin
    center_y = drone_position[0] * cell_size + (drone_position[0] + 1) * margin
    drone_size = cell_size // 4  # Size of each square in the drone

    # Center square (body)
    pygame.draw.rect(screen, blue, (center_x + drone_size, center_y + drone_size, drone_size, drone_size))
    
    # Diagonal wing squares
    pygame.draw.rect(screen, green, (center_x, center_y, drone_size, drone_size))                             # Top-left wing
    pygame.draw.rect(screen, green, (center_x + 2 * drone_size, center_y, drone_size, drone_size))            # Top-right wing
    pygame.draw.rect(screen, green, (center_x, center_y + 2 * drone_size, drone_size, drone_size))            # Bottom-left wing
    pygame.draw.rect(screen, green, (center_x + 2 * drone_size, center_y + 2 * drone_size, drone_size, drone_size))  # Bottom-right wing

# Main loop
running = True
initial_path = astar(start, end)
path = list(initial_path)
current_step = 0
obstacle_added = False
new_obstacle = None

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    screen.fill(black)

    # Draw grid and move drone along the path
    if current_step < len(path):
        draw(path[current_step], path, initial_path, new_obstacle)
        
        # After the drone has moved a few steps, add a new obstacle on the initial path
        if current_step == 2 and not obstacle_added:
            new_obstacle = initial_path[3]  # Place the new obstacle on the path
            obstacles[new_obstacle] = 3  # Add a new obstacle with a weight of 3
            bellman_ford_update(new_obstacle)  # Update paths with Bellman-Ford
            path = astar(path[current_step], end)  # Recalculate path with A*
            current_step = 0  # Reset step count to start following the new path
            obstacle_added = True
        
        current_step += 1
    else:
        draw(path[-1], path, initial_path, new_obstacle)  # Draw at the final position if the path is completed

    pygame.display.flip()
    pygame.time.delay(500)

pygame.quit()
sys.exit()