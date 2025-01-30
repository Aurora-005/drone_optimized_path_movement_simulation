import pygame
import sys
import random
import heapq

pygame.init()

# Grid and display settings
grid_size = 2
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

# Set up display
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("A* Algorithm with Drone Movement")

# Initialize font
font = pygame.font.Font(None, 36)

# Generate obstacles with weights
obstacles = {}
num_obstacles = 8
start = (0, 2)
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
            tentative_g_score = g_score[current] + (obstacles.get(neighbor, 1))
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

# Function to draw the grid and drone
def draw(drone_position, path=[]):
    for row in range(grid_size):
        for col in range(grid_size):
            y = row * cell_size + (row + 1) * margin
            x = col * cell_size + (col + 1) * margin
            color = cyan if (row, col) in obstacles else white
            pygame.draw.rect(screen, color, (x, y, cell_size, cell_size))

            if (row, col) in obstacles:
                weight = obstacles[(row, col)]
                text = font.render(str(weight), True, black)
                text_rect = text.get_rect(center=(x + cell_size // 2, y + cell_size // 2))
                screen.blit(text, text_rect)
    
    # Draw the drone as a cross shape with diagonal wings
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
path = astar(start, end)
current_step = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    screen.fill(black)

    # Draw grid and move drone along the path
    if current_step < len(path):
        draw(path[current_step])
        current_step += 1
    else:
        draw(path[-1])

    pygame.display.flip()
    pygame.time.delay(1500)

pygame.quit()
sys.exit()