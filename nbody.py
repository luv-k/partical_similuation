import pygame
import pymunk
import pymunk.pygame_util
import numpy as np

# Constants
WIDTH, HEIGHT = 800, 600
NUM_PARTICLES = 20
G = 500.0  # Gravitational constant
DT = 0.05  # Time step
ELASTICITY = 0.9  # Elasticity for better bounce
RECOIL_FACTOR = 1.2  # Recoil intensity

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
draw_options = pymunk.pygame_util.DrawOptions(screen)

# Initialize Pymunk space
space = pymunk.Space()
space.gravity = (0, 0)

# Particle class
class Particle:
    def __init__(self, x, y, mass, radius):
        self.mass = mass
        self.radius = radius
        self.body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, radius))
        self.body.position = (x, y)
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.elasticity = ELASTICITY
        space.add(self.body, self.shape)

# Create particles
particles = [Particle(*np.random.uniform([50, 50], [WIDTH-50, HEIGHT-50]), np.random.uniform(5, 20), np.random.uniform(5, 10)) for _ in range(NUM_PARTICLES)]

# Gravity function
def apply_gravity():
    for i, p1 in enumerate(particles):
        for j, p2 in enumerate(particles):
            if i != j:
                dx, dy = p2.body.position - p1.body.position
                dist_sq = dx**2 + dy**2 + 1e-4  # Avoid division by zero
                force = G * p1.mass * p2.mass / dist_sq
                direction = pymunk.Vec2d(dx, dy).normalized()
                p1.body.apply_force_at_local_point(force * direction)

# Main loop
def run_simulation():
    running = True
    while running:
        screen.fill((0, 0, 0))
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        apply_gravity()
        space.step(DT)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    run_simulation()
