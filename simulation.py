import pygame
import numpy as np
import random

# --- Constants ---
WIDTH, HEIGHT = 800, 600
try:
    # Keep the input, but provide a default if non-integer is entered
    NUM_PARTICLES = int(input("Enter the number of particles (e.g., 10-20): "))
    if NUM_PARTICLES <= 0:
        print("Number of particles must be positive. Setting to 10.")
        NUM_PARTICLES = 10
except ValueError:
    print("Invalid input. Setting number of particles to 10.")
    NUM_PARTICLES = 10

G = 6.674e-1 # Adjusted G for simulation scale, find value that looks good
DT = 0.05    # Time step - smaller is more accurate but slower
ELASTICITY = 0.8  # Coefficient of restitution for collisions (0=inelastic, 1=perfectly elastic)
MIN_MASS, MAX_MASS = 5.0, 20.0
MIN_RADIUS, MAX_RADIUS = 4.0, 10.0
VEL_LIMIT = 5.0 # Initial velocity spread
SOFTENING_FACTOR = 1 # Epsilon^2 in F = G*m1*m2 / (r^2 + eps^2), prevents extreme forces at close range

# --- Colors ---
BACKGROUND_COLOR = (10, 10, 20)
PARTICLE_COLOR = (200, 200, 255)
FPS_COLOR = (255, 255, 0)

# --- Particle Class ---
class Particle:
    def __init__(self, x, y, vx, vy, mass, radius):
        self.pos = np.array([x, y], dtype=float)
        self.vel = np.array([vx, vy], dtype=float)
        self.acc = np.zeros(2, dtype=float)
        self.mass = mass
        self.radius = radius
        self.color = PARTICLE_COLOR

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.pos.astype(int), int(self.radius))

# --- Simulation Functions ---
def initialize_particles(num, width, height):
    particles = []
    for _ in range(num):
        mass = random.uniform(MIN_MASS, MAX_MASS)
        radius = np.sqrt(mass) * 1.5 # Radius somewhat proportional to sqrt(mass)
        # Ensure particles don't start exactly on the edge
        padding = radius + 5
        pos = np.array([
            random.uniform(padding, width - padding),
            random.uniform(padding, height - padding)
        ])
        vel = np.random.uniform(-VEL_LIMIT, VEL_LIMIT, 2)
        particles.append(Particle(pos[0], pos[1], vel[0], vel[1], mass, radius))
    return particles

def calculate_gravity(particles):
    for p in particles:
        p.acc[:] = 0.0 # Reset acceleration for this step

    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            p1 = particles[i]
            p2 = particles[j]

            direction = p2.pos - p1.pos
            dist_sq = np.sum(direction**2)

            # Avoid division by zero and extreme forces using softening
            if dist_sq < 1e-4: # Avoid self-interaction issues if particles somehow perfectly overlap
                continue

            dist = np.sqrt(dist_sq)
            force_magnitude = G * p1.mass * p2.mass / (dist_sq + SOFTENING_FACTOR)
            force_vector = force_magnitude * direction / (dist + 1e-6) # Add small epsilon to avoid division by zero if dist is tiny

            p1.acc += force_vector / p1.mass
            p2.acc -= force_vector / p2.mass # Newton's third law

def update_particles(particles, dt):
    # Velocity Verlet: Step 1 - Update velocity by half step
    for p in particles:
        p.vel += 0.5 * p.acc * dt

    # Velocity Verlet: Step 2 - Update position fully
    for p in particles:
        p.pos += p.vel * dt

    # Velocity Verlet: Step 3 - Recalculate forces/accelerations at new positions
    calculate_gravity(particles) # Forces depend on the new positions

    # Velocity Verlet: Step 4 - Update velocity by the second half step
    for p in particles:
        p.vel += 0.5 * p.acc * dt

def handle_collisions(particles, width, height):
    num_particles = len(particles)

    # Particle-Particle Collisions
    for i in range(num_particles):
        for j in range(i + 1, num_particles):
            p1 = particles[i]
            p2 = particles[j]

            direction = p2.pos - p1.pos
            dist_sq = np.sum(direction**2)
            min_dist = p1.radius + p2.radius
            min_dist_sq = min_dist**2

            if dist_sq < min_dist_sq:
                dist = np.sqrt(dist_sq)
                normal = direction / (dist + 1e-6) # Normalize direction vector

                # --- Resolve Overlap ---
                overlap = min_dist - dist
                # Move particles apart proportionally to their inverse mass (lighter moves more)
                total_inv_mass = (1.0 / p1.mass) + (1.0 / p2.mass)
                if total_inv_mass < 1e-6: # Avoid division by zero if masses are huge
                    total_inv_mass = 1e-6
                p1.pos -= normal * overlap * (1.0 / p1.mass) / total_inv_mass
                p2.pos += normal * overlap * (1.0 / p2.mass) / total_inv_mass

                # --- Collision Response (Impulse) ---
                relative_vel = p1.vel - p2.vel
                vel_along_normal = np.dot(relative_vel, normal)

                # Only resolve if particles are moving towards each other
                if vel_along_normal < 0:
                    impulse_scalar = -(1 + ELASTICITY) * vel_along_normal
                    impulse_scalar /= (1 / p1.mass + 1 / p2.mass)

                    impulse_vector = impulse_scalar * normal

                    p1.vel += impulse_vector / p1.mass
                    p2.vel -= impulse_vector / p2.mass

    # Wall Collisions
    for p in particles:
        # Left wall
        if p.pos[0] - p.radius < 0:
            p.pos[0] = p.radius # Correct position
            p.vel[0] = abs(p.vel[0]) * ELASTICITY # Reverse velocity component
        # Right wall
        elif p.pos[0] + p.radius > width:
            p.pos[0] = width - p.radius
            p.vel[0] = -abs(p.vel[0]) * ELASTICITY
        # Top wall
        if p.pos[1] - p.radius < 0:
            p.pos[1] = p.radius
            p.vel[1] = abs(p.vel[1]) * ELASTICITY
        # Bottom wall
        elif p.pos[1] + p.radius > height:
            p.pos[1] = height - p.radius
            p.vel[1] = -abs(p.vel[1]) * ELASTICITY


# --- Main Simulation Loop ---
def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("N-Body Gravity Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24) # Font for FPS display

    particles = initialize_particles(NUM_PARTICLES, WIDTH, HEIGHT)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # --- Physics Steps ---
        # 1. Update positions and velocities (using Verlet integration)
        update_particles(particles, DT)

        # 2. Handle collisions (particle-particle and particle-wall)
        handle_collisions(particles, WIDTH, HEIGHT)


        # --- Drawing ---
        screen.fill(BACKGROUND_COLOR)

        for p in particles:
            p.draw(screen)

        # Draw FPS
        fps = clock.get_fps()
        fps_text = font.render(f"FPS: {fps:.1f}", True, FPS_COLOR)
        screen.blit(fps_text, (10, 10))

        pygame.display.flip()
        clock.tick(60) # Limit frame rate

    pygame.quit()

if __name__ == "__main__":
    run_simulation()