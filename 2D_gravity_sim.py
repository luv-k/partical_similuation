import pygame
import numpy as np

WIDTH, HEIGHT = 800, 600
NUM_PARTICLES = int(input("enter the number of particals (for performance <= 20):"))
G = 500.0  
DT = 0.05  
ELASTICITY = 0.9  
RECOIL_FACTOR = 1.2  


pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()


pos = np.random.uniform([50, 50], [WIDTH-50, HEIGHT-50], (NUM_PARTICLES, 2))
vel = np.random.uniform(-5, 5, (NUM_PARTICLES, 2))  
mass = np.random.uniform(5, 20, NUM_PARTICLES)
radius = np.random.uniform(5, 10, NUM_PARTICLES).astype(int)


def update():
    global pos, vel
    forces = np.zeros_like(pos)
    
    for i in range(NUM_PARTICLES):
        for j in range(NUM_PARTICLES):
            if i != j:
                direction = pos[j] - pos[i]
                dist = np.linalg.norm(direction) + 1e-4
                if dist > radius[i] + radius[j]:
                    forces[i] += (G * mass[i] * mass[j] / dist**2) * (direction / dist)
    
    vel += (forces.T / mass).T * DT
    pos += vel * DT


    for i in range(NUM_PARTICLES):
        for j in range(i + 1, NUM_PARTICLES):
            direction = pos[j] - pos[i]
            dist = np.linalg.norm(direction)
            if dist < radius[i] + radius[j]:
                normal = direction / (dist + 1e-6)
                relative_vel = vel[i] - vel[j]
                v_normal = np.dot(relative_vel, normal)
                
                if v_normal < 0:
                    impulse = -(1 + ELASTICITY) * v_normal / (1/mass[i] + 1/mass[j])
                    vel[i] += (impulse / mass[i]) * normal
                    vel[j] -= (impulse / mass[j]) * normal
                    
                    
                    pos[i] -= normal * RECOIL_FACTOR
                    pos[j] += normal * RECOIL_FACTOR

  
    for i in range(NUM_PARTICLES):
        if pos[i][0] - radius[i] < 0:
            vel[i][0] = abs(vel[i][0]) * ELASTICITY
            pos[i][0] = radius[i]
        elif pos[i][0] + radius[i] > WIDTH:
            vel[i][0] = -abs(vel[i][0]) * ELASTICITY
            pos[i][0] = WIDTH - radius[i]
        
        if pos[i][1] - radius[i] < 0:
            vel[i][1] = abs(vel[i][1]) * ELASTICITY
            pos[i][1] = radius[i]
        elif pos[i][1] + radius[i] > HEIGHT:
            vel[i][1] = -abs(vel[i][1]) * ELASTICITY
            pos[i][1] = HEIGHT - radius[i]


def run_simulation():
    running = True
    while running:
        screen.fill((0, 0, 0))
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        update()
        for i in range(NUM_PARTICLES):
            pygame.draw.circle(screen, (100, 200, 255), pos[i].astype(int), radius[i])
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    run_simulation()