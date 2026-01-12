import pygame as pg
import math

# Initialize Pygame
pg.init()

# Screen dimensions and setup
WIDTH, HEIGHT = 800, 800
window = pg.display.set_mode((WIDTH, HEIGHT))
pg.display.set_caption("Gravitational Slingshot Effect")

# Constants
PLANET_MASS = 100
SHIP_MASS = 5
G_CONSTANT = 60
FRAME_RATE = 60
PLANET_SIZE = 50
OBJ_SIZE = 5
VELOCITY_SCALE = 100

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

class Planet:
    def __init__(self, x, y, mass):
        self.x = x
        self.y = y
        self.mass = mass

    def draw(self):
        pg.draw.circle(window, RED, (WIDTH // 2, HEIGHT // 2), PLANET_SIZE)

class Spacecraft:
    def __init__(self, x, y, vel_x, vel_y, mass):
        self.x = x
        self.y = y
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.mass = mass

    def movement(self, planet=None):
        distance = math.sqrt(((self.x - planet.x) ** 2) + ((self.y - planet.y) ** 2))
        force = (G_CONSTANT * self.mass * planet.mass) / distance**2
        acceleration = force / self.mass
        angle = math.atan2(planet.y - self.y, planet.x - self.x)

        acceleration_x = acceleration * math.cos(angle)
        acceleration_y = acceleration * math.sin(angle)

        self.vel_x += acceleration_x
        self.vel_y += acceleration_y

        self.x += self.vel_x
        self.y += self.vel_y

    def draw(self):
        pg.draw.circle(window, WHITE, (int(self.x), int(self.y)), OBJ_SIZE)

def create_spacecraft(location, mouse):
    t_x, t_y = location
    m_x, m_y = mouse
    vel_x = int((m_x - t_x) / VELOCITY_SCALE)
    vel_y = int((m_y - t_y) / VELOCITY_SCALE)
    return Spacecraft(t_x, t_y, vel_x, vel_y, SHIP_MASS)

def main():
    running = True
    clock = pg.time.Clock()

    objects = []
    temp_obj_pos = None
    central_planet = Planet(WIDTH // 2, HEIGHT // 2, PLANET_MASS)

    while running:
        clock.tick(FRAME_RATE)
        mouse_pos = pg.mouse.get_pos()

        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
            elif event.type == pg.MOUSEBUTTONDOWN:
                if temp_obj_pos:
                    obj = create_spacecraft(temp_obj_pos, mouse_pos)
                    objects.append(obj)
                    temp_obj_pos = None
                else:
                    temp_obj_pos = mouse_pos

        window.fill(BLACK)

        if temp_obj_pos:
            pg.draw.circle(window, WHITE, temp_obj_pos, OBJ_SIZE)
            pg.draw.line(window, WHITE, temp_obj_pos, mouse_pos, 2)

        for obj in objects[:]:
            obj.draw()
            obj.movement(central_planet)

            collision = math.sqrt((obj.x - central_planet.x) ** 2 + (obj.y - central_planet.y) ** 2) < (PLANET_SIZE - 2)
            if (obj.x < 0 or obj.x > WIDTH or obj.y < 0 or obj.y > HEIGHT) or collision:
                objects.remove(obj)

        central_planet.draw()
        pg.display.update()

    pg.quit()

if __name__ == '__main__':
    main()