from panda3d.core import Point3, Vec3, WindowProperties
from direct.showbase.ShowBase import ShowBase
import random
import os
import math

# Constants
NUM_PARTICLES = 2  # Number of particles
G = 1000.0  # Gravitational constant
DT = 0.01  # Time step
BOUNDARY = 50  # Cube boundary size
MODEL_DIR = os.path.join(os.getcwd(), "models")  # Models directory
MODEL_PATH = os.path.join(MODEL_DIR, "scene.gltf")  # Absolute path to GLTF model

class Particle:
    def __init__(self, world, pos, mass, radius):
        self.mass = mass
        self.radius = radius

        # Check if model file exists
        try:
            if os.path.exists(MODEL_PATH):
                self.node = world.loader.loadModel(MODEL_PATH)
            else:
                # try a built-in/sample model
                try:
                    self.node = world.loader.loadModel("models/misc/sphere")
                except Exception:
                    # Last-resort: create an empty node so logic can continue
                    print(f"Warning: {MODEL_PATH} and fallback models not found. Using placeholder node.")
                    self.node = world.render.attachNewNode("particle")
        except Exception as e:
            print(f"Model load error: {e}. Using placeholder node.")
            self.node = world.render.attachNewNode("particle")

        self.node.setScale(radius)
        self.node.reparentTo(world.render)
        self.node.setPos(pos)
        self.velocity = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

    def update_position(self):
        new_pos = self.node.getPos() + self.velocity * DT
        # Bounce and clamp to boundary so particles don't escape too far
        if abs(new_pos.x) > BOUNDARY:
            new_pos.x = math.copysign(BOUNDARY, new_pos.x)
            self.velocity.x *= -1
        if abs(new_pos.y) > BOUNDARY:
            new_pos.y = math.copysign(BOUNDARY, new_pos.y)
            self.velocity.y *= -1
        if abs(new_pos.z) > BOUNDARY:
            new_pos.z = math.copysign(BOUNDARY, new_pos.z)
            self.velocity.z *= -1
        self.node.setPos(new_pos)

class GravitySimulation(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # Set fullscreen with fixed resolution
        props = WindowProperties()
        props.setSize(1920, 1080)
        props.setFullscreen(True)
        self.win.requestProperties(props)

        # position camera to see the whole simulation
        try:
            self.camera.setPos(0, -BOUNDARY * 3, 0)
            self.camera.lookAt(0, 0, 0)
        except Exception:
            pass

        # Initialize particles
        self.particles = [
            Particle(
                self,
                Point3(
                    random.uniform(-BOUNDARY, BOUNDARY),
                    random.uniform(-BOUNDARY, BOUNDARY),
                    random.uniform(-BOUNDARY, BOUNDARY),
                ),
                random.uniform(1, 5),
                random.uniform(0.5, 1.5),
            )
            for _ in range(NUM_PARTICLES)
        ]
        self.taskMgr.add(self.update_simulation, "update_simulation")

        # Close program when ESC is pressed
        self.accept("escape", self.exit_program)

        # Add zoom functionality
        self.accept("q", self.zoom_out)  # Zoom out
        self.accept("e", self.zoom_in)   # Zoom in

    def apply_gravity(self):
        for i, p1 in enumerate(self.particles):
            for j, p2 in enumerate(self.particles):
                if i != j:
                    r = p2.node.getPos() - p1.node.getPos()
                    dist = max(r.length(), 1e-4)
                    dist_sq = dist * dist
                    force = G * (p1.mass * p2.mass) / dist_sq
                    # compute direction safely
                    direction = r / dist
                    acceleration = direction * (force / p1.mass)
                    p1.velocity += acceleration * DT

    def update_simulation(self, task):
        self.apply_gravity()
        for p in self.particles:
            p.update_position()
        return task.cont

    def zoom_out(self):
        """Zoom out by moving the camera back."""
        current_pos = self.camera.getPos()
        self.camera.setPos(current_pos + Vec3(0, -10, 0))  # Move the camera backward

    def zoom_in(self):
        """Zoom in by moving the camera forward."""
        current_pos = self.camera.getPos()
        self.camera.setPos(current_pos + Vec3(0, 10, 0))  # Move the camera forward

    def exit_program(self):
        print("Exiting simulation...")
        self.userExit()

if __name__ == "__main__":
    app = GravitySimulation()
    app.run()