import torch
import vispy.app
import vispy.scene
import numpy as np
import math

N = int(input("Particles: "))

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

G = 6.674e-3
DT = 0.01
SOFTENING = 0.05
C = 500.0
MERGE_DIST_FACTOR = 1.0

pos = torch.randn(N,3,device=device)*5
vel = torch.zeros_like(pos)
mass = torch.rand(N,device=device)*50 + 10
radius = mass.clamp(min=1e-6).pow(1/3)*0.4

center_mass = mass.sum()
com = (pos*mass.unsqueeze(1)).sum(0)/center_mass

for i in range(N):
    r = pos[i]-com
    dist = torch.norm(r)+1e-6
    speed = math.sqrt(G*center_mass/dist.item())
    tangent = torch.tensor([-r[1], r[0], 0],device=device)
    tangent = tangent/torch.norm(tangent)
    vel[i] = tangent*speed*0.7

canvas = vispy.scene.SceneCanvas(keys='interactive',show=True,bgcolor='black')
view = canvas.central_widget.add_view()
view.camera = 'turntable'
view.camera.distance = 30

scatter = vispy.scene.visuals.Markers()
view.add(scatter)

def gravity():

    global pos,vel

    dx = pos[:,0].unsqueeze(1)-pos[:,0].unsqueeze(0)
    dy = pos[:,1].unsqueeze(1)-pos[:,1].unsqueeze(0)
    dz = pos[:,2].unsqueeze(1)-pos[:,2].unsqueeze(0)

    dist2 = dx**2+dy**2+dz**2+SOFTENING
    inv = torch.rsqrt(dist2**3)

    force = G*(mass.unsqueeze(1)*mass.unsqueeze(0))*inv
    force.fill_diagonal_(0)

    ax = -(force*dx).sum(1)/mass
    ay = -(force*dy).sum(1)/mass
    az = -(force*dz).sum(1)/mass

    acc = torch.stack((ax,ay,az),1)

    speed = torch.norm(vel,dim=1)
    gamma = 1/torch.sqrt(1-(speed/C)**2 + 1e-6)
    acc /= gamma.unsqueeze(1)

    vel += acc*DT
    pos += vel*DT

def merge_particles():

    global pos,vel,mass,radius

    n = len(pos)
    keep = torch.ones(n,dtype=torch.bool,device=device)

    for i in range(n):
        if not keep[i]: continue
        for j in range(i+1,n):
            if not keep[j]: continue

            d = torch.norm(pos[i]-pos[j])

            if d < (radius[i]+radius[j])*MERGE_DIST_FACTOR:

                m = mass[i]+mass[j]
                v = (vel[i]*mass[i]+vel[j]*mass[j])/m
                p = (pos[i]*mass[i]+pos[j]*mass[j])/m

                pos[i]=p
                vel[i]=v
                mass[i]=m
                radius[i]=m.clamp(min=1e-6).pow(1/3)*0.4

                keep[j]=False

    pos = pos[keep]
    vel = vel[keep]
    mass = mass[keep]
    radius = radius[keep]

def energy_momentum():

    ke = 0.5*mass*(torch.norm(vel,dim=1)**2)
    KE = ke.sum().item()

    dx = pos[:,0].unsqueeze(1)-pos[:,0].unsqueeze(0)
    dy = pos[:,1].unsqueeze(1)-pos[:,1].unsqueeze(0)
    dz = pos[:,2].unsqueeze(1)-pos[:,2].unsqueeze(0)

    dist = torch.sqrt(dx**2+dy**2+dz**2+SOFTENING)

    pe = -G*(mass.unsqueeze(1)*mass.unsqueeze(0))/dist
    PE = torch.triu(pe,1).sum().item()

    momentum = (vel*mass.unsqueeze(1)).sum(0)
    P = torch.norm(momentum).item()

    return KE,PE,P

def update(event):

    gravity()
    merge_particles()

    KE,PE,P = energy_momentum()

    pos_cpu = pos.detach().cpu().numpy()

    scatter.set_data(
        pos_cpu,
        face_color=(0.7,0.85,1,1),
        size=radius.detach().cpu().numpy()*6
    )

    canvas.title = f"N-Body GPU | KE {KE:.2e}  PE {PE:.2e}  |P| {P:.2e}  N {len(pos)}"

timer = vispy.app.Timer()
timer.connect(update)
timer.start(0)

vispy.app.run()
