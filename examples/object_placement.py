
import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.pyplot import imshow

from beamngpy import BeamNGpy, Vehicle, Scenario, ProceduralRing, StaticObject
from beamngpy.sensors import Camera


beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.0')

scenario = Scenario('west_coast_usa', 'object_placement')

vehicle = Vehicle('ego', model='etk800', licence='PYTHON', color='Green')
scenario.add_vehicle(vehicle, pos=(-198.5, -164.189, 119.7), rot=(0, 0, -126.25))

ramp = StaticObject(name='pyramp', pos=(277.5, 183.5, 118.75), rot=(0, 0, 55), scale=(1, 1, 1), shape='/levels/west_coast_usa/art/shapes/objects/ramp_massive.dae')
scenario.add_object(ramp)

ring = ProceduralRing(name='pyring', pos=(445, 301, 218), rot=(0, 0, 100), radius=5, thickness=2.5)
scenario.add_procedural_mesh(ring)

cam_pos = (391.5, 251, 197.8)
cam_dir = (445 - cam_pos[0], 301 - cam_pos[1], 208 - cam_pos[2])
cam = Camera(cam_pos, cam_dir, 60, (2048, 2048), near_far=(1, 4000), colour=True)
scenario.add_camera(cam, 'cam')

scenario.make(beamng)


bng = beamng.open()
bng.set_deterministic()
bng.load_scenario(scenario)
bng.start_scenario()

meshes = scenario.find_procedural_meshes()
ring_pos = None
for mesh in meshes:
    if mesh.name == 'pyring':
        ring_pos = np.array(mesh.position)

for i in range(5)[1:]:
    vehicle.control(throttle=i / 4)
    bng.step(150)

scenario.update()
distance = np.linalg.norm(np.array(vehicle.state['pos']) - ring_pos)
while distance > 5:
    scenario.update()
    distance = np.linalg.norm(np.array(vehicle.state['pos']) - ring_pos)

frames = scenario.render_cameras()
bng.close()

plt.figure(figsize=(20, 20))
imshow(np.asarray(frames['cam']['colour'].convert('RGB')))