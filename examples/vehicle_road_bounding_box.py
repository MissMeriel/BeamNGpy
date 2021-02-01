import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.pyplot import imshow
from PIL import Image
from shapely.geometry import Polygon

from beamngpy import BeamNGpy, Vehicle, Scenario, Road
from beamngpy.sensors import Camera


beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.0')

scenario = Scenario('GridMap', 'vehicle_bbox_example')
road = Road('track_editor_C_center', rid='main_road', texture_length=5)
orig = (-107, 70, 0)
goal = (-300, 70, 0)
road.nodes = [
    (*orig, 7),
    (*goal, 7),
]
scenario.add_road(road)

vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
overhead = Camera((0, -10, 5), (0, 1, -0.75), 60, (1024, 1024))
vehicle.attach_sensor('overhead', overhead)
scenario.add_vehicle(vehicle, pos=orig)

scenario.make(beamng)

bng = beamng.open()
bng.load_scenario(scenario)
bng.start_scenario()

road_geometry = bng.get_road_edges('main_road')
left_edge_x = np.array([e['left'][0] for e in road_geometry])
left_edge_y = np.array([e['left'][1] for e in road_geometry])
right_edge_x = np.array([e['right'][0] for e in road_geometry])
right_edge_y = np.array([e['right'][1] for e in road_geometry])

def plot_road(ax):
    x_min = min(left_edge_x.min(), right_edge_x.min()) - 10  # We add/subtract 10 from the min/max coordinates to pad
    x_max = max(left_edge_x.max(), right_edge_x.max()) + 10  # the area of the plot a bit
    y_min = min(left_edge_y.min(), right_edge_y.min()) - 10
    y_max = max(left_edge_y.max(), right_edge_y.max()) + 10
    ax.set_aspect('equal', 'datalim')
    ax.set_xlim(left=x_max, right=x_min) # pyplot & bng coordinate systems have different origins
    ax.set_ylim(bottom=y_max, top=y_min) # so we flip them here
    ax.plot(left_edge_x, left_edge_y, 'b-')
    ax.plot(right_edge_x, right_edge_y, 'b-')

plt.figure(figsize=(10, 10))
plot_road(plt.gca())
plt.show()

script = [{'x': orig[0], 'y': orig[1], 'z': .5, 't': 0}]
i = 0.2

while script[-1]['x'] > goal[0]:
    node = {
        'x': -10 * i + orig[0],
        'y': 8 * np.sin(i) + orig[1],
        'z': 0.3,
        't': 1.5 * i,
    }
    script.append(node)
    i += 0.2

script_x = [s['x'] for s in script]
script_y = [s['y'] for s in script]

def plot_script(ax):
    ax.plot(script_x, script_y, 'y-')

plt.figure(figsize=(10, 10))
plot_road(plt.gca())
plot_script(plt.gca())
plt.show()


vehicle.ai_set_script(script)
bng.pause()
bng.step(1)

road_poly = list(zip(left_edge_x, left_edge_y))
road_poly.extend(zip(right_edge_x[::-1], right_edge_y[::-1]))
road_poly = Polygon(road_poly)

def inbounds(bbox_x, bbox_y):
    bbox_poly = zip(bbox_x, bbox_y)
    bbox_poly = Polygon(bbox_poly)
    inter = bbox_poly.intersection(road_poly)
    return inter.area / bbox_poly.area > 0.5

def plot_bbox(ax):
    bbox = vehicle.get_bbox()
    boundary_x = [
        bbox['front_bottom_left'][0],
        bbox['front_bottom_right'][0],
        bbox['rear_bottom_right'][0],
        bbox['rear_bottom_left'][0],
        bbox['front_bottom_left'][0],
    ]
    boundary_y = [
        bbox['front_bottom_left'][1],
        bbox['front_bottom_right'][1],
        bbox['rear_bottom_right'][1],
        bbox['rear_bottom_left'][1],
        bbox['front_bottom_left'][1],
    ]
    if inbounds(boundary_x, boundary_y):
        ax.plot(boundary_x, boundary_y, 'g-')
    else:
        ax.plot(boundary_x, boundary_y, 'r-')

plt.figure(figsize=(10, 10))
plot_road(plt.gca())
plot_script(plt.gca())
plot_bbox(plt.gca())
plt.show()

def plot_overhead(ax):
    view = bng.poll_sensors(vehicle)['overhead']['colour']
    view = view.convert('RGB') # Drop alpha channel as it messes up the plot
    ax.imshow(np.asarray(view))
    ax.set_aspect('equal', 'datalim')

plt.figure(figsize=(10, 10))
plot_overhead(plt.gca())
plt.show()
plt.clf()
plt.close()

fig, ax = plt.subplots(10, 2, figsize=(20, 100))
for row in range(10):
    bng.step(400)
    plot_road(ax[row, 0])
    plot_script(ax[row, 0])
    plot_bbox(ax[row, 0])
    plot_overhead(ax[row, 1])
plt.show()