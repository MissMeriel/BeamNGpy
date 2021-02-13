from shapely.geometry import MultiLineString
from matplotlib import pyplot as plt

from beamngpy import BeamNGpy, Vehicle, Scenario


beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')

scenario = Scenario('west_coast_usa', 'road_map_example')
orig = (568.908386, 13.4217358,  148.56546)

vehicle = Vehicle('ego_vehicle', model='pickup', licence='PYTHON')
scenario.add_vehicle(vehicle, pos=orig)

scenario.make(beamng)

bng = beamng.open()
bng.load_scenario(scenario)
bng.start_scenario()

# fetch road data from the game
roads = bng.get_roads()
road_names = list(roads.keys())
road_spec = {}
for r_id, r_inf in roads.items():
 if r_inf['drivability'] != '-1':
  road_spec[r_id] = bng.get_road_edges(r_id)


# put data into shapely MultilineString
road = list()
lines = list()
for r_id in road_spec.keys():
    left = list()
    right = list()
    for r_point in road_spec[r_id]:
        x = r_point['left'][0]
        y = r_point['left'][1]
        left.append((x, y))
        x = r_point['right'][0]
        y = r_point['right'][1]
        right.append((x, y))
    if left:
        lines.append(tuple(left))
    if right:
        lines.append(tuple(right))

network = MultiLineString(lines)


# plot map
def plot_lines(ax, ob):
 blue = '#6699cc'
 for line in ob:
  x, y = line.xy
  ax.plot(x, y, color=blue, linewidth=1, solid_capstyle='round', zorder=2, alpha=0.7)


fig = plt.figure(1, figsize=[9.6, 9.6], dpi=100)

ax = fig.add_subplot()
plot_lines(ax, network)

_ = ax.set_axis_off()
_ = ax.set_title('road network West Coast, USA')
plt.savefig("leftright.png")