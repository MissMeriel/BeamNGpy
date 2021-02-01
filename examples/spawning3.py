from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject
from beamngpy import Config
import random


beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')

scenario = Scenario('smallgrid', 'spawn_objects_example')
#scenario = Scenario('west_coast_usa', 'research_test', description='Random driving for research')

vehicle = Vehicle('ego_vehicle', model='pickup', licence='PYTHON')

scenario.add_vehicle(vehicle, pos=(0,0,0), rot=None, rot_quat=(0,0,0,1))

scenario.make(beamng)

bng = beamng.open()
bng.load_scenario(scenario)
bng.start_scenario()

vehicle.save()

#bng.spawn_vehicle(vehicle, pos=(0,-20, 0), rot=None, rot_quat=(0, 1, 0, 0))

for _ in range(30):
    throttle = random.uniform(0.0, 1.0)
    steering = random.uniform(-1.0, 1.0)
    brake = random.choice([0, 0, 0, 1])
    vehicle.control(throttle=throttle, steering=steering, brake=brake)

    # bng.step(20)
    bng.step(20)

vehicle.load()
