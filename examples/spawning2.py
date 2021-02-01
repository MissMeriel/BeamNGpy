from beamngpy import BeamNGpy, Vehicle, Scenario, ScenarioObject
from beamngpy import Config
from beamngpy.sensors import Camera, GForces, Lidar, Electrics, Damage, Timer

import random

beamng = BeamNGpy('localhost', 64256, home='C:/Users/merie/Documents/BeamNG.research.v1.7.0.1')
bng = beamng.open()
beamng.deploy_mod()
#beamng.queue_lua_command('vehicle.save')
scenario = Scenario('smallgrid', 'respawn')
scenario._find_path(bng)

vehicle = Vehicle('ego_vehicle3', model='etk800', color='White')
cfg = Config()
#cfg.load('C:\\Users\\merie\\Documents\\BeamNG.research/vehicles/etk800/backcrash.pc')
#cfg.load('C:\\Users\\merie\\Documents\\BeamNG.research/vehicles/etk800/vehicle.save.json')

scenario.add_vehicle(vehicle, pos=(0,0,0), rot=None, rot_quat=(0,0,0,1))
scenario.make(beamng)

bng.load_scenario(scenario)
bng.start_scenario()

#bng.despawn_vehicle(vehicle)
#bng.spawn_vehicle(vehicle, pos=(0,-20, 0), rot=None, rot_quat=(0, 1, 0, 0))
#bng.despawn_vehicle(vehicle)
#vehicle.connect(beamng, bng.server, bng.port)
#cfg = vehicle.get_part_config()
vehicle.set_part_config(cfg)
#vehicle.update_vehicle()
#scenario.add_vehicle(vehicle, pos=(0,-20, 0), rot=None, rot_quat=(0, 1, 0, 0))
bng.spawn_vehicle(vehicle, pos=(0,-20, 0), rot=None, rot_quat=(0, 1, 0, 0))
for _ in range(30):
    throttle = random.uniform(0.0, 1.0)
    steering = random.uniform(-1.0, 1.0)
    brake = random.choice([0, 0, 0, 1])
    vehicle.control(throttle=throttle, steering=steering, brake=brake)
    pcfg = vehicle.get_part_config()

    # bng.step(20)
    bng.step(20)

    # Retrieve sensor data and show the camera data.
    sensors = bng.poll_sensors(vehicle)

    print("step in loop {}".format(_))