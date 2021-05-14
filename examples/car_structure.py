import json

def main():
    beamng_home = 'H:/BeamNG.research.v1.7.0.1clean/vehicles'
    jbeam_file = 'etk800/etk800.jbeam'
    filename = '{}/{}'.format(beamng_home, jbeam_file)
    with open(filename, "r") as f:
        lines = f.read().replace('\n', '')
        print(lines)
        loaded_json = json.loads(lines)
        print(loaded_json)

if __name__ == '__main__':
    main()