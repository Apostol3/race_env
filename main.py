import argparse
import time

import pynlab

import race_env
from map import Map
from map_config import MapConfig

__author__ = 'apostol3'


def above_zero(string):
    value = int(string)
    if value <= 0:
        msg = "{} not above zero".format(value)
        raise argparse.ArgumentTypeError(msg)
    return value


parser = argparse.ArgumentParser(description="race environment for nlab")
parser.add_argument("file", metavar="file", type=str, help="map/config file (default: %(default)s)", nargs='?',
                    default="default.json")
parser.add_argument("-u", "--uri",
                    help="connection URI in format '[tcp|winpipe]://hostname(/pipe_name|:port)'(default: %(default)s",
                    metavar="uri", type=str, dest="connection_uri", default="tcp://127.0.0.1:5005")
parser.add_argument("-c", "--count", help="number of players (default: %(default)s)",
                    metavar="n", type=above_zero, dest="count", default=25)
parser.add_argument("--no-gui", help="do not show gui", action="store_false",
                    dest="gui")
parser.add_argument("--challenge", help="challenge mode with collisions", action="store_true",
                    dest="challenge")
args = parser.parse_args()

print("initializing... ", end="", flush=True)
map_config = MapConfig(args.file)

mode = 'race' if args.challenge else 'time'

# pickup any available map to find number of cars
map_filename, rounds_left = map_config.choose_map()
map_ = Map.open_from_file(map_filename)

esi = pynlab.EStartInfo()
esi.count = min(args.count, len(map_.cars)) if mode == 'race' else args.count
esi.incount = 17
esi.outcount = 4
esi.mode = pynlab.SendModes.specified

last_time = time.perf_counter()
lab = pynlab.NLab(args.connection_uri)
game = race_env.Game(esi.count, map_, args.gui, mode)

print("complete")

print("connecting to nlab at {}... ".format(args.connection_uri), end="", flush=True)
lab.connect()
print("connected")

print("waiting for start information from nlab... ", end="", flush=True)
lab.set_start_info(esi)
lab.get_start_info()
print("ok")

last_seed = lab.get_state.round_seed
map_filename, rounds_left = map_config.choose_map(last_seed)
map_ = Map.open_from_file(map_filename)
game.restart(map_)

print("working")
while lab.is_ok != pynlab.VerificationHeader.stop:
    while not all(game.go):
        esdi = pynlab.ESendInfo()
        esdi.head = pynlab.VerificationHeader.ok
        game.outputs = game.get()
        esdi.data = game.outputs
        lab.set(esdi)

        get = lab.get()
        if lab.is_ok == pynlab.VerificationHeader.stop:
            print("get stop header from nlab. stopping")
            exit()
        game.inputs = get.data
        game.set(game.inputs)

        game.tick()
        new_time = time.perf_counter()
        if new_time - last_time > 1 / 30:
            last_time = new_time
            game.dispatch_messages()
            game.draw()

    eri = pynlab.ERestartInfo()
    eri.result = []
    for i in range(esi.count):
        eri.result.append(game.get_fitness(i))
    lab.restart(eri)

    lab.get()
    if lab.is_ok == pynlab.VerificationHeader.stop:
        print("get stop header from nlab. stopping")
        exit()

    if last_seed != lab.get_state.round_seed:
        rounds_left -= 1
        last_seed = lab.get_state.round_seed
    if rounds_left == 0:
        map_filename_new, rounds_left = map_config.choose_map(last_seed)
        if map_filename != map_filename_new:
            map_filename = map_filename_new
            map_ = Map.open_from_file(map_filename)
    game.restart(map_)
