import argparse
import time

import pynlab

import race_env
from map import Map

__author__ = 'apostol3'


def above_zero(string):
    value = int(string)
    if value <= 0:
        msg = "{} not above zero".format(value)
        raise argparse.ArgumentTypeError(msg)
    return value


parser = argparse.ArgumentParser(description="race environment for nlab")
parser.add_argument("file", metavar="file", type=str, help="map file (default: %(default)s)", nargs='?',
                    default="default.json")
parser.add_argument("-p", "--pipe", help="pipe name (default: %(default)s)",
                    metavar="name", type=str, dest="pipe_name", default="nlab")
parser.add_argument("-c", "--count", help="number of players (default: %(default)s)",
                    metavar="n", type=above_zero, dest="count", default=25)
parser.add_argument("--no-gui", help="do not show gui", action="store_false",
                    dest="gui")
args = parser.parse_args()

print("initializing... ", end="", flush=True)
pipe_str = "\\\\.\\pipe\\{}"
pipe_name = args.pipe_name

esi = pynlab.EStartInfo()
esi.count = args.count
esi.incount = 10
esi.outcount = 4
esi.mode = pynlab.SendModes.specified

last_time = time.perf_counter()
lab = pynlab.NLab(pipe_str.format(pipe_name))
map_ = Map.open_from_file(args.file)
game = race_env.Game(esi.count, map_, args.gui)
game.restart()
print("complete")

print("connecting to nlab at pipe \"{}\"... ".format(pipe_str.format(pipe_name)), end="", flush=True)
lab.connect()
print("connected")

print("waiting for start information from nlab... ", end="", flush=True)
lab.set_start_info(esi)
lab.get_start_info()
print("ok")

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
    game.restart()

    lab.get()
    if lab.is_ok == pynlab.VerificationHeader.stop:
        print("get stop header from nlab. stopping")
        exit()
