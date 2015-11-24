import argparse
import time

import env
import nlab

import race_env

__author__ = 'apostol3'


def above_zero(string):
    value = int(string)
    if value <= 0:
        msg = "{} not above zero".format(value)
        raise argparse.ArgumentTypeError(msg)
    return value


parser = argparse.ArgumentParser(description="race enviroment for nlab")
parser.add_argument("-p", "--pipe", help="pipe name (default: %(default)s)",
                    metavar="name", type=str, dest="pipe_name", default="nlab")
parser.add_argument("-c", "--count", help="number of players (default: %(default)s)",
                    metavar="n", type=above_zero, dest="count", default=25)
args = parser.parse_args()

pipe_str = "\\\\.\\pipe\\{}"
pipe_name = args.pipe_name

esi = env.EStartInfo()
esi.count = args.count
esi.incount = 10
esi.outcount = 4
esi.mode = env.SendModes.specified

last_time = time.perf_counter()
lab = nlab.NLab(pipe_str.format(pipe_name))
game = race_env.Game(esi.count)
game.restart()

lab.connect()

lab.set_start_info(esi)
lab.get_start_info()

while lab.is_ok != env.VerificationHeader.stop:
    while not all(game.go):
        esdi = env.ESendInfo()
        esdi.head = env.VerificationHeader.ok
        game.outputs = game.get()
        esdi.data = game.outputs
        lab.set(esdi)

        get = lab.get()
        if lab.is_ok == env.VerificationHeader.stop:
            exit()
        game.inputs = get.data
        game.set(game.inputs)

        game.tick()
        new_time = time.perf_counter()
        if new_time - last_time > 1 / 30:
            last_time = new_time
            game.dispatch_messages()
            game.draw()

    eri = env.ERestartInfo()
    eri.result = []
    for i in range(esi.count):
        dist = game.get_min_dist(i)
        eri.result.append(dist * 1000 + game.is_really_finish(i) * (12000 - game.time[i] * 100))
    lab.restart(eri)
    game.restart()

    lab.get()
    if lab.is_ok == env.VerificationHeader.stop:
        exit()
