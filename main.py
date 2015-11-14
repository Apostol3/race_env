__author__ = 'apostol3'
import time

import env
import nlab

import race_env

esi = env.EStartInfo()
esi.count = 25
esi.incount = 10
esi.outcount = 4
esi.mode = env.SendModes.specified

last_time = time.perf_counter()
lab = nlab.NLab()
game = race_env.Game(esi.count)
game.restart()

lab.connect()

print(lab.set_start_info(esi))
print(lab.get_start_info().count)

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
        game.inputs = [[max(0, min(1, i)) for i in j] for j in get.data]
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
        eri.result.append(game.get_min_dist(i) * 1000)
    print(lab.restart(eri))
    game.restart()

    print(lab.get())
    if lab.is_ok == env.VerificationHeader.stop:
        exit()
