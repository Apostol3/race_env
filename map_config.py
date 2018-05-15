import json
import random


class MapConfig:
    def __init__(self, filename):
        self.file = filename
        self.maps = []
        self.is_mapfile = None
        self.weights_sum = None
        self.reload()

    def reload(self):
        f = None
        try:
            maps = []
            weights_sum = 0
            f = open(self.file, 'r')
            doc = json.load(f)
            if type(doc) == dict:
                is_mapfile = True
            else:
                is_mapfile = False
                for i in doc:
                    maps.append((i['filename'], i['rounds'], i['weight']))
                    weights_sum += i['weight']
        except (KeyError, OSError) as ex:
            print('Warning: Error while reading config file; {}: {}'.format(type(ex).__name__, ex))
        else:
            self.maps = maps
            self.weights_sum = weights_sum
            self.is_mapfile = is_mapfile
        finally:
            f.close()

    def choose_map(self, seed=None):
        if self.is_mapfile:
            return self.file, 1

        counter = 0
        self.reload()
        if seed:
            chance = seed % self.weights_sum
        else:
            chance = random.randint(0, self.weights_sum - 1)
        for i in self.maps:
            counter += i[2]
            if chance < counter:
                return i[0], i[1]
