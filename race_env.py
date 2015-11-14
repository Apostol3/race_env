__author__ = 'apostol3'

import math
import sys
import time

import Box2D as b2
import pygame
from pygame.locals import *


class PygameDraw(b2.b2DrawExtended):
    """
    This debug draw class accepts callbacks from Box2D (which specifies what to draw)
    and handles all of the rendering.

    If you are writing your own game, you likely will not want to use debug drawing.
    Debug drawing, as its name implies, is for debugging.
    """
    surface = None
    axisScale = 10.0

    def __init__(self, width, height, **kwargs):
        b2.b2DrawExtended.__init__(self, **kwargs)
        self.flipX = False
        self.flipY = True
        self.convertVertices = True
        self.zoom = 1
        self.center = (width / 2, height / 2)
        self.offset = (0, 0)
        self.screenSize = (width, height)
        self.axisScale = 5

    def DrawPoint(self, p, size, color):
        """
        Draw a single point at point p given a pixel size and color.
        """
        self.DrawCircle(p, size / self.zoom, color, drawwidth=0)

    def DrawAABB(self, aabb, color):
        """
        Draw a wireframe around the AABB with the given color.
        """
        points = [(aabb.lowerBound.x, aabb.lowerBound.y),
                  (aabb.upperBound.x, aabb.lowerBound.y),
                  (aabb.upperBound.x, aabb.upperBound.y),
                  (aabb.lowerBound.x, aabb.upperBound.y)]

        pygame.draw.aalines(self.surface, color, True, points)

    def DrawSegment(self, p1, p2, color):
        """
        Draw the line segment from p1-p2 with the specified color.
        """
        pygame.draw.aaline(self.surface, color.bytes, p1, p2)

    def DrawTransform(self, xf):
        """
        Draw the transform xf on the screen
        """
        p1 = xf.position
        p2 = self.to_screen(p1 + self.axisScale * xf.R.col1)
        p3 = self.to_screen(p1 + self.axisScale * xf.R.col2)
        p1 = self.to_screen(p1)

        pygame.draw.aaline(self.surface, (255, 0, 0), p1, p2)
        pygame.draw.aaline(self.surface, (0, 255, 0), p1, p3)

    def DrawCircle(self, center, radius, color, drawwidth=1):
        """
        Draw a wireframe circle given the center, radius, axis of orientation and color.
        """
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        pygame.draw.circle(self.surface, color.bytes, center, radius, drawwidth)

    def DrawSolidCircle(self, center, radius, axis, color):
        """
        Draw a solid circle given the center, radius, axis of orientation and color.
        """
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        pygame.draw.circle(self.surface, (color / 2).bytes + [127], center, radius, 0)
        pygame.draw.circle(self.surface, color.bytes, center, radius, 1)
        pygame.draw.aaline(self.surface, (255, 0, 0), center,
                           (center[0] - radius * axis[0], center[1] + radius * axis[1]))

    def DrawPolygon(self, vertices, color):
        """
        Draw a wireframe polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices)
        else:
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)

    def DrawSolidPolygon(self, vertices, color):
        """
        Draw a filled polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices[1])
        else:
            pygame.draw.polygon(self.surface, (color / 2).bytes + [127], vertices, 0)
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)


class RayCastClosestCallback(b2.b2RayCastCallback):
    """This callback finds the closest hit"""

    def __init__(self, **kwargs):
        b2.b2RayCastCallback.__init__(self, **kwargs)
        self.fixture = []
        self.hit = False
        self.fraction = 1
        self.point = b2.b2Vec2()
        self.normal = b2.b2Vec2()

    def ReportFixture(self, fixture, point, normal, fraction):
        if fixture.filterData.categoryBits != 1:
            return -1
        self.hit = True
        self.fixture = fixture
        self.point = b2.b2Vec2(point)
        self.normal = b2.b2Vec2(normal)
        self.fraction = fraction

        return fraction


class Game:
    def __init__(self, count):
        pygame.init()
        self.size = self.width, self.height = 640, 480
        self.screen = pygame.display.set_mode(self.size)
        self.font = pygame.font.SysFont('Tahoma', 12, False, False)

        self.inputs = [[0] * 4 for i in range(count)]
        self.outputs = [[0] * 10 for i in range(count)]

        self.count = count
        self.cars = [[] for i in range(count)]
        self.time = [0] * count
        self.go = [False] * count

        self.sensors = []
        self.main_line = []

        self.world = []

    def create_car(self, x, y):
        fix_def = b2.b2FixtureDef()
        fix_def.friction = 0.3
        fix_def.restitution = 0.1
        fix_def.filter.categoryBits = 2
        fix_def.filter.maskBits = 1

        fix_def.shape = b2.b2PolygonShape(box=(5, 10))

        body_def = b2.b2BodyDef()
        body_def.type = b2.b2_dynamicBody
        body_def.position = (x, y)
        body_def.linearDamping = 0.2
        body_def.angularDamping = 15

        body = self.world.CreateBody(body_def)
        body.CreateFixture(fix_def)
        body.inertia = 3
        body.mass = 3
        body.sleepingAllowed = False
        body.angle = 3 * math.pi / 2

        return body

    def vf(self, vec):
        return vec.x, self.height - vec.y

    def draw_lamp(self, position, text, color, val):
        _text = self.font.render(text, True, (255, 255, 255))
        self.screen.blit(_text, (position[0] + 10, position[1]))

        pygame.draw.circle(self.screen, (128, 128, 128), (position[0], position[1] + 7), 5, 1)

        if val:
            pygame.draw.circle(self.screen, color, (position[0], position[1] + 7), 4, 0)

    def draw_bar(self, rect, color, val):
        pygame.draw.rect(self.screen, (128, 128, 128), ((rect[0][0], rect[0][1]), (rect[1][0], rect[1][1])), 1)
        if val:
            pygame.draw.rect(self.screen, color,
                             ((rect[0][0] + 1, rect[0][1] + rect[1][1] - 2), (rect[1][0] - 2, -val * (rect[1][1] - 4))))

    def draw_interface(self):
        # for j in range(self.count):
        #    if self.go[j]:
        #        continue
        #    for i in range(len(self.sensors)):
        #        pygame.draw.line(self.screen, (255, 255, 255), self.vf(self.cars[j].position),
        #                         #self.vf(self.cars[j].position + self.cars[j].transform.R * self.sensors[i] * (
        #                             1 - self.outputs[j][i + 3])))

        if not self.go[0]:
            self.draw_bar(((50, 10), (10, 50)), (235, 40, 40), self.inputs[0][0])
            self.draw_bar(((65, 10), (10, 50)), (40, 40, 235), self.inputs[0][1])
            self.draw_bar(((80, 10), (10, 50)), (40, 235, 40), self.inputs[0][2])
            self.draw_bar(((95, 10), (10, 50)), (40, 235, 40), self.inputs[0][3])

            self.draw_bar(((350, 10), (10, 50)), (220, 40, 220), self.outputs[0][0])
            self.draw_bar(((365, 10), (10, 50)), (220, 220, 40), self.outputs[0][1])
            self.draw_bar(((380, 10), (10, 50)), (220, 220, 40), self.outputs[0][2])

            for i in range(3, len(self.outputs[0])):
                self.draw_bar(((350 + i * 15, 10), (10, 50)), (220, 220, 220), self.outputs[0][i])

        self.screen.blit(self.font.render("Time: {:.2f}".format(self.time[0]), True, (255, 255, 255)), (250, 10))

        self.draw_bar(((125, 10), (10, 50)), (40, 235, 40), self.get_min_dist(0))

        self.draw_lamp((250, 30), 'Finished', (200, 200, 40), self.go[0])
        self.draw_lamp((250, 50), 'Touching', (200, 40, 40), any(i.contact.touching for i in self.cars[0].contacts))

    def restart(self):
        self.time = [0] * self.count
        self.go = [False] * self.count
        self.inputs = [[0] * 4 for i in range(self.count)]
        self.outputs = [[0] * 10 for i in range(self.count)]
        self.world = b2.b2World((0, 0), True)

        self.world.renderer = PygameDraw(self.width, self.height, surface=self.screen)
        self.world.renderer.flags = dict(
            drawShapes=True,
            drawJoints=True,
            drawAABBs=False,
            drawPairs=False,
            drawCOMs=True,
            convertVertices=isinstance(self.world.renderer, b2.b2DrawExtended)
        )

        self.w = 60
        self.r = self.width - 10
        self.l = 0 + 10
        self.t = self.height - 10 - 60
        self.b = 0 + 10
        w = self.w
        l = self.l
        r = self.r
        t = self.t
        b = self.b

        self.world.CreateStaticBody(shapes=[b2.b2EdgeShape(vertices=[(l, b), (r, b)]),
                                            b2.b2EdgeShape(vertices=[(r, b), (r, t)]),
                                            b2.b2EdgeShape(vertices=[(r, t), (l, t)]),
                                            b2.b2EdgeShape(vertices=[(l, t), (l, b)])])

        self.world.CreateStaticBody(shapes=[b2.b2EdgeShape(vertices=[(l + w, t - w), (r - w, t - w)]),
                                            b2.b2EdgeShape(vertices=[(r - w, t - w), (r - w, b + w)]),
                                            b2.b2EdgeShape(vertices=[(r - w, b + w), (l + w, b + w)]),
                                            b2.b2EdgeShape(vertices=[(l + w, b + w), (l + w, t - 3 * w)]),
                                            b2.b2EdgeShape(vertices=[(r - 3 * w, b + w), (r - 3 * w, t - 3 * w)]),
                                            b2.b2EdgeShape(vertices=[(r - 4 * w, b + w), (r - 4 * w, b)]), ])

        self.world.CreateStaticBody(shapes=[b2.b2EdgeShape(vertices=[(l, t - 2 * w), (r - 2 * w, t - 2 * w)]),
                                            b2.b2EdgeShape(vertices=[(r - 2 * w, t - 2 * w), (r - 2 * w, b + 2 * w)]),
                                            b2.b2EdgeShape(vertices=[(l + 2 * w, t - 2 * w),
                                                                     ((r - 4 * w + l + 2 * w) / 2, b + 2 * w)]),
                                            b2.b2EdgeShape(
                                                vertices=[(l + w, t - 3 * w), ((r - 4 * w + l + 2 * w) / 2, b + w)]),
                                            b2.b2EdgeShape(vertices=[((r - 4 * w + l + 2 * w) / 2, b + w),
                                                                     (r - 3 * w, t - 3 * w)]),
                                            b2.b2EdgeShape(vertices=[((r - 4 * w + l + 2 * w) / 2, b + 2 * w),
                                                                     (r - 4 * w, t - 2 * w)])])

        self.main_line = self.world.CreateStaticBody(
            shapes=[b2.b2EdgeShape(vertices=[(r - 7 * w / 2, b + w / 2), (r - w / 2, b + w / 2)]),
                    b2.b2EdgeShape(vertices=[(r - w / 2, b + w / 2), (r - w / 2, t - w / 2)]),
                    b2.b2EdgeShape(vertices=[(r - w / 2, t - w / 2), (l + w / 2, t - w / 2)]),
                    b2.b2EdgeShape(vertices=[(l + w / 2, t - w / 2), (l + w / 2, t - 3 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(l + w / 2, t - 3 * w / 2), (r - 3 * w / 2, t - 3 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(r - 3 * w / 2, t - 3 * w / 2), (r - 3 * w / 2, b + 3 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(r - 3 * w / 2, b + 3 * w / 2), (r - 5 * w / 2, b + 3 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(r - 5 * w / 2, b + 3 * w / 2), (r - 5 * w / 2, t - 5 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(r - 5 * w / 2, t - 5 * w / 2), (r - 7 * w / 2, t - 5 * w / 2)]),
                    b2.b2EdgeShape(
                        vertices=[(r - 7 * w / 2, t - 5 * w / 2), ((r - 5 * w + l + 3 * w) / 2, b + 3 * w / 2)]),
                    b2.b2EdgeShape(
                        vertices=[((r - 5 * w + l + 3 * w) / 2, b + 3 * w / 2), (l + 3 * w / 2, t - 5 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(l + 3 * w / 2, t - 5 * w / 2), (l + w / 2, t - 5 * w / 2)]),
                    b2.b2EdgeShape(vertices=[(l + w / 2, t - 5 * w / 2), (l + w / 2, b + w / 2)]),
                    b2.b2EdgeShape(vertices=[(l + w / 2, b + w / 2), (r - 9 * w / 2, b + w / 2)])])

        for i in self.main_line.fixtures:
            i.sensor = True
            i.filterData.categoryBits = 0

        self.cars = [self.create_car(r - 3 * w, l + w / 2) for i in range(self.count)]

        n = 3
        rad = 150
        angle = math.pi / 4
        self.sensors = [(math.sin(i * angle / n) * rad, math.cos(i * angle / n) * rad) for i in range(-n, n + 1)]

    def dispatch_messages(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_RETURN:
                    self.restart()
                elif event.key == K_ESCAPE:
                    sys.exit()

    def get_inputs(self):
        keys = pygame.key.get_pressed()
        inputs = [[0] * 4 for i in range(self.count)]
        inputs[0][0] = keys[K_UP]
        inputs[0][1] = keys[K_DOWN] | keys[K_LSHIFT]
        inputs[0][2] = keys[K_LEFT]
        inputs[0][3] = keys[K_RIGHT]

        return inputs

    def set(self, inputs):
        for i in range(self.count):
            if self.go[i]:
                continue
            car = self.cars[i]
            car.linearDamping = 0.2 + 3 * inputs[i][1]
            if car.linearVelocity.length != 0:
                car.linearVelocity -= car.transform.R.col1 * car.linearVelocity.length * (
                    car.linearVelocity.dot(car.transform.R.col1) / car.linearVelocity.length) * 0.1

            if car.linearVelocity.length > 9:
                _power = 150 * (inputs[i][0] * 0.5 + 0.5)
            else:
                _power = 150 * (inputs[i][0] * 0.5 + 0.5 - inputs[i][1])

            car.ApplyForce(
                car.transform.R * ((b2.b2Transform((0, 0), b2.b2Rot((inputs[i][2] - inputs[i][3]) / 4))) * (0, _power)),
                (car.position + (car.transform.R * (0, 5))),
                True)
            self.go[i] |= self.is_finish(i)
            if self.go[i]:
                car.linearVelocity = (0, 0)

    def get(self):
        outputs = [[] for i in range(self.count)]
        for i in range(self.count):
            if self.go[i]:
                continue
            car = self.cars[i]
            outputs[i] = [car.linearVelocity.length / 120, car.angularVelocity / 5, -car.angularVelocity / 5]

            for j in self.sensors:
                callback = RayCastClosestCallback()
                self.world.RayCast(callback, car.position, car.position + car.transform.R * j)
                outputs[i].append(1 - callback.fraction)

            outputs[i] = [min(max(j, 0), 1) for j in outputs[i]]
        return outputs

    def is_finish(self, j):
        return (self.r - 5 * self.w < self.cars[j].position.x < self.r - 4 * self.w and self.b < self.cars[
            j].position.y < self.b + self.w) or any(i.contact.touching for i in self.cars[j].contacts) or self.time[
                                                                                                              j] > 120

    def get_min_dist(self, j):
        min_d = -1
        min_point = (0, 0)
        min_fix = ()
        fixtures_cache = self.main_line.fixtures
        car_shape = self.cars[j].fixtures[0].shape
        fix_transform = self.main_line.transform
        car_transform = self.cars[j].transform
        for i in range(len(fixtures_cache)):
            _, pointB, dist, _ = b2.b2Distance(shapeA=car_shape, shapeB=fixtures_cache[i].shape,
                                               transformA=car_transform, transformB=fix_transform)
            if min_d > dist or min_d < 0:
                min_d = dist
                min_point = pointB
                min_fix = i

        dist_start = 0
        for i in range(min_fix):
            sh = fixtures_cache[i].shape
            dist_start += (b2.b2Vec2(sh.vertices[1]) - b2.b2Vec2(sh.vertices[0])).length

        dist_in = (b2.b2Vec2(fixtures_cache[min_fix].shape.vertices[0]) - b2.b2Vec2(min_point)).length

        result_dist = (dist_start + dist_in + min_d) / 3200
        return result_dist

    def draw(self):
        self.screen.fill((0, 0, 0))
        self.world.DrawDebugData()
        self.draw_interface()
        pygame.display.flip()

    def tick(self):
        self.time = list(map((lambda x, y: x + 1 / 60), self.time, self.go))
        self.world.Step(1 / 60, 10, 10)
        self.world.ClearForces()


def main():
    game = Game(2)
    game.restart()
    # This is our little game loop.
    old_time = time.perf_counter()
    old_old_time = time.perf_counter()

    while old_time - old_old_time < 60000:

        if time.perf_counter() - old_time > 1 / 60:
            old_time = time.perf_counter()
            if not all(game.go):
                game.time = list(map((lambda x, y: x + 1 / 60), game.time, game.go))
                game.world.Step(1 / 60, 10, 10)
                game.outputs = game.get()
                game.world.ClearForces()

                old_inputs = game.inputs[0]
                game.inputs = game.get_inputs()
                game.inputs[0] = list(map((lambda x, y: min(max(x + (y - 0.5) / 10, 0), 1)), old_inputs,
                                          game.inputs[0]))
                game.set(game.inputs)

            game.dispatch_messages()
            game.screen.fill((0, 0, 0))
            game.world.DrawDebugData()
            game.draw_interface()
            pygame.display.flip()


if __name__ == "__main__":
    main()
