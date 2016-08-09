import argparse
import math
import sys
import time

import Box2D as b2
import pygame

from map import Map

__author__ = 'apostol3'


class PygameDraw(b2.b2DrawExtended):
    surface = None

    def __init__(self, width, height, **kwargs):
        b2.b2DrawExtended.__init__(self, **kwargs)
        self.flipX = False
        self.flipY = True
        self.convertVertices = True
        self.zoom = 1
        self.center = (width / 2, height / 2)
        self.offset = (0, 0)
        self.screenSize = (width, height)
        self.axisScale = 0.5

    def DrawSegment(self, p1, p2, color):
        pygame.draw.aaline(self.surface, color.bytes, p1, p2)

    def DrawTransform(self, xf):
        p1 = xf.position
        p2 = self.to_screen(p1 + self.axisScale * xf.R.col1)
        p3 = self.to_screen(p1 + self.axisScale * xf.R.col2)
        p1 = self.to_screen(p1)

        pygame.draw.aaline(self.surface, (255, 0, 0), p1, p2)
        pygame.draw.aaline(self.surface, (0, 255, 0), p1, p3)

    def DrawCircle(self, center, radius, color, drawwidth=1):
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        pygame.draw.circle(self.surface, color.bytes, center, radius, drawwidth)

    def DrawSolidCircle(self, center, radius, axis, color):
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        pygame.draw.circle(self.surface, b2.b2Color(color / 2).bytes + [127], center, radius, 0)
        pygame.draw.circle(self.surface, color.bytes, center, radius, 1)
        pygame.draw.aaline(self.surface, (255, 0, 0), center,
                           (center[0] - radius * axis[0], center[1] + radius * axis[1]))

    def DrawPolygon(self, vertices, color):
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices[1])
        else:
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)

    def DrawSolidPolygon(self, vertices, color):
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices[1])
        else:
            pygame.draw.polygon(self.surface, b2.b2Color(color / 2).bytes + [127], vertices, 0)
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)

    def DrawPolygonShape(self, shape, transform, color):
        vert = [self.to_screen(b2.b2Mul(transform, v)) for v in shape.vertices]
        self.DrawSolidPolygon(vert, color)

    def DrawShape(self, shape, transform, color):
        if isinstance(shape, b2.b2PolygonShape):
            self.DrawPolygonShape(shape, transform, color)
        elif isinstance(shape, b2.b2EdgeShape):
            v1 = self.to_screen(b2.b2Mul(transform, shape.vertex1))
            v2 = self.to_screen(b2.b2Mul(transform, shape.vertex2))
            self.DrawSegment(v1, v2, color)
        elif isinstance(shape, b2.b2CircleShape):
            raise NotImplementedError()
        elif isinstance(shape, b2.b2LoopShape):
            vertices = shape.vertices
            v1 = b2.b2Mul(transform, vertices[-1])
            for v2 in vertices:
                v2 = b2.b2Mul(transform, v2)
                self.DrawSegment(v1, v2, color)
                v1 = v2

    def DrawBody(self, body, color):
        tr = body.transform
        for fixture in body:
            self.DrawShape(fixture.shape, tr, color)
        #self.DrawTransform(tr)


class RayCastClosestCallback(b2.b2RayCastCallback):
    """This callback finds the closest hit"""

    def __init__(self):
        b2.b2RayCastCallback.__init__(self)
        self.fraction = 1

    def ReportFixture(self, fixture, point, normal, fraction):
        if fixture.filterData.categoryBits != 1:
            return -1

        self.fraction = fraction
        return fraction


class Game:
    def __init__(self, count, map_, gui):
        self.gui = gui

        self.map = map_
        # self.size = self.width, self.height = int(self.map.size[0] / self.scale), int(
        #     self.map.size[1] / self.scale + 100)

        self.inputs = [[0] * 4 for _ in range(count)]
        self.outputs = [[0] * 10 for _ in range(count)]

        self.count = count
        self.cars = [None for _ in range(count)]
        self.cur_car = 0

        self.time = [0] * count
        self.ticks = 0
        self.go = [False] * count
        self.paths = [[] for _ in range(count)]

        self.sensors = []
        self.main_line = None
        self.walls = []
        self.all_dist = 0
        self.max_time = self.map.max_time
        self.finish = []

        self.world = None

        self.l = self.r = self.b = self.t = 0
        self.dt = 1 / 60
        self.car_power = 8500

        if not self.gui:
            return

        pygame.init()
        self.max_zoom = 7
        self.screen_size = (800, 600)
        self.screen = pygame.display.set_mode(self.screen_size, pygame.RESIZABLE)
        self.font = pygame.font.SysFont('Tahoma', 12, False, False)

        self.colors = {'car': (174, 130, 120), 'dead_car': (98, 54, 50),
                       'selected_car': (220, 200, 165), 'wall': (173, 255, 115), 'main_line': (13, 53, 72),
                       'finish': (122, 132, 163), 'sliding_tire': (255, 70, 70)}

        for k in self.colors:
            self.colors[k] = b2.b2Color(self.colors[k][0] / 255, self.colors[k][1] / 255, self.colors[k][2] / 255)

    def mf(self, v):
        return v[0], self.map.size[1] - v[1]

    def create_tire(self, car,
                    max_drive_force,
                    max_brake_force,
                    static_friction, sliding_friction):
        fix_def = b2.b2FixtureDef()
        fix_def.filter.categoryBits = 2
        fix_def.filter.maskBits = 1

        fix_def.shape = b2.b2PolygonShape(box=(0.245 / 2, 0.555 / 2))

        body_def = b2.b2BodyDef()
        body_def.type = b2.b2_dynamicBody

        body = self.world.CreateBody(body_def)
        body.car = car
        body.mass = 50
        body.inertia = 50
        body.sliding = False
        body.CreateFixture(fix_def)
        body.current_traction = 1
        body.max_drive_force = max_drive_force
        body.max_brake_force = max_brake_force
        body.static_friction = static_friction
        body.sliding_friction = sliding_friction

        def get_lateral_velocity():
            current_right_normal = body.GetWorldVector(b2.b2Vec2(1, 0))
            return b2.b2Dot(current_right_normal, body.linearVelocity) * current_right_normal

        def get_forward_velocity():
            current_forward_normal = body.GetWorldVector(b2.b2Vec2(0, 1))
            return b2.b2Dot(current_forward_normal, body.linearVelocity) * current_forward_normal

        def get_brake_orientation():
            current_forward_normal = body.GetWorldVector(b2.b2Vec2(0, 1))
            if b2.b2Dot(current_forward_normal, body.linearVelocity) > 0:
                return -current_forward_normal
            else:
                return current_forward_normal

        def update_drive(normal_reaction, acc, stop):
            current_forward_normal = body.GetWorldVector(b2.b2Vec2(0, 1))
            lateral_force = car.mass / 4 * -body.get_lateral_velocity() / self.dt
            drive_force = body.max_drive_force * (acc + 0.05) * current_forward_normal

            brake_force = b2.b2Vec2(0, 0)
            if get_forward_velocity().length > 2/3.6 and stop > 0:
                # braking
                brake_force = body.max_brake_force * stop * body.get_brake_orientation()
                if (drive_force + lateral_force).length > normal_reaction * body.static_friction:
                    # ABS unusable
                    # print("ABS unusable!")
                    pass
                elif (drive_force + lateral_force + brake_force).length > normal_reaction * body.static_friction:
                    # try ABS
                    abs_quality = 0.98
                    a = drive_force + lateral_force
                    b = brake_force
                    n = normal_reaction * body.static_friction * abs_quality
                    # solving ||a + x*b|| == n for x
                    xx = a.x * b.x
                    yy = a.y * b.y
                    d = 2 * (xx + yy) ** 2 - 4 * b.length ** 2 * (a.length ** 2 - n ** 2)
                    if d > 0:
                        abs_factor = min(max((math.sqrt(d) - 2 * (xx + yy)) /
                                             (2 * b.length ** 2), 0), 1)
                        brake_force *= abs_factor
                        # print("ABS! {}".format(abs_factor))

            res_force = lateral_force + drive_force + brake_force
            body.sliding = res_force.length > normal_reaction * body.static_friction
            if body.sliding:
                res_force *= normal_reaction * body.sliding_friction / res_force.length
                # print('Sliding! {}'.format(res_force))

            body.car.ApplyForce(res_force, body.worldCenter, True)

        body.get_forward_velocity = get_forward_velocity
        body.get_lateral_velocity = get_lateral_velocity
        body.get_brake_orientation = get_brake_orientation
        body.update_drive = update_drive

        return body

    def create_car(self, x, y):
        fix_def = b2.b2FixtureDef()
        # fix_def.friction = 0.3
        # fix_def.restitution = 0.1
        fix_def.filter.categoryBits = 2
        fix_def.filter.maskBits = 1

        fix_def.shape = b2.b2PolygonShape(box=(1.8 / 2, 4.6 / 2))
        # fix_def.shape = b2.b2PolygonShape(box=(0.5, 1))

        body_def = b2.b2BodyDef()
        body_def.type = b2.b2_dynamicBody
        body_def.position = (x, y)
        body_def.linearDamping = 0.05
        body_def.angularDamping = 3

        body = self.world.CreateBody(body_def)
        body.CreateFixture(fix_def)
        body.inertia = 600
        body.mass = 600
        body.sleepingAllowed = False

        body.max_forward_speed = 250 / 3.6
        body.max_backward_speed = -40 / 3.6

        back_tire_max_drive_force = 4500
        front_tire_max_drive_force = 0
        max_brake_force = 15000
        front_tire_static_friction = 0.95*3#0.95 * 3
        front_tire_sliding_friction = 0.8*3#0.8 * 3
        back_tire_static_friction = 0.8*3#0.95 * 3
        back_tire_sliding_friction = 0.7*3#0.8 * 3
        body.spoiler_max_downforce = body.mass * 1 * 9.8
        body.sploiler_min_speed = 40 / 3.6
        body.sploiler_max_speed = 250 / 3.6

        body.rftire = self.create_tire(body, front_tire_max_drive_force, max_brake_force,
                                       front_tire_static_friction,
                                       front_tire_sliding_friction)
        body.lftire = self.create_tire(body, front_tire_max_drive_force, max_brake_force,
                                       front_tire_static_friction,
                                       front_tire_sliding_friction)

        body.lbtire = self.create_tire(body, back_tire_max_drive_force, max_brake_force,
                                       back_tire_static_friction,
                                       back_tire_sliding_friction)
        body.rbtire = self.create_tire(body, back_tire_max_drive_force, max_brake_force,
                                       back_tire_static_friction,
                                       back_tire_sliding_friction)

        jointDef = b2.b2RevoluteJointDef()
        jointDef.bodyA = body
        jointDef.enableLimit = True
        jointDef.lowerAngle = 0
        jointDef.upperAngle = 0
        jointDef.localAnchorB.SetZero()

        jointDef.bodyB = body.lbtire
        body.lbtire.position = (x - 1.8 / 2, y - 0.7 * 4.6 / 2)
        jointDef.localAnchorA.Set(-1.8 / 2, -0.7 * 4.6 / 2)
        self.world.CreateJoint(jointDef)

        jointDef.bodyB = body.rbtire
        body.rbtire.position = (x + 1.8 / 2, y - 0.7 * 4.6 / 2)
        jointDef.localAnchorA.Set(1.8 / 2, -0.7 * 4.6 / 2)
        self.world.CreateJoint(jointDef)

        jointDef.bodyB = body.lftire
        body.lftire.position = (x - 1.8 / 2, y + 0.7 * 4.6 / 2)
        jointDef.localAnchorA.Set(-1.8 / 2, 0.7 * 4.6 / 2)
        body.lfJoint = self.world.CreateJoint(jointDef)

        jointDef.bodyB = body.rftire
        body.rftire.position = (x + 1.8 / 2, y + 0.7 * 4.6 / 2)
        jointDef.localAnchorA.Set(1.8 / 2, 0.7 * 4.6 / 2)
        body.rfJoint = self.world.CreateJoint(jointDef)

        body.angle = self.map.cars[0][2]
        body.lbtire.angle = self.map.cars[0][2]
        body.lftire.angle = self.map.cars[0][2]
        body.rbtire.angle = self.map.cars[0][2]
        body.rftire.angle = self.map.cars[0][2]

        body.lock_angle = math.radians(35)
        body.turn_speed_per_sec = math.radians(160)
        body.turn_per_time_step = body.turn_speed_per_sec * self.dt

        body.last_speed = b2.b2Vec2(0, 0)
        body.dv = b2.b2Vec2(0, 0)
        body.normal_reaction = 0

        def update(acc, stop, left, right):

            cur_speed = b2.b2Dot(body.GetWorldVector(b2.b2Vec2(0, 1)), body.linearVelocity)
            cur_spoiler_coeff = max(
                min((cur_speed - body.sploiler_min_speed) / (body.sploiler_max_speed - body.sploiler_min_speed), 1), 0)
            if cur_speed > 2 / 3 * body.max_forward_speed:
                # power limitation
                acc *= (body.max_forward_speed - cur_speed) / ((1 - 2 / 3) * body.max_forward_speed) / 2 + 0.5

            if cur_speed > body.max_forward_speed:
                # speed limitation
                acc = 0

            normal_reaction = body.mass * 0.5 * 0.5 * 9.8 + body.spoiler_max_downforce * cur_spoiler_coeff * 0.25
            body.normal_reaction = normal_reaction * 4
            body.lftire.update_drive(normal_reaction, acc, stop)
            body.rftire.update_drive(normal_reaction, acc, stop)
            body.lbtire.update_drive(normal_reaction, acc, stop)
            body.rbtire.update_drive(normal_reaction, acc, stop)

            desired_angle = body.lock_angle * (left - right)

            angle_now = body.rfJoint.angle
            angle_to_turn = min(max(desired_angle - angle_now, -body.turn_per_time_step), body.turn_per_time_step)
            new_angle = angle_now + angle_to_turn
            body.lfJoint.SetLimits(new_angle, new_angle)
            body.rfJoint.SetLimits(new_angle, new_angle)

        body.update = update
        return body

    def vf(self, vec):
        return vec.x, self.screen_size[1] - vec.y

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
        car = self.cur_car

        # You can uncomment this to draw sensors:
        # for j in range(self.count):
        #     if self.go[j]:
        #         continue
        #     for i in range(len(self.sensors)):
        #         pygame.draw.line(self.screen, (110, 110, 110, 155), self.world.renderer.to_screen(self.cars[j].position),
        #                          self.world.renderer.to_screen(
        #                              b2.b2Mul(self.cars[j].transform, (b2.b2Vec2(self.sensors[i]) * (
        #                                  1 - self.outputs[j][i + 3])))))

        if not self.go[car]:
            self.draw_bar(((50, 10), (10, 50)), (40, 40, 235), self.inputs[car][0])
            self.draw_bar(((65, 10), (10, 50)), (235, 40, 40), self.inputs[car][1])
            self.draw_bar(((80, 10), (10, 50)), (40, 235, 40), self.inputs[car][2])
            self.draw_bar(((95, 10), (10, 50)), (40, 235, 40), self.inputs[car][3])

            self.draw_bar(((350, 10), (10, 50)), (220, 40, 220), self.outputs[car][0])
            self.draw_bar(((365, 10), (10, 50)), (220, 220, 40), self.outputs[car][1])
            self.draw_bar(((380, 10), (10, 50)), (220, 220, 40), self.outputs[car][2])

            for i in range(3, len(self.outputs[car])):
                self.draw_bar(((350 + i * 15, 10), (10, 50)), (220, 220, 220), self.outputs[car][i])

        self.screen.blit(self.font.render("Time: {:.2f}".format(self.time[car]), True, (255, 255, 255)),
                         (250, 10))
        self.screen.blit(
            self.font.render("Speed: {:.2f} km/h".format(self.cars[car].linearVelocity.length * 3.6), True,
                             (255, 255, 255)), (550, 10))

        self.draw_bar(((125, 10), (10, 50)), (40, 235, 40), self.get_min_dist(car))

        self.draw_lamp((250, 30), 'Finished', (200, 200, 40), self.go[car])
        self.draw_lamp((250, 50), 'Touching', (200, 40, 40),
                       any(i.contact.touching for i in self.cars[car].contacts))

        car = self.cars[car]
        current_right_normal = car.GetWorldVector(b2.b2Vec2(1, 0))
        car_lat = (b2.b2Dot(current_right_normal, car.dv) / current_right_normal.length) / (9.8 * self.dt)
        current_forward_normal = car.GetWorldVector(b2.b2Vec2(0, 1))
        car_for = (b2.b2Dot(current_forward_normal, car.dv) / current_forward_normal.length) / (9.8 * self.dt)

        # overloads
        self.draw_bar(((730, 10), (10, 50)), (40, 235, 40), max(min(car_lat / 5, 1), 0))
        self.draw_bar(((745, 10), (10, 50)), (40, 235, 40), max(min(-car_lat / 5, 1), 0))
        self.draw_bar(((700, 10), (10, 50)), (40, 40, 235), max(min(-car_for / 5, 1), 0))
        self.draw_bar(((715, 10), (10, 50)), (235, 40, 40), max(min(car_for / 5, 1), 0))

        self.draw_bar(((765, 10), (10, 50)), (235, 40, 40), max(min(car.normal_reaction / car.mass / 9.8 / 5, 1), 0))

    def draw_paths(self):
        np = [[self.world.renderer.to_screen(j) for j in i] for i in self.paths]

        for j in range(self.count - 1, -1, -1):
            if self.go[j] and not self.cur_car == j:
                if len(np[j]) > 1:
                    pygame.draw.lines(self.screen, (self.colors['dead_car'] / 2).bytes, 0, np[j])

        for j in range(self.count - 1, -1, -1):
            if not self.go[j] and not self.cur_car == j:
                if len(np[j]) > 1:
                    pygame.draw.lines(self.screen, (self.colors['car'] / 2).bytes, 0, np[j])

        if len(np[self.cur_car]) > 1:
            pygame.draw.lines(self.screen, (self.colors['selected_car'] / 2).bytes, 0, np[self.cur_car])

    def draw_tire(self, tire, color):
        if tire.sliding:
            self.world.renderer.DrawBody(tire, (color + self.colors['sliding_tire']) / 2)
        else:
            self.world.renderer.DrawBody(tire, color)

    def draw_world(self):
        fdraw = self.world.renderer.to_screen(self.finish[0]), self.world.renderer.to_screen(self.finish[1])
        pygame.draw.rect(self.screen, (self.colors['finish'] / 2).bytes,
                         pygame.Rect(fdraw[0][0], fdraw[0][1], fdraw[1][0] - fdraw[0][0],
                                     fdraw[1][1] - fdraw[0][1]))
        for b in self.walls:
            self.world.renderer.DrawBody(b, self.colors['wall'])

        self.world.renderer.DrawBody(self.main_line, self.colors['main_line'])

        self.draw_paths()

        for j in range(self.count - 1, -1, -1):
            if self.go[j] and not self.cur_car == j:
                color = self.colors['dead_car']
                self.world.renderer.DrawBody(self.cars[j], color)
                self.draw_tire(self.cars[j].lbtire, color)
                self.draw_tire(self.cars[j].lftire, color)
                self.draw_tire(self.cars[j].rbtire, color)
                self.draw_tire(self.cars[j].rftire, color)

        for j in range(self.count - 1, -1, -1):
            if not self.go[j] and not self.cur_car == j:
                color = self.colors['car']
                self.world.renderer.DrawBody(self.cars[j], color)
                self.draw_tire(self.cars[j].lbtire, color)
                self.draw_tire(self.cars[j].lftire, color)
                self.draw_tire(self.cars[j].rbtire, color)
                self.draw_tire(self.cars[j].rftire, color)

        color = self.colors['selected_car']
        self.world.renderer.DrawBody(self.cars[self.cur_car], color)
        self.draw_tire(self.cars[self.cur_car].lbtire, color)
        self.draw_tire(self.cars[self.cur_car].lftire, color)
        self.draw_tire(self.cars[self.cur_car].rbtire, color)
        self.draw_tire(self.cars[self.cur_car].rftire, color)

    def restart(self):
        self.time = [0] * self.count
        self.ticks = 0
        self.go = [False] * self.count
        self.paths = [[] for _ in range(self.count)]
        self.inputs = [[0] * 4 for _ in range(self.count)]
        self.outputs = [[0] * 10 for _ in range(self.count)]
        self.world = b2.b2World((0, 0), True)

        self.r = r = self.map.size[0]
        self.l = l = 0
        self.t = t = self.map.size[1]
        self.b = b = 0

        self.walls = []

        self.walls.append(self.world.CreateStaticBody(shapes=[b2.b2EdgeShape(vertices=[(l, b), (r, b)]),
                                                              b2.b2EdgeShape(vertices=[(r, b), (r, t)]),
                                                              b2.b2EdgeShape(vertices=[(r, t), (l, t)]),
                                                              b2.b2EdgeShape(vertices=[(l, t), (l, b)])]))
        for wall in self.map.walls:
            shapes = []
            for i in range(len(wall) - 1):
                shapes.append(b2.b2EdgeShape(vertices=[self.mf(wall[i]), self.mf(wall[i + 1])]))
            self.walls.append(self.world.CreateStaticBody(shapes=shapes))

        main_line_shapes = []
        for i in range(len(self.map.headline) - 1):
            main_line_shapes.append(
                b2.b2EdgeShape(vertices=[self.mf(self.map.headline[i]), self.mf(self.map.headline[i + 1])]))
        self.main_line = self.world.CreateStaticBody(shapes=main_line_shapes)

        self.all_dist = 0
        for i in self.main_line.fixtures:
            i.sensor = True
            i.filterData.categoryBits = 0
            sh = i.shape
            self.all_dist += (b2.b2Vec2(sh.vertices[1]) - b2.b2Vec2(sh.vertices[0])).length

        self.cars = [self.create_car(*self.mf(self.map.cars[0][:2])) for _ in range(self.count)]
        self.cur_car = 0
        fin = [self.mf(self.map.finish[0]), self.mf(self.map.finish[1])]
        self.finish = (min(fin[0][0], fin[1][0]), min(fin[0][1], fin[1][1])), \
                      (max(fin[0][0], fin[1][0]), max(fin[0][1], fin[1][1]))

        n = 3
        rad = 30
        angle = math.pi / 4
        self.sensors = [(math.sin(i * angle / n) * rad, math.cos(i * angle / n) * rad) for i in range(-n, n + 1)]

        if not self.gui:
            return

        self.world.renderer = PygameDraw(self.screen_size[0], self.screen_size[1], surface=self.screen)
        self.world.renderer.flags = dict(
            drawShapes=True,
            drawJoints=False,
            drawAABBs=False,
            drawPairs=False,
            drawCOMs=True,
            convertVertices=isinstance(self.world.renderer, b2.b2DrawExtended)
        )

    def dispatch_messages(self):
        if not self.gui:
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    self.restart()
                elif event.key == pygame.K_ESCAPE:
                    sys.exit()
                elif event.key == pygame.K_KP_PLUS:
                    self.max_zoom *= 1.1
                elif event.key == pygame.K_KP_MINUS:
                    self.max_zoom /= 1.1
            if event.type == pygame.VIDEORESIZE:
                self.screen_size = event.size
                self.screen = pygame.display.set_mode(self.screen_size, pygame.RESIZABLE)

                self.world.renderer = PygameDraw(self.screen_size[0], self.screen_size[1], surface=self.screen)
                self.world.renderer.flags = dict(
                    drawShapes=True,
                    drawJoints=False,
                    drawAABBs=False,
                    drawPairs=False,
                    drawCOMs=True,
                    convertVertices=isinstance(self.world.renderer, b2.b2DrawExtended)
                )

    def get_inputs(self):
        keys = pygame.key.get_pressed()
        inputs = [[0] * 4 for _ in range(self.count)]
        inputs[0][0] = keys[pygame.K_UP]
        inputs[0][1] = keys[pygame.K_DOWN] | keys[pygame.K_LSHIFT]
        inputs[0][2] = keys[pygame.K_LEFT]
        inputs[0][3] = keys[pygame.K_RIGHT]
        return inputs

    def set(self, inputs):
        for i in range(self.count):
            if self.go[i]:
                continue
            car = self.cars[i]
            inputs[i] = [max(0, min(1, i)) for i in inputs[i]]
            car.update(*inputs[i])
            self.go[i] |= self.is_finish(i)
            if self.go[i]:
                car.linearVelocity = b2.b2Vec2(0, 0)

    def get(self):
        outputs = [None for _ in range(self.count)]
        callback = RayCastClosestCallback()
        for i in range(self.count):
            if self.go[i]:
                continue
            car = self.cars[i]
            car_pos = car.position
            car_r = car.transform.R
            current_right_normal = car.GetWorldVector(b2.b2Vec2(1, 0))
            car_lat = (b2.b2Dot(current_right_normal,
                                (car.last_speed - car.linearVelocity)) / current_right_normal.length) / (9.8 * self.dt)
            car.dv = b2.b2Vec2(car.last_speed - car.linearVelocity)

            car.last_speed = b2.b2Vec2(car.linearVelocity)

            outputs[i] = [min(max(car.linearVelocity.length / (330 / 3.6), 0), 1),
                          min(max(car_lat / 3, 0), 1),
                          min(max(-car_lat / 3, 0), 1)]

            for j in self.sensors:
                callback.fraction = 1
                self.world.RayCast(callback, car_pos, car_pos + car_r * j)
                outputs[i].append(min(max(1 - callback.fraction, 0), 1))

        return outputs

    def is_really_finish(self, j):
        return (self.finish[0][0] < self.cars[j].position.x < self.finish[1][0] and
                self.finish[0][1] < self.cars[j].position.y < self.finish[1][1])

    def is_finish(self, j):
        return self.is_really_finish(j) or any(i.contact.touching for i in self.cars[j].contacts_gen) \
               or self.time[j] > self.max_time

    def get_min_dist(self, j):
        min_d = -1
        min_point = (0, 0)
        min_fix = ()
        fixtures_cache = self.main_line.fixtures
        car_shape = self.cars[j].fixtures[0].shape
        fix_transform = self.main_line.transform
        car_transform = self.cars[j].transform
        for i in range(len(fixtures_cache)):
            _, point_b, dist, _ = b2.b2Distance(shapeA=car_shape, shapeB=fixtures_cache[i].shape,
                                                transformA=car_transform, transformB=fix_transform)
            if min_d > dist or min_d < 0:
                min_d = dist
                min_point = point_b
                min_fix = i

        dist_start = 0
        for i in range(min_fix):
            sh = fixtures_cache[i].shape
            dist_start += (b2.b2Vec2(sh.vertices[1]) - b2.b2Vec2(sh.vertices[0])).length

        dist_in = (b2.b2Vec2(fixtures_cache[min_fix].shape.vertices[0]) - b2.b2Vec2(min_point)).length

        result_dist = (dist_start + dist_in) / self.all_dist
        return result_dist

    def draw(self):
        if not self.gui:
            return
        # self.world.renderer.zoom = min(
        #     self.screen_size[0] / self.map.size[0],
        #     (self.screen_size[1] - 100) / self.map.size[1], self.max_zoom
        # )
        #
        # self.world.renderer.offset = (
        #     -(self.screen_size[0] / 2 - self.map.size[0] / 2 * self.world.renderer.zoom),
        #     -((self.screen_size[1] - 100) / 2 - self.map.size[1] / 2 * self.world.renderer.zoom)
        # )

        self.world.renderer.zoom = self.max_zoom * 1
        self.world.renderer.offset = (
            -(self.screen_size[0] / 2 - self.cars[self.cur_car].position[0] * self.world.renderer.zoom),
            -(self.screen_size[1] / 2 - self.cars[self.cur_car].position[1] * self.world.renderer.zoom)
        )
        # dx, dy = 0, 0
        #
        # if self.world.renderer.offset[0] > 0:
        #     dx -= self.world.renderer.offset[0]
        #
        # if self.world.renderer.offset[1] > 0:
        #     dy -= self.world.renderer.offset[1]
        #
        # if -self.world.renderer.offset[0] + self.map.size[0]*self.world.renderer.zoom - self.screen_size[0] > 0:
        #     dx += -self.world.renderer.offset[0] + self.map.size[0]*self.world.renderer.zoom - self.screen_size[0]
        #
        # if -self.world.renderer.offset[1] + self.map.size[1] * self.world.renderer.zoom - self.screen_size[1] +100> 0:
        #     dy += -self.world.renderer.offset[1] + self.map.size[1] * self.world.renderer.zoom - self.screen_size[1]+100
        #
        # self.world.renderer.offset = (
        #     self.world.renderer.offset[0] + dx,
        #     self.world.renderer.offset[1] + dy
        # )

        for i in range(self.count):
            if not self.go[i]:
                self.cur_car = i
                break

        self.screen.fill((0, 0, 0))
        # self.world.DrawDebugData()
        self.draw_world()
        self.draw_interface()
        pygame.display.flip()

    def tick(self):
        self.ticks += 1
        self.time = list(map((lambda x, y: x + (not y) * self.dt), self.time, self.go))
        self.world.Step(self.dt, 10, 10)
        self.world.ClearForces()

        if not self.gui:
            return

        if self.ticks % 10 == 0:
            for i in range(self.count):
                if self.go[i]:
                    continue
                self.paths[i].append(b2.b2Vec2(self.cars[i].position.x, self.cars[i].position.y))

    def get_fitness(self, i):
        return self.get_min_dist(i) * 1000 + self.is_really_finish(i) * (self.max_time - self.time[i]) * 100


def main():
    parser = argparse.ArgumentParser(description="race environment for nlab (MANUAL MODE)")
    parser.add_argument("file", metavar="file", type=str, help="map file (default: %(default)s)", nargs='?',
                        default="default.json")
    args = parser.parse_args()

    map_ = Map.open_from_file(args.file)

    game = Game(2, map_, True)
    game.restart()

    old_time = time.perf_counter()

    while True:
        if time.perf_counter() - old_time > game.dt:
            old_time = time.perf_counter()
            if not all(game.go):
                game.tick()
                game.outputs = game.get()
                old_inputs = game.inputs[0]
                game.inputs = game.get_inputs()
                game.inputs[0] = list(map((lambda x, y: min(max(x + (y - 0.5) * 6 * game.dt, 0), 1)), old_inputs,
                                          game.inputs[0]))
                game.set(game.inputs)

            game.dispatch_messages()
            game.draw()


if __name__ == "__main__":
    main()
