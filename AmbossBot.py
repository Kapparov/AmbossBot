import math
import time

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from RLUtilities.GameInfo import GameInfo
from RLUtilities.Simulation import Car, Ball
from RLUtilities.LinearAlgebra import *

from AmbossUtil import *
from RLUtilities.LinearAlgebra import *
from RLUtilities.Maneuvers import *

from RLUtilities.controller_input import controller

max_throttle_speed = 1410
max_boost_speed = 2300

GOAL_WIDTH = 1900
FIELD_LENGTH = 10280
FIELD_WIDTH = 8240



# TODO timer zu boostpickup hinzufügen um feststecken zu vermeiden

class Agent(BaseAgent):

    def initialize_agent(self):
        # self.info = GameInfo(self.index, self.team, self.get_field_info())
        # self.info2 = GameInfo(self.index, self.team)
        self.info = GameInfo(self.index, self.team)
        self.controls = SimpleControllerState()
        self.action = None
        self.picking_up_boost = 0
        self.kickoff = False
        self.startGrabbingBoost = time.time()
        self.time = time.time()
        self.own_mode = "BallChase"
        self.shoot_mode = calcShot()
        self.air_dodging_for_kickoff = False
        self.half_flipping = False
        self.driving_to_defend = False
        self.dodging_for_shot = False
        self.slowDOWN = False

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        # useful Variables
        self.slowDOWN = False
        self.info.read_packet(packet)
        target_speed = max_boost_speed
        ball = self.info.ball
        car = self.info.my_car
        team = car.team  # blue = 0 orange = 1
        delta_local = dot(ball.pos - car.pos, car.theta)
        phi = math.atan2(delta_local[1], delta_local[0])
        # check for driving_to_defend if its Active we dont want to go into any other states
        if self.driving_to_defend:
            return drive_to(self, self.info.my_goal.center)
        # check for AirDodge if its Active we don´t want to go into any other states
        if type(self.action) == AirDodge:
            if self.action.finished:
                self.air_dodging_for_kickoff = False
                self.dodging_for_shot = False
                self.action = None
            else:
                self.air_dodging_for_kickoff = True
                self.action.step(0.016666)
                return self.action.controls
        # check for Halfflip if its Active we don´t want to go into any other states
        if type(self.action) == HalfFlip and self.half_flipping:
            if self.action.finished:
                self.half_flipping = False
                self.action = None
                # switch to shoot
                return self.shoot_mode.execute(self)
            else:
                self.half_flipping = True
                self.action.step(0.016666)
                return self.action.controls
        # make prediction on ball position for later decisions
        b = Ball(self.info.ball)
        ball_predictions = []
        for i in range(180):
            b.step(1.0 / 60.0)
            ball_predictions.append(vec3(b.pos))
        # Activate Halfflip if Ball is behind Car TODO needs to get fixed it sucks atm
        # if self.picking_up_boost == 0 and car.on_ground and car.pos[2] < 270 and distance2D(ball.pos, car.pos) > 400 and self.driving_to_defend == False:
            # if abs(phi) >= 2:
                # self.half_flipping = True
                # if type(self.action) != HalfFlip:
                    # self.action = HalfFlip(self.info.my_car)
                # self.action.step(0.016666)
                # print("Initialised Halfflip")
                # return self.action.controls
        # Kickoff when Ball is in the Center and Kickoff game info
        if packet.game_info.is_kickoff_pause:
            self.kickoff = True
        else:
            self.kickoff = False
        if self.kickoff:
            self.state = kickOff(self)
            return self.state.execute(self)
        # PickupBoost if circumstances are right
        if self.picking_up_boost == 1 and self.kickoff == False:
            return drive_to_closest_pad(self)
        if ball.pos[2] > 270:
            if (car.team == 1 and car.pos[1] > 0) or (car.team == 0 and car.pos[1] < 0):
                if ball_predictions[60][2] > 300 and car.on_ground and not self.kickoff:
                    if car.boost < 35:
                        self.picking_up_boost = 1
                        return drive_to_closest_pad(self)
        else:
            self.picking_up_boost = 0
        # Turn to the ball if the angle is bad
        if abs(phi) >= 1 and self.driving_to_defend == False:
            self.controls.steer = clip(2.5 * phi, -1.0, 1.0)
            self.controls.throttle = 1
            if abs(phi) >= 1.5:
                # self.controls.handbrake = 1
                self.controls.handbrake = 1
            if abs(phi) < 1.5:
                self.controls.handbrake = 0
            if self.controls.handbrake == 1:
                self.controls.boost = 0
            return self.controls
        # Shoot if circumstances are right
        if (ballReady(self) and ballProject(self) > 500 - (distance2D(self.info.ball.pos, self.info.my_car.pos) / 2)) or distance2D(self.info.ball.pos, self.info.my_goal.center) < 3000:
            return self.shoot_mode.execute(self)
        # go into defending if Ball is in our half and if Ball is traveling towards our goal
        if team == 1:
            if car.pos[1] < ball.pos[1]:  # checks if we are on the wrong side of the ball
                if ball_predictions[120][0] < 2000 and ball_predictions[120][1] > 0:
                    self.driving_to_defend = True
                    return drive_to(self, self.info.my_goal.center)
        if team == 2:
            if car.pos[1] > ball.pos[1]:  # checks if we are on the wrong side of the ball
                if ball_predictions[120][0] < 2000 and ball_predictions[120][1] < 0:
                    self.driving_to_defend = True
                    return drive_to(self, self.info.my_goal.center)
        # shoot if everythin else sux
        return self.shoot_mode.execute(self)


def drive_to_closest_pad(self):
    self.controls = SimpleControllerState()
    target_speed = max_boost_speed

    closest_boost_pad = get_closest_pad(self)
    target_pos = closest_boost_pad.pos
    if closest_boost_pad.is_active == False:
        self.picking_up_boost = 0
    delta_local = dot(target_pos - self.info.my_car.pos, self.info.my_car.theta)
    phi = math.atan2(delta_local[1], delta_local[0])
    self.controls.steer = clip(2.5 * phi, -1.0, 1.0)
    if abs(phi) > 1.7:
        self.controls.handbrake = 1
    if abs(phi) < 1.5:
        self.controls.handbrake = 0
    if self.controls.handbrake == 1:
        self.controls.boost = 0
    else:
        vf = dot(self.info.my_car.vel, self.info.my_car.forward())
        if vf < target_speed:
            self.controls.throttle = 1.0
            if target_speed >= max_throttle_speed:
                self.controls.boost = 1
            else:
                self.controls.boost = 0
        else:
            if (vf - target_speed) > 75:
                self.controls.throttle = -1.0
            else:
                if self.info.my_car.up()[2] > 0.85:
                    self.controls.throttle = 0.0
                else:
                    self.controls.throttle = 0.01
                self.controls.boost = 0
    if norm(self.info.my_car.pos - target_pos) < 200 or self.info.my_car.boost > 50:
        self.picking_up_boost = 0
    return self.controls


def drive_to(self, targeted_position):
    self.controls = SimpleControllerState()
    target_speed = max_boost_speed

    target_pos = targeted_position

    delta_local = dot(target_pos - self.info.my_car.pos, self.info.my_car.theta)
    phi = math.atan2(delta_local[1], delta_local[0])
    self.controls.steer = clip(2.5 * phi, -1.0, 1.0)
    if abs(phi) > 1.7:
        self.controls.handbrake = 1
    if abs(phi) < 1.5:
        self.controls.handbrake = 0
    if self.controls.handbrake == 1:
        self.controls.boost = 0
    else:
        vf = dot(self.info.my_car.vel, self.info.my_car.forward())
        if vf < target_speed:
            self.controls.throttle = 1.0
            if target_speed >= max_throttle_speed:
                self.controls.boost = 1
            else:
                self.controls.boost = 0
        else:
            if (vf - target_speed) > 75:
                self.controls.throttle = -1.0
            else:
                if self.info.my_car.up()[2] > 0.85:
                    self.controls.throttle = 0.0
                else:
                    self.controls.throttle = 0.01
                self.controls.boost = 0
    # check for defense
    if self.info.team == 1:
        if self.info.my_car.pos[1] > self.info.ball.pos[1]:  # checks if we are on the right side of the ball
            self.driving_to_defend = False
    if self.info.team == 2:
        if self.info.my_car.pos[1] < self.info.ball.pos[1]:  # checks if we are on the right side of the ball
            self.driving_to_defend = False
    if self.slowDOWN:
        self.controls.boost = 0
        # if distance2D(target_pos, self.info.my_car.pos) < 800:
        self.controls.throttle = 0.1
    return self.controls


def get_closest_pad(agent):
    pads = agent.info.boost_pads
    closestPad = None
    distToClosestPad = math.inf
    for i in range(len(pads)):
        if pads[i].is_active:
            if distance3d(agent.info.my_car.pos, pads[i].pos) < distToClosestPad:
                distToClosestPad = distance3d(agent.info.my_car.pos, pads[i].pos)
                closestPad = pads[i]
    return closestPad


def distance3d(target_object, our_object):
    return math.sqrt((target_object[0] - our_object[0]) ** 2 + (target_object[1] - our_object[1]) ** 2 + (
                target_object[2] - our_object[2]) ** 2)


# TODO improve
class calcShot:
    def __init__(self):
        self.expired = False

    def __str__(self):
        return "Shooting"

    def available(self, agent):
        if (ballReady(agent) and ballProject(agent) > 500 - (
                distance2D(agent.info.ball.pos, agent.info.my_car.pos) / 2)) or distance2D(agent.info.ball.pos,
                                                                                           agent.info.my_goal.center) < 3000:
            return True
        return False

    def execute(self, agent):
        if agent.info.ball.pos[2] > 270:
            return wait_for_ball_to_drop(agent)
        leftPost = agent.info.their_goal.corners[3]
        rightPost = agent.info.their_goal.corners[2]
        center = agent.info.their_goal.center
        ball = agent.info.ball
        amboss = agent.info.my_car
        ballLeft = angle2D(ball.pos, leftPost)
        ballRight = angle2D(ball.pos, rightPost)
        agentLeft = angle2D(amboss.pos, leftPost)
        agentRight = angle2D(amboss.pos, rightPost)

        # determining if we are left/right/inside of cone
        if agentLeft > ballLeft and agentRight > ballRight:
            target = rightPost
        elif agentLeft > ballLeft and agentRight < ballRight:
            target = None
        elif agentLeft < ballLeft and agentRight < ballRight:
            target = leftPost
        else:
            target = None

        if target != None:
            goalToBall = normalize(ball.pos - target)
            goalToAgent = normalize(amboss.pos - target)
            difference = goalToBall - goalToAgent
            error = cap(abs(difference[0]) + abs(difference[1]), 1, 10)
        else:
            goalToBall = normalize(amboss.pos - ball.pos)
            error = cap(distance2D(ball.pos, amboss.pos) / 1000, 0, 1)

        goalToBall2D = vec2(goalToBall[0], goalToBall[1])
        testVector2D = dot(rotation(0.5 * math.pi), goalToBall2D)
        testVector = vec3(testVector2D[0], testVector2D[1], 0)

        # same as Gosling's old distance calculation, but now we consider dpp_skew which helps us handle when the ball is moving
        targetDistance = cap((40 + distance2D(ball.pos, amboss.pos) * (error ** 2)) / 1.8, 0, 4000)
        targetLocation = ball.pos + vec3((goalToBall[0] * targetDistance), goalToBall[1] * targetDistance, 0)

        # this adjusts the target based on the ball velocity perpendicular to the direction we're trying to hit it
        multiplier = cap(distance2D(amboss.pos, targetLocation) / 1500, 0, 2)
        targetModDistance = cap(dot(testVector, ball.vel) * multiplier, -1000, 1000)
        finalModVector = vec3(testVector[0] * targetModDistance, testVector[1] * targetModDistance, 0)
        preLoc = targetLocation
        targetLocation += finalModVector

        # another target adjustment that applies if the ball is close to the wall
        extra = 3850 - abs(targetLocation[0])
        if extra < 0:
            # we prevent our target from going outside the wall, and extend it so that Gosling gets closer to the wall before taking a shot, makes things more reliable
            targetLocation[0] = cap(targetLocation[0], -3850, 3850)
            targetLocation[1] = targetLocation[1] + (-sign(agent.team) * cap(extra, -800, 800))

        # getting speed, this would be a good place to modify because it's not very good
        targetLocal = dot(targetLocation - amboss.pos, amboss.theta)
        angleToTarget = cap(math.atan2(targetLocal[1], targetLocal[0]), -3, 3)
        distanceToTarget = distance2D(amboss.pos, targetLocation)
        if distanceToTarget > 2.5 * velocity2D(amboss):
            speed = 2300
        else:
            speed = 2300 - (340 * (angleToTarget ** 2))
        if not ballReady(agent):
            self.expired = True
            agent.action = None
        if agent.action is None:
            agent.action = Drive(agent.info.my_car, targetLocation, speed)
        else:
            agent.action.target_pos = targetLocation
            agent.action.target_speed = speed
        if distance2D(agent.info.my_car.pos, agent.info.ball.pos) < 400 and type(agent.action) != AirDodge:
            agent.action = AirDodge(agent.info.my_car, 0.1, agent.info.ball.pos)
            agent.dodging_for_shot = True
        agent.action.step(0.016666)
        return agent.action.controls


def wait_for_ball_to_drop(agent):
    agent.slowDOWN = True
    b = Ball(agent.info.ball)
    ball_predictions = []
    for i in range(180):
        b.step(1.0 / 60.0)
        ball_predictions.append(vec3(b.pos))
    if agent.info.my_car.team == 1:
        ball_predictions[120][1] = ball_predictions[120][1] - 1000
    else:
        ball_predictions[120][1] = ball_predictions[120][1] + 1000
    return drive_to(agent, ball_predictions[120])


class kickOff:
    def __init__(self, agent):
        self.expired = False
        agent.action = Drive(agent.info.my_car, agent.info.ball.pos, 2300)

    def __str__(self):
        return "Kickoff"

    def execute(self, agent):
        if not agent.kickoff or (type(agent.action) == AirDodge and agent.info.my_car.on_ground):
            self.expired = True
            agent.action = None
        if distance2D(agent.info.my_car.pos, agent.info.ball.pos) < 750 and type(agent.action) != AirDodge:
            agent.action = AirDodge(agent.info.my_car, 0.1, agent.info.ball.pos)
        agent.action.step(0.016666)
        return agent.action.controls
