import math, time, random
from RLUtilities.LinearAlgebra import *
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3 as StateVector3, Rotator
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState


def getClosestPad(agent):
    pads = agent.info.boost_pads
    closestPad = None
    distToClosestPad = math.inf
    for i in range(len(pads)):
        if(distance2D(agent.info.my_car.pos, pads[i].pos) < distToClosestPad):
            distToClosestPad = distance2D(agent.info.my_car.pos, pads[i].pos)
            closestPad = pads[i]
    return closestPad

def quad(a,b,c):
    inside = (b**2) - (4*a*c)
    if inside < 0 or a == 0:
        return 0.1
    else:
        n = ((-b - math.sqrt(inside))/(2*a))
        p = ((-b + math.sqrt(inside))/(2*a))
        if p > n:
            return p
        return n

def timeZ(ball):
    rate = 0.97
    return quad(-325, ball.vel[2] * rate, ball.pos[2]-92.75)

def ballReady(agent):
    ball = agent.info.ball
    if abs(ball.vel[2]) < 150 and timeZ(ball) < 1:
            return True
    return False


def ballProject(agent):
    goal = agent.info.their_goal
    goalToBall = normalize(agent.info.ball.pos - goal.center)
    diff = agent.info.my_car.pos - agent.info.my_car.pos
    return dot(diff, goalToBall)

def cap(x, low, high):
    if x < low:
        return low
    elif x > high:
        return high
    else:
        return x

def sign(x):
    if x <= 0:
        return -1
    else:
        return 1

def distance2D(targetObject, ourObject):
    difference = targetObject - ourObject
    return math.sqrt(difference[0]**2 + difference[1]**2)

def velocity2D(targetObject):
    return math.sqrt(targetObject.vel[0]**2 + targetObject.vel[1]**2)

def angle2D(targetLocation, objectLocation):
    difference = targetLocation - objectLocation
    return math.atan2(difference[1], difference[0])
