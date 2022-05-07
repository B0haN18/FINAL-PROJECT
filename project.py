#!/usr/bin/env python3
from environs import Env
import os
import lgsvl
import sys
import time
import math
import random
import pickle
from sympy import Point3D, Line3D, Segment3D, Point2D, Line2D, Segment2D
from environs import Env
import liability
from datetime import datetime
import util
import copy
from Chromosome import Chromosome
from random import randrange

def print_debug(info):
    print(info)
    with open('apollo6.0.log', 'a') as f:
        print(info, file=f)
        f.close()

class LgApSimulation:

    def __init__(self):
        ################################################################
        self.totalSimTime = 15
        self.bridgeLogPath = "/home/bohan/apollo-5.0/data/log/cyber_bridge.INFO"
        ################################################################
        self.sim = None
        self.ego = None  # There is only one ego
        self.initEvPos = lgsvl.Vector(7.44000864028931, 0.0, -40.2100067138672)
        self.npcList = []  # The list contains all the npc added
        self.initSimulator()
        self.loadMap()
        self.initEV()
        self.isEgoFault = False
        self.isHit = False
        self.connectEvToApollo()
        self.maxint = 130
        self.egoFaultDeltaD = 0
        self.env = Env()

    def initSimulator(self):
        sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
        # LGSVL__SIMULATOR_HOST = self.env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
        # LGSVL__SIMULATOR_PORT = self.env.int("LGSVL__SIMULATOR_PORT", 8181)
        # sim = lgsvl.Simulator(LGSVL__SIMULATOR_HOST, LGSVL__SIMULATOR_PORT)
        self.sim = sim

    def loadMap(self, mapName=lgsvl.wise.DefaultAssets.map_straight2lanesame):
        # 5d272540-f689-4355-83c7-03bf11b6865f
        sim = self.sim
        if sim.current_scene == mapName:
            sim.reset()
        else:
            sim.load(lgsvl.wise.DefaultAssets.map_straight2lanesame)

    def initEV(self):
        sim = self.sim
        egoState = lgsvl.AgentState()
        egoState.transform = sim.map_point_on_lane(self.initEvPos)
        # egoState.transform.position += 25  # 5m forwards
        ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5, lgsvl.AgentType.EGO, egoState)
        # sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5),lgsvl.AgentType.EGO, egoState)
        # print(ego)
        sensors = ego.get_sensors()
        for s in sensors:
            if s.name in ['velodyne', 'Main Camera', 'Telephoto Camera', 'GPS', 'IMU']:
                s.enabled = True
        self.ego = ego

    def connectEvToApollo(self):
        ego = self.ego
        print("Connecting to bridge")
        env = Env()
        # ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)
        ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host),
                           env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))
        while not ego.bridge_connected:
            time.sleep(1)
        print("Bridge connected")

    def addNpcVehicle(self, posVector, vehicleType="SUV"):
        sim = self.sim
        npcList = self.npcList
        # print("npc vehicle list: ")
        # print(npcList)
        npcState = lgsvl.AgentState()

        npcState.transform = sim.map_point_on_lane(posVector)
        npc = sim.add_agent(vehicleType, lgsvl.AgentType.NPC, npcState)
        # self.npcList = [npc]
        npcList.append(npc)

    def addFixedMovingNpc(self, posVector, vehicleType="SUV"):
        sim = self.sim
        npcState = lgsvl.AgentState()
        npcState.transform = sim.map_point_on_lane(posVector)
        npc = sim.add_agent(vehicleType, lgsvl.AgentType.NPC, npcState)
        npc.follow_closest_lane(True, 7.4)

    # This function send an instance action command to the NPC at the current time instance
    def setNpcSpeed(self, npc, speed):
        npc.follow_closest_lane(True, speed)

    # Direction is either "LEFT" or "RIGHT"
    def setNpcChangeLane(self, npc, direction):
        if direction == "LEFT":
            npc.change_lane(True)
        elif direction == "RIGHT":
            npc.change_lane(False)

    def setEvThrottle(self, throttle):
        ego = self.ego
        c = lgsvl.VehicleControl()
        c.throttle = throttle
        ego.apply_control(c, True)

    def brakeDist(self, speed):
        dBrake = 0.0467 * pow(speed, 2.0) + 0.4116 * speed - 1.9913 + 0.5
        if dBrake < 0:
            dBrake = 0
        return dBrake

    def findCollisionDeltaD(self, ego, npc):
        d = liability.findDistance(ego, npc) - 4.6  # 4.6 is the length of a car
        return d - self.brakeDist(ego.state.speed)

    def findDeltaD(self, ego, npc):
        d = liability.findDistance(ego, npc) - 4.6  # 4.6 is the length of a car
        deltaD = self.maxint  # The smaller delta D, the better
        deltaDFront = self.maxint
        deltaDSide = self.maxint

        # When npc is in front
        if npc.state.transform.position.x + 4.6 < ego.state.transform.position.x and npc.state.transform.position.x + 20 > ego.state.transform.position.x:
            if npc.state.transform.position.z > ego.state.transform.position.z - 2 and npc.state.transform.position.z < ego.state.transform.position.z + 2:
                deltaDFront = d - self.brakeDist(ego.state.speed)
                util.print_debug(" --- Delta D Front: " + str(deltaDFront))

        # When ego is changing line to npc's front
        if npc.state.transform.position.x - 4.6 > ego.state.transform.position.x and npc.state.transform.position.x - 20 < ego.state.transform.position.x:
            if npc.state.transform.position.z + 2 > ego.state.transform.position.z and npc.state.transform.position.z - 2 < ego.state.transform.position.z and (
                    ego.state.rotation.y < 269 or ego.state.rotation.y > 271):
                deltaDSide = d - self.brakeDist(npc.state.speed)
                util.print_debug(" --- Delta D Side: " + str(deltaDSide))

        deltaD = min(deltaDSide, deltaDFront)
        # print("mydelta d is")
        # print(deltaDSide)

        return deltaD

    def random_select(self,x):
        return (randrange(x))

    def runSimulation(self):
        test_times = 10
        current_test_time = 0
        # while current_test_time < 10:
        # for current_test_time in range(test_times):
        chromosome = Chromosome([[0, 35], [0, 3]], 1, 10)
        chromosome.rand_init()
        scenarioObj = chromosome.scenario

        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        util.print_debug("\n === Run simulation === [" + date_time + "]")
        sim = self.sim
        npcList = self.npcList
        ego = self.ego

        init_degree = ego.state.rotation.y
        numOfTimeSlice = 10
        numOfNpc = 1
        deltaDList = [[self.maxint for i in range(numOfTimeSlice)] for j in
                      range(numOfNpc)]  # 1-D: NPC; 2-D: Time Slice
        dList = [[self.maxint for i in range(numOfTimeSlice)] for j in range(numOfNpc)]  # 1-D: NPC; 2-D: Time Slice
        spawns = sim.get_spawn()

        # Add NPCs: Hard code for now, the number of npc need to be consistent.
        ################################################################
        self.addNpcVehicle(lgsvl.Vector(3.39741945266724, 0, -65.2101135253906), "Sedan")
        self.addFixedMovingNpc(lgsvl.Vector(7.39741945266724, 0, -25.2101135253906), "Sedan")
        #self.addNpcVehicle(lgsvl.Vector(7.39741945266724, 0, -25.2101135253906), "Sedan")
        #################################################################
        for npc in npcList:
            npc.follow_closest_lane(True, random.randint(1,9))

        self.isEgoFault = False
        self.isHit = False

        def on_collision(agent1, agent2, contact):
            #util.print_debug(" --- On Collision, ego speed: " + str(agent1.state.speed) + ", NPC speed: " + str(agent2.state.speed))
            if self.isHit == True:
               return
            self.isHit = True
            if agent2 is None or agent1 is None:
                self.isEgoFault = True
                util.print_debug(" --- Hit road obstacle --- ")
                return

            apollo = agent1
            npcVehicle = agent2
            if agent2.name == "Lincoln2017MKZ":
                apollo = agent2
                npcVehicle = agent1
            util.print_debug(" --- On Collision, ego speed: " + str(apollo.state.speed) + ", NPC speed: " + str(npcVehicle.state.speed))
            if apollo.state.speed <= 0.005:
               self.isEgoFault = False
               return
            self.isEgoFault = liability.isEgoFault(apollo, npcVehicle, sim, init_degree)
            # Compute deltaD when it is ego fault
            if self.isEgoFault == True:
                self.egoFaultDeltaD = self.findCollisionDeltaD(apollo, npcVehicle)
                util.print_debug("it is ego fault")
                util.print_debug("====current collision")
                util.print_debug("= current npc location")
                util.print_debug(npc.state.transform.position)
                util.print_debug("= current npc speed")
                util.print_debug(npc.state.speed)
                util.print_debug("= current npc angle")
                util.print_debug(npc.state.transform.rotation)
                util.print_debug("= current ego location")
                util.print_debug(ego.state.transform.position)
                util.print_debug("= current ego speed")
                util.print_debug(ego.state.speed)
                util.print_debug("= current ego angle")
                util.print_debug(ego.state.transform.rotation)
                util.print_debug("******************************************************************")

        ego.on_collision(on_collision)
        totalSimTime = self.totalSimTime
        actionChangeFreq = totalSimTime/numOfTimeSlice
        hitTime = numOfNpc
        util.print_debug("= NPC speed and turning cmd ")
        util.print_debug(scenarioObj[0])
        for t in range(0, int(numOfTimeSlice)):
            #i = 0
            for npc in npcList:
                self.setNpcSpeed(npc, scenarioObj[0][t][0])
                turnCommand = scenarioObj[0][t][1]

                if turnCommand == 1:
                    direction = "LEFT"
                    self.setNpcChangeLane(npc, direction)
                elif turnCommand == 2:
                    direction = "RIGHT"
                    self.setNpcChangeLane(npc, direction)
                #i += 1

                # Stop if there is accident
                if self.isEgoFault == True or liability.isHitEdge(ego, sim, init_degree):
                   self.isHit = True
                   self.isEgoFault = True
                if self.isHit == True:
                   break
                for j in range(0, int(actionChangeFreq) * 2):
                    for npc in npcList:
                        util.print_debug("= current npc location")
                        util.print_debug(npc.state.transform.position)
                        util.print_debug("= current npc speed")
                        util.print_debug(npc.state.speed)
                        util.print_debug("= current npc angle")
                        util.print_debug(npc.state.transform.rotation)
                        util.print_debug("= current ego location")
                        util.print_debug(ego.state.transform.position)
                        util.print_debug("= current ego speed")
                        util.print_debug(ego.state.speed)
                        util.print_debug("= current ego angle")
                        util.print_debug(ego.state.transform.rotation)

                        sim.run(0.5)

        sim.close()
        return


##################################### MAIN ###################################
# Read scenario obj
# objPath = sys.argv[1]
# resPath = sys.argv[2]
# objPath = 'scenario.obj'
resPath = 'result.obj'
# print("asdad")
# print(str(objPath))
# print(resPath)
# objF = open(objPath, 'rb')
# print(objF)
# scenarioObj = pickle.load(objF)
# objF.close()
print("the simulation is  running")



i =1886

while i < 2000:
    #for i in range(1000):
        #print("== current is " +str(i)+" th experiment")
        util.print_debug("== current is " +str(i)+" th experiment")
        sim = LgApSimulation()
        resultDic = sim.runSimulation()
        i += 1



