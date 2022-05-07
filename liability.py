import os
import lgsvl
import sys
import time
import math
import random
import pickle
import inspect
import pprint
import sympy
import util
from sympy import Point3D, Line3D, Segment3D, Point2D, Line2D, Segment2D

#Determine who is liable based on safety violation constraints in global coordinate system of LGSVL.

def isHitEdge(ego, sim, init_degree):
	# init_degree = ego.state.rotation.y
	lane_center = sim.map_point_on_lane(ego.state.transform.position)
	#print("myline center")
	#print(lane_center)

	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	ego_point = Point3D(ego_x, ego_y, ego_z)

	mp_x = lane_center.position.x
	mp_y = lane_center.position.y
	mp_z = lane_center.position.z
	mp_point = Point3D(mp_x, mp_y, mp_z)

	#x1, y1, z1 = 160.809997558594, 10.1931667327881, 8.11004638671875
	#lgsvl.Vector(7.44000864028931, 0.0, -68.2100067138672)
	x1,y1,z1 = 9.6, 0.0, -10.2100067138672
	x_e_1, y_e_1, z_e_1 = 9.6, 0.0, -50.2100067138672
	x6, y6, z6 =-0.45, 0.0, -57.77267646789551
	x_e_6, y_e_6, z_e_6 = -0.45, 0.0, -3.77213048934937
	l1 = Line3D(Point3D(x1, y1, z1), Point3D(x_e_1, y_e_1, z_e_1))
	l6 = Line3D(Point3D(x6, y6, z6), Point3D(x_e_6, y_e_6, z_e_6))

	diagnal_length = pow(ego.bounding_box.size.z, 2) + pow(ego.bounding_box.size.x, 2)
	diagnal_length = math.sqrt(diagnal_length)
	rotate_degree = abs(ego.state.rotation.y - init_degree) + 23.86
	ego_size_z = (diagnal_length / 2.0) * math.sin(math.radians(rotate_degree))

	if (l1.distance(mp_point) <= 1):
		lane_bound = mp_z + 2.2
		if (ego.state.transform.position.z + ego_size_z >= lane_bound ):
			util.print_debug("--- Cross the boundary --- ")
			return True
	if(l6.distance(mp_point) <= 1):
		lane_bound = mp_z + 2.14
		if (ego.state.transform.position.z + ego_size_z >= lane_bound):
			util.print_debug(" --- Cross the yellow line")
			return True
	return False

def isHitYellowLine(ego, sim, init_degree):
	lane_center = sim.map_point_on_lane(ego.state.transform.position)

	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	ego_point = Point3D(ego_x, ego_y, ego_z)
	mp_x = lane_center.position.x
	mp_y = lane_center.position.y
	mp_z = lane_center.position.z
	mp_point = Point3D(mp_x, mp_y, mp_z)

	x1,y1,z1 = 8.44000864028931, 0.0, -10.2100067138672
	x_e_1, y_e_1, z_e_1 = 8.44000864028931, 0.0, -50.2100067138672
	x6, y6, z6 = 2.84000864028931, 0.0, -57.77267646789551
	x_e_6, y_e_6, z_e_6 = 2.84000864028931, 0.0, -3.77213048934937

	l1 = Line3D(Point3D(x1, y1, z1), Point3D(x_e_1, y_e_1, z_e_1))
	l6 = Line3D(Point3D(x6, y6, z6), Point3D(x_e_6, y_e_6, z_e_6))

	diagnal_length = pow(ego.bounding_box.size.z, 2) + pow(ego.bounding_box.size.x, 2)
	diagnal_length = math.sqrt(diagnal_length)
	rotate_degree = abs(ego.state.rotation.y - init_degree) + 23.86
	ego_size_z = (diagnal_length / 2.0) * math.sin(math.radians(rotate_degree))

	if (l1.distance(mp_point) <= 1):
		lane_bound = mp_z + 2.2
		if (ego.state.transform.position.z + ego_size_z >= lane_bound ):
			util.print_debug("--- Cross the boundary --- ")
			return True
	if(l6.distance(mp_point) <= 1):
		lane_bound = mp_z + 2.14
		if (ego.state.transform.position.z + ego_size_z >= lane_bound):
			util.print_debug(" --- Cross the yellow line")
			return True
	return False


def isCrossedLine(ego, sim, init_degree):
	lane_center = sim.map_point_on_lane(ego.state.transform.position)
	print(lane_center)
	right_z = lane_center.position.x - 2.34
	left_z = lane_center.position.x + 2.34

	rotate_degree = abs(ego.state.rotation.y - init_degree) + 23.86
	diagnal_length = pow(ego.bounding_box.size.z, 2) + pow(ego.bounding_box.size.x, 2)
	diagnal_length = math.sqrt(diagnal_length)
	print("diagonal length")
	print(diagnal_length)
	ego_size_z = (diagnal_length / 2.0) * math.sin(math.radians(rotate_degree))
	print(ego_size_z)
	if not (
			ego.state.transform.position.x + ego_size_z < left_z and ego.state.transform.position.x - ego_size_z > right_z):
		print(" === Ego cross line === ")
		return True
	else:
		return False

	return False

def debugPos(ego, npc):
	egoRotation = ego.state.rotation.y
	npcRotation = npc.state.rotation.y
	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	npc_x = npc.state.transform.position.x
	npc_y = npc.state.transform.position.y
	npc_z = npc.state.transform.position.z
	print(" ^^^^^^^^ Ego Rotation: " + str(egoRotation) + ", NPC rotation: " + str(npcRotation) + " ^^^^^^^")
	print("Ego: " + str(ego_x) + ", " + str(ego_y) + ", " + str(ego_z))
	print("NPC: " + str(npc_x) + ", " + str(npc_y) + ", " + str(npc_z))


def findDistance(ego, npc):
	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	npc_x = npc.state.transform.position.x
	npc_y = npc.state.transform.position.y
	npc_z = npc.state.transform.position.z
	dis = math.pow(npc_x - ego_x , 2) + math.pow(npc_y - ego_y, 2) + math.pow(npc_z - ego_z, 2)
	dis = math.sqrt(dis)
	return dis

def isEgoFault(ego, npc, sim, init_degree):
	if npc is None:
		return True

	isCrossed = isCrossedLine(ego, sim, init_degree)
	ego_x = ego.state.transform.position.x
	ego_y = ego.state.transform.position.y
	ego_z = ego.state.transform.position.z
	#print("ego-z")
	#print(ego_z)
	npc_x = npc.state.transform.position.x

	npc_y = npc.state.transform.position.y
	npc_z = npc.state.transform.position.z
	#print("npc-z")
	#print(npc_z)
	debugPos(ego, npc)

	if (isHitYellowLine(ego, sim, init_degree)):
		return False

	# Longitudinal hit
	if isCrossed == True:
		if ego_z - 4.3 > npc_z and (npc.state.rotation.y < 271 and npc.state.rotation.y > 269):

			util.print_debug(" --- Ego cross , NPC is behind, NPC FAULT --- ")
			return False
		else:
			util.print_debug(" --- Ego cross, side collision or front colision to NPC, EGO FAULT --- ")
			return True
	else:
		# if ego_x - 4.3 > npc_x and npc_z > ego_z - 2 and npc_z < ego_z + 2:
		if ego_z + 4.3 <  npc_z and (npc.state.rotation.y > 271 or npc.state.rotation.y < 269):
			# NPC is in front
			util.print_debug(" --- Ego stays in line , NPC is in front, EGO FAULT --- ")
			return True
		else:
			util.print_debug(" --- Ego stays in line, side or rear collision to EGO, NPC FAULT --- ")
			return False

