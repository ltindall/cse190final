#!/usr/bin/env python
"""
This file was pulled from pa1 and modified to handle movement with walls. 
It also publishes to the /robot_location topic and the /found_goal topic. 
The /robot_location topic is used by the data transcriber to create the video 
of the robot moving through the map. The /robot_location topic is not subscribed 
to by the robot node in the notion that the robot should never know for certain 
where exactly it is, unless it is at the goal. The /found_goal topic 
publishes True when the robot has reached the goal. This is used by the robot node
to stop the simulation. 


"""
import rospy
import random as r
import math as m
import numpy as np
from std_msgs.msg import String, Float32, Bool
from copy import deepcopy
from cse_190_assi_4.srv import requestMapData, moveService
from read_config import read_config
from cse_190_assi_4.msg import RobotLocation

class MapServer():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()
        self.config["prob_move_correct"] = .75
        rospy.init_node("map_server")
        self.map_data_service = rospy.Service(
                "requestMapData",
                requestMapData,
                self.handle_data_request
        )
        self.move_service = rospy.Service(
                "moveService",
                moveService,
                self.handle_move_request
        )
	self.found_goal_publisher = rospy.Publisher(
		"/found_goal",
		Bool, 
		queue_size = 10
	)
	self.robot_location_publisher = rospy.Publisher(
		"/robot_location",
		RobotLocation, 
		queue_size = 10
	) 
        self.pos = self.initialize_position()
        r.seed(self.config['seed'])
        print "STARTING POSITION: ", self.pos
        rospy.spin()

    def initialize_position(self):
        """Set starting position."""
	start = [0,0]
	while (start in self.config['walls']):
	    start = [r.randint(1,self.config['map_size'][0]-2),r.randint(1,self.config['map_size'][1]-2)]
        #pos = self.config["starting_pos"]
	pos = start
        return pos

    def handle_data_request(self, request):
        """Service to provide ground truth data to sensors"""
        if request.data_type == "temp":
            temp = self.config['pipe_map'][self.pos[0]][self.pos[1]]
            return temp
        if request.data_type == "tex":
            tex = self.config['texture_map'][self.pos[0]][self.pos[1]]
            return tex

    def handle_move_request(self, request):
        """Service that moves the robot according to a move request.

        self.config['uncertain_motion'] determines the motion model
        used: either certain motion or motion with a set probability
        (randomotherwise).
        """
        move = list(request.move)
        if self.config['uncertain_motion']:
            roll = r.uniform(0,1)
            if roll < self.config["prob_move_correct"]:
                self.make_move(move)
		print "ROBOT MOVED CORRECTLY :)"
            else:
		print "ROBOT MOVED INCORRECTLY :(" 
                possible_moves = deepcopy(self.config['possible_moves'])
                possible_moves.remove(move)
                random_move = r.choice(possible_moves)
                self.make_move(random_move)
        elif not self.config['uncertain_motion']:
            self.make_move(move)
        return []

    def make_move(self, move):
        """Changes the robot's position"""
        num_rows = len(self.config['pipe_map'])
        num_cols = len(self.config['pipe_map'][0])
	if([self.pos[0]+move[0], self.pos[1]+move[1]] not in self.config['walls']): 
            self.pos[0] = (self.pos[0] + move[0]) % num_rows
            self.pos[1] = (self.pos[1] + move[1]) % num_cols
	self.robot_location_publisher.publish(self.pos)
	if([self.pos[0],self.pos[1]] == self.config['goal']): 
	    self.found_goal_publisher.publish(True)
        #print self.pos


if __name__ == '__main__':
    ms = MapServer()
