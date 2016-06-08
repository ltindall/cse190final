#!/usr/bin/env python
'''
Author: Lucas Tindall

Publisher on temp senser activation topic

Subsciber on temp sensor data topic

Sensor proxy on texture sensor service

Sensor proxy on map move service

Calculate position

Publish position, temp and texture

Check if robot has made all the moves in move list

If so publish to sim_complete 
And shutdown 
'''
from __future__ import division
import rospy
import itertools
import random as r
import math as m
import numpy as np
import image_util
from copy import deepcopy
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_4.msg import temperatureMessage, RobotProbabilities, PolicyList, RobotLocation
from cse_190_assi_4.srv import requestMapData, moveService, requestTexture
from read_config import read_config
from mdp import mdp 
from util import print_2d_floats



class robot():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()

        #self.config["prob_move_correct"] = .75
        rospy.init_node("robot")
	self.temperature_sensor_activation = rospy.Publisher(
                "/temp_sensor/activation",
                Bool,
                queue_size = 10
        )
	self.temperature_sensor_data = rospy.Subscriber(
                "/temp_sensor/data",
                temperatureMessage,
                self.handle_temperature_sensor_data
        )
	self.robot_texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )
	self.robot_move_requester = rospy.ServiceProxy(
                "moveService",
                moveService
        )
	self.position_publisher = rospy.Publisher(
                "/results/probabilities",
                RobotProbabilities,
                queue_size = 10
        )
	self.temperature_publisher = rospy.Publisher(
                "/results/temperature_data",
                Float32,
                queue_size = 10
        )
	self.texture_publisher = rospy.Publisher(
                "/results/texture_data",
                String,
                queue_size = 10
        )
	self.simulation_complete_publisher = rospy.Publisher(
                "/map_node/sim_complete",
                Bool,
                queue_size = 10
        )
	self.temperature_requester = rospy.ServiceProxy(
                "requestMapData",
                requestMapData
        )
        self.policy_publisher = rospy.Publisher(
            "/results/policy_list",
            PolicyList,
            queue_size = 10
        )
	self.found_goal_subscriber = rospy.Subscriber(
                "/found_goal",
                Bool,
                self.handle_found_goal
        )
	self.robot_location = rospy.Subscriber(
                "/robot_location",
                RobotLocation,
                self.handle_robot_location
        )

	self.temp_dict = {
                'H': 40.0,
                'C': 20.0,
                '-': 25.0
        }
	self.policy_dict = {
		'N': [-1,0], 
		'S': [1,0], 
		'E': [0,1], 
		'W': [0,-1],
		'GOAL':[0,0]
	}
	
	# wait for publishers to be created 
	rospy.sleep(3)
	
	self.foundGoal = False 
	self.iteration_number = 0 
	self.current_robot_location = []
	######################
	# SETUP FOR PLATH PLANNING (MDP PA 3) 
 	########################

	map_size = self.config['map_size']
        rows = map_size[0]
        columns = map_size[1]
        goal = self.config['goal']
        walls = self.config['walls']
        pits = self.config['pits']
        move_list = self.config['mdp_move_list']


        oldV = [[0 for x in range(columns)] for y in range(rows)]

        # initialize V values 
        #oldV = [[0 for x in range(columns)] for y in range(rows)]              
        for i in range(rows):
            for j in range(columns):
                tempGrid = [i,j]
                if tempGrid == goal:
                    oldV[i][j] = self.config['reward_for_reaching_goal']
                elif tempGrid in walls:
                    oldV[i][j] = self.config['reward_for_hitting_wall']
                elif tempGrid in pits:
                    oldV[i][j] = self.config['reward_for_falling_in_pit']
                else:
                    #this is redundant 
                    oldV[i][j] = 0

        stoppingSum = 0
        firstThreshold = False
        secondThreshold = False
        firstX = 0
        secondX = -100
	
	self.policiesFlat = []
	for x in range(self.config['max_iterations']):
            mdpPolicy = mdp(self.config,oldV)
            self.policiesFlat = mdpPolicy.policies
            self.policy_publisher.publish(self.policiesFlat)
            for i in range(rows):
                for j in range(columns):
                    stoppingSum += abs(oldV[i][j]-mdpPolicy.newV[i][j])
            if(stoppingSum < self.config['threshold_difference']):
                secondX = x
            if(secondX - 1 == firstX ):
                break
            if(stoppingSum < self.config['threshold_difference'] ):
                firstX = x
                firstThreshold = True

            oldV = mdpPolicy.newV

	i = 0 
	self.policies = []
	while i < (rows*columns): 
	    self.policies.append(self.policiesFlat[i:i+columns])
	    i += columns 
	
	#print "PRINTING POLICIES = ",self.policies
	rospy.sleep(5)

	####################
	# SETUP FOR LOCALIZATION (PA 1) 
	####################

	#self.temp_map = list(itertools.chain.from_iterable(self.config['pipe_map']))
	self.temp_map = self.config['pipe_map']
	#print "temp_map", self.temp_map
	#print "temp_map[0][0]", self.temp_map[0][0]
	#print self.config['pipe_map']
	#print self.config['texture_map']
        #print "temp_map[0][0]",self.config['pipe_map'][0][0]
	#self.tex_map = list(itertools.chain.from_iterable(self.config['texture_map']))
	self.tex_map = self.config['texture_map']
	self.std_dev = self.config['temp_noise_std_dev'] 
	self.move_index = 0 
	self.moves = self.config['move_list']
	
        self.num_rows = len(self.config['pipe_map'])
        self.num_cols = len(self.config['pipe_map'][0])
	
	self.positions = []
	self.numValidSquares = (self.num_rows*self.num_cols) - (2*self.num_rows + 2*(self.num_cols-2))
	#print "numValidSquares: ",self.numValidSquares
	for i in range(self.num_rows):
	    rowPositions = []
	    for j in range(self.num_cols):  
		if(i == 0 or i == self.num_rows-1 or j == 0 or j == self.num_cols -1 ): 
		    rowPositions.append(0)
		else: 
	    	    rowPositions.append(1/self.numValidSquares)
	    self.positions.append(rowPositions)
	#print "self.positions: ",self.positions
	#print "initial positions ",self.positions
	rospy.sleep(2)
	self.processing = False 
	self.temperature_sensor_activation.publish(True)
        
        rospy.spin()

    def handle_robot_location(self,message):
        self.current_robot_location = message.data

    def handle_found_goal(self,message): 
	self.foundGoal = True 


    def convert_list_to_2d_array(self, policy_list):
        x, y = self.config["map_size"]
        return [policy_list[i : i + y] for i in xrange(0, len(policy_list), y)]

    def handle_temperature_sensor_data(self, message): 
    	"""
	Callback function for the temperature sensor data subscriber 

	Each time the robot receives new data from the temperature sensor
	it should request data from the texture sensor using the ServiceProxy
   	robot_texture_requester and then move the robot using the 
	ServiceProxy robot_move_requester. 
    	"""
	while(self.processing != False): 
	    pass
	#rospy.sleep(2)
	self.process(message)

    def process(self,message): 
	self.processing = True
	# Get sensor readings
 	temperature = message.temperature
	texture_response = self.robot_texture_requester()
	texture = texture_response.data
	
	# Prepare to update position 
	temporary_temp_pos = [] 
	normalization_constant = 0.0

	#print "TEMP ",temperature
	# Calculate position based on temperature sensor  
	for i in range(self.num_rows): 
	    tempRow = []
	    for j in range(self.num_cols):
		if([i,j] in self.config['walls']): 
		    tempRow.append(0)
		else: 
		    #print "problem ", self.temp_map[i][j]
	            expected_temp = self.temp_dict[self.temp_map[i][j]]
	            tempRow.append(((1/(self.std_dev * m.sqrt(2*m.pi))) *
			 m.pow(m.e,-1*(m.pow((temperature-expected_temp),2))/
	                 (2*m.pow(self.std_dev,2))))*self.positions[i][j])
	        normalization_constant += tempRow[j]
	    temporary_temp_pos.append(tempRow)
	for i in range(self.num_rows):
	    for j in range(self.num_cols):  
	        self.positions[i][j]=(temporary_temp_pos[i][j]/normalization_constant)
	#print "normalized positions: ",self.positions
	#print "Temperature ",temperature	
	self.temperature_publisher.publish(temperature) 
	#rospy.sleep(2)

	# Calculate position based on texture sensor
	normalization_constant = 0.0
	temporary_tex_pos = []

	for i in range(self.num_rows):
	    tempTexRow = [] 
	    for j in range(self.num_cols): 
		if([i,j] in self.config['walls']): 
		    tempTexRow.append(0)
		else: 
	            expected_tex = self.tex_map[i][j]
	            if (texture == expected_tex): 
	                prob_tex_given_x = self.config['prob_tex_correct']
                    else: 
		        prob_tex_given_x = 1-self.config['prob_tex_correct'] 
	            tempTexRow.append(prob_tex_given_x * self.positions[i][j] )
                normalization_constant += tempTexRow[j]
	    temporary_tex_pos.append(tempTexRow)
	for i in range(self.num_rows): 
	    for j in range(self.num_cols): 
	        self.positions[i][j] = (temporary_tex_pos[i][j]/normalization_constant)

	#print "newer positions: ",self.positions
	#print "Texture ",texture
	self.texture_publisher.publish(texture)
	#rospy.sleep(2)
	
  	"""
	i = 0
	formated_positions = []
	print "move index: ",self.move_index
        print "positions: ",self.positions
	while i < len(self.positions):
	    formated_positions.append(self.positions[i:i+self.num_cols])
	    #print self.positions[i:i+self.num_cols]
	    i += self.num_cols
	"""
        #formated_positions_copy = deepcopy(formated_positions)		
	formated_positions_copy = deepcopy(self.positions)
	#print "formatted: ",formated_positions_copy

	##################
	# End of recomputing position based on sensors 
        ##################
	
	#################
	# FIND POSITION WITH HIGHEST PROBABILITY 
	# AND MOVE ACCORDING TO THE POLICY FOR THAT SQUARE 
	######################
	maxProb = 0
	maxProbIndex = []
	for i in range(self.num_rows): 
	    for j in range(self.num_cols): 
	        if(self.positions[i][j] > maxProb): 
		    maxProb = self.positions[i][j]
		    maxProbIndex = [i,j]
	#########################################
	print "ROBOT HAS HIGHEST LIKELIHOOD AT INDEX: ",maxProbIndex
        print "HIGHEST LIKELIHOOD: ",maxProb


	"""
	# check if finished moves and send shutdown
        if self.move_index == len(self.moves):
	    flattened_positions = list(itertools.chain.from_iterable(formated_positions_copy))
	    #print "Positions ",flattened_positions
	    self.position_publisher.publish(flattened_positions) 
            self.simulation_complete_publisher.publish(True)
	    rospy.sleep(5)
    	    rospy.signal_shutdown("Completed all moves, shutting down")
	"""
	##########################
	# MOVE ROBOT 
	###########################
	print "OPTIMAL MOVE: ",self.policies[maxProbIndex[0]][maxProbIndex[1]]
        self.robot_move_requester(self.policy_dict[self.policies[maxProbIndex[0]][maxProbIndex[1]]]) 
        #print "Move ",self.moves[self.move_index]

	###########################
	# CREATE IMAGE FOR VIDEO 
	##############################
	"""
	flattened_policies = list(itertools.chain.from_iterable(self.policies))
	data_to_publish = self.convert_list_to_2d_array(flattened_policies)
	print "data_to_publish ",data_to_publish
        image_util.save_image_for_iteration(data_to_publish, self.iteration_number)
        self.iteration_number += 1
	"""


	#########################
	# IF ROBOT IS AT GOAL THEN STOP SIMULATION
	# *ASSUME ROBOT HAS SOME SPECIAL SENSOR WHICH ACTIVATES  
	# WHEN IT ARRIVES AT THE GOAL 
	#################################	
	if(self.foundGoal == True): 
	    #image_util.generate_video(self.iteration_number)
	    self.simulation_complete_publisher.publish(True)
	    rospy.sleep(5)
    	    rospy.signal_shutdown("Completed all moves, shutting down")
		
	
	# Calculate position based on movement  
	expected_move = self.policy_dict[self.policies[maxProbIndex[0]][maxProbIndex[1]]]
	for i in range(self.num_rows): 
	    for j in range(self.num_cols): 
	        formated_positions_copy[i][j] = 0
		for move in self.config['possible_moves']:
		    if expected_move == move: 
			prob_move = self.config['prob_move_correct']
		    else: 
			prob_move = (1-self.config['prob_move_correct'])/(
				    len(self.config['possible_moves'])-1)
		    if([i,j] not in self.config['walls']):
		        formated_positions_copy[i][j] += self.positions[(i-move[0])%self.num_rows][(j-move[1])%self.num_cols] * prob_move
	
	flattened_positions = list(itertools.chain.from_iterable(formated_positions_copy))
        self.positions = formated_positions_copy
	#print "FINAL POSITIONS: ",self.positions	
	print_2d_floats(self.positions)
	#print "Positions ",flattened_positions
	self.position_publisher.publish(flattened_positions) 

	#publish temp.tex from square 0, move, publish move, publish temp,tex check possible move	
	
        self.move_index += 1 
	self.processing = False
	
	
if __name__ == '__main__':
    robo = robot()
