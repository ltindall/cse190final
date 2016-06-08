#!/usr/bin/env python

import rospy
from cse_190_assi_4.msg import RobotProbabilities,PolicyList,RobotLocation
from std_msgs.msg import String, Float32, Bool
import json
import image_util
from read_config import read_config


class RobotLogger():
    def __init__(self):
        rospy.init_node("robot_logger")
        self.temp_results_sub = rospy.Subscriber(
                "/results/temperature_data",
                Float32,
                self.handle_incoming_temperature_data
        )
        self.tex_results_sub = rospy.Subscriber(
                "/results/texture_data",
                String,
                self.handle_incoming_texture_data
        )
        self.prob_results_sub = rospy.Subscriber(
                "/results/probabilities",
                RobotProbabilities,
                self.handle_incoming_probabilities
        )
        """
        self.path_result = rospy.Subscriber(
                "/results/path_list",
                AStarPath,
                self.handle_a_star_algorithm_path
        )
	"""        
        self.robot_location = rospy.Subscriber(
                "/robot_location",
                RobotLocation,
                self.handle_robot_location
        ) 
        self.policy_result = rospy.Subscriber(
                "/results/policy_list",
                PolicyList,
                self.handle_mdp_policy_data
        ) 

        self.simulation_complete_sub = rospy.Subscriber(
                "/map_node/sim_complete",
                Bool,
                self.handle_shutdown
        )
        self.init_files()
        self.config = read_config()
        self.generate_video = self.config["generate_video"] == 1 
	self.current_robot_location = []
	self.robot_location = self.config["starting_pos"]
	self.policy_list = []
        rospy.spin()

    def init_files(self):
        with open('texture_results.json', 'w+') as infile:
            pass
        open('temperature_results.json', 'w+').close()
        open('probability_results.json', 'w+').close()
        self.texture_data = []
        self.temperature_data = []
        self.probability_data = []

	
        #open('path_list.json', 'w+').close()
        open('policy_list.json', 'w+').close()

        self.policy_list = []
        #self.path_list = []
        self.iteration_number = 0
	
    def handle_robot_location(self,message): 
	self.robot_location = list(message.data)  
        if self.generate_video:
            data_to_publish = self.convert_list_to_2d_array(self.policy_list[-1])
            image_util.save_image_for_iteration(data_to_publish, self.iteration_number,self.robot_location)
            self.iteration_number += 1

    def handle_incoming_texture_data(self, message):
        texture = message.data
        self.texture_data.append(texture)

    def handle_incoming_temperature_data(self, message):
        temperature = message.data
        self.temperature_data.append(temperature)

    def handle_incoming_probabilities(self, message):
        probabilities_list = [round(x, 5) for x in message.data]
        self.probability_data.append(probabilities_list)

    
    
    def convert_list_to_2d_array(self, policy_list):
        x, y = self.config["map_size"]
        return [policy_list[i : i + y] for i in xrange(0, len(policy_list), y)]

    def handle_mdp_policy_data(self, policy_list):
        self.policy_list.append(policy_list.data)
	"""
        if self.generate_video:
            data_to_publish = self.convert_list_to_2d_array(policy_list.data)
            image_util.save_image_for_iteration(data_to_publish, self.iteration_number,self.robot_location)
            self.iteration_number += 1
	"""

    def handle_a_star_algorithm_path(self, path_list):
        self.path_list.append(path_list.data)
    



    def handle_shutdown(self, message):
        print "sim complete!", message.data

        if message.data:
            with open('texture_results.json', 'w') as tex:
                json.dump(self.texture_data, tex)
            with open('temperature_results.json', 'w') as temp:
                json.dump(self.temperature_data, temp)
            with open('probability_results.json', 'w') as prob:
                json.dump(self.probability_data, prob)

        
        if self.generate_video:
            image_util.generate_video(self.iteration_number)
        if message.data:
            #with open('path_list.json', 'w') as path:
                #Saving the entire path to be confirmed
            #    json.dump(self.path_list, path)
            with open('policy_list.json', 'w') as policy:
                #Saving only the last policy to be compared
                json.dump(self.policy_list[-1], policy)
	

if __name__ == '__main__':
    rl = RobotLogger()
