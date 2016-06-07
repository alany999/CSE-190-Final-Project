#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_final_project.srv import requestMapData, requestTexture, moveService
from cse_190_final_project.msg import temperatureMessage, RobotProbabilities
from std_msgs.msg import String
from std_msgs.msg import Bool, Float32
from read_config import read_config


class Robot():
    pos_prob = []
    moveList = []
    rows = 0
    cols = 0
    
    def __init__(self):
        global rows
        global cols
        global pos_prob
	global moveList

	rospy.init_node("robot")
        """Read config file and setup ROS things"""
        self.config = read_config()
	#print "Initializing robot"
	moveList = deepcopy(self.config['move_list'])

        rows = len(self.config['pipe_map'])
        cols = len(self.config['pipe_map'][0])
        numPositions = rows*cols
        pos_prob = [[float(1/(numPositions*1.0))]*
                cols for _ in range(rows)]
        #print "Initial Position probabilities"
        #print pos_prob
       
        #numS = sum(row.count("S") for row in self.config['texture_map'])
        #numR = sum(row.count("R") for row in self.config['texture_map'])

        # Wait until the nodes are ready

	self.temp_sub = rospy.Subscriber(
		"/temp_sensor/data", 
		temperatureMessage, 
		self.getTexture
	)
	
	self.activator = rospy.Publisher(
		"/temp_sensor/activation",
		Bool, 
		queue_size = 10
	)

	self.prob_pub = rospy.Publisher(
		"/results/probabilities",
		RobotProbabilities, 
		queue_size = 10
	)

	self.temp_pub = rospy.Publisher(
		"/results/temperature_data",
		Float32, 
		queue_size = 10
	)

	self.tex_pub = rospy.Publisher(
		"/results/texture_data",
		String, 
		queue_size = 10
	)

	self.sim_complete = rospy.Publisher(
		"/map_node/sim_complete",
		Bool, 
		queue_size = 10
	)

        rospy.sleep(1)
	self.activator.publish(True)

	rospy.spin()

    def getTexture(self, temp):
	    # START OF TEMP PROBABILITY
	    global pos_prob
	    ##print "temp: " + str(temp.temperature)

	    #Normalization factor
	    temp_prob_sum = 0.0
	    for rowIndex in range(rows):
		for colIndex in range(cols):
			sigma = self.config['temp_noise_std_dev']
			num1 = 1.0 / (sigma * m.sqrt(2.0*m.pi))

			#Calculate mu value
			current_temp = self.config['pipe_map'][rowIndex][colIndex]
			if current_temp == 'H':
				mu = 40.0
			elif current_temp == '-':
				mu = 25.0
			else:
				mu = 20.0

			#Calculate the probability P(X|temp) * P(X)
			temp1 = m.pow( (temp.temperature - mu), 2.0) * -1.0
			power_e = temp1 / (2.0 * pow(sigma, 2.0))
			prob = num1 * m.pow(m.e, power_e)
			pos_prob[rowIndex][colIndex]=prob*pos_prob[rowIndex][colIndex]
			temp_prob_sum = temp_prob_sum + (pos_prob[rowIndex][colIndex])

	    #Normalize the temperature
	    for rowIndex in range(rows):
		for colIndex in range(cols):
			pos_prob[rowIndex][colIndex] = pos_prob[rowIndex][colIndex] / temp_prob_sum	

	    #print "temparature prob:"
	    #print pos_prob
			 



	    #START OF TEXTURE PROBABILITY
            #self.config = read_config()
	    texture = rospy.ServiceProxy("requestTexture", requestTexture)

            #Update probability based on texture
	    prob_tex_correct = self.config["prob_tex_correct"]
	    tex_data = texture().data
	    #print "prob texture is correct: " + str(prob_tex_correct)
	    tex_prob_sum = 0.0
            for rowIndex in range(rows):
                for colIndex in range(cols):
                    if (tex_data == self.config['texture_map'][rowIndex][colIndex]):
                        pos_prob[rowIndex][colIndex] = (prob_tex_correct*pos_prob[rowIndex][colIndex])
			tex_prob_sum = tex_prob_sum + pos_prob[rowIndex][colIndex]
                    else:
                        pos_prob[rowIndex][colIndex] = ((1.0 - prob_tex_correct)*pos_prob[rowIndex][colIndex])
			tex_prob_sum = tex_prob_sum + pos_prob[rowIndex][colIndex]

	    ##print "tex: " + str(tex_data)
	
	    #Normalize the probability
	    for rowIndex in range(rows):
		for colIndex in range(cols):
			pos_prob[rowIndex][colIndex] = pos_prob[rowIndex][colIndex] / tex_prob_sum

	    #print pos_prob

	    self.moveRobot(temp.temperature, tex_data)
	
    def moveRobot(self, temp, tex):
            global pos_prob

            prob_move_correct = self.config["prob_move_correct"]
	    move = rospy.ServiceProxy("moveService", moveService)
	    #print "Available moves are: "
	    #print moveList

            #Should allow for the robot to sense, then shutdown (need to double check)
            #When all moves are made, exit
	    if len(moveList) == 0:
		#print pos_prob
		flat_pos_prob = [x for sublist in pos_prob for x in sublist]
	    	self.prob_pub.publish(flat_pos_prob)
	    	self.temp_pub.publish(temp)
	    	self.tex_pub.publish(tex)
                rospy.sleep(1)
		self.sim_complete.publish(True)
                self.activator.publish(False)
                rospy.sleep(1)
		rospy.signal_shutdown("Finished")   

	    curr_move = moveList.pop(0)
	    #print curr_move

	    ##print "move: " + str(curr_move)


	    #Move robot
	    move(curr_move)
	    
	    x_move = curr_move[0]
	    y_move = curr_move[1]

            new_pos_prob = [[0.0]*
                cols for _ in range(rows)] #Make a empty copy of the map
 
	    #Need to calculate new probability based on move
	    
	    for rowIndex in range(rows):
                for colIndex in range(cols):
                    curr_prob = new_pos_prob[rowIndex][colIndex]
                    #new_pos_prob[rowIndex+x_move % rows][colIndex+y_move % cols] = 
                    if (x_move == 1 and y_move == 0):
                        new_pos_prob[(rowIndex+1)%rows][(colIndex)%cols] += prob_move_correct*pos_prob[(rowIndex)][(colIndex)] #Moved correct
                        new_pos_prob[(rowIndex-1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex+1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex-1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                    elif (x_move == -1 and y_move == 0):
                        new_pos_prob[(rowIndex-1)%rows][(colIndex)%cols] += prob_move_correct*pos_prob[(rowIndex)][(colIndex)] #Moved correct
                        new_pos_prob[(rowIndex+1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex+1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex-1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect

                    elif (x_move == 0 and y_move == 1):
                        new_pos_prob[(rowIndex)%rows][(colIndex+1)%cols] += prob_move_correct*pos_prob[(rowIndex)][(colIndex)] #Moved correct
                        new_pos_prob[(rowIndex+1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex-1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex-1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect

                    elif (x_move == 0 and y_move == -1):
                        new_pos_prob[(rowIndex)%rows][(colIndex-1)%cols] += prob_move_correct*pos_prob[(rowIndex)][(colIndex)] #Moved correct
                        new_pos_prob[(rowIndex+1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex+1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex-1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect

                    elif (x_move == 0 and y_move == 0):
                        new_pos_prob[(rowIndex)%rows][(colIndex)%cols] += prob_move_correct*pos_prob[(rowIndex)][(colIndex)] #Moved correct
                        new_pos_prob[(rowIndex+1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex-1)%rows][(colIndex)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex+1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect
                        new_pos_prob[(rowIndex)%rows][(colIndex-1)%cols] += ((1.0-prob_move_correct)/4.0)*pos_prob[(rowIndex)][(colIndex)] #Move incorrect

            
            #Add up probabilities for normalization
            #move_prob_sum = 0.0
            #for rowIndex in range(rows):
	    #    for colIndex in range(cols):
	    #	    move_prob_sum += new_pos_prob[rowIndex][colIndex]

            #Normalize the probability
	    #for rowIndex in range(rows):
	    #	for colIndex in range(cols):
	    #		new_pos_prob[rowIndex][colIndex] = new_pos_prob[rowIndex][colIndex] / move_prob_sum

            pos_prob = deepcopy(new_pos_prob)
            
            #print "Move prob"
            #print pos_prob

	    flat_pos_prob = [x for sublist in pos_prob for x in sublist]
	    self.prob_pub.publish(flat_pos_prob)
	    self.temp_pub.publish(temp)
	    self.tex_pub.publish(tex)
			
            '''
	    #When all moves are made, exit
	    if len(moveList) == 0:
		self.sim_complete.publish(True)
		self.activator.publish(False)
		rospy.sleep(1)
		rospy.signal_shutdown("Finished")
                '''


if __name__ == '__main__':
	Rb = Robot()
