#! /usr/bin/env python
"""
Author: Lucas Tindall
"""

# mdp implementation needs to go here
class mdp():
    def __init__(self,config,oldV):
        self.config = config
        map_size = self.config['map_size']
        rows = map_size[0]
        columns = map_size[1]
        goal = self.config['goal']
        walls = self.config['walls']
        pits = self.config['pits']
        move_list = self.config['mdp_move_list']

        # initialize V values 
        #oldV = [[0 for x in range(columns)] for y in range(rows)]      
        """
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
        """
        self.policies = [["" for x in range(columns)] for y in range(rows)]
 
    #for x in range(self.config['max_iterations']): 
 
        # make list where index corresponds to direction/a (up,right,down,left)
        # for each direction/a(really stands for step) find the sum of the next state equation 
        # ###V(k+1)(S) = sum over next possible state {P(S'|S,a){R(S,a,S')+Y Vk(S')}}
        # get max from list: alist.index(max(alist))
 
    
        self.newV = [[0 for x in range(columns)] for y in range(rows)]
    
        for i in range(rows):
            for j in range(columns):

                if [i,j] in walls:
                    self.newV[i][j] = oldV[i][j]
                    self.policies[i][j] = "WALL"
                    continue
                elif [i,j] in pits:
                    self.newV[i][j] = oldV[i][j]
                    self.policies[i][j] = "PIT"
                    continue
                elif [i,j] == goal:
                    self.newV[i][j] = oldV[i][j]
                    self.policies[i][j] = "GOAL"
                    continue

                # newV[i][j] = 
                aMove = []
                moveIndex = 0
                moveStrings = []
                for move in move_list:
                    if move == [1,0]:
                        moveStrings.append("S")
                    elif move == [0,1]:
                        moveStrings.append("E")
                    elif move == [-1,0]:
                        moveStrings.append("N")
                    else:
                        moveStrings.append("W")

		    sPrimeSum = 0

		    for nextState in move_list:
			if nextState == move :
			    probNextState = self.config['prob_move_forward']
			elif abs(move[0]) == abs(nextState[0]):
			    probNextState = self.config['prob_move_backward']
			elif ((abs(move[0]) == 1 and move[0] == nextState[1]) or
			    (move[0] == 0 and abs(move[1]) == abs(nextState[0]))):
			    probNextState = self.config['prob_move_right']
			else:
			    probNextState = self.config['prob_move_left']

			newLocation = [i+nextState[0], j+nextState[1]]
			if newLocation == goal:
			    reward = (self.config['reward_for_reaching_goal'] +
			      self.config['reward_for_each_step'])
			elif ((newLocation in walls) or (newLocation[0] < 0) or
					     (newLocation[1] < 0) or (newLocation[0] >= rows) or
			    (newLocation[1] >= columns)):
			    reward = self.config['reward_for_hitting_wall']
			    newLocation = [i,j]
			elif newLocation in pits:
			    reward = self.config['reward_for_falling_in_pit'] + self.config['reward_for_each_step']
			else:
			    reward = self.config['reward_for_each_step']
			sPrimeSum += probNextState * (reward + (self.config['discount_factor'] * oldV[newLocation[0]][newLocation[1]]))

		    aMove.append(sPrimeSum)
		    moveIndex += 1

                self.newV[i][j] = max(aMove)
                self.policies[i][j] = moveStrings[aMove.index(max(aMove))]
        #print self.policies
        #oldV = newV
        self.policies = [policy for temp in self.policies for policy in temp]

