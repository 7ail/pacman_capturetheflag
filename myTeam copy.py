# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game
from collections import namedtuple
from time import time
#from capture import GameState


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='RealAgent', second='DummyAgent'):
    """
    This function should return a list of two agents that will form the
    team, initialized using firstIndex and secondIndex as their agent
    index numbers.  isRed is True if the red team is being created, and
    will be False if the blue team is being created.

    As a potentially helpful development aid, this function can take
    additional string-valued keyword arguments ("first" and "second" are
    such arguments in the case of this function), which will come from
    the --redOpts and --blueOpts command-line arguments to capture.py.
    For the nightly contest, however, your team will be created without
    any extra arguments, so you should make sure that the default
    behavior is what you want for the nightly contest.
    """

    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class DummyAgent(CaptureAgent):
    """
    A Dummy agent to serve as an example of the necessary agent structure.
    You should look at baselineTeam.py for more details about how to
    create an agent as this is the bare minimum.
    """

    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the
        agent to populate useful fields (such as what team
        we're on).

        A distanceCalculator instance caches the maze distances
        between each pair of positions, so your agents can use:
        self.distancer.getDistance(p1, p2)

        IMPORTANT: This method may run for at most 15 seconds.
        """

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)

        '''
        Your initialization code goes here, if you need any.
        '''

    def chooseAction(self, gameState):
        """
        Picks among actions randomly.
        """
        actions = gameState.getLegalActions(self.index)

        '''
        You should change this in your own agent.
        '''

        return random.choice(actions)


class TestAgent(CaptureAgent):
    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the
        agent to populate useful fields (such as what team
        we're on).

        A distanceCalculator instance caches the maze distances
        between each pair of positions, so your agents can use:
        self.distancer.getDistance(p1, p2)

        IMPORTANT: This method may run for at most 15 seconds.
        """

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)


    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        myState = gameState.getAgentState(self.index)

        otherState=gameState.getAgentState(self.getOpponents(gameState)[0])
        pos = otherState.getPosition()
        print(pos)

        p2=gameState.data.layout.agentPositions[self.getOpponents(gameState)[0]][-1]
        g=5
        '''
        You should change this in your own agent.
        '''

        return random.choice(actions)


class RealAgent(CaptureAgent):
    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the
        agent to populate useful fields (such as what team
        we're on).

        A distanceCalculator instance caches the maze distances
        between each pair of positions, so your agents can use:
        self.distancer.getDistance(p1, p2)

        IMPORTANT: This method may run for at most 15 seconds.
        """

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)
        #set up distribution list that will hold belief distributions for agents
        self.mDistribs=[None]*gameState.getNumAgents()
        #list of walls for getSuccessor
        self.walls = gameState.getWalls()
        #basic cost function for A* search
        self.costFn = lambda x: 1
        opps=self.getOpponents(gameState)
        for i in range(gameState.getNumAgents()):
            if i in opps:
                dist=util.Counter()
                oppPos=gameState.getInitialAgentPosition(i)
                dist[oppPos]=1.0
                self.mDistribs[i]=dist
            else:
                self.mDistribs[i]=None
        #should be all legal positions
        self.legalPositions = gameState.data.layout.walls.asList(key = False) #NEEDS TO BE CHECKED
        #initialize belief distribution to be 0
        for p in self.legalPositions: #NEEDS TO BE CHECKED
            for i in opps:
                if p not in self.mDistribs[i]:
                    self.mDistribs[i][p]=0.0

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def chooseAction(self, gameState):
        myState = gameState.getAgentState(self.index)

        otherState=gameState.getAgentState(self.getOpponents(gameState)[0])
        pos = otherState.getPosition()
        #print(pos)

        #get a list of actions
        #update belieft distribution about enemy positions
        #is anyone scared
        #food far away from know enemies
        #food close to enemies but near capsule
        #if theres one action:  update belief distribution and take that action
        #
        #else calculate utility function for each action
        #   

        p2=gameState.data.layout.agentPositions[self.getOpponents(gameState)[0]][-1]
        g=5
        '''
        You should change this in your own agent.
        '''
        return self.actionSearch(self.index, gameState)
        #return random.choice(actions)

    def actionSearch(self, agentIndex, gameState):
        ##do a breadth first search until time runs out
        #dictionary to keep track of visited spots so we can look up their utility in constant time
        visited = dict()
        #set to keep track of 
        visitedInSequence = set()
        #queue of action states to visit
        toVisit = util.Queue()
        actions = []
        #lower bound and upper bound set to arbitrary values for testing purposes
        upperBound = 999
        lowerBound = -1
        #start time so we can terminate before 1 second time limit
        start_time = time()
        debug = False
        #way to keep track of best action so far????
        bestActionSequence = [gameState.getLegalActions()]
        bestActionSequenceUtility = None
        #named tuple for readability
        State = namedtuple('State', 'agentIndex actions visitedInActionSequence gameState enemy_belief_states utility')
        toVisit.push(State(agentIndex, actions, visitedInSequence, gameState, [], 0))

        while time() - start_time < .75 and not toVisit.isEmpty():
            curr_state = toVisit.pop()

            for next_action in curr_state.gameState.getLegalActions():
                next_game_state = curr_state.gameState.generateSuccessor(curr_state.agentIndex, next_action)

                if debug:
                    print("curr state actions: ", curr_state.actions)
                    print("next action: ", next_action)

                new_actions = list(curr_state.actions)
                new_actions.append(next_action)

                if debug:
                    print("new actions: ", new_actions)

                #I dont think this dictionary works because all state objects will be different
                #Either need to define a new dictionary class that compares internal values of states
                #Or store more specific information in the dictionary - such as index positions
                if next_game_state in visited:
                    state_utility = visited[next_game_state]
                else:
                    state_utility = self.Utility(next_game_state)
                    visited[next_game_state] = state_utility
                #do we want to do the bounds check on just the utility of that state, or the state's utility + past_utility
                if state_utility > lowerBound and state_utility < upperBound:
                    total_utility = state_utility + curr_state.utility
                    if not bestActionSequenceUtility or total_utility > bestActionSequenceUtility:
                        bestActionSequenceUtility = total_utility
                        bestActionSequence = new_actions
                    #update enemy belief states based on move
                    enemy_belief_states = list(curr_state.enemy_belief_states)
                    toVisit.push(State(agentIndex, new_actions, visitedInSequence.add(next_action), next_game_state, enemy_belief_states, total_utility))
        return bestActionSequence[0]



    def Utility(self, gameState):
        return 0

    def _setKnownPosDist(self, agentIndex, knownPos):
        dist=self.mDistribs[agentIndex]
        for pos, _ in dist:
            if pos!=knownPos:
                dist[pos]=0
            else:
                dist[pos]=1.0

    #does inference based on noisy distance to agents and updates opponents distributions
    def positionDistanceInfer(self, agentIndex, gameState=None):
        if not gameState:
            gameState=self.getCurrentObservation()
        #myState=gameState.getAgentState(self.index)

        # noisyDistance = observation
        # emissionModel = busters.getObservationDistribution(noisyDistance)
        myPos = gameState.getAgentPosition()


        noisyDistance = gameState.getAgentDistances()[agentIndex]
        beliefs=self.mDistribs[agentIndex]
        allPossible = util.Counter()


        for p in self.legalPositions:
            trueDistance = util.manhattanDistance(p, myPos)
            if beliefs[p]==0:
                #don't need to do anything
                pass
            elif trueDistance==0:
                #no probability of ghost here, bc is current position
                allPossible[p]=0
            #NOTE: original code had the check below, but this isn't a good idea because if that prob is 0, the belief
            #for p should be updated with that in mind, so this check is silly.
            #elif gameState.getDistanceProb(trueDistance, noisyDistance)>0: #only do anything if there is any possibility of getting the given noisy distance from this true distance
            else:
                allPossible[p]=beliefs[p]*gameState.getDistanceProb(trueDistance, noisyDistance)

        allPossible.normalize()
        self.mDistribs[agentIndex]=allPossible

    #does inference based on where the agent could move to and updates opponents distributions
    def positionMoveInfer(self, agentIndex, gameState=None):
        if not gameState:
            gameState=self.getCurrentObservation()
        #myState=gameState.getAgentState(self.index)
        myPos = gameState.getAgentPosition(self.index)

        possiblePositions = util.Counter()
        beliefs=self.mDistribs[agentIndex]
        for pos in self.legalPositions:
            if beliefs[pos] > 0:
                newPosDist = self.getPositionDistribution(agentIndex, pos)
                for position, prob in newPosDist.items():
                    possiblePositions[position] += prob * beliefs[pos]

        possiblePositions.normalize()
        self.mDistribs[agentIndex]=possiblePositions

    #returns a probability distribution for the agents subsequent position, given that it is at curPos
    def getPositionDistribution(self, agentIndex, curPos):

        actions=self.getCurrentObservation().getLegalActions(agentIndex)
        probs={}
        #Currently VERY dumb impl, assumes agent moves randomly
        for action in actions:
            probs[game.Actions.getSuccessor(curPos, action)]=1/len(actions)

        return probs

    #checks if we can see either of the opponents, if so, updates their belief state and doesn't do inference
    #if not, does inference
    def updatePosDist(self):
        gameState=self.getCurrentObservation()
        for i in self.getOpponents(gameState):
            if gameState.getAgentPosition(i): #can observe the given agent
                self._setKnownPosDist(i, gameState.getAgentPosition(i))
            else:
                self.positionDistanceInfer(i)
                self.positionMoveInfer(i)

    def nullHeuristic(self, state, problem=None):
        """
        A heuristic function estimates the cost from the current state to the nearest
        goal in the provided SearchProblem.  This heuristic is trivial.
        """
        return 0

    def isGoalState(self, index):
        return self.getFood()[index[0]][index[1]]

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

        As noted in search.py:
           For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x,y = state
          dx, dy = Actions.directionToVector(action)
          nextx, nexty = int(x + dx), int(y + dy)
          if not self.walls[nextx][nexty]:
              nextState = (nextx, nexty)
              cost = self.costFn(nextState)
              successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
          self._visited[state] = True
          self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

    def aStarSearch(self, heuristic=nullHeuristic):
      """Search the node that has the lowest combined cost and heuristic first."""
      "*** YOUR CODE HERE ***"
      visited = set()
      #create priority q that gets priority from 4th element in the item
      toVisit = util.PriorityQueueWithFunction(lambda i: i[3])
      #store the total path cost to reach a node as the third element and the total path cost plus the heuristic cost in the 4th element
      toVisit.push([self.index, [], 0, 0])
      while not toVisit.isEmpty():
          #unpack the list, can ignore the 4th element because it is only used to determine priority
          cur_state, actions, pathCost, _ = toVisit.pop()

          if cur_state not in visited:
              #check goal state when node is visited
              if isGoalState(cur_state):
                  return actions
              else:
                  visited.add(cur_state)
        
              for child in problem.getSuccessors(cur_state):
                  nextState, nextAction, cost = child
                  #check if child has been visited to avoid loops
                  if nextState not in visited:
                      newActions = list(actions)
                      newActions.append(nextAction)
                      newChild = [nextState, newActions, pathCost + cost, pathCost + cost + heuristic(nextState, problem)]
                      toVisit.push(newChild)

    def foodHeuristic(self, state, problem):
        """
        Your heuristic for the FoodSearchProblem goes here.

        This heuristic must be consistent to ensure correctness.  First, try to come
        up with an admissible heuristic; almost all admissible heuristics will be
        consistent as well.

        If using A* ever finds a solution that is worse uniform cost search finds,
        your heuristic is *not* consistent, and probably not admissible!  On the
        other hand, inadmissible or inconsistent heuristics may find optimal
        solutions, so be careful.

        The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
        (see game.py) of either True or False. You can call foodGrid.asList() to get
        a list of food coordinates instead.

        If you want access to info like walls, capsules, etc., you can query the
        problem.  For example, problem.walls gives you a Grid of where the walls
        are.

        If you want to *store* information to be reused in other calls to the
        heuristic, there is a dictionary called problem.heuristicInfo that you can
        use. For example, if you only want to count the walls once and store that
        value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
        Subsequent calls to this heuristic can access
        problem.heuristicInfo['wallCount']
        """
        import collections

        #finds the manhatten distance between 2 positions
        def manDist(pos1, pos2):
            return abs(pos1[0]-pos2[0])+abs(pos1[1]-pos2[1])
        #grid is a grid, pos is a 2 tuple representing a position. returns grid[pos]
        def getPosInGrid(grid, pos):
            return grid[pos[0]][pos[1]]
        
        #creates an ordered dic of the distances from foodi to all the food in foodList, based on the info in curGameState
        #food is sorted in the dictionary from closest to farthest
        def createOrderedDistanceDic(foodi, foodList, curGameState):
            #create a temporary dictionary to store the distances as they are calculated
            #this will be replaced by a OrderedDict at the end
            tmpDic={}
            for foodj in foodList:
                #don't keep distance to itself
                if foodi==foodj:
                    continue
                #use mazeDistance to increase accuracy of estimated distances
                #since this only happens once per solve, is not too expensive
                tmpDic[foodj]=mazeDistance(foodi, foodj, curGameState)
            #create an OrderedDict sorted based on the distance between foods 
            tmp=sorted(tmpDic.items(), key=lambda i : i[1])
            return collections.OrderedDict(tmp)

        #constant key into problem.heuristicInfo
        DIST_DIC_KEY = "DIST_DIC"
        #Constant key for first position the heuristic is called on
        FIRST_POS_KEY = "FIRST_POS"
        
        position, foodGrid = state

        foodList = foodGrid.asList()

        if len(foodList)==0:
            #if foodList is 0, problem is solved
            #check here to save computation time, and also to avoid messy corner cases down the line
            return 0

        #create a generic object with no attributes for use as a dummy game state object
        class Object:
            pass
        
        #Assemble an object that has the methods expected by the mazeDistance method
        #since food and position depend on the state, but the walls depends on the problem, a new object was needed to hold this data
        curGameState = Object()
        curGameState.getPacmanPosition = lambda : position
        curGameState.getFood = lambda : foodGrid
        curGameState.getWalls = lambda : problem.walls

        #if this is the first call to the heuristic for this problem, we need to construct
        #a dictionary that holds the distances between foods
        if DIST_DIC_KEY not in problem.heuristicInfo:
            #save the first position
            #this is necessary because if this position has food in it, in this call that food will not be in foodList, because it has been eaten
            #that food might exist in later calls, however, so it has to be in distDic. By saving it here, it is easy to check in future calls if this needs to be calculated
            problem.heuristicInfo[FIRST_POS_KEY] = position
            distDicT = {}
             
            for foodi in foodList:
                distDicT[foodi]=createOrderedDistanceDic(foodi, foodList, curGameState)
            problem.heuristicInfo[DIST_DIC_KEY]=distDicT
        #checks if the first position this heuristic was called on is not in the distance dictionary and that the position should be (it has food)
        elif problem.heuristicInfo[FIRST_POS_KEY] not in  problem.heuristicInfo[DIST_DIC_KEY] and getPosInGrid(foodGrid, problem.heuristicInfo[FIRST_POS_KEY]):
            foodi=problem.heuristicInfo[FIRST_POS_KEY]
            problem.heuristicInfo[DIST_DIC_KEY][foodi]=createOrderedDistanceDic(foodi, foodList, curGameState)
        distDic = problem.heuristicInfo[DIST_DIC_KEY]


        ####
        #General Strategy:
        #Since the Minimum Spanning Tree of a Travelling Salesperson problem is always less than or equal to the optimal solution
        #if we find the MST of the food, with the current pacman position as the root, the sum of the distances in the MST is an admissable heuristic for this problem
        #therefore we implement Prim's algorithm to find the MST of the food
        #this heuristic is also consistent because we are dealing with distances, so the triangle inequality holds
        ####
        
        #the dictionary might have some food that has been eaten, this provides an easy way to check 
        #if a given piece of food has been eaten or not
        uneaten=set(foodList) #set of uneaten food
        
        
        #dictionary that tracks the minimum distance from the MST to all the food not in the MST
        #at the start, this is just the distance from the root of the MST (current position) to each node in the tree
        #NOTE: manhattan distance is used here as opposed to real distance. This increases the number of expanded nodes by a factor of ~10, but saves time
        minDistance = dict((food, manDist(position, food)) for food in foodList)
        #minDistance = dict((food, mazeDistance(position, food, curGameState)) for food in foodList)

        #total distance traversed    
        distance=0
        visited=set() #track food pieces that have been visited on this search

        #Note that while it is unlikely that the MST will change significantly between calls to the heuristic, regenerating it every time ensures all changes are properly updated

        #as long as have not visited all pieces of food
        while minDistance:
            closestFood = min(minDistance.iteritems(), key = lambda x : x[1])
            visited.add(closestFood[0])
            nextDistances=distDic[closestFood[0]]
            distance+=closestFood[1]
            del minDistance[closestFood[0]] #remove the visited food from the dictionary, as the shortest path to it has been found

            #since nextDistances is an OrderedDict, we will iterate of the food from closest to furthest from cPos
            for food, dist in nextDistances.iteritems():
                if food in visited or food not in uneaten:
                    #this condition serves as a double check we are not tracking any food that have already been visited or eaten
                    #note that nextDistances will have distances to all food that ever existed, even if they have been eaten already, necessitating the second check
                    continue
                #if a shorter path to food has been found, update the distance in minDistance
                elif minDistance[food] >dist:
                    minDistance[food]=dist

        #return the length of the MST
        return distance
