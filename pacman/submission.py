from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
	"""
		A reflex agent chooses an action at each choice point by examining
		its alternatives via a state evaluation function.

		The code below is provided as a guide.  You are welcome to change
		it in any way you see fit, so long as you don't touch our method
		headers.
	"""
	def __init__(self):
		self.lastPositions = []
		self.dc = None


	def getAction(self, gameState):
		"""
		getAction chooses among the best options according to the evaluation function.

		getAction takes a GameState and returns some Directions.X for some X in the set {North, South, West, East, Stop}
		------------------------------------------------------------------------------
		Description of GameState and helper functions:

		A GameState specifies the full game state, including the food, capsules,
		agent configurations and score changes. In this function, the |gameState| argument 
		is an object of GameState class. Following are a few of the helper methods that you 
		can use to query a GameState object to gather information about the present state 
		of Pac-Man, the ghosts and the maze.
		
		gameState.getLegalActions(): 
				Returns the legal actions for the agent specified. Returns Pac-Man's legal moves by default.

		gameState.generateSuccessor(agentIndex, action): 
				Returns the successor state after the specified agent takes the action. 
				Pac-Man is always agent 0.

		gameState.getPacmanState():
				Returns an AgentState object for pacman (in game.py)
				state.configuration.pos gives the current position
				state.direction gives the travel vector

		gameState.getGhostStates():
				Returns list of AgentState objects for the ghosts

		gameState.getNumAgents():
				Returns the total number of agents in the game

		gameState.getScore():
				Returns the score corresponding to the current state of the game

		
		The GameState class is defined in pacman.py and you might want to look into that for 
		other helper methods, though you don't need to.
		"""
		# Collect legal moves and successor states
		legalMoves = gameState.getLegalActions()

		# Choose one of the best actions
		scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
		bestScore = max(scores)
		bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
		chosenIndex = random.choice(bestIndices) # Pick randomly among the best




		return legalMoves[chosenIndex]

	def evaluationFunction(self, currentGameState, action):
		"""
		The evaluation function takes in the current and proposed successor
		GameStates (pacman.py) and returns a number, where higher numbers are better.

		The code below extracts some useful information from the state, like the
		remaining food (oldFood) and Pacman position after moving (newPos).
		newScaredTimes holds the number of moves that each ghost will remain
		scared because of Pacman having eaten a power pellet.
		"""
		# Useful information you can extract from a GameState (pacman.py)
		successorGameState = currentGameState.generatePacmanSuccessor(action)
		newPos = successorGameState.getPacmanPosition()
		oldFood = currentGameState.getFood()
		newGhostStates = successorGameState.getGhostStates()
		newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]


		return successorGameState.getScore()


def scoreEvaluationFunction(currentGameState):
	"""
		This default evaluation function just returns the score of the state.
		The score is the same one displayed in the Pacman GUI.

		This evaluation function is meant for use with adversarial search agents
		(not reflex agents).
	"""
	return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
	"""
		This class provides some common elements to all of your
		multi-agent searchers.  Any methods defined here will be available
		to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

		You *do not* need to make any changes here, but you can if you want to
		add functionality to all your adversarial search agents.  Please do not
		remove anything, however.

		Note: this is an abstract class: one that should not be instantiated.  It's
		only partially specified, and designed to be extended.  Agent (game.py)
		is another abstract class.
	"""

	def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
		self.index = 0 # Pacman is always agent index 0
		self.evaluationFunction = util.lookup(evalFn, globals())
		self.depth = int(depth)

######################################################################################
# Problem 1b: implementing minimax

class MinimaxAgent(MultiAgentSearchAgent):
	"""
		Your minimax agent (problem 1)
	"""

	def getAction(self, gameState):
		"""
			Returns the minimax action from the current gameState using self.depth
			and self.evaluationFunction. Terminal states can be found by one of the following: 
			pacman won, pacman lost or there are no legal moves. 

			Here are some method calls that might be useful when implementing minimax.

			gameState.getLegalActions(agentIndex):
				Returns a list of legal actions for an agent
				agentIndex=0 means Pacman, ghosts are >= 1

			Directions.STOP:
				The stop direction, which is always legal

			gameState.generateSuccessor(agentIndex, action):
				Returns the successor game state after an agent takes an action

			gameState.getNumAgents():
				Returns the total number of agents in the game

			gameState.getScore():
				Returns the score corresponding to the current state of the game
		
			gameState.isWin():
				Returns True if it's a winning state
		
			gameState.isLose():
				Returns True if it's a losing state

			self.depth:
				The depth to which search should continue

		"""

		# BEGIN_YOUR_CODE (around 30 lines of code expected)
		def Vopt(gameState, depth, agent_index):
			if gameState.isWin() or gameState.isLose():
				return (gameState.getScore(), None)
			elif depth == 0:
				return (self.evaluationFunction(gameState), None)
			else:
				scores = []
				actions = gameState.getLegalActions(agent_index)
				if agent_index == gameState.getNumAgents() - 1:
					for action in actions:
						newState = gameState.generateSuccessor(agent_index, action)
						scores.append( (Vopt(newState, depth - 1, 0)[0], action) )
				else:
					for action in actions:
						newState = gameState.generateSuccessor(agent_index, action)
						scores.append( (Vopt(newState, depth, agent_index + 1)[0], action) )

				if agent_index == 0:
					# choose random action uniformly from max scored actions
					max_score = max(scores)[0]
					max_choices = [score for score in scores if score[0] == max_score]
					return random.choice(max_choices)
				else:
					min_score = min(scores)[0]
					min_choices = [score for score in scores if score[0] == min_score]
					return random.choice(min_choices)

		score, action = Vopt(gameState, self.depth, self.index)

		# print "Vopt: ", score
		# exit(0)

		return action

		# call with gameState, self.depth, and 0 (pacman) to start
		# call with agent 1 if we start with all of pacmans moves and get the best?
		# def Vopt(gameState, depth, agent_index):
		#   # game over - end state so return utility (score)
		#   if gameState.isWin() or gameState.isLose():
		#     return gameState.getScore()
		#   elif depth == 0:
		#     return self.evaluationFunction(gameState)
		#   else:
		#     # pacman a_0
		#     if agent_index == 0:
		#       # possible actions
		#       actions = gameState.getLegalActions(agent_index)
		#       # TODO: check for no actions
		#       return max([Vopt(gameState.generateSuccessor(agent_index, action), depth, agent_index + 1) for action in actions])
		#     # agents a_1 to a_n-1
		#     elif agent_index < (gameState.getNumAgents() - 1):
		#       # possible actions
		#       actions = gameState.getLegalActions(agent_index)
		#       return min([Vopt(gameState.generateSuccessor(agent_index, action), depth, agent_index + 1) for action in actions])
		#     # agent a_n
		#     elif agent_index == gameState.getNumAgents() - 1:
		#       # possible actions
		#       actions = gameState.getLegalActions(agent_index)
		#       return min([Vopt(gameState.generateSuccessor(agent_index, action), depth - 1, 0) for action in actions])


		# actions = gameState.getLegalActions(0)
		# scores = [(Vopt(gameState.generateSuccessor(0, action), self.depth, 1), action) for action in actions]
		# # print "scores: ", scores
		# # exit(0)
		# return max(scores)[1]
		# END_YOUR_CODE

######################################################################################
# Problem 2a: implementing alpha-beta

class AlphaBetaAgent(MultiAgentSearchAgent):
	"""
		Your minimax agent with alpha-beta pruning (problem 2)
	"""

	def getAction(self, gameState):
		"""
			Returns the minimax action using self.depth and self.evaluationFunction
		"""

		# BEGIN_YOUR_CODE (around 50 lines of code expected)
		# alpha is lower bound on Vopt for max nodes
		# beta is upper bound on Vopt for min nodes

		# def Vopt(gameState, alpha, beta, agent_index, depth):
		#   if gameState.isWin() or gameState.isLose():
		#     return (gameState.getScore(), None)
		#   elif depth == 0:
		#     return (self.evaluationFunction(gameState), None)

		#   if agent_index == 0:
		#     actions = gameState.getLegalActions(0)
		#     bestAction = None
		#     for action in actions:
		#       newState = gameState.generateSuccessor(0, action)
		#       score = Vopt(newState, alpha, beta, agent_index + 1, depth)[0]
		#       if score > alpha:
		#         alpha = score
		#         bestAction = action
		#       if alpha >= beta:
		#         return (alpha, bestAction)

		#     return (alpha, bestAction)

		#   else:
		#     actions = gameState.getLegalActions(agent_index)
		#     bestAction = None
		#     for action in actions:
		#       newState = gameState.generateSuccessor(agent_index, action)
		#       score = Vopt(newState, alpha, beta, agent_index + 1, depth)[0]
		#       if agent_index == gameState.getNumAgents() - 1:
		#         score = Vopt(newState, alpha, beta, 0, depth - 1)[0]
		#       else:
		#         score = Vopt(newState, alpha, beta, agent_index + 1, depth)[0]

		#       if score < beta:
		#         beta = score
		#         bestAction = action

		#       if beta <= alpha:
		#         return (beta, bestAction)

		#     return (beta, bestAction)

		def Vopt(gameState, alpha, beta, agent_index, depth):
			if gameState.isWin() or gameState.isLose():
				return gameState.getScore()
			elif depth == 0:
				return self.evaluationFunction(gameState)
			# pacman
			elif agent_index == 0:
				actions = gameState.getLegalActions(agent_index)
				for action in actions:
					
					newState = gameState.generateSuccessor(0, action)
					# retrieve new alpha beta and update
					score = Vopt(newState, alpha, beta, agent_index + 1, depth)
					if score > alpha:
						alpha = score
					if alpha >= beta:
						return alpha
						# break

				return alpha

			else:

				actions = gameState.getLegalActions(agent_index)
				for action in actions:
					newState = gameState.generateSuccessor(agent_index, action)
					if agent_index == gameState.getNumAgents() - 1:
						score = Vopt(newState, alpha, beta, 0, depth - 1)
					else:
						score = Vopt(newState, alpha, beta, agent_index + 1, depth)
					if score < beta:
						beta = score
					if beta <= alpha:
						# break
						return beta
				return beta

		actions = gameState.getLegalActions(0)
		alpha = float('-inf')
		beta = float('inf')
		scores = [(Vopt(gameState.generateSuccessor(0, action), alpha, beta, 1, self.depth), action) for action in actions]

		max_score = max(scores)[0]
		max_choices = [score for score in scores if score[0] == max_score]
		choice = random.choice(max_choices)

		# print "Vopt: ", choice[0]
		# exit(0)

		return choice[1]

		# END_YOUR_CODE

######################################################################################
# Problem 3b: implementing expectimax

class ExpectimaxAgent(MultiAgentSearchAgent):
	"""
		Your expectimax agent (problem 3)
	"""

	def getAction(self, gameState):
		"""
			Returns the expectimax action using self.depth and self.evaluationFunction

			All ghosts should be modeled as choosing uniformly at random from their
			legal moves.
		"""

		# BEGIN_YOUR_CODE (around 25 lines of code expected)
		def Vopt(gameState, depth, agent_index):
			if gameState.isWin() or gameState.isLose():
				return (gameState.getScore(), None)
			elif depth == 0:
				return (self.evaluationFunction(gameState), None)

			scores = []
			actions = gameState.getLegalActions(agent_index)
			if agent_index == gameState.getNumAgents() - 1:
				for action in actions:
					newState = gameState.generateSuccessor(agent_index, action)
					scores.append( (Vopt(newState, depth -1, 0)[0] , action) )
			else:
				for action in actions:
					newState = gameState.generateSuccessor(agent_index, action)
					scores.append( (Vopt(newState, depth, agent_index + 1)[0] , action) )

			if agent_index == 0:
				max_score = max(scores)[0]
				max_choices = [score for score in scores if score[0] == max_score]
				return random.choice(max_choices)
				# return max(scores)
			else:
				return (sum([score for score, action in scores]) / len(scores), random.choice([action for score, action in scores]))

		# def Vopt(gameState, depth, agent_index):
		#   if gameState.isWin() or gameState.isLose():
		#     return gameState.getScore()
		#   elif depth == 0:
		#     return self.evaluationFunction(gameState)
		#   elif agent_index == 0:
		#     actions = gameState.getLegalActions(0)
		#     return max([Vopt(gameState.generateSuccessor(agent_index, action), depth, agent_index + 1) for action in actions])
		#   elif agent_index < (gameState.getNumAgents() - 1):
		#     actions = gameState.getLegalActions(agent_index)
		#     numActions = len(actions) * 1.0
		#     current_sum = 0
		#     for action in actions:
		#       newState = gameState.generateSuccessor(agent_index, action)
		#       current_sum += Vopt(newState, depth, agent_index + 1)
		#     return current_sum / numActions
		#     # return sum([Vopt(gameState.generateSuccessor(agent_index, action), depth, agent_index + 1) / numActions for action in actions]) 
		#   elif agent_index == (gameState.getNumAgents() - 1):
		#     actions = gameState.getLegalActions(agent_index)
		#     numActions = len(actions) * 1.0
		#     for action in actions:
		#       newState = gameState.generateSuccessor(agent_index, action)
		#       current_sum += Vopt(newState, depth -1, 0)
		#     return current_sum / numActions
		#     # return sum([Vopt(gameState.generateSuccessor(agent_index, action), depth - 1, 0) / numActions for action in actions]) 

		# actions = gameState.getLegalActions(0)
		# scores = [(Vopt(gameState.generateSuccessor(0, action), 1, self.depth), action) for action in actions]
		# return max(scores)[1]

		return Vopt(gameState, self.depth, self.index)[1]

		# END_YOUR_CODE

######################################################################################
# Problem 4a (extra credit): creating a better evaluation function

def betterEvaluationFunction(currentGameState):
	"""
		Your extreme, unstoppable evaluation function (problem 4).

		DESCRIPTION: 
		-Minimizes average distance to food
		-Being close to scared ghosts gives a big score boost
		-Tries to finish off eating ghosts before ending the game
		-always tries to diminish the number of food 

	"""

	# features

	# BEGIN_YOUR_CODE (around 30 lines of code expected)
	pacman = currentGameState.getPacmanState()
	ghosts = currentGameState.getGhostStates()
	score = currentGameState.getScore()
	pacman_position = pacman.getPosition()


	def wallDistance(p1, p2):
		dist = util.manhattanDistance(p1, p2)
		if (p1[0] == p2[0]):
			if p1[1] > p2[1]:
				for i in range(int(p2[1] + 1), int(p1[1])):
					if currentGameState.hasWall(p2[0], i):
						dist += 2
			else:
				for i in range(int(p1[1] + 1), p2[1]):
					if currentGameState.hasWall(p2[0], i):
						dist += 2
		elif p1[1] == p2[1]:
			if p1[0] > p2[0]:
				for i in range(p2[0] + 1, int(p1[0])):
					if currentGameState.hasWall(i, p2[1]):
						dist += 2
			else:
				for i in range(int(p1[0] + 1), p2[0]):
					if currentGameState.hasWall(i, p2[1]):
						dist += 2
		return dist

	# finish eating ghosts before winning
	if currentGameState.getNumFood() == 1:
		if any(ghost.scaredTimer > 0 for ghost in ghosts):
			hunting_ghosts = True
		elif len(currentGameState.getCapsules()) > 0:
			hunting_ghosts = True
		else:
			hunting_ghosts = False
	else:
		hunting_ghosts = False

	ghost_positions = []
	ghost_distances = []



	# run away from normal ghosts, go towards scared ghosts
	for ghost in ghosts:
		if ghost.scaredTimer == 0:
			distance = wallDistance(ghost.getPosition(), pacman.getPosition())

			ghost_positions.append(ghost.getPosition())
			ghost_distances.append(distance)


		else:
			# 1800/1900 with the added distance on medium

			# if (ghost.getPosition()[0] == pacman.getPosition()[0]):
			# 	distance = util.manhattanDistance(ghost.getPosition(), pacman.getPosition())
			# 	if ghost.getPosition()[1] > pacman.getPosition()[1]:
			# 		for i in range(int(pacman.getPosition()[1] + 1), int(ghost.getPosition()[1])):
			# 			if currentGameState.hasWall(pacman.getPosition()[0], i):
			# 				distance += 2
			# 	else:
			# 		for i in range(int(ghost.getPosition()[1] + 1), pacman.getPosition()[1]):
			# 			if currentGameState.hasWall(pacman.getPosition()[0], i):
			# 				distance += 2
			# elif ghost.getPosition()[1] == pacman.getPosition()[1]:
			# 	distance = util.manhattanDistance(ghost.getPosition(), pacman.getPosition())
			# 	if ghost.getPosition()[0] > pacman.getPosition()[0]:
			# 		for i in range(pacman.getPosition()[0] + 1, int(ghost.getPosition()[0])):
			# 			if currentGameState.hasWall(i, pacman.getPosition()[1]):
			# 				distance += 2
			# 	else:
			# 		for i in range(int(ghost.getPosition()[0] + 1), pacman.getPosition()[0]):
			# 			if currentGameState.hasWall(i, pacman.getPosition()[1]):
			# 				distance += 2
			# else:
			distance = wallDistance(ghost.getPosition(), pacman.getPosition())

			# only hunt ghosts if they're near
			# if distance <= ghost.scaredTimer/2:
			#   score += (1.0 / distance) * 150
			if hunting_ghosts:
				score += ((1.0 / distance) * 500) + 500
			elif distance <= ghost.scaredTimer:
				score += ((1.0 / distance) * 100) + 50

	total_ghost_distance = sum(ghost_distances)
	if total_ghost_distance > 0:
		score += (len(currentGameState.getLegalActions()) / (total_ghost_distance ** 4)) * 40

	# if more than 1 ghosts are near, and approaching from multiple directions than emphasize mobility
	# num_close_ghosts = sum([1 for g in ghost_distances if g <= 4]) # 2-1493 3- 1492 # 4 - 1543
	# if num_close_ghosts > 1:

	#   threat_directions = 0

	#   ghost_right = False
	#   ghost_left = False
	#   ghost_top = False
	#   ghost_bot = False
	#   # check directions ghosts are coming from
	#   for pos in ghost_positions:
	#     if pos[0] - pacman_position[0] > 0:
	#       ghost_right = True
	#     else:
	#       ghost_left = True

	#     if pos[1] - pacman_position[1] > 0:
	#       ghost_bot = True
	#     else:
	#       ghost_top = True

	#   if ghost_right and not currentGameState.hasWall(pacman_position[0] + 1, pacman_position[1]):
	#     threat_directions += 1
	#   if ghost_left and not currentGameState.hasWall(pacman_position[0] - 1, pacman_position[1]):
	#     threat_directions += 1
	#   if ghost_bot and not currentGameState.hasWall(pacman_position[0], pacman_position[1] + 1):
	#     threat_directions += 1
	#   if ghost_top and not currentGameState.hasWall(pacman_position[0], pacman_position[1] - 1):
	#     threat_directions += 1

	#   if threat_directions == 4:
	#     score -= 100
	#   elif threat_directions == 3:
	#     score -= 50




	# always try to diminish number of food pellets
	score += (1.0 / currentGameState.getNumFood()) * 50


	# reduce average (reciprocal?) distance to food

	currentFood = currentGameState.getFood()

	distances = []

	for x in range(currentGameState.data.layout.width):
		for y in range(currentGameState.data.layout.height):
	# for x in range(currentFood.width):
	#   for y in range(currentFood.height):
			if currentFood[x][y] == True:
				distances.append( wallDistance((x, y) , pacman_position) )
				# distances.append( 1.0 / util.manhattanDistance((x, y) , pacman_position) )

	average_distance = sum(distances) / len(distances)
	# average_reciprocal_distance = sum(distances) / len(distances)

	score += (1.0 / average_distance) * 30


	score -= min(distances)




	# reduce distance to closest pellet
	# score -= min(distances) * 80

	# consume capsules

	return score
	# END_YOUR_CODE

# Abbreviation
better = betterEvaluationFunction


