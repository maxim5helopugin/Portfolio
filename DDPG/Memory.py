from collections import deque
import random
import numpy as np

# Memory Buffer class
class Memory:
	def __init__(self, size):
		self.buffer = deque(maxlen=size)

# Random sample
	def sample(self, size):
		batch = random.sample(self.buffer, size)
		random_index_list = np.random.choice(np.arange(len(self.buffer)), size = size, replace = False)
		sublist =  [self.buffer[index] for index in random_index_list]
		states, actions, rewards, next_states = zip(*sublist)
		return states, actions, rewards, next_states

	def remember(self, experience):
		self.buffer.append(experience)

