import torch.nn as nn
import numpy as np
import torch

# Actor network - map states to actions
class Actor(nn.Module):
	def __init__(self, state_dim, action_dim):
		super(Actor, self).__init__()

		self.hidden_layer_1 = nn.Linear(state_dim,256)
		self.hidden_layer_2 = nn.Linear(256,128)
		self.hidden_layer_3 = nn.Linear(128,64)
		self.output_layer = nn.Linear(64,action_dim)

		self.hidden_layer_1.weight.data = nn.init.kaiming_normal_(self.hidden_layer_1.weight.data)	
		self.hidden_layer_2.weight.data = nn.init.kaiming_normal_(self.hidden_layer_2.weight.data)
		self.hidden_layer_3.weight.data = nn.init.kaiming_normal_(self.hidden_layer_3.weight.data)		
		self.output_layer.weight.data = nn.init.kaiming_normal_(self.output_layer.weight.data)

# Forward prop
	def forward(self, state):
		state = torch.relu(self.hidden_layer_1(state))
		state = torch.relu(self.hidden_layer_2(state))
		state = torch.relu(self.hidden_layer_3(state))
		action = torch.tanh(self.output_layer(state)).float()
		return action