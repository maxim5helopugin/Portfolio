import torch.nn as nn
import numpy as np
import torch

# Critic network - approximates the value of the state
class Critic(nn.Module):

	def __init__(self, state_dim, action_dim):
		super(Critic, self).__init__()

		self.hidden_state_layer_1 = nn.Linear(state_dim,256)
		self.hidden_state_layer_2 = nn.Linear(256,128)
		self.hidden_action_layer_1 = nn.Linear(action_dim,128)
		self.merged_hidden_layer = nn.Linear(256,128)
		self.output_layer = nn.Linear(128,1)

		self.hidden_state_layer_1.weight.data = nn.init.kaiming_normal_(self.hidden_state_layer_1.weight.data)		
		self.hidden_state_layer_2.weight.data = nn.init.kaiming_normal_(self.hidden_state_layer_2.weight.data)		
		self.hidden_action_layer_1.weight.data = nn.init.kaiming_normal_(self.hidden_action_layer_1.weight.data)		
		self.merged_hidden_layer.weight.data = nn.init.kaiming_normal_(self.merged_hidden_layer.weight.data)		
		self.output_layer.weight.data = nn.init.kaiming_normal_(self.output_layer.weight.data)
# Forward prop
# Final layer is linear to output the continuous value
	def forward(self, state, action):
		state = torch.relu(self.hidden_state_layer_1(state))
		state = torch.relu(self.hidden_state_layer_2(state))
		action = torch.relu(self.hidden_action_layer_1(action))
		q_val = torch.cat((state,action),dim=1)
		q_val = torch.relu(self.merged_hidden_layer(q_val))
		q_val = self.output_layer(q_val)
		return q_val