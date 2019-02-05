from Actor import Actor
from Critic import Critic
import numpy as np
import torch
from torch.autograd import Variable
import torch.nn as nn

LEARNING_RATE = 0.00001	
BATCH_SIZE = 256
GAMMA = 0.999
TAU = 0.0001
# Agent
# consists of Actor, Critic, Target Actor and Target Critic
class ActorCritic:
	def __init__(self, state_dim, action_dim, memory, load):
		self.memory = memory
		self.noise = OrnsteinUhlenbeckActionNoise(action_dim)

		self.actor = Actor(state_dim, action_dim)
		self.critic = Critic(state_dim, action_dim)
		self.target_actor = Actor(state_dim, action_dim)
		self.target_critic = Critic(state_dim, action_dim)

		self.critic.cuda()
		self.actor.cuda()
		self.target_critic.cuda()
		self.target_actor.cuda()

		self.actor_optimizer = torch.optim.Adam(self.actor.parameters(),LEARNING_RATE)
		self.critic_optimizer = torch.optim.Adam(self.critic.parameters(),LEARNING_RATE)

		self.loss_funct = nn.SmoothL1Loss()
		if load != 0:
			self.load_models(load) #load the model

# Target and trained networks are the same when initializing
		self.net_update(self.target_actor, self.actor, True)
		self.net_update(self.target_critic, self.critic, True)

# Predict an action with or without noise depending on the "train" flag
	def get_action(self, state, train):
		state = Variable(torch.from_numpy(np.float32(state)).type(torch.cuda.FloatTensor))
		action = self.actor.forward(state).detach().cpu().numpy()
		if train:
			noise = np.float32(self.noise.sample())
			return action + noise
		return action

# Run the optimization:
#	Get predicted action from the next state by Target Actor
#	Base on that predict the Value of that action by Target Critic
#	Use the predicted value to update Critic, and then Actor
#	Soft update target networks to mirror the progress
	def optimize(self):
		state,action,reward,next_state = self.memory.sample(BATCH_SIZE)

		state = Variable(torch.from_numpy(np.float32(state)).type(torch.cuda.FloatTensor))
		action = Variable(torch.from_numpy(np.float32(action)).type(torch.cuda.FloatTensor))
		reward = Variable(torch.from_numpy(np.float32(reward)).type(torch.cuda.FloatTensor))
		next_state = Variable(torch.from_numpy(np.float32(next_state)).type(torch.cuda.FloatTensor))

		next_action = self.target_actor.forward(next_state).detach()
		target = reward + GAMMA*torch.squeeze(self.target_critic.forward(next_state, next_action).detach())

		prediction = torch.squeeze(self.critic.forward(state, action))

		loss_critic = self.loss_funct(prediction, target)
		self.critic_optimizer.zero_grad()
		loss_critic.backward()
		self.critic_optimizer.step()

		action = self.actor.forward(state)
		loss_actor = -1*torch.sum(self.critic.forward(state, action))
		self.actor_optimizer.zero_grad()
		loss_actor.backward()
		self.actor_optimizer.step()

		self.net_update(self.target_actor, self.actor, False)
		self.net_update(self.target_critic, self.critic, False)

	# Apply soft or hard update on the network
	def net_update(self,target, source, hard):
		degree = 1
		if not hard: degree = TAU
		for target_param, param in zip(target.parameters(), source.parameters()):
			target_param.data.copy_(target_param.data * (1.0 - degree) + param.data * degree)

# Store the models
	def save_models(self, episode):
		torch.save(self.target_actor.state_dict(), 'Models/' + str(episode) + '_actor.pt')
		torch.save(self.target_critic.state_dict(), 'Models/' + str(episode) + '_critic.pt')
		
# Load the models
	def load_models(self, episode):
		self.actor.load_state_dict(torch.load('Models/' + str(episode) + '_actor.pt'))
		self.critic.load_state_dict(torch.load('Models/' + str(episode) + '_critic.pt'))
		self.net_update(self.target_actor, self.actor, True)
		self.net_update(self.target_critic, self.critic, True)
		print('Models loaded succesfully')

		# Generate the noise 
class OrnsteinUhlenbeckActionNoise:
	def __init__(self, action_dim):
		self.action_dim = action_dim
		self.theta = 0.15
		self.sigma = 0.2
		self.dx = np.zeros(self.action_dim)

	def sample(self):
		self.dx = self.dx + self.theta * (-self.dx) + self.sigma * np.random.randn(len(self.dx))
		return self.dx
