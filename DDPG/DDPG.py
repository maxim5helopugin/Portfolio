# Maxim Shelopuhin
# DDPG on Bipedal Walker
# 
#	Noise function based on http://math.stackexchange.com/questions/1287634/implementing-ornstein-uhlenbeck-in-matlab
#					and     https://github.com/openai/baselines/blob/master/baselines/ddpg/noise.py
#	Network architecture based on https://github.com/vy007vikas/PyTorch-ActorCriticRL
#	Algorithm based on "Continuous control with deep reinforcement learning"
#					by Timothy P. Lillicrap, Jonathan J. Hunt, Alexander Pritzel, Nicolas Heess, Tom Erez, Yuval Tassa, David Silver, Daan Wierstra
#	
import gym
from Memory import Memory
from ActorCritic import ActorCritic
import numpy as np
import gc
import matplotlib.pyplot as plt
import sys
import time

MAX_EPISODES = 5000
MAX_STEPS = 10000
MAX_BUFFER = 200000
MAX_TOTAL_REWARD = 300

#Train for a single run
# Training done with exploration = true
def env_run(env, episode, trainer, memory, train):
	state = env.reset()
	epoch_reward = 0
	print(episode)
# Take step
	for step in range(MAX_STEPS):
		if not train:
			env.render()
		action = trainer.get_action(state, train)
		next_state, reward, done, _ = env.step(action)
		epoch_reward +=reward
		if train:
			if done:
				break
			memory.remember((state, action, reward, next_state))
			state = next_state
			trainer.optimize()
		else:
			if done:
				env.close()
				print("\n Testing agent got a reward of :",epoch_reward)
				break
		
		state = next_state
	gc.collect()
	if episode%100 == 1:
		trainer.save_models(episode)
	return epoch_reward

def prepopulate_memory(memory, env):
	state = env.reset()
	for _ in range(MAX_BUFFER):
	# Take a random action
		action = env.action_space.sample()
		next_state, reward, done, _ = env.step(action)
		memory.remember((state, action, reward, next_state))
		if done:
			state = env.reset()
		else:
			state = next_state

# Create environment
# Read arguments
# Depending on args:
#	decide if training
#	determine the testing interval
#	determine which file to load
#	output stats to the screen
def main(args):

	training = int(args[1])
	test_interwal = int(args[2])
	load = int(args[3])

	env = gym.make('BipedalWalker-v2')
	memory = None

	if training == 1:
		memory = Memory(MAX_BUFFER)
		prepopulate_memory(memory, env)
		
	rewards = []
	start_time = time.time()
	max_reward = 0

	trainer = ActorCritic(env.observation_space.shape[0], env.action_space.shape[0], memory, load)

	for episode in np.arange(MAX_EPISODES):
		if training == 1:
			env_run(env, episode, trainer, memory, True)
		if episode%test_interwal == 0:
			max_reward += env_run(env, episode,trainer, None, False)
			rewards.append(max_reward/((episode/test_interwal)+1))
	plt.plot(rewards)
	plt.show()

# use this to plot Ornstein Uhlenbeck random motion
if __name__ == '__main__':
	main(sys.argv)