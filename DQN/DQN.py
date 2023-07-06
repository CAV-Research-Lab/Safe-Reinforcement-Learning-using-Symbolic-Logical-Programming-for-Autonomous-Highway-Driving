# =====================================================================
# Date  : 20 May 2023
# Title : DQN
# Creator : Iman Sharifi
# =====================================================================

import torch
from torch import nn
import numpy as np
from collections import deque
import random
from utils import get_decayed_param

device = torch.device('cuda:2' if torch.cuda.is_available() else 'cpu')

# dqn parameters
STATE_SIZE = 10
N_ACTIONS = 3
LEARNING_RATE = 0.001  # learning rate
LEARNING_RATE_FINAL = 0.00001 # final learning rate
LEARNING_RATE_DECAY = 5e-7 # learning rate decay 
GAMMA = 0.995  # discount factor
EPS = 0.1  # initial exploration rate
EPS_FINAL = 0.001  # final exploration rate
EPS_DECAY = 5e-5  # exploration rate decay
MEM_SIZE = int(1e4)  # memory size
BATCH_SIZE = 128  # experience the batch size

# dqn
FC1_UNITS = 256  # fc 1 units
FC2_UNITS = 256  # fc layer 2 units
TARGET_UPDATE = 200  # number of steps to update the target


# =====================================================================
# the deep q network model
class DQN(nn.Module):

    # =====================================================================
    # constructor
    #   lr = learning rate
    #   state_size = number of variables in the state
    #   fc1_units  = fc1 units
    #   fc2_units  = fc2 units
    #   n_actions  = numver of actions
    def __init__(self, state_size, fc1_units, fc2_units, n_actions, weight_file_path):
        super(DQN, self).__init__()

        # initialize
        self.lr =  LEARNING_RATE
        self.state_size = state_size
        self.fc1 = nn.Linear(state_size, fc1_units)     # layer 1
        self.fc2 = nn.Linear(fc1_units, fc2_units)      # layer 2
        self.fc3 = nn.Linear(fc2_units, n_actions)      # layer 3

        # the opimizer
        self.optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)

        # the loss
        self.loss = nn.MSELoss()
        self.to(device)

        self.weight_file_path = weight_file_path

        # load from  the saved weight file if available
        try:  
            self.load_state_dict(torch.load(weight_file_path))
            print("The best weight file have been loaded.")

        except:
            print("We could not load the best weight file.")

    # =====================================================================
    # forward function
    def forward(self, state):
        x = nn.functional.relu(self.fc1(state))
        x = nn.functional.relu(self.fc2(x))
        actions = self.fc3(x)

        return actions

    # =====================================================================
    # save weights to the file
    def save_weights(self, file_path):
        self.weight_file_path = file_path
        torch.save(self.state_dict(), file_path)


# =====================================================================
# the replay memory
class Memory:

    # =====================================================================
    # constructor
    def __init__(self, max_size):
        self.index = 0  # next index to store
        self.max_size = max_size  # max memory size
        self.memory = deque(maxlen=max_size)  # memory deque

    # =====================================================================
    # push the given imput as a tuple to the memmory
    def push(self, state, next_state, action, reward, done):
        # extend until max memory
        if len(self.memory) < self.max_size:
            self.memory.append(None)

        # set the tuple
        self.memory[self.index] = (state, next_state, action, reward, done)
        self.index = (self.index + 1) % self.max_size  # update the index

    # =====================================================================
    # return a random batch of batch size
    def sample(self, batch_size):
        # select a random sample
        batch = random.sample(self.memory, batch_size)

        # convert them to seperate lists
        states = [i[0] for i in batch]
        next_states = [i[1] for i in batch]
        actions = [i[2] for i in batch]
        rewards = [i[3] for i in batch]
        done = [i[4] for i in batch]

        return states, next_states, actions, rewards, done

    # =====================================================================
    # get the memory size
    def __len__(self):
        return len(self.memory)


# =====================================================================
# the DQN agent
class DQAgent:

    # =====================================================================
    #   constructor
    #   lr         = learnint rate
    #   gamma      = discount factor
    #   eps        = initial epsilon value
    #   eps_final  = the final value of the epsilon (minimum)
    #   mem_size   = capacity of the memory
    #   state_size = number of state variables
    #   batch_size = size of the batch of experience to fit the model
    #   n_actions  =  no of actions
    #   target_update = number of steps to update the target net
    def __init__(self, weight_file_path=''):

        # initialize
        self.state_size = STATE_SIZE
        self.n_actions = N_ACTIONS
        self.eps = EPS
        self.gamma = GAMMA
        self.batch_size = BATCH_SIZE
        self.mem_size = MEM_SIZE

        self.target_update = TARGET_UPDATE

        # the policy net
        self.net = DQN(self.state_size, FC1_UNITS, FC2_UNITS, self.n_actions, weight_file_path)

        # the target net
        self.target_net = DQN(self.state_size, FC1_UNITS, FC2_UNITS, self.n_actions, weight_file_path)
        self.target_net.load_state_dict(self.net.state_dict())

        # the memory
        self.memory = Memory(self.mem_size)

        # number of learning steps
        self.step_count = 0

        # store losses of each step
        self.losses = []

        # Possible actions
        self.possible_actions = []

    # =====================================================================
    # get the next actions for a state
    def next_action(self, state, safe):

        if safe:
            # assign [0, 1, 2] to [lane_keeping, left_lane_change, right_lane_change] in the safe action set
            indexes = []
            for action in self.possible_actions:
                if action == 'lane_keeping':
                    indexes.append(0)
                elif action == 'left_lane_change':
                    indexes.append(1)
                else:
                    indexes.append(2)
        else:
            # DQN can choose both safe and unsafe actions
            indexes = [0, 1, 2]

        # do the epsilon check
        if random.random() > self.eps:  # predict next action with the policy net
            state_t = torch.tensor([state]).to(device)
            actions = self.net.forward(state_t)
            actions = list(actions[0][indexes])

            # return torch.argmax(actions).item()
            return indexes[actions.index(max(actions))]
        else:
            # the next random action
            return random.choice(indexes)

    def update_decayed_params(self, episode):
        # update learning rate
        lr = get_decayed_param(initial_value=LEARNING_RATE, decay_value=LEARNING_RATE_DECAY, final_value=LEARNING_RATE_FINAL, episode=episode)

        # update the exploration rate
        eps = get_decayed_param(initial_value=EPS, decay_value=EPS_DECAY, final_value=EPS_FINAL, episode=episode)

        return lr, eps
        


    # =====================================================================
    # learn method
    def learn(self, episode):

        # check the memory
        if len(self.memory) < self.batch_size:
            return

        self.net.optimizer.zero_grad()

        # get the experience batch
        state_batch, next_state_batch, action_batch, reward_batch, done_batch = \
            self.memory.sample(self.batch_size)

        # conver to torch tensors
        state_batch = torch.tensor(state_batch).to(device)
        next_state_batch = torch.tensor(next_state_batch).to(device)
        reward_batch = torch.tensor(reward_batch).to(device)
        done_batch = torch.tensor(done_batch).to(device)

        batch_index = np.arange(self.batch_size)  # indice to batch

        # Q values for each state and action
        q_eval = self.net.forward(state_batch)[batch_index, action_batch]

        # Q values for each next state
        q_next = self.target_net.forward(next_state_batch)

        # for done states q_targer = reward. so make q_next value 0
        q_next[done_batch] = 0.0

        # Belleman equation
        q_target = reward_batch + self.gamma * torch.max(q_next, dim=1)[0]
        # print(type(q_target))
        # fit the policy net
        loss = self.net.loss(q_target, q_eval).to(device)
        loss.backward()
        self.net.optimizer.step()

        # save the loss
        self.losses.append(loss.item())

        # update the target net
        if self.step_count % self.target_update == 0:
            self.target_net.load_state_dict(self.net.state_dict())

        self.step_count += 1

        # update important parameters
        lr, eps = self.update_decayed_params(episode)
        self.net.lr = lr
        self.target_net.lr = lr
        self.eps = eps
