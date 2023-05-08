# =====================================================================
# Date  : 14 Nov 2020
# Title : DQN
# =====================================================================

import torch
from torch import nn
import numpy as np
from collections import deque
import random

device = torch.device('cuda:2' if torch.cuda.is_available() else 'cpu')

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
    def __init__(self, lr, state_size, fc1_units, fc2_units, n_actions,
                 weight_file_path="weights/best_weights_25.0_20230506-210352"):
        super(DQN, self).__init__()

        # initialize
        self.state_size = state_size
        self.fc1 = nn.Linear(state_size, fc1_units)  # layer 1
        self.fc2 = nn.Linear(fc1_units, fc2_units)  # layer 2
        self.fc3 = nn.Linear(fc2_units, n_actions)  # layer 3

        # the opimizer
        self.optimizer = torch.optim.Adam(self.parameters(), lr=lr)

        # the loss
        self.loss = nn.MSELoss()
        self.to(device)

        self.weight_file_path = weight_file_path

        try:  # load from  the saved weight file if available
            self.load_state_dict(torch.load(weight_file_path))
            print("weights are loaded")

        except:
            print("could not load the weight file")

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
    # constructor
    #   lr         = learnint rate
    #   gamma      = discount factor
    #   eps        = initial epsilon value
    #   eps_final  = the final value of the epsilon (minimum)
    #   mem_size   = capacity of the memory
    #   state_size = number of state variables
    #   batch_size = size of the btach of experience to fit the model
    #   n_actions  =  no of actions
    #   target_update = number of steps to update the target net
    def __init__(self, lr, gamma, eps, eps_final, eps_dec,
                 mem_size, state_size, batch_size, n_actions):

        # initialize
        self.gamma = gamma
        self.eps = eps
        self.eps_final = eps_final
        self.eps_dec = eps_dec
        self.batch_size = batch_size
        self.state_size = state_size
        self.mem_size = mem_size
        self.lr = lr
        self.n_actions = n_actions
        self.target_update = TARGET_UPDATE

        # the policy net
        self.net = DQN(lr, state_size, FC1_UNITS, FC2_UNITS, n_actions)

        # the target net
        self.target_net = DQN(lr, state_size, FC1_UNITS, FC2_UNITS, n_actions)
        self.target_net.load_state_dict(self.net.state_dict())

        # the memory
        self.memory = Memory(mem_size)

        # number of learning steps
        self.step_count = 0

        # store losse of each step
        self.losses = []

        # Possible actions
        self.possible_actions = []

    # =====================================================================
    # get the next actions for a state
    def next_action(self, state):

        # Extracting safe action set
        # print(f'\nSafe Action Set: {self.possible_actions}')
        indexes = []
        for action in self.possible_actions:
            if action == 'lane_keeping':
                indexes.append(0)
            elif action == 'left_lane_change':
                indexes.append(1)
            else:
                indexes.append(2)

        # do the epsilon check
        if random.random() > self.eps:  # predict next action with the policy net
            state_t = torch.tensor([state]).to(device)
            actions = self.net.forward(state_t)
            actions = list(actions[0][indexes])

            # return torch.argmax(actions).item()
            return indexes[actions.index(max(actions))]
        else:
            # the random next action
            # return random.randint(0, self.n_actions-1)
            # return random.randint(0, len(self.possible_actions)-1)
            return random.choice(indexes)

    # =====================================================================
    # learn method
    def learn(self):
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

        # reduce epsilon
        if self.eps > self.eps_final:
            self.eps -= self.eps_dec

        else:
            self.eps = self.eps_final
