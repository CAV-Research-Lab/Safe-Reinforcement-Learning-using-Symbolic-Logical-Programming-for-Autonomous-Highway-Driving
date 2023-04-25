# =====================================================================
# Date  : 11 Nov 2020
# Title : agent
# =====================================================================

import pygame
import numpy as np
from DQN import DQAgent
from utils import truncate, get_distance, get_lane
import random
from math import sqrt

# lane details
# y values of lanes ( do not change )
LANES_Y = [25, 36, 48, 81, 93, 108, 116]
ROAD_LENGTH = 1366.0  # image pixel
REAL_ROAD_LENGTH = 420.0  # meter
TIME_STEP = 1.0/25.0

# agent car details
ACTIONS = ["do nothing", "left", "right" ]

MIN_SPEED = 30       # minimum speed of the agent car
MAX_SPEED = 125      # maximum speed
ACCELERATION = 3     # acceleration
DEACCELERATION = 5   # deacceleration
SENSOR_RANGE =  250  # maximum range of the sensors of the car

# dqn
LEARNING_RATE = 0.00001   # learning rate
GAMMA = 0.995             # discount factor
EPS = 1.0                 # initial epsilon
EPS_FINAL = 0.001         # final epsilon
EPS_DECAY = 9e-7          # epsilon decay
MEM_SIZE = int(1e3)       # memory size                            ***
BATCH_SIZE = 128          # experience the batch size

# reward details
CAR_HIT_REWARD = -10             # if hit by a car
END_OF_LANE_REWARD = 30          # for reaching to the end of the lane
REWARD_LANE_CHANGE = -1          # reward for unnecessary lane change
REWARD_OUT_OF_LEGAL_LANES = -50  # reward for getting out of the highway lanes
REWARD_TIME_WASTE = -0.1         # reward for wasting time
# the car infront is away from the agent than this amount
# but agent change the lane it is a unnecessary lane change

# REWARD_VELOCITY_THRESHOLD = 50   # reward is given only if the speed is greater than this
# REWARD_VELOCITY_FACTOR = 50      # the factor for reward velocity
# REWARD_CAR_PASS = 100            # reward for passing a car
# REWARD_DISTANCE_FACTOR = 10      # reward factor for the distance based reward
# REWARD_TIME_BONUS_FACTOR = 500.0 # reward factor for the time based reward
# LANE_CHANGE_REWARD_MIN_DISTANCE_TO_CAR = 100

# =====================================================================
# the car class for the DQ
class AgentCar:

    min_speed = MIN_SPEED
    max_speed = MAX_SPEED
    acceleration = ACCELERATION
    deacceleration = DEACCELERATION
    sensor_range = SENSOR_RANGE
#    max_velocity_other_cars = MAX_SPEED

    # =====================================================================
    # eps = initial epsilon value
    def __init__( self, width=15, height=8, lane=5, speed=72, direction=1, eps=0.1 ):
        
        self.dt = TIME_STEP
        self.init_lane = lane     # initial lane
        self.init_speed = speed     # initial speed
        self.width = width          # width of the car
        self.height = height        # height
        self.direction = direction  # 1 is right, -1 left
        self.reset()                # initial states
        self.reset_rewards()        # reset rewards

        self.n_hits = 0             # number of collision
        self.n_hits_right_front = 0
        self.n_hits_right_back = 0
        self.n_hits_left_front = 0
        self.n_hits_left_back = 0

        # reactangle for the car
        self.rect = pygame.Rect(self.x-int(width/2), self.y-int(height/2), self.width, self.height)

        # the DQN agent
        self.dq_agent = DQAgent(lr=LEARNING_RATE, gamma=GAMMA, eps=eps, eps_final=EPS_FINAL,
                                eps_dec=EPS_DECAY, mem_size=MEM_SIZE, state_size=7,
                                batch_size=BATCH_SIZE, n_actions=3)

        # number of done states (current)
        self.terminal_count = 0

        self.car_front1 = ROAD_LENGTH if self.direction == 1 else 0
        self.car_front2 = ROAD_LENGTH if self.direction == 1 else 0

        # Safe action derived from Prolog
        self.safe_action = 'lane_keeping'
        self.possible_actions = []
        self.V_x_desired = 70

    # =====================================================================
    # restart
    def reset( self ):
        self.lane = random.randint(3,5)

        # get the initial x value
        if self.direction == 1:
            self.x = 20
        else:
            self.x = 1000

        # get the initial y value
        self.y = LANES_Y[self.lane]-5
        self.lane = get_lane(self.y, LANES_Y)

        # reset states
        self.velocity_x = random.randint(40, 80)
        self.velocity_y = 0

        self.color = (0, 255, 0)

        self.score = 0  # score of this episode
        self.time = 0 # time of this episode

        # state and action of  the previous frame
        self.last_state = None
        self.last_action = 0

        self.rewards = []   # all the rewards of the currect episode
        self.level = 0      #

        # current distance travelled
        self.distance_traveled = 0

        self.n_lane_changes = 0
        self.s = 1

        # reset integrator error for PID control of y
        self.e_i_y = 0
        self.e_i_V = 0
    
    def reset_rewards(self):
        self.lane_change_reward_sum = 0
        self.collision_reward_sum = 0
        self.out_of_highway_reward_sum = 0
        self.end_of_lane_reward_sum = 0
        self.time_waste_reward_sum = 0

    # =====================================================================
    # execute action (y control)
    def perform_action(self, action):

        lane = get_lane(self.y, LANES_Y)
        if lane >= 4:
            if action == 'right_lane_change':
                y_desired = LANES_Y[lane]-5
                self.n_lane_changes+=1
            elif action == 'left_lane_change':
                y_desired = LANES_Y[lane-2]-5
                self.n_lane_changes+=1
            else:
                y_desired = LANES_Y[lane-1]-5

        else:
            if action == 'right_lane_change':
                y_desired = LANES_Y[lane-2]-5
                self.n_lane_changes+=1
            elif action == 'left_lane_change':
                y_desired = LANES_Y[lane]-5
                self.n_lane_changes+=1
            else:
                y_desired = LANES_Y[lane-1]-5           

        self.velocity_y = self.y_PI_Control(y_desired)

    # =====================================================================
    # convert the dqn action to symbolic actions {LK, LLC, RLC}
    def translate_action(self, action):
        if action == 0:
            return 'lane_keeping'
        elif action == 1:
            return 'left_lane_change'
        else:
            return 'right_lane_change'

    def y_PI_Control(self, y_desired):
        
        Kp, Ki = 2, 1
        e_p = y_desired - self.y
        self.e_i_y += e_p*self.dt
        u = Kp*e_p+Ki*self.e_i_y

        return u

    def Vx_PI_Control(self, Vx_desired):
        
        Kp, Ki = 2, 1
        e_p = Vx_desired - self.velocity_x
        self.e_i_V += e_p*self.dt
        u = Kp*e_p + Ki*self.e_i_V

        return u

    # =====================================================================
    # reward functions
    def get_lane_change_reward( self, action ):

        if action == 'left_lane_change' or action == 'right_lane_change':
            return REWARD_LANE_CHANGE
        else:
            return 0

    def get_collision_reward(self, car_list2):

        reward_collision = 0
        done = False
        width, height = self.width, self.height

        for car in car_list2:

            car_x, car_y, car_w, car_h = car[0], car[1], car[2], car[3]

            if (car_x+car_w) >= (self.x-int(width/2)) >= car_x and (car_y+car_h) >= (self.y-int(height/2)) >= car_y and self.x>100:
                print('left_back_crash')
                reward_collision = CAR_HIT_REWARD
                self.n_hits_left_back+=1
                done=True

            elif ((car_x+car_w) >= (self.x-int(width/2))+width) >= car_x and (car_y+car_h) >= (self.y-int(height/2)) >= car_y and self.x>100:
                print('left__front_crash')
                reward_collision = CAR_HIT_REWARD
                self.n_hits_left_front+=1
                done=True

            elif (car_x+car_w) >= (self.x-int(width/2)) >= car_x and ((car_y+car_h) >= (self.y-int(height/2))+height) >= car_y and self.x>100:
                print('right_back_crash')
                reward_collision = CAR_HIT_REWARD
                self.n_hits_right_back+=1
                done=True

            elif ((car_x+car_w) >= (self.x-int(width/2))+width) >= car_x and ((car_y+car_h) >= (self.y-int(height/2))+height) >= car_y and self.x>100:
                print('right_front_crash')
                reward_collision = CAR_HIT_REWARD
                self.n_hits_right_front+=1
                done=True

        return reward_collision, done
    
    def get_end_of_lane_reward(self):
        # End of the lane
        done = False
        if self.x >= (ROAD_LENGTH - 20):
            reward_end = END_OF_LANE_REWARD
    #                self.next_turn()
            done = True
        else:
            reward_end = 0

        return reward_end, done
    
    def get_out_of_legal_lanes_reward( self):
        done = False
        Border1, Border2 = 65, 112
        if Border1 <= self.y <= Border2:
            reward_out = 0
        else:
            done = True
            reward_out = REWARD_OUT_OF_LEGAL_LANES

        return reward_out, done
    # =====================================================================
    # update the agent's car each frame
    # return = done, score
    #   dt       = frame time
    #   car_list = list of other cars as rectangle objecs
    #   surface  = provide to draw the surface
    #   learn    = enable or disable learning
    def update( self, dt, car_list, car_list2, velocities, surface=None, learn=True ):    

        print(f"Ego position: X = {self.x:.4f}, Y = {self.y:.4f}, Lane = {get_lane(self.y, LANES_Y)}.")
        # get distances to surrounding cars
        distances, velocities_ = self.radar( car_list, velocities, surface )
        state = self.get_state( distances, velocities_ )

        # Safe action set
        self.dq_agent.possible_actions = self.possible_actions

        # (Safe) DQN Action =====================================
        action = self.dq_agent.next_action( state )
        dqn_action = self.translate_action(action)

        # Reward function =======================================
        reward_lane_change = self.get_lane_change_reward(dqn_action)
        reward_collision, done1 = self.get_collision_reward(car_list2)
        reward_out, done2 = self.get_out_of_legal_lanes_reward()
        reward_end, done3 = self.get_end_of_lane_reward()
        reward_time = REWARD_TIME_WASTE
        reward = reward_lane_change + reward_collision + reward_out + reward_end + reward_time

        # Sum of individual rewards
        self.lane_change_reward_sum += reward_lane_change
        self.collision_reward_sum += reward_collision
        self.out_of_highway_reward_sum += reward_out
        self.end_of_lane_reward_sum += reward_end
        self.time_waste_reward_sum += reward_time

        # Learning Process ======================================
        score = 0
        avg_reward = 0
        # if learn is enabled
        if learn:
            if self.last_state is not None:
                self.dq_agent.memory.push(self.last_state, state, self.last_action, reward, done3 )

            self.last_state = state
            self.last_action = action
            self.score += reward
            score  = self.score

            self.dq_agent.learn()

        self.rewards.append( reward )

        # if terminal state met
        done = done1 or done2 or done3
        if done:
            if (len(self.rewards) > 0):
                avg_reward = np.average( self.rewards )
            print( f'Episode {self.terminal_count+1}: epsilon:{truncate(self.dq_agent.eps, 3)}, avg-reward: {truncate(avg_reward, 2)}' )
            
            # Number of hits
            self.n_hits = self.n_hits_right_front + self.n_hits_right_back + self.n_hits_left_front + self.n_hits_left_back

            self.terminal_count += 1
            self.reset()

        # Velocity Control =================================
        ax = self.Vx_PI_Control(self.V_x_desired)
        self.velocity_x +=  ax*dt

        # Execute action ==================================
        # self.perform_action(self.safe_action)
        self.perform_action(dqn_action)

        # Update longitudinal and lateral positions of Ego Vehicle and the assigned rectangle
        self.x += self.velocity_x * dt
        self.y += self.velocity_y * dt
        self.rect.center = ( self.x, self.y )

        # time
        self.time += dt

        return done, score

    # =====================================================================
    # return the state as an array ( for the DQN agent )
    # state = [front_car_dis, back_car_dis,
    #          left_front_car_dis, left_back_car_dis,
    #          right_front_car_dis, right_back_car_dis,
    #          lane, speed]      ( all are normalized(0 - 1))
    def get_state( self, distances, velocities ):
        state = distances.copy()

        if self.direction == -1:
            # swap front and back car  distances
            for i in range(0, 5, 2):
                temp = state[i+1]
                state[i+1] = state[i]
                state[i] = temp

        state = [i / AgentCar.sensor_range for i in state]
        state.append( self.velocity_x / AgentCar.max_speed )
        return state

    # =====================================================================
    # get the distances to nearest cars in this lane, and left and right lanes
    def radar( self, car_list, velocities, surface = None ):
        car_lists = [[], [], [], [], [], []]

        # go through each car
        #print('car lists radar en basi',car_lists, len( car_list ))
        for i in range( len( car_list ) ):
            car = car_list[i]
            velocity = velocities[i]

            lane = get_lane( self.y, LANES_Y )
            # cars in the lane
            if lane == self.lane:
                if car.left  > self.x:
                    car_lists[0].append( (car, velocity) )

                else:
                    car_lists[1].append( (car, velocity) )

            # cars in the above lane
            elif lane == self.lane - 1:
                 # ignore cars between 2 main lanes
                if self.direction == 1 and lane == 2:
                    continue

                if car.center[0] > self.x:
                    car_lists[2].append( (car, velocity) )

                else:
                    car_lists[3].append( (car, velocity) )

            # cars in the below lane
            elif lane == self.lane + 1:
                # ignore cars between 2 main lanes
                if self.direction == -1 and lane == 3:
                    continue

                if car.center[0] > self.x:
                    car_lists[4].append( (car, velocity) )

                else:
                    car_lists[5].append( (car, velocity) )

        distances = []
        velocities_ = []

        # get distances and draw rays (if surfaces is provided)
        for i in range( 6 ):

            if i % 2 == 0:
                c, distance, velocity = self.get_closest_car_lane( car_lists[i] )

                if c is not None:
                    if self.direction == 1:
                        if i == 2:
                            self.car_front1 = c.left

                        if i == 4:
                            self.car_front2 = c.left

                    else:
                        if i == 3:
                            self.car_front1 = c.right

                        if i == 5:
                            self.car_front2 = c.right

                    if surface is not None:
                        if distance > 0:
                            self.draw_ray( surface, c );
                        else:
                            self.draw_ray( surface, c, side=True )

            else:
                c, distance, velocity = self.get_closest_car_lane( car_lists[i], False )

                if c is not None:
                    if surface is not None:
                        if distance > 0:
                            self.draw_ray( surface, c, pos=False )
                        else:
                            self.draw_ray( surface, c, pos=False, side=True );


            distances.append( distance )
            velocities_.append( velocity )

        return distances, velocities_

    # =====================================================================
    # get the closet car of a given car list on a lane
    # pos true if x positive direction
    def get_closest_car_lane( self, car_list, pos=True ):
        if len(car_list) > 0:   # if the list is not empty
            l = [] # distance to all the cars

            # go through each car
            for car, _ in car_list:
                k = self.width / 2 + car.width / 2 + 5

                if pos: # for the positive x direction
                    if car.center[0] - self.x > k:
                        l.append( car.left - self.x )

                    else: # if the car is very close to the agent append 0
                        l.append( 0 )

                else: # for the negative direction
                    if self.x - car.center[0] > k:
                        l.append( self.x - car.right )

                    else:
                        l.append( 0 )

            # get the minimum length
            i = np.argmin(l)
            car = car_list[i][0]

            # calculate distance
            if pos:
                distance = car.left - self.x
            else:
                distance = self.x - car.right

            # if the car is out of the range
            #print('distance',distance)
            if distance > AgentCar.sensor_range:
                return None, AgentCar.sensor_range, 0.0

            return car, distance, car_list[i][1]

        return None, AgentCar.sensor_range, 0.0

    # =====================================================================
    # draw a ray to closest car
    def draw_ray( self, surface, car, side=False, pos=True ):
        lane = get_lane( self.y, LANES_Y )

        if not side:
            if pos:
                pygame.draw.line( surface, (255, 0, 0), (self.x, LANES_Y[lane] ), (car.left, LANES_Y[lane] ), 2 )

            else:
                pygame.draw.line( surface, (255, 0, 0), (self.x, LANES_Y[lane] ), (car.right, LANES_Y[lane] ), 2 )

        else:
             pygame.draw.line( surface, (0, 255, 255), (self.x, self.y ), (self.x, LANES_Y[lane] ), 2 )

    # =====================================================================
    # draw the agent
    def draw( self, surface ):
        pygame.draw.rect( surface, self.color, self.rect )

    # =====================================================================
    # get the lane of the given rectangle car
    @staticmethod
    def get_track( rect ):
        temp = np.array( LANES_Y )
        return int( np.argmin( np.abs(temp - rect.center[1] )) )

