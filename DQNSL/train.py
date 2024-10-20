# =====================================================================
# Date  : 20 May 2023
# Title : train
# Creator : Iman Sharifi
# =====================================================================

from decimal import Decimal
import pandas as pd
import csv
import pygame
import sys
import os
from pygame.locals import *
from agent import AgentCar
import matplotlib.pyplot as plt
import math
import numpy as np
import time
from utils import get_distance, get_lane
from pyswip import Prolog
import warnings

warnings.filterwarnings('ignore')

prolog = Prolog()

# Adjustable parameters ============================================
SAFE = True  # True for DQN + Symbolic Logical Programming (SLP) and False for DQN only
TRAIN = True  # 1 for train and 0 for test
N_EPISODES = 1500  # number of episodes for training
N_EPOCHS = 2  # each epoch is 420 meters
SAVE_DATA_STEPS = 20  # save weights after this number of steps
DIRECTION = 1  # highway direction [1 for left to right & -1 for right to left]
BEST_WEIGHT_FILE = 'weights/weights_20230524-223147'
WEIGHT_FILE = 'weights/weights'
MEMORY_FILE = 'memory/memory.pkl'
DATASET_DIRECTORY = '../dataset/'
if SAFE:
    if TRAIN:
        OUTPUT_EXCEL_FILE = 'dqn_slp_train_data.xlsx'
    else:
        OUTPUT_EXCEL_FILE = 'dqn_slp_test_data.xlsx'
else:
    if TRAIN:
        OUTPUT_EXCEL_FILE = 'dqn_train_data.xlsx'
    else:
        OUTPUT_EXCEL_FILE = 'dqn_test_data.xlsx'


# ==================================================================
RADAR_RENGE = 70 # PIXEL
LANES_Y = [10, 25, 36, 48, 81, 93, 108, 120]
timestr = time.strftime("%Y%m%d-%H%M%S")


# =====================================================================
# create the frame list from the csv files
def create_frame_list():
    recordingMeta = csv.DictReader(open(DATASET_DIRECTORY + "recordingMeta.csv")).__next__()
    tracks = csv.DictReader(open(DATASET_DIRECTORY + "tracks.csv"))

    frame_list = []
    frame_len = int(Decimal(recordingMeta['duration']) * int(recordingMeta['frameRate']))
    for i in range(frame_len):
        frame_list.append([])

    print('Loading frame list ...')
    i = 0
    for row in tracks:
        frame_list[int(row['frame']) - 1].append(row)
        if i % 10000 == 0:
            pass
        i = i + 1

    return frame_list


# =====================================================================
# create the car list for a frame
def get_car_list(frame):
    vehicles_pos = []
    vehicles_vel = []
    vehicles_id = []
    vehicles_lane = []

    X_vel = []
    for row in frame_list[frame]:
        # print(row)
        a = float(row['x']) * X / road_length
        b = (float(row['y']) + 0.2) * Y / road_width
        w = float(row['width']) * X / road_length
        h = float(row['height']) * Y / road_width
        velocity_x = float(row['xVelocity']) * X / road_length
        velocity_y = float(row['yVelocity']) * Y / road_width

        lane = get_lane(b, LANES_Y)
        vehicles_id.append(row['id'])
        vehicles_lane.append(lane)
        vehicles_pos.append((a, b, w, h))
        vehicles_vel.append((velocity_x, velocity_y))
        X_vel.append(velocity_x)

    AgentCar.max_velocity_other_cars = max(X_vel)

    return vehicles_id, vehicles_lane, vehicles_pos, vehicles_vel, X_vel


# =====================================================================
# main
if __name__ == '__main__':

    # the dimentional parameters of the real-world highway and the scaled highway in pygame
    road_length, road_width, X, Y = 420, 36.12, 1366, 118
    timestep = 1 / 25

    # Load the vehicle information in each frame
    frame_list = []
    a_frame_list = create_frame_list()
    for frame in a_frame_list:
        frame_list.append(frame)

    # ===============================================================
    # initialize the AgentCar object
    agent = AgentCar(width=15, height=8, direction=DIRECTION, weight_file_path=BEST_WEIGHT_FILE)

    # Pygame Settings ===============================================
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    WHITE = (255, 255, 255)

    pygame.init()  # initialize pygame
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((X, Y))

    # Load the background image here. Make sure the file exists!
    bg = pygame.image.load(DATASET_DIRECTORY + "highway.jpg")
    bg = pygame.transform.scale(bg, (X, Y))
    pygame.mouse.set_visible(1)
    pygame.display.set_caption('Highway')

    # create list for rewards
    R_lc, R_v, R_d, R_c, R_out, R_end, R_t, R_avg, R_sum = [], [], [], [], [], [], [], [], []
    epsilons, scores, avg_losses = [], [], []
    n_hits, n_lane_changes = [], []
    steps, Steps = 0, []

    # episode counter
    episode = 0
    best_score = -math.inf
    max_distance = N_EPISODES * road_length

    # frame counter
    frame = 0
    print('start training ...')

    # the main training loop =======================================
    while episode < N_EPISODES:

        steps += 1

        # get vehicle info
        vehicles_id, vehicles_lane, vehicles_pos, vehicles_vel, X_vel = get_car_list(frame)
        vehicle_pos_pygame = [pygame.Rect(rect) for rect in vehicles_pos]
        neighboring_vehicles_pos = []
        neighboring_vehicles_pos_pygame = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
        screen.blit(bg, (0, 0))
        x_ego, y_ego = agent.x, agent.y

        with open('prolog files/vehicles_info.pl', 'w') as f:
            for vId, vLane, v_pos, v_pos_pygame, v_vel in zip(vehicles_id, vehicles_lane, vehicles_pos,
                                                              vehicle_pos_pygame, vehicles_vel):
                d = get_distance([x_ego, y_ego], [v_pos[0], v_pos[1]])
                
                if d < RADAR_RENGE:

                    if  (DIRECTION == 1 and 4 <= int(vLane) <= 6) or (DIRECTION == -1 and 1 <= int(vLane) <= 3):
                        pygame.draw.rect(screen, RED, v_pos_pygame)

                        # extract the adjacent vehicles positions
                        neighboring_vehicles_pos.append(v_pos)
                        neighboring_vehicles_pos_pygame.append(v_pos_pygame)

                        # Center of rectangle for x, y positions
                        X_pos = v_pos[0] + v_pos[2] / 2
                        Y_pos = v_pos[1] + v_pos[3] / 2

                        # Sending vehicles info to prolog
                        facts = f"vehicle(v{vId}, {vLane}, {X_pos:.4f}, {Y_pos:.4f}, {v_pos[2]:.4f}, {v_pos[3]:.4f}, {v_vel[0]:.4f}, {v_vel[1]:.4f}).\n"
                        f.write(facts)

                    else:
                        pygame.draw.rect(screen, WHITE, v_pos_pygame)

                else:
                        pygame.draw.rect(screen, WHITE, v_pos_pygame)

            # Sending Ego info
            EgoLane = get_lane(agent.y, LANES_Y)
            EgoFact = f"vehicle(ego, {EgoLane}, {agent.x:.4f}, {agent.y:.4f}, {agent.width:.4f}, {agent.height:.4f}, {agent.velocity_x:.4f}, {agent.velocity_y:.4f}).\n"
            f.write(EgoFact)
            f.close()

        prolog.consult('prolog files/symbolic_logical_programming.pl')

        # Action = list(prolog.query('safe_actions(Action)'))
        L = list(prolog.query('possible_actions(Actions)'))
        safeActions = []
        for action in L[0]['Actions']:
            safeActions.append(str(action))
        # update the safe action set
        agent.possible_actions = safeActions
        # print(f">>> Safe action set: {safeActions}.")

        front_is_free = list(prolog.query('front_is_free'))
        if front_is_free== [{}]:
            agent.front_is_free = True
        else:
            agent.front_is_free = False
        # print(agent.front_is_free)

        Desired_velocity_x = list(prolog.query('longitudinal_velocity(Vx)'))
        # update the desired velocity
        agent.V_x_desired = Desired_velocity_x[0]['Vx']
        # print(f">>> Desired Velocity x: {Desired_velocity_x[0]['Vd_x']}.", end="\n")

        L = list(prolog.query("states(State)"))
        states = []
        for state in L[0]['State']:
            states.append(state)
        # update state
        agent.state = states

        L = list(prolog.query("rewards(Rewards)"))
        Rewards = []
        for reward in L[0]['Rewards']:
            Rewards.append(reward)
        # update state
        agent.reward_slp = sum(Rewards)

        EgoRect, EgoColor = agent.rect, agent.color
        pygame.draw.rect(screen, EgoColor, EgoRect)
        pygame.display.update()
        clock.tick(40)

        # update the agent car
        done, score = agent.update(timestep, episode, neighboring_vehicles_pos, safe=SAFE, train=TRAIN, max_distance=max_distance)

        # end the episode if done 
        if done or agent.epoch == N_EPOCHS:

            if TRAIN:
                frame = 0
            else:
                pass

            if episode == 0 or agent.n_hits == 0:

                # compare this episodes and the current best
                # episode's score
                if score >= best_score:
                    best_score = score  # make best episode this episode
                    best_weights_file = "weights/best_weights_" + str(best_score) + '_' + str(timestr)
                    agent.dq_agent.net.save_weights(best_weights_file)
                
                avg_loss = np.average(agent.dq_agent.losses)
                avg_reward = np.average(agent.rewards)
                scores.append(score)

                # Total steps
                Steps.append(steps)

                # Append rewards into lists
                R_lc.append(agent.lane_change_reward_sum)
                R_v.append(agent.velocity_reward_sum)
                R_d.append(agent.distance_reward_sum)
                R_c.append(agent.collision_reward_sum)
                R_out.append(agent.out_of_highway_reward_sum)
                R_end.append(agent.end_of_lane_reward_sum)
                R_t.append(agent.time_waste_reward_sum)

                R_avg.append(avg_reward)
                R_sum.append(agent.cumulative_reward)

                # Number of hits
                n_hits.append(agent.n_hits)
                n_lane_changes.append(int(agent.n_lane_changes))

                # Loss average
                avg_losses.append(avg_loss)

                # create dataframe to save rewards in excel
                df = pd.DataFrame()

                # specify column name to each list
                df['steps'] = Steps
                df['R_lc'] = R_lc
                df['R_v'] = R_v
                df['R_d'] = R_d
                df['R_c'] = R_c
                df['R_out'] = R_out
                df['R_end'] = R_end
                df['R_t'] = R_t
                df['R_avg'] = R_avg
                df['Score'] = R_sum
                df['n_hits'] = n_hits
                df['n_lc'] = n_lane_changes
                df['Loss'] = avg_losses

                memory_size = len(agent.dq_agent.memory.memory)

                # Print important data
                print(f'Episode:{episode + 1}/{N_EPISODES}| R_sum:{agent.cumulative_reward:.4f}|' +  
                      f'R_avg:{avg_reward:.4f}| Loss:{avg_loss:.4f}| LaneChange:{int(agent.n_lane_changes)}|' +
                      f'Hits:{agent.n_hits}| LR: {agent.dq_agent.net.lr:.5f}| Eps:{agent.dq_agent.eps:.4f}|' + 
                      f'Traveled_distance: {agent.traveled_distance:.4f}| Epoch: {agent.epoch}| MemSize:{memory_size}',
                      end="\n\n")

                # Save dataframe to excel
                if int((episode + 1) % SAVE_DATA_STEPS) == 0:
                    df.to_excel(OUTPUT_EXCEL_FILE)

                episode += 1
                steps = 0

                # save weights periodically
                agent.dq_agent.net.save_weights(WEIGHT_FILE)

                # save memory
                agent.dq_agent.memory.save_memory(MEMORY_FILE)

            else:
                print("Loading weights and memory ... \n")
                agent.dq_agent.net.load_weights(WEIGHT_FILE)
                agent.dq_agent.target_net.load_weights(best_weights_file)
                agent.dq_agent.memory.load_memory(MEMORY_FILE)

            # make the summation of each reward subfunction zero
            agent.reset_episode()

        frame += 1
        # restart
        if frame >= len(frame_list):
            print(frame)
            frame = 0

    # save weights at the end of the training
    agent.dq_agent.net.save_weights("weights/weights_" + str(episode) + '_' + str(timestr))

    # plot the losses
    plt.plot(avg_losses)
    plt.title('Loss')
    plt.xlabel('Episode')
    plt.ylabel('Loss')
    plt.grid()
    plt.savefig('img/fig_Q_Loss.png')
    plt.show()

    #    plotData( avg_losses, 'Loss', 'Episode', 'Loss', 'fig_Q_loss.png' )

    # plot scores
    plt.plot(scores)
    plt.title('Score')
    plt.xlabel('Episode')
    plt.ylabel('Score')
    plt.grid()
    plt.savefig('img/fig_scores.png')
    plt.show()

    #    plotData( scores, 'Score', 'Episode', 'Score', 'fig_score.png' )
