# =====================================================================
# Date  : 15 Nov 2020
# Title : train
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

prolog = Prolog()

from torch.utils.tensorboard import SummaryWriter

show_video = 1
N_EPISODES = 20000  # number of episodes for training
WEIGHT_SAVE_STEPS = 50  # save weights after this number of steps

timestr = time.strftime("%Y%m%d-%H%M%S")


# =====================================================================
# create the frame list from the csv files
def create_frame_list(map_id):
    recordingMeta = csv.DictReader(open("../HighD/Metas/" + str(map_id) + '_recordingMeta.csv')).__next__()
    tracks = csv.DictReader(open("../HighD/Tracks/" + str(map_id) + '_tracks.csv'))
    # tracksMeta = csv.DictReader(open("../HighD/Statics/" + str(map_id) + '_tracksMeta.csv'))
    # print(recordingMeta)

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

    road_length, road_width, X, Y = 420, 36.12, 1366, 118
    LANES_Y = [25, 36, 48, 81, 93, 108, 114]
    timestep = 1 / 25

    frame_list = []
    map_list = ["01", "02", "03", 11, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
    for track_id in map_list:
        print('get', track_id)
        a_frame_list = create_frame_list(track_id)
        for frame in a_frame_list:
            frame_list.append(frame)

    epsilons = []
    scores = []
    avg_velocities = []
    velocities = []
    avg_losses = []

    Name = "DQN_IDM_Collision_reward_m_train_{}".format(str(timestr))
    writer = SummaryWriter(log_dir="m_train/{}".format(Name))

    # the AgentCar object
    agent = AgentCar(width=15, height=8, lane=5, speed=72, direction=1)

    # number of episodes
    n_episodes = N_EPISODES

    # episode counter
    episode = 0
    best_score = -math.inf

    # frame counter
    frame = 0
    print('start training ...')

    # Pygame Settings ===============================================
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    WHITE = (255, 255, 255)

    pygame.init()  # initialize pygame
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((X, Y))

    # Load the background image here. Make sure the file exists!
    bg = pygame.image.load("../HighD/highway.jpg")
    bg = pygame.transform.scale(bg, (X, Y))
    pygame.mouse.set_visible(1)
    pygame.display.set_caption('Highway')

    # create list for rewards
    R_lc, R_v, R_c, R_out, R_end, R_t, R_avg = [], [], [], [], [], [], []
    n_hits = []
    steps, Steps = 0, []

    # the main training loop =======================================
    while episode < n_episodes:

        steps += 1

        # get vehicle info
        vehicles_id, vehicles_lane, vehicles_pos, vehicles_vel, X_vel = get_car_list(frame)
        vehicle_pos_pygame = [pygame.Rect(rect) for rect in vehicles_pos]
        # vehicles_vel.append((agent.velocity_x, agent.velocity_y))
        neighboring_vehicles_pos = []
        neighboring_vehicles_pos_pygame = []

        if show_video == 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
            screen.blit(bg, (0, 0))
            x_ego, y_ego = agent.x, agent.y

            # prolog.query("retractall(vehicle)")
            with open('prolog files/vehicle_clauses.pl', 'w') as f:
                for vId, vLane, v_pos, v_pos_pygame, v_vel in zip(vehicles_id, vehicles_lane, vehicles_pos,
                                                                  vehicle_pos_pygame, vehicles_vel):
                    d = get_distance([x_ego, y_ego], [v_pos[0], v_pos[1]])
                    if d < 70 and 4 <= int(vLane) <= 6:
                        pygame.draw.rect(screen, RED, v_pos_pygame)

                        # extract the adjacent vehicles positions
                        neighboring_vehicles_pos.append(v_pos)
                        neighboring_vehicles_pos_pygame.append(v_pos_pygame)

                        # Center of rectangle for x, y positions
                        X_pos = v_pos[0] + v_pos[2] / 2
                        Y_pos = v_pos[1] + v_pos[3] / 2

                        # Sending vehicles info to prolog
                        facts = f"vehicle(v{vId}, {vLane}, {X_pos:.4f}, {Y_pos:.4f}, {v_pos[2]:.4f}, {v_pos[3]:.4f}, {v_vel[0]:.4f}, {v_vel[1]:.4f}, 100).\n"
                        f.write(facts)
                    else:
                        pygame.draw.rect(screen, WHITE, v_pos_pygame)

                # Sending Ego info
                EgoLane = get_lane(agent.y, LANES_Y)
                EgoFact = f"vehicle(ego, {EgoLane}, {agent.x:.4f}, {agent.y:.4f}, {agent.width:.4f}, {agent.height:.4f}, {agent.velocity_x:.4f}, {agent.velocity_y:.4f}, 100).\n"
                f.write(EgoFact)
                f.close()

            # reconsult the prolog file to load clauses for finding safe actions
            # you should add 'reconsult' command like 'consult' in pyswip file -> find prolog.py in pyswip installed directory
            prolog.reconsult('prolog files/choosing_actions_highway.pl')
            Action = list(prolog.query('safe_actions(Action)'))
            L = list(prolog.query('possible_actions(Actions)'))
            safeActions = []
            for action in L[0]['Actions']:
                safeActions.append(str(action))
            # Desired_velocity_x = list(prolog.query('desired_velocity_x(Vd_x)'))
            Acceleration_x = list(prolog.query('acceleration_x(Ax)'))

            # print(f">>> Safe action: {Action[0]['Action']}.")
            print(f">>> Safe action set: {safeActions}.")

            agent.safe_action = Action[0]['Action']
            agent.possible_actions = safeActions
            # agent.possible_actions = ['lane_keeping','left_lane_change','right_lane_change']

            # agent.V_x_desired = Desired_velocity_x[0]['Vd_x']
            agent.Acceleration_x = Acceleration_x[0]['Ax']
            # print(f">>> Desired Velocity x: {Desired_velocity_x[0]['Vd_x']}.", end="\n")
            # print(list(prolog.query('listing(vehicle)')),end='\n\n')

            EgoRect, EgoColor = agent.rect, agent.color
            pygame.draw.rect(screen, EgoColor, EgoRect)
            pygame.display.update()
            clock.tick(40)
            # time.sleep(1)

        # update the agent car
        done, score = agent.update(timestep, neighboring_vehicles_pos_pygame, neighboring_vehicles_pos, X_vel,
                                   train=True)

        # end the episode if done
        if done:

            print(f'Episode {episode + 1} is done !===============================')

            # frame = 0
            # compare this episodes and the current best
            # episode's score
            if score >= best_score:
                best_score = score  # make best episode this episode
                agent.dq_agent.net.save_weights("weights/best_weights_" + str(best_score) + '_' + str(timestr))

            avg_velocity = np.average(velocities)
            avg_velocities.append(avg_velocity)
            velocities = []

            avg_loss = np.average(agent.dq_agent.losses)
            avg_losses.append(avg_loss)
            writer.add_scalar("Loss", avg_loss, episode)
            writer.add_scalar("Score", score, episode)
            writer.add_scalar("Velocities", avg_velocity, episode)

            agent.dq_agent.losses = []
            scores.append(score)

            print(f'Number of hits: {agent.n_hits}')

            # Total steps
            Steps.append(steps)

            # Append rewards into lists
            R_lc.append(agent.lane_change_reward_sum)
            R_v.append(agent.velocity_reward_sum)
            R_c.append(agent.collision_reward_sum)
            R_out.append(agent.out_of_highway_reward_sum)
            R_end.append(agent.end_of_lane_reward_sum)
            R_t.append(agent.time_waste_reward_sum)

            # r_sum = agent.lane_change_reward_sum+agent.collision_reward_sum+agent.out_of_highway_reward_sum \
            #         +agent.end_of_lane_reward_sum+agent.time_waste_reward_sum

            R_avg.append(agent.avg_rewards)

            # Number of hits
            n_hits.append(agent.n_hits)

            # create dataframe to save rewards in excel
            df = pd.DataFrame()

            # specify column name to each list
            df['steps'] = Steps
            df['R_lc'] = R_lc
            df['R_v'] = R_v
            df['R_c'] = R_c
            df['R_out'] = R_out
            df['R_end'] = R_end
            df['R_t'] = R_t
            df['R_avg'] = R_avg
            df['n_hits'] = n_hits
            df['Loss'] = avg_losses

            # Save dataframe to excel
            if int((episode + 1) % WEIGHT_SAVE_STEPS) == 0:
                df.to_excel('dqn_exported_data.xlsx')

            episode += 1
            steps = 0

            # make the summation of each reward subfunction zero
            agent.reset_rewards()
            agent.n_hits = 0

        frame += 1
        # restart
        if frame >= len(frame_list):
            print(frame)
            frame = 0

        # save
        if episode % WEIGHT_SAVE_STEPS == 0:
            agent.dq_agent.net.save_weights("weights/weights_" + str(timestr))

        # time.sleep(1)

    writer.close()
    f = open("result_collision.txt", "a")
    f.write("Number of Collision: " + str(agent.n_hits) + "\n")
    f.close()

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

    # plot average velocities
    plt.plot(avg_velocities)
    plt.title('Average Velocities')
    plt.xlabel('Episode')
    plt.ylabel('Velocity(avg)')
    plt.grid()
    plt.savefig('img/fig_avg_velocities.png')
    plt.show()

#    plotData( avg_velocities, 'Average Velocities', 'Episode', 'Velocity(avg)', 'fig_avg_velocities.png' )
