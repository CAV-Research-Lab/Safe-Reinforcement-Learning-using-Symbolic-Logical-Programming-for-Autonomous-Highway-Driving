import csv
from decimal import Decimal
import os
import pygame
import time
import random
import agent
import matplotlib.pyplot as plt
import numpy as np
from utils import get_distance, get_lane
from pyswip import Prolog
prolog = Prolog()

LANES_Y = [25, 36, 48, 81, 93, 108, 116]

LEARN = False # enable learn if true

# agent details
# need to retrain after changing those
WIDTH = 15
HEIGHT = 8

# Solid Colors
RED = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

INIT_TRACK = 5
INIT_SPEED = 70
INIT_DIRECTION = 1

# =====================================================================
def init_pygame():
    x = 0
    y = 300
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)
    pygame.init()

    X = 1366
    Y = 118
    ds = pygame.display.set_mode((X, Y))
    pygame.display.set_caption('Image')

    return ds

# =====================================================================
class ScenarioData():
    def __init__(self, map_id, timestep, road_length, road_width, X, Y):

        self.timestep = timestep
        self.road_length = road_length
        self.road_width = road_width
        self.X = X
        self.Y = Y
        self.recordingMeta = csv.DictReader(open('../11/'+str(map_id) + '_recordingMeta.csv')).__next__()
        self.tracks = csv.DictReader(open('../11/'+str(map_id) + '_tracks.csv'))
        self.tracksMeta = csv.DictReader(open('../11/'+str(map_id) + '_tracksMeta.csv'))
        self.running = True
        self.timestep = 1/25
        self.start_time = 0
        self.t_count = 1
        self.white = (255, 255, 255)
        self.image = pygame.image.load(r'../11/11_highway.jpg')
        self.image = pygame.transform.scale(self.image, (X, Y))

        # add the agent car
        self.agent = agent.AgentCar( width=15, height=8, lane=4, speed=72, direction=1, eps=0.0 )

    def init_scenario(self):
        print('scenario initializing, please wait')
        self.frame_list = []
        self.frame_len = int(Decimal(self.recordingMeta['duration']) * int(self.recordingMeta['frameRate']))
        for i in range(self.frame_len):
            self.frame_list.append([])
        i_len = 0

        for i in self.tracksMeta:
            i_len = i_len + int(i['numFrames'])

        i = 0

        for row in self.tracks:
            self.frame_list[int(row['frame'])-1].append(row)
            if i%10000 == 0:
#                print('scenario initializing, please wait',i,'/',i_len)
                pass
            i = i + 1

        self.numVehicles = int(self.recordingMeta['numVehicles'])
        self.vehicle_list = []

        for i in range(self.numVehicles):
            self.vehicle_list.append(pygame.Color(int(random.random()*256), int(random.random()*256), int(random.random()*256)))

    def plot_scenario(self, screen):
        print('scenario start')
        self.start_time = time.time()
        avg_losses = []

        i = 0

        while self.running:
            screen.fill(self.white)
            screen.blit(self.image, (0, 0))
            vehicles_pos = []
            vehicles_vel = []
            vehicles_id = []
            vehicles_lane = []
            X_vel = []

#            try:
            for row in self.frame_list[self.t_count - 1]:
                a = float(row['x']) * self.X / self.road_length
                b = (float(row['y'])+0.2) * self.Y / self.road_width
                w = float(row['width']) * self.X / self.road_length
                h = float(row['height']) * self.Y / self.road_width
                velocity_x = float(row['xVelocity']) * self.X / self.road_length
                velocity_y = float(row['yVelocity']) * self.Y / self.road_width

                lane = get_lane(b, LANES_Y)
                vehicles_id.append(row['id'])
                vehicles_lane.append(lane)
                vehicles_pos.append((a, b, w, h))
                vehicles_vel.append((velocity_x, velocity_y))
                X_vel.append(velocity_x)

            vehicle_pos_pygame = [pygame.Rect(rect) for rect in vehicles_pos]
            agent.AgentCar.max_velocity_other_cars = max(X_vel)

            x_ego, y_ego = self.agent.x, self.agent.y 

            # prolog.query("retractall(vehicle)")
            with open('prolog files/vehicle_clauses.pl','w') as f:
                for vId, vLane, v_pos, v_pos_pygame, v_vel in zip(vehicles_id, vehicles_lane, vehicles_pos, vehicle_pos_pygame, vehicles_vel):
                    d = get_distance([x_ego,y_ego], [v_pos[0],v_pos[1]])
                    if d < 150 and 4 <= int(vLane) <= 6:
                        pygame.draw.rect(screen, RED, v_pos_pygame)

                        # Sending vehicles info to prolog
                        facts = f"vehicle(v{vId}, {vLane}, {v_pos[0]:.4f}, {v_pos[1]:.4f}, {v_vel[0]:.4f}, {v_vel[1]:.4f}, 100).\n"
                        f.write(facts)
                    else:
                        pygame.draw.rect(screen, WHITE, v_pos_pygame)

                # Sending Ego info
                EgoLane = get_lane(self.agent.y, LANES_Y)
                EgoFact = f"vehicle(ego, {EgoLane}, {self.agent.x:.4f}, {self.agent.y:.4f}, {self.agent.velocity_x:.4f}, {self.agent.velocity_y:.4f}, 100).\n"
                f.write(EgoFact)
                f.close()

            # reconsult the prolog file to load clauses for finding safe actions
            # you should add 'reconsult' command like 'consult' in pyswip file -> find prolog.py in pyswip installed directory
            prolog.reconsult('prolog files/choosing_actions_highway.pl')
            # Action = list(prolog.query('safe_actions(Action)'))
            # L = list(prolog.query('possible_actions(Actions)'))
            # Actions = []
            # for action in L[0]['Actions']:
            #     Actions.append(str(action))
            Desired_velocity_x = list(prolog.query('desired_velocity_x(Vd_x)'))

            # print(f">>> Safe action: {Action[0]['Action']}.")
            # print(f">>> Possible actions: {Actions}.")
            # print(f">>> Desired Velocity x: {Desired_velocity_x[0]['Vd_x']}.", end="\n")

            # self.agent.safe_action = Action[0]['Action']
            # self.agent.possible_actions = Actions
            self.agent.V_x_desired = Desired_velocity_x[0]['Vd_x']

            i += 1

            # update and draw the agent
            reset, _  = self.agent.update( timestep, vehicle_pos_pygame, vehicles_pos, X_vel )
            self.agent.draw( screen )

            avg_losses.append( np.average( self.agent.dq_agent.losses ) )
            self.agent.dq_agent.losses = []

            pygame.display.update()
            if i==int(self.frame_len):
                pygame.QUIT
                self.running=False
                print("Number of collision:",self.agent.n_hits)
                f = open("result_collision_test.txt", "a+")
                f.write("Number of Collision_Test: "+str(self.agent.n_hits)+"\n")
                f.close()
                self.agent.dq_agent.net.save_weights("weights/test_weights")
                plt.plot(avg_losses)
                plt.title( 'Loss' )
                plt.xlabel( 'Episode')
                plt.ylabel( 'Loss' )
                plt.grid()
                plt.savefig( 'fig_Q_validateion_Loss.png' )
                #plt.show()

            for event in pygame.event.get() :
                if event.type == pygame.QUIT :
                    self.running = False
                    print("Number of collision:",self.agent.n_hits)

                    # save weight file
                    if LEARN:
                        self.agent.dq_agent.net.save_weights("weights/weights")
                        # plot the losses
                        plt.plot(avg_losses)
                        plt.title( 'Loss' )
                        plt.xlabel( 'Episode')
                        plt.ylabel( 'Loss' )
                        plt.grid()
                        plt.savefig( 'fig_Q_validateion_Loss.png' )
                        plt.show()

            self.sleep()
            self.t_count = self.t_count + 1

            if self.t_count == len( self.frame_list ):
                self.t_count = 1

        pygame.quit()

    def sleep(self):
        local_time = time.time() - self.start_time
        sleep_time = self.t_count * self.timestep - local_time
        if sleep_time > 0.1:
            sleep_time = 0.1
        if sleep_time > 0:
            time.sleep(sleep_time)

    def close(self):
        self.running = False

if __name__ == '__main__':
    timestep = 1/25
    road_length, road_width, X, Y = 420, 36.12, 1366, 118
    map_id = 11
    screen = init_pygame()
    scenario_data = ScenarioData(map_id, timestep, road_length, road_width, X, Y)
    scenario_data.init_scenario()
    scenario_data.plot_scenario(screen)
