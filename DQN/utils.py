import math
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
import numpy as np
import pygame
import sys
import os
from pygame.locals import *
import time

#from train import N_EPISODES

PLOT_INTERPOLATE = 10

# =====================================================================
def get_distance( p1, p2 ):
    return math.sqrt( (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 )

def get_lane(y, LANES_Y):

    if LANES_Y[0] <= y <= LANES_Y[1]:
        return 1
    elif LANES_Y[1]< y <=LANES_Y[2]:
        return 2
    elif LANES_Y[2]< y <=LANES_Y[3]:
        return 3
    elif       65  < y <=LANES_Y[4]:
        return 4
    elif LANES_Y[4]< y <=LANES_Y[5]:
        return 5
    elif LANES_Y[5]< y <=LANES_Y[6]:
        return 6
    else:
        return 0

# =====================================================================
def truncate(number, decimals=0):
    """
    Returns a value truncated to a specific number of decimal places.
    """
    if not isinstance(decimals, int):
        raise TypeError("decimal places must be an integer.")
    elif decimals < 0:
        raise ValueError("decimal places has to be 0 or more.")
    elif decimals == 0:
        return math.trunc(number)

    factor = 10.0 ** decimals
    return math.trunc(number * factor) / factor


# =====================================================================
def plotData( data, title, xlabel, ylabel, fig_name ):
    
    n_episodes = len(data)
    x = np.arange( len( data ) )

    x_ = np.arange( int(n_episodes / PLOT_INTERPOLATE) if n_episodes > PLOT_INTERPOLATE else n_episodes)
    
    #define x as 200 equally spaced values between the min and max of original x 
 
    
    #define spline
    spl = make_interp_spline(x, data, k=3)
    y_smooth = spl(x_)
    
    #create smooth line chart 
    n = x_ * PLOT_INTERPOLATE if n_episodes > PLOT_INTERPOLATE else x_
    plt.plot( n, y_smooth)
    plt.title( title )
    plt.xlabel( xlabel )
    plt.ylabel( ylabel )
    plt.grid()
    plt.savefig( fig_name )
    plt.show()
    
def showCars(pygame_rectlist):
    RED = (255, 0, 0)
    WHITE = (255, 255, 255)

    pygame.init()  # initialize pygame
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((1022, 136))

    # Load the background image here. Make sure the file exists!
    bg = pygame.image.load("HighD/Images/25_highway.jpg")
    pygame.mouse.set_visible(1)
    pygame.display.set_caption('Highway')


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    # screen.fill(WHITE)
    screen.blit(bg, (0, 0))

    for carRect in pygame_rectlist:
        pygame.draw.rect(screen, WHITE, carRect)

    pygame.display.update()
    clock.tick(40)
    time.sleep(0.01)

    pygame.quit()

