import pygame
import sys
import os
from pygame.locals import *
import time
from utils import get_lane

e_i_y = 0
def y_PI_Control(y, y_desired):
    global e_i_y
    Kp, Ki = 10, 5
    e_p = y_desired - y
    e_i_y += e_p*dt
    u = Kp*e_p+Ki*e_i_y

    return u

RED = (255, 0, 0)
WHITE = (255, 255, 255)
X, Y = 1366, 118
pygame.init()  # initialize pygame
clock = pygame.time.Clock()
screen = pygame.display.set_mode((X, Y))

# Load the background image here. Make sure the file exists!
bg = pygame.image.load("../HighD/highway.jpg")
bg = pygame.transform.scale(bg, (X, Y))
pygame.mouse.set_visible(1)
pygame.display.set_caption('Highway')


# fix indentation

i = 0
dt = 0.04
y = 75
x = 50
while True:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    # screen.fill((255,255,255))
    screen.blit(bg, (0,0))

    if 0<=i< 20:
        y_desired = 75
    elif 20<=i<50:
        y_desired = 95
    elif 50<=i<80:
        y_desired = 75
    elif 80<=i<110:
        y_desired = 95
    else:
        y_desired = 75

    v_x = 100
    v_y = y_PI_Control(y, y_desired)

    x = x+v_x*dt
    y = y+v_y*dt

    rect = pygame.Rect(x, y, 30, 10)
    pygame.draw.rect(screen, RED, rect)

    # rect1 = pygame.Rect(1300-2*i, 35, 20, 10)
    # pygame.draw.rect(screen, WHITE, rect1)

    # intersection = rect1.clip(rect)
    # print(intersection.w)

    i+=1

    pygame.display.update()
    clock.tick(40)
    time.sleep(0.01)

pygame.quit()