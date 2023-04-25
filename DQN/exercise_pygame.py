import pygame
import sys
import os
from pygame.locals import *
import time

RED = (255, 0, 0)
WHITE = (255, 255, 255)
X, Y = 1366, 118
pygame.init()  # initialize pygame
clock = pygame.time.Clock()
screen = pygame.display.set_mode((X, Y))

# Load the background image here. Make sure the file exists!
bg = pygame.image.load("HighD/highway.jpg")
bg = pygame.transform.scale(bg, (X, Y))
pygame.mouse.set_visible(1)
pygame.display.set_caption('Highway')


# fix indentation

i = 0
while True:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    # screen.fill((255,255,255))
    screen.blit(bg, (0,0))
    rect = pygame.Rect(50+i, 85, 30, 10)
    pygame.draw.rect(screen, RED, rect)

    rect1 = pygame.Rect(1300-2*i, 35, 20, 10)
    pygame.draw.rect(screen, WHITE, rect1)
    y = 115
    pygame.draw.line(screen, RED, (20,y), (100,y))

    i+=1

    pygame.display.update()
    clock.tick(40)
    time.sleep(0.01)

pygame.quit()