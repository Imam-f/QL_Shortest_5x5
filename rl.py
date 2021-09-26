import pygame
import random 
rdm = 0
reward = 0

#initialize pygame
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
pygame.init()

#background
bgnd = pygame.image.load('background2.png')

#create the screen
screen = pygame.display.set_mode((600, 600))

#Title and Icon
pygame.display.set_caption("Reinforcement Learning")

#player
agenIcon = pygame.image.load('walk.png')
demonIcon = pygame.image.load('demon.png')
goldIcon = pygame.image.load('ingots.png')

agenX = 0
agenY = 0
goldX = 480
goldY = 480


def agen(x, y):
    screen.blit(agenIcon, (x, y))

def demon(x):
    if x == 1:
        screen.blit(demonIcon, (480, 0)) #
    elif x == 2:
        screen.blit(demonIcon, (120, 120)) #
    elif x == 3:
        screen.blit(demonIcon, (240, 120)) #
    elif x == 4:
        screen.blit(demonIcon, (360, 240)) #
    elif x == 5:
        screen.blit(demonIcon, (120, 360)) #
    elif x == 6:
        screen.blit(demonIcon, (360, 360)) #
    elif x == 7:
        screen.blit(demonIcon, (480, 360)) #
    elif x == 8:
        screen.blit(demonIcon, (120, 480)) #

def gold():
    screen.blit(goldIcon, (goldX, goldY))

def isCollision(x, y):
    if (x==120):
        if (y==120 or y==360 or y==480):
            return True
        else:
            return False
    elif (x==240):
        if (y==120):
            return True
        else:
            return False
    elif (x==360):
        if (y==240 or y==360):
            return True
        else:
            return False
    elif (x==480):
        if (y==0 or y==360):
            return True
        else:
            return False
    elif (x<0 or x>480 or y<0 or y>480):
        return True
    else:
        return False

def isGoal():
    if(agenX==480 and agenY==480):
        return True
    else:
        return False

#Game loop
running = True
while running:
    screen.fill(WHITE)
    agen(agenX, agenY)
    for i in range(8):
        demon(i)
    gold()
    screen.blit(bgnd, (0, 0))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_DOWN:
                agenY += 120
            if event.key == pygame.K_UP:
                agenY -= 120
            if event.key == pygame.K_LEFT:
                agenX -= 120
            if event.key == pygame.K_RIGHT:
                agenX += 120

    collision = isCollision(agenX, agenY)
    if collision:
        agenX=0
        agenY=0
        reward -= 1
        print("reward = ")
        print(reward)

    goal = isGoal()
    if goal:
        agenX=0
        agenY=0
        reward += 1
        print("reward = ")
        print(reward)

    pygame.display.update()
