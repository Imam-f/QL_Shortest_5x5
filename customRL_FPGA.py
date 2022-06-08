import pygame
import math
import numpy as np
import random
import time

import serial
from pynq import PL
from pynq import Overlay, MMIO, Clocks, allocate
import os, warnings
# from IPython import get_ipython
import matplotlib.pyplot as plt

########################################################

WIDTH = 120
ROWS = 5      # down
COLS = 6      # side

WIN = pygame.display.set_mode((WIDTH*COLS, WIDTH*ROWS))
pygame.display.set_caption("REINFORCEMENT LEARNING MAZE FINDER")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Spot:
    def __init__(self,  col, row, width, total_rows, total_cols):
        self.row = row
        self.col = col
        self.x = col * width
        self.y = row * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.total_cols = total_cols

    def get_pos(self):
        return self.row, self.col

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = GREEN 

    def make_white(self):
        self.color = WHITE
    
    def make_red(self):
        self.color = RED

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def __lt__(self, other):
        return False

def make_grid(rows, cols, width):
    grid = []
    gap = width
    for i in range(rows):
        grid.append([])
        for j in range(cols):
            spot = Spot(j, i, gap, rows, cols)
            grid[i].append(spot)
    return grid

def draw_grid(win, rows, cols, width):
    gap = width
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width*COLS, i * gap))
    for j in range(cols):
        pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width*ROWS))

def draw(win, grid, rows, cols, width):
    win.fill(WHITE)

    for col in grid:
        for spot in col:
            spot.draw(win)

    draw_grid(win, rows, cols, width)
    pygame.display.update()

def get_clicked_pos(pos, width):
    gap = width
    x, y = pos

    row = y // gap
    col = x // gap

    # print(f"[{y},{x}]")
    # print(f"<{row},{col}>")
    return col, row

def info_state(env, row, col):
    if((0<row<=ROWS) and (0<col<=COLS)):
        in_state = True
        if (env[row][col] == 2):
            reward = -100
        elif (env[row][col] == 3):
            reward = 100
        else:
            reward = 0
    else:
        reward = -100
        in_state = False
    return reward, in_state

def q_learning(q_table, env, start_state, start_row, start_col, grid, win, width):
    exploration_rate = 1
    learning_rate = 0.5
    discount_rate = 0.9

    start_pos = start_state;
    end_pos = 0;
    obstacles = list()

    for k,v in enumerate(env):
        for k1,v1 in enumerate(v):
            if(env[k][k1] == 2):
                obstacles.append(k*COLS + k1)
            elif(env[k][k1] == 3):
                end_pos = k*COLS + k1

    # q_table = np.zeros((total_state, total_action))
    learning_trial = 1;
    while learning_trial:
    
        print("One")
        ol = Overlay("/home/xilinx/jupyter_notebooks/RTL/design.bit")
        print("two")

        Action = 0x46000000
        Nextstate = 0x42000000
        Valid = 0x44000000
        Reward = 0x40000000
        Control = 0x43C00000

        ADDRESS_RANGE_8 = 0x2000
        ADDRESS_RANGE_64 = 0x10000



        act_bram = MMIO(Action, ADDRESS_RANGE_8)
        nxt_bram = MMIO(Nextstate, ADDRESS_RANGE_8)
        vld_bram = MMIO(Valid, ADDRESS_RANGE_8)
        rwd_bram = MMIO(Reward, ADDRESS_RANGE_8)
        ctrl_bram = MMIO(Control, ADDRESS_RANGE_64)



        #reset
        ctrl_bram.write(0, 1)
        ctrl_bram.write(0, 0)
        #start
        ctrl_bram.write(1*4, 0)

        #alpha
        ctrl_bram.write(5*4, 0x4000FFFF)
        #gamma
        ctrl_bram.write(6*4, 0x7000FFFF)

        #max eps
        ctrl_bram.write(7*4, 1000)
        #max step
        ctrl_bram.write(8*4, 15)
        #start state
        ctrl_bram.write(9*4, start_pos)
        #end state
        ctrl_bram.write(10*4, end_pos)
        #invalid penalty
        ctrl_bram.write(11*4, 50 * 2**16)
        #number of state
        ctrl_bram.write(25*4, ROWS*COLS)

        #read seed
        ctrl_bram.write(2*4, 0)
        #epsilon
        ctrl_bram.write(4*4, 8)
        #rng seed
        ctrl_bram.write(3*4, 675)
        ctrl_bram.write(2*4, 1)
        ctrl_bram.write(2*4, 0)
        ctrl_bram.write(3*4, 0)

        #reward
        #next statet
        #valid action
        #initialization


        """
        Reward logic
        """
        for i in range(0,25*4,4):
            rwd_bram.write(i, 0)

        # Punishment
        for i in obstacles:
            rwd_bram.write(i*4, -100 * 2**16)

        # Reward
        rwd_bram.write(end_pos*4, 100 * 2**16)


        """
        Next state logic
        Valid transtition logic
        """
        for i in range(ROWS * COLS):
            for j in range(4):
                address = i*16 + j*4
                if (j == 0):
                    nextstate = i - COLS;
                elif (j == 1):
                    nextstate = i + 1;
                elif (j == 2):
                    nextstate = i + COLS;
                else:
                    nextstate = i - 1;

                if (nextstate < 0):
                    isValid = 0;
                elif (nextstate > ((ROWS * COLS) - 1)):
                    isValid = 0;
                elif ((i % COLS == (COLS - 1)) and (j == 1)):
                    isValid = 0;
                elif ((i % COLS == 0) and (j == 3)):
                    isValid = 0;
                else:
                    isValid = 1;

                if (isValid):
                    nxt_bram.write(address, nextstate)
                    vld_bram.write(address, 1)
                else:
                    nxt_bram.write(address, i)
                    vld_bram.write(address, 0)

        """
        Action Bram setting
        """
        for i in range(0, 25*4*4, 4):
            act_bram.write(i, 0)

        start = time.time()
        ctrl_bram.write(1*4, 1)
        while(not ctrl_bram.read(12*4)):
            pass
        end = time.time()

        time1 = (end - start) * 1000
        print("Time : %.2f ms" % (time1))


        Q_Table = list();
        for i in range(0,25*4,4):
            print(act_bram.read(i*4), act_bram.read(i*4+4), act_bram.read(i*4+8), act_bram.read(i*4+12))
            Q_Table.append([act_bram.read(i*4), act_bram.read(i*4+4), act_bram.read(i*4+8), act_bram.read(i*4+12)])


        print("[")
        for i,j in enumerate(Q_Table):
            for k,l in enumerate(j):
                if(Q_Table[i][k] & 0x80000000):
                    Q_Table[i][k] = -0x100000000 + Q_Table[i][k]
                Q_Table[i][k] = (Q_Table[i][k] / 2**16)
            print(Q_Table[i])
        print("]")


        print(Q_Table)
        for k,v in enumerate(q_table):
            for k1,v1 in enumerate(v):
                q_table[k][k1] = Q_Table[k][k1]
        
        if(q_table[k].sum() < 1):
            learning_trial = 0 
        time.sleep(1)

    return q_table
    

def main(win, width):    
    grid = make_grid(ROWS, COLS, width)
    env = np.zeros((ROWS, COLS))
    total_state=ROWS*COLS
    total_action=4
    
    q_table = np.zeros((total_state, total_action))
    
    start = None
    end = None

    run = True
    while run:

        draw(win, grid, ROWS, COLS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print(q_table)
                run = False

            if pygame.mouse.get_pressed()[0]: # LEFT
                pos = pygame.mouse.get_pos()
                col, row = get_clicked_pos(pos, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                    print("Bikin Start")
                    start_row = row
                    start_col = col
                    start_state = (start_row*ROWS) + (start_col)
                    print("State : "+str(start_state))
                    print("(x,y):("+str(start_row)+","+str(start_col)+")")
                    
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                    finish = (col*ROWS)+(row)
                    
                    # end = 3
                    env[row][col] = 3

                elif spot != end and spot != start:
                    spot.make_barrier()

                    # barrier = 2
                    env[row][col] = 2
                #print(env)

            elif pygame.mouse.get_pressed()[2]: # RIGHT
                pos = pygame.mouse.get_pos()
                col, row = get_clicked_pos(pos, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

                # no interest = 0
                env[row][col] = 0
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    print(q_table)
                if event.key == pygame.K_SPACE and start and end:
                    q_table = q_learning(q_table, env, start_state, start_row, start_col, grid, win, width)
                    print(q_table)
                    pos_row = start_row
                    pos_col = start_col
                    pos_row_temp = pos_row
                    pos_col_temp = pos_col
                    state = start_state
                    list_path = []
                    obs = 0
                    found_path = False

                    while not(found_path):
                        for event in pygame.event.get():
                            if event.type == pygame.QUIT:
                                pygame.quit()
                                
                        action = np.argmax(q_table[state][:])
                        action_temp = action

                        if action==0:
                            pos_row_temp -= 1
                        elif action==1:
                            pos_row_temp += 1
                        elif action==2:
                            pos_col_temp -= 1
                        else:
                            pos_col_temp += 1

                        reward_end, in_state_end = info_state(env, pos_row_temp, pos_col_temp)
                        while(reward_end < 0):
                            obs+=1
                            if (obs>15):
                                break
                            pos_row_temp = pos_row
                            pos_col_temp = pos_col
                            action = random.randrange(0,4)
                            while action == action_temp:
                                action = random.randrange(0,4)
                            action_temp = action
                            if action==0:
                                pos_row_temp -= 1
                            elif action==1:
                                pos_row_temp += 1
                            elif action==2:
                                pos_col_temp -= 1
                            else:
                                pos_col_temp += 1

                            reward_end, in_state_end = info_state(env, pos_row_temp, pos_col_temp)

                        pos_row = pos_row_temp
                        pos_col = pos_col_temp
                        list_path.append((pos_row, pos_col))

                        state = (pos_row)*ROWS + pos_col
                        if(state==finish or obs>15):
                            found_path = True
                        else:
                            spot = grid[pos_col][pos_row]
                            spot.make_red()
                            draw(win, grid, ROWS, COLS, width)
                            time.sleep(0.2)
                            spot.make_path()

                elif event.key == pygame.K_s and start and end:    
                    pos_row = start_row
                    pos_col = start_col
                    pos_row_temp = pos_row
                    pos_col_temp = pos_col
                    state = start_state
                    list_path = []
                    obs = 0
                    found_path = False

                    while not(found_path):
                        for event in pygame.event.get():
                            if event.type == pygame.QUIT:
                                pygame.quit()
                                
                        action = np.argmax(q_table[state][:])
                        action_temp = action

                        if action==0:
                            pos_row_temp -= 1
                        elif action==1:
                            pos_row_temp += 1
                        elif action==2:
                            pos_col_temp -= 1
                        else:
                            pos_col_temp += 1

                        reward_end, in_state_end = info_state(env, pos_row_temp, pos_col_temp)
                        while(reward_end < 0):
                            obs+=1
                            if (obs>15):
                                break
                            pos_row_temp = pos_row
                            pos_col_temp = pos_col
                            action = random.randrange(0,4)
                            while action == action_temp:
                                action = random.randrange(0,4)
                            action_temp = action
                            if action==0:
                                pos_row_temp -= 1
                            elif action==1:
                                pos_row_temp += 1
                            elif action==2:
                                pos_col_temp -= 1
                            else:
                                pos_col_temp += 1

                            reward_end, in_state_end = info_state(env, pos_row_temp, pos_col_temp)

                        pos_row = pos_row_temp
                        pos_col = pos_col_temp
                        list_path.append((pos_row, pos_col))

                        state = (pos_row)*ROWS + pos_col
                        if(state==finish or obs>15):
                            found_path = True
                        else:
                            spot = grid[pos_col][pos_row]
                            spot.make_red()
                            draw(win, grid, ROWS, COLS, width)
                            time.sleep(0.2)
                            spot.make_path()

    pygame.quit()

main(WIN, WIDTH)
