import pygame
import math
import numpy as np
import random
import time
WIDTH = 600
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("REINFORCEMENT LEARNING MAZE FINDER")

ROWS = 5

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
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

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

def make_grid(rows, width):
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid


def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()


def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col

def info_state(env, row, col):
    if((0<row<=ROWS) and (0<col<=ROWS)):
        in_state = True
        if (env[row-1][col-1] == 2):
            reward = -10
        elif (env[row-1][col-1] == 3):
            reward = 100
        else:
            reward = 0
    else:
        reward = -10
        in_state = False
    return reward, in_state

def move(state, action, env, row, col):
    new_row = row
    new_col = col
    if(action == 0): #UP
        new_row = row - 1
    elif (action == 1): #DOWN
        new_row = row + 1
    elif (action == 2): #LEFT
        new_col = col - 1
    else: #RIGHT
        new_col = col + 1
    reward, in_state = info_state(env, new_row, new_col)

    if (in_state and reward >= 0):
        state = (new_row-1)*ROWS + new_col
    else:
        new_row = row
        new_col = col

    if reward > 0:
        done = True
    else:
        done = False
    return state, reward, done, new_row, new_col, in_state

def determine_state(state, old_state, reward):
    if(reward==0):
        new_state = state
    else:
        new_state = old_state
    return new_state

def q_learning(q_table, env, start_state, start_row, start_col, grid, win, width):
	exploration_rate = 1
	learning_rate = 0.5
	discount_rate = 0.9
	num_episode = ROWS*ROWS*10
	end_step = ROWS*ROWS
	for episode in range(num_episode):
		if (episode%2 == 0):
			row = random.randrange(1, ROWS)
			col = random.randrange(1, ROWS)
			reward_q, in_state_q = info_state(env, row, col)
			while(reward_q < 0):
				row = random.randrange(1, ROWS)
				col = random.randrange(1, ROWS)
				reward_q, in_state_q = info_state(env, row, col)
			state = (row-1)*ROWS + col
			
		else:
			row = start_row+1
			col = start_col+1
			state = start_state
		done = False
		step=0
		print("-------Episode " + str(episode) +"---------")
		while not(done) and step <= end_step:
			step += 1
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					print(q_table)
					pygame.quit()
			choose_action = random.random()
			if(choose_action >= exploration_rate):
				action = np.argmax(q_table[state-1][:])
			else:
				action = random.randrange(0,4)

			new_state, reward, done, row, col, in_state = move(state, action, env, row, col)
			
			if (reward <= 0 and step > end_step-2):
				reward = -50
			if(in_state):
				q_table[state-1][action]=(q_table[state-1][action]*(1-learning_rate))+(learning_rate*(reward+discount_rate*np.max(q_table[new_state-1][:])))
			else:
				q_table[state-1][action]=(q_table[state-1][action]*(1-learning_rate))+(learning_rate*(reward))
			state = determine_state(new_state, state, reward)
			
		exploration_rate = 1-(episode/(num_episode+1))
	return q_table

def main(win, width):
	grid = make_grid(ROWS, width)
	env = np.zeros((ROWS, ROWS))
	total_state=ROWS*ROWS
	total_action=4
	q_table = np.zeros((total_state, total_action))
	
	start = None
	end = None

	run = True
	while run:

		draw(win, grid, ROWS, width)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				print(q_table)
				run = False

			if pygame.mouse.get_pressed()[0]: # LEFT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				if not start and spot != end:
					start = spot
					start.make_start()
					print("Bikin Start")
					start_row = col
					start_col = row
					start_state = (start_row*ROWS)+(start_col+1)
					print("State : "+str(start_state))
					print("(x,y):("+str(start_row)+","+str(start_col)+")")
					

				elif not end and spot != start:
					end = spot
					end.make_end()
					finish = (col*ROWS)+(row+1)
					env[col][row]=3

				elif spot != end and spot != start:
					spot.make_barrier()
					env[col][row]=2
				#print(env)

			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None
				elif spot == end:
					end = None
				env[col][row]=0
				
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_1:
					print(q_table)
				if event.key == pygame.K_SPACE and start and end:
					q_table = q_learning(q_table, env, start_state, start_row, start_col, grid, win, width)
					print(q_table)
					pos_row = start_row+1
					pos_col = start_col+1
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
								
						action = np.argmax(q_table[state-1][:])
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

						state = (pos_row-1)*ROWS + pos_col
						if(state==finish or obs>15):
							found_path = True
						else:
							spot = grid[pos_col-1][pos_row-1]
							spot.make_red()
							draw(win, grid, ROWS, width)
							time.sleep(0.2)
							spot.make_path()

				elif event.key == pygame.K_s and start and end:	
					pos_row = start_row+1
					pos_col = start_col+1
					pos_row_temp = pos_row
					pos_col_temp = pos_col
					state = start_state
					print("new state :" +str(state))
					list_path = []
					obs = 0
					found_path = False
					while not(found_path):
						for event in pygame.event.get():
							if event.type == pygame.QUIT:
								pygame.quit()
								print(q_table)
						print("Find Path")
						action = np.argmax(q_table[state-1][:])
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
						state = (pos_row-1)*ROWS + pos_col
						if(state==finish or obs>15):
							found_path = True
						else:
							spot = grid[pos_col-1][pos_row-1]
							spot.make_red()
							draw(win, grid, ROWS, width)
							time.sleep(0.2)
							spot.make_path()
	pygame.quit()

main(WIN, WIDTH)
