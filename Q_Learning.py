"""
Python code for MBKM RL SoC
"""
import random
import matplotlib.pyplot as plt

# Q learning parameter
alpha = 0.5
gamma = 0.875


# Initialize Q table
#####################
Q_table = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]

# fill Q table
for i in range(25):
    for j in range(4):   
       Q_table[i][j] = 0 # random.random()



def print_Qtable(Q_table): ####################################
    """
    Function to print Q table
    param : Q_table
    return : void
    """
    print("\nState\t\tRight\t\t\tUp\t\t\tLeft\t\t\tDown")
    print("-----\t\t-----\t\t\t-----\t\t\t-----\t\t\t-----")

    for s in range(25):
        if(Q_table[s][0] >= 90):
            print(s+1, "\t", Q_table[s][0], "\t\t\t", Q_table[s][1], "\t", Q_table[s][2], "\t", Q_table[s][3])
        else:
            print(s+1, "\t", Q_table[s][0], "\t", Q_table[s][1], "\t", Q_table[s][2], "\t", Q_table[s][3])
    s += 1

    print(s+1, "\t", Q_table[s][0], "\t\t\t", Q_table[s][1], "\t\t\t", Q_table[s][2], "\t\t\t", Q_table[s][3], "\n")



def Search_Location(maze):
    """
    A function to convert maze state to object location
    param : maze location
    return : linear position, position x, position y
    """
    for i in range(5):
        for j in range(5):
            if maze[i][j] == 1:
                return (5 * i + j), i, j



def Action(mazes, Q_tables, epsilons):
    """
    A function to take action based on epsilon delta and Q matrix
    param : position in maze, Q table, epsilon
    return : action, new position
    """
    out = 0

    while out == 0:
        # Find location in maze
        state, state_i, state_j = Search_Location(mazes)

        rdm = random.random()

        # Pick action based random number epsilon yields
        if epsilons <= rdm:
            ########################
            max_value = max(Q_tables[state][0], Q_tables[state][1], Q_table[state][2], Q_table[state][3]) 
            
            for j in range(4):
                #######################
                if max_value == Q_tables[state][j]:
                    act = j
        else:
            random_value = random.random()

            if random_value <= 0.25:
                act = 0
            elif random_value <= 0.5:
                act = 1
            elif random_value <= 0.25:
                act = 2
            else:
                act = 3

        # Next state given certain action
        if act == 0:
            state_j += 1
        elif act == 1:
            state_i -= 1 
        elif act == 2:
            state_j -= 1 
        else:
            state_i += 1

        # Check if exit map
        # If yes, retake action
        if state_i == -1:
            out = 0 
        elif state_j == -1:
            out = 0
        elif state_i == 5:
            out = 0
        elif state_j == 5:
            out = 0
        else:
            out = out + 1

    # Calculate new position in maze
    new_mazes = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    new_mazes[state_i][state_j] = 1
    return act, new_mazes



def Routing(Q_table):
    """
    A function to show fastest route based on Q table
    param : Q table
    return : void
    """
    print("Route")
    s = 0
    route = list()

    for i in range(25):
        # Error if find obstacle
        if s==4 or s==6 or s==7 or s==13 or s==16 or s==18 or s==19 or s==21:
            print("error")
            break

        route.append(s)
        # Success if find end
        if s == 24:
            print("S", s, " Goal!", sep="")
            break
       
        # Print current state
        print("S", s, end=" ", sep="")

        # Find next step
        #######################################
        M = max(Q_table[s][0], Q_table[s][1], Q_table[s][2], Q_table[s][3])
        I = 0
        while Q_table[s][I] != M:   ##############
            I += 1
        I += 1

        # Take next step
        if I == 1:
            if s%5 != 4:
                s = s+1
            else:
                print("error")
                break
        elif I == 2:
            if s > 4:
                s = s-5
            else:
                print("error")
                break
        elif I == 3:
            if s%5 != 0:
                s = s-1
            else:
                print("error")
                break
        elif I == 4:
            if s < 20:
                s = s+5
            else:
                print("error")
                break
    if i == 25:
        print("error")
    return route


def update_Qvalue(states, new_states, Q_tabless, acts, alphas, gammas, t):
    """ 
    A function to find reward and update Q function based on action
    param : linear position, new state, Q table, action, alpha, gamma, time step
    return : Q table, new state
    """

    # Calculate rewards
    if new_states == 24:
        reward = 100
    elif t == 15:     
        reward = -50 
    elif new_states == 4:
        reward = -50
    elif new_states == 6:
        reward = -50
    elif new_states == 7:
        reward = -50
    elif new_states == 13:
        reward = -50
    elif new_states == 16:
        reward = -50
    elif new_states == 18:
        reward = -50
    elif new_states == 19:
        reward = -50
    elif new_states == 21:
        reward = -50
    else:
        reward = 0 
    
    # Update Q table
    ##################################################
    before = Q_tabless[states][acts]
    beforetable = Q_tabless[new_states]
    Q_tabless[states][acts] = (1 - alphas) * Q_tabless[states][acts] + alphas * (reward + gammas * max(Q_tabless[new_states][0], Q_tabless[new_states][1], Q_tabless[new_states][2], Q_tabless[new_states][3]))

    return Q_tabless, new_states, (Q_tabless[states][acts], before, beforetable), reward, ########################




# Print state before learning
print("\nQ-Table and route before learning")
print_Qtable(Q_table)
Routing(Q_table)


# Learning algorithm
for Episode in range(5):
    print(f"Episode {Episode}")

    # Initial maze
    maze = [[1, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    
    # Decaying Epsilon
    epsilon = 0.5 # 1 - (Episode / 301)

    # Steps in episode
    for i in range(6):

        # Find location
        state, state_i, state_j = Search_Location(maze)
        
        # Finish condition
        if state == 24:
            break
        
        # Take action based on state
        actss, new_maze = Action(maze, Q_table, epsilon)

        # Move to next state
        new_state, new_state_i, new_state_j = Search_Location(new_maze)
        
        # Update Q value based on rewards
        Q_table, new_state, (Qnew, before, beforetable), rewards = update_Qvalue(state, new_state, Q_table, actss, alpha, gamma, i)
        
        # Store new state
        maze = new_maze


# Print result
print("\nQ-Table and route after learning")
print_Qtable(Q_table)
Routing(Q_table)



def Visualize(Q_table):
    line = Routing(Q_table)

    map = [[0 for k in range(5)] for l in range(5)]

    obstacles = [4, 6, 7, 13, 16, 18, 19, 21]
    for i in obstacles:
        map[(i)//5][(i)%5] = 2
    for i in line:
        map[(i)//5][(i)%5] = 4
    map[0][0] = 1;
    map[4][4] = 3;
    
    fig, ax = plt.subplots()
    im1 = ax.imshow(map)
    
    plt.show()

Visualize(Q_table)
