# -*- coding: utf-8 -*-
"""
Created on Sat Mar 20 21:51:17 2021

@author: divyam
"""
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

########## D* Lite #############
global map
map_cost = {}
global map_key
map_key = {}
## global variables ## [g,rhs,heuristic]

def obstacle(x,y):
    if x>10 and x < 13 and y > 10 and y < 13:
        return True
    if (x - 8)**2 + (y - 7)**2 <= 4:
        return True
    return False

### key ----> [min(g,rhs) + h + k, min(g,rhs)]
############################################################################
    
def initialize(map_cost,map_key,height,width,goal):  # takes in goal to initialise rhs(goal) = 0
    global INF
    INF = 201
    for i in range(height):
        for j in range(width):
            map_cost[(i,j)] = [INF-1,INF-1,INF-1] ## g,rhs,h
            map_key[(i,j)] = [INF-1,INF-1] ## key
    map_cost[goal] = [INF-1,0,INF-1]
    #queue.append(key) ###  list/priority_queue
    return map_cost, map_key
############################################################################
    
def hueristic(start,current):
    h = math.sqrt((start[0] - current[0])**2 + (start[1] - current[1])**2)
    return h
#############################################################################
    
def updatedge(obstacle_node,queue):
    
    node = obstacle_node
    x,y = node[0],node[1]
    
################## update rhs value of neighbours ##########################
        
    '''update rhs value of children (predecessors)'''
    children = []
    move = ['up','down','right','left','upright','downright','upleft','downleft']
    for child in move:
        a = movement[child][0] + x 
        b = movement[child][1] + y
        if obstacle(a,b):
            continue
        if a > 0 and a < 20 and b > 0 and b < 20:
            map_cost[(a,b)][1] = INF
            children.append((a,b))
    ##############################################

    ########### key into queue ###################
    for child in children:
        if map_cost[child][0] != map_cost[child][1]:            ## inconsistent (i.e g != rhs)
            map_key[child] = calculateKey(map_cost[child][0],map_cost[child][1],map_cost[child][2],km)
            queue.put((map_key[child],child))
    return queue
############################################################################
    
def calculateKey(g,rhs,h,km):
    key = [min(g,rhs) + h + km, min(g,rhs)]
    return key
############################################################################

movement = {'up': (0,1), 'down' : (0,-1), 'left': (-1,0), 'right' : (1,0), 'upright' : (1,1), 'downright' : (1,-1), 'upleft':(-1,1), 'downleft':(-1,-1) }

#################### compute shortest distance #############################
def computeShortestPath(start,map_cost,map_key):  ## simple backward A*
    
    count = 0
    visited = []
    
    queue = PriorityQueue()
    queue.put((map_key[goal],goal))
    node = goal
    movement = {'up': (0,1), 'down' : (0,-1), 'left': (-1,0), 'right' : (1,0), 'upright' : (1,1), 'downright' : (1,-1), 'upleft':(-1,1), 'downleft':(-1,-1) }
    
    while map_cost[start][0] != map_cost[start][1] or map_key[node] < map_key[start] and map_cost[start][0] != INF and map_cost[start][1] != INF:
        #print(count)
        count += 1
        ''' Update State '''
        ##################################################################
        node = queue.get()[1]                              # store coords tuple
        if map_cost[node][0] > map_cost[node][1]:
            map_cost[node][0] = map_cost[node][1]          ## g value = rhs value after dequeing

        elif map_cost[node][0] < map_cost[node][1]:
            map_cost[node][0] = INF                        ## g = infinity if g < rhs

        else:
            continue
        ##################################################################
        #if map_cost[node][0] == map_cost[node][1] and map_cost[node][0] != INF:
        visited.append(node)
        x,y = node[0], node[1]

        ############## actions ##################
        '''update rhs value of children (predecessors)'''
        children = []
        move = ['up','down','right','left','upright','downright','upleft','downleft']
        for child in move:
            if child == 'up' or child == 'down' or child == 'left' or child == 'right':
                cost = 1
            if child == 'upright' or child == 'downright' or child == 'upleft' or child == 'downleft':
                cost = 1.42
            a = movement[child][0] + x 
            b = movement[child][1] + y
            if obstacle(a,b):
                cost = INF   #### make final map_cost[(a,b)] as INF
            if a > 0 and a < 20 and b > 0 and b < 20:
                if child not in visited:
                    if map_cost[node][0] + cost < map_cost[(a,b)][1]:        ## comparison with previous cost
                        map_cost[(a,b)][1] = min(map_cost[a+1,b][0],map_cost[a-1,b][0],map_cost[a,b+1][0],map_cost[a,b-1][0],map_cost[a+1,b+1][0],map_cost[a+1,b-1][0],map_cost[a-1,b+1][0],map_cost[a-1,b-1][0]) + cost
                        map_cost[(a,b)][2] = hueristic(node,start)           ## start node is our goal here
                        children.append((a,b))
        ##############################################
        
        ########### key into queue ###################
        for child in children:
            if child not in visited:
                if map_cost[child][0] != map_cost[child][1]:            ## inconsistent (i.e g != rhs)
                    map_key[child] = calculateKey(map_cost[child][0],map_cost[child][1],map_cost[child][2],km)
                    queue.put((map_key[child],child))
        ########### if the node is still inconsistent ############
        map_key[node] = calculateKey(map_cost[node][0],map_cost[node][1],map_cost[node][2],km)
        if map_cost[node][0] != map_cost[node][1]:
            queue.put((map_key[node],node))

    print(len(visited))
    print(map_key)
    
    return map_cost,queue,visited


####################### main #############################################

height = 21
width = 21
start = (2,2)
goal = (19,19)
map_cost = {}
map_key = {}
map_key_dup = {}

map_cost, map_key = initialize(map_cost,map_key,height,width,goal)

map_cost[start][1] = INF-2  ## new_INF = INF - 1 -----> start_rhs = new_INF - 1
queue =[]  # [node coordinate,key]

km = 0
h = hueristic(start,goal)
map_key[goal] = calculateKey(map_cost[goal][0],map_cost[goal][1],h,km)


queue.append([goal,map_key[goal]])
curr_key = queue[0][1]

map_cost, map_key, visited = computeShortestPath(start,map_cost,map_key)


########################## Animation #######################################
fig,ax = plt.subplots()

def animate(i):
    if i < len(visited):
        x = visited[i][0]
        y = visited[i][1]
        plt.scatter(x,y)
        
animation = FuncAnimation(fig, func = animate, interval = 0.1)
fig.show()
plt.draw()
plt.show()


