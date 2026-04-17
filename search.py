#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: sonia martinez
"""

# Please do not distribute or publish solutions to this
# exercise. You are free to use these problems for educational purposes, please refer to the source.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

from mazemods import maze
from mazemods import makeMaze
from mazemods import collisionCheck
from mazemods import makePath
from mazemods import getPathFromActions
from mazemods import getCostOfActions
from mazemods import stayWestCost
from mazemods import stayEastCost




def depthFirstSearch(xI,xG,n,m,O):
  """
  Search the deepest nodes in the search tree first.
 
  Your search algorithm needs to return a list of actions
  and a path that reaches the goal.  
  Make sure to implement a graph search algorithm.
  Your algorithm also needs to return the cost of the path. 
  Use the getCostOfActions function to do this.
  Finally, the algorithm should return the number of visited
  nodes in your search.
 
  """
  "*** YOUR CODE HERE ***"
  stack= [xI]
  visited=set()
  visited.add(xI)
  parent = {xI: None}
  parentAction = {xI:None}
  U = [(1,0),(-1,0),(0,1),(0,-1)]  # legal controls
  while stack:
    s= stack.pop()
    if s ==xG:
        actions = []
        curr = s
        while parentAction[curr] is not None:
          actions.append(parentAction[curr])
          curr=parent[curr]
        actions.reverse()
        cost= getCostOfActions(xI,actions,O)
        return actions, cost, len(visited)
    
    for u in U:
         if not collisionCheck(s, u, O):
             sNext = (s[0] + u[0], s[1] + u[1])
             if sNext not in visited:
              visited.add(sNext)
              parent[sNext] = s
              parentAction[sNext] = u
              stack.append(sNext)
  return [], 0, len(visited)  # FAILURE
  
def breadthFirstSearch(xI,xG,n,m,O):
  """
  Search the shallowest nodes in the search tree first [p 85].
 
  Your search algorithm needs to return a list of actions
  and a path that reaches the goal. Make sure to implement a graph 
  search algorithm.
  Your algorithm also needs to return the cost of the path. 
  Use the getCostOfActions function to do this.
  Finally, the algorithm should return the number of visited
  nodes in your search.

  """
  "*** YOUR CODE HERE ***"
  queue= [xI]
  visited=set()
  visited.add(xI)
  parent = {xI: None}
  parentAction = {xI:None}
  U = [(1,0),(-1,0),(0,1),(0,-1)]  # legal controls
  while queue:
    s= queue.pop(0)
    if s ==xG:
        actions = []
        curr = s
        while parentAction[curr] is not None:
          actions.append(parentAction[curr])
          curr=parent[curr]
        actions.reverse()
        cost= getCostOfActions(xI,actions,O)
        return actions, cost, len(visited)
    
    for u in U:
         if not collisionCheck(s, u, O):
             sNext = (s[0] + u[0], s[1] + u[1])
             if sNext not in visited:
              visited.add(sNext)
              parent[sNext] = s
              parentAction[sNext] = u
              queue.append(sNext)
  return [], 0, len(visited)  # FAILURE
  

def DijkstraSearch(xI,xG,n,m,O,cost=stayWestCost):
  """
  Search the nodes with least cost first. 
  
  Your search algorithm needs to return a list of actions
  and a path that reaches the goal. Make sure to implement a graph 
  search algorithm.
  Your algorithm also needs to return the total cost of the path using
  either the stayWestCost or stayEastCost function.
  Finally, the algorithm should return the number of visited
  nodes in your search.
  """
  "*** YOUR CODE HERE ***"
  frontier= [(0,xI)] # Cost and State List
  visited=set()
  g={xI:0}
  parent = {xI: None}
  parentAction = {xI:None}
  U = [(1,0),(-1,0),(0,1),(0,-1)]  # legal controls
  while frontier:
    min_idx = 0
    for i in range(len(frontier)):
       if frontier[i][0] < frontier[min_idx][0]:
          min_idx=i
    cost_s,s=frontier.pop(min_idx)
    if s in visited:
       continue
    visited.add(s)
    if s == xG:        # same as BFS
        actions = []
        curr = s
        while parentAction[curr] is not None:
            actions.append(parentAction[curr])
            curr = parent[curr]
        actions.reverse()
        cost = getCostOfActions(xI, actions, O)
        return actions, g[xG], len(visited)

    for u in U:
        if not collisionCheck(s, u, O):
            sNext = (s[0] + u[0], s[1] + u[1])
            new_cost = g[s] +cost(s,[u],O)    # step 13: compute cost to reach sNext
            if sNext not in g or new_cost < g[sNext]:  # step 13: only update if cheaper
                g[sNext] = new_cost
                parent[sNext] = s
                parentAction[sNext] = u
                frontier.append((new_cost, sNext))     # CHANGED: store cost with state

  return [], 0, len(visited)
  


def nullHeuristic(state,goal):
   """
   A heuristic function estimates the cost from the current state to the nearest
   goal.  This heuristic is trivial.

   """
   return 0

def manhattanHeuristic(state,goal):
   """
   A heuristic function estimates the cost from the current state to the nearest
   goal.  This heuristic is trivial.

   """
   return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

def euclideanHeuristic(state,goal):
   """
   A heuristic function estimates the cost from the current state to the nearest
   goal.  This heuristic is trivial.

   """
   return ((state[0] - goal[0])**2 + (state[1] - goal[1])**2)**0.5

def aStarSearch(xI,xG,n,m,O,heuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  """The function uses a function heuristic as an argument. We have used
  the null heuristic here first, you should redefine heuristics as part of 
  the homework. 
  Your algorithm also needs to return the total cost of the path using
  getCostofActions functions. 
  Finally, the algorithm should return the number of visited
  nodes during the search."""
  "*** YOUR CODE HERE ***"
  frontier= [(0,xI)] # Cost and State List
  visited=set()
  g={xI:0}
  parent = {xI: None}
  parentAction = {xI:None}
  U = [(1,0),(-1,0),(0,1),(0,-1)]  # legal controls
  while frontier:
    min_idx = 0
    for i in range(len(frontier)):
       if frontier[i][0] < frontier[min_idx][0]:
          min_idx=i
    cost_s,s=frontier.pop(min_idx)
    if s in visited:
       continue
    visited.add(s)
    if s == xG:        # same as BFS
        actions = []
        curr = s
        while parentAction[curr] is not None:
            actions.append(parentAction[curr])
            curr = parent[curr]
        actions.reverse()
        return actions, g[xG], len(visited)

    for u in U:
        if not collisionCheck(s, u, O):
            sNext = (s[0] + u[0], s[1] + u[1])
            new_cost = g[s] + 1    # step 13: compute cost to reach sNext
            if sNext not in g or new_cost < g[sNext]:  # step 13: only update if cheaper
                g[sNext] = new_cost
                parent[sNext] = s
                parentAction[sNext] = u
                frontier.append((new_cost + heuristic(sNext,xG), sNext))     # CHANGED: store cost with state

  return [], 0, len(visited)

  

    
# Plots the path
def showPath(xI,xG,path,n,m,O):
    gridpath = makePath(xI,xG,path,n,m,O)
    fig, ax = plt.subplots(1, 1) # make a figure + axes
    ax.imshow(gridpath) # Plot it
    ax.invert_yaxis() # Needed so that bottom left is (0,0)
     
if __name__ == '__main__':
    # Run test using smallMaze.py (loads n,m,O)
    #from smallMaze import *
    from mediumMaze import *  # try these mazes too
    #from bigMaze import *     # try these mazes too
    #maze(n,m,O) # prints the maze
    
    # Sample collision check
    #x, u = (5,4), (1,0)
    #testObs = [[6,6,4,4]]
    #collided = collisionCheck(x,u,testObs)
    #print('Collision!' if collided else 'No collision!')
    
    # Inital and Goal States
    xI = (1,1)
    xG = (34,16)
    #xG = (20,1)
    
    #actions = [(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(0,1),
    #           (1,0),(1,0),(1,0),(0,-1),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0)]
    #path = getPathFromActions(xI,actions)
    #showPath(xI,xG,path,n,m,O)
    
    #DFS Path and Cost
    actions_dfs, cost_dfs, nodes_dfs = depthFirstSearch(xI, xG, n, m, O)
    path_dfs = getPathFromActions(xI, actions_dfs)
    showPath(xI, xG, path_dfs, n, m, O)
    simplecostdfs = getCostOfActions(xI,actions_dfs,O)
    print('DFS Actions:', actions_dfs)
    print('DFS Cost:', cost_dfs)
    print('Nodes Explored:', nodes_dfs)
    plt.title("DFS Path")
    
    #BFS Path and Cost
    actions_bfs, cost_bfs, nodes_bfs = breadthFirstSearch(xI, xG, n, m, O)
    path_bfs = getPathFromActions(xI, actions_bfs)
    showPath(xI, xG, path_bfs, n, m, O)
    simplecostbfs = getCostOfActions(xI,actions_bfs,O)
    print('BFS Actions:', actions_bfs)
    print('BFS Cost:', cost_bfs)
    print('Nodes Explored:', nodes_bfs)
    plt.title("BFS Path")
    
    # Call Dijkstra with west cost
    actions_dijk_w, cost_dijk_w, nodes_dijk_w = DijkstraSearch(xI, xG, n, m, O, stayWestCost)
    path_dijk_w = getPathFromActions(xI, actions_dijk_w)
    showPath(xI, xG, path_dijk_w, n, m, O)
    plt.title("Dijkstra West Cost Path")

    # Call Dijkstra with east cost
    actions_dijk_e, cost_dijk_e, nodes_dijk_e = DijkstraSearch(xI, xG, n, m, O, stayEastCost)
    path_dijk_e = getPathFromActions(xI, actions_dijk_e)
    showPath(xI, xG, path_dijk_e, n, m, O)
    plt.title("Dijkstra East Cost Path")
    
    # Cost of Dijkstra West
    westcostdij = stayWestCost(xI,actions_dijk_w,O)
    eastcostdij = stayEastCost(xI,actions_dijk_e,O)
    print('Dijkstra stay west cost was %d and stay east cost was %d' %
          (westcostdij,eastcostdij))

    # Call A* Manhattan
    actions_astar, cost_astar, nodes_astar = aStarSearch(xI, xG, n, m, O, manhattanHeuristic)
    path_Astar=getPathFromActions(xI, actions_astar)
    showPath(xI, xG, path_Astar, n, m, O)
    plt.title("$A^*$ Cost Path with Manhattan")
    

    # Call A* Euclidean
    actions_astare, cost_astare, nodes_astare = aStarSearch(xI, xG, n, m, O, euclideanHeuristic)
    path_Astare=getPathFromActions(xI, actions_astare)
    showPath(xI, xG, path_Astare, n, m, O)
    plt.title("$A^*$ Cost Path with Euclidean")
    

    # Cost of A* 
    A_man=getCostOfActions(xI,actions_astar,O)
    A_euc=getCostOfActions(xI,actions_astare,O)
    print('A* with Manhattan Heuristic cost was %d. Nodes Explored: %d.' %
          (A_man, nodes_astar))
    print('A* with Euclidean Heuristic cost was %d. Nodes Explored: %d.' %
          (A_euc,nodes_astare))
    
    plt.show()