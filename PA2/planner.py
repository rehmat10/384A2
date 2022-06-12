#Look for ### IMPLEMENT BELOW ### tags in this file. These tags indicate what has
#to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os
# Search engines
from search import * 
# Warehouse specific classes
from warehouse import WarehouseState, Direction, warehouse_goal_state

def heur_displaced(state):
  '''A trivial example heuristic that is admissible'''
  '''INPUT: a warehouse state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  '''In this case, simply the number of displaced boxes.'''   
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
    return count

def heur_manhattan_distance(state):

    '''admissible heuristic: manhattan distance'''
    '''INPUT: a warehouse state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.


    ### IMPLEMENT BELOW ###
    sum = 0
    for box in state.boxes:
        min_dist = -1
        if box not in state.storage:
            for storage in state.storage:
                temp = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                if min_dist == -1 or temp < min_dist:
                    min_dist = temp
            sum += min_dist
    ### END OF IMPLEMENTATION ###
    
    return sum


def fval_fn(state, weight, heuristic):
    return state.gval + (1/weight - 1) * heuristic(state)


def general_goal_fn(state):
    storage_spaces = []
    for storage in state.storage:
        storage_spaces.append(storage)
    for box in state.boxes:
        if box in storage_spaces:
            storage_spaces.remove(box)
        else:
            return False
    return True


def weighted_astar(initial_state, heuristic, weight, timebound = 10):

    '''Provides an implementation of weighted a-star, as described in the PA2 handout'''
    '''INPUT: a warehouse state that represents the start state, the heursitic to be used,'''
    '''       weight for the A* search (w >= 1), and a timebound (number of seconds)'''
    '''OUTPUT: A WarehouseState (if a goal is found), else False'''

    search_eng = SearchEngine('custom', 'default')
    wrap_fval_fn = (lambda sN: fval_fn(sN, weight, heuristic))
    search_eng.init_search(initial_state, general_goal_fn, heuristic, wrap_fval_fn)
    return search_eng.search(timebound)[0]




def iterative_astar(initial_state, heuristic, weight, timebound = 10):

    '''Provides an implementation of iterative a-star, as described in the PA2 handout'''
    '''INPUT: a warehouse state that represents the start state, the heursitic to be used,'''
    '''       weight for the A* search (w >= 1), and a timebound (number of seconds)'''
    '''OUTPUT: A WarehouseState (if a goal is found), else False'''
    
    # HINT: Use os.times()[0] to obtain the clock time. Your code should finish within the timebound.'''
    init_time = os.times()[0]
    decrement = weight / 10
    best_cost = None
    cur_weight = weight
    time_left = timebound
    while time_left > 0 and cur_weight >= 0:
        search_eng = SearchEngine('custom', 'default')
        wrap_fval_fn = (lambda state: fval_fn(state, cur_weight, heuristic))
        search_eng.init_search(initial_state, general_goal_fn, heuristic, wrap_fval_fn)
        result = search_eng.search(time_left)[0]
        if not result:
            return result
        cur_time = os.times()[0]
        if best_cost is None or best_cost.gval > result.gval:
            best_cost = result

        time_left = timebound - (cur_time - init_time)
        if time_left <= 0:
            return best_cost
        cur_weight -= decrement

    if best_cost is None:
        return False
    return best_cost


def heur_alternate(state):

    '''a better warehouse heuristic'''
    '''INPUT: a warehouse state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
  
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    sum = 0
    for box in state.boxes:
        min_dist = -1
        if box not in state.storage:
            for storage in state.storage:
                temp = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                if min_dist == -1 or temp < min_dist:
                    min_dist = temp

            #cost for closest robot to get to box
            close_robot_dist = -1
            for robot in state.robots:
                temp = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
                if close_robot_dist == -1 or temp < close_robot_dist:
                    close_robot_dist = temp

            sum += min_dist
            sum += close_robot_dist - 1

    return sum

