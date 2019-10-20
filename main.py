import math
import time
import heapq
import numpy as np
from copy import deepcopy


class Node:
    def __init__(self, h_, board_):
        self.h = h_
        self.board = board_
        
    def __lt__(self, other):
        return self.h < other.h
    
    def __eq__(self, other):
        return self.board == other.board
    
    def __hash__(self):
        return hash(frozenset(self.board))
            
    
def read_input():
    n = int(input("Enter the number of entries:")) + 1
    length = int(math.sqrt(n))

    print("Enter the position of 0 (default is ", length - 1, length - 1, "): ")
    zero_goal_position = list(map(int, input().split()))
    if zero_goal_position == [-1]:
        zero_goal_position = [length - 1, length - 1]

    print("Zero goal position is: ", zero_goal_position)
    return n, length, zero_goal_position


def init_board(length):
    print("Enter the entries in a single line (separated by space): ")
    entries = list(map(int, input().split()))
    board = np.array(entries).reshape(length, length)
    return board


def get_neighbours(length, zero_position):
    neighbours = []
    if zero_position[0] + 1 < length:
        neighbours.append([zero_position[0] + 1, zero_position[1]])
    if zero_position[0] - 1 >= 0:
        neighbours.append([zero_position[0] - 1, zero_position[1]])
    if zero_position[1] + 1 < length:
        neighbours.append([zero_position[0], zero_position[1] + 1])
    if zero_position[1] - 1 >= 0:
        neighbours.append([zero_position[0], zero_position[1] - 1])
    return neighbours


def find_element_position(board, element):
    for i in range(0, len(board)):
        for j in range(0, len(board[i])):
            if board[i][j] == element:
                return [i, j]
    raise Exception("element ", element, "not found in ", board)


def calculate_goal_state(length, zero_goal_position, n):
    goal_state = np.array([0 for i in range(0, n)]).reshape(length, length)
    value = 1
    for i in range(0, len(goal_state)):
        for j in range(0, len(goal_state)):
            if i == zero_goal_position[0] and j == zero_goal_position[1]:
                continue
            else:
                goal_state[i][j] = value
                value += 1
    return goal_state


def iterative_deepening_a_star(board, goal):
    max_fscore = 100
    limit_fscore = get_heuristic(board, goal)
    path = [board]
    while True:
        # print("Trying with:", limit_fscore)
        found, fscore = search(goal, path, 0, limit_fscore)
        if found:
            return True, path, limit_fscore
        if fscore == math.inf:
            return False, [], 0
        limit_fscore = fscore
        if limit_fscore >= max_fscore:
            return False, [], limit_fscore
        

def search(goal, path, g, max_fscore):   
    # print("PATH: ", path)    
    state = path[-1]
    fscore = g + get_heuristic(state, goal)
    if fscore > max_fscore:
        return False, fscore
    if np.array_equal(state, goal):
        return True, max_fscore
    
    min = math.inf
    for s in successors(state, goal):
        if not is_visited(path, s.board):
            path.append(s.board)
            found, f = search(goal, path, g + 1, max_fscore)
            if found:
                return True, max_fscore
            if f < min:
                min = f
            path.pop()
    return False, min
                

def successors(board, goal):
    zero_position = find_element_position(board, 0)
    possible_routes = get_neighbours(len(board[0]), zero_position)
    states = swap(board, possible_routes, zero_position)
    ss = [Node(get_heuristic(state, goal), state) for state in states]
    heapq.heapify(ss)
    # print("SS")
    # for s in ss:
    #     print(s.board)
    # print()
    return ss


def is_visited(path, p):
    for step in path:
        if np.array_equal(step, p):
            return True
    return False


def swap(board, possible_routes, zero_position):
    states = []
    for route in possible_routes:
        state = deepcopy(board)
        state[route[0]][route[1]], state[zero_position[0]][zero_position[1]] = state[zero_position[0]][zero_position[1]], state[route[0]][route[1]]
        states.append(state)
    return states
   

heuristics = {}


def get_heuristic(board, goal):
    b = tuple(board.flatten())
    if b in heuristics:
        return heuristics.get(b)
    
    h = 0
    for i in range(0, len(board)):
        for j in range(0, len(board[i])):
            h += heuristic([i, j], find_element_position(goal, board[i][j]))
    
    heuristics[b] = h 
    return h


def heuristic(current_pos, goal_pos):
    """ Manhattan distance """
    return abs(current_pos[0] - goal_pos[0]) + abs(current_pos[1] - goal_pos[1])
    

def main():
    n, length, zero_goal_position = read_input()
    goal_state = calculate_goal_state(length, zero_goal_position, n)
    board = init_board(length)
    start = time.time()
    found, path, f = iterative_deepening_a_star(board, goal_state)
    end = time.time()
    print("Time: ", end - start)
    print("Found solution:", found)
    # print(f)
    print("Solution with length: ", len(path)-1)
    print("Solution steps:")
    for i in range(1, len(path)):
        prev = find_element_position(path[i-1], 0)
        curr = find_element_position(path[i], 0)
        if curr[0] > prev[0]:
            print("down")
        if curr[0] < prev[0]:
            print("up")
        if curr[1] > prev[1]:
            print("left")
        if curr[1] < prev[1]:
            print("right")
            

if __name__ == "__main__":
    main()
