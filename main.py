import math
import time
import heapq
import numpy as np
from copy import deepcopy


class Node:
    def __init__(self, f_, board_):
        self.f = f_
        self.board = board_
        
    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f
    
    def __eq__(self, other):
        return tuple(self.board.flatten()) == tuple(other.board.flatten())
    
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


def iterative_deepening_a_star(board):
    max_fscore = 100
    limit_fscore = get_heuristic(board)
    path = [board]
    while True:
        # print("Trying with:", limit_fscore)
        fscore = search(path, 0, limit_fscore)
        if fscore == 0:
            return True, path
        if fscore == math.inf:
            return False, []
        limit_fscore = fscore
        if limit_fscore >= max_fscore:
            return False, []
        

def search(path, steps, max_fscore):   
    state = path[-1]
    h = get_heuristic(state)
    fscore = steps + h

    if fscore > max_fscore:
        return fscore
    if h == 0:
        return 0
    
    min = math.inf
    for s in successors(state, steps):
        if not is_visited(path, s.board):
            path.append(s.board)
            f = search(path, steps + 1, max_fscore)
            if f == 0:
                return 0
            if f < min:
                min = f
            path.pop()
    return min
                

def successors(board, steps):
    zero_position = find_element_position(board, 0)
    possible_routes = get_neighbours(len(board[0]), zero_position)
    states = swap(board, possible_routes, zero_position)
    ss = [Node(get_heuristic(state)+steps, state) for state in states]
    heapq.heapify(ss)
    return ss


def is_visited(path, p):
    pf = p.flatten()
    for step in path:
        if len([i for i, j in zip(step.flatten(), pf) if i == j]) == len(pf):
            return True
    return False


def swap(board, possible_routes, zero_position):
    states = []
    for route in possible_routes:
        state = deepcopy(board)
        state[route[0]][route[1]], state[zero_position[0]][zero_position[1]] = state[zero_position[0]][zero_position[1]], state[route[0]][route[1]]
        states.append(state)
    return states
   

def get_heuristic(board):  
    h = 0
    for i in range(0, len(board)):
        for j in range(0, len(board[i])):
            if board[i][j] == 0:
                continue
            h += heuristic([i, j], find_element_position(goal_state, board[i][j]))
    
    return h


def heuristic(current_pos, goal_pos):
    """ Manhattan distance """
    return abs(current_pos[0] - goal_pos[0]) + abs(current_pos[1] - goal_pos[1])

    
goal_state = ''


def main():
    n, length, zero_goal_position = read_input()
    
    global goal_state 
    goal_state = calculate_goal_state(length, zero_goal_position, n)
    
    board = init_board(length)
    
    start = time.time()
    found, path = iterative_deepening_a_star(board)
    end = time.time()
    
    print("Time: ", end - start)
    print("Found solution:", found)
    if len(path)-1 >= 0:
        print("Solution with length: ", len(path)-1)
        print("Solution steps:")
        for i in range(1, len(path)):
            prev = find_element_position(path[i-1], 0)
            curr = find_element_position(path[i], 0)
            if curr[0] > prev[0]:
                print("up")
            if curr[0] < prev[0]:
                print("down")
            if curr[1] > prev[1]:
                print("left")
            if curr[1] < prev[1]:
                print("right")
            

if __name__ == "__main__":
    main()
