import numpy as np
import math

MAX_ITER = 100


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
    for i in range(0, MAX_ITER):
        found, solution = search(board, goal, i)
        if found:
            return solution
    return []


def search(board, goal, max_iterations):
    found, solution = False, []
    return found, solution


def heuristic(current_pos, goal_pos):
    """ Manhattan distance """
    return abs(current_pos[0] - goal_pos[0]) + abs(current_pos[1] - goal_pos[1])
    

def main():
    n, length, zero_goal_position = read_input()
    goal_state = calculate_goal_state(length, zero_goal_position, n)
    board = init_board(length)
    iterative_deepening_a_star(board, goal_state)

    print(board)
    print(goal_state)
    print(get_neighbours(length, find_element_position(board, 0)))


if __name__ == "__main__":
    main()
