import numpy as np
import time

import sys

goal_state = np.array([1, 2, 3, 4, 5, 6, 7, 8, 0]).reshape(3, 3)

class Node():
    def __init__(self,state,parent,action,depth,step_cost,path_cost,heuristic_cost):
        self.state = state
        self.parent = parent  # parent node
        self.action = action  # move up, left, down, right
        self.depth = depth  # depth of the node in the tree
        self.step_cost = step_cost  # g(n), the cost to take the step
        self.path_cost = path_cost  # accumulated g(n), the cost to reach the current node
        self.heuristic_cost = heuristic_cost  # h(n), cost to reach goal state from the current node

        # children node
        self.move_up = None
        self.move_left = None
        self.move_down = None
        self.move_right = None

    def print_path(self):
        # create FILO stacks to place the trace
        state_trace = [self.state]
        action_trace = [self.action]
        depth_trace = [self.depth]
        step_cost_trace = [self.step_cost]
        path_cost_trace = [self.path_cost]
        heuristic_cost_trace = [self.heuristic_cost]

        # add node information as tracing back up the tree
        while self.parent:
            self = self.parent
            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            step_cost_trace.append(self.step_cost)
            path_cost_trace.append(self.path_cost)
            heuristic_cost_trace.append(self.heuristic_cost)

        # print out the path
        step_counter = 0
        while state_trace:
            print('step', step_counter)
            print(state_trace.pop())
            print('action=', action_trace.pop(),
                  ', depth=', str(depth_trace.pop()),
                  ', step cost=', str(step_cost_trace.pop()),
                  ', total_cost=', str(path_cost_trace.pop() + heuristic_cost_trace.pop()))
            step_counter += 1

    # see if moving down is valid
    def try_move_down(self):
        # index of the empty tile
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[0] == 0:
            #you can move down when empty tile in first row
            return 0,0
        else:
            up_value = self.state[zero_index[0] - 1, zero_index[1]]
            # value of the upper tile
            new_state = self.state.copy()
            #copying the value from up to down
            new_state[zero_index[0], zero_index[1]] = up_value
            #making the value as 0
            new_state[zero_index[0] - 1, zero_index[1]] = 0
            return new_state, 1

    # see if moving right is valid
    def try_move_right(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[1] == 0:
            return 0,0
        else:
            left_value = self.state[zero_index[0], zero_index[1] - 1]  # value of the left tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = left_value
            new_state[zero_index[0], zero_index[1] - 1] = 0
            return new_state, 1

    # see if moving up is valid
    def try_move_up(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[0] == 2:
            return 0,0
        else:
            lower_value = self.state[zero_index[0] + 1, zero_index[1]]  # value of the lower tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = lower_value
            new_state[zero_index[0] + 1, zero_index[1]] = 0
            return new_state, 1

    # see if moving left is valid
    def try_move_left(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[1] == 2:
            return 0,0
        else:
            right_value = self.state[zero_index[0], zero_index[1] + 1]  # value of the right tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = right_value
            new_state[zero_index[0], zero_index[1] + 1] = 0
            return new_state, 1

        # return heuristic cost: number of misplaced tiles

    def h_misplaced_cost(self, new_state):
        cost = np.sum(new_state != goal_state) - 1  # minus 1 to exclude the empty tile
        if cost > 0:
            return cost
        else:
            return 0  # when all tiles matches

    # return heuristic cost: sum of Manhattan distance to reach the goal state
    def h_manhattan_cost(self, new_state):
        current = new_state
        # digit and coordinates they are supposed to be
        goal_position = {1: (0, 0),
                             2: (0, 1),
                             3: (0, 2),
                             4: (1, 0),
                             5: (1, 1),
                             6: (1, 2),
                             7: (2, 0),
                             8: (2, 1),
                             0: (2, 2)}
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i, j] != 0:
                    sum_manhattan += sum(abs(a - b) for a, b in zip((i, j), goal_position[current[i, j]]))
        return sum_manhattan

    def a_star_search(self, heuristic_function):
        start = time.time()
        queue = [(self,0)]
        queue__num_nodes_popped = 0
        queue_max_length = 1
        depth_queue = [(0,0)]
        if heuristic_function is 'ucs':
            path_cost_queue = [0]
        else:
            path_cost_queue = [(0,0)]
        visited = set([])

        while queue:
            queue = sorted(queue, key= lambda x: x[1])
            depth_queue = sorted(depth_queue , key= lambda x: x[1])

            if heuristic_function is 'ucs':
                path_cost_queue = sorted(path_cost_queue, key= lambda x : x)
                current_path_cost = path_cost_queue.pop(0)
            else:
                path_cost_queue = sorted(depth_queue, key= lambda x: x[1])
                current_path_cost = path_cost_queue.pop(0)[0]

            if len(queue) > queue_max_length:
                queue_max_length = len(queue)

            current_node = queue.pop(0)[0]
            queue__num_nodes_popped += 1
            current_depth = depth_queue.pop(0)[0]
            visited.add(tuple(current_node.state.reshape(1, 9)[0]))

            if np.array_equal(current_node.state, goal_state):
                current_node.print_path()
                print('Time performance:', str(queue__num_nodes_popped), 'nodes popped off the queue.')
                print('Space performance:', str(queue_max_length), 'nodes in the queue at its max.')
                print('Time spent: %0.2fs' % (time.time() - start))
                return True
            else:
                #TRY_MOVE_DOWN
                new_state, up_value = current_node.try_move_down()
                if up_value:
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + up_value
                        depth = current_depth+1
                        if heuristic_function is 'misplaced':
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0
                        total_cost = path_cost + h_cost
                        current_node.move_down = Node(state=new_state,
                                                      parent=current_node,
                                                      action='down',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)
                        queue.append((current_node.move_down,total_cost))
                        if h_cost == 0:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append(path_cost)
                        else:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))

                #TRY_MOVE_RIGHT
                new_state, up_value = current_node.try_move_right()
                if up_value:
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + up_value
                        depth = current_depth + 1
                        if heuristic_function is 'misplaced':
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0
                        total_cost = path_cost + h_cost
                        current_node.move_down = Node(state=new_state,
                                                      parent=current_node,
                                                      action='right',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)
                        queue.append((current_node.move_down, total_cost))
                        #depth_queue.append((current_depth + 1, total_cost))
                        if h_cost == 0:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append(path_cost)
                        else:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))

                # TRY_MOVE_UP
                new_state, up_value = current_node.try_move_up()
                if up_value:
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + up_value
                        depth = current_depth + 1
                        if heuristic_function is 'misplaced':
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0
                        total_cost = path_cost + h_cost
                        current_node.move_down = Node(state=new_state,
                                                      parent=current_node,
                                                      action='up',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)
                        queue.append((current_node.move_down, total_cost))
                        if h_cost == 0:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append(path_cost)
                        else:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))

                # TRY_MOVE_LEFT
                new_state, up_value = current_node.try_move_left()
                if up_value:
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:
                        path_cost = current_path_cost + up_value
                        depth = current_depth + 1
                        if heuristic_function is 'misplaced':
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0
                        total_cost = path_cost + h_cost
                        current_node.move_down = Node(state=new_state,
                                                      parent=current_node,
                                                      action='left',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)
                        queue.append((current_node.move_down, total_cost))
                        #depth_queue.append((current_depth + 1, total_cost))
                        if h_cost == 0:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append(path_cost)
                        else:
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))

if __name__ == "__main__":
    sys.stdout.flush()
    """
    print("Welcome to Siddharth Arun's 8-Puzzle Solver.       :GOAL:")
    print("Choose the following options:                      1 2 3")
    print("1. Default Puzzle                                  4 5 6")
    print("2. Custom Puzzle                                   7 8 0")
    choice = int(input())
    if choice == 2:
        print("Enter your puzzle, using a zero to represent the blank. Please only enter valid 8-puzzles. Enter the puzzle demilimiting the numbers with a space. RET only when finished.\n ")
        puzzle_row_one = input("Enter the first row: ")
        puzzle_row_two = input("Enter the second row: ")
        puzzle_row_three = input("Enter the third row: ")
        puzzle_row_one = puzzle_row_one.split()
        puzzle_row_two = puzzle_row_two.split()
        puzzle_row_three = puzzle_row_three.split()
        for i in range(0, 3):
            puzzle_row_one[i] = int(puzzle_row_one[i])
            puzzle_row_two[i] = int(puzzle_row_two[i])
            puzzle_row_three[i] = int(puzzle_row_three[i])
        user_puzzle = puzzle_row_one+puzzle_row_two+puzzle_row_three
        initial_state = np.array(user_puzzle).reshape(3,3)
    else:
        print("Choose among 1. easy, 2. medium, 3. hard")
        choice2 = input()
        if choice2 == 1:
            initial_state = np.array([1, 2, 3, 4, 8, 0, 7, 6,5]).reshape(3, 3)
        elif choice2 == 2:
            initial_state = np.array([2, 8, 1, 0, 4, 3, 7, 6, 5]).reshape(3, 3)
        else:
            initial_state = np.array([5, 6, 7, 4, 0, 8, 3, 2, 1]).reshape(3, 3)

    print(initial_state)
    """
    print("Enter the choice of algorithm :")
    print("1. Uniform Cost Search") 
    print("2. A* - Manhattan distance heuristic")
    print("3. A* - Misplaced Tile heuristic")
    choice1 = int(input())
    if choice1 == 1:
        heuristic_function = 'ucs'
    elif choice1 == 2:
        heuristic_function = 'manhattan'
    else:
        heuristic_function = 'misplaced'
    initial_state = np.array([1,2,0,4,5,3,7,8,6]).reshape(3, 3)
    root = Node(state=initial_state,parent=None,action=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
    root.a_star_search(heuristic_function)


