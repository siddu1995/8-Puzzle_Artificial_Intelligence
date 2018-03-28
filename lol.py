import numpy as np
import time
import sys
#declaring the goal_state
goal_state = np.array([1, 2, 3, 4, 5, 6, 7, 8, 0]).reshape(3, 3)

class Node(): #Node class
    def __init__(self,state,parent,action,depth,step_cost,path_cost,heuristic_cost):
        self.state = state # state of the node
        self.parent = parent  # parent node
        self.action = action  # action(down,right,up,left)
        self.depth = depth  # depth of the node in the tree
        self.step_cost = step_cost  # g(n), cost to get to a node
        self.path_cost = path_cost  # accumulated g(n), the cost to reach the current node from initial node
        self.heuristic_cost = heuristic_cost  # h(n), estimated distance to the goal

        # children node
        self.move_up = None
        self.move_left = None
        self.move_down = None
        self.move_right = None

    def print_path(self):
        # stacks to place the trace from initial state to goal state (LIFO)
        state_trace = [self.state]
        action_trace = [self.action]
        depth_trace = [self.depth]
        step_cost_trace = [self.step_cost]
        path_cost_trace = [self.path_cost]
        heuristic_cost_trace = [self.heuristic_cost]

        # add node information as tracing back to initial node
        while self.parent:
            self = self.parent
            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            step_cost_trace.append(self.step_cost)
            path_cost_trace.append(self.path_cost)
            heuristic_cost_trace.append(self.heuristic_cost)

        # printing the path by printing the trace
        step_counter = 0
        while state_trace:
            print('step', step_counter)
            print(state_trace.pop())
            print('action=', action_trace.pop(),
                  ', depth=', str(depth_trace.pop()),
                  ', step cost=', str(step_cost_trace.pop()),
                  ', total_cost=', str(path_cost_trace.pop() + heuristic_cost_trace.pop()))
            step_counter += 1
    # checking if the value can be moved into empty tile
    # to check if moving down is valid
    def try_move_down(self):
        # to get the index of the empty tile
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[0] == 0: #moving down is not possible when empty tile is in first row
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

    # to check if moving right is valid
    def try_move_right(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[1] == 0: #moving right is not possible when empty tile is in first column 
            return 0,0
        else:
            left_value = self.state[zero_index[0], zero_index[1] - 1]  # value of the left tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = left_value
            new_state[zero_index[0], zero_index[1] - 1] = 0
            return new_state, 1

    # to check if moving up is valid
    def try_move_up(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[0] == 2: #moving up is not possible when empty tile is in last row
            return 0,0
        else:
            lower_value = self.state[zero_index[0] + 1, zero_index[1]]  # value of the lower tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = lower_value
            new_state[zero_index[0] + 1, zero_index[1]] = 0
            return new_state, 1

    # to check if moving left is valid
    def try_move_left(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[1] == 2: #moving left is not possible when empty tile is in last column
            return 0,0
        else:
            right_value = self.state[zero_index[0], zero_index[1] + 1]  # value of the right tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = right_value
            new_state[zero_index[0], zero_index[1] + 1] = 0
            return new_state, 1

    #return heuristic cost h(n): number of misplaced tiles
    def h_misplaced_cost(self, new_state):
        cost = np.sum(new_state != goal_state) - 1  # minus 1 to exclude the empty tile
        if cost > 0:
            return cost
        else:
            return 0  # when all tiles matches

    # return heuristic cost h(n): sum of Manhattan distance to reach the goal state
    def h_manhattan_cost(self, new_state):
        current = new_state
        # indexes of the value in the goal_state
        goal_position = {   1: (0, 0),
                            2: (0, 1),
                            3: (0, 2),
                            4: (1, 0),
                            5: (1, 1),
                            6: (1, 2),
                            7: (2, 0),
                            8: (2, 1),
                            0: (2, 2)}
        msum = 0
        for i in range(3):
            for j in range(3):
                if current[i, j] != 0:
                    msum += sum(abs(a - b) for a, b in zip((i, j), goal_position[current[i, j]]))
        return msum

    # A* search based on heuristic_function
    def a_star_search(self, heuristic_function):
        start = time.time() #to calculate the time
        queue = [(self,0)] #queue to store (unvisited nodes,total_cost)
        queue_expanded = 0 # measuring time complexity, no.of nodes expanded
        queue_max_length = 1 # measuring space complexity, max no.of queue at its max
        cost_queue = [(0,0)] # queue of node cost (depth,total_cost)
        visited = set([])  #stores all the visited nodes, to ignore repeated states

        while queue:
            queue = sorted(queue, key= lambda x: x[1]) #sort based on total_cost
            cost_queue = sorted(cost_queue , key= lambda x: x[1]) #sort based on total_cost
            current_node = queue.pop(0)[0] #getting the first node from the queue
            current_cost = cost_queue.pop(0)[0] #getting the first node's depth from the cost_queue
            if len(queue) > queue_max_length: #updating queue max length
                queue_max_length = len(queue)
            queue_expanded += 1 #updating for getting no.of nodes expanded
            visited.add(tuple(current_node.state.reshape(1, 9)[0])) #adding the visited nodes
            if np.array_equal(current_node.state, goal_state): #to check if goal_state is reached
                current_node.print_path() #to print the sequence of nodes from root to goal_state
                print('Time performance:', str(queue_expanded), 'nodes expanded.')
                print('Space performance:', str(queue_max_length), 'nodes in the queue at its max.')
                print('Time spent: %0.2fs' % (time.time() - start))
                return True
            else:
                #TRY_MOVE_DOWN
                new_state, up_value = current_node.try_move_down() #to get new child node
                if up_value: #to check if moving down is possible
                    if tuple(new_state.reshape(1, 9)[0]) not in visited: #if not in visited, else it is repeated
                        path_cost = current_cost + up_value #cost taken to reach this child node
                        depth = current_cost+1 #increment the depth
                        if heuristic_function is 'misplaced': #to get misplaced heuristic cost
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan': #to get manhattan heuristic cost
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0 #h(n)=0 for uniform cost search
                        total_cost = path_cost + h_cost # total_cost f(n) = g(n)+h(n)
                        current_node.move_down = Node(state=new_state,
                                                      parent=current_node,
                                                      action='down',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost) #operation
                        queue.append((current_node.move_down,total_cost)) #appending the (new node,total_cost) to queue
                        cost_queue.append((depth, total_cost)) #appending (depth, total_cost)

                # TRY_MOVE_RIGHT
                new_state, up_value = current_node.try_move_right()  # to get new child node
                if up_value:  # to check if moving right is possible
                    if tuple(new_state.reshape(1, 9)[0]) not in visited:  # if not in visited, else it is repeated
                        path_cost = current_cost + up_value  # cost taken to reach this child node
                        depth = current_cost + 1  # increment the depth
                        if heuristic_function is 'misplaced':  # to get misplaced heuristic cost
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':  # to get manhattan heuristic cost
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0  # h(n)=0 for uniform cost search
                        total_cost = path_cost + h_cost  # total_cost f(n) = g(n)+h(n)
                        current_node.move_right = Node(state=new_state,
                                                      parent=current_node,
                                                      action='right',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)  # operation
                        queue.append((current_node.move_right,total_cost))  # appending the (new node,total_cost) to queue
                        cost_queue.append((depth, total_cost))  # appending (depth, total_cost)

                # TRY_MOVE_UP
                new_state, up_value = current_node.try_move_up()  # to get new child node
                if up_value:  # to check if moving down is possible
                    if tuple(new_state.reshape(1, 9)[
                                 0]) not in visited:  # if not in visited, else it is repeated
                        path_cost = current_cost + up_value  # cost taken to reach this child node
                        depth = current_cost + 1  # increment the depth
                        if heuristic_function is 'misplaced':  # to get misplaced heuristic cost
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':  # to get manhattan heuristic cost
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0  # h(n)=0 for uniform cost search
                        total_cost = path_cost + h_cost  # total_cost f(n) = g(n)+h(n)
                        current_node.move_up = Node(state=new_state,
                                                      parent=current_node,
                                                      action='up',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)  # operation
                        queue.append((current_node.move_up,
                                      total_cost))  # appending the (new node,total_cost) to queue
                        cost_queue.append((depth, total_cost))  # appending (depth, total_cost)

                # TRY_MOVE_LEFT
                new_state, up_value = current_node.try_move_left()  # to get new child node
                if up_value:  # to check if moving down is possible
                    if tuple(new_state.reshape(1, 9)[
                                 0]) not in visited:  # if not in visited, else it is repeated
                        path_cost = current_cost + up_value  # cost taken to reach this child node
                        depth = current_cost + 1  # increment the depth
                        if heuristic_function is 'misplaced':  # to get misplaced heuristic cost
                            h_cost = self.h_misplaced_cost(new_state)
                        elif heuristic_function == 'manhattan':  # to get manhattan heuristic cost
                            h_cost = self.h_manhattan_cost(new_state)
                        else:
                            h_cost = 0  # h(n)=0 for uniform cost search
                        total_cost = path_cost + h_cost  # total_cost f(n) = g(n)+h(n)
                        current_node.move_left = Node(state=new_state,
                                                      parent=current_node,
                                                      action='left',
                                                      depth=depth,
                                                      step_cost=up_value,
                                                      path_cost=path_cost,
                                                      heuristic_cost=h_cost)  # operation
                        queue.append((current_node.move_left,
                                      total_cost))  # appending the (new node,total_cost) to queue
                        cost_queue.append((depth, total_cost))  # appending (depth, total_cost)

def ifsolvable(array): #to check if an custom entered instance of 8 puzzle is solvable?
    inversions = 0
    for i in range(len(array)):
        for j in range(i+1,len(array)):
            if array[j] > array[i]:
                inversions+=1
    numbers = [1,2,3,4,5,6,7,8,0]
    for i in range(len(array)):
        if i in numbers:
            numbers.pop(i)
        else:
            return 2
    if inversions%2 == 1:
        return 0
    else:
        return 1

if __name__ == "__main__":
    sys.stdout.flush()
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
        ch = ifsolvable(user_puzzle)
        if ch ==2:
            print("Sorry! Please enter numbers from 1 to 8 and 0 to represent blank.")
            print("And please dont repeat any of the numbers")
            exit(0)
        elif not ch:
            print("Sorry! The given puzzle is not solvable!")
            exit(0)
        initial_state = np.array(user_puzzle).reshape(3,3)
    else:
        print("Choose among 1. easy, 2. medium, 3. hard, 4. Very hard")
        choice2 = int(input())
        if choice2 == 1:
            initial_state = np.array([1,2,0,4,5,3,7,8,6]).reshape(3, 3)
        elif choice2 == 2:
            initial_state = np.array([1,8,2,0,4,3,7,6,5]).reshape(3, 3)
        elif choice2 == 3:
            initial_state = np.array([1,3,8,7,0,4,5,2,6]).reshape(3, 3)
        else:
            initial_state = np.array([8,7,1,6,0,2,5,4,3]).reshape(3, 3)

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
    root = Node(state=initial_state,parent=None,action=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
    root.a_star_search(heuristic_function)