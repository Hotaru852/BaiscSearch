import math
import sys

d_row = [-1, 0, 1, 0]
d_col = [0, 1, 0, -1]


# Characteristics of a state
class Node(object):
    velocity = int
    cost = float
    x = int
    y = int
    actions = str
    direction = int
    steps = int
    fuel_costs = float

    def __init__(self, velocity, costs, direction, x, y, actions, steps, fuel_costs):
        self.velocity = velocity
        self.cost = costs
        self.direction = direction
        self.x = x
        self.y = y
        self.actions = actions
        self.steps = steps
        self.fuel_costs = fuel_costs


def line2ints(line):
    return [int(m) for m in line.split(" ")]


# Heuristic function: Estimated distance to goal (Manhattan Distance)
def manhattan(a, b, c, d):
    return abs(a - c) + abs(b - d)


# Check for collisions (can be improved)
def check(x, y, direction, velocity):
    if N > x >= 0 and N > y >= 0:
        if direction % 2 == 0:
            for wall in h_walls:
                prev_y = y - d_row[direction] * velocity
                if min(prev_y, y) <= wall[0] <= max(prev_y, y) and wall[1] <= x <= wall[2]:
                    return False
        else:
            for wall in v_walls:
                prev_x = x - d_col[direction] * velocity
                if min(prev_x, x) <= wall[0] <= max(prev_x, x) and wall[1] <= y <= wall[2]:
                    return False
        return True
    return False


# Generate cost for each Algorithm (UCS is total fuel costs, Greedy is estimated distance to goal, A* is total fuel costs plus estimated distance to goal)
def cost(current_cost, current_x, current_y, current_velocity):
    # Uniform Cost Search
    if method == 1:
        return current_cost + fuel_cost + math.sqrt(current_velocity)
    # Greedy Best-first Search
    elif method == 2:
        return manhattan(current_x, current_y, xg, yg)
    # A* Search
    else:
        return current_cost + fuel_cost + math.sqrt(current_velocity) + manhattan(current_x, current_y, xg, yg)


# Generalized Search Algorithm
def Search():
    # Here we use a queue to store all the possible states
    queue = [Node(0, 0, 0, xs, ys, "", 0, 0)]
    visited = {}
    goals = []
    while len(queue) > 0:
        # Get the state with the lowest cost
        queue = sorted(queue, key=lambda x: x.cost)
        current = queue[0]
        # If a state is already visited then delete it from the queue and get to the next state
        if visited.get((current.velocity, current.direction, current.x, current.y), False) is True:
            del queue[0]
            continue
        # If a state is not visited yet then mark it as visited
        visited[(current.velocity, current.direction, current.x, current.y)] = True
        del queue[0]
        # If velocity is greater than 0 there will be 3 possible states: speed up (only if velocity is not max), slow down and no action
        if current.velocity > 0:
            if current.velocity < V_max:
                queue.append(Node(current.velocity + 1, cost(current.cost, current.x, current.y, current.velocity), current.direction, current.x, current.y, current.actions + "+", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
            check_x = current.x + d_col[current.direction] * current.velocity
            check_y = current.y + d_row[current.direction] * current.velocity
            if check(check_x, check_y, current.direction, current.velocity):
                # If current location is the goal then add its path to the list of possible paths
                if check_x == xg and check_y == yg:
                    goals.append(Node(current.velocity, cost(current.cost, check_x, check_y, current.velocity), current.direction, check_x, check_y, current.actions + "O", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
                queue.append(Node(current.velocity, cost(current.cost, check_x, check_y, current.velocity), current.direction, check_x, check_y, current.actions + "O", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
            queue.append(Node(current.velocity - 1, cost(current.cost, current.x, current.y, current.velocity), current.direction, current.x, current.y, current.actions + "-", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
        # If velocity is 0 there will be 3 possible states: turn left, turn right and speed up
        else:
            queue.append(Node(current.velocity, cost(current.cost, current.x, current.y, current.velocity), (current.direction + 1) % 4, current.x, current.y, current.actions + "R", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
            queue.append(Node(current.velocity, cost(current.cost, current.x, current.y, current.velocity), (current.direction - 1) % 4, current.x, current.y, current.actions + "L", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
            queue.append(Node(current.velocity + 1, cost(current.cost, current.x, current.y, current.velocity), current.direction, current.x, current.y, current.actions + "+", current.steps + 1, current.fuel_costs + fuel_cost + math.sqrt(current.velocity)))
    # Return a list of paths that can lead to the goal
    return goals


# Get command line arguments
n = len(sys.argv)
input_file = sys.argv[1]
output_file = sys.argv[2]
method = int(sys.argv[3])

# Read input file
ifile = open(input_file, "r")
lines = [line for line in ifile.readlines()]
N, n_wall, V_max, fuel_cost = line2ints(lines[0])
xs, ys, xg, yg = line2ints(lines[1])
h_walls = []
v_walls = []
for i in range(n_wall):
    x1, y1, x2, y2 = line2ints(lines[2+i])
    if x1 == x2:
        v_walls.append((x1, min(y1, y2), max(y1, y2)))
    else:
        h_walls.append((y1, min(x1, x2), max(x1, x2)))
ifile.close()

# Main
answers = Search()
# Get the path with the lowest cost
answers.sort(key=lambda x: x.cost)
if len(answers) == 0:
    print("-1-1")
else:
    # write to output file
    ofile = open(output_file, "w")
    ofile.write(str(answers[0].steps))
    ofile.write(" ")
    ofile.write(f"{answers[0].fuel_costs}\n")
    ofile.write(answers[0].actions)
    ofile.close()

# Expected runtime
# For each point in the maze there is four possible directions to follow: N, S, E, W
# Multiply that by the number of possible velocities (Vmax) we have the total number of possible states
# Number of possible states: N * N * 4 * Vmax
# Here we are using a queue to store all the possible states
# The branching factor here is 3 which means at each step we can add a maximum of 3 states into the queue
# Setting and getting an element with the built-in python dict all take O(n) in worst case
# Adding and deleting a state all take O(1) so the time complexity of these operations is not important
# The sorting algorithm used to sort the queue here is Timsort which has the worst case performance of O(nlogn) where n is the number of states
# So the time complexity of this algorithm will be O(n^2logn) where n is the number of states (horrible runtime)
# I think I can improve the runtime if I check a state before putting it into the queue instead of checking after I have put it into the queue