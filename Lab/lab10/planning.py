import asyncio
from lab10.grid import *
from lab10.visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

'''
Setting some general global values
'''
init_x = 0
init_y = 0
grid_scale = 25

'''
Basic rounding function which rounds the x and y coordinates so that we can easily plot them in our grid
'''
def rounding(start):
    return (int(round(start[0])), int(round(start[1])))


'''
This function takes cozmo's real pose 
and converts it to a coordinate in the grid based on the scale of realworld vs grid
'''
def convertToGridCoord(pose: cozmo.util.Pose):
    pos = pose.position
    x = ((pos.x - init_x) / grid_scale) + 3
    y = ((pos.y - init_y) / grid_scale) + 2
    return (x, y)

'''
This function takes in two coordinates and return the euclidean distance on the grid
'''
def grid_dist(a, b):
    ax, ay = a
    bx, by = b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

'''
This is the heuristic function leveraged by the astar algorithm
The heuristic is basic euclidean distance between current node and the goal
'''
def heuristic(current, goal):
    return grid_dist(current, goal)

'''
Astar function that takes grid and heuristic and sets the optimal path and visited nodes on the grid
'''
def astar(grid: CozGrid, heuristic):
    start = grid.getStart()
    goal = grid.getGoals()[0]  
    path, visited = as_support(heuristic, start, goal, grid)
    grid.setPath(path)
    for v in visited:
        grid.addVisited(v)

'''
This is a supporting function to the astar function above 
This function tryly implements astar by taking in the heuristic, start, goal and the grid
using priority queue to track nodes and returns path and visited nodes at the end
'''
def as_support(heuristic, start, goal, grid:CozGrid):
    start = (int(round(start[0])), int(round(start[1])))
    goal = (int(round(goal[0])), int(round(goal[1])))
    visited = [start]
    costs = {}
    costs[start] = 0
    front = PriorityQueue()
    front.put((0 + heuristic(start, goal), 0, (start, [start])))
    while not front.empty():
        node = front.get()
        cost, counter, data = node
        cell, path = data

        visited.append(cell)
        if grid_dist(cell, goal) < 1:
            return path, visited
        else:
            for neighborWithCost in grid.getNeighbors(cell):
                neighbor, cost = neighborWithCost
                if neighbor in grid._obstacles:
                    print(neighbor)
                newCost = costs[cell] + cost
                if neighbor not in costs or newCost < costs[neighbor]:
                    costs[neighbor] = newCost
                    priority = newCost + heuristic(neighbor, goal)
                    newpath = path[:]
                    newpath.append(neighbor)
                    front.put((priority, counter + 1, (neighbor, newpath)))
    return [start], visited

'''
Basic initialization for cozmo at the start of traversal
'''
def init(robot: cozmo.robot.Robot):
    global init_x, init_y
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    init_x = robot.pose.position.x
    init_y = robot.pose.position.y

'''
Straight line drive to a particluar pose function
'''
def drive_to(robot: cozmo.robot.Robot, pos):
    nx, ny = pos
    rx, ry = convertToGridCoord(robot.pose)
    dx = nx - rx
    dy = ny - ry
    rz = robot.pose.rotation.angle_z
    rotz = math.degrees(math.atan2(dy + .6, dx + .6))
    robot.turn_in_place(degrees(rotz) - rz).wait_for_completed()
    rx, ry = convertToGridCoord(robot.pose)
    dx = nx - rx
    dy = ny - ry
    rotd = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    robot.drive_straight(distance_mm(rotd * 25), speed_mmps(50)).wait_for_completed()


'''
Function to add general obstacles to the grid
'''
def obstacle_to_grid(grid: CozGrid, obstacle):
    print("Obstacle cuble found : Added to grid")
    ox, oy = rounding(obstacle)
    for dx in [ -3, -2, -1, 0, 1, 2, 3 ]:
        for dy in [-3, -2, -1, 0, 1, 2, 3]:
            grid.addObstacle((ox + dx, oy + dy))
            
'''
Function to add the goal cube to the grid
'''
def goal_to_grid(grid: CozGrid, obstacle, cube):
    print("Cube 1 found : Goal added to grid")
    ox, oy = rounding(obstacle)
    oz = 180 + cube.pose.rotation.angle_z.degrees
    for dx in [-4, -3, -2, -1, 0, 1, 2, 3, 4]:
        for dy in [-4, -3, -2, -1, 0, 1, 2, 3, 4]:
            grid.addObstacle((ox + dx, oy + dy))
    dx = 5 * math.cos(math.radians(oz))
    dy = 5 * math.sin(math.radians(oz))
    g = rounding((ox + dx, oy + dy))
    grid.addGoal(g)
    return g

'''
Stay on lookout for cubes and add them to map when found.Return true if any cubes were added
'''
def find_and_update_cubes(robot: cozmo.robot.Robot, seen_cubes: dict, grid: CozGrid, oldGoalPosition):
    goal_pos_0 = oldGoalPosition
    try:
        cubes = list(robot.world.visible_objects)
    except asyncio.TimeoutError:
        print("TimeoutError")
        return False, seen_cubes
    else:
        changed = False
        for cube in cubes:
            is_cube_1 = cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
            is_cube_2 = cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube2Id].object_id
            is_cube_3 = cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube3Id].object_id
            if 1 not in seen_cubes and is_cube_1:
                print("Cube 1 located: " + str(convertToGridCoord(cube.pose)))
                seen_cubes[1] = cube
                changed = True
                goal_pos_0 = goal_to_grid(grid, convertToGridCoord(cube.pose), seen_cubes[1])
            elif 2 not in seen_cubes and is_cube_2:
                print("Cube 2 located: " + str(convertToGridCoord(cube.pose)))
                seen_cubes[2] = cube
                changed = True
                obstacle_to_grid(grid, convertToGridCoord(cube.pose))
            elif 3 not in seen_cubes and is_cube_3:
                print("Cube 3 located:" + str(convertToGridCoord(cube.pose)))
                seen_cubes[3] = cube
                changed = True
                obstacle_to_grid(grid, convertToGridCoord(cube.pose))
        return changed, seen_cubes, goal_pos_0



def cozmoBehavior(robot: cozmo.robot.Robot):

    global grid, stopevent
    state = "stopped"
    init(robot)
    cubes = {}
    goalPosition = None
    path = []
    path_pos = -1
    grid.setStart((3, 2))
    '''
    Adding a small boundary to the grid as obstacle so that robot doesnt cross that 
    '''
    for i in range(26):
        grid.addObstacle((i, 0))
        grid.addObstacle((i, 17))

    for i in range(18):
        grid.addObstacle((0, i))
        grid.addObstacle((25, i))

    '''
    while stop event is not set, keep looking for cubes and update the map
    if cube 1 is not seen, just go to the center or if it is seen, plot path to cube 1
    if cube is in the center from previous step, search for cubes by roataing by 30 degress each time
    once cozmo has reached its desired cube position, rotate to face the right face of the cube and say "Path traversed"
    
    '''
    while not stopevent.is_set():
        cubes_changed, cubes, goalPosition = find_and_update_cubes(robot, cubes, grid, goalPosition)
        if cubes_changed:
            state = "stopped"            

        grid.setStart(rounding(convertToGridCoord(robot.pose)))
        if state != "completed":
            print("Current state:", state)

        if state == "stopped":
            if 1 not in cubes:
                path, visited = as_support(heuristic, grid.getStart(), (13, 9), grid)
                grid.clearVisited()
                for v in visited:
                    grid.addVisited(v)
                path_pos = 0
                grid.clearGoals()
                grid.addGoal((13, 9))
                grid.setPath(path)                
                state = "drive_to_middle"
            else:
                dest = goalPosition
                path, visited = as_support(heuristic, grid.getStart(), dest, grid)
                grid.clearVisited()
                for v in visited:
                    grid.addVisited(v)
                path_pos = 0
                grid.clearGoals()
                grid.addGoal(dest)
                grid.setPath(path)
                print("Path to goal cube found")
                state = "drive"

        elif state == "drive_to_middle":
            if 1 not in cubes:
                if grid_dist(convertToGridCoord(robot.pose), (13, 9)) < 1.41:
                    state = "search"
                elif path_pos == len(path):
                    print("Out of path options")
                    state = "stopped"
                else:
                    print("Going to center")
                    drive_to(robot, path[path_pos])
                    path_pos += 1
            else:
                state = "stopped"

        elif state == "search":
            if 1 not in cubes:
                robot.turn_in_place(degrees(30)).wait_for_completed()
            else:
                state = "drive"

        elif state == "drive":           
            if 1 not in cubes:
                state = "stopped"
            else:
                if grid_dist(convertToGridCoord(robot.pose), goalPosition) < 1:
                    state = "rotate"
                elif path_pos == len(path):
                    print("Out of path options")
                    state = "stopped"
                else:
                    print("drive driving from " + str(convertToGridCoord(robot.pose)) + " to " + str(path[path_pos]))
                    drive_to(robot, path[path_pos])
                    path_pos += 1

        elif state == "rotate":
            cozmoZ = robot.pose.rotation.angle_z.degrees
            cubeZ = cubes[1].pose.rotation.angle_z.degrees + 180
            angleZ = (cubeZ + 180)%360 - cozmoZ
            print("Cozmo's position is: ", robot.pose.rotation.angle_z.degrees)
            print("Cube's position is: ", cubes[1].pose.rotation.angle_z.degrees + 180)            
            robot.turn_in_place(degrees(angleZ)).wait_for_completed()
            robot.say_text("Path traversed")
            state = "completed"

######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()
