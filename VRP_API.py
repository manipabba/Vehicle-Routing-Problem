from haversine import Unit
import haversine as hs
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt
import random


def compute_dist(cord1, cord2, dist_mode = 1):
    if dist_mode == 1:
        return hs.haversine(cord1, cord2, unit=Unit.METERS)
    else:
        return (cord1[0] - cord2[0]) ** 2 + (cord1[1] - cord2[1]) ** 2

def fetch_distance(from_idx, to_idx):
    global distMat
    from_node = manager.IndexToNode(from_idx)
    to_node = manager.IndexToNode(to_idx)
    return distMat[from_node][to_node]

def disp_solution(data, manager, routing, solution):
    global N, V
    paths = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        path = []
        while not routing.IsEnd(index):
            path.append(manager.IndexToNode(index) + 1)
            index = solution.Value(routing.NextVar(index))
        path.append(manager.IndexToNode(index) + 1)
        paths.append(path)
    print(paths)
    print("fitness: ", fitness(paths, N, V))
    disp_graph(paths, N, V)

def disp_graph(paths, N, V):
    global cord_lst
    isFirst = True
    cnt = 1
    for cords in cord_lst:
        plt.plot(cords[0], cords[1], 'ro' if isFirst else 'bo', markersize = 10)
        plt.annotate(str(cnt), (cords[0], cords[1]))
        if isFirst == True:
            isFirst = False
        cnt = cnt + 1

    # add edges, show solution
    plot_edges(paths, N, V)
    plt.title("Solution Google OR Tools")
    plt.savefig('google_ortools.jpg')
    plt.show()
    

def plot_edges(paths, N, V):
    global cord_lst
    color_lst = ['k', 'g', 'm', 'y', 'c']
    if V >= len(color_lst):
        print("Unable to add edges, too many vechicles")
        return

    for path in paths:
        curr_color = random.choice(color_lst)
        color_lst.remove(curr_color)
        for i in range(len(path)-1):
            cord1 = cord_lst[path[i]-1]
            cord2 = cord_lst[path[i+1]-1]
            plt.plot([cord1[0], cord2[0]], [cord1[1], cord2[1]], curr_color)

def fitness(paths, N, V):
    global distMat
    pathLens = []
    # compute path lengths
    for path in paths:
        dist = 0
        for i in range(0,len(path) - 1):
            startIdx = path[i] - 1
            endIdx = path[i+1] - 1
            dist = dist + distMat[startIdx][endIdx]
        pathLens.append(dist)
    return max(pathLens)


        

# ---- Program parameters -----
V = 2 # number of vechicles for problem
cord_lst = [(38.985470, -76.946960), (38.984810, -76.952049), (38.982770, -76.947830), (38.984901, -76.945221), (38.985780, -76.946930), (38.982208, -76.946953), (38.983080, -76.944970), (38.984480, -76.941310)]
# STAMP, hornbake, business school, McKeldin, ESJ, Mowatt Lane Garage, Prince Fredrick Hall, skinner building

# -----------------------------
N = len(cord_lst)
distMat = [[0]*N for _ in range(N)] 
for i in range(0,N):
    for j in range(0,N):
        if i != j:
            distMat[i][j] = compute_dist(cord_lst[i], cord_lst[j], 1)

# set up data dict
data = {}
data['distance_matrix'] = distMat
data['num_vehicles'] = V
data['depot'] = 0 # assuming starting is depot


# Create the routing index manager.
manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
routing = pywrapcp.RoutingModel(manager)

transit_callback = routing.RegisterTransitCallback(fetch_distance)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback)

# Add Distance constraint.
dimension_name = 'Distance'
routing.AddDimension(
    transit_callback,
    0,  # no slack
    3000,  # vehicle maximum travel distance
    True,  # start cumul to zero
    dimension_name)
distance_dimension = routing.GetDimensionOrDie(dimension_name)
distance_dimension.SetGlobalSpanCostCoefficient(100)

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

solution = routing.SolveWithParameters(search_parameters)

if solution:
    disp_solution(data, manager, routing, solution)
else:
    print('No solution found')


