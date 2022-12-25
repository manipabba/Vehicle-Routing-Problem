'''
This program solves the vehicle routing problem without constraints using a genetic algorithm
The alogorithm uses two-point crossover, mutation, roulette wheel with “elite list” selection

'''

from audioop import reverse
from importlib.resources import path
import haversine as hs
from haversine import Unit
import sys

from matplotlib import markers
from entity import Entity
import random
import matplotlib.pyplot as plt

# ---- Program parameters -----
V = 2 # number of vechicles for problem
GEN_COUNT = 40
POP_SIZE = 10000
ELITISM_PERC = 0.2
SELECT_PERC = 0.95
MUTATE_RATE = 0.2
cord_lst = [(38.985470, -76.946960), (38.984810, -76.952049), (38.982770, -76.947830), (38.984901, -76.945221), (38.985780, -76.946930), (38.982208, -76.946953), (38.983080, -76.944970), (38.984480, -76.941310)]
# STAMP, hornbake, business school, McKeldin, ESJ, Mowatt Lane Garage, Prince Fredrick Hall, skinner building

DIST_FUN = 1 # 1 to use coord sets, other value generate dataset
GENERATE_N = 6

# with sequencing, N+V < 10 (update to array sequencing)
# -----------------------------

# calculate fitness -> based on longest path among all vechicles
def fitness(gnome, N, V):
    global distMat
    paths = retrive_paths(gnome, N, V)  
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

# returns paths in gnome string
def retrive_paths(gnome, N, V):
    paths = []
    currPath = [1]
    pushPath = False
    for c in gnome:
        # new vechicle found
        if c > N:
            pushPath = True
        else:
            currPath.append(c)
        
        if pushPath is True:
            currPath.append(1)
            paths.append(currPath)
            currPath = [1]
            pushPath = False
        
    currPath.append(1)
    paths.append(currPath)
    return paths
 

def create_gene(N, V):
    gnome = []
    # randomly seelect 23456
    choiceLst = []
    for i in range(2, (N+V)):
        choiceLst.append(i)

    choiceLen = len(choiceLst)
    for i in range(0, choiceLen):
        c = random.choice(choiceLst)
        choiceLst.remove(c)
        gnome.append(c)
    return gnome

# mutate gnome at given rate
def mutate(gnome, mutateRate):
    gnomeLst = gnome
    for _ in range(len(gnome)):
        if random.random() < mutateRate:
            idx1 = -1
            idx2 = -1
            while idx1 != idx2:
                idx1 = random.random(0, len(gnome))
                idx2 = random.random(0, len(gnome))
            tmp = gnomeLst[idx1]
            gnomeLst[idx1] = gnomeLst[idx2]
            gnomeLst[idx2] = tmp
    return gnomeLst

# mutate entire pop at rate, update fitness score as well
def mutatePop(pop, mutateRate, N, V):
    for entity in pop:
        entity.gnome = mutate(entity.gnome, mutateRate)
        entity.fitness = fitness(entity.gnome, N, V)
    return pop

# breed 2 gnomes using 2 point crossover, also enforce unique elements across gnomes
def breed(gnome1, gnome2):
    idx1 = random.choice(range(0, len(gnome1)))
    idx2 = random.choice(range(0, len(gnome2)))

    start = min(idx1, idx2)
    end = max(idx1, idx2)

    sChild1 = []
    sChild2 = []
    
    for i in range(start, end):
        sChild1.append(gnome1[i])
    
    for c in gnome2:
        if c not in sChild1:
            sChild2.append(c)
    return sChild1 + sChild2

# select routes for next generation based on fitness score, using roulette section and elitism approach
def selectRoutes(population, elitismSize, selectedPercent):
    selected = []
    population.sort(key=lambda x: x.fitness, reverse=True)

    # add elitim routes
    for i in range(elitismSize):
        selected.append(population[i])
    population = population[elitismSize:]


    # compute total fitness
    totalFitness = 0
    for entity in population:
        totalFitness = totalFitness + entity.fitness
    
    prob_lst = []
    for entity in population:
        prob_lst.append(entity.fitness*100/totalFitness) # need to invert pobability!!!!
    
    elementsToSelect = int(len(population) * selectedPercent)
    sum = 100
    for _ in range(0, elementsToSelect):
        selectIdx = int(random.random() * sum)
        acc = 0
        for j in range(0, len(population)):
            entity = population[j]
            acc = acc + prob_lst[j]

            # if element is chosen
            if acc > selectIdx:
                selected.append(entity)
                population.remove(entity)
                sum = sum - prob_lst[j]
                break
    return selected

# mate selected population, return new population, keep elitismSize in population
def breedPopulation(matingPool, elitismSize, N, V):
    matingPool.sort(key=lambda x: x.fitness)

    res = []
    for i in range(0, elitismSize):
        res.append(matingPool[i])

    for i in range(elitismSize, len(matingPool) - 1):
        new_gnome = breed(matingPool[i].gnome, matingPool[i+1].gnome)
        child = Entity()
        child.gnome = new_gnome
        child.fitness = fitness(new_gnome, N, V)
        res.append(child)

    return res


# plots edges on graph according to gnome
# 24653 N = 5, V = 2
def plot_edges(gnome, N, V):
    global cord_lst
    color_lst = ['k', 'g', 'm', 'y', 'c']
    if V >= len(color_lst):
        print("Unable to add edges, too many vechicles")
        return

    paths = retrive_paths(gnome, N, V) # get paths in gnome

    for path in paths:
        curr_color = random.choice(color_lst)
        color_lst.remove(curr_color)
        for i in range(len(path)-1):
            cord1 = cord_lst[path[i]-1]
            cord2 = cord_lst[path[i+1]-1]
            plt.plot([cord1[0], cord2[0]], [cord1[1], cord2[1]], curr_color)

def compute_dist(cord1, cord2, dist_mode = 1):
    if dist_mode == 1:
        return hs.haversine(cord1, cord2, unit=Unit.METERS)
    else:
        return (cord1[0] - cord2[0]) ** 2 + (cord1[1] - cord2[1]) ** 2

def disp_graph(final_gnome):
    global cord_lst, N, V
    isFirst = True
    cnt = 1
    for cords in cord_lst:
        plt.plot(cords[0], cords[1], 'ro' if isFirst else 'bo', markersize = 10)
        plt.annotate(str(cnt), (cords[0], cords[1]))
        if isFirst == True:
            isFirst = False
        cnt = cnt + 1

    # add edges, show solution
    plot_edges(final_gnome, N, V)
    plt.title("Solution Genetic Algorithm")
    plt.savefig("genetic_algo.jpg")
    plt.show()

# find permutations of list. reference: https://www.geeksforgeeks.org/generate-all-the-permutation-of-a-list-in-python/
def permute(lst):
    if len(lst) == 0:
        return []
    if len(lst) == 1:
        return [lst]
    perms = []
    for i in range(len(lst)):
        tmp = lst[i]
        rem = lst[:i] + lst[i+1:]
        for ele in permute(rem):
            perms.append([tmp] + ele)
    return perms

    
# generate distance matrix
#cords = [(38.971500, -76.940308), (37.228382, -80.423416), (39.288430, -76.625980), (35.910259, -79.055473), (39.328640, -76.623039), (39.395901, -76.605461), (38.190601,-76.426300)]

# if need to generate data
if DIST_FUN != 1:
    cord_lst = []
    for _ in range(GENERATE_N):
        cord_lst.append((int(random.random() * 100), int(random.random() * 100)))

N = len(cord_lst)
distMat = [[0]*N for _ in range(N)] 
for i in range(0,N):
    for j in range(0,N):
        if i != j:
            distMat[i][j] = compute_dist(cord_lst[i], cord_lst[j], DIST_FUN)




'''
# encodinng structure
1: depot
2, 3, 4, 5: customer locations
6, 7: vechicles (however, 6 not included)

ex encoding
24653
vechicle 7: 1 -> 2 -> 4 -> 1
vechicle 6: 1 -> 5 -> 3 -> 1
'''
gen = 1
population = []
for i in range(POP_SIZE):
    entity = Entity()
    entity.gnome = create_gene(N, V)
    entity.fitness = fitness(entity.gnome, N, V)
    population.append(entity)

while gen <= GEN_COUNT:
    print("------------------------")
    print("Generation: " + str(gen))
    # sort population by fitness
    population.sort(key=lambda x: x.fitness)
    population = selectRoutes(population, int(len(population) * ELITISM_PERC), SELECT_PERC)
    print("selected: " + str(len(population)))
    population = breedPopulation(population, int(len(population) * ELITISM_PERC), N, V)
    print("bread: " + str(len(population)))
    population = mutatePop(population, MUTATE_RATE, N, V)
    print("mutate: " + str(len(population)))

    gen = gen + 1

final_gnome = min(population, key = lambda x: x.fitness).gnome
print("==========================================")
print("final gnome: ", final_gnome)
print("fitness: ", fitness(final_gnome, N, V))

disp_graph(final_gnome)
# print(fitness('64253', N, V))
# print(fitness('54263', N, V)) # looks like its an issue for long functions
# final_gnome = '54263'
# visualize solutions graph
# add verticies
print("==========================================")
print("running brute force search")
perms = permute(final_gnome)
min_sol = min(perms, key = lambda x: fitness(x, N, V))
print(min_sol)
print(fitness(min_sol, N, V))
# run all permutations of gnome










