import csv
from csv import reader
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

encStates = []
clusterStates = []
obstacles = [ [1,1,0.6], [0,2,0.2], [2,-0.5,0.3], [-1, -6, 0.5] ] # drone canNOT go through these, list of [x, y, radius]
waypoints = [[0, 0, 0.2], [-0.8, 0, 0.2], [-0.8, -0.6, 0.2], [-0.7, -0.7, 0.2], [0, -0.2, 0.2], [0, 0, 0.2]]
 # drone MUST go through these, list of [x,y, radius]
bounds = [-2.5, 2.5, -2.5, 2.5] #  b[0] < x < b[1], b[2] < y < b[3] else crash

 
def check_collision(path):
    global obstacles
    for point in path:
        px, py, pz, kb = point
        for obs in obstacles:
            ox, oy, orad = obs
            if ( abs(px - ox) < orad and abs(py - oy) < orad ):
                return True
        if not (bounds[0] < px < bounds[1] and bounds[2] < py < bounds[3]): 
            return True
    return False 

def check_waypoints(path, partial=False):
    global waypoints
    points_passed = 0
    for wp in waypoints:
        wx, wy, wrad = wp
        passed_waypoint = False
        for point in path:
            px, py, pz, kb = point
            if  ( abs(px - wx) < wrad and abs(py - wy) < wrad ):
                passed_waypoint = True
        points_passed += 1.0*(passed_waypoint)
        if not passed_waypoint and not partial:
            return False
    if not partial:
        return True 
    return 1.0*points_passed

def dist(s1, s2):
    return ((s1[0] - s2[0])**2 + (s1[1]-s2[1])**2 + (s1[2] - s2[2])**2)**(0.5)

def eve_split_trajs(num_paths):
    #get the num_paths possible trajectories, hopefully
    global encStates, clusterStates
    paths = [ ] #num_paths dimensional
    for i in range(len(encStates)):
        nextPoint = encStates[i]
        shortestDist = 100
        bestPath = None
        for j in range(len(paths)):
            path = paths[j]
            shortestPathDist = 100
            if (len(path) > 0):
                point = path[-1]
                d = dist(point,nextPoint)
                if (d < shortestPathDist):
                    shortestPathDist = d
            if shortestPathDist < shortestDist:
                shortestDist = shortestPathDist
                bestPath = j
        if (bestPath is None or shortestDist > 1) and len(paths) < num_paths:
            paths.append( [ nextPoint ] )
        else:
            paths[bestPath].append(nextPoint)
    return paths

def eve_possibles(paths):
    possible_paths = []
    for path in paths:
        if check_waypoints(path) and not check_collision(path):
            possible_paths.append(path)
    return possible_paths

def eve_probables(paths):
    weights = []
    S = 0
    for path in paths:
        if check_collision(path):
            weights.append(0)
        else:
            prob = check_waypoints(path,partial=True)
            weights.append(prob )
            S += prob
    norm_weights = [(w+.0000001)/(S+0000001) for w in weights]
    print("weights then norm weights:" + str(weights))
    return norm_weights


def eve_best_guess(possibles, weights=None):
    if weights is None:
        weights = [1.0/len(possibles) for p in possibles] #change if probs known, weight for specific path
    guess_path = [ ]
    if len(possibles) < 1:
        return None
    largest_len = np.max( [len(p) for p in possibles] )
    for j in range(largest_len):
        estimate_point = [0,0,0]
        N = 0
        for i in range(len(possibles)):
            if j < len(possibles[i]):
                N = N + weights[i]
                estimate_point = [estimate_point[ii] + weights[i]*possibles[i][j][ii] for ii in range(3)]
        estimate_point = [estimate_point[ii]/(N+0.01) for ii in range(3)]
        guess_path.append(estimate_point)
    print("len guess_path:{}".format(len(guess_path)) )
    return guess_path

def check_bit(path):
    for point in path:
        if point[3] == 1:
            print("ERROR! got wrong traj")
            return
    print("got right traj")

with open("data.csv", 'rb') as csvfile:
    f = csv.reader(csvfile, delimiter=',')
    for row in f:
        if len(row) == 4:
            frow = []
            for i in range(len(row)):
                num = row[i]
                frow.append(float(num))
                if i == 2:
                    clusterStates.append(frow)
            encStates.append(frow)
            


paths = eve_split_trajs(10)
for p in paths: 
    xs = [point[0] for point in p]
    ys = [point[1] for point in p]
    color = np.random.rand(3,)
    plt.plot(xs, ys, c=color)
    plt.scatter(xs, ys, c=color)

possible_paths = eve_possibles(paths)
if len(possible_paths) == 1:
    check_bit(possible_paths[0])
best_path = eve_best_guess(possible_paths)
print("Length of paths:{}, length of possible_paths:{}".format(len(paths), len(possible_paths)) )
if best_path is not None:
    xs = [point[0] for point in best_path]
    ys = [point[1] for point in best_path]
    plt.plot(xs, ys, marker='^', linewidth=2.0, c="blue" ) 
    plt.scatter(xs[0], ys[0], marker='^',s=100 ) 

"""
probs = eve_probables(paths)
print("Probabilities:" + str(probs))
prob_path = eve_best_guess(paths, probs)
if prob_path is not None:
    xs = [point[0] for point in prob_path]
    ys = [point[1] for point in prob_path]
    plt.plot(xs, ys, marker='^', linewidth=2.0, c="yellow" ) 
    plt.scatter(xs[0], ys[0], marker='^',s=100 ) 
"""

fig = plt.gcf()
ax = plt.gca()
# add in obstacles, waypoints
for wp in waypoints:
    wx, wy, wrad = wp
    wpcircle = plt.Circle( (wx, wy), wrad, color='green', fill=False )
    ax.add_artist(wpcircle)

for obs in obstacles:
    ox, oy, orad = obs
    obscircle = plt.Circle( (ox, oy), orad, color='red', fill=False )
    ax.add_artist(obscircle)

xy = [bounds[0], bounds[2] ]
width = bounds[1]-bounds[0]
height = bounds[3]-bounds[2]
rect = plt.Rectangle(xy, width, height, fill=False, color='black')
ax.add_artist(rect)
ax.set_xlim(left=-3,right=3)
ax.set_ylim(bottom=-3,top=3)

plt.show()

