# -*- coding: utf-8 -*-
"""
Created on Mon Oct  5 14:54:09 2020

@author: Amlan Ghosh
"""
from __future__ import print_function

import os
import math
import random as rand
import numpy as np
from collections import namedtuple
import itertools as itr
import time

#start = time.time()

#from __future__ import print_function
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


Point = namedtuple("Point", ['x', 'y'])

'''Distance between two points'''
def length(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# def obj(solution, points, nodeCount):
#     ob = length(points[solution[-1]], points[solution[0]]) #Distance between end point and first point
#     for index in range(0, nodeCount-1):
#         ob += length(points[solution[index]], points[solution[index+1]]) #summation of the distances between points
#     return ob

# input_data = open('data/tsp_51_1', 'r').read()

# lines = input_data.split('\n')

# nodeCount = int(lines[0])

# points = []
# for i in range(1, nodeCount+1):
#     line = lines[i]
#     parts = line.split()
#     points.append((float(parts[0]), float(parts[1])))

# def create_data_model():
#     data = {}
#     data['locations'] = points
#     data['num_vehicles'] = 1
#     data['depot'] = 0
#     return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = round(length(from_node, to_node),2)*100
    return distances

def print_solution(manager, routing, solution, distanceCallback):
    """Prints solution on console."""
    print('Objective: {}'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    totalTravelTime = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        totalTravelTime += distanceCallback(previous_index,index)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    print(totalTravelTime)
    plan_output += 'Objective: {}m\n'.format(route_distance)

def get_routes(solution, routing, manager):
  """Get vehicle routes from a solution and store them in an array."""
  # Get vehicle routes and store them in a two dimensional array whose
  # i,j entry is the jth location visited by vehicle i along its route.
  for route_nbr in range(routing.vehicles()):
    index = routing.Start(route_nbr)
    route = [manager.IndexToNode(index)]
    while not routing.IsEnd(index):
      index = solution.Value(routing.NextVar(index))
      route.append(manager.IndexToNode(index))
  return route

# def main():
#     """Entry point of the program."""
#     # Instantiate the data problem.
#     data = create_data_model()

#     # Create the routing index manager.
#     manager = pywrapcp.RoutingIndexManager(len(data['locations']),
#                                            data['num_vehicles'], data['depot'])

#     # Create Routing Model.
#     routing = pywrapcp.RoutingModel(manager)

#     distance_matrix = compute_euclidean_distance_matrix(data['locations'])

#     def distance_callback(from_index, to_index):
#         """Returns the distance between the two nodes."""
#         # Convert from routing variable Index to distance matrix NodeIndex.
#         from_node = manager.IndexToNode(from_index)
#         to_node = manager.IndexToNode(to_index)
#         return distance_matrix[from_node][to_node]

#     transit_callback_index = routing.RegisterTransitCallback(distance_callback)

#     # Define cost of each arc.
#     routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

#     # Setting first solution heuristic.
#     search_parameters = pywrapcp.DefaultRoutingSearchParameters()
#     search_parameters.first_solution_strategy = (
#         routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

#     # Solve the problem.
#     solution = routing.SolveWithParameters(search_parameters)

#     # Print solution on console.
#     if solution:
#         print_solution(manager, routing, solution)
#         s = get_routes(solution, routing, manager)
#     return s


# if __name__ == '__main__':
#     print (main())

def solve_it(input_data, file_name):
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append((float(parts[0]), float(parts[1])))  # yapf: disable

    def create_data_model():
        data = {}
        data['locations'] = points
        data['num_vehicles'] = 1
        data['depot'] = 0
        return data
    
    data = create_data_model()

    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 180
    search_parameters.log_search = True
    # search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        #print_solution(manager, routing, solution, distance_callback)
        s = get_routes(solution, routing, manager)   
    
    output_data = '%.2f' % (solution.ObjectiveValue()/100) + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, s[:-1]))
    
    h = open('OR Tools solution/' + file_name + '_ortools', 'w')
    h.write(output_data)
    h.close()
    
    return output_data

files = os.listdir('data')
t = 59
for i in sorted(files, key = lambda x: len(x))[59:69]:
    print (t)
    solve_it(open('data/' + i, 'r').read(), i)
    t += 1

#start = time.time()
#print (solve_it(open('data/tsp_51_1', 'r').read()))
#end = time.time()
#print (end - start)
   

