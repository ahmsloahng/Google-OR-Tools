#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import math
from collections import namedtuple
import time


from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])

def length(customer1, customer2):
    return math.sqrt((customer1[0] - customer2[0])**2 + (customer1[1] - customer2[1])**2)

def vehicle_distance(customers, v):
    t = 0
    for i in range(len(v)-1):
        t += length(customers[v[i]], customers[v[i+1]])
    t  += length(customers[v[-1]], customers[v[0]])
    return t

def opt(customers, vec):
    for k in range(10):
        for i in range(1,len(vec) - 1):
            for j in range(i+2, len(vec)+1):
                p = vec[:]
                vec[i:j] = p[j-1:i-1:-1]
                if vehicle_distance(customers, p) < vehicle_distance(customers, vec):  vec = p 
    return vec

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

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            print (plan_output)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance/100)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance/100))
    print('Total load of all routes: {}'.format(total_load))

def output_data(data, manager, routing, solution):
    total_distance = 0
    sequence_for_all_vehicles = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_distance = 0
        seq = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            seq.append(node_index)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        seq.append(manager.IndexToNode(index))
        total_distance += route_distance
        sequence_for_all_vehicles.append(seq)
        sequence_for_all_vehicles = [i for i in sequence_for_all_vehicles if len(i) > 2]
    return [total_distance/100] + sequence_for_all_vehicles
            
# def solve_it(input_data):
#     # Modify this code to run your optimization algorithm

#     # parse the input
#     lines = input_data.split('\n')

#     parts = lines[0].split()
#     customer_count = int(parts[0])
#     vehicle_count = int(parts[1])
#     vehicle_capacity = int(parts[2])
    
#     customers = []
#     for i in range(1, customer_count+1):
#         line = lines[i]
#         parts = line.split()
#         customers.append(Customer(i-1, int(parts[0]), float(parts[1]), float(parts[2])))

#     # build a trivial solution
    
#     customer_order = opt(customers, [i.index for i in customers])
#     customer_order.remove(0)
    
#     final_vehicle = []
#     for v in range(0, vehicle_count):
#         final_vehicle.append([])
#         capacity_remaining = vehicle_capacity
#         while sum([capacity_remaining >= customers[i].demand for i in customer_order]) > 0:
#             order = customer_order[:]
#             for c in order:
#                 if capacity_remaining >= customers[c].demand:
#                     capacity_remaining -= customers[c].demand
#                     final_vehicle[v].append(c)
#                     customer_order.remove(c)

#     # checks that the number of customers served is correct
#     assert sum([len(v) for v in final_vehicle]) == len(customers) - 1    
    
#     for vehicle in final_vehicle:
#         vehicle = opt(customers, [0] + vehicle)
#         del vehicle[0]
        
# # calculate the cost of the solution; for each vehicle the length of the route
#     obj = 0
#     for v in range(0, vehicle_count):
#         if len(final_vehicle[v]) > 0:
#             obj += vehicle_distance(customers, [0] + final_vehicle[v])

#     # prepare the solution in the specified output format
#     outputData = '%.2f' % obj + ' ' + str(0) + '\n'
#     for v in range(0, vehicle_count):
#         outputData += str(0) + ' ' + ' '.join([str(customer) for customer in final_vehicle[v]]) + ' ' + str(0) + '\n'

#     return outputData

def solve_it(input_data, binary):
#lines = open('data/vrp_5_4_1', 'r').read().split('\n')   
    lines = input_data.split('\n')
    
    parts = lines[0].split()
    customer_count = int(parts[0])
    vehicle_count = int(parts[1])
    vehicle_capacity = int(parts[2])
    
    customers_location = []
    customers_demand = []
    for i in range(1, customer_count+1):
        line = lines[i]
        parts = line.split()
        customers_location.append((float(parts[1]), float(parts[2])))  
        customers_demand.append(int(parts[0]))
    
    def create_data_model():
        data = {}
        data['locations'] = customers_location
        data['demands'] = customers_demand
        data['vehicle_capacities'] = [vehicle_capacity]*vehicle_count
        data['num_vehicles'] = vehicle_count
        data['depot'] = 0
        return data
    
    data = create_data_model()
    
    manager = pywrapcp.RoutingIndexManager(len(data['locations']), data['num_vehicles'], data['depot'])
    
    routing = pywrapcp.RoutingModel(manager)
    
    distance_matrix = compute_euclidean_distance_matrix(data['locations'])
    
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, data['vehicle_capacities'], True, 'Capacity')
    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    if binary == 0:
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    else:
        search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = 60
        search_parameters.log_search = True
    
    solution = routing.SolveWithParameters(search_parameters)
    
    if solution:
        print_solution(data, manager, routing, solution)
    
        # result = output_data(data, manager, routing, solution)
        
        # output = str(result[0]) + ' ' + '0' + '\n'
        # for i in range(1,len(result)):
        #     output += ' '.join([str(j) for j in result[i]]) + '\n'
            
        # h = open('solution/' + file_name + '_ortools', 'w')
        # #h = open('Solution/vrp_200_16_1_ortools', 'w')
        # h.write(output)
        # h.close()
    
    return data, sum(customers_demand)
    
#start = time.time()
files = os.listdir('data')
# for i in files:
#     print (t)
#     solve_it(open('data/' + i, 'r').read(), 1, i)
print (solve_it(open('data/vrp_31_9_1', 'r').read(), 1))
#end = time.time()
#print (end - start)
