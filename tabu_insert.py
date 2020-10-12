# -*- coding: utf-8 -*-
"""
Created on Thu Aug  6 13:05:09 2020

@author: Amlan Ghosh
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 20:17:25 2020

@author: Amlan Ghosh
"""

import math
import random as rand
import numpy as np
from collections import namedtuple
import itertools as itr
import time

Point = namedtuple("Point", ['x', 'y'])

'''Distance between two points'''
def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

'''Calculating the total distance of a tsp route'''
def obj(solution, points, nodeCount):
    ob = length(points[solution[-1]], points[solution[0]]) #Distance between end point and first point
    for index in range(0, nodeCount-1):
        ob += length(points[solution[index]], points[solution[index+1]]) #summation of the distances between points
    return ob

'''Nearest Neighbourhood'''
def nn(node, start, points):
    ini = list(range(0, node))
    solution = [start]
    for i in range(node - 1):
        solution.append(sorted([i for i in ini if i not in solution], key = lambda x: length(points[solution[-1]], points[x]))[0])
    return solution

'''2-Opt'''
def opt2(solution, points, nodeCount):
    for m in range(10):
        for i in range(1,len(solution) - 1):
            for j in range(i+2, len(solution)+1):
                p = solution[:]
                solution[i:j] = p[j-1:i-1:-1]
                if obj(p, points, nodeCount) < obj(solution, points, nodeCount) : solution = p
    return solution


def new_sol(solution, r):
    solution[r[0]], solution[r[1]] = solution[r[1]], solution[r[0]]
    return solution


def difference(solution, r, n, points):
    
    solution = solution + [solution[0]]

    if r[0] < r[1]:
        if r[0] == 0 and r[1] == n-1:
            diff = 0
        else:
            diff = length(points[solution[r[0]-1]], points[solution[r[0]]]) + length(points[solution[r[0]]], points[solution[r[0]+1]]) + length(points[solution[r[1]]], points[solution[r[1]+1]]) - length(points[solution[r[0]-1]], points[solution[r[0]+1]]) - length(points[solution[r[1]]], points[solution[r[0]]]) - length(points[solution[r[0]]], points[solution[r[1]+1]])
    else:
        if r[0] == n-1 and r[1] == 0:
            diff = 0
        else:
            diff = length(points[solution[r[0]-1]], points[solution[r[0]]]) + length(points[solution[r[0]]], points[solution[r[0]+1]]) + length(points[solution[r[1]-1]], points[solution[r[1]]]) - length(points[solution[r[0]-1]], points[solution[r[0]+1]]) - length(points[solution[r[1]-1]], points[solution[r[0]]]) - length(points[solution[r[0]]], points[solution[r[1]]])
        
    return diff 

def tabu(iterations, solution, tabu_tenure, points, nodeCount):
    best = solution[:]
    tabu_list = np.array([0 for i in range(nodeCount)])
    
    for iteration in range(iterations):
        # print ('------------')
        # print (iteration)
        # print (obj(solution, points, nodeCount))
        
        if (iteration+1)%50 == 0:
            u = rand.sample(range(0, nodeCount), 2)
            temp = solution[:]
            if u[0] < u[1]:
                temp = temp[:u[1]+1] + [temp[u[0]]] + temp[u[1]+1:]
                del temp[u[0]]         
            else:
                temp = temp[:u[1]] + [temp[u[0]]] + temp[u[1]:]
                del temp[u[0]+1]         
        
#        r = rand.sample(range(0, nodeCount), 2)
        m = [rand.sample(range(0, nodeCount), 2) for i in range(100)]
        m.sort(key = lambda x : difference(solution, x, nodeCount, points))
        r = m[-1]
        
        if tabu_list[r[0]] == 0:
#            if difference(solution, r, nodeCount, points) > 0:

            temp = solution[:]
            if r[0] < r[1]:
                temp =temp[:r[1]+1] + [temp[r[0]]] + temp[r[1]+1:]
                del temp[r[0]]
            else:
                temp =temp[:r[1]] + [temp[r[0]]] + temp[r[1]:]
                del temp[r[0]+1] 
            solution = temp
            tabu_list[r[0]] = tabu_tenure
            if obj(solution, points, nodeCount) < obj(best, points, nodeCount): best = solution[:]
            for q in range(nodeCount):
                if tabu_list[q] > 0: tabu_list[q] -= 1                           
        else:
            
            temp = solution[:]
            if r[0] < r[1]:
                temp =temp[:r[1]+1] + [temp[r[0]]] + temp[r[1]+1:]
                del temp[r[0]]
            else:
                temp =temp[:r[1]] + [temp[r[0]]] + temp[r[1]:]
                del temp[r[0]+1]  
            
            if obj(temp, points, nodeCount) < obj(best, points, nodeCount):
                
                tabu_list[r[0]]= tabu_tenure
                solution = temp
                best = solution[:]
                for q in range(nodeCount):
                    if tabu_list[q] > 0: tabu_list[q] -= 1   
    return best

def solve_it(input_data):
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1])))

    sol = nn(nodeCount, 2, points)
    
    sol = opt2(sol, points, nodeCount)
    
    solution = tabu(20000, sol, 6, points, nodeCount)
    
    solution = opt2(solution, points, nodeCount)
    
    
    output_data = '%.2f' % obj(solution, points, nodeCount) + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))
    
    # h = open('solution/tsp_51', 'w')
    # h.write(output_data)
    
    return output_data

start = time.time()
print (solve_it(open('data/tsp_51_1', 'r').read()))
end = time.time()
print (end - start)
                
            
        
