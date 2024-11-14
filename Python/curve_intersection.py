# -*- coding: utf-8 -*-
"""
Created on Wed Nov  3 15:36:25 2021

@author: Dr. Ngoc Thinh Nguyen

Modified by Huu-Thinh DO to find the sequence of polytopic corridor

Create the scenario used for exercise 2
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import gdspy
from poly_decomp import poly_decomp as pd
from matplotlib.patches import Circle, Wedge, Polygon
from numpy import linalg as LA
#import win32clipboard as clipboard
from scipy.sparse.csgraph import shortest_path
# from numpy import array



# def toClipboardForExcel(array):
#     """
#     Copies an array into a string format acceptable by Excel.
#     Columns separated by \t, rows separated by \n
#     """
#     # Create string from array
#     line_strings = []
#     for line in array:
#         line_strings.append("\t".join(line.astype(str)).replace("\n", ""))
#     array_string = "\r\n".join(line_strings)

#     # Put string into clipboard (open, clear, set, close)
#     clipboard.OpenClipboard()
#     clipboard.EmptyClipboard()
#     clipboard.SetClipboardText(array_string)
#     clipboard.CloseClipboard()


def plot_poly_map(ws, ax, col):
    # plot the whole polytope map in subplot ax with color col
    for i in range(len(ws)):
        x_de = [ws[i][k][0] for k in range(len(ws[i]))]
        x_de.append(ws[i][0][0])
        y_de = [ws[i][k][1] for k in range(len(ws[i]))]
        y_de.append(ws[i][0][1])
        ax.plot(x_de, y_de, color=col)
    return 0


ws_limit = [(np.float64(0), np.float64(0)), (np.float64(2), np.float64(0)), (np.float64(2), np.float64(2)), (np.float64(0), np.float64(2))]
obstacle1 = [(0.5, 0.5), (1.2, 0.4), (1., 0.7), (1., 1.3)]
obstacle2 = [(1.8, 1.), (1.6, 0.9), (1.5, 1.5), (1.8, 1.7)]
obstacle3 = [(0.2, 1.3), (0.5, 1.4), (0.4, 1.7),(0.3, 1.6),(1.45, 1.3)]
obstacle4 = [(0.125,1.825),(0.2, 1.7),(0.4, 1.7),(0.45, 1.8),()]

# print(type(ws_limit))
# print(type(ws_limit[2]))
# print(type(ws_limit[2][0]))

ws_poly = gdspy.Polygon(ws_limit)
hole1 = gdspy.Polygon(obstacle1)
hole2 = gdspy.Polygon(obstacle2)
hole3 = gdspy.Polygon(obstacle3)
# enlarge obstacle a little bit
safety_offset = 0.0
hole1_large = gdspy.offset(hole1, safety_offset)
hole2_large = gdspy.offset(hole2, safety_offset)
hole3_large = gdspy.offset(hole3, safety_offset)

print(hole1_large.polygons[0])
print(gdspy.Polygon(hole1_large.polygons[0]))
print(type(gdspy.Polygon(hole1_large.polygons[0])))
# subtraction
poly_with_hole = gdspy.boolean(ws_poly, [hole1_large, hole2_large, hole3_large], "not")

# print(poly_with_hole.polygons[0])

# decomposition
ws = pd.polygonQuickDecomp(poly_with_hole.polygons[0])

# initial pose and final goal
init = [1, 1.75]
# goal = [0.25, 1.]
goal = [0.5, 0.25]
"""    
# TODO: now find the sequence of polytopes

def generate_sequence(workspace, init, goal):
    sequence = []
    return sequence

sequence = generate_sequence(ws, init, goal) 
"""


# Check if two decomposed polytopes lies next to each other
# if the there are exactly 2 vertices of one inside the other ===> true
def check_consecutive_polytopes(p1, p2):
    check = False
    s = 0
    check_list = gdspy.inside(p1, gdspy.Polygon(p2))
    for i in range(len(check_list)):
        if check_list[i]:
            s = s + 1
    if s == 2:
        check = True
    return check


def center(polyp):
    xc = np.matrix(polyp)
    return xc.mean(0)


def find_polyp(workspace, points):  ######### check point in polygon
    polyp_index = -1
    for i in range(len(workspace)):
        if (gdspy.inside([points], gdspy.Polygon(ws[i])))[0]:
            polyp_index = i
    return polyp_index


def generate_sequence(workspace, init_pts, goal_pts):
    init_polyp = find_polyp(workspace, init_pts)
    goal_polyp = find_polyp(workspace, goal_pts)
    seq = [init_polyp, goal_polyp]
    return seq


n_polyp = len(ws)
init_goal_idx = (generate_sequence(ws, init, goal))
# print((ws[2]))
center_list = []
for j in range(n_polyp):
    center_list.append(center(ws[j]).A1)
# print(center_list[0])
N = np.empty((0, n_polyp), float)
print(f'ahihihihi {N.shape}')
N_temp = np.array([])
for i in range(n_polyp):
    N_temp = []
    for j in range(n_polyp):
        if check_consecutive_polytopes(ws[i], ws[j]) and (i != j):
            temp = [center_list[i], center_list[j]]
            N_temp.append(LA.norm(temp))
        else:
            N_temp.append(0)
    N = np.append(N, [N_temp], axis=0)
print(N)
# Dijkstra’s algorithm
D, Pr = shortest_path(N, directed=False, method='D', return_predecessors=True)
print(Pr)
print(ws[0])
print(gdspy.Polygon(ws[0]))
print(type(gdspy.Polygon(ws[0])))
def get_path(Pr, i, j):
    path = [j]
    k = j
    while Pr[i, k] != -9999:
        path.append(Pr[i, k])
        k = Pr[i, k]
    return path[::-1]


sequence = get_path(Pr, init_goal_idx[0], init_goal_idx[1])
print(init_goal_idx[1])
print(sequence)


# print("-----------------------------------------")
# print("from polytope " + str(init_goal_idx[0]) + " to polytope " + str(init_goal_idx[1]) + ", sequence:")
# print(sequence)
# print("-----------------------------------------")

# Send the map to clipboard
# toClipboardForExcel(N)

fig = plt.figure()
ax = fig.add_subplot(111)
plot_poly_map(ws, ax, 'black')
# plot_poly_map([obstacle1, obstacle2, obstacle3], ax, 'blue')
plt.scatter(init[0], init[1], color='red', s=80, marker='*')
plt.scatter(goal[0], goal[1], color='blue', s=80, marker='*')
plt.scatter([center_list[i][0] for i in range(len(center_list))], [center_list[i][1] for i in range(len(center_list))],
            color='k', s=30)
plot_poly_map([ws[i] for i in sequence], ax, 'red')
p = Polygon(hole1_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole2_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole3_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
# for i in range(5):
#     plt.scatter(ws[5][i][0], ws[5][i][1],color='k', s=50)
plt.show()