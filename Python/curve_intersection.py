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
# from nonlinear_mpc import MPC_controller
from bspline_path_planner import bspline_path_planner_polytope_map
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString
from shapely.geometry import Polygon as pg
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


ws_limit = [(np.float64(20), np.float64(0)), (np.float64(20), np.float64(-120)), (np.float64(100), np.float64(-120)), 
            (np.float64(100), np.float64(0))]
obstacle11 = [(83, -76), (91, -76), (91, -84), (83, -84)]
obstacle12 = [(61, -76), (70, -76), (70, -84), (61, -84)]
obstacle13 = [(39, -76), (48, -76), (48, -84), (39, -84)]
obstacle21 = [(83, -54), (91, -54), (91, -62), (83, -62)]
obstacle22 = [(61, -54), (69, -54), (69, -62), (61, -62)]
obstacle23 = [(39, -54), (47, -54), (47, -62), (39, -62)]
obstacle31 = [(40, -32), (48, -32), (48, -40), (40, -40)]
obstacle32 = [(83, -32), (91, -32), (91, -40), (83, -40)]
obstacle33 = [(61, -32), (69, -32), (69, -40), (61, -40)]

# print(type(ws_limit))
# print(type(ws_limit[2]))
# print(type(ws_limit[2][0]))

ws_poly = gdspy.Polygon(ws_limit)
hole11 = gdspy.Polygon(obstacle11)
hole12 = gdspy.Polygon(obstacle12)
hole13 = gdspy.Polygon(obstacle13)
hole21 = gdspy.Polygon(obstacle21)
hole22 = gdspy.Polygon(obstacle22)
hole23 = gdspy.Polygon(obstacle23)
hole31 = gdspy.Polygon(obstacle31)
hole32 = gdspy.Polygon(obstacle32)
hole33 = gdspy.Polygon(obstacle33)
# enlarge obstacle a little bit
safety_offset = 0.0
hole11_large = gdspy.offset(hole11, safety_offset)
hole12_large = gdspy.offset(hole12, safety_offset)
hole13_large = gdspy.offset(hole13, safety_offset)
hole21_large = gdspy.offset(hole21, safety_offset)
hole22_large = gdspy.offset(hole22, safety_offset)
hole23_large = gdspy.offset(hole23, safety_offset)
hole31_large = gdspy.offset(hole31, safety_offset)
hole32_large = gdspy.offset(hole32, safety_offset)
hole33_large = gdspy.offset(hole33, safety_offset)

print(hole11_large.polygons[0])
print(gdspy.Polygon(hole11_large.polygons[0]))
print(type(gdspy.Polygon(hole11_large.polygons[0])))
print(hole12_large.polygons[0])
print(gdspy.Polygon(hole12_large.polygons[0]))
print(type(gdspy.Polygon(hole12_large.polygons[0])))
print(hole13_large.polygons[0])
print(gdspy.Polygon(hole13_large.polygons[0]))
print(type(gdspy.Polygon(hole13_large.polygons[0])))
print(hole21_large.polygons[0])
print(gdspy.Polygon(hole21_large.polygons[0]))
print(type(gdspy.Polygon(hole21_large.polygons[0])))
print(hole22_large.polygons[0])
print(gdspy.Polygon(hole22_large.polygons[0]))
print(type(gdspy.Polygon(hole22_large.polygons[0])))
print(hole23_large.polygons[0])
print(gdspy.Polygon(hole23_large.polygons[0]))
print(type(gdspy.Polygon(hole23_large.polygons[0])))
print(hole31_large.polygons[0])
print(gdspy.Polygon(hole31_large.polygons[0]))
print(type(gdspy.Polygon(hole31_large.polygons[0])))
print(hole32_large.polygons[0])
print(gdspy.Polygon(hole32_large.polygons[0]))
print(type(gdspy.Polygon(hole32_large.polygons[0])))
print(hole33_large.polygons[0])
print(gdspy.Polygon(hole33_large.polygons[0]))
print(type(gdspy.Polygon(hole33_large.polygons[0])))
# subtraction
poly_with_hole = gdspy.boolean(ws_poly, [hole11_large, hole12_large, hole13_large, hole21_large, 
                                         hole22_large, hole23_large, hole31_large, hole32_large, hole33_large], "not")
# poly_with_hole = gdspy.boolean(ws_poly, [hole11_large], "not")

# print(poly_with_hole.polygons[0])

# decomposition
ws = pd.polygonQuickDecomp(poly_with_hole.polygons[0])

# initial pose and final goal
init = [50, -30]
# goal = [0.25, 1.]
goal = [85, -68]

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
print(sequence)

# print("-----------------------------------------")
# print("from polytope " + str(init_goal_idx[0]) + " to polytope " + str(init_goal_idx[1]) + ", sequence:")
# print(sequence)
# print("-----------------------------------------")

# Send the map to clipboard
# toClipboardForExcel(N)

# MPC controller
# X0 = np.array([init[0], init[1], 1.5])
# XREF = np.array([goal[0], goal[1], 0.25])
# xsim = MPC_controller(X0, XREF)

# Use the sequence of polytopes found by Dijkstra’s algorithm
# Convert the sequence into a workspace for the path planner
ws_sequence = [ws[i] for i in sequence]

def find_common_edges(p1, p2):
    """
    Finds the common edges between two polygons.
    """
    edges_p1 = [(p1[i], p1[(i + 1) % len(p1)]) for i in range(len(p1))]
    edges_p2 = [(p2[i], p2[(i + 1) % len(p2)]) for i in range(len(p2))]

    common_edges = []
    for e1 in edges_p1:
        for e2 in edges_p2:
            if np.allclose(e1, e2, atol=1e-3) or np.allclose(e1[::-1], e2, atol=1e-3):
                common_edges.append(e1)
    return common_edges

def edge_midpoint(edge):
    """
    Calculate the midpoint of an edge.
    """
    return [(edge[0][0] + edge[1][0]) / 2, (edge[0][1] + edge[1][1]) / 2]

waypoints = [init]

for idx in range(len(sequence) - 1):
    poly1 = ws[sequence[idx]]
    poly2 = ws[sequence[idx + 1]]
    
    common_edges = find_common_edges(poly1, poly2)

    for edge in common_edges:
        waypoints.append(edge_midpoint(edge))

waypoints.append(goal)

print(waypoints)

# Extract x and y coordinates of waypoints
x_waypoints = [point[0] for point in waypoints]
y_waypoints = [point[1] for point in waypoints]

# Interpolate the waypoints using cubic spline
cs_x = CubicSpline(range(len(x_waypoints)), x_waypoints)  # Spline for x
cs_y = CubicSpline(range(len(y_waypoints)), y_waypoints)  # Spline for y

# Generate fine points along the spline
num_fine_points = 200
t_fine = np.linspace(0, len(x_waypoints) - 1, num_fine_points)
x_fine = cs_x(t_fine)
y_fine = cs_y(t_fine)


# print("Optimal Trajectory:", xsim)
fig = plt.figure()
ax = fig.add_subplot(111)
plot_poly_map(ws, ax, 'black')
# plot_poly_map([obstacle1, obstacle2, obstacle3], ax, 'blue')
plt.scatter(init[0], init[1], color='red', s=80, marker='*')
plt.scatter(goal[0], goal[1], color='blue', s=80, marker='*')
plt.scatter([center_list[i][0] for i in range(len(center_list))], [center_list[i][1] for i in range(len(center_list))],
            color='k', s=30)
plot_poly_map([ws[i] for i in sequence], ax, 'red')
p = Polygon(hole11_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole12_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole13_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole21_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole22_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole23_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole31_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole32_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
p = Polygon(hole33_large.polygons[0], facecolor='k', alpha=0.5)
plt.gca().add_patch(p)
# for i in range(5):
#     plt.scatter(ws[5][i][0], ws[5][i][1],color='k', s=50)
# plt.plot(xsim[0, :], xsim[1, :], color='green', label='MPC Trajectory', linewidth=2)
plt.plot(x_fine, y_fine, color='blue', label='Cubic Spline Trajectory', linewidth=2)

plt.show()