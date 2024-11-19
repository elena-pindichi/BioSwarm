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
from nonlinear_mpc import MPC_controller
from bspline_path_planner import bspline_path_planner_polytope_map
from scipy.interpolate import CubicSpline
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


ws_limit = [(np.float64(-3), np.float64(3)), (np.float64(-3), np.float64(-3)), (np.float64(3), np.float64(-3)), (np.float64(3), np.float64(3))]
# obstacle1 = [(0.5, 0.5), (1.2, 0.4), (1., 0.7), (1., 1.3)]
# obstacle2 = [(1.8, 1.), (1.6, 0.9), (1.5, 1.5), (1.8, 1.7)]
# obstacle3 = [(0.2, 1.3), (0.5, 1.4), (0.4, 1.7),(0.3, 1.6),(1.45, 1.3)]
# obstacle4 = [(0.125,1.825),(0.2, 1.7),(0.4, 1.7),(0.45, 1.8),()]
obstacle11 = [(-0.6, -0.6), (-0.6, -1.6), (-1.6, -1.6), (-1.6, -0.6)]
# obstacle12 = [(-0.6, 0.5), (-0.6, -0.5), (-1.6, -0.5), (-1.6, 0.5)]
# obstacle13 = [(-0.6, 1.6), (-0.6, 0.6), (-1.6, 0.6), (-1.6, 1.6)]
# obstacle21 = [(0.5, -0.6), (0.5, -1.6), (-0.5, -1.6), (-0.5, -0.6)]
# obstacle22 = [(0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5)]
# obstacle23 = [(0.5, 1.6), (0.5, 0.6), (-0.5, 0.6), (-0.5, 1.6)]
# obstacle31 = [(1.6, -0.6), (1.6, -1.6), (0.6, -1.6), (0.6, -0.6)]
# obstacle32 = [(1.6, 0.5), (1.6, -0.5), (0.6, -0.5), (0.6, 0.5)]
# obstacle33 = [(1.6, 1.6), (1.6, 0.6), (0.6, 0.6), (0.6, 1.6)]

# print(type(ws_limit))
# print(type(ws_limit[2]))
# print(type(ws_limit[2][0]))

ws_poly = gdspy.Polygon(ws_limit)
hole11 = gdspy.Polygon(obstacle11)
# hole12 = gdspy.Polygon(obstacle12)
# hole13 = gdspy.Polygon(obstacle13)
# hole21 = gdspy.Polygon(obstacle21)
# hole22 = gdspy.Polygon(obstacle22)
# hole23 = gdspy.Polygon(obstacle23)
# hole31 = gdspy.Polygon(obstacle31)
# hole32 = gdspy.Polygon(obstacle32)
# hole33 = gdspy.Polygon(obstacle33)
# enlarge obstacle a little bit
safety_offset = 0.0
hole11_large = gdspy.offset(hole11, safety_offset)
# hole12_large = gdspy.offset(hole12, safety_offset)
# hole13_large = gdspy.offset(hole13, safety_offset)
# hole21_large = gdspy.offset(hole21, safety_offset)
# hole22_large = gdspy.offset(hole22, safety_offset)
# hole23_large = gdspy.offset(hole23, safety_offset)
# hole31_large = gdspy.offset(hole31, safety_offset)
# hole32_large = gdspy.offset(hole32, safety_offset)
# hole33_large = gdspy.offset(hole33, safety_offset)

print(hole11_large.polygons[0])
print(gdspy.Polygon(hole11_large.polygons[0]))
print(type(gdspy.Polygon(hole11_large.polygons[0])))
# print(hole12_large.polygons[0])
# print(gdspy.Polygon(hole12_large.polygons[0]))
# print(type(gdspy.Polygon(hole12_large.polygons[0])))
# print(hole13_large.polygons[0])
# print(gdspy.Polygon(hole13_large.polygons[0]))
# print(type(gdspy.Polygon(hole13_large.polygons[0])))
# print(hole21_large.polygons[0])
# print(gdspy.Polygon(hole21_large.polygons[0]))
# print(type(gdspy.Polygon(hole21_large.polygons[0])))
# print(hole22_large.polygons[0])
# print(gdspy.Polygon(hole22_large.polygons[0]))
# print(type(gdspy.Polygon(hole22_large.polygons[0])))
# print(hole23_large.polygons[0])
# print(gdspy.Polygon(hole23_large.polygons[0]))
# print(type(gdspy.Polygon(hole23_large.polygons[0])))
# print(hole31_large.polygons[0])
# print(gdspy.Polygon(hole31_large.polygons[0]))
# print(type(gdspy.Polygon(hole31_large.polygons[0])))
# print(hole32_large.polygons[0])
# print(gdspy.Polygon(hole32_large.polygons[0]))
# print(type(gdspy.Polygon(hole32_large.polygons[0])))
# print(hole33_large.polygons[0])
# print(gdspy.Polygon(hole33_large.polygons[0]))
# print(type(gdspy.Polygon(hole33_large.polygons[0])))
# subtraction
# poly_with_hole = gdspy.boolean(ws_poly, [hole11_large, hole12_large, hole13_large, hole21_large, hole22_large, hole23_large, hole31_large, hole32_large, hole33_large], "not")
poly_with_hole = gdspy.boolean(ws_poly, [hole11_large], "not")

# print(poly_with_hole.polygons[0])

# decomposition
ws = pd.polygonQuickDecomp(poly_with_hole.polygons[0])

# initial pose and final goal
init = [-1, -2]
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

def get_edge_midpoints(sequence, workspace):
    """
    Computes the midpoints of the edges of the polygons in the given sequence.

    Parameters:
        sequence (list): List of polygon indices in the sequence.
        workspace (list): List of all polygons in the workspace.

    Returns:
        dict: A dictionary where each key is a polygon index from the sequence,
              and the value is a list of midpoints for the edges of that polygon.
    """
    edge_midpoints = {}

    for poly_idx in sequence:
        # Get the vertices of the polygon
        polygon = workspace[poly_idx]
        num_vertices = len(polygon)
        
        # Compute midpoints of each edge
        midpoints = []
        for i in range(num_vertices):
            # Get current and next vertex (wrapping around at the last vertex)
            current_vertex = polygon[i]
            next_vertex = polygon[(i + 1) % num_vertices]
            
            # Compute midpoint
            midpoint = [(current_vertex[0] + next_vertex[0]) / 2,
                        (current_vertex[1] + next_vertex[1]) / 2]
            midpoints.append(midpoint)
        
        # Store the midpoints for the current polygon
        edge_midpoints[poly_idx] = midpoints

    return edge_midpoints

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

# MPC controller
X0 = np.array([init[0], init[1], 1.5])
XREF = np.array([goal[0], goal[1], 0.25])
xsim = MPC_controller(X0, XREF)

# Use the sequence of polytopes found by Dijkstra’s algorithm
# Convert the sequence into a workspace for the path planner
ws_sequence = [ws[i] for i in sequence]

# Get the edge points from the marked polygons
edge_midpoints = get_edge_midpoints(sequence, ws)
x_points = []
y_points = []

for key, points in edge_midpoints.items():
    for point in points:
        x_points.append(point[0])  # Add the first element (x)
        y_points.append(point[1])  # Add the second element (y)

# sorted_indices = np.argsort(x_points)
# x_points_sorted = np.array(x_points)[sorted_indices]

# x_points = x_points_sorted
x_points = [-3, -2, -1, 0, 0, 1, 1, 1, 3]
y_points = [2, -2, 1, 0, 2, 1, 1, -1, -1]

cs = CubicSpline(x_points, y_points)
x_fine = np.linspace(min(x_points), max(x_points), 200)
y_fine = cs(x_fine)

plt.figure(figsize=(8, 5))
plt.plot(x_points, y_points, 'o', label = 'Given Points', color = 'red')
plt.plot(x_fine, y_fine, label = 'Cubic Spline Trajectory', color = 'blue')
plt.show()

# Print the values of the edges
for poly_idx, midpoints in edge_midpoints.items():
    print(f"Polygon {poly_idx}: Edge Midpoints = {midpoints}")

print("Optimal Trajectory:", xsim)
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
# p = Polygon(hole12_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole13_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole21_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole22_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole23_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole31_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole32_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# p = Polygon(hole33_large.polygons[0], facecolor='k', alpha=0.5)
# plt.gca().add_patch(p)
# for i in range(5):
#     plt.scatter(ws[5][i][0], ws[5][i][1],color='k', s=50)
plt.plot(xsim[0, :], xsim[1, :], color='green', label='MPC Trajectory', linewidth=2)

plt.show()