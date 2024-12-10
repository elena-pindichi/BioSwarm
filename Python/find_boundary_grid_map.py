import cv2
import numpy as np

import matplotlib.pyplot as plt
import gdspy
from poly_decomp import poly_decomp as pd
from matplotlib.patches import Circle, Wedge, Polygon
from numpy import linalg as LA
#import win32clipboard as clipboard
from scipy.sparse.csgraph import shortest_path
from scipy.interpolate import BSpline

def compare_2_areas_get_1(contour_1, contour_2):
    area_1 = cv2.contourArea(contour_1)
    area_2 = cv2.contourArea(contour_2)
    max_ = max(area_1, area_2)
    return max_


def plot_poly_map(ws, ax, col):
    # plot the whole polytope map in subplot ax with color col
    for i in range(len(ws)):
        x_de = [ws[i][k][0] for k in range(len(ws[i]))]
        x_de.append(ws[i][0][0])
        y_de = [ws[i][k][1] for k in range(len(ws[i]))]
        y_de.append(ws[i][0][1])
        ax.plot(x_de, y_de, color=col)
    return 0

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

def get_path(Pr, i, j):
    path = [j]
    k = j
    while Pr[i, k] != -9999:
        path.append(Pr[i, k])
        k = Pr[i, k]
    return path[::-1]


print(cv2.__version__)
resolution = 0.1   # resolution in file .yaml
image = cv2.imread('/home/truontmh/python_code/binary_map_after_gimp.png')
# convert image to gray image 
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

height, width = image.shape
print(f'Height in real environment: {height * resolution} (m)')
print(f'width in real environment: {width * resolution} (m)')

# set a threshold, convert image to binary image (0 and 255)
# 0 is black, 255 is white
threshold = 127
_,binary_image = cv2.threshold(image, threshold,255,cv2.THRESH_BINARY)
image = binary_image

########### find contour of black pixels  ###########

inverted_image = cv2.bitwise_not(image) # function cv2.findContours will find the contours of white pixels
contours,_ = cv2.findContours(inverted_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
print("len of contours: ", len(contours))

########### compute areas of contours ###############
area_list = []
for contour in contours:
    area = cv2.contourArea(contour)
    area_list.append(area)

max_area = max(area_list)
max_area_index = area_list.index(max_area)

area_list[max_area_index] = 0 # convert max value to 0, -> find the second largest value
second_max_area = max(area_list)
second_max_area_index = area_list.index(second_max_area)

########### remove largest and second largest contour ############
filter_contours = [x for i,x in enumerate(contours) if i not in (max_area_index, second_max_area_index)]
filter_area = [x for i,x in enumerate(area_list) if i not in (max_area_index, second_max_area_index)]


########### compute moments of contours ##############
centroid_list = []
for contour in filter_contours:
    moment = cv2.moments(contour)
    # Compute centroids from moments
    if moment['m00'] != 0:
        cX = int(moment['m10']/moment['m00'])
        cY = int(moment['m01']/moment['m00'])
    else:
        cX, cY = 0,0  # if the contour doesn't have area
    centroid_list.append([cX,cY])


########### Find contour pairs with close center points. After that, chose one which have larger area.
thresh_distance = 5
get_contour_index_list = []
for i in range(len(centroid_list)):
    for j in range(len(centroid_list)):
        if i != j:
            point_1 = np.array(centroid_list[i])
            point_2 = np.array(centroid_list[j])
            distance = np.linalg.norm(point_2 - point_1)
            if distance < thresh_distance:
                max_ = compare_2_areas_get_1(filter_contours[i],filter_contours[j])
                if max_ == filter_area[i] and i not in get_contour_index_list:
                    get_contour_index_list.append(i)
                elif max_ == filter_area[j] and j not in get_contour_index_list:
                    get_contour_index_list.append(j)
                


############ find convex for contours
hull = []
hull.append(cv2.convexHull(contours[max_area_index]))
for index in get_contour_index_list:
    hull.append(cv2.convexHull(filter_contours[index]))

############ reduce the number of point of each hull -> reduce the computations
approx_hull_list = []
for convex_hull in hull:
    convex_hull = convex_hull.reshape(-1,1,2)
    desired_vertices = len(convex_hull)//1.67
    epsilon = 0.01 * cv2.arcLength(convex_hull, True)
    approx_hull = convex_hull
    while len(approx_hull) > desired_vertices:
        epsilon += 0.01 * cv2.arcLength(convex_hull, True)
        approx_hull = cv2.approxPolyDP(convex_hull, epsilon, True)
    while len(approx_hull) < desired_vertices:
        epsilon = epsilon/1.01
        approx_hull = cv2.approxPolyDP(convex_hull, epsilon, True)
    approx_hull_list.append(approx_hull)





############ Create polytope
obstacles_list = []
hole_list = []
hole_large_list = []
safety_offset = 5


for i in range(len(approx_hull_list)):
    if i == 0:
        ws_limit = np.array(approx_hull_list[0])
        ws_limit = [tuple(x[0]) for x in ws_limit]
    if i!=0:
        obstacle = approx_hull_list[i]
        obstacle = np.array(obstacle)
        obstacle = [tuple(x[0]) for x in obstacle]
        obstacles_list.append(obstacle)




ws_poly = gdspy.Polygon(ws_limit)
for i in range(len(obstacles_list)):
    hole_list.append(gdspy.Polygon(obstacles_list[i]))

# enlarge obstacle a little bit
for i in range(len(hole_list)):
    hole_large_list.append(gdspy.offset(hole_list[i],safety_offset))

# subtraction
poly_with_hole = gdspy.boolean(ws_poly, hole_large_list, "not")

# decomposition
ws = pd.polygonQuickDecomp(poly_with_hole.polygons[0])

# initial point and goal
init = [100, 50]
goal = [186,182]
n_polyp = len(ws)
init_goal_idx = (generate_sequence(ws, init, goal))
center_list = []
for j in range(n_polyp):
    center_list.append(center(ws[j]).A1)


# print(center_list[0])
N = np.empty((0, n_polyp), float)
#print(f'ahihihihi {N.shape}')
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
#print(N)
# Dijkstra’s algorithm
D, Pr = shortest_path(N, directed=False, method='D', return_predecessors=True)
sequence = get_path(Pr, init_goal_idx[0], init_goal_idx[1])

# create b_spline curve
sequence_point = []
sequence_point.append(init)
for i in sequence:
    sequence_point.append(center_list[i])
sequence_point.append(goal)

points = np.array(sequence_point)
degree = 3
t = np.linspace(0, 1, len(points) + degree + 1 - 2*degree)
knots = np.concatenate(([0]*degree, t, [1]*degree))
bspline = BSpline(knots, points, degree)
t_eval = np.linspace(0, 1, 1000)
spline_points = bspline(t_eval)






fig = plt.figure()
ax = fig.add_subplot(111)
plot_poly_map(ws, ax, 'black')
# plot_poly_map([obstacle1, obstacle2, obstacle3], ax, 'blue')
plt.scatter(init[0], init[1], color='red', s=80, marker='*')
plt.scatter(goal[0], goal[1], color='blue', s=80, marker='*')
plt.scatter([center_list[i][0] for i in range(len(center_list))], [center_list[i][1] for i in range(len(center_list))],
            color='k', s=30)
plot_poly_map([ws[i] for i in sequence], ax, 'red')
for hole_large in hole_large_list:
    p = Polygon(hole_large.polygons[0], facecolor = 'k', alpha = 0.5)
    plt.gca().add_patch(p)
# plt.plot(x_fine, y_fine, label='Cubic Spline Trajectory', color='blue')  # trajectory
plt.plot(spline_points[:, 0], spline_points[:, 1], label='B-spline trajectory')

plt.show()




# Create a white image
white_image = image.copy()
for i in range(height):
    for j in range(width):
        white_image[i][j] = 255
white_image = cv2.cvtColor(white_image, cv2.COLOR_GRAY2BGR) 

white_image_2 = white_image.copy()

# Show
cv2.drawContours(white_image, hull, -1, (255,0,0), thickness=2)
cv2.drawContours(white_image_2,approx_hull_list,-1,(0,0,255), thickness=2)
cv2.imshow('Image',white_image)
#cv2.imshow('original_image', image)
cv2.imshow('reduce_convex', white_image_2)
cv2.waitKey(0)
cv2.destroyAllWindows()