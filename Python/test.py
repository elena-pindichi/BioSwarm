# import numpy as np
# from scipy.interpolate import CubicSpline
# import matplotlib.pyplot as plt

# # Define waypoints
# waypoints = np.array([
#     [0, 0],  # Point 1 (x, y)
#     [1, 2],  # Point 2 (x, y)
#     [4, 3],  # Point 3 (x, y)
#     [6, 1]   # Point 4 (x, y)
# ])

# # Separate x and y coordinates
# x = waypoints[:, 0]
# y = waypoints[:, 1]

# # Create an array for parameter t (e.g., distance or time)
# t = np.linspace(0, 1, len(x))

# # Fit cubic spline
# spline_x = CubicSpline(t, x)
# spline_y = CubicSpline(t, y)

# # Generate a finer trajectory
# t_fine = np.linspace(0, 1, 100)
# x_fine = spline_x(t_fine)
# y_fine = spline_y(t_fine)

# # Plot the results
# plt.figure(figsize=(8, 6))
# plt.plot(x, y, 'o', label='Waypoints')  # Waypoints
# plt.plot(x_fine, y_fine, '-', label='Trajectory')  # Smooth trajectory
# plt.legend()
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Trajectory through Waypoints')
# plt.grid()
# plt.show()



#####################################################################################
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.interpolate import BSpline

# points = np.array([[0, 0], [1, 2], [2, 1], [3, 3], [4, 2], [5, 0]])

# degree = 3
# t = np.linspace(0, 1, len(points) + degree + 1 - 2 * degree)
# knots = np.concatenate(([0] * degree, t, [1] * degree))
# bspline = BSpline(knots, points, degree)

# t_eval = np.linspace(0, 1, 1000)
# spline_points = bspline(t_eval)

# plt.plot(spline_points[:, 0], spline_points[:, 1])
# plt.scatter(points[:, 0],points[:, 1], c = 'red')
# plt.show()




######################################################################################
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

x_points = [0, 1, 2, 3, 4, 5]
y_points = [0, 2, 1, 3, 0, -1]

cs = CubicSpline(x_points, y_points)

x_fine = np.linspace(min(x_points), max(x_points), 200)
y_fine = cs(x_fine)

plt.figure(figsize=(8, 5))
plt.plot(x_points, y_points, 'o', label = 'Given Points', color = 'red')
plt.plot(x_fine, y_fine, label = 'Cubic Spline Trajectory', color = 'blue')
plt.show()
